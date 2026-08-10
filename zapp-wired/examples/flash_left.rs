//! Writes the **left** half's nRF from a `.zsab`, and nothing else.
//!
//! A bring-up tool, not a product: `zapp nrf flash` is the supported way to
//! update a keyboard and it always writes both halves together, because halves
//! on different builds cannot talk to each other. This deliberately does the
//! thing the product refuses to do, for one reason.
//!
//! The right half's I2C mailbox is currently broken — an upload never advances
//! past `off: 0` and nobody knows why yet. The left half is reached over the
//! UART between it and the STM32 and shares none of that: no I2C, no mailbox,
//! no TRRS bus. So this is how the layers underneath get proved out — the bulk
//! pipe, the NLIP codec, `SmpClient`'s upload loop — with every I2C question
//! taken out of the picture.
//!
//! It opens the session on the left half alone, which makes ignition skip the
//! right-entry sequence entirely (see `session_tick`), so it runs with the
//! right half unplugged, unpowered, or not built yet.
//!
//! ```text
//! cargo run -p zapp-wired --example flash_left -- fw.zsab
//! cargo run -p zapp-wired --example flash_left -- --force --keep fw.zsab
//! RUST_LOG=zapp_wired=trace cargo run -p zapp-wired --example flash_left -- fw.zsab
//! ```
//!
//! The keyboard has to be in its bootloader: press the reset button on the left
//! half first.

use std::process::ExitCode;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use indicatif::{ProgressBar, ProgressStyle};

use zapp_core::flash::FlashProgress;
use zapp_smp::{bundle, image, Half, ImageSlot, SmpClient};
use zapp_wired::ignition::{usb::UsbLink, Halves, Ignition, IgnitionLink, SessionState};
use zapp_wired::{LeftTransport, WiredError};

/// How long to give ignition to open the session.
///
/// A left-only session skips the right-entry sequence and goes active on the
/// first tick, so this is only ever hit by a bootloader that is stuck.
const BEGIN_TIMEOUT: Duration = Duration::from_secs(10);

/// How long to wait for one SMP reply.
///
/// Longer than the product's 15 s on purpose: a first run against new firmware
/// wants to see a slow answer rather than a timeout that hides one.
const REPLY_TIMEOUT: Duration = Duration::from_secs(30);

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp(None)
        .init();

    let options = match Options::parse(std::env::args().skip(1)) {
        Ok(Some(options)) => options,
        Ok(None) => return ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("{err}");
            eprintln!();
            eprintln!("{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    match run(&options) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!();
            eprintln!("Failed: {err}");
            ExitCode::FAILURE
        }
    }
}

const USAGE: &str = "\
usage: flash_left [--force] [--keep] <bundle.zsab>

  --force  write the image even if the left half already carries it
  --keep   leave the half in MCUboot rather than releasing MD and resetting,
           so the next run does not have to re-enter the bootloader

The right half is not touched and does not have to be present.";

struct Options {
    path: String,
    force: bool,
    keep: bool,
}

impl Options {
    /// `Ok(None)` for `--help`, which is a request rather than a mistake.
    fn parse(args: impl Iterator<Item = String>) -> Result<Option<Self>, String> {
        let mut path = None;
        let mut force = false;
        let mut keep = false;

        for arg in args {
            match arg.as_str() {
                "--force" => force = true,
                "--keep" => keep = true,
                "-h" | "--help" => {
                    println!("{USAGE}");
                    return Ok(None);
                }
                other if other.starts_with('-') => {
                    return Err(format!("{other:?} is not an option this example takes"))
                }
                other if path.is_none() => path = Some(other.to_string()),
                other => return Err(format!("only one bundle at a time, and {other:?} is a second")),
            }
        }

        match path {
            Some(path) => Ok(Some(Self { path, force, keep })),
            None => Err("no bundle given".to_string()),
        }
    }
}

fn run(options: &Options) -> Result<(), String> {
    // ---- The file --------------------------------------------------------
    let blob = std::fs::read(&options.path).map_err(|err| format!("{}: {err}", options.path))?;
    let bundle = bundle::parse(&blob).map_err(|err| format!("{}: {err}", options.path))?;
    let wanted = bundle
        .hash(Half::Left)
        .ok_or("the bundle's left image carries no hash, so nothing could be checked")?;
    let left = bundle.image(Half::Left);

    println!(
        "Bundle {} · {} · {} bytes",
        options.path,
        version_of(&bundle.header.build_version),
        blob.len()
    );
    println!(
        "  left   v{} · {} bytes · {}",
        image::inspect(left)
            .map(|info| info.version)
            .unwrap_or_else(|| "?".to_string()),
        left.len(),
        short(&wanted)
    );
    println!("  right  not touched by this example");

    // ---- The board -------------------------------------------------------
    let link = match UsbLink::open() {
        Ok(link) => link,
        Err(WiredError::NoBootloader) => {
            return Err("no ZSA bootloader is plugged in. Press the reset button on the \
                        left half — that is what puts the STM32 into ignition, and \
                        everything here goes through ignition."
                .to_string())
        }
        Err(err) => return Err(format!("could not open the bootloader: {err}")),
    };

    let shared = Shared::new(link);
    let mut ignition = Ignition::new(shared.clone());

    // ---- Open the session, left half only --------------------------------
    //
    // `Halves::LEFT` rather than `BOTH`: ignition's entry sequence checks the
    // bitmap and skips the right half's reboot-over-I2C and mailbox poll
    // entirely when it is not named, so nothing in this run so much as touches
    // the TRRS bus. That is what lets it work with no right half attached.
    println!();
    println!("Opening a session on the left half");
    ignition
        .begin(Halves::LEFT)
        .map_err(|err| format!("BEGIN was refused: {err}"))?;
    let status = wait_active(&mut ignition)?;
    println!(
        "  active · {} bytes per message · pipe v{}",
        status.max_payload, status.proto_version
    );

    ignition
        .select(Half::Left)
        .map_err(|err| format!("SELECT was refused: {err}"))?;

    // ---- What it is carrying ---------------------------------------------
    let budget = status.payload_budget();
    let mut client = SmpClient::new(Box::new(LeftTransport::with_payload_budget(
        Ignition::new(shared.clone()),
        budget,
    )))
    .with_timeout(REPLY_TIMEOUT);
    println!("  {} bytes per SMP request", client.request_limit());

    println!();
    println!("Reading what is installed");
    let before = client
        .image_state()
        .map_err(|err| explain("could not read the image state", &err.to_string()))?;
    report(&before);

    let mut wrote = false;
    if installed(&before, &wanted) && !options.force {
        println!("  it already carries this image — pass --force to write it anyway");
    } else {
        // ---- Write it ----------------------------------------------------
        println!();
        println!("Uploading {} bytes", left.len());
        let started = Instant::now();
        upload(&mut client, left)?;
        let took = started.elapsed();
        println!(
            "  {} bytes in {:.1} s ({:.1} KiB/s)",
            left.len(),
            took.as_secs_f64(),
            left.len() as f64 / took.as_secs_f64() / 1024.0
        );

        // ---- And check it landed -----------------------------------------
        //
        // MCUboot's serial recovery writes the primary slot directly, so what
        // it reports now is the image it will actually boot. An upload that
        // reported success and a slot that reports something else is the
        // failure this whole example exists to be able to see.
        println!();
        println!("Reading it back");
        let after = client
            .image_state()
            .map_err(|err| explain("could not read the image state back", &err.to_string()))?;
        report(&after);
        if !installed(&after, &wanted) {
            return Err(format!(
                "the half took every byte and then reported {} in its primary slot, not the \
                 {} that was sent — the upload did not land",
                primary(&after)
                    .map(|slot| short(&slot.hash))
                    .unwrap_or_else(|| "no slot at all".to_string()),
                short(&wanted)
            ));
        }
        println!("  it carries what was sent");
        wrote = true;
    }

    // ---- Put it back -----------------------------------------------------
    println!();
    if options.keep {
        println!("Leaving the left half in MCUboot (--keep).");
        println!("  the MD pin is still held, so it re-enters recovery on any reset");
    } else {
        // Before the reset, always. The MD pin is what put the half into
        // MCUboot; a reset with it still held lands straight back in recovery.
        ignition
            .release_md()
            .map_err(|err| format!("RELEASE_MD was refused: {err}"))?;
        ignition
            .reset_target(Half::Left)
            .map_err(|err| format!("RESET_TARGET was refused: {err}"))?;
        println!("Released the MD pin and restarted the left half.");
    }

    ignition
        .end()
        .map_err(|err| format!("END was refused: {err}"))?;
    println!(
        "Session ended. {}",
        match (wrote, options.keep) {
            (true, false) => "The left half is booting the image from the bundle.",
            (true, true) => "The image is written; reset the half to boot it.",
            (false, false) => "Nothing was written.",
            (false, true) => "Nothing was written, and the half is still in MCUboot.",
        }
    );
    Ok(())
}

/// Polls STATUS until the session is open.
///
/// No sleep in here on purpose: each poll is a USB control transfer, which is
/// its own pacing, and a fixed wait is exactly what this rewrite got rid of.
fn wait_active<L: IgnitionLink>(
    ignition: &mut Ignition<L>,
) -> Result<zapp_wired::Status, String> {
    let started = Instant::now();
    let mut polls = 0u32;
    loop {
        let status = ignition
            .status()
            .map_err(|err| format!("could not read STATUS: {err}"))?;
        polls += 1;
        match status.state {
            SessionState::Active => {
                log::debug!("active after {polls} polls, {:?}", started.elapsed());
                return Ok(status);
            }
            SessionState::Pending => {}
            SessionState::Fault => {
                return Err(format!(
                    "the bootloader faulted opening the session (error {:#06x}). A left-only \
                     session touches nothing but the MD pin and the reset line, so this is \
                     the bootloader itself rather than anything about the halves.",
                    status.error
                ))
            }
            SessionState::Idle => {
                return Err("the bootloader says no session is open, having just been asked \
                            to open one. It is running a build without the wired update path \
                            in it."
                    .to_string())
            }
            SessionState::Unrecognised(code) => {
                return Err(format!(
                    "the bootloader reports session state {code}, which this build does not \
                     know — it and this example are not from the same release"
                ))
            }
        }
        if started.elapsed() >= BEGIN_TIMEOUT {
            return Err(format!(
                "the session was still opening after {} s and {polls} polls. A left-only \
                 session goes active on the bootloader's first tick, so its main loop is \
                 not running.",
                BEGIN_TIMEOUT.as_secs()
            ));
        }
    }
}

fn upload(client: &mut SmpClient, image: &[u8]) -> Result<(), String> {
    let bar = ProgressBar::new(image.len() as u64);
    bar.set_style(
        ProgressStyle::default_bar()
            .template("⚡ {bar:40.cyan/blue} {bytes}/{total_bytes} {bytes_per_sec} {msg}")
            .unwrap()
            .progress_chars("█▓░"),
    );
    let result = client.upload(0, image, &|progress| match progress {
        FlashProgress::Writing { bytes_written, .. } => {
            bar.set_position(bytes_written as u64);
            bar.set_message("Uploading...");
        }
        FlashProgress::Complete => bar.finish_with_message("Done"),
        _ => {}
    });
    bar.finish_and_clear();
    result.map_err(|err| explain("the upload failed", &err.to_string()))
}

/// A failure, with whatever this end can add to it.
///
/// The left half is a UART all the way down, so the two things that go wrong
/// are worth naming by hand: a half that is not in MCUboot answers nothing at
/// all, and a codec disagreement shows up as frames that never reassemble.
fn explain(what: &str, err: &str) -> String {
    let hint = if err.contains("no reply") || err.contains("stopped taking") {
        "\n  Nothing came back. Either the left half is not in MCUboot — check the STATUS \
         line above said so — or its serial recovery is not built in. Ignition's UART is \
         the only link in this path with no diagnostic of its own, so silence here is all \
         there is to go on: RUST_LOG=zapp_wired=trace shows the lines going out."
    } else if err.contains("checksum") || err.contains("line") {
        "\n  The bytes came back mangled, which is the codec rather than the half: the \
         bootloader and this example disagree about the line format."
    } else {
        ""
    };
    format!("{what}: {err}{hint}")
}

// ---------------------------------------------------------------------------
// Reading the board back
// ---------------------------------------------------------------------------

/// The slot a recovery upload writes. Serial recovery goes straight into the
/// primary slot rather than staging, so slot 0 is the only one to compare.
fn primary(slots: &[ImageSlot]) -> Option<&ImageSlot> {
    slots.iter().find(|slot| slot.slot == 0)
}

fn installed(slots: &[ImageSlot], wanted: &[u8]) -> bool {
    primary(slots).is_some_and(|slot| slot.hash == wanted)
}

fn report(slots: &[ImageSlot]) {
    if slots.is_empty() {
        println!("  it reports no images at all");
        return;
    }
    for slot in slots {
        println!(
            "  slot {} · {} · {}{}",
            slot.slot,
            version_of(&slot.version),
            match slot.hash.is_empty() {
                true => "no hash".to_string(),
                false => short(&slot.hash),
            },
            match slot.flags().join(" ") {
                flags if flags.is_empty() => String::new(),
                flags => format!(" · {flags}"),
            }
        );
    }
}

/// Enough of a hash to recognise: a SHA-512 is 128 characters of hex.
fn short(hash: &[u8]) -> String {
    let hex = image::hex(hash);
    match hex.len() > 20 {
        true => format!("{}…{}", &hex[..8], &hex[hex.len() - 8..]),
        false => hex,
    }
}

fn version_of(version: &str) -> String {
    match version.is_empty() {
        true => "(no version)".to_string(),
        false => version.to_string(),
    }
}

// ---------------------------------------------------------------------------
// The link
// ---------------------------------------------------------------------------

/// One link, held by the control path and by the transport at the same time.
///
/// `SmpClient` owns its transport and never gives it back, so the vendor
/// requests that come after the upload — RELEASE_MD, RESET_TARGET, END — need a
/// second handle on the same link. Nothing here is concurrent; the mutex is
/// shared ownership rather than synchronisation.
///
/// Deliberately spelled out here rather than shared with `zapp`: this example
/// is meant to be read top to bottom by whoever is debugging the pipe, and the
/// one non-obvious thing about the plumbing should be on the page.
struct Shared<L: IgnitionLink>(Arc<Mutex<L>>);

impl<L: IgnitionLink> Shared<L> {
    fn new(link: L) -> Self {
        Self(Arc::new(Mutex::new(link)))
    }
}

impl<L: IgnitionLink> Clone for Shared<L> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<L: IgnitionLink> IgnitionLink for Shared<L> {
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
        self.0.lock().unwrap().control_out(request, value)
    }

    fn control_in(&mut self, request: u8, value: u16, length: usize) -> Result<Vec<u8>, WiredError> {
        self.0.lock().unwrap().control_in(request, value, length)
    }

    fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
        self.0.lock().unwrap().bulk_out(message)
    }

    fn bulk_in(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError> {
        self.0.lock().unwrap().bulk_in(timeout)
    }
}
