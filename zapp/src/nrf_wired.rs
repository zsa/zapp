//! Updating both nRFs over the cable, through ignition.
//!
//! The Bluetooth path in [`crate::zsab`] has to talk the halves into their own
//! bootloaders and then reach the right one through the left's radio relay.
//! Over the wire none of that applies: ignition drives both halves' entry
//! itself — the left over its UART, the right by writing the reboot flag over
//! the TRRS I2C bus — and the host reaches both through one vendor interface on
//! the STM32. So there is no update session to declare, no battery gate, no
//! reconnect between halves, and nothing that waits on a radio.
//!
//! What is left is an order and a set of checks, which is what this file is.
//! Every wait polls: either ignition's STATUS or an SMP reply. There are no
//! fixed sleeps anywhere in here, and there is nothing to tear down and rebuild
//! between the two halves.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use indicatif::{ProgressBar, ProgressStyle};

use zapp_core::flash::FlashProgress;
use zapp_smp::bundle::{self, Bundle};
use zapp_smp::proto::{self, Op};
use zapp_smp::{cbor, image, Half, ImageSlot, SmpClient, SmpError};
use zapp_wired::ignition::{usb::UsbLink, Halves, SessionState, TargetState};
use zapp_wired::{
    Ignition, IgnitionLink, LeftTransport, Retrying, RightTransport, Status, WiredError,
};

use crate::zsab::{named, short, wrap};

/// How long to give ignition's entry sequence.
///
/// Its own mailbox poll gives up after 5 s and moves the session to FAULT, so
/// this only has to outlast that — a host that waits longer than the firmware
/// is waiting for nothing.
const BEGIN_TIMEOUT: Duration = Duration::from_secs(15);

/// The zephyr basic group, and the command that erases the settings partition.
const BASIC_GROUP: u16 = 63;
const BASIC_CMD_ERASE_STORAGE: u8 = 0;

/// `MGMT_ERR_ENOTSUP`: a build that does not carry the command at all.
const MGMT_ERR_ENOTSUP: i64 = 8;

/// What the diagnostic scan probes, in the order ignition reports it.
const SCANNED: [(u8, &str); 5] = [
    (0x20, "MCP23018, the right half's matrix expander"),
    (0x45, "the right nRF's MCUboot mailbox"),
    (0x54, "the right nRF's application"),
    (0x74, "IS31FL3731, the left half's LEDs"),
    (0x77, "IS31FL3731, the right half's LEDs"),
];

// ---------------------------------------------------------------------------
// Subcommands
// ---------------------------------------------------------------------------

/// Writes both halves from one bundle.
pub fn flash(path: &Path, force: bool, timeout: Duration) -> Result<()> {
    let blob = std::fs::read(path).with_context(|| format!("Failed to read {}", path.display()))?;
    let bundle = bundle::parse(&blob)
        .with_context(|| format!("{} is not a bundle this build can read", path.display()))?;
    announce(&bundle, path, blob.len());

    let shared = Shared::new(open()?);
    let mut ignition = Ignition::new(shared.clone());

    match run(&shared, &mut ignition, &bundle, force, timeout) {
        Ok(()) => {
            // Only once everything landed: an END on the way out of a failure
            // would throw away the STATUS the failure is explained from.
            if let Err(err) = ignition.end() {
                log::debug!("could not end the session: {err}");
            }

            // Leave a keyboard, not a bootloader. Both halves are back on
            // their own firmware by this point, and the STM32 is the only
            // thing still sitting in recovery — it has no reason to, and
            // otherwise the update ends with the board needing a power cycle
            // before it types anything. The bootloader answers this and then
            // reboots, so the device vanishing is the success case.
            match ignition.reset_self() {
                Ok(()) => {}
                Err(err) => log::debug!("the bootloader went down with the reset: {err}"),
            }

            println!("Done!");
            Ok(())
        }
        Err(err) => {
            // Asked before anything is put back and put back before anything
            // is printed, in that order and for two different reasons: the
            // notes are read out of a session that is still open, and the next
            // attempt needs one that is not — the bootloader refuses BEGIN
            // outright while a session is active.
            let notes = advice(&mut ignition, &err);
            if let Err(err) = ignition.end() {
                log::debug!("could not end the session after the failure: {err}");
            }

            println!();
            println!("Update failed: {err:#}");
            for note in notes {
                println!();
                println!("{note}");
            }
            println!();
            println!("{}", wrap(LEFT_BEHIND));
            bail!("the keyboard was not fully updated")
        }
    }
}

/// What is left on a keyboard whose wired update stopped part-way.
///
/// Short, because there is genuinely little to say: a recovery upload writes
/// the primary slot, so an interrupted one leaves a half with no image rather
/// than with its old one, and the way out is the same command again.
const LEFT_BEHIND: &str = "Both halves are still in their bootloaders and will stay there \
     until something resets them, which is what makes this recoverable: the session has been \
     closed, so running the same command again picks up from where this stopped. A half whose \
     upload did not finish has a part-written slot and will not boot, so leave the keyboard \
     plugged in rather than power-cycling it. `zapp nrf status` says what the bootloader can \
     still see.";

/// Ignition's STATUS, and what answers on the TRRS bus.
pub fn status() -> Result<()> {
    let mut ignition = Ignition::new(open()?);
    let status = ignition
        .status()
        .context("Could not read ignition's status")?;
    print_status(&status);

    println!();
    println!("TRRS I2C scan");
    match ignition.i2c_scan() {
        Ok(scan) => {
            for line in scan_lines(&scan) {
                println!("  {line}");
            }
        }
        Err(err) => println!("  could not scan: {err}"),
    }

    if let Some(note) = session_error(&status) {
        println!();
        println!("{}", wrap(&note));
    }
    Ok(())
}

/// Closes a session someone else left open, and lets go of the left's MD pin.
pub fn end() -> Result<()> {
    let mut ignition = Ignition::new(open()?);
    let before = ignition
        .status()
        .context("Could not read ignition's status")?;

    // MD first. END releases it too, but a bootloader that answers the first
    // request and not the second should at least have let go of the pin that
    // keeps the left half out of its own firmware.
    if let Err(err) = ignition.release_md() {
        log::debug!("could not release the MD pin: {err}");
    }
    ignition.end().context("Could not end ignition's session")?;

    match before.state {
        SessionState::Idle => println!("No session was open; the MD pin is released either way."),
        state => println!("Ended a session that was {}.", session_as(state)),
    }
    print_status(
        &ignition
            .status()
            .context("Could not read ignition's status")?,
    );
    println!();
    println!(
        "{}",
        wrap(
            "Neither half was reset, so a half that was already in MCUboot is still there. \
             Power-cycle the keyboard to boot both halves, or run \
             `zapp nrf flash <bundle.zsab>` to write them."
        )
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// The flash itself
// ---------------------------------------------------------------------------

fn run<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    bundle: &Bundle,
    force: bool,
    timeout: Duration,
) -> Result<()> {
    let mut steps = Steps { total: 3, done: 0 };

    steps.announce("Opening a session on both halves");
    ignition
        .begin(Halves::BOTH)
        .context("Could not open a session on the bootloader")?;
    let status = wait_active(ignition)?;
    log::debug!(
        "session active: TRRS {} kHz, {} bytes per message, pipe v{}, left {}, right {}",
        status.i2c_khz,
        status.max_payload,
        status.proto_version,
        running(status.left_state),
        running(status.right_state)
    );

    // The right half first. Not for the reason the Bluetooth path has one —
    // there is no relay here and nothing depends on the left's application
    // being alive — but because the right is the half that can fail to come
    // back at all, and finding that out before the left has been rewritten
    // leaves a keyboard with one working half rather than none.
    for half in [Half::Right, Half::Left] {
        steps.announce(&format!("{} half", titled(half)));
        write_half(shared, ignition, half, bundle, &status, force, timeout)?;
    }
    Ok(())
}

/// One half: what it is carrying, what it needs, and the reset that boots it.
fn write_half<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    half: Half,
    bundle: &Bundle,
    status: &Status,
    force: bool,
    timeout: Duration,
) -> Result<()> {
    ignition
        .select(half)
        .with_context(|| format!("Could not point the bridge at the {half} half"))?;

    let mut client = connect(shared, half, status.payload_budget(), timeout);
    let wanted = bundle
        .hash(half)
        .with_context(|| format!("The bundle's {half} image carries no hash to check it by"))?;

    let before = client
        .image_state()
        .with_context(|| format!("Could not read the {half} half's image state"))?;
    report_slot(&before);

    if installed(&before, &wanted) && !force {
        log::debug!("{half}: already carrying this image, not written again");
    } else {
        // Refuse to go backwards. A bundle built from a stale directory writes
        // an old half onto a keyboard whose other half has moved on, and two
        // halves on different builds is a keyboard that hangs the moment they
        // are wired together — which is exactly how this check came to exist.
        // The versions are already in hand and were already being printed;
        // nothing was comparing them.
        if let Some(older) = (!force)
            .then(|| going_backwards(&before, bundle.image(half), half))
            .flatten()
        {
            bail!("{older}");
        }
        // The right half only. Its settings partition holds the pairing state
        // the two halves share, and a right that comes back on a new build
        // reading an old one is the failure this avoids.
        if half == Half::Right {
            erase_storage(&mut client)?;
        }

        let image = bundle.image(half);
        upload(&mut client, half, image)?;

        // Read back rather than trust the upload's own success. A recovery
        // upload writes the primary slot directly, so what the board reports
        // afterwards is the image it will boot, and it is the only evidence
        // that it is the image that was sent.
        let after = client
            .image_state()
            .with_context(|| format!("Could not read the {half} half's image state back"))?;
        landed(&after, &wanted, half)?;
        log::debug!(
            "{half}: {} bytes written, slot 0 now carries {}",
            image.len(),
            short(&wanted)
        );
    }

    // Both halves are restarted whether or not they were written: ignition's
    // BEGIN put them in their bootloaders, and END does not take them out.
    // How they leave differs, because only the left has a reset line.
    match half {
        Half::Left => {
            // Or it comes straight back into MCUboot, MD still held.
            ignition
                .release_md()
                .context("Could not release the left half's MD pin")?;
            log::debug!("released the MD pin so the left boots its application");
            ignition
                .reset_target(half)
                .context("Could not restart the left half")?;
        }
        Half::Right => {
            // RESET_TARGET is the wrong instrument here: with the MCP23018
            // path gone the STM32 cannot reach the right half's reset line,
            // so it asks the right's *application* to reboot into MCUboot —
            // the wrong direction, and unanswerable anyway while MCUboot is
            // what is running. Leaving serial recovery is MCUboot's own job,
            // and its SMP reset is how you ask.
            client
                .reset()
                .context("Could not restart the right half out of MCUboot")?;

            // Point the bridge away before the reboot finishes. Ignition polls
            // whichever half is selected, so leaving it on the right means it
            // keeps addressing a mailbox on a chip that is coming back up —
            // one of those polls lands mid-reboot and costs a bus recovery,
            // reliably one per update. The left is where the run goes next.
            ignition
                .select(Half::Left)
                .context("Could not point the bridge away from the right half")?;
        }
    }
    log::debug!("{half}: restarting");
    Ok(())
}

/// Polls STATUS until the entry sequence is done.
///
/// The whole reason `BEGIN` answers immediately: ignition's control callback
/// returns and its main loop runs the multi-second right-entry sequence, so the
/// host watches the state move rather than guessing at how long to wait. Each
/// poll is a USB control transfer, which is its own pacing.
fn wait_active<L: IgnitionLink>(ignition: &mut Ignition<L>) -> Result<Status> {
    let started = Instant::now();
    loop {
        let status = ignition
            .status()
            .context("Could not read ignition's status")?;
        match status.state {
            SessionState::Active => return Ok(status),
            SessionState::Fault => {
                let reason = session_error(&status)
                    .unwrap_or_else(|| "the bootloader gave no reason".to_string());
                bail!("The bootloader could not open a session: {reason}");
            }
            SessionState::Idle => bail!(
                "The bootloader says no session is open, having just been asked to open one. \
                 It is running a build that does not carry the wired update path."
            ),
            SessionState::Pending => {}
            SessionState::Unrecognised(code) => bail!(
                "The bootloader reports session state {code}, which this build does not know. \
                 It and this zapp are not from the same release."
            ),
        }
        if started.elapsed() >= BEGIN_TIMEOUT {
            bail!(
                "The bootloader was still opening the session {} s after it was asked to. \
                 Its own entry sequence gives up well inside that, so it is stuck rather \
                 than slow.",
                BEGIN_TIMEOUT.as_secs()
            );
        }
    }
}

/// An SMP client onto one half, over the wired stack.
fn connect<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    half: Half,
    payload_budget: usize,
    timeout: Duration,
) -> SmpClient {
    let ignition = Ignition::new(shared.clone());
    // Not `negotiate_request_limit`: MCUboot's serial recovery has no
    // `os mcumgr_params` command, so asking costs one whole timeout and
    // changes nothing. The pipe's own budget is the real limit here.
    let transport: Box<dyn zapp_smp::SmpTransport> = match half {
        Half::Left => Box::new(LeftTransport::with_payload_budget(ignition, payload_budget)),
        Half::Right => Box::new(RightTransport::with_payload_budget(ignition, payload_budget)),
    };
    SmpClient::new(transport).with_timeout(timeout)
}

/// Clears the right half's settings partition.
///
/// A build without the command is a warning rather than a failure: it is the
/// bootloader saying it has never heard of the question, which is not the same
/// as refusing to answer it.
fn erase_storage(client: &mut SmpClient) -> Result<()> {
    let empty = cbor::encode(&cbor::map(vec![]));
    let reply = client
        .transact(Op::Write, BASIC_GROUP, BASIC_CMD_ERASE_STORAGE, &empty)
        .context("Could not erase the right half's settings partition")?;

    match cbor::decode(&reply).and_then(|value| proto::check_rc(&value, BASIC_GROUP)) {
        Ok(()) => {
            log::debug!("erased the settings partition");
            Ok(())
        }
        Err(SmpError::Remote {
            rc: MGMT_ERR_ENOTSUP,
            ..
        }) => {
            log::debug!("this MCUboot has no storage-erase command; leaving the settings alone");
            Ok(())
        }
        Err(err) => Err(err).context("Could not erase the right half's settings partition"),
    }
}

/// Sends one half's image, with a bar on it.
fn upload(client: &mut SmpClient, half: Half, image: &[u8]) -> Result<()> {
    let bar = ProgressBar::new(image.len() as u64);
    bar.set_style(
        ProgressStyle::default_bar()
            // The half's name rides next to the bolt rather than trailing the
            // bar: it says what is being written, so it belongs at the front,
            // and padding it keeps the two bars lined up under each other.
            .template("⚡ {msg:<5} {bar:40.cyan/blue} {bytes}/{total_bytes} {bytes_per_sec}")
            .unwrap()
            .progress_chars("█▓░"),
    );
    let label = titled(half);
    let result = client.upload(0, image, &|progress| match progress {
        FlashProgress::Writing { bytes_written, .. } => {
            bar.set_position(bytes_written as u64);
            bar.set_message(label);
        }
        FlashProgress::Complete => bar.finish_with_message(label),
        _ => {}
    });
    bar.finish_and_clear();
    result.with_context(|| format!("The {half} half's upload failed"))
}

// ---------------------------------------------------------------------------
// What the board is carrying
// ---------------------------------------------------------------------------

/// The slot a recovery upload writes.
///
/// MCUboot's serial recovery writes the primary slot directly rather than
/// staging into the spare, so slot 0 is both what an upload here changes and
/// the only slot worth comparing anything against. A device that leaves the
/// number out is describing its one slot, which `img::parse_state` already
/// reads as zero.
fn primary(slots: &[ImageSlot]) -> Option<&ImageSlot> {
    slots.iter().find(|slot| slot.slot == 0)
}

/// Whether the primary slot already holds this exact image.
/// `major.minor.patch` out of an MCUboot version, ignoring any `+build`.
///
/// Anything unparseable returns None, which is read as "cannot tell" and lets
/// the write through: refusing on a version string nobody can read would be a
/// worse failure than the one this guards against.
fn ordering_of(version: &str) -> Option<(u32, u32, u32)> {
    let mut parts = version.split('+').next()?.split('.');
    let major = parts.next()?.parse().ok()?;
    let minor = parts.next()?.parse().ok()?;
    let patch = parts.next().unwrap_or("0").parse().ok()?;
    Some((major, minor, patch))
}

/// Says so if the bundle would put an older image on than the half carries.
///
/// Two halves on different builds is a keyboard whose halves cannot talk over
/// the wire between them, and the way boards get there is a bundle built from
/// a directory that was rebuilt afterwards — the versions are right there in
/// the output, going backwards, and nothing was reading them.
fn going_backwards(slots: &[ImageSlot], image: &[u8], half: Half) -> Option<String> {
    let installed = ordering_of(&slots.iter().find(|slot| slot.slot == 0)?.version)?;
    let offered_text = image::inspect(image)?.version;
    let offered = ordering_of(&offered_text)?;
    if offered >= installed {
        return None;
    }
    let (a, b, c) = installed;
    Some(format!(
        "The bundle's {half} image is v{offered_text}, older than the v{a}.{b}.{c} that half \
         is already carrying. Writing it would leave the halves on different builds, which is \
         a keyboard whose halves cannot talk to each other over the wire between them — the \
         usual cause is a bundle built from a directory that was rebuilt afterwards, so check \
         the bundle is the one you meant. Pass --force if the downgrade is deliberate."
    ))
}

fn installed(slots: &[ImageSlot], wanted: &[u8]) -> bool {
    primary(slots).is_some_and(|slot| slot.hash == wanted)
}

/// Fails unless the image that was written is the image the board now reports.
fn landed(slots: &[ImageSlot], wanted: &[u8], half: Half) -> Result<()> {
    let Some(slot) = primary(slots) else {
        bail!(
            "The {half} half took the whole image and then reported no primary slot at all, \
             so there is nothing to check it against. Nothing on that half can be trusted \
             until it is read again."
        );
    };
    if slot.hash == wanted {
        return Ok(());
    }
    if slot.hash.is_empty() {
        bail!(
            "The {half} half took the whole image and then reported an empty hash for its \
             primary slot, which is what a slot MCUboot could not verify looks like. The \
             image on it is not bootable.",
        );
    }
    bail!(
        "The {half} half took the whole image and then reported {} in its primary slot, not \
         the {} that was sent. The upload did not land where it was meant to.",
        short(&slot.hash),
        short(wanted)
    )
}

fn report_slot(slots: &[ImageSlot]) {
    match primary(slots) {
        Some(slot) if slot.hash.is_empty() => {
            log::debug!("slot 0 is empty or unverifiable")
        }
        Some(slot) => log::debug!(
            "slot 0 carries {} ({})",
            short(&slot.hash),
            named(&slot.version)
        ),
        None => log::debug!("it reports no primary slot"),
    }
}

// ---------------------------------------------------------------------------
// Saying what went wrong
// ---------------------------------------------------------------------------

/// Everything worth printing under a failure.
///
/// The board is asked again rather than remembered: STATUS carries the reason
/// the bootloader itself recorded, the bus recovery count and the bytes it had
/// to drop, and all three are current at the moment the failure is being
/// explained rather than at the last time anything happened to look.
fn advice<L: IgnitionLink>(ignition: &mut Ignition<L>, err: &anyhow::Error) -> Vec<String> {
    let mut notes = Vec::new();
    if let Some(note) = event_advice(err) {
        notes.push(wrap(&note));
    }
    match ignition.status() {
        Ok(status) => {
            if let Some(note) = session_error(&status) {
                notes.push(wrap(&note));
            } else if let Some(note) = bus_health(&status) {
                notes.push(wrap(&note));
            }
        }
        Err(err) => notes.push(wrap(&format!(
            "The bootloader stopped answering as well ({err}), so there is nothing left to \
             ask about what happened. Unplug the keyboard and plug it back in."
        ))),
    }
    notes
}

/// The bootloader's own reason, in a sentence.
///
/// `None` for a session that recorded none, and for a code from a firmware
/// newer than this build — the raw number is more honest than a guess.
fn session_error(status: &Status) -> Option<String> {
    let bus = bus_phrase(status);
    Some(match status.error {
        0x0000 => return None,
        0x0001 => "The left half never answered over the UART between it and the STM32. It is \
             the one link in this path with no diagnostic of its own, so this is either a \
             left half that is not in MCUboot or one whose serial recovery is not built in."
            .to_string(),
        0x0002 => format!(
            "The right half never brought up its MCUboot mailbox on the TRRS bus ({bus}). \
             The STM32 asks it to reboot into recovery over that same bus, so either the \
             ask did not reach it or it did not come back — check the TRRS cable and that \
             the right half has charge in it."
        ),
        0x0003 => format!(
            "The TRRS cable came out, or never made contact ({bus}). Nothing reaches the \
             right half without it. Reseat both ends."
        ),
        0x0004 => format!(
            "The right half's matrix expander did not acknowledge on the TRRS bus ({bus}). \
             That chip answers whenever the bus is wired and powered, so this is the cable \
             or the right half's supply rather than anything to do with the update."
        ),
        0x0005 => format!(
            "The right half stopped answering on the TRRS bus ({bus}) — check the cable. \
             The bus is shared with the matrix and the LEDs, so a marginal connection \
             shows up here first."
        ),
        0x0006 => "The bootloader is bridging to a serial port that is not open. Nothing on \
             this machine has the CDC interface claimed, which on Linux usually means the \
             udev rule for it is not installed."
            .to_string(),
        0x0007 => "A line came back from a half longer than the bootloader's buffer, so it \
             was dropped. The two ends disagree about the line codec, which means the \
             bootloader and this zapp are not from the same release."
            .to_string(),
        0x0008 => "Frames were sent before a half was chosen. That is this zapp getting the \
             order wrong rather than anything about the keyboard."
            .to_string(),
        0x0009 => format!(
            "The bridge gave up waiting for a half to answer ({bus}). Whichever half was \
             selected went quiet part-way through an exchange."
        ),
        0x00FF => "The bootloader hit an internal error. There is nothing to act on from \
             here: unplug the keyboard, plug it back in, and try once more."
            .to_string(),
        other => format!(
            "The bootloader reported error {other:#06x}, which this build does not have a \
             sentence for. It is running a newer firmware than this zapp knows."
        ),
    })
}

/// A note about a bus that coped but had to work at it.
///
/// Only when the session recorded no error of its own: a run that failed
/// somewhere else while the bus was recovering repeatedly is worth pointing at
/// the bus anyway.
fn bus_health(status: &Status) -> Option<String> {
    if status.recover_count == 0 && status.rx_drops == 0 {
        return None;
    }
    Some(format!(
        "The bootloader recorded no fault, but the TRRS bus did not have an easy time of it \
         ({}). That is the first thing to look at: reseat the cable and try again.",
        bus_phrase(status)
    ))
}

/// "3 bus recoveries at 50 kHz, 7 bytes dropped".
fn bus_phrase(status: &Status) -> String {
    let mut parts = vec![format!(
        "{} bus recover{} at {} kHz",
        status.recover_count,
        if status.recover_count == 1 { "y" } else { "ies" },
        status.i2c_khz
    )];
    if status.rx_drops > 0 {
        parts.push(format!("{} bytes dropped on the way back", status.rx_drops));
    }
    parts.join(", ")
}

/// The bridge's own diagnostic, when one is what failed the transfer.
///
/// A fatal `EVENT` comes back through `SmpClient` as a transport error carrying
/// the event's own words, which is the only thing left of it by the time it
/// reaches here — so the codes are matched by the sentence each one prints.
/// `None` for every other failure, including an event code from a newer
/// firmware, which prints its own number and needs nothing added.
fn event_advice(err: &anyhow::Error) -> Option<String> {
    use zapp_wired::pipe::EventCode::*;

    let text = format!("{err:#}");
    let reported = |code: zapp_wired::pipe::EventCode| text.contains(&code.to_string());

    if reported(MailboxGone) {
        return Some(
            "The right half's MCUboot mailbox stopped answering part-way through. That half \
             restarted, or the TRRS bus dropped out from under it — either way what was \
             written to it is incomplete."
                .to_string(),
        );
    }
    if reported(TargetNotSelected) {
        return Some(
            "The bootloader was sent frames with no half selected, which is this zapp \
             getting the order wrong rather than anything about the keyboard."
                .to_string(),
        );
    }
    if reported(RxOverflow) || reported(UartOverrun) {
        return Some(
            "The bootloader's receive buffer overflowed, so bytes coming back from a half \
             were lost. The two ends disagree about how much may be in flight, which means \
             the bootloader and this zapp are not from the same release."
                .to_string(),
        );
    }
    if reported(I2cTimeout) || reported(I2cNack) {
        return Some(
            "The right half stopped acknowledging on the TRRS bus. Reseat the cable at both \
             ends and try again."
                .to_string(),
        );
    }
    None
}

// ---------------------------------------------------------------------------
// Printing
// ---------------------------------------------------------------------------

fn announce(bundle: &Bundle, _path: &Path, _len: usize) {
    let sizes: Vec<String> = [Half::Left, Half::Right]
        .iter()
        .map(|half| format!("{} {} bytes", half.as_str(), bundle.image(*half).len()))
        .collect();
    println!(
        "Bundle loaded: {} ({})",
        named(&bundle.header.build_version),
        sizes.join(" + ")
    );
    for half in [Half::Left, Half::Right] {
        if let Some(info) = image::inspect(bundle.image(half)) {
            let named_hash = bundle.hash(half).map(|hash| short(&hash)).unwrap_or_default();
            log::debug!("{half}: v{} {named_hash}", info.version);
        }
    }
    log::debug!("radio key fingerprint {:08x}", bundle.header.key_fingerprint);
}

fn print_status(status: &Status) {
    println!("Ignition · session {}", session_as(status.state));
    println!("  target  {}", target_as(status.target));
    println!("  left    {}", running(status.left_state));
    println!("  right   {}", running(status.right_state));
    println!(
        "  TRRS    {} kHz · {} recover{} · {} bytes dropped",
        status.i2c_khz,
        status.recover_count,
        if status.recover_count == 1 { "y" } else { "ies" },
        status.rx_drops
    );
    println!(
        "  pipe    v{} · {} bytes per message",
        status.proto_version, status.max_payload
    );
    println!(
        "  error   {}",
        match status.error {
            0 => "none".to_string(),
            code => format!("{code:#06x}"),
        }
    );
}

/// One line per address ignition probed.
fn scan_lines(scan: &[u8]) -> Vec<String> {
    SCANNED
        .iter()
        .enumerate()
        .map(|(at, (address, what))| {
            let result = scan.get(at).copied().unwrap_or(0xFF);
            format!("{address:#04x}  {what:<40}  {}", answered(result))
        })
        .collect()
}

/// What one probe byte means. Zero is an ACK; the rest are ignition's own
/// negative error codes in a signed byte.
fn answered(result: u8) -> &'static str {
    match result as i8 {
        0 => "answered",
        -1 => "no answer (NACK)",
        -2 => "bus error",
        -3 => "timed out",
        _ => "a result this build does not know",
    }
}

fn session_as(state: SessionState) -> String {
    match state {
        SessionState::Idle => "idle".to_string(),
        SessionState::Active => "active".to_string(),
        SessionState::Fault => "faulted".to_string(),
        SessionState::Pending => "opening".to_string(),
        SessionState::Unrecognised(code) => format!("a state this zapp does not know ({code})"),
    }
}

fn target_as(target: zapp_wired::pipe::Target) -> &'static str {
    use zapp_wired::pipe::Target;
    match target {
        Target::Selected => "none selected",
        Target::Left => "the left half",
        Target::Right => "the right half",
        Target::Stm32 => "the STM32",
    }
}

fn running(state: TargetState) -> &'static str {
    match state {
        TargetState::Unknown => "not looked at",
        TargetState::App => "running its application",
        TargetState::McuBoot => "in MCUboot",
        TargetState::Stale => "answered once and has not since",
        TargetState::Unrecognised(_) => "a state this zapp does not know",
    }
}

fn titled(half: Half) -> &'static str {
    match half {
        Half::Left => "Left",
        Half::Right => "Right",
    }
}

/// Numbered phases, so a wait of a minute reads as progress.
struct Steps {
    total: usize,
    done: usize,
}

impl Steps {
    /// The steps are the progress bar's labels now rather than headings of
    /// their own: flashing a keyboard image prints four lines in total and
    /// this should not look like a different program.
    fn announce(&mut self, label: &str) {
        self.done += 1;
        log::debug!("step {}/{}: {label}", self.done, self.total);
    }
}

// ---------------------------------------------------------------------------
// The link
// ---------------------------------------------------------------------------

/// Finds the one plugged-in bootloader.
///
/// The link comes back wrapped in `Retrying`, which absorbs the control stall
/// ignition answers with in the gap `BEGIN` opens. Nothing above here has to
/// know that, which is the point of it being the link's own behaviour.
fn open() -> Result<Retrying<UsbLink>> {
    open_waiting(WAIT_FOR_BOOTLOADER)
}

/// How long to wait for someone to press the reset button.
///
/// Long enough to walk over to the keyboard, because that is the actual task.
const WAIT_FOR_BOOTLOADER: Duration = Duration::from_secs(60);

/// Finds the bootloader, waiting for it to show up rather than giving up.
///
/// The keyboard is nearly always running its own firmware when someone decides
/// to update it, so "not plugged in" is the normal starting state, not an
/// error. Waiting matches what flashing a QMK image already does and turns the
/// whole thing into one command and one button press.
fn open_waiting(limit: Duration) -> Result<Retrying<UsbLink>> {
    match UsbLink::open() {
        Ok(link) => {
            log::debug!("found the ZSA bootloader over USB");
            return Ok(link);
        }
        Err(WiredError::NoBootloader) => {}
        Err(err) => return Err(err).context("Could not open the ZSA bootloader"),
    }

    let spinner = new_spinner(
        "Waiting for the bootloader — press the reset button on the left half of the keyboard",
    );
    let deadline = Instant::now() + limit;

    loop {
        match UsbLink::open() {
            Ok(link) => {
                spinner.finish_and_clear();
                log::debug!("found the ZSA bootloader over USB");
                return Ok(link);
            }
            Err(WiredError::NoBootloader) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(250));
            }
            Err(WiredError::NoBootloader) => {
                spinner.finish_and_clear();
                bail!(
                    "No ZSA bootloader appeared in {} seconds.\n\n{}",
                    limit.as_secs(),
                    wrap(
                        "Pressing the reset button on the left half is what puts the STM32 into \
                         ignition, and ignition is what this whole path goes through. A keyboard \
                         running its own firmware does not serve any of it. `zapp devices` lists \
                         what is plugged in."
                    )
                )
            }
            Err(err) => {
                spinner.finish_and_clear();
                return Err(err).context("Could not open the ZSA bootloader");
            }
        }
    }
}

/// A spinner that says what is being waited for.
fn new_spinner(message: &str) -> ProgressBar {
    let spinner = ProgressBar::new_spinner();
    spinner.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.cyan} {msg}")
            .unwrap(),
    );
    spinner.set_message(message.to_string());
    spinner.enable_steady_tick(Duration::from_millis(100));
    spinner
}

/// One link, held by the control path and by a transport at the same time.
///
/// `SmpClient` owns its transport and never gives it back, so the vendor
/// requests that sit between the uploads — SELECT, RELEASE_MD, RESET_TARGET —
/// need a second handle on the same link. Nothing here is concurrent: the mutex
/// is shared ownership, not synchronisation.
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

    fn control_in(
        &mut self,
        request: u8,
        value: u16,
        length: usize,
    ) -> Result<Vec<u8>, WiredError> {
        self.0.lock().unwrap().control_in(request, value, length)
    }

    fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
        self.0.lock().unwrap().bulk_out(message)
    }

    fn bulk_in(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError> {
        self.0.lock().unwrap().bulk_in(timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_smp::{client, img};
    use zapp_wired::ignition;
    use zapp_wired::loopback::{LoopbackDevice, LoopbackLink};
    use zapp_wired::pipe;

    /// The hash the in-memory device reports for its one slot, whatever has
    /// been written to it.
    const DEVICE_HASH: [u8; 32] = [0x11; 32];

    /// A signed image whose trailer names it by `hash`.
    ///
    /// Versioned to match what the fake board reports it is carrying, so the
    /// downgrade guard has nothing to object to. A test that wants the guard
    /// to fire asks for an older one with `signed_v`.
    fn signed(payload: u8, hash: &[u8]) -> Vec<u8> {
        signed_v(payload, hash, (9, 9, 9))
    }

    /// The same, at a stated `major.minor.revision`.
    fn signed_v(payload: u8, hash: &[u8], version: (u8, u8, u16)) -> Vec<u8> {
        const MCUBOOT_MAGIC: u32 = 0x96f3_b83d;
        let body = vec![payload; 512];

        let mut out = Vec::new();
        out.extend_from_slice(&MCUBOOT_MAGIC.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // load address
        out.extend_from_slice(&32u16.to_le_bytes()); // header size
        out.extend_from_slice(&0u16.to_le_bytes()); // protected tlvs
        out.extend_from_slice(&(body.len() as u32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // flags
        out.push(version.0); // image_version: major
        out.push(version.1); // minor
        out.extend_from_slice(&version.2.to_le_bytes()); // revision
        out.extend_from_slice(&0u32.to_le_bytes()); // build number
        out.extend_from_slice(&0u32.to_le_bytes()); // pad
        out.extend_from_slice(&body);

        out.extend_from_slice(&0x6907u16.to_le_bytes()); // tlv area
        out.extend_from_slice(&((4 + 4 + hash.len()) as u16).to_le_bytes());
        out.extend_from_slice(&0x0010u16.to_le_bytes()); // SHA-256
        out.extend_from_slice(&(hash.len() as u16).to_le_bytes());
        out.extend_from_slice(hash);
        out
    }

    /// A whole .zsab, laid out the way `tools/zsa_bundle.py` writes one.
    fn bundled(left: &[u8], right: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"ZSAB");
        out.extend_from_slice(&1u16.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&0x3Cu32.to_le_bytes());
        out.extend_from_slice(&(left.len() as u32).to_le_bytes());
        out.extend_from_slice(&((0x3C + left.len()) as u32).to_le_bytes());
        out.extend_from_slice(&(right.len() as u32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // key fingerprint
        let mut version = [0u8; 32];
        version[.."0.1.9-wired".len()].copy_from_slice(b"0.1.9-wired");
        out.extend_from_slice(&version);
        out.extend_from_slice(left);
        out.extend_from_slice(right);
        out
    }

    /// Runs the whole orchestration against the in-memory keyboard.
    fn flash_loopback(blob: &[u8]) -> (Result<()>, Vec<(u8, u16)>, LoopbackDevice) {
        flash_loopback_with(blob, false)
    }

    /// The same, with the downgrade and skip guards overridden.
    fn flash_loopback_forced(blob: &[u8]) -> (Result<()>, Vec<(u8, u16)>, LoopbackDevice) {
        flash_loopback_with(blob, true)
    }

    fn flash_loopback_with(
        blob: &[u8],
        force: bool,
    ) -> (Result<()>, Vec<(u8, u16)>, LoopbackDevice) {
        let bundle = bundle::parse(blob).expect("a bundle");
        let (link, device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        let shared = Shared::new(link);
        let mut ignition = Ignition::new(shared.clone());

        let result = run(
            &shared,
            &mut ignition,
            &bundle,
            force,
            Duration::from_millis(50),
        );
        // END the way `flash` does, so the recorded sequence is the whole one.
        if result.is_ok() {
            ignition.end().unwrap();
        }
        let control = shared.0.lock().unwrap().control.clone();
        (result, control, device)
    }

    /// Every `(group, command)` of a given kind the device was sent.
    fn count(device: &LoopbackDevice, request: (u16, u8)) -> usize {
        device
            .requests()
            .iter()
            .filter(|sent| **sent == request)
            .count()
    }

    #[test]
    fn a_half_already_carrying_the_bundles_image_is_not_written_again() {
        // Both images are named by the hash the in-memory board reports, so
        // both halves read as installed and neither upload should happen.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, _control, device) = flash_loopback(&blob);

        result.expect("nothing to write is not a failure");
        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0);
        // And the storage erase belongs to an upload, not to a skipped half.
        assert_eq!(count(&device, (BASIC_GROUP, BASIC_CMD_ERASE_STORAGE)), 0);
        // It still read both halves before deciding.
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 2);
    }

    #[test]
    fn an_image_the_board_does_not_report_after_the_upload_fails_the_run() {
        // The post-upload read is the whole point: the upload itself succeeds
        // here — every byte is acknowledged — and the run still has to fail,
        // because the slot does not come back carrying what was sent.
        let blob = bundled(&signed(0x11, &[0xa1; 32]), &signed(0x22, &[0xb2; 32]));
        let (result, _control, device) = flash_loopback(&blob);

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("took the whole image"), "{err}");
        assert!(err.contains("not the"), "{err}");
        // It got as far as writing: one upload, and the read back after it.
        assert!(count(&device, (img::GROUP, img::CMD_UPLOAD)) > 0, "{err}");
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 2);
    }

    #[test]
    fn the_right_halfs_storage_is_erased_before_its_image_and_the_lefts_is_not() {
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &[0xb2; 32]));
        // The right is written and the left is already installed, so exactly
        // one erase can happen and it has to be the right's.
        let (result, _control, device) = flash_loopback(&blob);
        assert!(result.is_err(), "the loopback cannot report a landed hash");

        let requests = device.requests();
        let erase = requests
            .iter()
            .position(|sent| *sent == (BASIC_GROUP, BASIC_CMD_ERASE_STORAGE))
            .expect("the right half's storage erase");
        let upload = requests
            .iter()
            .position(|sent| *sent == (img::GROUP, img::CMD_UPLOAD))
            .expect("the right half's upload");
        assert!(erase < upload, "{requests:?}");
        assert_eq!(count(&device, (BASIC_GROUP, BASIC_CMD_ERASE_STORAGE)), 1);
    }

    #[test]
    fn the_right_half_is_selected_before_the_left() {
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, _device) = flash_loopback(&blob);
        result.unwrap();

        let selects: Vec<u16> = control
            .iter()
            .filter(|(request, _)| *request == ignition::REQ_SELECT)
            .map(|(_, value)| *value)
            .collect();
        // 2 is right, then 1 twice: the bridge is pointed away from the right
        // as soon as it has been told to reboot, and then at the left again as
        // the left's own turn begins.
        assert_eq!(selects, vec![2, 1, 1]);
    }

    #[test]
    fn the_whole_session_is_one_begin_and_one_end_with_both_resets_inside_it() {
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, _device) = flash_loopback(&blob);
        result.unwrap();

        // Every vendor request, in order, with the STATUS polling dropped: it
        // is a poll and its count is not part of the sequence.
        let sequence: Vec<(u8, u16)> = control
            .into_iter()
            .filter(|(request, _)| *request != ignition::REQ_STATUS)
            .collect();
        assert_eq!(
            sequence,
            vec![
                (ignition::REQ_BEGIN, Halves::BOTH.bits()),
                (ignition::REQ_SELECT, 2),
                // Twice: once to stop the bridge polling the right half while
                // it reboots, then again as the left's own turn begins. The
                // second is redundant and costs one control transfer, which is
                // cheaper than the bus recovery the first one avoids.
                (ignition::REQ_SELECT, 1),
                (ignition::REQ_SELECT, 1),
                (ignition::REQ_RELEASE_MD, 0),
                (ignition::REQ_RESET_TARGET, 1),
                (ignition::REQ_END, 0),
            ]
        );
        // Only the left is reset through ignition. The right has no reset line
        // the STM32 can reach, so it leaves recovery by its own SMP reset —
        // asserted in a_right_half_is_restarted_by_its_own_smp_reset.
    }

    #[test]
    fn a_half_that_was_skipped_is_still_reset_out_of_its_bootloader() {
        // BEGIN puts both halves in MCUboot and END does not take them out
        // again, so a keyboard whose halves both already carry the bundle
        // still has to be restarted or it is left sitting in recovery.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, device) = flash_loopback(&blob);
        result.unwrap();

        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0);
        assert!(
            control.contains(&(ignition::REQ_RESET_TARGET, 1)),
            "the left half was not reset: {control:?}"
        );
        assert_eq!(
            count(&device, (client::OS_GROUP, client::OS_CMD_RESET)),
            1,
            "the right half was not reset out of MCUboot"
        );
    }

    #[test]
    fn a_bundle_older_than_what_is_installed_is_refused_and_nothing_is_written() {
        // The mistake this exists to stop: a bundle built from a directory
        // that was rebuilt afterwards carried an image eleven versions behind
        // the half it was written to, and the resulting keyboard hung the
        // moment its halves were wired together. The board reports 9.9.9.
        let blob = bundled(
            &signed_v(0x11, &[0xAA; 32], (0, 1, 0)),
            &signed_v(0x22, &[0xBB; 32], (0, 1, 0)),
        );
        let (result, _control, device) = flash_loopback(&blob);

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("older"), "{err}");
        assert!(err.contains("--force"), "{err}");
        assert_eq!(
            count(&device, (img::GROUP, img::CMD_UPLOAD)),
            0,
            "a refused downgrade still wrote something"
        );
    }

    #[test]
    fn a_downgrade_goes_through_when_it_is_asked_for() {
        let blob = bundled(
            &signed_v(0x11, &[0xAA; 32], (0, 1, 0)),
            &signed_v(0x22, &[0xBB; 32], (0, 1, 0)),
        );
        let (_result, _control, device) = flash_loopback_forced(&blob);

        // Only that the write was attempted. The run still fails afterwards,
        // because the fake reports a fixed hash whatever it was sent and the
        // read-back check correctly disbelieves it — which is the same reason
        // no test here can watch a successful upload all the way through.
        assert!(
            count(&device, (img::GROUP, img::CMD_UPLOAD)) > 0,
            "--force did not get past the downgrade guard"
        );
    }

    #[test]
    fn a_right_half_is_restarted_by_its_own_smp_reset_not_by_reset_target() {
        // RESET_TARGET on the right asks its *application* to reboot into
        // MCUboot: the wrong direction, and unanswerable while MCUboot is
        // what is running, so a right half restarted that way is left sitting
        // in recovery. It has to be MCUboot's own reset.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, device) = flash_loopback(&blob);
        result.unwrap();

        assert!(
            !control.contains(&(ignition::REQ_RESET_TARGET, 2)),
            "the right half was sent RESET_TARGET: {control:?}"
        );
        assert_eq!(
            count(&device, (client::OS_GROUP, client::OS_CMD_RESET)),
            1,
            "expected exactly one SMP reset, for the right half"
        );
    }

    #[test]
    fn the_left_halfs_md_pin_is_released_before_it_is_reset_and_never_after() {
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, _device) = flash_loopback(&blob);
        result.unwrap();

        let md = control
            .iter()
            .position(|sent| *sent == (ignition::REQ_RELEASE_MD, 0))
            .expect("the MD release");
        let reset = control
            .iter()
            .position(|sent| *sent == (ignition::REQ_RESET_TARGET, 1))
            .expect("the left half's reset");
        assert!(md < reset, "{control:?}");
        // And it is not sent for the right half, which has no MD pin held.
        assert_eq!(
            control
                .iter()
                .filter(|sent| **sent == (ignition::REQ_RELEASE_MD, 0))
                .count(),
            1
        );
    }

    #[test]
    fn the_session_is_polled_rather_than_waited_out() {
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, _device) = flash_loopback(&blob);
        result.unwrap();

        // At least one STATUS, and it comes after BEGIN rather than before it.
        let begin = control
            .iter()
            .position(|(request, _)| *request == ignition::REQ_BEGIN)
            .expect("BEGIN");
        let polled = control
            .iter()
            .position(|(request, _)| *request == ignition::REQ_STATUS)
            .expect("a STATUS poll");
        assert!(begin < polled, "{control:?}");
    }

    // ---- The decisions, on their own -----------------------------------

    fn slot(number: u32, hash: &[u8]) -> ImageSlot {
        ImageSlot {
            slot: number,
            hash: hash.to_vec(),
            version: "1.0.0".into(),
            ..ImageSlot::default()
        }
    }

    #[test]
    fn the_slot_a_recovery_upload_writes_is_the_one_that_is_compared() {
        let wanted = [0xa1; 32];
        assert!(installed(&[slot(0, &wanted)], &wanted));
        // The same image sitting in the spare slot is not installed: recovery
        // writes the primary, so slot 1 is not what an upload would change.
        assert!(!installed(&[slot(1, &wanted)], &wanted));
        assert!(!installed(&[slot(0, &[0x99; 32])], &wanted));
        assert!(!installed(&[], &wanted));
    }

    #[test]
    fn a_board_that_leaves_the_slot_number_out_is_describing_slot_zero() {
        // Which is what `img::parse_state` already reads it as, and what a
        // single-slot recovery answers with.
        let wanted = [0xa1; 32];
        let reported = ImageSlot {
            hash: wanted.to_vec(),
            ..ImageSlot::default()
        };
        assert!(installed(&[reported], &wanted));
    }

    #[test]
    fn an_image_that_landed_is_accepted_and_anything_else_is_not() {
        let wanted = [0xa1; 32];
        assert!(landed(&[slot(0, &wanted)], &wanted, Half::Left).is_ok());

        let wrong = landed(&[slot(0, &[0x99; 32])], &wanted, Half::Right)
            .unwrap_err()
            .to_string();
        assert!(wrong.contains("right"), "{wrong}");
        assert!(wrong.contains("did not land"), "{wrong}");

        // A slot MCUboot could not verify reports no hash at all, which is a
        // different thing to say than "it carries the wrong image".
        let empty = landed(&[slot(0, &[])], &wanted, Half::Left)
            .unwrap_err()
            .to_string();
        assert!(empty.contains("not bootable"), "{empty}");

        let missing = landed(&[], &wanted, Half::Left).unwrap_err().to_string();
        assert!(missing.contains("no primary slot"), "{missing}");
    }

    // ---- What a failure says -------------------------------------------

    fn faulted(error: u16, recoveries: u8, drops: u8) -> Status {
        let mut bytes = vec![0u8; ignition::STATUS_LEN];
        bytes[0] = ignition::STATUS_VERSION;
        bytes[1] = 2; // fault
        bytes[5..7].copy_from_slice(&error.to_le_bytes());
        bytes[7] = 50;
        bytes[8] = recoveries;
        bytes[9] = drops;
        bytes[10..12].copy_from_slice(&1024u16.to_le_bytes());
        bytes[12] = 1;
        Status::parse(&bytes).unwrap()
    }

    #[test]
    fn an_i2c_fault_reads_like_a_cable_and_not_like_a_timeout() {
        let note = session_error(&faulted(0x0005, 3, 0)).unwrap();
        assert!(note.contains("stopped answering on the TRRS bus"), "{note}");
        assert!(note.contains("3 bus recoveries at 50 kHz"), "{note}");
        assert!(note.contains("check the cable"), "{note}");
        assert!(!note.contains("timeout"), "{note}");
    }

    #[test]
    fn every_error_the_bootloader_can_record_has_its_own_sentence() {
        let notes: Vec<String> = (1..=9)
            .chain(std::iter::once(0xFF))
            .map(|code| session_error(&faulted(code, 0, 0)).expect("a sentence"))
            .collect();
        for note in &notes {
            assert!(note.len() > 40, "{note}");
            assert!(!note.contains("0x00"), "a raw code in prose: {note}");
        }
        // And they are ten different sentences, not one repeated.
        let mut unique = notes.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(unique.len(), notes.len());
    }

    #[test]
    fn a_session_with_nothing_wrong_says_nothing() {
        assert_eq!(session_error(&faulted(0, 0, 0)), None);
        // And a code from a newer firmware is named by its number rather than
        // dressed up as an error this build understands.
        let newer = session_error(&faulted(0x0042, 0, 0)).unwrap();
        assert!(newer.contains("0x0042"), "{newer}");
        assert!(newer.contains("does not have a sentence"), "{newer}");
    }

    #[test]
    fn a_bus_that_coped_but_had_to_work_at_it_is_still_worth_a_line() {
        assert_eq!(bus_health(&faulted(0, 0, 0)), None);
        let note = bus_health(&faulted(0, 2, 9)).unwrap();
        assert!(note.contains("2 bus recoveries at 50 kHz"), "{note}");
        assert!(note.contains("9 bytes dropped"), "{note}");
    }

    #[test]
    fn one_recovery_is_not_pluralised() {
        assert!(bus_phrase(&faulted(0, 1, 0)).contains("1 bus recovery at 50 kHz"));
        assert!(bus_phrase(&faulted(0, 0, 0)).contains("0 bus recoveries"));
    }

    #[test]
    fn a_bridge_event_that_failed_a_transfer_is_explained_in_its_own_terms() {
        let failed = |code: pipe::EventCode| {
            anyhow::Error::from(SmpError::Transport(zapp_smp::TransportError::Io(
                WiredError::Event(pipe::Event { code, detail: 0x45 }).to_string(),
            )))
            .context("The right half's upload failed")
        };

        let gone = event_advice(&failed(pipe::EventCode::MailboxGone)).unwrap();
        assert!(gone.contains("stopped answering part-way"), "{gone}");

        let bus = event_advice(&failed(pipe::EventCode::I2cTimeout)).unwrap();
        assert!(bus.contains("Reseat the cable"), "{bus}");

        let overflow = event_advice(&failed(pipe::EventCode::RxOverflow)).unwrap();
        assert!(overflow.contains("receive buffer overflowed"), "{overflow}");

        // A code from a newer firmware prints its own number and gets nothing
        // invented on top of it, and neither does an ordinary failure.
        assert!(event_advice(&failed(pipe::EventCode::Unknown(0x7f))).is_none());
        assert!(event_advice(&anyhow::anyhow!("the upload timed out")).is_none());
    }

    // ---- The diagnostic scan -------------------------------------------

    #[test]
    fn the_scan_names_every_address_and_what_it_answered() {
        // ACK, NACK, bus error, timeout, ACK — one of each ignition can report.
        let scan = [0x00, 0xFF, 0xFE, 0xFD, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let lines = scan_lines(&scan);
        assert_eq!(lines.len(), 5);
        assert!(lines[0].starts_with("0x20"), "{}", lines[0]);
        assert!(lines[0].ends_with("answered"), "{}", lines[0]);
        assert!(lines[1].contains("MCUboot mailbox"), "{}", lines[1]);
        assert!(lines[1].ends_with("no answer (NACK)"), "{}", lines[1]);
        assert!(lines[2].ends_with("bus error"), "{}", lines[2]);
        assert!(lines[3].ends_with("timed out"), "{}", lines[3]);
    }

    #[test]
    fn a_scan_that_came_back_short_is_not_read_past_its_end() {
        let lines = scan_lines(&[0x00]);
        assert_eq!(lines.len(), 5);
        assert!(lines[0].ends_with("answered"), "{}", lines[0]);
        assert!(lines[4].ends_with("no answer (NACK)"), "{}", lines[4]);
    }
}
