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
//! Every wait polls: ignition's STATUS, an SMP reply, or the slot listing that
//! says a half has finished installing — see [`SETTLE_TIMEOUT`].
//!
//! The write goes into the spare slot, and here it is marked to be kept rather
//! than to be tried — which is where this path parts company with the Bluetooth
//! one, and it is not a preference.
//!
//! Each half's firmware keeps a test image only once it has proved its link to
//! the rest of the keyboard: the left when its UART link to the STM32 running
//! QMK comes up, the right when it first reaches the left over the radio or
//! answers a telemetry read QMK asked for over the wire. None of that can happen
//! during a wired session, because the STM32 is running ignition and not QMK
//! from the first BEGIN to the reset at the end. A test image therefore boots
//! unconfirmed, and the next thing that happens to it is this file re-entering
//! the bootloaders to read it back — a reboot, on which MCUboot reverts it. The
//! read-back then finds the old image, forever. That is not a check failing; it
//! is the check destroying what it came to look at.
//!
//! So over the cable the mark is permanent. `bs_set` in MCUboot's serial
//! recovery passes the `confirm` flag straight to `boot_set_pending_multi`,
//! which writes `image_ok` and `BOOT_SWAP_TYPE_PERM` into the spare slot's
//! trailer, and the loader sets `image_ok` on the primary after a `PERM` swap
//! "to avoid a new revert" — so the swap happens once and sticks. What is given
//! up is MCUboot's automatic way back, and what stands in for it is the cable
//! itself: reaching this path at all means physical access to a keyboard whose
//! bootloader can always be entered again. What is not given up is the check —
//! the read-back still compares hashes MCUboot only publishes for images it has
//! verified, and [`came_back`] still makes both halves prove their links before
//! the run is called a success.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use indicatif::{ProgressBar, ProgressStyle};

use zapp_core::device::{self, Keyboard, UsbDevice};
use zapp_core::flash::FlashProgress;
use zapp_core::hid::{BatteryReading, Telemetry};
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

/// How long to wait for each reply from a half.
const REPLY_TIMEOUT: Duration = Duration::from_secs(15);

/// How long to keep asking for a session while the right half is missing.
///
/// One attempt is one five-second window, because that is where ignition's own
/// entry sequence gives up. The way a right half that did not show up is
/// rescued is power-cycling it — unplug the TRRS cable and plug it back in —
/// while the bootloader is polling for it, so what matters is that a poll is
/// running when it comes back. On the bench that was the third or fourth
/// attempt, never the first.
const WAIT_FOR_RIGHT: Duration = Duration::from_secs(30);

/// A pause between attempts, so a bootloader that refuses one straight away is
/// not asked several thousand times.
const BETWEEN_ATTEMPTS: Duration = Duration::from_millis(250);

/// How long both halves get to swap and come back up before the read-back gives
/// up on them.
///
/// A ceiling rather than a wait: the read-back re-reads both halves every
/// [`SETTLE_POLL`] and stops the moment they carry the bundle. Swap-using-move
/// carries a 0xb2000 slot through RRAM sector by sector, which is minutes of
/// the seconds-per-sector that made a whole-slot erase a "tens-of-seconds"
/// stall, so the ceiling is deliberately long.
const SETTLE_TIMEOUT: Duration = Duration::from_secs(120);

/// How long the halves are left alone between read-backs.
///
/// Every read-back costs them the install they are running: ignition's entry
/// sequence holds the left half's MD pin and blips its reset, and hard-resets
/// the right (`ignition/src/session.c:107,139`), and neither bootloader swaps
/// while it is in recovery. MCUboot picks an interrupted swap up where it
/// stopped, so this is the uninterrupted time it gets to make progress in.
const SETTLE_POLL: Duration = Duration::from_secs(5);

/// The image number a recovery upload names the spare slot by.
///
/// `CONFIG_MCUBOOT_SERIAL_DIRECT_IMAGE_UPLOAD` maps 0 and 1 to
/// `slot0_partition` and 2 to `slot1_partition`
/// (`bootloader/mcuboot/boot/zephyr/flash_map_extended.c:129`).
const SPARE_IMAGE: u32 = 2;

/// The zephyr basic group, and the command that erases the settings partition.
const BASIC_GROUP: u16 = 63;
const BASIC_CMD_ERASE_STORAGE: u8 = 0;

/// `MGMT_ERR_ENOTSUP`: a build that does not carry the command at all.
const MGMT_ERR_ENOTSUP: i64 = 8;

// ---------------------------------------------------------------------------
// The command
// ---------------------------------------------------------------------------

/// Writes both halves from one bundle.
pub fn flash(path: &Path, force: bool) -> Result<()> {
    let blob = std::fs::read(path).with_context(|| format!("Failed to read {}", path.display()))?;
    let bundle = bundle::parse(&blob)
        .with_context(|| format!("{} is not a bundle this build can read", path.display()))?;
    announce(&bundle, path, blob.len());

    let shared = Shared::new(open()?);
    let mut ignition = Ignition::new(shared.clone());

    match run(
        &shared,
        &mut ignition,
        &bundle,
        force,
        REPLY_TIMEOUT,
        SETTLE_TIMEOUT,
    ) {
        Ok(outcome) => {
            // Nothing was written and nothing was checked, so this one line is
            // the whole of the output. The session is still put back the same
            // way, because BEGIN left both halves in their bootloaders.
            if outcome == Outcome::UpToDate {
                println!("The keyboard is up to date, skipping the flash.");
            }

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

            // The last check, and not an optional one: the halves keep their
            // new images only once they have proved their links, and this is
            // the only point in the run where they can.
            if let Err(err) = finish(outcome, || came_back(CONFIRM_WINDOW)) {
                log::debug!("failure detail: {err:#}");
                println!();
                println!("{}", wrap(&format!("Error: {err}.")));
                std::process::exit(1);
            }

            if outcome == Outcome::Flashed {
                println!("Done!");
            }
            Ok(())
        }
        Err(err) => {
            // Asked before anything is put back and put back before anything
            // is printed, in that order and for two different reasons: the
            // notes are read out of a session that is still open, and the next
            // attempt needs one that is not — the bootloader refuses BEGIN
            // outright while a session is active.
            let notes = advice(&mut ignition);
            if let Err(err) = ignition.end() {
                log::debug!("could not end the session after the failure: {err}");
            }

            // One line, plus whatever the thing that actually failed said.
            // The middle of the chain names bridges, mailboxes and event
            // codes, which is what this looks like from the inside and no help
            // to someone holding a keyboard — that stays in the debug log. The
            // far end of it is different: it is the board's own words, and
            // leaving them out once cost a whole bench session spent guessing
            // at a failure the board had already named.
            log::debug!("failure detail: {err:#}");
            let mut line = match root_cause(&err) {
                Some(cause) => format!("Error: {err} ({cause})."),
                None => format!("Error: {err}."),
            };
            for note in notes {
                line.push(' ');
                line.push_str(&note);
            }
            println!();
            println!("{}", wrap(&line));
            std::process::exit(1);
        }
    }
}

// ---------------------------------------------------------------------------
// The flash itself
// ---------------------------------------------------------------------------

/// What a run did to the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Outcome {
    /// At least one half was written.
    Flashed,
    /// Both halves already carried the bundle, so nothing was.
    UpToDate,
}

fn run<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    bundle: &Bundle,
    force: bool,
    timeout: Duration,
    settle: Duration,
) -> Result<Outcome> {
    let mut steps = Steps { total: 5, done: 0 };

    steps.announce("Opening a session on both halves");
    let status = open_session(ignition, WAIT_FOR_RIGHT)?;
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
    let mut carried = Vec::new();
    let mut wrote = false;
    for half in [Half::Right, Half::Left] {
        steps.announce(&format!("{} half", titled(half)));
        let (version, staged) = write_half(shared, ignition, half, bundle, &status, force, timeout)?;
        wrote |= staged == Staged::Image;
        carried.push(Carried {
            half,
            version,
            hash: None,
        });
    }

    // Both halves were already carrying the bundle, so nothing was uploaded and
    // nothing was staged. There is no swap to sit through and no read-back to
    // make: what the halves are running is what was checked before the decision
    // to skip them. Only `--force` writes a half that already matches, so this
    // is unreachable with it.
    if !wrote {
        log::debug!("neither half needed writing");
        return Ok(Outcome::UpToDate);
    }

    // Before `flash` resets the STM32: once it is back in QMK there is no
    // reliable way into ignition again, so this is the last chance to look.
    steps.announce("Reading both halves back after they restarted");
    verify(
        shared,
        ignition,
        bundle,
        &mut carried,
        timeout,
        settle,
        WAIT_FOR_RIGHT,
    )?;

    steps.announce("Checking both halves came out on the bundle's version");
    agreed(bundle, &carried)?;
    Ok(Outcome::Flashed)
}

// ---------------------------------------------------------------------------
// Reading the halves back after they restarted
// ---------------------------------------------------------------------------

/// Re-reads what each half's slot 0 carries until both agree with the bundle.
///
/// The read-back inside `write_half` happens with the half still sitting in the
/// recovery that wrote it, and the image still in the spare slot. This one runs
/// after MCUboot swapped and the half restarted, so slot 0 carrying the
/// bundle's hash is the swap having completed on an image MCUboot verified.
///
/// Both answers come from the halves' own bootloaders. Nothing here asks an
/// application anything, which is what makes it the check the run passes or
/// fails on — and also what it cannot see: whether either half's firmware runs.
/// That half of the story is [`came_back`]'s.
///
/// Each pass costs the halves a reboot, which is the reason the images are
/// marked to be kept rather than tried: a test image would be reverted by the
/// first of these passes and every pass after it would read the old one.
///
/// A half that is still swapping answers nothing, or answers with the image it
/// had, so a pass that fails is retried until `settle` runs out and the last
/// failure is what the run ends on.
///
/// Records each half's hash and version on `carried`, for the whole-keyboard
/// check.
fn verify<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    bundle: &Bundle,
    carried: &mut [Carried],
    timeout: Duration,
    settle: Duration,
    limit: Duration,
) -> Result<()> {
    let started = Instant::now();
    let deadline = started + settle;
    // Named with its length: this is the longest wait in the run and an
    // unqualified spinner reads as a hang.
    let spinner = (!settle.is_zero()).then(|| new_spinner(&installing(Duration::ZERO, settle)));

    loop {
        let failure = match read_back(shared, ignition, bundle, carried, timeout, limit) {
            Ok(()) => {
                if let Some(spinner) = &spinner {
                    spinner.finish_and_clear();
                }
                return Ok(());
            }
            Err(failure) => failure,
        };

        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            if let Some(spinner) = &spinner {
                spinner.finish_and_clear();
            }
            return Err(failure);
        }

        log::debug!(
            "not installed yet with {} s left: {failure:#}",
            left.as_secs()
        );
        boot_halves(ignition);
        std::thread::sleep(SETTLE_POLL.min(left));
        if let Some(spinner) = &spinner {
            spinner.set_message(installing(started.elapsed(), settle));
        }
    }
}

/// The spinner's line while the halves are installing.
fn installing(gone: Duration, settle: Duration) -> String {
    format!(
        "Installing the update on both halves ({} s of up to {} s)",
        gone.as_secs(),
        settle.as_secs()
    )
}

/// Lets both halves get on with the install a read-back interrupted.
///
/// Ignition's entry sequence holds the left half's MD pin low and blips its
/// reset, so the left comes up in recovery, where nothing swaps, and stays
/// there until the pin is let go and it is reset again. The right has no MD
/// pin: the same sequence hard-resets it, so it is already on its way.
///
/// Best effort. Reaching here means a pass already failed, and a request that
/// fails on top of that says nothing the next pass will not say better.
fn boot_halves<L: IgnitionLink>(ignition: &mut Ignition<L>) {
    if let Err(err) = ignition.release_md() {
        log::debug!("could not release the left half's MD pin between read-backs: {err}");
    }
    if let Err(err) = ignition.reset_target(Half::Left) {
        log::debug!("could not restart the left half between read-backs: {err}");
    }
}

/// One pass: a session, a slot listing from each half, and the reset that puts
/// each of them back on its own firmware.
fn read_back<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    bundle: &Bundle,
    carried: &mut [Carried],
    timeout: Duration,
    limit: Duration,
) -> Result<()> {
    let status = match open_session(ignition, limit) {
        Ok(status) => status,
        Err(err) => {
            log::debug!("could not re-open a session to read the halves back: {err:#}");
            // Staging is not installing. Each half still has a swap to get
            // through on its next restart, and a swap can be interrupted, so a
            // check that was not made cannot be reported as an update that
            // took.
            if right_absent(ignition) {
                return Err(err).context(unverified());
            }
            return Err(err);
        }
    };

    for entry in carried.iter_mut() {
        let half = entry.half;
        ignition
            .select(half)
            .with_context(|| format!("Could not point the bridge at the {half} half"))?;
        let mut client = connect(shared, half, status.payload_budget(), timeout);

        let wanted = bundle
            .hash(half)
            .with_context(|| format!("The bundle's {half} image carries no hash to check it by"))?;
        let slots = client
            .image_state()
            .with_context(|| format!("Could not read the {half} half's image state back"))?;
        report_slot(&slots);
        landed(&slots, &wanted, half, Checked::Restarted)?;
        if let Some(slot) = primary(&slots) {
            entry.hash = Some(slot.hash.clone());
            entry.version = slot.version.clone();
        }
        log::debug!("{half}: restarted carrying {}", short(&wanted));

        // Out of the bootloaders again, the same way `write_half` does it.
        match half {
            Half::Left => {
                ignition
                    .release_md()
                    .context("Could not release the left half's MD pin")?;
                ignition
                    .reset_target(half)
                    .context("Could not restart the left half")?;
            }
            Half::Right => {
                client
                    .reset()
                    .context("Could not restart the right half out of MCUboot")?;
                ignition
                    .select(Half::Left)
                    .context("Could not point the bridge away from the right half")?;
            }
        }
    }

    log::debug!("both halves read back carrying the bundle's images");
    Ok(())
}

/// What a read-back nobody could make says. Both images are staged and armed to
/// be kept, which is not the same as installed: the swap happens on each half's
/// next restart and can fail on the way, and nothing here saw either of them
/// come out of it.
fn unverified() -> String {
    "both images were staged but could not be checked afterwards, because the right half \
     did not answer when the bootloaders were re-entered. Each half installs its image on \
     the next restart and keeps it, but nothing here saw whether that finished, so this \
     update has not been checked. Run the same command again"
        .to_string()
}

/// Whether the bootloader's last fault was the right half not answering.
fn right_absent<L: IgnitionLink>(ignition: &mut Ignition<L>) -> bool {
    match ignition.status() {
        Ok(status) => right_missing(status.error),
        Err(err) => {
            log::debug!("could not read ignition's status after the failed re-open: {err}");
            false
        }
    }
}

// ---------------------------------------------------------------------------
// Getting a session open
// ---------------------------------------------------------------------------

/// Opens a session, giving a right half that is not there time to appear.
///
/// The one attempt this used to make gave the operator about five seconds, and
/// the rescue for a right half that did not answer is to power-cycle it while
/// the bootloader is polling — which nobody can time to a window that has
/// already closed. Asking again for half a minute is the same procedure with
/// enough room in it to work.
fn open_session<L: IgnitionLink>(ignition: &mut Ignition<L>, limit: Duration) -> Result<Status> {
    keep_asking(limit, || {
        // No END between attempts: `begin` already closes a session it finds
        // in the way, and ending one here would throw away the fault the last
        // attempt recorded — which is what the failure is explained from.
        if let Err(err) = ignition.begin(Halves::BOTH) {
            return Err(NotOpened::fatal(
                anyhow::Error::new(err).context("Could not open a session on the bootloader"),
            ));
        }
        wait_active(ignition)
    })
}

/// A session that would not open, and whether waiting could change that.
struct NotOpened {
    error: anyhow::Error,
    /// Set for the faults a right half that is not answering produces. Those
    /// are the only ones worth asking again for: the right half is the one
    /// thing in this path someone can power-cycle while the run waits.
    right_missing: bool,
}

impl NotOpened {
    fn fatal(error: anyhow::Error) -> Self {
        Self {
            error,
            right_missing: false,
        }
    }
}

/// The faults a right half that is not answering produces.
///
/// Codes 0x0002 to 0x0005: no mailbox, no cable, no matrix expander, and a
/// bus that stopped answering. The same situation from four angles — nothing
/// on the TRRS bus, or nothing at the far end of it — and one answer to all
/// of them: unplug the cable, plug it back in, and let the next attempt find
/// it.
fn right_missing(error: u16) -> bool {
    matches!(error, 0x0002..=0x0005)
}

/// The retry itself, with the board's side of it passed in.
///
/// Separated from the link so it can be driven without one: what is worth
/// testing here is when it gives up and when it stops, not how a session is
/// opened.
fn keep_asking<F>(limit: Duration, mut attempt: F) -> Result<Status>
where
    F: FnMut() -> std::result::Result<Status, NotOpened>,
{
    let started = Instant::now();
    let mut spinner: Option<ProgressBar> = None;
    loop {
        let outcome = attempt();
        let done = match &outcome {
            Ok(_) => true,
            Err(not_opened) => !not_opened.right_missing || started.elapsed() >= limit,
        };
        if done {
            if let Some(spinner) = spinner {
                spinner.finish_and_clear();
            }
            return outcome.map_err(|not_opened| not_opened.error);
        }
        // One line for the whole wait, however many attempts it takes.
        if spinner.is_none() {
            spinner = Some(new_spinner(
                "Waiting for the right half — unplug the cable between the halves \
                 and plug it back in",
            ));
        }
        std::thread::sleep(BETWEEN_ATTEMPTS);
    }
}

/// One half: what it is carrying, what it needs, and the reset that boots it.
///
/// Answers with the version of the slot carrying the bundle's image once it is
/// done, written or not, which is what the whole-keyboard check at the end
/// compares. That slot is the spare one when the half was written, because
/// MCUboot has not swapped yet.
fn write_half<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    half: Half,
    bundle: &Bundle,
    status: &Status,
    force: bool,
    timeout: Duration,
) -> Result<(String, Staged)> {
    ignition
        .select(half)
        .with_context(|| format!("Could not point the bridge at the {half} half"))?;

    // Close whatever line the half was left half-way through. Everything on
    // this side of the bridge is flushed by BEGIN and SELECT, but the buffer
    // that matters here is MCUboot's own: a transfer interrupted part-way
    // through a line — Ctrl-C during an upload — leaves it holding a fragment
    // and waiting for the newline that ends it. The next request lands on the
    // back of that fragment and never parses. A bare newline finishes it, and
    // MCUboot discards the nonsense it completes; a half that was not
    // mid-line sees an empty line and ignores it just the same.
    if let Err(err) = ignition.send(half.into(), b"\n") {
        log::debug!("could not send the {half} half a resync newline: {err}");
    }

    let mut client = connect(shared, half, status.payload_budget(), timeout);
    let wanted = bundle
        .hash(half)
        .with_context(|| format!("The bundle's {half} image carries no hash to check it by"))?;

    let before = client
        .image_state()
        .with_context(|| format!("Could not read the {half} half's image state"))?;
    report_slot(&before);

    let mut staged = Staged::Nothing;
    let carried = if installed(&before, &wanted) && !force {
        log::debug!("{half}: already carrying this image, not written again");
        if primary(&before).is_some_and(|slot| !slot.confirmed) {
            log::debug!("{half}: it has not confirmed that image, so it has yet to link up once");
        }
        before
    } else {
        staged = Staged::Image;
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

        // Read back rather than trust the upload's own success. The bytes went
        // into the spare slot, so that is where they have to show up. Checked
        // before the mark, so a bad upload is reported as the hash mismatch it
        // is rather than as a mark that could not find its image.
        let after = client
            .image_state()
            .with_context(|| format!("Could not read the {half} half's image state back"))?;
        landed(&after, &wanted, half, Checked::Written)?;

        // Permanent, not for test. A test image has to be confirmed by the
        // half's own firmware, which cannot happen while the STM32 is running
        // ignition — and the read-back's own session re-entry is a reboot, so
        // MCUboot would revert the image this run just wrote before anything
        // could look at it. Marking it kept is what makes the read-back
        // possible; see the note at the top of this file.
        arm(&mut client, half, &wanted)?;
        log::debug!(
            "{half}: {} bytes written, spare slot now carries {}",
            image.len(),
            short(&wanted)
        );
        after
    };

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
    Ok((version_carrying(&carried, &wanted), staged))
}

/// Marks the spare slot to be kept, and puts up with a reply it cannot read.
///
/// The reply to a mark is a whole slot listing, which MCUboot encodes into one
/// fixed buffer — and arming the swap is what makes that listing too long for
/// it, because the `pending` and `permanent` flags it adds are the last straw
/// on a build with SHA-512 hashes and two slots that both hold an image. So the
/// most ordinary mark on this path answers with a listing that stops mid-way.
///
/// Reading that as a failed mark is wrong, and provably so: MCUboot's `bs_set`
/// answers a refusal with `{"rc": n}` — ten bytes, which always fit and always
/// decode — and only reaches the listing once the trailer is written. A reply
/// that will not decode is therefore a mark that went through. The listing is
/// re-read on the next pass anyway, so nothing is lost by dropping this one.
fn arm(client: &mut SmpClient, half: Half, wanted: &[u8]) -> Result<()> {
    match client.mark(wanted, true) {
        Ok(_) => Ok(()),
        Err(SmpError::Cbor(detail)) => {
            log::debug!(
                "{half}: the mark went through and its listing did not fit the board's \
                 buffer ({detail})"
            );
            Ok(())
        }
        Err(err) => Err(err)
            .with_context(|| format!("Could not mark the {half} half's new image to be kept")),
    }
}

/// Whether a half has an image waiting in its spare slot to swap in.
///
/// Only a half that was written has a swap to sit through, and the swap is the
/// whole reason the read-back waits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Staged {
    Image,
    Nothing,
}

/// Polls STATUS until the entry sequence is done.
///
/// The whole reason `BEGIN` answers immediately: ignition's control callback
/// returns and its main loop runs the multi-second right-entry sequence, so the
/// host watches the state move rather than guessing at how long to wait. Each
/// poll is a USB control transfer, which is its own pacing.
fn wait_active<L: IgnitionLink>(
    ignition: &mut Ignition<L>,
) -> std::result::Result<Status, NotOpened> {
    let started = Instant::now();
    loop {
        let status = match ignition.status() {
            Ok(status) => status,
            Err(err) => {
                return Err(NotOpened::fatal(
                    anyhow::Error::new(err).context("Could not read ignition's status"),
                ));
            }
        };
        match status.state {
            SessionState::Active => return Ok(status),
            SessionState::Fault => {
                let reason = session_error(&status)
                    .unwrap_or_else(|| "the bootloader gave no reason".to_string());
                return Err(NotOpened {
                    right_missing: right_missing(status.error),
                    error: anyhow!("The bootloader could not open a session: {reason}"),
                });
            }
            SessionState::Idle => {
                return Err(NotOpened::fatal(anyhow!(
                    "The bootloader says no session is open, having just been asked to open \
                     one. It is running a build that does not carry the wired update path."
                )));
            }
            SessionState::Pending => {}
            SessionState::Unrecognised(code) => {
                return Err(NotOpened::fatal(anyhow!(
                    "The bootloader reports session state {code}, which this build does not \
                     know. It and this zapp are not from the same release."
                )));
            }
        }
        if started.elapsed() >= BEGIN_TIMEOUT {
            return Err(NotOpened::fatal(anyhow!(
                "The bootloader was still opening the session {} s after it was asked to. \
                 Its own entry sequence gives up well inside that, so it is stuck rather \
                 than slow.",
                BEGIN_TIMEOUT.as_secs()
            )));
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

/// Sends one half's image into its spare slot, with a bar on it.
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
    let result = client.upload(SPARE_IMAGE, image, &|progress| match progress {
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

/// The slot the half is running: what it carries now, and what a swap puts
/// there. A device that leaves the number out is describing its one slot,
/// which `img::parse_state` already reads as zero.
fn primary(slots: &[ImageSlot]) -> Option<&ImageSlot> {
    slots.iter().find(|slot| slot.slot == 0)
}

/// The spare slot: where an upload lands, until MCUboot swaps it in.
fn secondary(slots: &[ImageSlot]) -> Option<&ImageSlot> {
    slots.iter().find(|slot| slot.slot == 1)
}

/// `major.minor.patch` out of a version, ignoring any `-suffix` or `+build`.
///
/// A board reports what MCUboot formatted out of the image header; a bundle
/// reports whatever it was packed with, which is where the `-suffix` comes
/// from. Anything unparseable returns None, which is read as "cannot tell" and
/// lets the write through: refusing on a version string nobody can read would
/// be a worse failure than the one this guards against.
fn ordering_of(version: &str) -> Option<(u32, u32, u32)> {
    let mut parts = version.split(['-', '+']).next()?.split('.');
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

/// Whether the half is already running the image the bundle carries.
///
/// The hash on its own, deliberately, and the confirm is not part of it: a
/// confirm says the half's links have proved out since it booted, which is a
/// fact about the keyboard being alive rather than about what is in flash.
/// Reading it as "not installed" would send the same image down again — and
/// MCUboot resolves a mark's hash to the slot the half is running and refuses,
/// so the re-write could not be armed anyway.
fn installed(slots: &[ImageSlot], wanted: &[u8]) -> bool {
    primary(slots).is_some_and(|slot| slot.hash == wanted)
}

/// When a slot is being compared against the bundle, and which slot that is.
#[derive(Clone, Copy)]
enum Checked {
    /// Straight after the upload, with the half still in recovery and the
    /// image sitting in the spare slot.
    Written,
    /// On a fresh session, after MCUboot swapped and the half restarted.
    Restarted,
}

impl Checked {
    /// The slot this looks at.
    fn slot(self, slots: &[ImageSlot]) -> Option<&ImageSlot> {
        match self {
            Checked::Written => secondary(slots),
            Checked::Restarted => primary(slots),
        }
    }

    /// What that slot is called, in a sentence.
    fn named_slot(self) -> &'static str {
        match self {
            Checked::Written => "spare slot",
            Checked::Restarted => "primary slot",
        }
    }

    /// The lead-in a failure is worded from.
    fn lead(self, half: Half) -> String {
        match self {
            Checked::Written => format!("The {half} half took the whole image and then reported"),
            Checked::Restarted => format!("After restarting, the {half} half reports"),
        }
    }

    /// What a wrong hash means at this point.
    fn wrong(self) -> &'static str {
        match self {
            Checked::Written => "The upload did not land where it was meant to.",
            Checked::Restarted => {
                "What is in flash is not what the bundle carries: the swap did not complete."
            }
        }
    }
}

/// Fails unless the slot the board reports is the image the bundle carries.
///
/// The hash is the whole of it, at both points. A hash only appears in
/// MCUboot's listing for an image it has read and verified, so the bundle's
/// hash in the primary slot is the swap having physically completed on a signed
/// image — which is what this run can check and all it needs to.
///
/// Not the confirm. A permanent swap sets `image_ok` on the primary as it
/// finishes, so a half this run wrote does report itself confirmed — but that
/// is a consequence of the mark rather than evidence of anything, and reading
/// it would only mean failing halves this run did not write: one skipped while
/// mid-test-boot from an older zapp, or any half whose firmware confirms on a
/// link it cannot have while the STM32 is running ignition. Whether the halves
/// work is [`came_back`]'s to answer, after the session.
fn landed(slots: &[ImageSlot], wanted: &[u8], half: Half, checked: Checked) -> Result<()> {
    let lead = checked.lead(half);
    let named_slot = checked.named_slot();
    let Some(slot) = checked.slot(slots) else {
        bail!(
            "{lead} no {named_slot} at all, so there is nothing to check it against. Nothing \
             on that half can be trusted until it is read again."
        );
    };
    if slot.hash.is_empty() {
        bail!(
            "{lead} an empty hash for its {named_slot}, which is what a slot MCUboot could \
             not verify looks like. The image on it is not bootable.",
        );
    }
    if slot.hash != wanted {
        bail!(
            "{lead} {} in its {named_slot}, not the {} that was sent. {}",
            short(&slot.hash),
            short(wanted),
            checked.wrong()
        );
    }
    Ok(())
}

/// The version of the slot carrying `wanted`.
fn version_carrying(slots: &[ImageSlot], wanted: &[u8]) -> String {
    slots
        .iter()
        .find(|slot| slot.hash == wanted)
        .map(|slot| slot.version.clone())
        .unwrap_or_default()
}

/// The version the bundle names.
///
/// Its header string is what the release calls itself and is what a person
/// recognises, but it is free text the packer supplies; when it is not a
/// version this can read, the images' own MCUboot headers answer instead.
fn wanted_version(bundle: &Bundle) -> String {
    if ordering_of(&bundle.header.build_version).is_some() {
        return bundle.header.build_version.clone();
    }
    image::inspect(bundle.image(Half::Left))
        .map(|info| info.version)
        .unwrap_or_default()
}

/// Both halves are up; they are just not carrying the same thing.
///
/// Its own type so the failure epilogue can tell it apart from every other
/// way this can end, all of which leave a half sitting in a bootloader.
#[derive(Debug)]
struct Mismatched(String);

impl std::fmt::Display for Mismatched {
    fn fmt(&self, out: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        out.write_str(&self.0)
    }
}

impl std::error::Error for Mismatched {}

/// What one half came out of the run carrying.
struct Carried {
    half: Half,
    /// What its primary slot reported while it was being written.
    version: String,
    /// Its primary slot's hash, re-read after the restart. `None` when the
    /// read-back could not be made.
    hash: Option<Vec<u8>>,
}

/// Fails unless both halves and the bundle name one build.
///
/// Each half's read-back proves that half took the image it was sent, and
/// nothing so far has put the two answers next to each other. A run that
/// wrote one half and left the other on what it had is a keyboard on two
/// builds, whose halves cannot talk over the wire between them, and it is not
/// a success however well each half went on its own.
///
/// Both the version strings and the hashes read back after the restart, so a
/// pair that disagrees is caught by identity rather than only by what it calls
/// itself.
fn agreed(bundle: &Bundle, carried: &[Carried]) -> Result<()> {
    let wanted = wanted_version(bundle);
    let reading = ordering_of(&wanted);

    let mut found: Vec<String> = Vec::new();
    for entry in carried {
        let half = entry.half;
        let wrong_hash = match (&entry.hash, bundle.hash(half)) {
            (Some(hash), Some(bundled)) => *hash != bundled,
            _ => false,
        };
        if wrong_hash {
            found.push(format!(
                "the {half} half is carrying {}",
                short(entry.hash.as_deref().unwrap_or_default())
            ));
        } else if reading.is_none() || ordering_of(&entry.version) != reading {
            found.push(format!("the {half} half is on {}", named(&entry.version)));
        }
    }
    if found.is_empty() {
        log::debug!("both halves are on {}", named(&wanted));
        return Ok(());
    }

    found.push(format!("the bundle is {}", named(&wanted)));
    Err(Mismatched(format!(
        "The keyboard did not come out of this on one build: {}. Both halves have to be on \
         the same build or they cannot talk over the wire between them. Run the same command \
         again; if it says the same thing, the bundle is not the one you meant.",
        found.join(", ")
    ))
    .into())
}

fn report_slot(slots: &[ImageSlot]) {
    if slots.is_empty() {
        log::debug!("it reports no slots at all");
        return;
    }
    for slot in slots {
        if slot.hash.is_empty() {
            log::debug!("slot {} is empty or unverifiable", slot.slot);
            continue;
        }
        log::debug!(
            "slot {} carries {} ({}) [{}]",
            slot.slot,
            short(&slot.hash),
            named(&slot.version),
            slot.flags().join(" ")
        );
    }
}

// ---------------------------------------------------------------------------
// Confirming the keyboard came back
// ---------------------------------------------------------------------------

/// How long to give the keyboard to come back typing.
///
/// It has to enumerate, its halves have to link up, and only then has either
/// of them anything to say about itself. Generous on purpose: the cost of a
/// window that is too short is telling someone an update that worked did not,
/// and since the halves keep their images on the strength of those same links,
/// that is now a run that fails rather than a note under a run that passed.
/// The window is also what drives half of what it is waiting for — QMK's first
/// telemetry read is what the right half proves itself on — so it is looking
/// for something it is helping to happen, not sitting out a fixed delay.
const CONFIRM_WINDOW: Duration = Duration::from_secs(15);

/// The pause between looks, while the keyboard is still on its way back.
const CONFIRM_GAP: Duration = Duration::from_millis(500);

/// What the restart check's answer means for the run.
///
/// Everything before this was read out of a bootloader, which can say what is
/// in flash and nothing about the keyboard working. This is the one thing in
/// the run that watches the new firmware actually run, and it is the same
/// evidence the firmware's own health gate wants: the left's reading only
/// reaches the host once its UART link to QMK is up, and the right's crossed
/// the wire between the halves to be served to a QMK telemetry read. Both
/// answering is both halves alive and talking to each other on the build that
/// was just written.
///
/// Required rather than advisory. The images are permanent by the time this
/// runs, so nothing undoes a bad one on its own and a run that skipped this
/// would be reporting an install as a working keyboard on no evidence at all.
///
/// Nothing to look for when nothing was written: both halves are running what
/// they were running before the session.
fn finish(outcome: Outcome, look: impl FnOnce() -> bool) -> Result<()> {
    if outcome != Outcome::Flashed || look() {
        return Ok(());
    }
    bail!("{}", unproven())
}

/// What a keyboard that never came back after the update says.
///
/// Deliberately not "this update was undone": over the cable the images are
/// written to be kept, so both halves are on the bundle whatever happens next.
/// What has not been shown is that they work, and the difference matters to
/// someone deciding what to do about it.
fn unproven() -> String {
    "both halves took the update and are keeping it, but the keyboard has not come back on USB \
     with both halves reporting. The images are installed — this is the keyboard not running \
     them where this could see it. Unplug it and plug it back in, check the cable between the \
     halves is properly seated, and if it still does not come back, run the same command again \
     to read both halves and see where it stopped"
        .to_string()
}

/// Whether the keyboard came back on its own firmware, with both halves in it.
///
/// Everything before this read a half's slot while that half sat in MCUboot,
/// which proves the image landed and nothing about either half booting it.
/// This is the first thing in the run that talks to the running firmware, and
/// it asks for both halves at once: the right half's answer crosses the wire
/// between the halves, so getting both is both applications up and the two of
/// them talking to each other.
fn came_back(limit: Duration) -> bool {
    let spinner = new_spinner("Waiting for the keyboard to restart");
    let back = keep_looking(
        limit,
        CONFIRM_GAP,
        Instant::now,
        std::thread::sleep,
        both_halves_report,
    );
    spinner.finish_and_clear();
    back
}

/// Polls `look` until it answers, or until the window closes.
///
/// The clock and the pause are passed in so the window can be tested without
/// waiting one out: what is worth testing here is that it keeps asking for the
/// whole window and stops the moment it gets an answer.
fn keep_looking(
    limit: Duration,
    gap: Duration,
    mut now: impl FnMut() -> Instant,
    mut pause: impl FnMut(Duration),
    mut look: impl FnMut() -> bool,
) -> bool {
    let deadline = now() + limit;
    loop {
        if look() {
            return true;
        }
        if now() >= deadline {
            return false;
        }
        pause(gap);
    }
}

/// One look: is there a keyboard on USB, and does it answer for both halves?
fn both_halves_report() -> bool {
    let attached = attached().unwrap_or_default();
    let Some(keyboard) = voyager2_among(&attached) else {
        log::debug!("no keyboard on USB yet");
        return false;
    };

    let Some(telemetry) = crate::usb_telemetry(keyboard) else {
        log::debug!("the keyboard is back but has not answered yet");
        return false;
    };
    both_reported(&telemetry)
}

/// What is plugged in, or nothing when the bus cannot be read.
fn attached() -> Option<Vec<UsbDevice>> {
    device::list_devices()
        .inspect_err(|err| log::debug!("could not look at what is plugged in: {err}"))
        .ok()
}

/// The attached Voyager 2 running its own firmware, if one is there.
///
/// The model is checked, not just "a ZSA keyboard": an original Voyager on the
/// same cable is a board no bundle can be written to, and counting it sends the
/// update down the wire at something that will never answer.
///
/// A board still in a bootloader is not it either. For the restart wait that is
/// the whole point — the STM32 has to have left one — and for routing it is
/// [`bootloader_present`]'s to find.
fn voyager2_among(attached: &[UsbDevice]) -> Option<&UsbDevice> {
    attached
        .iter()
        .find(|device| device.bootloader.is_none() && device.keyboard == Some(Keyboard::Voyager2))
}

/// Whether a Voyager 2 running its own firmware is plugged in right now.
///
/// For deciding which way to send a bundle, alongside [`bootloader_present`]:
/// between them they are every USB state a bundle can be written from.
pub fn voyager2_present() -> bool {
    attached().is_some_and(|attached| voyager2_among(&attached).is_some())
}

/// Whether both halves are in what the board answered.
fn both_reported(telemetry: &Telemetry) -> bool {
    let quiet: Vec<&str> = [("left", telemetry.left), ("right", telemetry.right)]
        .into_iter()
        .filter(|(_, reading)| !reported(*reading))
        .map(|(half, _)| half)
        .collect();
    if quiet.is_empty() {
        log::debug!(
            "both halves answered: left {:?}, right {:?}",
            telemetry.left,
            telemetry.right
        );
        return true;
    }
    log::debug!("nothing from the {} half yet", quiet.join(" or the "));
    false
}

/// Whether one half said anything about itself.
///
/// A level is the ordinary answer, and the flags are the second one: they are
/// set only where the reading behind them is real, so a half whose fuel gauge
/// has not converged yet still counts as having answered. Which is the point —
/// this is asking whether the half is there, not what its battery is doing.
fn reported(reading: BatteryReading) -> bool {
    reading.level.is_some() || reading.charging.is_some() || reading.usb.is_some()
}

// ---------------------------------------------------------------------------
// Saying what went wrong
// ---------------------------------------------------------------------------

/// The far end of a failure's chain, when it says something the top does not.
///
/// `anyhow`'s alternate form prints every link, which on this path is mostly
/// plumbing. The last link is the exception: nothing wrapped it, so it is the
/// error as the board or the bus reported it, and it is the one thing a person
/// cannot work out from the sentence above it.
fn root_cause(err: &anyhow::Error) -> Option<String> {
    let root = err.chain().last()?.to_string();
    (root != err.to_string()).then_some(root)
}

/// Everything worth printing under a failure.
///
/// The board is asked again rather than remembered: STATUS carries the reason
/// the bootloader itself recorded, the bus recovery count and the bytes it had
/// to drop, and all three are current at the moment the failure is being
/// explained rather than at the last time anything happened to look.
fn advice<L: IgnitionLink>(ignition: &mut Ignition<L>) -> Vec<String> {
    let mut notes = Vec::new();
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
    if status.error == 0x0000 {
        return None;
    }
    log::debug!(
        "session error {:#06x}, bus {}",
        status.error,
        bus_phrase(status)
    );
    Some(
        match status.error {
            // Everything the right half is reached through is on the one
            // cable, so every way it can go quiet has the same answer.
            0x0002..=0x0005 | 0x0009 => "Make sure the TRRS cable is properly seated.",
            0x0001 => "The left half did not respond.",
            0x0006 => "This computer could not open the keyboard. On Linux, check the udev \
                 rule is installed.",
            0x0007 => "The keyboard's bootloader and this version of zapp do not match.",
            0x0008 => "Something went wrong inside zapp.",
            0x00FF => "Unplug the keyboard, plug it back in, and try again.",
            _ => "The keyboard's bootloader is newer than this version of zapp.",
        }
        .to_string(),
    )
}

/// A note about a bus that coped but had to work at it.
///
/// Only when the session recorded no error of its own: a run that failed
/// somewhere else while the bus was recovering repeatedly is worth pointing at
/// the bus anyway.
///
/// No counts. Reaching here means the far half was in its bootloader and the
/// transfer was moving, so the cable is the story and the numbers behind that
/// conclusion are ours, not the reader's.
fn bus_health(status: &Status) -> Option<String> {
    if status.recover_count == 0 && status.rx_drops == 0 {
        return None;
    }
    log::debug!("bus health at the failure: {}", bus_phrase(status));
    Some("Make sure the TRRS cable is properly seated.".to_string())
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

/// Whether a bootloader is plugged in right now, without waiting for one.
///
/// For deciding which way to send a bundle. Waiting would defeat the point:
/// the answer "not here" is what routes it over Bluetooth instead.
pub fn bootloader_present() -> bool {
    match UsbLink::open() {
        Ok(_) => true,
        Err(WiredError::NoBootloader) => false,
        Err(err) => {
            // Something is plugged in that cannot be opened — a permissions
            // problem, most likely. Say so rather than silently going over
            // the radio, where the same board would work and the cause would
            // stay hidden.
            log::warn!("a bootloader is present but could not be opened: {err}");
            false
        }
    }
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

    let spinner = new_spinner("Press your keyboard's reset button");
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
                    "no bootloader appeared in {} seconds. Press the reset button on the left \
                     half; `zapp devices` lists what is plugged in",
                    limit.as_secs()
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
    use zapp_core::device::BootloaderKind;
    use zapp_smp::fake::FakeSlot;
    use zapp_smp::{client, img};
    use zapp_wired::ignition;
    use zapp_wired::loopback::{LoopbackDevice, LoopbackLink};
    use zapp_wired::pipe;

    /// The hash the in-memory device reports for its one slot, whatever has
    /// been written to it.
    const DEVICE_HASH: [u8; 32] = [0x11; 32];

    /// The hash a bundle that is not already installed is named by.
    const BUNDLE_HASH: [u8; 32] = [0xab; 32];

    /// One image listing: the half is running `primary` with nothing pending.
    fn settled(primary: &[u8], spare: &[u8]) -> Vec<FakeSlot> {
        vec![
            FakeSlot::new(0, primary).active().confirmed(),
            FakeSlot::new(1, spare),
        ]
    }

    /// The same, with an image sitting in the spare slot waiting to swap. What
    /// the post-upload read sees, which happens before the mark.
    fn staged(primary: &[u8], spare: &[u8]) -> Vec<FakeSlot> {
        vec![
            FakeSlot::new(0, primary).confirmed(),
            FakeSlot::new(1, spare).pending(),
        ]
    }

    /// `BOOT_SWAP_TYPE_REVERT`: the new image is running and unconfirmed, so
    /// the next boot puts the old one back.
    fn reverting(primary: &[u8], spare: &[u8]) -> Vec<FakeSlot> {
        vec![
            FakeSlot::new(0, primary).active(),
            FakeSlot::new(1, spare).confirmed(),
        ]
    }

    /// The listings a run that writes both halves reads, in order: before and
    /// after each half's upload, then one per half on the read-back pass.
    fn a_whole_run(after_restart: Vec<FakeSlot>) -> Vec<Vec<FakeSlot>> {
        vec![
            settled(&DEVICE_HASH, &[]),
            staged(&DEVICE_HASH, &BUNDLE_HASH),
            settled(&DEVICE_HASH, &[]),
            staged(&DEVICE_HASH, &BUNDLE_HASH),
            after_restart.clone(),
            after_restart,
        ]
    }

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
    ///
    /// Named for the version the fake board reports, so the whole-keyboard
    /// check at the end of a run has nothing to object to. A test that wants
    /// it to object asks for another with `bundled_v`.
    fn bundled(left: &[u8], right: &[u8]) -> Vec<u8> {
        bundled_v(left, right, "9.9.9")
    }

    /// The same, calling itself something else.
    fn bundled_v(left: &[u8], right: &[u8], build_version: &str) -> Vec<u8> {
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
        version[..build_version.len()].copy_from_slice(build_version.as_bytes());
        out.extend_from_slice(&version);
        out.extend_from_slice(left);
        out.extend_from_slice(right);
        out
    }

    /// Runs the whole orchestration against the in-memory keyboard.
    fn flash_loopback(blob: &[u8]) -> (Result<Outcome>, Vec<(u8, u16)>, LoopbackDevice) {
        flash_loopback_with(blob, false)
    }

    /// The same, with the downgrade and skip guards overridden.
    fn flash_loopback_forced(blob: &[u8]) -> (Result<Outcome>, Vec<(u8, u16)>, LoopbackDevice) {
        flash_loopback_with(blob, true)
    }

    /// A run that writes both halves and reads them back carrying the bundle.
    ///
    /// Its images are named by a hash the board is not already reporting, so
    /// neither half is skipped and the run goes through both passes.
    fn flash_loopback_written() -> (Result<Outcome>, Vec<(u8, u16)>, LoopbackDevice) {
        let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
        flash_loopback_setup(&blob, false, |device| {
            device.tweak(|fake| fake.listings = a_whole_run(settled(&BUNDLE_HASH, &DEVICE_HASH)))
        })
    }

    fn flash_loopback_with(
        blob: &[u8],
        force: bool,
    ) -> (Result<Outcome>, Vec<(u8, u16)>, LoopbackDevice) {
        flash_loopback_setup(blob, force, |_| {})
    }

    /// The same, with the fake keyboard set up first.
    fn flash_loopback_setup(
        blob: &[u8],
        force: bool,
        setup: impl FnOnce(&LoopbackDevice),
    ) -> (Result<Outcome>, Vec<(u8, u16)>, LoopbackDevice) {
        let bundle = bundle::parse(blob).expect("a bundle");
        let (link, device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        setup(&device);
        let shared = Shared::new(link);
        let mut ignition = Ignition::new(shared.clone());

        let result = run(
            &shared,
            &mut ignition,
            &bundle,
            force,
            Duration::from_millis(50),
            // No settle: the in-memory halves have nothing to swap or reboot.
            Duration::ZERO,
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

        assert_eq!(
            result.expect("nothing to write is not a failure"),
            Outcome::UpToDate
        );
        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0);
        // And the storage erase belongs to an upload, not to a skipped half.
        assert_eq!(count(&device, (BASIC_GROUP, BASIC_CMD_ERASE_STORAGE)), 0);
        // One read per half, which is the read that decided it.
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 2);
    }

    #[test]
    fn a_keyboard_that_already_carries_the_bundle_stops_before_the_read_back() {
        // Nothing was staged, so there is nothing to swap and nothing to check:
        // a second session and a settle wait would both be spent on a keyboard
        // that was never touched.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, device) = flash_loopback(&blob);
        assert_eq!(result.unwrap(), Outcome::UpToDate);

        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            1,
            "the halves were re-entered to read them back: {control:?}"
        );
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 2);
        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0);
    }

    #[test]
    fn force_writes_a_keyboard_that_already_carries_the_bundle() {
        // The skip is what --force is for: an image the board is already
        // carrying is written again rather than reported as up to date.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (_result, _control, device) = flash_loopback_forced(&blob);

        assert!(
            count(&device, (img::GROUP, img::CMD_UPLOAD)) > 0,
            "--force skipped a half anyway"
        );
    }

    #[test]
    fn an_image_the_board_does_not_report_after_the_upload_fails_the_run() {
        // The post-upload read is the whole point: the upload itself succeeds
        // here — every byte is acknowledged — and the run still has to fail,
        // because the spare slot does not come back carrying what was sent.
        let blob = bundled(&signed(0x11, &[0xa1; 32]), &signed(0x22, &[0xb2; 32]));
        let (result, _control, device) = flash_loopback_setup(&blob, false, |device| {
            device.tweak(|fake| fake.listings = vec![settled(&DEVICE_HASH, &[0x99; 32])])
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("took the whole image"), "{err}");
        assert!(err.contains("not the"), "{err}");
        // It got as far as writing: one upload, then the read that caught it.
        // The mark never runs, because nothing worth arming is there.
        assert!(count(&device, (img::GROUP, img::CMD_UPLOAD)) > 0, "{err}");
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 2);
        assert!(device.marks().is_empty(), "a bad upload was armed anyway");
    }

    #[test]
    fn the_post_upload_check_reads_the_spare_slot_and_not_the_one_being_run() {
        // Slot 0 is already the bundle's image here and slot 1 is not, which
        // only fails if the check looks where the upload actually went.
        let blob = bundled(&signed(0x11, &[0xa1; 32]), &signed(0x22, &[0xb2; 32]));
        let (result, _control, _device) = flash_loopback_setup(&blob, true, |device| {
            device.tweak(|fake| fake.listings = vec![settled(&[0xb2; 32], &[0x99; 32])])
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("in its spare slot"), "{err}");
        assert!(err.contains("did not land"), "{err}");
    }

    #[test]
    fn a_written_half_stages_into_image_two_and_marks_it_to_be_kept() {
        let (result, _control, device) = flash_loopback_written();
        result.expect("both halves swapped");

        // Image 2 is slot1_partition under direct image upload. Image 0 or 1
        // would be the slot the half is running.
        assert_eq!(device.uploads(), vec![SPARE_IMAGE, SPARE_IMAGE]);
        // Permanent, not for test. `confirm: false` here is what took a whole
        // hardware run to diagnose: the half boots the test image with nothing
        // it can confirm on, the read-back's session re-entry reboots it, and
        // MCUboot puts the old image back before anything has read the new one.
        assert_eq!(
            device.marks(),
            vec![(BUNDLE_HASH.to_vec(), true), (BUNDLE_HASH.to_vec(), true)]
        );
    }

    /// A listing cut off after a key, the way MCUboot's runs out of buffer.
    ///
    /// The shape rather than the bytes: `zapp_smp::img` pins the real 243-byte
    /// capture from the board. What matters here is only that the reply does
    /// not decode.
    fn cut_short() -> Vec<u8> {
        let mut bytes = vec![0xbf]; // map(indefinite)
        bytes.push(0x66);
        bytes.extend_from_slice(b"images"); // "images"
        bytes.push(0x9f); // array(indefinite)
        bytes.push(0xbf); // map(indefinite)
        bytes.push(0x67);
        bytes.extend_from_slice(b"version"); // a key with no value after it
        bytes.extend_from_slice(&[0xff, 0xff, 0xff]);
        bytes
    }

    #[test]
    fn a_mark_whose_listing_did_not_fit_the_board_is_still_a_mark() {
        // The reply to a mark is a whole listing, and arming the swap is what
        // makes that listing too long: MCUboot answers a refusal with ten
        // bytes of `rc` that always decode, so a reply that will not decode
        // can only have come from the path that already wrote the trailer.
        let mut fake = zapp_smp::fake::FakeDevice::new(128);
        fake.buffers = None;
        fake.listing_bytes = Some(cut_short());
        let marks = Arc::clone(&fake.marks);
        let mut client = SmpClient::new(Box::new(fake)).with_timeout(Duration::from_millis(50));

        arm(&mut client, Half::Right, &BUNDLE_HASH)
            .expect("a listing that did not fit is not a mark that did not happen");
        // And the mark really went out, permanently.
        assert_eq!(
            marks.lock().unwrap().clone(),
            vec![(BUNDLE_HASH.to_vec(), true)]
        );
    }

    #[test]
    fn a_mark_the_board_actually_refused_still_fails() {
        // The other side of it: an `rc` the board sent on purpose decodes
        // perfectly well, and must not be swallowed with the short listings.
        let mut fake = zapp_smp::fake::FakeDevice::new(128);
        fake.buffers = None;
        fake.fail_with = Some(33); // IMG_MGMT_ERR_IMAGE_SETTING_TEST_TO_ACTIVE_DENIED
        let mut client = SmpClient::new(Box::new(fake)).with_timeout(Duration::from_millis(50));

        let err = arm(&mut client, Half::Right, &BUNDLE_HASH)
            .expect_err("a refusal is not a short listing");
        let said = format!("{err:#}");
        assert!(said.contains("Could not mark the right half"), "{said}");
    }

    #[test]
    fn a_failure_says_what_the_board_called_it_and_not_only_what_this_end_was_doing() {
        // The line the wired path prints used to be the outermost context on
        // its own, which turned "bad CBOR from the board" into "Could not read
        // the right half's image state" and cost a bench session.
        let err = anyhow::anyhow!("bad CBOR from the board: invalid type: break")
            .context("Could not read the right half's image state");
        assert_eq!(
            root_cause(&err).as_deref(),
            Some("bad CBOR from the board: invalid type: break")
        );
        // A failure nothing wrapped has nothing to add, and must not repeat
        // itself in brackets.
        assert_eq!(root_cause(&anyhow::anyhow!("the cable came out")), None);
    }

    #[test]
    fn a_test_mark_is_never_sent_over_the_cable() {
        // Separately from the whole-run assertion above, because this is the
        // property and not the sequence: nothing on this path may arm an image
        // that MCUboot is entitled to take away again.
        let (_result, _control, device) = flash_loopback_written();
        assert!(
            device.marks().iter().all(|(_, permanent)| *permanent),
            "an image was armed for test: {:?}",
            device.marks()
        );
    }

    #[test]
    fn a_swap_that_completed_and_was_confirmed_passes_the_read_back() {
        let (result, control, device) = flash_loopback_written();
        assert_eq!(result.expect("both halves swapped"), Outcome::Flashed);

        assert!(count(&device, (img::GROUP, img::CMD_UPLOAD)) > 0);
        // Two sessions: the one that wrote, and the one that read back after
        // both halves swapped.
        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            2
        );
    }

    #[test]
    fn a_swap_the_halves_have_not_confirmed_yet_is_not_a_failure() {
        // A permanent mark means a half this run wrote comes back confirmed, so
        // this is not the ordinary case any more — it is the one a half nobody
        // marked lands in: skipped as already carrying the image while it sits
        // mid-test-boot from an older zapp. The hash is in the right slot, and
        // a half's links are not something a bootloader can report on.
        let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
        let (result, _control, _device) = flash_loopback_setup(&blob, false, |device| {
            device.tweak(|fake| fake.listings = a_whole_run(reverting(&BUNDLE_HASH, &DEVICE_HASH)))
        });

        assert_eq!(
            result.expect("an unconfirmed swap is not a failed one"),
            Outcome::Flashed
        );
    }

    #[test]
    fn a_half_that_went_back_to_its_old_image_is_the_failure_the_hardware_saw() {
        // What a test mark actually produced on a board: the half booted the
        // new image with nothing it could confirm on, the read-back's session
        // re-entry rebooted it, MCUboot reverted, and every pass afterwards
        // read the old hash until the settle window ran out. The mark is
        // permanent now, so this listing should no longer be reachable — but
        // it is still what a reverted or half-swapped half looks like, and the
        // run has to end on the hash it read rather than on a timeout nobody
        // can interpret.
        let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
        let (result, _control, _device) = flash_loopback_setup(&blob, false, |device| {
            // Slot 0 back on what the board had, the bundle's image sitting in
            // the spare: BOOT_SWAP_TYPE_REVERT, after it ran.
            device.tweak(|fake| fake.listings = a_whole_run(settled(&DEVICE_HASH, &BUNDLE_HASH)))
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("After restarting"), "{err}");
        assert!(err.contains("in its primary slot"), "{err}");
        assert!(err.contains("not what the bundle carries"), "{err}");
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
        let (result, control, _device) = flash_loopback_written();
        result.unwrap();

        let selects: Vec<u16> = control
            .iter()
            .filter(|(request, _)| *request == ignition::REQ_SELECT)
            .map(|(_, value)| *value)
            .collect();
        // 2 is right, then 1 twice: the bridge is pointed away from the right
        // as soon as it has been told to reboot, and then at the left again as
        // the left's own turn begins. The read-back pass repeats the pattern.
        assert_eq!(selects, vec![2, 1, 1, 2, 1, 1]);
    }

    #[test]
    fn the_write_pass_and_the_read_back_pass_are_one_session_each_and_one_end() {
        let (result, control, _device) = flash_loopback_written();
        result.unwrap();

        // Every vendor request, in order, with the STATUS polling dropped: it
        // is a poll and its count is not part of the sequence.
        let sequence: Vec<(u8, u16)> = control
            .into_iter()
            .filter(|(request, _)| *request != ignition::REQ_STATUS)
            .collect();
        let pass = vec![
            // No END between the passes: BEGIN closes a session it finds in
            // the way, and ending one here would throw away the fault a
            // failure is explained from.
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
        ];
        let mut expected = pass.clone();
        expected.extend(pass);
        expected.push((ignition::REQ_END, 0));
        assert_eq!(sequence, expected);
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
        assert_eq!(result.unwrap(), Outcome::UpToDate);

        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0);
        assert!(
            control.contains(&(ignition::REQ_RESET_TARGET, 1)),
            "the left half was not reset: {control:?}"
        );
        // One pass, so one reset: the read-back that would make a second is
        // not run when nothing was written.
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
        let (result, control, device) = flash_loopback_written();
        result.unwrap();

        assert!(
            !control.contains(&(ignition::REQ_RESET_TARGET, 2)),
            "the right half was sent RESET_TARGET: {control:?}"
        );
        assert_eq!(
            count(&device, (client::OS_GROUP, client::OS_CMD_RESET)),
            2,
            "expected one SMP reset for the right half in each pass"
        );
    }

    #[test]
    fn the_left_halfs_md_pin_is_released_before_it_is_reset_and_never_after() {
        let (result, control, _device) = flash_loopback_written();
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
        // And it is not sent for the right half, which has no MD pin held: one
        // release per pass, both of them the left's.
        assert_eq!(
            control
                .iter()
                .filter(|sent| **sent == (ignition::REQ_RELEASE_MD, 0))
                .count(),
            2
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

    // ---- Reading the halves back ---------------------------------------

    #[test]
    fn both_halves_are_read_again_after_they_restart_and_a_match_passes() {
        let (result, control, device) = flash_loopback_written();
        result.expect("both halves still carry the bundle after their restart");

        // Four state commands per half: the read that decides, the read after
        // the upload, the mark that arms it — a write of the same command —
        // and the read on a session opened after both halves restarted.
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 8);
        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            2,
            "the halves were not put back in their bootloaders to be read: {control:?}"
        );
    }

    #[test]
    fn a_half_that_stops_carrying_the_bundles_image_after_it_restarts_fails_the_run() {
        // Both halves take the whole image and land it, and the slot they are
        // running afterwards carries something else. Nothing before it went
        // wrong, which is the whole reason to look again.
        let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
        let (result, _control, _device) = flash_loopback_setup(&blob, false, |device| {
            device.tweak(|fake| fake.listings = a_whole_run(settled(&[0x99; 32], &DEVICE_HASH)))
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("After restarting"), "{err}");
        assert!(err.contains("not what the bundle carries"), "{err}");
        assert!(!err.contains("took the whole image"), "{err}");
    }

    /// A link whose STATUS always reports one recorded fault.
    struct Faulting(u16);

    impl IgnitionLink for Faulting {
        fn control_out(
            &mut self,
            _request: u8,
            _value: u16,
        ) -> std::result::Result<(), WiredError> {
            Ok(())
        }

        fn control_in(
            &mut self,
            _request: u8,
            _value: u16,
            length: usize,
        ) -> std::result::Result<Vec<u8>, WiredError> {
            let mut bytes = vec![0u8; ignition::STATUS_LEN];
            bytes[0] = ignition::STATUS_VERSION;
            bytes[1] = 2; // fault
            bytes[5..7].copy_from_slice(&self.0.to_le_bytes());
            bytes.truncate(length);
            Ok(bytes)
        }

        fn bulk_out(&mut self, _message: &[u8]) -> std::result::Result<(), WiredError> {
            Ok(())
        }

        fn bulk_in(&mut self, _timeout: Duration) -> std::result::Result<Vec<u8>, WiredError> {
            Ok(Vec::new())
        }
    }

    #[test]
    fn a_session_the_right_half_would_not_answer_is_the_one_read_back_worth_excusing() {
        // The right's bootloader entry is the flaky step and the write pass
        // already proved both halves' bytes, so this is the one way the
        // read-back is allowed to end inconclusively.
        for error in [0x0002, 0x0003, 0x0004, 0x0005] {
            assert!(
                right_absent(&mut Ignition::new(Faulting(error))),
                "{error:#06x}"
            );
        }
        // Everything else is a failure, including a session with no fault at
        // all — which is not a reason the read-back could not be made.
        for error in [0x0000, 0x0001, 0x0006, 0x0007, 0x0008, 0x00FF] {
            assert!(
                !right_absent(&mut Ignition::new(Faulting(error))),
                "{error:#06x}"
            );
        }
    }

    #[test]
    fn a_read_back_the_right_half_will_not_answer_fails_the_run() {
        // Staging is not installing: the write pass proved the bytes reached
        // the spare slot, and the swap that puts them in the slot the half runs
        // from has still to happen. A check that could not be made is not a
        // success.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let bundle = bundle::parse(&blob).unwrap();
        let shared = Shared::new(Faulting(0x0002));
        let mut ignition = Ignition::new(shared.clone());
        let mut carried = [
            on_version(Half::Right, "9.9.9"),
            on_version(Half::Left, "9.9.9"),
        ];

        let err = verify(
            &shared,
            &mut ignition,
            &bundle,
            &mut carried,
            Duration::from_millis(50),
            Duration::ZERO,
            Duration::ZERO,
        )
        .expect_err("a read-back that could not be made is not a success");

        // And it says which half went quiet and what is still at stake, rather
        // than surfacing the session fault on its own.
        let said = format!("{err}");
        assert!(said.contains("right half did not answer"), "{said}");
        assert!(said.contains("has not been checked"), "{said}");
        // Nothing was read, so nothing is claimed about either half.
        assert!(carried.iter().all(|entry| entry.hash.is_none()));
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

    /// The same, confirmed: a primary slot with no swap outstanding.
    fn kept(number: u32, hash: &[u8]) -> ImageSlot {
        ImageSlot {
            confirmed: true,
            ..slot(number, hash)
        }
    }

    #[test]
    fn the_slot_the_half_is_running_is_what_decides_it_is_already_installed() {
        let wanted = [0xa1; 32];
        assert!(installed(&[slot(0, &wanted)], &wanted));
        // The same image staged in the spare slot has not been swapped in yet,
        // so the half is not carrying it.
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
    fn an_image_that_landed_in_the_spare_slot_is_accepted_and_anything_else_is_not() {
        let wanted = [0xa1; 32];
        // Slot 1, because the upload stages rather than overwrites: the same
        // image in slot 0 is the *old* half still running, not the new one.
        assert!(landed(
            &[kept(0, &[0x99; 32]), slot(1, &wanted)],
            &wanted,
            Half::Left,
            Checked::Written
        )
        .is_ok());
        assert!(landed(&[kept(0, &wanted)], &wanted, Half::Left, Checked::Written).is_err());

        let wrong = landed(
            &[kept(0, &[0x11; 32]), slot(1, &[0x99; 32])],
            &wanted,
            Half::Right,
            Checked::Written,
        )
        .unwrap_err()
        .to_string();
        assert!(wrong.contains("right"), "{wrong}");
        assert!(wrong.contains("did not land"), "{wrong}");

        // A slot MCUboot could not verify reports no hash at all, which is a
        // different thing to say than "it carries the wrong image".
        let empty = landed(&[slot(1, &[])], &wanted, Half::Left, Checked::Written)
            .unwrap_err()
            .to_string();
        assert!(empty.contains("not bootable"), "{empty}");

        let missing = landed(&[], &wanted, Half::Left, Checked::Written)
            .unwrap_err()
            .to_string();
        assert!(missing.contains("no spare slot"), "{missing}");
    }

    #[test]
    fn a_read_back_after_the_restart_says_so_rather_than_blaming_the_upload() {
        // The same comparison, at a point where the upload is long finished:
        // an upload that did not land is the wrong thing to tell someone whose
        // half has already restarted.
        let wanted = [0xa1; 32];
        assert!(landed(&[kept(0, &wanted)], &wanted, Half::Left, Checked::Restarted).is_ok());

        let wrong = landed(
            &[kept(0, &[0x99; 32])],
            &wanted,
            Half::Right,
            Checked::Restarted,
        )
        .unwrap_err()
        .to_string();
        assert!(wrong.contains("After restarting"), "{wrong}");
        assert!(wrong.contains("not what the bundle carries"), "{wrong}");
        assert!(!wrong.contains("took the whole image"), "{wrong}");
    }

    #[test]
    fn a_swap_that_completed_passes_whether_or_not_the_half_has_confirmed_it() {
        // MCUboot's BOOT_SWAP_TYPE_REVERT listing: the new image is in slot 0
        // and running with nothing confirming it. Over the cable that is what
        // every half looks like — the STM32 is running ignition, so no half has
        // a link to prove itself on — and the hash is a verified image in the
        // slot the half runs from, which is the swap having completed.
        let wanted = [0xa1; 32];
        let unconfirmed = [
            ImageSlot {
                active: true,
                ..slot(0, &wanted)
            },
            kept(1, &[0x99; 32]),
        ];
        assert!(landed(&unconfirmed, &wanted, Half::Left, Checked::Restarted).is_ok());

        // And the hash still decides: the same unconfirmed slot carrying
        // something else is the failure it always was.
        let wrong = landed(
            &[ImageSlot {
                active: true,
                ..slot(0, &[0x99; 32])
            }],
            &wanted,
            Half::Right,
            Checked::Restarted,
        )
        .unwrap_err()
        .to_string();
        assert!(wrong.contains("not what the bundle carries"), "{wrong}");

        // An unverifiable slot reports no hash at all, confirm or no confirm.
        let empty = landed(&[slot(0, &[])], &wanted, Half::Left, Checked::Restarted)
            .unwrap_err()
            .to_string();
        assert!(empty.contains("not bootable"), "{empty}");
    }

    #[test]
    fn a_half_carrying_the_image_without_a_confirm_is_still_carrying_it() {
        // The confirm is a fact about the keyboard's links being up, not about
        // what is in flash, so the skip decision does not read it. A half
        // pulled into recovery by this very run cannot have confirmed anything.
        let wanted = [0xa1; 32];
        assert!(installed(&[slot(0, &wanted)], &wanted));
        assert!(installed(&[kept(0, &wanted)], &wanted));
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
    fn every_way_the_right_half_goes_quiet_gives_the_same_answer() {
        // Cable, mailbox, expander, bus: one cable carries all of it, so one
        // sentence covers all of it. The codes and counts go to the log.
        for code in [0x0002, 0x0003, 0x0004, 0x0005, 0x0009] {
            let note = session_error(&faulted(code, 3, 0)).unwrap();
            assert_eq!(note, "Make sure the TRRS cable is properly seated.");
        }
    }

    #[test]
    fn every_error_the_bootloader_can_record_says_something_plain() {
        for code in (1..=9).chain(std::iter::once(0xFF)) {
            let note = session_error(&faulted(code, 0, 0)).expect("a sentence");
            // Nothing a person cannot act on: no raw codes, no counters, no
            // parts of the machine they have never heard of.
            assert!(!note.contains("0x"), "a raw code in prose: {note}");
            assert!(!note.to_lowercase().contains("mcuboot"), "{note}");
            assert!(!note.to_lowercase().contains("mailbox"), "{note}");
            assert!(!note.contains("bus recover"), "{note}");
        }
    }

    #[test]
    fn a_session_with_nothing_wrong_says_nothing() {
        assert_eq!(session_error(&faulted(0, 0, 0)), None);
        // A code from a firmware newer than this build still gets a sentence
        // someone can act on rather than the number itself.
        let newer = session_error(&faulted(0x0042, 0, 0)).unwrap();
        assert!(!newer.contains("0x0042"), "{newer}");
        assert!(newer.contains("newer than this version"), "{newer}");
    }

    #[test]
    fn a_bus_that_coped_but_had_to_work_at_it_is_still_worth_a_line() {
        assert_eq!(bus_health(&faulted(0, 0, 0)), None);
        let note = bus_health(&faulted(0, 2, 9)).unwrap();
        // The counts belong in the debug log, not in front of someone whose
        // cable has come loose: reaching here means the far half was in its
        // bootloader and bytes were moving, so seating the cable is the whole
        // of the advice.
        assert_eq!(note, "Make sure the TRRS cable is properly seated.");
    }

    #[test]
    fn one_recovery_is_not_pluralised() {
        assert!(bus_phrase(&faulted(0, 1, 0)).contains("1 bus recovery at 50 kHz"));
        assert!(bus_phrase(&faulted(0, 0, 0)).contains("0 bus recoveries"));
    }

    // ---- Waiting for the right half ------------------------------------

    /// An attempt that fails `misses` times with the given fault before the
    /// session opens, and counts how many times it was asked.
    fn attempts_until(misses: usize, error: u16) -> (Result<Status>, usize) {
        let mut asked = 0;
        let result = keep_asking(Duration::from_secs(30), || {
            asked += 1;
            if asked > misses {
                return Ok(faulted(0, 0, 0));
            }
            let status = faulted(error, 0, 0);
            Err(NotOpened {
                right_missing: right_missing(status.error),
                error: anyhow!("{}", session_error(&status).unwrap()),
            })
        });
        (result, asked)
    }

    #[test]
    fn a_right_half_that_is_not_there_yet_is_asked_for_again() {
        // The one attempt this used to make gave the operator about five
        // seconds; on the bench the power-cycle landed on the third or
        // fourth. Anything that gives up on the first is the old bug back.
        let (result, asked) = attempts_until(3, 0x0002);
        result.expect("the fourth attempt found it");
        assert_eq!(asked, 4);
    }

    #[test]
    fn every_fault_a_missing_right_half_makes_is_waited_out() {
        for error in [0x0002, 0x0003, 0x0004, 0x0005] {
            let (result, asked) = attempts_until(2, error);
            assert!(result.is_ok(), "{error:#06x} was not retried");
            assert_eq!(asked, 3, "{error:#06x}");
        }
    }

    #[test]
    fn a_fault_the_right_half_has_nothing_to_do_with_fails_on_the_first_ask() {
        // The left half not answering over its UART, and an internal error:
        // waiting through either is thirty seconds of nothing happening.
        for error in [0x0001, 0x0006, 0x0007, 0x0008, 0x00FF] {
            let (result, asked) = attempts_until(1, error);
            assert!(result.is_err(), "{error:#06x} was waited out");
            assert_eq!(asked, 1, "{error:#06x}");
        }
    }

    #[test]
    fn a_right_half_that_never_comes_back_gives_up_with_the_bootloaders_reason() {
        let mut asked = 0;
        let result = keep_asking(Duration::ZERO, || {
            asked += 1;
            Err(NotOpened {
                right_missing: true,
                error: anyhow!("{}", session_error(&faulted(0x0002, 0, 0)).unwrap()),
            })
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("TRRS cable"), "{err}");
        assert_eq!(asked, 1, "it kept asking past its own limit");
    }

    // ---- One version across the keyboard -------------------------------

    #[test]
    fn a_keyboard_whose_halves_disagree_is_not_a_success() {
        // Both halves take the bundle's images and read back carrying them, so
        // every check up to here passed — and the run still has to fail,
        // because the version they came out on is not the one the bundle names.
        let blob = bundled_v(
            &signed(0x11, &BUNDLE_HASH),
            &signed(0x22, &BUNDLE_HASH),
            "0.1.9",
        );
        let (result, _control, _device) = flash_loopback_setup(&blob, false, |device| {
            device.tweak(|fake| fake.listings = a_whole_run(settled(&BUNDLE_HASH, &DEVICE_HASH)))
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(
            err.contains("did not come out of this on one build"),
            "{err}"
        );
        assert!(err.contains("the left half is on 9.9.9"), "{err}");
        assert!(err.contains("the right half is on 9.9.9"), "{err}");
        assert!(err.contains("the bundle is 0.1.9"), "{err}");
    }

    #[test]
    fn a_version_mismatch_is_told_apart_from_a_half_left_in_its_bootloader() {
        // Which is what the failure epilogue keys off: by the time the
        // versions are compared both halves have been restarted.
        let mismatch: anyhow::Error = Mismatched("the halves disagree".into()).into();
        assert!(mismatch.chain().any(|cause| cause.is::<Mismatched>()));
        let other = anyhow::anyhow!("the upload timed out");
        assert!(!other.chain().any(|cause| cause.is::<Mismatched>()));
    }

    /// One half's answer at the end of a run, with no hash read back.
    fn on_version(half: Half, version: &str) -> Carried {
        Carried {
            half,
            version: version.to_string(),
            hash: None,
        }
    }

    /// The same, with a hash the read-back found.
    fn carrying(half: Half, version: &str, hash: &[u8]) -> Carried {
        Carried {
            half,
            version: version.to_string(),
            hash: Some(hash.to_vec()),
        }
    }

    #[test]
    fn one_version_across_both_halves_and_the_bundle_passes() {
        let bundle = bundle::parse(&bundled(
            &signed(0x11, &DEVICE_HASH),
            &signed(0x22, &DEVICE_HASH),
        ))
        .unwrap();
        let same = [
            on_version(Half::Left, "9.9.9"),
            on_version(Half::Right, "9.9.9"),
        ];
        assert!(agreed(&bundle, &same).is_ok());

        // A build number on one side and not the other is the same version.
        let padded = [
            on_version(Half::Left, "9.9.9"),
            on_version(Half::Right, "9.9.9+0"),
        ];
        assert!(agreed(&bundle, &padded).is_ok());

        // A half that reported no version at all cannot be checked, and an
        // unchecked half is not a success either.
        let missing = [on_version(Half::Left, "9.9.9"), on_version(Half::Right, "")];
        let err = agreed(&bundle, &missing).unwrap_err().to_string();
        assert!(err.contains("(no version)"), "{err}");
    }

    #[test]
    fn a_half_whose_read_back_hash_is_not_the_bundles_is_reported_by_identity() {
        // Both halves call themselves the bundle's version and one of them is
        // carrying something else, which a version string cannot show.
        let bundle = bundle::parse(&bundled(
            &signed(0x11, &DEVICE_HASH),
            &signed(0x22, &DEVICE_HASH),
        ))
        .unwrap();
        let split = [
            carrying(Half::Left, "9.9.9", &DEVICE_HASH),
            carrying(Half::Right, "9.9.9", &[0x99; 32]),
        ];

        let err = agreed(&bundle, &split).unwrap_err();
        let text = err.to_string();
        assert!(
            text.contains("did not come out of this on one build"),
            "{text}"
        );
        assert!(text.contains("the right half is carrying"), "{text}");
        assert!(!text.contains("the left half"), "{text}");
        assert!(text.contains("cannot talk over the wire"), "{text}");
        // And it is still the type the failure epilogue tells apart.
        assert!(err.chain().any(|cause| cause.is::<Mismatched>()));
    }

    #[test]
    fn both_halves_carrying_the_bundles_own_hashes_passes() {
        let bundle = bundle::parse(&bundled(
            &signed(0x11, &DEVICE_HASH),
            &signed(0x22, &DEVICE_HASH),
        ))
        .unwrap();
        let good = [
            carrying(Half::Left, "9.9.9", &bundle.hash(Half::Left).unwrap()),
            carrying(Half::Right, "9.9.9", &bundle.hash(Half::Right).unwrap()),
        ];
        assert!(agreed(&bundle, &good).is_ok());
    }

    // ---- Confirming the keyboard came back -----------------------------

    #[test]
    fn a_keyboard_that_does_not_come_back_fails_a_run_that_wrote_something() {
        // It used to be a note under a successful run. It cannot be: the
        // images are permanent by now, so nothing takes a bad one away on its
        // own, and this is the only thing in the run that watches the new
        // firmware run at all.
        let err = finish(Outcome::Flashed, || false)
            .expect_err("a keyboard that never came back is not a finished update");
        let said = format!("{err}");
        assert!(said.contains("has not come back on USB"), "{said}");
        // And it says the images are installed rather than about to disappear:
        // over the cable nothing reverts, so sending someone to power-cycle in
        // the hope of getting the old firmware back would be a lie.
        assert!(said.contains("The images are installed"), "{said}");
        assert!(!said.contains("old firmware back"), "{said}");
        assert!(said.contains("cable between the halves"), "{said}");
    }

    #[test]
    fn a_keyboard_that_comes_back_with_both_halves_reporting_finishes_the_run() {
        assert!(finish(Outcome::Flashed, || true).is_ok());
    }

    #[test]
    fn a_run_that_wrote_nothing_is_not_waited_on() {
        // Both halves are running what they were running before the session,
        // which they had already proved. There is nothing new to keep, so
        // nothing to prove, and a keyboard someone unplugged after an
        // up-to-date run is not a failure.
        assert!(finish(Outcome::UpToDate, || panic!("it looked anyway")).is_ok());
    }

    /// Runs the window against a keyboard that answers on the `answers_on`th
    /// look, on a clock the pause moves itself. Answers with whether it was
    /// found, how many looks it took, and how much of the window went by.
    fn looking(limit: Duration, answers_on: usize) -> (bool, usize, Duration) {
        let base = Instant::now();
        let gone = std::cell::Cell::new(Duration::ZERO);
        let mut looks = 0;
        let back = keep_looking(
            limit,
            CONFIRM_GAP,
            || base + gone.get(),
            |gap| gone.set(gone.get() + gap),
            || {
                looks += 1;
                looks >= answers_on
            },
        );
        (back, looks, gone.get())
    }

    #[test]
    fn a_keyboard_that_is_already_back_is_not_waited_for() {
        let (back, looks, gone) = looking(CONFIRM_WINDOW, 1);
        assert!(back);
        assert_eq!(looks, 1);
        assert_eq!(gone, Duration::ZERO);
    }

    #[test]
    fn a_keyboard_that_takes_a_moment_is_still_a_success() {
        // Both halves need a few seconds after the reset before either has
        // anything to say, so a first look that finds nothing is the ordinary
        // case, not a failure.
        let (back, looks, _) = looking(CONFIRM_WINDOW, 6);
        assert!(back);
        assert_eq!(looks, 6);
    }

    #[test]
    fn a_keyboard_that_never_comes_back_is_asked_for_the_whole_window() {
        let (back, looks, gone) = looking(CONFIRM_WINDOW, usize::MAX);
        assert!(!back);
        assert!(gone >= CONFIRM_WINDOW, "it gave up early, after {gone:?}");
        assert_eq!(looks, 31); // one at zero, then one every half second
    }

    /// One half's reading: a level, and whether the board said its flags mean
    /// anything.
    fn heard(level: Option<u8>, flags: bool) -> BatteryReading {
        BatteryReading {
            level,
            charging: flags.then_some(false),
            usb: flags.then_some(true),
            source: None,
        }
    }

    fn answering(left: BatteryReading, right: BatteryReading) -> Telemetry {
        Telemetry {
            left,
            right,
            ..Default::default()
        }
    }

    #[test]
    fn one_half_answering_is_not_the_keyboard_being_back() {
        // The left's reading is the left nRF's application talking to the
        // STM32; the right's crossed the wire between the halves. Either one
        // alone leaves the other half unaccounted for.
        assert!(!both_reported(&answering(
            heard(Some(84), true),
            heard(None, false)
        )));
        assert!(!both_reported(&answering(
            heard(None, false),
            heard(Some(81), true)
        )));
        assert!(!both_reported(&answering(
            heard(None, false),
            heard(None, false)
        )));
        assert!(both_reported(&answering(
            heard(Some(84), true),
            heard(Some(81), true)
        )));
    }

    /// A ZSA board on USB, the way `list_devices` reports one.
    fn plugged_in(keyboard: Option<Keyboard>, pid: u16) -> UsbDevice {
        UsbDevice {
            keyboard,
            bootloader: None,
            serial: Some("layout/revision".to_string()),
            vid: 0x3297,
            pid,
        }
    }

    /// The other ZSA keyboards, none of which a bundle can be written to.
    fn the_other_models() -> Vec<UsbDevice> {
        vec![
            plugged_in(Some(Keyboard::Voyager), 0x1977),
            plugged_in(Some(Keyboard::Moonlander), 0x1969),
            plugged_in(Some(Keyboard::ErgodoxEz), 0x1307),
            plugged_in(Some(Keyboard::PlanckEz), 0x6060),
        ]
    }

    #[test]
    fn another_zsa_keyboard_on_the_cable_is_not_a_voyager_2() {
        // The bug this guards: a wired Voyager plugged in while the Voyager 2
        // is on Bluetooth used to count as "a keyboard on USB", so the bundle
        // went down the cable and sat waiting for a reset button on a board it
        // could never be written to.
        for other in the_other_models() {
            let pid = other.pid;
            assert!(
                voyager2_among(&[other]).is_none(),
                "{pid:#06x} was taken for a Voyager 2"
            );
        }
        assert!(voyager2_among(&the_other_models()).is_none());
    }

    #[test]
    fn a_voyager_2_among_other_boards_is_still_found() {
        // Whichever order the bus happens to report them in: a user with two
        // ZSA keyboards plugged in must still get the cable.
        let mut mixed = the_other_models();
        mixed.push(plugged_in(Some(Keyboard::Voyager2), 0x1978));
        assert_eq!(voyager2_among(&mixed).map(|found| found.pid), Some(0x1978));
        mixed.reverse();
        assert_eq!(voyager2_among(&mixed).map(|found| found.pid), Some(0x1978));
    }

    #[test]
    fn a_voyager_2_in_a_bootloader_is_not_one_running_its_firmware() {
        // `bootloader_present` finds that one. Counting it here would report a
        // board as back from its restart while it is still sitting in MCUboot.
        let mut booting = plugged_in(Some(Keyboard::Voyager2), 0x0792);
        booting.bootloader = Some(BootloaderKind::IgnitionV2);
        assert!(voyager2_among(&[booting]).is_none());
        // A bootloader that names no keyboard at all is not it either.
        let mut generic = plugged_in(None, 0xDF11);
        generic.vid = 0x0483;
        generic.bootloader = Some(BootloaderKind::Stm32Dfu);
        assert!(voyager2_among(&[generic]).is_none());
        assert!(voyager2_among(&[]).is_none());
    }

    #[test]
    fn a_half_whose_gauge_has_not_converged_still_answered() {
        // The right half's flags come from state its nRF holds locally, so
        // they are there whenever the read across the halves worked — which is
        // the thing being confirmed. A fresh boot with no level yet is a half
        // that is up, and 0% is a real level, not a missing one.
        assert!(both_reported(&answering(
            heard(Some(0), true),
            heard(None, true)
        )));
    }

    #[test]
    fn a_bundle_named_after_its_release_is_still_read_as_a_version() {
        // `0.1.9-wired` is what a real bundle is packed as, and it names the
        // same build as the `0.1.9` a board reports.
        assert_eq!(ordering_of("0.1.9-wired"), Some((0, 1, 9)));
        assert_eq!(ordering_of("0.1.9"), Some((0, 1, 9)));
        assert_eq!(ordering_of("0.1.9+42"), Some((0, 1, 9)));
        assert_eq!(ordering_of(""), None);

        // And a bundle whose name is not a version falls back to what its
        // own images say they are.
        let blob = bundled_v(
            &signed_v(0x11, &DEVICE_HASH, (2, 3, 4)),
            &signed_v(0x22, &DEVICE_HASH, (2, 3, 4)),
            "nightly",
        );
        let bundle = bundle::parse(&blob).unwrap();
        assert_eq!(ordering_of(&wanted_version(&bundle)), Some((2, 3, 4)));
    }
}
