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
//! Every wait polls: ignition's STATUS, an SMP reply, or the running half's
//! answer to a confirm query — see [`SETTLE_TIMEOUT`].
//!
//! The write goes into the spare slot and is marked for test, exactly as the
//! Bluetooth path marks it, and nothing is kept until both halves have been
//! seen running what was written. That was not possible until ignition grew the
//! confirm tunnel, and the reason is worth keeping:
//!
//! Each half's firmware used to keep a test image once it had proved its own
//! link to the rest of the keyboard, and none of those proofs can happen during
//! a wired session — the STM32 runs ignition, not QMK, from the first BEGIN to
//! the reset at the end. So the mark was permanent, blind, before anything had
//! been verified, and the check that followed was a read-back that re-entered
//! the bootloaders: a reboot, on which MCUboot would have reverted a test image
//! before anyone could look at it.
//!
//! The tunnel replaces both. Ignition now carries one small request to a half
//! that is *running its application* — over the UART for the left, over the
//! app's own I2C mailbox at 0x54 for the right — naming the image the host
//! expects: the half answers with the hash it is actually executing and
//! confirms its trailer only if the two match. That is a strictly better check
//! than the read-back was, because it comes from the running firmware rather
//! than from a bootloader looking at flash, and it needs no reboot to make. So
//! both halves are verified first and kept afterwards, together, and a pair
//! that does not come out of it right is reset back onto what it was running —
//! see [`revert_both`]. [`came_back`] still runs last and still asks the same
//! separate question: whether the keyboard is up and typing.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use indicatif::ProgressBar;

use zapp_core::device::access::ACCESS_RETRY;
use zapp_core::device::{self, AccessGrace, Keyboard, UsbDevice};
use zapp_core::flash::FlashProgress;
use zapp_core::hid::{BatteryReading, Telemetry};
use zapp_smp::bundle::{self, Bundle};
use zapp_smp::settings::{self, EsbConfig};
use zapp_smp::{image, Half, ImageSlot, SmpClient, SmpError, TransportError};
use zapp_wired::ignition::{
    usb::UsbLink, ConfirmMode, ConfirmRc, ConfirmResult, ConfirmState, ConfirmStatus, Halves,
    SessionState, TargetState,
};
use zapp_wired::{
    Ignition, IgnitionLink, LeftTransport, Retrying, RightTransport, Status, WiredError,
};

use crate::new_spinner;
use crate::zsab::{named, short, sizes, Landed};

/// How long to give ignition's entry sequence.
///
/// Its own mailbox poll gives up after 5 s and moves the session to FAULT, so
/// this only has to outlast that — a host that waits longer than the firmware
/// is waiting for nothing.
const BEGIN_TIMEOUT: Duration = Duration::from_secs(15);

/// How long to wait for each reply from a half.
/// How many times the first read after SELECT is tried before the half is
/// called gone, and the pause between tries.
/// How many times one half's write-then-read-back pairing exchange is tried.
/// Both halves of it are idempotent, and a single dropped frame is not a
/// reason to leave a keyboard unprovisioned.
const BOND_ATTEMPTS: u32 = 4;
const BOND_RETRY: Duration = Duration::from_millis(500);

/// Whether a failed run should leave the keyboard exactly as it is, for
/// reading a half's own counters over SWD before anything zeroes them.
fn keep_state_on_failure() -> bool {
    std::env::var("ZAPP_KEEP_STATE_ON_FAILURE").is_ok_and(|v| v != "0" && !v.is_empty())
}

const SELECT_ATTEMPTS: u32 = 3;
const SELECT_RETRY: Duration = Duration::from_millis(250);

const REPLY_TIMEOUT: Duration = Duration::from_secs(15);

/// Reply timeout override, for measuring a half that answers slowly rather
/// than not at all. Seconds; unset or unparsable means [`REPLY_TIMEOUT`].
fn reply_timeout() -> Duration {
    std::env::var("ZAPP_REPLY_TIMEOUT_SECS")
        .ok()
        .and_then(|secs| secs.parse().ok())
        .map(Duration::from_secs)
        .unwrap_or(REPLY_TIMEOUT)
}

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

/// How long an attempt may run before the wait says what it is waiting for.
///
/// The first attempt is the long one: ignition settles the left half and then
/// polls the mailbox twice, eleven seconds of a session that is going nowhere.
/// Announcing the wait once it is clearly a wait, rather than once an attempt
/// has failed, is the difference between a line and a blank terminal.
const RIGHT_SPINNER_AFTER: Duration = Duration::from_secs(1);

/// How long both halves get to swap and come back up before the confirm gives
/// up on them.
///
/// A ceiling rather than a wait: the tunnel asks each half again every
/// [`SETTLE_POLL`] and stops the moment it answers with the image that was
/// written. Swap-using-move carries a 0xb2000 slot through RRAM sector by
/// sector, which is minutes of the seconds-per-sector that made a whole-slot
/// erase a "tens-of-seconds" stall, so the ceiling is deliberately long.
const SETTLE_TIMEOUT: Duration = Duration::from_secs(120);

/// How long the halves are left alone between confirm attempts.
///
/// Nothing is interrupted by asking, unlike the read-back this replaced: a
/// query to a half that is still swapping simply goes unanswered. So this is
/// only about not asking thousands of times over a wait of minutes.
const SETTLE_POLL: Duration = Duration::from_secs(5);

/// How a queued confirm exchange is followed: how often it is asked after, how
/// long an idle answer is given before the request is sent again, and how long
/// the whole thing may take.
#[derive(Debug, Clone, Copy)]
struct Exchange {
    /// Between reads of ignition's confirm result block.
    poll: Duration,
    /// How long the block may stay idle after a CONFIRM before the host
    /// concludes the request never took and sends it again.
    ///
    /// Long enough that a bootloader which queues in its main loop rather than
    /// in the control callback has come round; the repeat is refused with a
    /// stall if it had, and that is read as "held", so a short guess costs
    /// nothing worse than a stalled transfer.
    resend_after: Duration,
    /// How many times a confirm that never took is sent again.
    resends: u32,
    /// How long ignition may report the exchange busy before the host gives up
    /// on it, counted from the last time it was asked and not counting polls
    /// the bus lost.
    ///
    /// Ignition enforces its own 2 s and then reports a timeout, so this only
    /// has to outlast that. It is well past it because the bridge has been
    /// seen to stop — no USB handshake for whole seconds, on a Mac — and come
    /// back; reaching this means it never did.
    limit: Duration,
}

const EXCHANGE: Exchange = Exchange {
    poll: Duration::from_millis(50),
    resend_after: Duration::from_millis(250),
    resends: 2,
    limit: Duration::from_secs(10),
};

/// The image number a recovery upload names the spare slot by.
///
/// `CONFIG_MCUBOOT_SERIAL_DIRECT_IMAGE_UPLOAD` maps 0 and 1 to
/// `slot0_partition` and 2 to `slot1_partition`
/// (`bootloader/mcuboot/boot/zephyr/flash_map_extended.c:129`).
const SPARE_IMAGE: u32 = 2;

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
        reply_timeout(),
        SETTLE_TIMEOUT,
    ) {
        Ok(outcome) => {
            // Nothing was written and nothing was checked, so this one line is
            // the whole of the output. The session is still put back the same
            // way, because BEGIN left both halves in their bootloaders.
            if outcome == Outcome::UpToDate {
                println!("{}", crate::zsab::UP_TO_DATE);
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
                report_failure(err);
            }

            if outcome == Outcome::Flashed {
                println!("Done!");
            }
            Ok(())
        }
        Err(err) => {
            // Leave a keyboard, not a bootloader, on failure too. Nothing
            // unmarked boots any different for it, and a half that was
            // already marked for test reverts now instead of on the next
            // power cycle — either way both halves come back on the build
            // they ran. Best-effort: a half the bus lost cannot hear a reset.
            //
            // Unless someone is debugging: a reset zeroes the halves' own
            // counters, which are the only record of what a failed exchange
            // did. ZAPP_KEEP_STATE_ON_FAILURE=1 leaves everything as it is,
            // bootloader included, so it can be read over SWD afterwards.
            if keep_state_on_failure() {
                println!();
                println!(
                    "ZAPP_KEEP_STATE_ON_FAILURE: both halves left in their bootloaders, \
                     nothing reset."
                );
                report_failure(err);
            }

            if let Err(err) = ignition.release_md() {
                log::debug!("could not release MD after the failure: {err}");
            }
            for half in [Half::Left, Half::Right] {
                if let Err(err) = ignition.reset_target(half) {
                    log::debug!("could not reset the {half} half after the failure: {err}");
                }
            }
            if let Err(err) = ignition.end() {
                log::debug!("could not end the session after the failure: {err}");
            }
            if let Err(err) = ignition.reset_self() {
                log::debug!("the bootloader went down with the reset: {err}");
            }

            report_failure(err);
        }
    }
}

/// Says what went wrong, in one line, and stops.
fn report_failure(err: anyhow::Error) -> ! {
    crate::flash_log::fail(&err);
    eprintln!("Error: {err:#}");
    std::process::exit(1);
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
    let mut deciding = false;
    for half in [Half::Right, Half::Left] {
        steps.announce(&format!("{} half", titled(half)));
        let written = write_half(shared, ignition, half, bundle, &status, force, timeout)
            .map_err(|err| half_gone(half, err))?;
        deciding |= written.staged != Staged::Nothing;
        carried.push(Carried {
            half,
            version: written.version,
            was: written.was,
            hash: None,
        });
    }

    // Both halves are already carrying the bundle and already keeping it, so
    // nothing was uploaded, nothing was staged and there is nothing to decide.
    // Only `--force` writes a half that already matches, so this is unreachable
    // with it.
    if !deciding {
        log::debug!("neither half needed writing or confirming");
        return Ok(Outcome::UpToDate);
    }

    // Out of the session before either application is asked anything. Ignition
    // refuses a confirm while a recovery session is open, because the replies
    // come back on the UART and the TRRS mailbox that session owns — and the
    // halves have to be running their own firmware to answer at all, which is
    // what the resets at the end of `write_half` have just started.
    if let Err(err) = ignition.end() {
        log::debug!("could not end the write session before confirming: {err}");
    }

    // Before `flash` resets the STM32: once it is back in QMK there is no
    // reliable way into ignition again, so this is the last chance to look.
    //
    // ---- Read both, then keep both, or put both back ---------------------
    //
    // One decision for the pair, and the *same* decision the Bluetooth path
    // makes — the two now go through one [`crate::confirm::settle`]. The read
    // writes nothing, so both halves are judged together before either is
    // confirmed: a right that came back and a left that did not never leaves
    // the right confirmed on its own.
    steps.announce("Waiting for both halves to come back on the new firmware");
    steps.announce("Checking both halves came out on the bundle's version");
    let spinner = (!settle.is_zero()).then(|| new_spinner("Waiting for the halves to restart..."));
    let mut pair = WiredPair {
        shared,
        ignition,
        bundle,
        carried,
        timeout,
        settle,
        started: Instant::now(),
        spinner,
        payload_budget: status.payload_budget(),
    };
    let settled =
        crate::confirm::settle(&mut pair, &[Half::Right, Half::Left], &[Half::Right, Half::Left]);
    // Whatever it came to, the line the spinner owns is given back before
    // anything else is said.
    pair.clear();
    settled.map(|()| Outcome::Flashed)
}

// ---------------------------------------------------------------------------
// The confirm/verify pair, over the cable
// ---------------------------------------------------------------------------

/// The wired side of [`crate::confirm::PairConfirm`].
///
/// `running` asks a half what it is executing (mode READ) and writes nothing;
/// `commit` tells it to keep that image (mode COMMIT). Splitting the two is the
/// whole of the fix: both halves are read before either is committed, so
/// nothing is kept on the strength of one half alone. The two reads share one
/// settle window, because the halves swap in parallel.
struct WiredPair<'a, L: IgnitionLink + 'static> {
    shared: &'a Shared<L>,
    ignition: &'a mut Ignition<Shared<L>>,
    bundle: &'a Bundle,
    carried: Vec<Carried>,
    timeout: Duration,
    settle: Duration,
    /// When the shared settle window opened.
    started: Instant,
    /// One spinner for the whole settle, its message swapped as the phase
    /// changes. Nothing else may print while it holds the line.
    spinner: Option<ProgressBar>,
    /// What the write-pass session said one message may carry, for the
    /// connections the revert reboot makes.
    payload_budget: usize,
}

impl<L: IgnitionLink + 'static> crate::confirm::PairConfirm for WiredPair<'_, L> {
    fn running(&mut self, half: Half) -> Landed {
        let wanted = self.bundle.hash(half).unwrap_or_default();
        read_settle(self.ignition, half, &wanted, self.started, self.settle)
    }

    fn mismatch(&mut self, landed: &[(Half, Landed)]) -> Option<String> {
        // The whole-keyboard check, from the hashes the reads reported. Built on
        // a copy of `carried` so a read half's reported hash is the one
        // compared, exactly as the Bluetooth path compares its image-state read.
        let mut carried = self.carried.clone();
        for (half, verdict) in landed {
            if let Some(entry) = carried.iter_mut().find(|entry| entry.half == *half) {
                entry.hash = hash_of(verdict);
            }
        }
        agreed(self.bundle, &carried).err().map(|err| err.to_string())
    }

    fn commit(&mut self, half: Half) -> Result<()> {
        let wanted = self
            .bundle
            .hash(half)
            .with_context(|| format!("The bundle's {half} image carries no hash"))?;
        commit_once(self.ignition, half, &wanted)
    }

    fn revert(&mut self, half: Half) -> Result<()> {
        self.ignition
            .select(half)
            .with_context(|| format!("Could not point the bridge at the {half} half"))?;
        let mut client = connect(self.shared, half, self.payload_budget, self.timeout);
        boot_out(self.ignition, &mut client, half)
    }

    fn read_summary(&mut self, landed: &[(Half, Landed)]) {
        for (half, verdict) in landed {
            if let Landed::Ok { active, .. } = verdict {
                log::debug!("{half}: running {} and keeping it", short(active));
            }
        }
    }

    fn keeping(&mut self) {
        self.saying("Confirming both halves...");
    }

    /// Resets both halves onto the firmware they were running.
    ///
    /// Every image this run wrote was marked for test and only a commit keeps
    /// one, so a reset is the whole of the revert: MCUboot puts the old image
    /// back on the next ordinary boot of a half that never confirmed. And
    /// because the read never wrote a trailer, this is now genuinely true — the
    /// halves are on the images that were written, unconfirmed, so an ordinary
    /// restart reverts them.
    ///
    /// Both halves are pulled back into their bootloaders to do it. The left
    /// could be reset on its own, but the right has no reset line the STM32 can
    /// reach, so the session is re-opened and each half is taken out of it the
    /// same way `write_half` does — the boot MCUboot reverts on. Whatever went
    /// wrong ends up in the error this returns; nothing here is printed on the
    /// way, so the caller's one line is the whole story.
    fn revert_pair(&mut self, landed: Vec<(Half, Landed)>, reasons: Vec<String>) -> anyhow::Error {
        self.clear();
        for (half, verdict) in &landed {
            log::debug!("{half}: {}", verdict.why(*half));
        }

        let summary = reasons.join(" ");
        if let Err(err) = self.reboot_both() {
            log::debug!("could not reopen a session to revert: {err:#}");
            return anyhow!("Flashing failed, the right side did not confirm.");
        }

        let mut restored: Vec<(Half, bool)> = Vec::new();
        let started = Instant::now();
        for half in [Half::Right, Half::Left] {
            let was = self
                .carried
                .iter()
                .find(|entry| entry.half == half)
                .map(|entry| entry.was.clone())
                .unwrap_or_default();
            // Nothing to check against: this end never read what the half had.
            if was.is_empty() {
                restored.push((half, false));
                continue;
            }
            // A READ, not a commit: putting a half back must never write its
            // trailer. It is back where it was only if it is now running `was`.
            let back = read_settle(self.ignition, half, &was, started, self.settle);
            let ok = matches!(&back, Landed::Ok { active, .. } if *active == was);
            restored.push((half, ok));
        }

        if restored.iter().all(|(_, back)| *back) {
            return anyhow!("Flashing failed and reverted to the previous firmware; {summary}");
        }

        let unrestored: Vec<String> = restored
            .iter()
            .filter(|(_, back)| !back)
            .map(|(half, _)| half.to_string())
            .collect();
        anyhow!(
            "Flashing failed, the {} side did not revert; {summary}",
            unrestored.join(" and ")
        )
    }
}

impl<L: IgnitionLink + 'static> WiredPair<'_, L> {
    /// Puts the spinner on `message`, for the phase that is starting.
    ///
    /// A run with no settle window carries no spinner at all, and stays that
    /// way: the tests drive one, and nothing there is waiting for anything.
    fn saying(&mut self, message: &str) {
        if let Some(spinner) = &self.spinner {
            spinner.set_message(message.to_string());
        }
    }

    /// Gives the line back, so something else can be said on it.
    fn clear(&mut self) {
        if let Some(spinner) = self.spinner.take() {
            spinner.finish_and_clear();
        }
    }

    /// Takes both halves through one ordinary boot, right then left.
    ///
    /// The session it opens is what puts them in their bootloaders; the resets
    /// that end it are what boots them again, and that boot is where MCUboot
    /// reverts an image nothing confirmed.
    fn reboot_both(&mut self) -> Result<()> {
        open_session(self.ignition, WAIT_FOR_RIGHT)
            .context("Could not re-open a session to restart the halves")?;

        for half in [Half::Right, Half::Left] {
            <Self as crate::confirm::PairConfirm>::revert(self, half)?;
        }

        // The reads that follow need an idle session, the same as the first one
        // did.
        if let Err(err) = self.ignition.end() {
            log::debug!("could not end the session after restarting the halves: {err}");
        }
        Ok(())
    }
}

/// The hash a verdict says the half is running, if it said one at all.
fn hash_of(verdict: &Landed) -> Option<Vec<u8>> {
    match verdict {
        Landed::Ok { active, .. } | Landed::Wrong { active, .. } if !active.is_empty() => {
            Some(active.clone())
        }
        _ => None,
    }
}

/// One half, read until it answers or the window that opened at `started` runs
/// out.
///
/// A half in the middle of a swap-using-move is in MCUboot, not running
/// anything its application can answer, so the read comes back [`Landed::Gone`]
/// and is worth asking again. Once the half is up it answers on the first ask,
/// however that answer reads, so the retry is only ever for a half still on its
/// way back — a swap-using-move carries a 0xb2000 slot sector by sector, which
/// is minutes, so the window is deliberately long.
fn read_settle<L: IgnitionLink>(
    ignition: &mut Ignition<L>,
    half: Half,
    wanted: &[u8],
    started: Instant,
    settle: Duration,
) -> Landed {
    let deadline = started + settle;
    loop {
        let verdict = read_once(ignition, half, wanted);
        // Anything but Gone is the half having answered; only a half that is not
        // up yet is worth asking again.
        if !matches!(verdict, Landed::Gone { .. }) {
            log::debug!(
                "{half}: read after {} s: {}",
                started.elapsed().as_secs(),
                verdict.why(half)
            );
            return verdict;
        }

        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            log::debug!("{half}: gave up reading: {}", verdict.why(half));
            return verdict;
        }
        log::debug!(
            "{half}: not up yet with {} s left: {}",
            left.as_secs(),
            verdict.why(half)
        );
        std::thread::sleep(SETTLE_POLL.min(left));
    }
}

/// One READ exchange: ask what the half is running, and write nothing.
fn read_once<L: IgnitionLink>(ignition: &mut Ignition<L>, half: Half, wanted: &[u8]) -> Landed {
    if let Err(err) = ignition.confirm_start(half, ConfirmMode::Read, wanted) {
        // A stall here is ignition refusing the request rather than the half
        // refusing anything: a session still open, or a bootloader from before
        // the tunnel existed.
        return Landed::Gone {
            why: format!("The {half} half could not be asked what it is running: {err}."),
        };
    }
    match poll_confirm(ignition, half, ConfirmMode::Read, wanted, EXCHANGE) {
        Ok(result) => read_landed(result, half),
        Err(err) => Landed::Gone {
            why: format!("The {half} half's answer could not be read back: {err}."),
        },
    }
}

/// The exchange-level failure a result carries, if any — ignition's answer
/// rather than the target's, and never anything about what is in flash.
fn exchange_failed(result: &ConfirmResult, half: Half) -> Option<Landed> {
    let why = match result.rc {
        ConfirmRc::Ok => return None,
        ConfirmRc::Timeout => {
            format!("The {half} half did not answer the bootloader within 2 s.")
        }
        ConfirmRc::Transport => format!("The {half} half could not be reached to ask it anything."),
        ConfirmRc::BadReply => {
            format!("The {half} half answered with something this build cannot read.")
        }
        ConfirmRc::Unrecognised(code) => format!(
            "The bootloader ended the {half} half's exchange with code {code}, which this \
             build does not know."
        ),
    };
    Some(Landed::Gone { why })
}

/// What one finished READ means for the half it was asked of.
///
/// The firmware does not compare — it only reports — so a half running the wrong
/// image comes back [`Landed::Ok`] carrying that wrong hash, and the pair check
/// in [`WiredPair::mismatch`] is what catches it, exactly as the Bluetooth path
/// compares the hash `image_state` returns. The version string is left empty
/// throughout: the tunnel names an image by its hash and nothing else.
fn read_landed(result: ConfirmResult, half: Half) -> Landed {
    // Not fatal on its own — one query is in flight at a time, so a stale block
    // can only come from a bootloader that answered the wrong one — but it is
    // not an answer about this half either.
    if result.target != Some(half) {
        return Landed::Gone {
            why: format!("The {half} half's read came back naming another half."),
        };
    }
    if let Some(gone) = exchange_failed(&result, half) {
        return gone;
    }

    match result.status {
        // Running the reported image, not confirmed. The ordinary READ answer.
        ConfirmStatus::Running => Landed::Ok {
            confirmed: false,
            active: result.hash,
        },
        // Running it and already keeping it, from this run or an earlier one.
        ConfirmStatus::Already | ConfirmStatus::Ok => Landed::Ok {
            confirmed: true,
            active: result.hash,
        },
        // It answered, so it is up; it just cannot say what it is running. An
        // empty hash is what [`Landed::Wrong`] words as "reports nothing
        // running", which is exactly the case.
        ConfirmStatus::Unreadable => Landed::Wrong {
            active: Vec::new(),
            version: String::new(),
        },
        // Not expected from a READ, which writes nothing and does not compare,
        // but mapped defensively to the wrong-image case it most resembles.
        ConfirmStatus::Mismatch => Landed::Wrong {
            active: result.hash,
            version: String::new(),
        },
        ConfirmStatus::WriteFail => Landed::Gone {
            why: format!("The {half} half answered a read with a write-failure status."),
        },
        ConfirmStatus::Unrecognised(code) => Landed::Gone {
            why: format!(
                "The {half} half answered its read with status {code:#04x}, which this build \
                 does not know."
            ),
        },
    }
}

/// Reads the result block until ignition has finished with the exchange.
///
/// Ignition times the exchange out itself, at 2 s, so this only guards against
/// a bootloader that stops moving the state on at all.
/// Follows the confirm `ignition.confirm_start(half, mode, wanted)` just queued
/// until ignition reports it done, sending it again if it never took.
///
/// The result block says one of three things and they are kept apart, because
/// they call for different things: `Done` is the answer; `Busy` is the bridge
/// working, to be waited on; `Idle` is the bridge holding no request at all,
/// which after a CONFIRM the host saw acknowledged means the request was lost
/// — and CONFIRM is queued rather than acted on, and refused unless idle, so it
/// is safe to send again. A poll the bus loses says nothing about any of it
/// and is not charged against the wait.
fn poll_confirm<L: IgnitionLink>(
    ignition: &mut Ignition<L>,
    half: Half,
    mode: ConfirmMode,
    wanted: &[u8],
    pacing: Exchange,
) -> Result<ConfirmResult, WiredError> {
    let mut asked = Instant::now();
    let mut deadline = asked + pacing.limit;
    let mut resent = 0;
    let mut last = ConfirmState::Busy;
    loop {
        let polled = Instant::now();
        match ignition.confirm_result() {
            Ok(result) if result.done() => return Ok(result),
            Ok(result) => {
                last = result.state;
                if result.state == ConfirmState::Idle
                    && asked.elapsed() >= pacing.resend_after
                    && resent < pacing.resends
                {
                    resent += 1;
                    log::debug!(
                        "CONFIRM {half} {mode:?} never took: the result block is still idle \
                         {:?} after it was sent; sending it again ({resent} of {})",
                        asked.elapsed(),
                        pacing.resends
                    );
                    match ignition.confirm_start(half, mode, wanted) {
                        Ok(()) => {}
                        // Refused: ignition is holding the first one after
                        // all, so there is nothing to do but keep watching.
                        Err(WiredError::Stalled { .. }) => {
                            log::debug!(
                                "CONFIRM {half} {mode:?}: the repeat was refused, so the \
                                 first one holds"
                            );
                        }
                        Err(err) => return Err(err),
                    }
                    asked = Instant::now();
                    deadline = asked + pacing.limit;
                }
            }
            // No answer is not an answer: a poll the bus lost — even after the
            // link's own retries — says nothing about the exchange, which is
            // still running on the bridge. Ask again, and do not count the time
            // it took against the exchange: the bridge was not answering
            // anyone, so its own clock on the half was not running either. A
            // stall is different: that is ignition refusing the poll, and
            // stays an error.
            Err(WiredError::Faulted { reason, .. }) => {
                log::debug!("CONFIRM_RESULT poll lost on the bus, asking again: {reason}");
                deadline += polled.elapsed();
            }
            Err(err) => return Err(err),
        }
        if Instant::now() >= deadline {
            let reason = match last {
                ConfirmState::Idle => format!(
                    "the bootloader never took the confirm: sent {} times and its result \
                     block stayed idle",
                    resent + 1
                ),
                _ => format!(
                    "the bootloader was still working on the confirm {} s after it was asked",
                    pacing.limit.as_secs()
                ),
            };
            return Err(WiredError::Request {
                request: zapp_wired::ignition::REQ_CONFIRM_RESULT,
                reason,
            });
        }
        std::thread::sleep(pacing.poll);
    }
}

/// One COMMIT exchange: tell the half to keep the image it is running, if that
/// is still the one `wanted` names.
///
/// Only ever reached once both halves' reads have matched the bundle, so a
/// MISMATCH here is a half that changed under us rather than the ordinary case.
fn commit_once<L: IgnitionLink>(
    ignition: &mut Ignition<L>,
    half: Half,
    wanted: &[u8],
) -> Result<()> {
    ignition
        .confirm_start(half, ConfirmMode::Commit, wanted)
        .map_err(|err| anyhow!("The {half} half could not be told to keep its image: {err}"))?;
    let result = poll_confirm(ignition, half, ConfirmMode::Commit, wanted, EXCHANGE)
        .map_err(|err| anyhow!("The {half} half's keep could not be read back: {err}"))?;
    commit_outcome(result, half)
}

/// What one finished COMMIT means: kept, or a clear reason it was not.
///
/// This is the run's last step, and the product owner's residual: if it fails
/// there is nothing more to do but say so.
fn commit_outcome(result: ConfirmResult, half: Half) -> Result<()> {
    if result.target != Some(half) {
        bail!("The {half} half's keep came back naming another half.");
    }
    if let Some(gone) = exchange_failed(&result, half) {
        bail!("{}", gone.why(half));
    }

    match result.status {
        // Kept: this query wrote the trailer, or an earlier one already had.
        ConfirmStatus::Ok | ConfirmStatus::Already => Ok(()),
        ConfirmStatus::Mismatch => {
            bail!("The {half} half is no longer running the image that was written.")
        }
        ConfirmStatus::Unreadable => {
            bail!("The {half} half could not read the hash of what it is running.")
        }
        ConfirmStatus::WriteFail => bail!(
            "The {half} half is running the image that was written and could not keep it: \
             writing its trailer failed."
        ),
        ConfirmStatus::Running => bail!(
            "The {half} half reported it is only running its image, not keeping it."
        ),
        ConfirmStatus::Unrecognised(code) => bail!(
            "The {half} half answered its keep with status {code:#04x}, which this build does \
             not know."
        ),
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
    keep_asking(limit, |waiting| {
        // No END between attempts: `begin` already closes a session it finds
        // in the way, and ending one here would throw away the fault the last
        // attempt recorded — which is what the failure is explained from.
        if let Err(err) = ignition.begin(Halves::BOTH) {
            return Err(NotOpened::fatal(
                anyhow::Error::new(err).context("Could not open a session on the bootloader"),
            ));
        }
        wait_active(ignition, waiting)
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

/// The one line the whole wait is said on, however many attempts it takes.
///
/// Handed to each attempt so the wait can be announced from inside one, which
/// is where the first eleven seconds of it are spent.
#[derive(Default)]
struct Waiting {
    spinner: Option<ProgressBar>,
}

impl Waiting {
    /// Puts the line up, if it is not up already.
    fn show(&mut self) {
        self.spinner.get_or_insert_with(|| {
            new_spinner("Waiting for the right half...")
        });
    }

    /// Gives the line back.
    fn clear(self) {
        if let Some(spinner) = self.spinner {
            spinner.finish_and_clear();
        }
    }
}

/// The retry itself, with the board's side of it passed in.
///
/// Separated from the link so it can be driven without one: what is worth
/// testing here is when it gives up and when it stops, not how a session is
/// opened.
fn keep_asking<F>(limit: Duration, mut attempt: F) -> Result<Status>
where
    F: FnMut(&mut Waiting) -> std::result::Result<Status, NotOpened>,
{
    let started = Instant::now();
    let mut waiting = Waiting::default();
    loop {
        let outcome = attempt(&mut waiting);
        let done = match &outcome {
            Ok(_) => true,
            Err(not_opened) => !not_opened.right_missing || started.elapsed() >= limit,
        };
        if done {
            waiting.clear();
            return outcome.map_err(|not_opened| not_opened.error);
        }
        // An attempt that failed quickly may not have said anything yet.
        waiting.show();
        std::thread::sleep(BETWEEN_ATTEMPTS);
    }
}

/// What one half's turn in the write pass came to.
struct Written {
    /// The version of the slot carrying the bundle's image, which is the spare
    /// one when the half was written because MCUboot has not swapped yet. What
    /// the whole-keyboard check at the end compares.
    version: String,
    staged: Staged,
    /// The hash of what the half was running before any of this, so a revert
    /// has something to check itself against.
    was: Vec<u8>,
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
) -> Result<Written> {
    let wanted = bundle
        .hash(half)
        .with_context(|| format!("The bundle's {half} image carries no hash to check it by"))?;

    // The first exchange after SELECT occasionally comes back "no target is
    // selected": the bridge has not finished pointing at the half when the
    // data lands. Nothing has been written yet, so the select is simply done
    // again. A half that is really gone fails the same way every time.
    let mut attempt = 0;
    let (mut client, before) = loop {
        attempt += 1;
        ignition
            .select(half)
            .with_context(|| format!("Could not point the bridge at the {half} half"))?;

        // Close whatever line the half was left half-way through. Everything
        // on this side of the bridge is flushed by BEGIN and SELECT, but the
        // buffer that matters here is MCUboot's own: a transfer interrupted
        // part-way through a line — Ctrl-C during an upload — leaves it
        // holding a fragment and waiting for the newline that ends it. The
        // next request lands on the back of that fragment and never parses. A
        // bare newline finishes it, and MCUboot discards the nonsense it
        // completes; a half that was not mid-line sees an empty line and
        // ignores it just the same.
        if let Err(err) = ignition.send(half.into(), b"\n") {
            log::debug!("could not send the {half} half a resync newline: {err}");
        }

        let mut client = connect(shared, half, status.payload_budget(), timeout);
        match client.image_state() {
            Ok(before) => break (client, before),
            Err(SmpError::Transport(TransportError::Closed)) if attempt < SELECT_ATTEMPTS => {
                log::debug!("{half}: link closed on the first read, selecting again ({attempt})");
                std::thread::sleep(SELECT_RETRY);
            }
            Err(err) => {
                return Err(err)
                    .with_context(|| format!("Could not read the {half} half's image state"))
            }
        }
    };
    report_slot(&before);

    let was = primary(&before).map(|slot| slot.hash.clone()).unwrap_or_default();
    let mut staged = Staged::Nothing;
    let carried = if installed(&before, &wanted) && !force {
        log::debug!("{half}: already carrying this image, not written again");
        // Not the whole story any more. Since the mark is for test, a half can
        // be carrying the bundle on a boot nothing kept — an earlier run that
        // wrote it and stopped before deciding — and skipping it here would
        // leave it to revert on the next restart. It is not written again, but
        // it is asked to confirm with the rest.
        if primary(&before).is_some_and(|slot| !slot.confirmed) {
            log::debug!("{half}: it is carrying that image on a boot nothing has kept");
            staged = Staged::Unconfirmed;
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

        let image = bundle.image(half);
        upload(&mut client, half, image)?;

        // Read back rather than trust the upload's own success. The bytes went
        // into the spare slot, so that is where they have to show up. Checked
        // before the mark, so a bad upload is reported as the hash mismatch it
        // is rather than as a mark that could not find its image.
        let after = client
            .image_state()
            .with_context(|| format!("Could not read the {half} half's image state back"))?;
        landed(&after, &wanted, half)?;

        // For test, never confirmed. An image confirmed before it has run is an
        // image with no way back, and the confirm tunnel means this run can wait
        // until the half is up and running it before deciding — see the note at
        // the top of this file.
        if let Err(err) = arm(&mut client, half, &wanted) {
            if matches!(err.downcast_ref::<SmpError>(), Some(SmpError::Timeout(_))) {
                probe_silent_mark(shared, ignition, half, status, timeout);
            }
            return Err(err);
        }
        log::debug!(
            "{half}: {} bytes written, spare slot now carries {}",
            image.len(),
            short(&wanted)
        );
        after
    };

    boot_out(ignition, &mut client, half)?;
    Ok(Written {
        version: version_carrying(&carried, &wanted),
        staged,
        was,
    })
}

/// Takes one half out of its bootloader, so it boots its own firmware.
///
/// Both halves are restarted whether or not they were written: ignition's BEGIN
/// put them in their bootloaders, and END does not take them out. How they leave
/// differs, because only the left has a reset line.
fn boot_out<L: IgnitionLink>(
    ignition: &mut Ignition<L>,
    client: &mut SmpClient,
    half: Half,
) -> Result<()> {
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

/// Marks the spare slot for test, and puts up with a reply it cannot read.
///
/// The reply to a mark is a whole slot listing, which MCUboot encodes into one
/// fixed buffer — and arming the swap is what makes that listing too long for
/// it, because the `pending` flag it adds is the last straw on a build with
/// SHA-512 hashes and two slots that both hold an image. So the most ordinary
/// mark on this path answers with a listing that stops mid-way.
///
/// Reading that as a failed mark is wrong, and provably so: MCUboot's `bs_set`
/// answers a refusal with `{"rc": n}` — ten bytes, which always fit and always
/// decode — and only reaches the listing once the trailer is written. A reply
/// that will not decode is therefore a mark that went through. The listing is
/// re-read on the next pass anyway, so nothing is lost by dropping this one.
fn arm(client: &mut SmpClient, half: Half, wanted: &[u8]) -> Result<()> {
    match client.mark(wanted, false) {
        Ok(_) => Ok(()),
        Err(SmpError::Cbor(detail)) => {
            log::debug!(
                "{half}: the mark went through and its listing did not fit the board's \
                 buffer ({detail})"
            );
            Ok(())
        }
        Err(err) => Err(err)
            .with_context(|| format!("Could not mark the {half} half's new image for test")),
    }
}

/// Looks at a half whose mark went unanswered: hung, restarted, or answered
/// into the void. Logs only; every step's failure is logged and dropped.
fn probe_silent_mark<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    half: Half,
    status: &Status,
    timeout: Duration,
) {
    log::warn!("{half}: no reply to the mark, probing");
    match ignition.status() {
        Ok(status) => log::warn!("{half}: ignition status after the silent mark: {status:?}"),
        Err(err) => log::warn!("{half}: could not read ignition status: {err}"),
    }

    if let Err(err) = ignition.send(half.into(), b"\n") {
        log::warn!("{half}: could not send a resync newline: {err}");
    }
    let mut client = connect(shared, half, status.payload_budget(), timeout);
    match client.image_state() {
        Ok(slots) if slots.is_empty() => log::warn!("{half}: image state lists no slots"),
        Ok(slots) => {
            for slot in &slots {
                log::warn!(
                    "{half}: slot {} carries {} ({}) [{}]",
                    slot.slot,
                    short(&slot.hash),
                    named(&slot.version),
                    slot.flags().join(" ")
                );
            }
            let pending = slots.iter().any(|slot| slot.slot == 1 && slot.pending);
            log::warn!("{half}: slot 1 pending: {pending}");
        }
        Err(err) => log::warn!("{half}: image state read failed too: {err}"),
    }

    match ignition.status() {
        Ok(status) => log::warn!("{half}: ignition status after the probe: {status:?}"),
        Err(err) => log::warn!("{half}: could not read ignition status: {err}"),
    }
}

/// What a half has outstanding when its turn in the write pass is over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Staged {
    /// An image is waiting in the spare slot to swap in, which is the wait the
    /// confirm sits through.
    Image,
    /// Already carrying the bundle, on a boot nothing has kept: no swap to wait
    /// for, and still a confirm to make.
    Unconfirmed,
    /// Carrying the bundle and keeping it. Nothing left to do to this half.
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
    waiting: &mut Waiting,
) -> std::result::Result<Status, NotOpened> {
    let started = Instant::now();
    loop {
        let status = match ignition.status() {
            Ok(status) => status,
            // The same as a poll that has not seen the state move yet: the
            // bus lost this one, the entry sequence is still running, and the
            // deadline below is what bounds the wait.
            Err(WiredError::Faulted { reason, .. }) if started.elapsed() < BEGIN_TIMEOUT => {
                log::debug!("STATUS poll lost on the bus, asking again: {reason}");
                continue;
            }
            Err(err) => {
                return Err(NotOpened::fatal(
                    anyhow::Error::new(err).context("Could not read ignition's status"),
                ));
            }
        };
        match status.state {
            SessionState::Active => return Ok(status),
            SessionState::Fault => {
                return Err(NotOpened {
                    right_missing: right_missing(status.error),
                    error: anyhow!(
                        "The bootloader could not open a session (error {:#06x})",
                        status.error
                    ),
                });
            }
            SessionState::Idle => {
                return Err(NotOpened::fatal(anyhow!(
                    "The bootloader reports no session open after BEGIN."
                )));
            }
            SessionState::Pending => {
                // The entry sequence is the long part of the wait, so the wait
                // is announced from here rather than after the attempt fails.
                if started.elapsed() >= RIGHT_SPINNER_AFTER {
                    waiting.show();
                }
            }
            SessionState::Unrecognised(code) => {
                return Err(NotOpened::fatal(anyhow!(
                    "The bootloader reports session state {code}, which this build does not know."
                )));
            }
        }
        if started.elapsed() >= BEGIN_TIMEOUT {
            return Err(NotOpened::fatal(anyhow!(
                "The bootloader was still opening the session {} s after BEGIN.",
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

/// Sends one half's image into its spare slot, with a bar on it.
fn upload(client: &mut SmpClient, half: Half, image: &[u8]) -> Result<()> {
    let bar = crate::flash_bar(100, titled(half));
    let result = client.upload(SPARE_IMAGE, image, &|progress| match progress {
        FlashProgress::Writing {
            bytes_written,
            total_bytes,
        } => {
            bar.set_position(((bytes_written * 100) / total_bytes) as u64);
            bar.set_message("Writing...");
        }
        FlashProgress::Complete => bar.finish_with_message("Done!"),
        _ => {}
    });
    if let Err(err) = result {
        // The bar owns its line until something finishes it.
        bar.finish_and_clear();
        return Err(err).with_context(|| format!("The {half} half's upload failed"));
    }
    Ok(())
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
         carries."
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

/// Fails unless the spare slot the board reports is the image the bundle
/// carries.
///
/// The hash is the whole of it. A hash only appears in MCUboot's listing for an
/// image it has read and verified, so the bundle's hash in the spare slot is
/// the upload having landed whole on something signed.
///
/// Only the spare slot, and only straight after the upload: the half is still
/// sitting in the recovery that wrote it and the image has not swapped yet.
/// What the half is running once it has is not a question for a bootloader any
/// more — [`confirm_both`] asks the firmware itself.
fn landed(slots: &[ImageSlot], wanted: &[u8], half: Half) -> Result<()> {
    let lead = format!("The {half} half took the whole image and then reported");
    // A slot MCUboot cannot validate is left out of its listing (or listed
    // without a hash): the whole image arrived and the bootloader refused it.
    let rejected = format!("the {half} half rejected the image: bad signature or corrupt file");
    let Some(slot) = secondary(slots) else {
        bail!("{rejected}");
    };
    if slot.hash.is_empty() {
        bail!("{rejected}");
    }
    if slot.hash != wanted {
        bail!(
            "{lead} {} in its spare slot, not the {} that was sent.",
            short(&slot.hash),
            short(wanted),
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

/// A half's link went away while it was being written — the TRRS cable came
/// out, or the mailbox behind it did.
///
/// Its own type, and one carrying no source, so the failure epilogue prints
/// this sentence and nothing else. What the chain underneath says is bridges,
/// mailboxes and event codes, which is the truth from the inside and no help
/// at all to someone holding a keyboard with a cable half out.
#[derive(Debug)]
struct HalfGone(Half);

impl std::fmt::Display for HalfGone {
    fn fmt(&self, out: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(out, "The {} half stopped responding", self.0)
    }
}

impl std::error::Error for HalfGone {}

/// Turns a write pass that lost its link into [`HalfGone`], and leaves every
/// other failure as it is.
fn half_gone(half: Half, err: anyhow::Error) -> anyhow::Error {
    let closed = err
        .chain()
        .filter_map(|link| link.downcast_ref::<SmpError>())
        .any(|smp| matches!(smp, SmpError::Transport(TransportError::Closed)));
    if !closed {
        return err;
    }
    log::debug!("{half} half link closed: {err:#}");
    anyhow::Error::new(HalfGone(half))
}

/// What one half came out of the run carrying.
#[derive(Clone)]
struct Carried {
    half: Half,
    /// What its primary slot reported while it was being written.
    version: String,
    /// The hash its running firmware answered the confirm with. `None` when it
    /// never answered.
    hash: Option<Vec<u8>>,
    /// What it was running before any of this, for a revert to check itself
    /// against.
    was: Vec<u8>,
}

/// Fails unless both halves and the bundle name one build.
///
/// Each half's confirm proves that half is running the image it was sent, and
/// nothing so far has put the two answers next to each other. A run that
/// wrote one half and left the other on what it had is a keyboard on two
/// builds, whose halves cannot talk over the wire between them, and it is not
/// a success however well each half went on its own.
///
/// Both the version strings and the hashes the halves reported, so a pair that
/// disagrees is caught by identity rather than only by what it calls itself.
fn agreed(bundle: &Bundle, carried: &[Carried]) -> Result<()> {
    let wanted = wanted_version(bundle);
    let reading = ordering_of(&wanted);

    let mut found: Vec<String> = Vec::new();
    for entry in carried {
        let half = entry.half;
        let wrong_hash = match (&entry.hash, bundle.hash(half)) {
            // A prefix, not the whole thing: the tunnel names an image by the
            // leading 32 bytes of a hash MCUboot publishes 64 of.
            (Some(hash), Some(bundled)) => hash.is_empty() || !bundled.starts_with(hash),
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
        "The halves are on different builds: {}.",
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
/// A separate question from the confirm, and still worth asking after it. The
/// confirm proves each half is executing the image that was written; this
/// proves the *keyboard* works: the left's reading only reaches the host once
/// its UART link to QMK is up, and the right's crossed the wire between the
/// halves to be served to a QMK telemetry read. Both answering is both halves
/// alive and talking to each other on the build that was just written — which
/// no query to one half on its own can show.
///
/// Required rather than advisory. Both images are confirmed by the time this
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
/// Deliberately not "this update was undone": by the time this runs both halves
/// have said they are executing the bundle and both have confirmed it, so they
/// are on it whatever happens next. What has not been shown is that the
/// keyboard works, and the difference matters to someone deciding what to do
/// about it.
fn unproven() -> String {
    "Both halves kept the update but the keyboard did not come back on USB with both halves \
     reporting."
        .to_string()
}

/// Whether the keyboard came back on its own firmware, with both halves in it.
///
/// The confirm before this talked to each half's application, so this is no
/// longer the first thing in the run that reaches running firmware — but it is
/// the only thing that reaches it *through the keyboard*: the right half's
/// answer crosses the wire between the halves and is served to a QMK telemetry
/// read, so getting both is both applications up, the STM32 back on its own
/// firmware, and the two halves talking to each other.
fn came_back(limit: Duration) -> bool {
    let spinner = new_spinner("Waiting for the keyboard to restart...");
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

    let Some((telemetry, _, _)) = crate::usb_telemetry(keyboard, false) else {
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
// Printing
// ---------------------------------------------------------------------------

fn announce(bundle: &Bundle, _path: &Path, _len: usize) {
    println!(
        "Bundle loaded: {} ({}) flashing over USB",
        named(&bundle.header.build_version),
        sizes(bundle)
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
        Err(WiredError::AccessDenied(err)) => {
            // Plugged in, and either not opened up by udev yet or never will
            // be. Both are the cable: the wired path waits out the first and
            // names the second.
            log::debug!("a bootloader is present but refused to open: {err}");
            true
        }
        Err(err) => {
            // Something is plugged in that cannot be opened. Say so rather
            // than silently going over the radio, where the same board would
            // work and the cause would stay hidden.
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
///
/// A bootloader that refuses to open is waited for too, for
/// [`ACCESS_GRACE`](zapp_core::device::access::ACCESS_GRACE)
/// from the first refusal: right after a reset that is udev not having opened
/// the node up yet.
fn open_waiting(limit: Duration) -> Result<Retrying<UsbLink>> {
    let deadline = Instant::now() + limit;
    let mut grace = AccessGrace::default();

    match UsbLink::open() {
        Ok(link) => {
            log::debug!("found the ZSA bootloader over USB");
            return Ok(link);
        }
        Err(WiredError::NoBootloader) => {}
        Err(WiredError::AccessDenied(err)) => {
            grace.retry(Instant::now());
            log::debug!("the bootloader refused to open ({err}), trying again");
        }
        Err(err) => return Err(err).context("Could not open the ZSA bootloader"),
    }

    let spinner = new_spinner("Waiting for keyboard in bootloader mode...");

    loop {
        match UsbLink::open() {
            Ok(link) => {
                spinner.finish_and_clear();
                log::debug!("found the ZSA bootloader over USB");
                return Ok(link);
            }
            Err(WiredError::NoBootloader) if Instant::now() < deadline => {
                grace.reset();
                std::thread::sleep(Duration::from_millis(250));
            }
            Err(WiredError::NoBootloader) => {
                spinner.finish_and_clear();
                bail!("No bootloader appeared in {} seconds.", limit.as_secs())
            }
            Err(WiredError::AccessDenied(err))
                if grace.retry(Instant::now()) && Instant::now() < deadline =>
            {
                log::debug!("the bootloader refused to open ({err}), trying again");
                std::thread::sleep(ACCESS_RETRY);
            }
            Err(WiredError::AccessDenied(err)) => {
                spinner.finish_and_clear();
                return Err(access_denied(&err));
            }
            Err(err) => {
                spinner.finish_and_clear();
                return Err(err).context("Could not open the ZSA bootloader");
            }
        }
    }
}

/// The error for a bootloader that is plugged in and stayed closed to this
/// user.
fn access_denied(cause: &str) -> anyhow::Error {
    if cfg!(target_os = "linux") {
        anyhow!(
            "The ZSA bootloader is plugged in, but this user is not allowed to open it \
             ({cause}). The udev rule for ZSA devices (vendor 3297) is missing or has not \
             been applied. It should read:\n\n    \
             SUBSYSTEM==\"usb\", ATTR{{idVendor}}==\"3297\", MODE:=\"0666\"\n\n\
             Install it under /etc/udev/rules.d/, then run `sudo udevadm control \
             --reload-rules && sudo udevadm trigger` and replug the keyboard."
        )
    } else {
        anyhow!(
            "The ZSA bootloader is plugged in, but this user is not allowed to open it \
             ({cause})."
        )
    }
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

    fn control_out_data(
        &mut self,
        request: u8,
        value: u16,
        data: &[u8],
    ) -> Result<(), WiredError> {
        self.0.lock().unwrap().control_out_data(request, value, data)
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

// ---------------------------------------------------------------------------
// Bonding: the ESB pairing key onto both halves
// ---------------------------------------------------------------------------

/// Provisions a pairing key onto both halves over the cable.
///
/// Generates a fresh key unless one is given as 60 hex characters. Both halves
/// take a byte-identical bundle — there is no role field, and the pair links
/// only when both carry the same one. Each half's key is written and read back,
/// then the run ends the way [`flash`] does, by watching the keyboard come back
/// on USB with both halves answering — which shows both rebooted into their
/// applications. It does not show the radio link: the left disables its ESB
/// client while it has USB power, so on USB the right's telemetry comes over
/// the TRRS cable.
pub fn bond(key: Option<&str>) -> Result<()> {
    let config = match key {
        Some(text) => EsbConfig::parse_hex(text)
            .map_err(|err| anyhow!("that is not a usable pairing key: {err}"))?,
        None => EsbConfig::generate(),
    };

    match key {
        Some(_) => println!("Using the pairing key you supplied."),
        None => println!("Generated a fresh pairing key."),
    }
    describe_config(&config);

    let shared = Shared::new(open()?);
    let mut ignition = Ignition::new(shared.clone());

    match bond_run(&shared, &mut ignition, &config, reply_timeout()) {
        Ok(()) => {
            // Both halves have the bundle and read it back; put the session
            // away and let each reboot into its application to reload it.
            end_and_reset(&mut ignition);
            prove_bond(came_back(CONFIRM_WINDOW));
            Ok(())
        }
        Err(err) => bond_failed(&mut ignition, err),
    }
}

/// Reads both halves' pairing keys and reports whether they match, writing
/// nothing.
///
/// The support command for "are these two halves even paired": it never puts a
/// key on the wire, only reads each half's `esb/config` back and prints the two
/// fingerprints and whether they agree. A half that was never provisioned reads
/// back as such rather than as a failure.
pub fn bond_show() -> Result<()> {
    let shared = Shared::new(open()?);
    let mut ignition = Ignition::new(shared.clone());

    match bond_show_run(&shared, &mut ignition, REPLY_TIMEOUT) {
        Ok(report) => {
            // A read-only look still opened a session, which left both halves
            // in their bootloaders; boot them back the same way.
            end_and_reset(&mut ignition);
            print_show(&report);
            Ok(())
        }
        Err(err) => bond_failed(&mut ignition, err),
    }
}

/// Writes both halves and reads each back, the right half first.
fn bond_run<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    config: &EsbConfig,
    timeout: Duration,
) -> Result<()> {
    let status = open_session(ignition, WAIT_FOR_RIGHT)?;
    log::debug!(
        "session active: TRRS {} kHz, {} bytes per message, left {}, right {}",
        status.i2c_khz,
        status.max_payload,
        running(status.left_state),
        running(status.right_state)
    );

    // The right half first, for the same reason the flash has it: the right is
    // the half that can fail to come back at all, and finding that out before
    // the left is touched leaves a keyboard with one provisioned half rather
    // than a session that will not reopen.
    let mut readbacks: Vec<(Half, Vec<u8>)> = Vec::new();
    for half in [Half::Right, Half::Left] {
        let readback = provision_half(shared, ignition, half, config, &status, timeout)?;
        readbacks.push((half, readback));
    }

    // Both halves take the same bundle, so two halves that read back different
    // bytes are a keyboard whose pair could never link. Refuse it here rather
    // than reboot into a mismatch — the per-half retry already had its go.
    let right = &readbacks[0].1;
    let left = &readbacks[1].1;
    if !keys_agree(left, right) {
        bail!(
            "The two halves read back different pairing keys (left {}, right {}).",
            settings::fingerprint_hex(left),
            settings::fingerprint_hex(right)
        );
    }
    Ok(())
}

/// Points the bridge at one half, writes the bundle, reads it back, and reboots
/// the half into its application.
///
/// One retry: a single line dropped on the TRRS bus is worth trying again
/// before the run is given up, and the read-back is what catches it. The half
/// is rebooted out of recovery whether the write took or not, so a failure does
/// not leave it sitting in the bootloader.
fn provision_half<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    half: Half,
    config: &EsbConfig,
    status: &Status,
    timeout: Duration,
) -> Result<Vec<u8>> {
    ignition
        .select(half)
        .with_context(|| format!("Could not point the bridge at the {half} half"))?;
    // Finish any line MCUboot was left holding, the way the flash does.
    if let Err(err) = ignition.send(half.into(), b"\n") {
        log::debug!("could not send the {half} half a resync newline: {err}");
    }

    let mut client = connect(shared, half, status.payload_budget(), timeout);

    // The last attempt's read-back when it was a mismatch rather than a failed
    // exchange, which is what decides how the run ends.
    let mut mismatch: Option<Vec<u8>> = None;
    let outcome = bond_attempts(ignition, half, |attempt| {
        let exchange = client
            .settings_write(settings::ESB_CONFIG_NAME, config.as_bytes())
            .with_context(|| format!("Could not write the {half} half's pairing key"))
            .and_then(|()| {
                client
                    .settings_read(settings::ESB_CONFIG_NAME)
                    .with_context(|| format!("Could not read the {half} half's pairing key back"))
            });
        let readback = match exchange {
            Ok(readback) => readback,
            Err(err) => {
                mismatch = None;
                return Attempt::Again(err);
            }
        };

        if readback.as_slice() == config.as_bytes().as_slice() {
            log::debug!("{half}: wrote and read back the pairing key");
            return Attempt::Done(readback);
        }
        log::warn!(
            "{half}: read back {} not {}, on attempt {attempt}",
            settings::fingerprint_hex(&readback),
            settings::hex(&config.key_fingerprint())
        );
        mismatch = Some(readback);
        Attempt::Again(anyhow!("the {half} half read back a different pairing key"))
    });

    match (outcome, mismatch) {
        (Ok(readback), _) => {
            reboot_half(ignition, &mut client, half)?;
            let label = format!("{} half", titled(half));
            println!(
                "  {label:<10}: wrote and read back {} bytes.",
                settings::ESB_CONFIG_LEN
            );
            Ok(readback)
        }
        (Err(err), None) => Err(err),
        (Err(_), Some(readback)) => {
            // Boot it out of recovery even on failure, then report.
            let _ = reboot_half(ignition, &mut client, half);
            bail!(
                "The {half} half read back a different pairing key than was written, \
                 {BOND_ATTEMPTS} times (got {}).",
                settings::fingerprint_hex(&readback)
            )
        }
    }
}

/// One attempt's answer inside [`bond_attempts`].
enum Attempt<T> {
    Done(T),
    Again(anyhow::Error),
}

/// Runs one exchange with a half up to [`BOND_ATTEMPTS`] times.
///
/// A dropped request or reply fails one exchange, not the command. Attempts
/// after the first send the resync newline that finishes a line the half is
/// still holding, then wait; the last attempt's error is returned. Late replies
/// from an earlier attempt are dropped by the client, which only takes the
/// sequence number it asked for.
fn bond_attempts<L: IgnitionLink + 'static, T>(
    ignition: &mut Ignition<Shared<L>>,
    half: Half,
    mut exchange: impl FnMut(u32) -> Attempt<T>,
) -> Result<T> {
    let mut last: Option<anyhow::Error> = None;
    for attempt in 1..=BOND_ATTEMPTS {
        if attempt > 1 {
            if let Err(err) = ignition.send(half.into(), b"\n") {
                log::debug!("could not send the {half} half a resync newline: {err}");
            }
            std::thread::sleep(BOND_RETRY);
        }
        match exchange(attempt) {
            Attempt::Done(value) => return Ok(value),
            Attempt::Again(err) => {
                log::debug!("{half}: bond exchange failed on attempt {attempt}: {err:#}");
                last = Some(err);
            }
        }
    }
    Err(last.expect("BOND_ATTEMPTS is at least one"))
}

/// Reads both halves' pairing keys, writing nothing.
fn bond_show_run<L: IgnitionLink + 'static>(
    shared: &Shared<L>,
    ignition: &mut Ignition<Shared<L>>,
    timeout: Duration,
) -> Result<ShowReport> {
    let status = open_session(ignition, WAIT_FOR_RIGHT)?;

    let mut keys: Vec<(Half, HalfKey)> = Vec::new();
    for half in [Half::Right, Half::Left] {
        ignition
            .select(half)
            .with_context(|| format!("Could not point the bridge at the {half} half"))?;
        if let Err(err) = ignition.send(half.into(), b"\n") {
            log::debug!("could not send the {half} half a resync newline: {err}");
        }
        let mut client = connect(shared, half, status.payload_budget(), timeout);

        // The left half intermittently drops the tail of the first frame after
        // SELECT, so the read gets the same retry the write path has.
        let read = bond_attempts(ignition, half, |_| {
            match client.settings_read(settings::ESB_CONFIG_NAME) {
                Ok(value) => Attempt::Done(HalfKey::Provisioned(value)),
                // ENOENT: the whitelisted setting was never written. Not a
                // failure — a half running on the development fallback key,
                // and an answer, so it is not retried.
                Err(SmpError::Remote { rc: 5, .. }) => Attempt::Done(HalfKey::NotProvisioned),
                Err(err) => Attempt::Again(anyhow::Error::new(err)),
            }
        });
        let key = match read {
            Ok(key) => key,
            Err(err) => {
                let _ = reboot_half(ignition, &mut client, half);
                return Err(err)
                    .with_context(|| format!("Could not read the {half} half's pairing key"));
            }
        };
        reboot_half(ignition, &mut client, half)?;
        keys.push((half, key));
    }

    Ok(ShowReport {
        right: keys.remove(0).1,
        left: keys.remove(0).1,
    })
}

/// Reboots one half out of recovery and into its application, the way the flash
/// does: the right through its own SMP reset, the left by releasing the MD pin
/// and resetting it. The right also points the bridge back at the left, so no
/// poll lands on a chip that is mid-reboot.
fn reboot_half<L: IgnitionLink + 'static>(
    ignition: &mut Ignition<Shared<L>>,
    client: &mut SmpClient,
    half: Half,
) -> Result<()> {
    match half {
        Half::Right => {
            client
                .reset()
                .context("Could not restart the right half out of MCUboot")?;
            ignition
                .select(Half::Left)
                .context("Could not point the bridge away from the right half")?;
        }
        Half::Left => {
            ignition
                .release_md()
                .context("Could not release the left half's MD pin")?;
            ignition
                .reset_target(Half::Left)
                .context("Could not restart the left half")?;
        }
    }
    Ok(())
}

/// Whether two read-back bundles are the same, so the pair can link.
fn keys_agree(left: &[u8], right: &[u8]) -> bool {
    left == right
}

/// Puts the session away and reboots the STM32 out of its own recovery, best
/// effort — both are on the way out and a failure here says nothing useful.
fn end_and_reset<L: IgnitionLink>(ignition: &mut Ignition<Shared<L>>) {
    if let Err(err) = ignition.end() {
        log::debug!("could not end the session: {err}");
    }
    if let Err(err) = ignition.reset_self() {
        log::debug!("the bootloader went down with the reset: {err}");
    }
}

/// The failure epilogue shared by both bond commands, worded like the flash's.
fn bond_failed<L: IgnitionLink>(ignition: &mut Ignition<Shared<L>>, err: anyhow::Error) -> ! {
    if let Err(err) = ignition.end() {
        log::debug!("could not end the session after the failure: {err}");
    }
    report_failure(err)
}

/// Says what was written, and never the key: the AES key reaches neither a log
/// nor a screen, so a fingerprint stands in for it.
fn describe_config(config: &EsbConfig) {
    println!("  {:<15} {}", "channel", config.rf_channel());
    println!(
        "  {:<15} {}:{}",
        "address",
        settings::hex(config.base0()),
        settings::hex(&[config.prefix0()])
    );
    println!("  {:<15} {}", "device id", settings::hex(config.device_id()));
    println!(
        "  {:<15} {}",
        "key fingerprint",
        settings::hex(&config.key_fingerprint())
    );
}

/// What the restart check's answer means for a bond.
///
/// The bytes are on both halves and read back before this runs, so nothing here
/// undoes the provisioning. What it shows is that both halves rebooted into
/// their applications and answer, not that the radio link is up — that is only
/// visible on battery with no cables. A run that does not see both halves come
/// back exits non-zero, the way the flash's does.
fn prove_bond(came_back: bool) {
    if came_back {
        println!();
        println!(
            "The pairing key was written and read back on both halves; both halves answered \
             after the restart."
        );
    } else {
        println!();
        println!("{}", unproven_bond());
        std::process::exit(1);
    }
}

/// What a bond whose halves did not come back on USB says.
fn unproven_bond() -> String {
    "The pairing key was written and read back on both halves, but the keyboard did not come \
     back on USB with both halves answering."
        .to_string()
}

/// One half's pairing key, as `--show` found it.
#[derive(Debug, Clone, PartialEq, Eq)]
enum HalfKey {
    Provisioned(Vec<u8>),
    /// ENOENT: the setting was never written.
    NotProvisioned,
}

/// What `--show` read from both halves.
#[derive(Debug, Clone, PartialEq, Eq)]
struct ShowReport {
    right: HalfKey,
    left: HalfKey,
}

/// Whether a keyboard's two halves are paired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MatchState {
    /// Both provisioned, same key.
    Paired,
    /// Both provisioned, different keys.
    Mismatched,
    /// At least one half is not provisioned, so there is no pair to judge.
    Incomplete,
}

impl ShowReport {
    fn match_state(&self) -> MatchState {
        match (&self.left, &self.right) {
            (HalfKey::Provisioned(left), HalfKey::Provisioned(right)) => {
                if keys_agree(left, right) {
                    MatchState::Paired
                } else {
                    MatchState::Mismatched
                }
            }
            _ => MatchState::Incomplete,
        }
    }
}

/// The line one half's key reads as, fingerprint only.
fn show_line(name: &str, key: &HalfKey) -> String {
    let label = format!("{name} half");
    match key {
        HalfKey::Provisioned(bytes) => {
            format!("  {label:<10}: {}", settings::fingerprint_hex(bytes))
        }
        HalfKey::NotProvisioned => format!("  {label:<10}: not provisioned"),
    }
}

/// The verdict line, from whether the two halves' keys agree.
fn show_verdict(state: MatchState) -> String {
    match state {
        MatchState::Paired => "The two halves carry the same pairing key.".to_string(),
        MatchState::Mismatched => "The two halves carry different pairing keys.".to_string(),
        MatchState::Incomplete => "At least one half is not provisioned.".to_string(),
    }
}

fn print_show(report: &ShowReport) {
    println!("{}", show_line("Left", &report.left));
    println!("{}", show_line("Right", &report.right));
    println!("{}", show_verdict(report.match_state()));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zsab::{decide, Decision};

    // ---- The confirm poll ------------------------------------------------

    /// A link whose CONFIRM_RESULT block is scripted: idle for the first
    /// `idle_polls` polls and until `takes` CONFIRMs have been accepted, then
    /// busy for `busy` polls, then done — or busy forever.
    #[derive(Default)]
    struct ScriptedConfirm {
        takes: u32,
        idle_polls: u32,
        busy: u32,
        busy_forever: bool,
        /// Refuse any CONFIRM after the first with a stall, the way ignition
        /// does while it holds one — and stop reading idle from then on.
        refuses_repeats: bool,
        /// Fault the first `faults` polls, the way a lost bus does.
        faults: u32,
        /// CONFIRMs sent, refused ones included.
        starts: u32,
        /// CONFIRMs accepted.
        taken: u32,
        polls: u32,
        /// Busy polls still to answer once the idle stretch is over.
        busy_left: Option<u32>,
    }

    impl IgnitionLink for ScriptedConfirm {
        fn control_out(&mut self, _r: u8, _v: u16) -> std::result::Result<(), WiredError> {
            Ok(())
        }
        fn control_out_data(
            &mut self,
            request: u8,
            _v: u16,
            _d: &[u8],
        ) -> std::result::Result<(), WiredError> {
            assert_eq!(request, ignition::REQ_CONFIRM);
            self.starts += 1;
            if self.refuses_repeats && self.starts > 1 {
                // Refused because the bridge is holding the first one: from
                // here the block reads busy, not idle.
                self.idle_polls = 0;
                return Err(WiredError::Stalled { request });
            }
            self.taken += 1;
            Ok(())
        }
        fn control_in(
            &mut self,
            request: u8,
            _v: u16,
            length: usize,
        ) -> std::result::Result<Vec<u8>, WiredError> {
            assert_eq!(request, ignition::REQ_CONFIRM_RESULT);
            if self.faults > 0 {
                self.faults -= 1;
                return Err(WiredError::Faulted {
                    request,
                    reason: "the device did not respond".into(),
                });
            }
            self.polls += 1;
            let mut block = vec![0u8; length];
            block[3] = 1; // left
            block[0] = if self.taken < self.takes || self.polls <= self.idle_polls {
                0 // idle
            } else {
                self.busy_left = self.busy_left.or(Some(self.busy));
                match self.busy_left {
                    _ if self.busy_forever => 1, // busy
                    Some(0) => 2,                // done
                    Some(left) => {
                        self.busy_left = Some(left - 1);
                        1 // busy
                    }
                    None => unreachable!(),
                }
            };
            Ok(block)
        }
        fn bulk_out(&mut self, _m: &[u8]) -> std::result::Result<(), WiredError> {
            unreachable!("the confirm tunnel never touches the bulk pipe")
        }
        fn bulk_in(&mut self, _t: Duration) -> std::result::Result<Vec<u8>, WiredError> {
            unreachable!("the confirm tunnel never touches the bulk pipe")
        }
    }

    const FAST: Exchange = Exchange {
        poll: Duration::from_millis(1),
        resend_after: Duration::from_millis(20),
        resends: 2,
        limit: Duration::from_millis(200),
    };

    type Polled = (std::result::Result<ConfirmResult, WiredError>, ScriptedConfirm);

    fn poll(link: ScriptedConfirm) -> Polled {
        let mut ignition = Ignition::new(link);
        ignition
            .confirm_start(Half::Left, ConfirmMode::Commit, &[0x5a; 32])
            .unwrap();
        let result =
            poll_confirm(&mut ignition, Half::Left, ConfirmMode::Commit, &[0x5a; 32], FAST);
        (result, ignition.into_link())
    }

    #[test]
    fn a_confirm_that_never_took_is_sent_again_and_then_answers() {
        // The block stays idle after the first CONFIRM — the request was lost
        // — and the second one takes.
        let (result, link) = poll(ScriptedConfirm {
            takes: 2,
            ..ScriptedConfirm::default()
        });
        assert!(result.unwrap().done());
        assert_eq!(link.starts, 2);
    }

    #[test]
    fn a_repeat_ignition_refuses_means_the_first_one_holds() {
        // The block read idle just before the bridge moved to busy: the repeat
        // stalls, which is not an error, and the exchange is followed to its
        // end.
        let (result, link) = poll(ScriptedConfirm {
            takes: 1,
            idle_polls: u32::MAX,
            refuses_repeats: true,
            busy: 5,
            ..ScriptedConfirm::default()
        });
        assert!(result.unwrap().done());
        assert_eq!(link.starts, 2, "sent once, repeated once, and the repeat was refused");
        assert_eq!(link.taken, 1);
    }

    #[test]
    fn a_confirm_nothing_takes_says_so_once_the_repeats_are_spent() {
        let (result, link) = poll(ScriptedConfirm {
            takes: u32::MAX,
            ..ScriptedConfirm::default()
        });
        let err = result.unwrap_err();
        assert!(err.to_string().contains("never took"), "{err}");
        assert_eq!(link.starts, 1 + FAST.resends);
    }

    #[test]
    fn a_confirm_that_stays_busy_says_so_and_is_not_sent_again() {
        let (result, link) = poll(ScriptedConfirm {
            takes: 1,
            busy_forever: true,
            ..ScriptedConfirm::default()
        });
        let err = result.unwrap_err();
        assert!(err.to_string().contains("still working"), "{err}");
        assert_eq!(link.starts, 1, "a busy bridge holds the request; it is not repeated");
    }

    #[test]
    fn polls_the_bus_loses_are_asked_again_and_the_exchange_still_lands() {
        let (result, link) = poll(ScriptedConfirm {
            takes: 1,
            faults: 3,
            busy: 2,
            ..ScriptedConfirm::default()
        });
        assert!(result.unwrap().done());
        assert_eq!(link.starts, 1);
        assert_eq!(link.polls, 3);
    }
    use zapp_core::device::BootloaderKind;
    use zapp_smp::fake::FakeSlot;
    use zapp_smp::{client, img};
    use zapp_wired::ignition;
    use zapp_wired::loopback::{ConfirmAnswer, LoopbackDevice, LoopbackLink};
    use zapp_wired::pipe;

    /// The zephyr basic group, which carries the storage-erase command this
    /// flash deliberately does not send.
    const ZEPHYR_BASIC_GROUP: u16 = 63;

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
    /// after each half's upload. Nothing is read after the restart any more —
    /// the halves are asked what they are running instead.
    fn a_write_pass() -> Vec<Vec<FakeSlot>> {
        vec![
            settled(&DEVICE_HASH, &[]),
            staged(&DEVICE_HASH, &BUNDLE_HASH),
            settled(&DEVICE_HASH, &[]),
            staged(&DEVICE_HASH, &BUNDLE_HASH),
        ]
    }

    /// The same, on a keyboard whose halves both answer their confirm with the
    /// bundle's hash — a pair that installed and came back.
    fn a_whole_run(device: &LoopbackDevice) {
        device.tweak(|fake| fake.listings = a_write_pass());
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
        flash_loopback_setup(&blob, false, a_whole_run)
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
        // One read per half, which is the read that decided it.
        assert_eq!(count(&device, (img::GROUP, img::CMD_STATE)), 2);
    }

    #[test]
    fn a_keyboard_that_already_carries_the_bundle_stops_before_the_confirm() {
        // Nothing was staged and both halves are already keeping what they
        // carry, so there is nothing to swap and nothing to decide: a settle
        // wait would be spent on a keyboard that was never touched.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, control, device) = flash_loopback(&blob);
        assert_eq!(result.unwrap(), Outcome::UpToDate);

        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            1,
            "a second session was opened on a keyboard nothing was written to: {control:?}"
        );
        assert!(device.confirms().is_empty(), "{:?}", device.confirms());
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
        assert!(err.contains("not the"), "{err}");
    }

    #[test]
    fn a_written_half_stages_into_image_two_and_marks_it_for_test() {
        let (result, _control, device) = flash_loopback_written();
        result.expect("both halves swapped");

        // Image 2 is slot1_partition under direct image upload. Image 0 or 1
        // would be the slot the half is running.
        assert_eq!(device.uploads(), vec![SPARE_IMAGE, SPARE_IMAGE]);
        // For test, so a pair that does not come out of this right still has a
        // way back. What keeps the image is the confirm that follows.
        assert_eq!(
            device.marks(),
            vec![(BUNDLE_HASH.to_vec(), false), (BUNDLE_HASH.to_vec(), false)]
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
        // (`arm` marks for test; the flag is asserted at the bottom.)
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
        // And the mark really went out, for test.
        assert_eq!(
            marks.lock().unwrap().clone(),
            vec![(BUNDLE_HASH.to_vec(), false)]
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
    fn a_half_whose_link_went_away_is_one_sentence_with_nothing_under_it() {
        // What the cable coming out mid-upload actually reaches `flash` as:
        // the bridge's event, flattened to a closed transport by zapp-wired and
        // wrapped in the upload's own context.
        let err = anyhow::Error::new(SmpError::Transport(TransportError::Closed))
            .context("The right half's upload failed");
        let err = half_gone(Half::Right, err);
        assert_eq!(err.to_string(), "The right half stopped responding");
        assert!(err.downcast_ref::<HalfGone>().is_some());

        // Every other failure is left alone.
        let other = half_gone(Half::Right, anyhow::anyhow!("the board refused the mark"));
        assert_eq!(other.to_string(), "the board refused the mark");
    }

    #[test]
    fn a_permanent_mark_is_never_sent_over_the_cable() {
        // Separately from the whole-run assertion above, because this is the
        // property and not the sequence: nothing on this path may commit to an
        // image before it has been seen running, which is exactly what a
        // permanent mark does.
        let (_result, _control, device) = flash_loopback_written();
        assert!(
            device.marks().iter().all(|(_, permanent)| !*permanent),
            "an image was armed permanently: {:?}",
            device.marks()
        );
    }

    #[test]
    fn both_halves_are_asked_what_they_are_running_and_a_match_keeps_it() {
        let (result, control, device) = flash_loopback_written();
        assert_eq!(result.expect("both halves came back on the bundle"), Outcome::Flashed);

        // One session, not two: the halves are asked through the tunnel while
        // they run their own firmware, so nothing pulls them back into their
        // bootloaders to look.
        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            1,
            "the halves were re-entered after being written: {control:?}"
        );
        // Right first, and each named by the image that was written to it —
        // read once to see what it is running, then committed to keep it.
        let both = vec![
            (Half::Right, BUNDLE_HASH.to_vec()),
            (Half::Left, BUNDLE_HASH.to_vec()),
        ];
        assert_eq!(device.reads(), both, "the halves were not read first");
        assert_eq!(device.commits(), both, "the halves were not committed after");
    }

    #[test]
    fn a_half_already_carrying_the_bundle_on_a_boot_nothing_kept_is_confirmed_not_skipped() {
        // The hole a test mark opens: an earlier run wrote both halves and
        // stopped before deciding, so the pair is running the bundle
        // unconfirmed and reverts on its next restart. Re-running has to
        // finish it rather than call it up to date.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &DEVICE_HASH));
        let (result, _control, device) = flash_loopback_setup(&blob, false, |device| {
            // Active, and nothing confirming it: BOOT_SWAP_TYPE_REVERT.
            device.tweak(|fake| fake.listings = vec![reverting(&DEVICE_HASH, &BUNDLE_HASH)])
        });

        assert_eq!(result.expect("an undecided pair is finished, not skipped"), Outcome::Flashed);
        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0, "it was written again");
        // Read as running the bundle unconfirmed, then committed to keep it.
        let both = vec![
            (Half::Right, DEVICE_HASH.to_vec()),
            (Half::Left, DEVICE_HASH.to_vec()),
        ];
        assert_eq!(device.reads(), both);
        assert_eq!(device.commits(), both);
    }

    #[test]
    fn a_half_running_something_else_puts_both_halves_back() {
        // What a swap that did not complete looks like from the tunnel: the
        // half is up and reports the image it had, which the pair check catches.
        // Neither half is committed — and that is the fix, read both before
        // committing either.
        let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
        let (result, control, device) = flash_loopback_setup(&blob, false, |device| {
            a_whole_run(device);
            device.answers_confirm(
                Half::Left,
                ConfirmAnswer {
                    hash: Some(DEVICE_HASH.to_vec()), // running the image it had
                    ..ConfirmAnswer::default()
                },
            );
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("Flashing failed"), "{err}");
        assert!(err.contains("The halves are on different builds"), "{err}");
        // Nothing was kept: the left came back wrong, so the pair reverts and no
        // commit is ever sent — not even to the right, which read back fine.
        assert!(device.commits().is_empty(), "a half was committed: {:?}", device.commits());
        // Both halves were pulled back into their bootloaders and booted again,
        // which is the boot MCUboot reverts an unconfirmed image on.
        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            2,
            "the halves were not restarted to revert them: {control:?}"
        );
        // Two reads for the pair, then two more naming what they had, on the
        // revert — and no commit among them.
        let asked: Vec<Vec<u8>> = device.reads().into_iter().map(|(_, hash)| hash).collect();
        assert_eq!(
            asked,
            vec![
                BUNDLE_HASH.to_vec(),
                BUNDLE_HASH.to_vec(),
                DEVICE_HASH.to_vec(),
                DEVICE_HASH.to_vec(),
            ]
        );
    }

    #[test]
    fn a_half_that_never_answers_puts_both_halves_back() {
        // The right half is the one that can go away entirely. Nothing is kept
        // on the strength of the left alone.
        let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
        let (result, _control, device) = flash_loopback_setup(&blob, false, |device| {
            a_whole_run(device);
            device.answers_confirm(
                Half::Right,
                ConfirmAnswer {
                    rc: 1, // the target never answered
                    ..ConfirmAnswer::default()
                },
            );
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("did not answer the bootloader"), "{err}");
        // The right never answered, so the left is never committed either — read
        // both before committing either.
        assert!(device.commits().is_empty(), "a half was committed: {:?}", device.commits());
        // The left was reached and matched, and is still put back: a pair or
        // nothing.
        assert!(
            device.reads().iter().any(|(half, hash)| *half == Half::Left
                && *hash == DEVICE_HASH.to_vec()),
            "the left was not asked about its old image: {:?}",
            device.reads()
        );
    }

    #[test]
    fn neither_half_is_committed_until_both_have_been_read() {
        // The bug this whole change fixes: the wired path used to read-and-commit
        // each half in one step, so a right half that was gone still left the
        // left committed on its own. Now a read that comes back wrong on either
        // half must stop any commit reaching the other. Both directions:
        for gone in [Half::Right, Half::Left] {
            let blob = bundled(&signed(0x11, &BUNDLE_HASH), &signed(0x22, &BUNDLE_HASH));
            let (result, _control, device) = flash_loopback_setup(&blob, false, |device| {
                a_whole_run(device);
                // This half never answers; the other reads back perfectly.
                device.answers_confirm(
                    gone,
                    ConfirmAnswer {
                        rc: 1,
                        ..ConfirmAnswer::default()
                    },
                );
            });

            assert!(result.is_err(), "a missing {gone} half was not a failure");
            // Both halves were read (the settle reads come first, before the
            // revert's own re-reads)...
            let reads = device.reads();
            assert!(
                reads.iter().take(2).any(|(half, _)| *half == Half::Left)
                    && reads.iter().take(2).any(|(half, _)| *half == Half::Right),
                "both halves were not read first: {reads:?}"
            );
            // ...and neither was committed, whichever one was missing.
            assert!(
                device.commits().is_empty(),
                "a half was committed with the {gone} half gone: {:?}",
                device.commits()
            );
        }
    }

    #[test]
    fn an_update_never_erases_the_right_halfs_settings() {
        // The right's settings partition holds `esb/config`, the inter-half
        // pairing key, provisioned once and read at every boot. An update
        // writes the image slot and nothing else.
        let blob = bundled(&signed(0x11, &DEVICE_HASH), &signed(0x22, &[0xb2; 32]));
        let (result, _control, device) = flash_loopback(&blob);
        assert!(result.is_err(), "the loopback cannot report a landed hash");

        let requests = device.requests();
        assert!(
            requests
                .iter()
                .all(|(group, _)| *group != ZEPHYR_BASIC_GROUP),
            "{requests:?}"
        );
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
        // the left's own turn begins. One pass now, because the confirm needs
        // no session of its own.
        assert_eq!(selects, vec![2, 1, 1]);
    }

    #[test]
    fn the_write_pass_is_one_session_and_the_confirms_come_after_it_is_ended() {
        let (result, control, _device) = flash_loopback_written();
        result.unwrap();

        // Every vendor request, in order, with the polls dropped: STATUS and
        // the confirm result reads are polls and their counts are not part of
        // the sequence.
        let sequence: Vec<(u8, u16)> = control
            .into_iter()
            .filter(|(request, _)| {
                *request != ignition::REQ_STATUS && *request != ignition::REQ_CONFIRM_RESULT
            })
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
                // The session is ended before either half is asked anything:
                // ignition refuses a confirm while one owns the UART and the
                // TRRS mailbox the replies come back on.
                (ignition::REQ_END, 0),
                // Both halves are READ first (mode 0, so the value is the plain
                // half number), right then left — and only once both reads are
                // in are both COMMITTED (mode 1 in the high byte of the value),
                // right then left. Read-both-before-commit-either.
                (ignition::REQ_CONFIRM, 0x0002),
                (ignition::REQ_CONFIRM, 0x0001),
                (ignition::REQ_CONFIRM, 0x0102),
                (ignition::REQ_CONFIRM, 0x0101),
                // And `flash`'s own END on the way out.
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
        assert_eq!(result.unwrap(), Outcome::UpToDate);

        assert_eq!(count(&device, (img::GROUP, img::CMD_UPLOAD)), 0);
        assert!(
            control.contains(&(ignition::REQ_RESET_TARGET, 1)),
            "the left half was not reset: {control:?}"
        );
        // One pass, so one reset.
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
        assert!(err.contains("older than the v9.9.9"), "{err}");
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
        // post-upload check correctly disbelieves it.
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
            1,
            "expected one SMP reset for the right half"
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
        // release for the one pass, and it is the left's.
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

    // ---- The tunnel's READ and COMMIT answers, on their own -------------

    /// A finished result block, as ignition hands it over.
    fn answered(rc: u8, status: u8, target: u8, hash: &[u8]) -> ConfirmResult {
        let mut bytes = vec![0u8; 36];
        bytes[0] = 2; // done
        bytes[1] = rc;
        bytes[2] = status;
        bytes[3] = target;
        bytes[4..4 + hash.len()].copy_from_slice(hash);
        ConfirmResult::parse(&bytes).unwrap()
    }

    /// A READ verdict for the right half, which is 2.
    fn verdict(rc: u8, status: u8, hash: &[u8]) -> Landed {
        read_landed(answered(rc, status, 2, hash), Half::Right)
    }

    #[test]
    fn a_read_that_reports_the_image_running_is_ok_and_still_unconfirmed() {
        // RUNNING is the ordinary READ answer: up, running the reported image,
        // not yet keeping it — which is what the commit that follows is for. The
        // firmware does not compare on a READ, so whatever hash it names comes
        // back verbatim and the pair check judges it, exactly as the Bluetooth
        // path judges the hash `image_state` returns.
        assert_eq!(
            verdict(0, 0x05, &BUNDLE_HASH),
            Landed::Ok {
                confirmed: false,
                active: BUNDLE_HASH.to_vec(),
            }
        );
        assert_eq!(
            verdict(0, 0x05, &DEVICE_HASH),
            Landed::Ok {
                confirmed: false,
                active: DEVICE_HASH.to_vec(),
            }
        );
    }

    #[test]
    fn a_read_of_a_half_already_confirmed_is_ok_and_needs_no_commit() {
        // ALREADY: running the image and already keeping it, so settle sends it
        // no commit.
        assert_eq!(
            verdict(0, 0x01, &BUNDLE_HASH),
            Landed::Ok {
                confirmed: true,
                active: BUNDLE_HASH.to_vec(),
            }
        );
    }

    #[test]
    fn a_half_that_cannot_read_its_own_hash_reports_nothing_running() {
        // It answered, so it is up — it just cannot say what it is executing,
        // which is not the same as being unreachable and not the same as
        // carrying the wrong thing.
        let verdict = verdict(0, 0x03, &[]);
        assert_eq!(
            verdict,
            Landed::Wrong {
                active: Vec::new(),
                version: String::new(),
            }
        );
        assert!(verdict.why(Half::Right).contains("reports nothing running"), "{verdict:?}");
    }

    #[test]
    fn every_way_the_exchange_itself_fails_is_a_half_that_could_not_be_asked() {
        // Ignition's own codes, not the target's: none of them says anything
        // about what is in flash, so none of them may keep an image.
        for (rc, phrase) in [
            (1u8, "did not answer the bootloader"),
            (2, "could not be reached"),
            (3, "cannot read"),
            (9, "does not know"),
        ] {
            let verdict = verdict(rc, 0xFF, &[]);
            assert!(matches!(verdict, Landed::Gone { .. }), "rc {rc}: {verdict:?}");
            assert!(verdict.why(Half::Right).contains(phrase), "rc {rc}");
        }
    }

    #[test]
    fn a_read_status_from_a_newer_firmware_is_not_guessed_at() {
        let verdict = verdict(0, 0x7f, &BUNDLE_HASH);
        assert!(matches!(verdict, Landed::Gone { .. }), "{verdict:?}");
        assert!(verdict.why(Half::Right).contains("0x7f"));
    }

    #[test]
    fn a_read_naming_another_half_is_not_an_answer_about_this_one() {
        // One query is in flight at a time, so this can only be a bootloader
        // answering the wrong one — which is exactly why it must not be read
        // as this half being fine.
        let verdict = read_landed(answered(0, 0x05, 1, &BUNDLE_HASH), Half::Right);
        assert!(matches!(verdict, Landed::Gone { .. }), "{verdict:?}");
        assert!(verdict.why(Half::Right).contains("naming another half"));
    }

    #[test]
    fn a_commit_the_half_accepts_or_had_already_accepted_is_kept() {
        // OK is this query having written the trailer, ALREADY an earlier one
        // having done it. Either keeps the image.
        for status in [0x00u8, 0x01] {
            commit_outcome(answered(0, status, 2, &BUNDLE_HASH), Half::Right)
                .unwrap_or_else(|err| panic!("status {status:#04x}: {err}"));
        }
    }

    #[test]
    fn a_commit_the_half_refuses_is_a_clear_error_each_way() {
        for (status, phrase) in [
            (0x02u8, "no longer running the image that was written"),
            (0x03, "could not read the hash"),
            (0x04, "writing its trailer failed"),
            (0x05, "only running its image, not keeping it"),
            (0x7f, "does not know"),
        ] {
            let err = commit_outcome(answered(0, status, 2, &BUNDLE_HASH), Half::Right)
                .expect_err("a status that kept nothing was read as kept");
            assert!(format!("{err}").contains(phrase), "status {status:#04x}: {err}");
        }
    }

    #[test]
    fn a_commit_whose_exchange_never_completed_is_an_error() {
        let err = commit_outcome(answered(1, 0xFF, 2, &[]), Half::Right)
            .expect_err("a target that never answered kept nothing");
        assert!(format!("{err}").contains("did not answer the bootloader"), "{err}");
        // And a reply about the other half is not trusted.
        let other = commit_outcome(answered(0, 0x00, 1, &BUNDLE_HASH), Half::Right)
            .expect_err("a reply naming another half was trusted");
        assert!(format!("{other}").contains("naming another half"), "{other}");
    }

    #[test]
    fn only_a_verdict_that_names_a_hash_is_recorded_against_the_half() {
        // `agreed` reads these, and a half that never said what it is running
        // must leave nothing behind rather than an empty hash that would read
        // as a mismatch.
        assert_eq!(
            hash_of(&Landed::Ok {
                confirmed: true,
                active: BUNDLE_HASH.to_vec()
            }),
            Some(BUNDLE_HASH.to_vec())
        );
        assert_eq!(
            hash_of(&Landed::Wrong {
                active: DEVICE_HASH.to_vec(),
                version: String::new()
            }),
            Some(DEVICE_HASH.to_vec())
        );
        assert_eq!(
            hash_of(&Landed::Wrong {
                active: Vec::new(),
                version: String::new()
            }),
            None
        );
        assert_eq!(hash_of(&Landed::Gone { why: "nothing".into() }), None);
    }

    #[test]
    fn one_half_short_of_perfect_is_a_revert_for_the_pair() {
        // The decision itself is `zsab`'s and tested there; this pins that the
        // wired path reaches it with the same three shapes, so a right that
        // confirmed and a left that did not is not a keeper.
        let kept = |half| {
            (
                half,
                Landed::Ok {
                    confirmed: true,
                    active: BUNDLE_HASH.to_vec(),
                },
            )
        };
        assert_eq!(
            decide(&[kept(Half::Right), kept(Half::Left)], None),
            Decision::Keep
        );

        let mixed = [
            kept(Half::Right),
            (Half::Left, verdict(1, 0xFF, &[])),
        ];
        let Decision::Revert { reasons } = decide(&mixed, None) else {
            panic!("one half unreachable is not a keeper");
        };
        assert_eq!(reasons.len(), 1);
        assert!(reasons[0].contains("did not answer"), "{reasons:?}");

        // And a pair that both answered but on two builds is a revert too.
        assert!(matches!(
            decide(
                &[kept(Half::Right), kept(Half::Left)],
                Some("the halves disagree".into())
            ),
            Decision::Revert { .. }
        ));
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
        assert!(landed(&[kept(0, &[0x99; 32]), slot(1, &wanted)], &wanted, Half::Left).is_ok());
        assert!(landed(&[kept(0, &wanted)], &wanted, Half::Left).is_err());

        let wrong = landed(
            &[kept(0, &[0x11; 32]), slot(1, &[0x99; 32])],
            &wanted,
            Half::Right,
        )
        .unwrap_err()
        .to_string();
        assert!(wrong.contains("right"), "{wrong}");
        assert!(wrong.contains("not the"), "{wrong}");

        // A slot MCUboot could not verify reports no hash at all (or is left
        // out of the listing): the bootloader rejected what arrived.
        let empty = landed(&[slot(1, &[])], &wanted, Half::Left)
            .unwrap_err()
            .to_string();
        assert!(empty.contains("bad signature or corrupt file"), "{empty}");

        let missing = landed(&[], &wanted, Half::Left)
            .unwrap_err()
            .to_string();
        assert!(missing.contains("bad signature or corrupt file"), "{missing}");
    }

    #[test]
    fn a_half_carrying_the_image_without_a_confirm_is_still_carrying_it() {
        // What is in the slot and whether the half is keeping it are two
        // questions, and this one only reads the first: an unconfirmed half is
        // not written again, it is confirmed. Which is the difference between
        // `Staged::Unconfirmed` and `Staged::Image`.
        let wanted = [0xa1; 32];
        assert!(installed(&[slot(0, &wanted)], &wanted));
        assert!(installed(&[kept(0, &wanted)], &wanted));
    }

    /// A status the way the bootloader answers one that faulted.
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

    // ---- Waiting for the right half ------------------------------------

    /// An attempt that fails `misses` times with the given fault before the
    /// session opens, and counts how many times it was asked.
    fn attempts_until(misses: usize, error: u16) -> (Result<Status>, usize) {
        let mut asked = 0;
        let result = keep_asking(Duration::from_secs(30), |_| {
            asked += 1;
            if asked > misses {
                return Ok(faulted(0, 0, 0));
            }
            let status = faulted(error, 0, 0);
            Err(NotOpened {
                right_missing: right_missing(status.error),
                error: anyhow!("the bootloader faulted ({:#06x})", status.error),
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
        let result = keep_asking(Duration::ZERO, |_| {
            asked += 1;
            Err(NotOpened {
                right_missing: true,
                error: anyhow!("the bootloader faulted (0x0002)"),
            })
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("0x0002"), "{err}");
        assert_eq!(asked, 1, "it kept asking past its own limit");
    }

    #[test]
    fn the_wait_outlives_the_attempt_that_said_it() {
        // An attempt announces the wait long before it fails, so the line has
        // to survive into the attempts after it, and asking for it again is a
        // no-op rather than a second line.
        let mut asked = 0;
        let mut lines = Vec::new();
        let result = keep_asking(Duration::from_secs(30), |waiting| {
            asked += 1;
            waiting.show();
            waiting.show();
            lines.push(waiting.spinner.is_some());
            if asked > 2 {
                return Ok(faulted(0, 0, 0));
            }
            Err(NotOpened {
                right_missing: true,
                error: anyhow!("no right half"),
            })
        });

        result.expect("the third attempt found it");
        assert_eq!(lines, [true, true, true]);
    }

    // ---- One version across the keyboard -------------------------------

    #[test]
    fn a_keyboard_whose_halves_disagree_is_not_a_success() {
        // Both halves take the bundle's images and answer their confirm with
        // them, so every check up to here passed — and the run still has to
        // fail, because the version they came out on is not the one the bundle
        // names.
        let blob = bundled_v(
            &signed(0x11, &BUNDLE_HASH),
            &signed(0x22, &BUNDLE_HASH),
            "0.1.9",
        );
        let (result, _control, _device) = flash_loopback_setup(&blob, false, a_whole_run);

        let err = format!("{:#}", result.unwrap_err());
        // A mismatch is a revert case like any other: two halves on two builds
        // cannot talk to each other, so neither is kept.
        assert!(err.contains("Flashing failed"), "{err}");
        assert!(
            err.contains("The halves are on different builds"),
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

    /// One half's answer at the end of a run, with no hash reported.
    fn on_version(half: Half, version: &str) -> Carried {
        Carried {
            half,
            version: version.to_string(),
            hash: None,
            was: Vec::new(),
        }
    }

    /// The same, with the hash the half answered its confirm with.
    fn carrying(half: Half, version: &str, hash: &[u8]) -> Carried {
        Carried {
            half,
            version: version.to_string(),
            hash: Some(hash.to_vec()),
            was: Vec::new(),
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
        assert!(text.contains("The halves are on different builds"), "{text}");
        assert!(text.contains("the right half is carrying"), "{text}");
        assert!(!text.contains("the left half"), "{text}");
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
        // It used to be a note under a successful run. It cannot be: both
        // images are confirmed by now, so nothing takes a bad one away on its
        // own, and this is the only thing in the run that watches the keyboard
        // itself work.
        let err = finish(Outcome::Flashed, || false)
            .expect_err("a keyboard that never came back is not a finished update");
        let said = format!("{err}");
        assert!(said.contains("did not come back on USB"), "{said}");
        // And it says the images were kept rather than about to disappear:
        // over the cable nothing reverts.
        assert!(said.contains("kept the update"), "{said}");
        assert!(!said.contains("old firmware back"), "{said}");
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

    // -----------------------------------------------------------------------
    // Bonding
    // -----------------------------------------------------------------------

    /// A fixed, valid pairing bundle: channel 42, base0 1..4, prefix 5, and a
    /// distinct key so its fingerprint is stable across runs.
    fn a_config() -> EsbConfig {
        let mut bytes = [0u8; settings::ESB_CONFIG_LEN];
        for (i, byte) in bytes.iter_mut().enumerate() {
            *byte = i as u8;
        }
        bytes[0] = 42; // rf_channel
        EsbConfig::from_bytes(bytes).expect("a valid fixed bundle")
    }

    /// Runs `bond_run` against the in-memory keyboard.
    fn bond_loopback(
        config: &EsbConfig,
        setup: impl FnOnce(&LoopbackDevice),
    ) -> (Result<()>, Vec<(u8, u16)>, LoopbackDevice) {
        let (link, device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        setup(&device);
        let shared = Shared::new(link);
        let mut ignition = Ignition::new(shared.clone());
        let result = bond_run(&shared, &mut ignition, config, Duration::from_millis(50));
        if result.is_ok() {
            ignition.end().unwrap();
        }
        let control = shared.0.lock().unwrap().control.clone();
        (result, control, device)
    }

    /// Runs `bond_show_run` against the in-memory keyboard.
    fn show_loopback(setup: impl FnOnce(&LoopbackDevice)) -> (Result<ShowReport>, LoopbackDevice) {
        let (link, device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        setup(&device);
        let shared = Shared::new(link);
        let mut ignition = Ignition::new(shared.clone());
        let result = bond_show_run(&shared, &mut ignition, Duration::from_millis(50));
        if result.is_ok() {
            ignition.end().unwrap();
        }
        (result, device)
    }

    #[test]
    fn bonding_writes_the_same_bundle_to_both_halves_and_reads_it_back() {
        let config = a_config();
        let (result, _control, device) = bond_loopback(&config, |_| {});
        result.expect("both halves provisioned");

        // The value that ended up on the board is exactly the bundle.
        assert_eq!(device.esb_config().as_deref(), Some(config.as_bytes().as_slice()));
        // One write per half, no retries, and both carried the whole bundle.
        let writes = device.setting_writes();
        assert_eq!(writes.len(), 2, "a half was written more than once");
        assert!(writes.iter().all(|value| value == config.as_bytes()));
    }

    #[test]
    fn bonding_reads_each_half_back_over_group_three() {
        let (result, _control, device) = bond_loopback(&a_config(), |_| {});
        result.unwrap();
        // Per half: one write and one read on the settings group.
        let settings_ops = device
            .requests()
            .iter()
            .filter(|(group, command)| *group == settings::GROUP && *command == settings::CMD_RW)
            .count();
        assert_eq!(settings_ops, 4, "expected a write and a read on each half");
    }

    #[test]
    fn bonding_opens_one_session_and_reboots_both_halves() {
        let (result, control, device) = bond_loopback(&a_config(), |_| {});
        result.unwrap();
        // One BEGIN: the whole thing is one session.
        assert_eq!(
            control
                .iter()
                .filter(|(request, _)| *request == ignition::REQ_BEGIN)
                .count(),
            1,
            "{control:?}"
        );
        // The right half is rebooted out of MCUboot with its own SMP reset.
        assert_eq!(device.resets(), 1, "the right half was not rebooted");
    }

    #[test]
    fn a_half_that_will_not_take_the_write_is_retried_then_fails() {
        // The board acks the write but keeps its old value — a dropped line —
        // so the read-back never matches. Retried, then the run gives up.
        let preset = vec![0x77u8; settings::ESB_CONFIG_LEN];
        let (result, _control, device) = bond_loopback(&a_config(), |device| {
            device.tweak(|fake| {
                fake.esb_config = Some(preset.clone());
                fake.esb_config_frozen = true;
            })
        });

        let err = format!("{:#}", result.unwrap_err());
        assert!(err.contains("read back a different pairing key"), "{err}");
        assert!(err.contains("4 times"), "{err}");
        // The right half is tried first, and every attempt writes once before
        // the run is given up — the left is never reached.
        assert_eq!(
            device.setting_writes().len(),
            BOND_ATTEMPTS as usize,
            "one write per attempt"
        );
    }

    #[test]
    fn a_dropped_pairing_exchange_is_retried_rather_than_failing_the_command() {
        // The first exchange with each half goes missing, the way a dropped
        // request or reply looks from here. Provisioning is idempotent, so the
        // run should carry on and come out paired.
        let (result, _control, device) = bond_loopback(&a_config(), |device| {
            device.tweak(|fake| fake.drop_first_settings_exchange = true)
        });
        // Without the retry this is the error case; with it, both halves come
        // out provisioned, each by the attempt that got through.
        assert!(result.is_ok(), "{:#}", result.unwrap_err());
        assert_eq!(device.setting_writes().len(), 2, "one landed write per half");
    }

    #[test]
    fn a_dropped_show_read_is_retried_rather_than_failing_the_command() {
        // The first read goes missing, the way a half that drops the tail of
        // the frame after SELECT looks from here. The read is idempotent, so
        // the next attempt should get the answer.
        let bundle = a_config().as_bytes().to_vec();
        let (result, _device) = show_loopback(|device| {
            device.tweak(|fake| {
                fake.esb_config = Some(bundle.clone());
                fake.drop_first_settings_exchange = true;
            })
        });
        let report = result.expect("a read-only look");
        assert_eq!(report.right, HalfKey::Provisioned(bundle.clone()));
        assert_eq!(report.left, HalfKey::Provisioned(bundle));
    }

    #[test]
    fn show_reads_both_halves_and_calls_a_matching_pair_paired() {
        let bundle = a_config().as_bytes().to_vec();
        let (result, _device) = show_loopback(|device| {
            device.tweak(|fake| fake.esb_config = Some(bundle.clone()))
        });
        let report = result.expect("a read-only look");
        assert_eq!(report.match_state(), MatchState::Paired);
        assert_eq!(report.left, HalfKey::Provisioned(bundle.clone()));
        assert_eq!(report.right, HalfKey::Provisioned(bundle));
    }

    #[test]
    fn show_writes_nothing() {
        let bundle = a_config().as_bytes().to_vec();
        let (result, device) = show_loopback(|device| {
            device.tweak(|fake| fake.esb_config = Some(bundle.clone()))
        });
        result.unwrap();
        assert!(device.setting_writes().is_empty(), "--show wrote to a half");
    }

    #[test]
    fn show_reports_a_half_that_was_never_provisioned_rather_than_failing() {
        // esb_config left as None, so the board answers ENOENT.
        let (result, _device) = show_loopback(|_| {});
        let report = result.expect("ENOENT is not a failure on --show");
        assert_eq!(report.left, HalfKey::NotProvisioned);
        assert_eq!(report.right, HalfKey::NotProvisioned);
        assert_eq!(report.match_state(), MatchState::Incomplete);
    }

    #[test]
    fn the_match_verdict_says_whether_the_two_keys_agree() {
        // Two halves cannot disagree over the single in-memory device, so the
        // verdict is checked on the values directly.
        let mismatched = ShowReport {
            left: HalfKey::Provisioned(vec![0x01; settings::ESB_CONFIG_LEN]),
            right: HalfKey::Provisioned(vec![0x02; settings::ESB_CONFIG_LEN]),
        };
        assert_eq!(mismatched.match_state(), MatchState::Mismatched);
        assert!(show_verdict(MatchState::Mismatched).contains("different pairing keys"));
        assert!(show_verdict(MatchState::Paired).contains("same pairing key"));
        assert!(show_verdict(MatchState::Incomplete).contains("not provisioned"));
    }

    #[test]
    fn a_show_line_carries_a_fingerprint_and_never_the_key() {
        let config = a_config();
        let line = show_line("Left", &HalfKey::Provisioned(config.as_bytes().to_vec()));
        // The fingerprint, not the 30 bytes.
        assert!(line.contains(&settings::fingerprint_hex(config.as_bytes())));
        // None of the AES-key bytes spelled out in full.
        let key_hex = settings::hex(&config.as_bytes()[14..30]);
        assert!(!line.contains(&key_hex), "the key leaked into the line: {line}");
    }

    #[test]
    fn identical_bundles_agree_and_different_ones_do_not() {
        let config = a_config();
        assert!(keys_agree(config.as_bytes(), config.as_bytes()));
        let mut other = *config.as_bytes();
        other[14] ^= 0xff;
        assert!(!keys_agree(config.as_bytes(), &other));
    }

    #[test]
    fn the_unproven_bond_message_says_what_did_not_happen() {
        let said = unproven_bond();
        assert!(said.contains("read back on both halves"), "{said}");
        assert!(said.contains("did not come back"), "{said}");
        assert!(!said.contains("run bond again"), "{said}");
    }
}
