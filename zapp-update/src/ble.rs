//! Updating a whole keyboard over Bluetooth: a `.zsab` bundle onto both nRFs,
//! or a keyboard image onto the STM32.
//!
//! Everything under this is already there: `zapp_ble::smp` has the client, the
//! upload loop and the relay that reaches the right half. This is the order to
//! do it in, what to check first, and what to say when it goes wrong — which is
//! the part a person needs, because the keyboard is unusable for the couple of
//! minutes it takes and a failure half-way leaves the two halves on different
//! firmware.
//!
//! There is no downgrade guard on this path, and there never was: a bundle
//! older than what a half carries is written like any other.
//! [`crate::guard::going_backwards`] takes the same image-state listing this
//! path reads, should that change.

use std::cell::RefCell;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};

use zapp_ble::dfu::{self, Wait as Readiness};
use zapp_ble::smp::bundle::{self, Bundle, Step};
use zapp_ble::smp::session::{self, Declaration, Next};
use zapp_ble::smp::{self, Half, ImageSlot, SmpClient, SmpError};
use zapp_ble::{Battery, Level};
use zapp_core::device::{self, ids::Keyboard};

use crate::confirm::{reverted, Landed, PairConfirm};
use crate::error::{classify, in_half, kind_of, ErrorKind, UpdateError};
use crate::event::{Chip, Event, Phase, Reporter, RevertStep, Wait};
use crate::text::{named, short, titled};
use crate::{Options, Outcome};

/// How long to leave a half alone after telling it to restart.
///
/// It has to actually go down before there is any point knocking, and BlueZ
/// will happily hand back a link that is on its way out.
pub(crate) const REBOOT_SETTLE: Duration = Duration::from_secs(5);

/// How long to keep trying to reach a half that is coming back.
///
/// Generous on purpose. A reset is a reboot, an MCUboot swap and a BLE
/// re-connect, and one failed attempt can cost most of a minute on its own:
/// BlueZ waits 20 s for a link it may hand back stale, and a request on a dead
/// one waits out the reply timeout after that. Three attempts have to fit.
pub(crate) const REBOOT_TIMEOUT: Duration = Duration::from_secs(180);

/// Between attempts to reach a half that has not answered yet.
pub(crate) const RETRY_EVERY: Duration = Duration::from_secs(3);

/// How many times to try to confirm a half before giving up on it.
///
/// Nothing else keeps the image, so this is worth retrying: the attempts are one
/// short command each, `retry_every` apart.
const CONFIRM_TRIES: usize = 3;

/// How often a long wait says it is still a wait.
const NOTE_EVERY: Duration = Duration::from_secs(15);

/// The left holds one DFU lock for both halves, and relaying to the right takes
/// it for the whole session — which lingers a few seconds past the last frame so
/// a multi-frame exchange is not cut in half. Anything that touched the right,
/// including merely reading its image state, therefore blocks the left's own
/// upload with "busy" until that lapses. Wait it out rather than racing it.
pub(crate) const RELAY_SETTLE: Duration = Duration::from_secs(5);

/// How long the STM32 gets to reach its bootloader.
pub(crate) const BLE_BOOTLOADER_WAIT: Duration = Duration::from_secs(30);

/// Bluetooth writes spent on one request when nobody said otherwise.
///
/// What the board says it can reassemble is more than it will take. Reassembly
/// across writes carries no sequence numbers and no checksum — neither Zephyr's
/// nor the left relay's — so a fragment lost anywhere discards the request
/// silently and no reply ever comes; the more writes a request spans, the more
/// of that exposure it buys. Measured on a Wireless Voyager's left half (reports 2475,
/// 249 per write): os_mgmt echo answers up to 8 writes and stops dead at 9, and
/// an upload of a 316 KB image takes 51 s at one write, 25 s at five, 22 s at
/// eight. Five keeps margin under the ceiling for most of the gain.
///
/// The right half is not measured: its ceiling is the 1024-byte frame the relay
/// and its own reassembly buffer share, which five writes already exceeds, so
/// the reported buffer is what binds and the count only matters if that frame
/// size changes. Same default either way; `writes` and `chunk` override it.
const WRITES_PER_REQUEST: usize = 5;

/// Updates both halves from a bundle, over Bluetooth.
///
/// Reads what both halves carry, gates on the battery, declares the update to
/// the left, writes the right then the left (each marked for test and
/// restarted), reads both back and keeps both or puts both back.
pub fn update_bundle(
    bundle: &Bundle,
    options: &Options,
    events: &mut dyn FnMut(Event),
) -> std::result::Result<Outcome, UpdateError> {
    let mut reporter = Reporter::new(events);
    let mut radio = Radio {
        device: options.device.as_deref(),
    };
    let mut link = Link {
        board: &mut radio,
        options,
        reporter: &mut reporter,
    };
    update(&mut link, bundle).map_err(classify)
}

/// What the update needs of a Bluetooth board: a session on either half, and
/// the two readings the battery gate is made from.
pub(crate) trait Board {
    /// An SMP client onto `half`, the right reached through the left's relay.
    fn open(&mut self, half: Half) -> std::result::Result<SmpClient, SmpError>;
    /// What the halves have left, as the board last published it.
    fn battery(&mut self) -> std::result::Result<Battery, SmpError>;
    /// Whether the pack is on the charger.
    fn charging(&mut self) -> bool;
}

/// The board a hint names, over the platform's Bluetooth stack.
struct Radio<'a> {
    device: Option<&'a str>,
}

impl Board for Radio<'_> {
    fn open(&mut self, half: Half) -> std::result::Result<SmpClient, SmpError> {
        smp::open(self.device, half)
    }

    fn battery(&mut self) -> std::result::Result<Battery, SmpError> {
        smp::battery(self.device)
    }

    fn charging(&mut self) -> bool {
        charging()
    }
}

/// A board, the options it is reached with, and where progress goes.
pub(crate) struct Link<'a, 'r, B: Board> {
    pub(crate) board: &'a mut B,
    pub(crate) options: &'a Options,
    pub(crate) reporter: &'a mut Reporter<'r>,
}

impl<B: Board> Link<'_, '_, B> {
    /// Connects to `half`, saying so while it happens.
    ///
    /// A board that is bonded but idle takes seconds to wake, and silence
    /// there looks like a hang. Only for a connect nothing else is already
    /// covering: a caller holding a wait of its own wants [`Self::connect_quiet`].
    fn connect(&mut self, half: Half) -> Result<SmpClient> {
        self.reporter.wait(Wait::Connecting(half));
        let client = self.connect_quiet(half);
        self.reporter.clear();
        client
    }

    /// The same connection, for a caller that is already saying what it is
    /// waiting for.
    fn connect_quiet(&mut self, half: Half) -> Result<SmpClient> {
        let client = self.board.open(half);

        let mut client = client.with_context(|| match half {
            Half::Left => "Could not reach the board over Bluetooth".to_string(),
            Half::Right => "Could not reach the right half over Bluetooth".to_string(),
        })?;
        let link = self.options.link;
        if let Some(mtu) = link.mtu {
            client = client.with_mtu(mtu);
        }
        let mut client = client.with_timeout(self.options.reply_timeout);

        // The timeout has to be set first: this is a request like any other, and
        // it is the first thing the board is asked.
        match link.chunk {
            Some(chunk) => client = client.with_request_limit(chunk),
            None => {
                let writes = link.writes.unwrap_or(WRITES_PER_REQUEST).max(1);
                let reported = client.negotiate_request_limit();
                let safe = (client.mtu() * writes).min(reported);
                if reported > safe {
                    log::info!(
                        "board reports {reported} per request; using {safe} \
                         ({writes} writes) — pass --chunk or --writes to override"
                    );
                    client = client.with_request_limit(safe);
                }
            }
        }
        log::debug!(
            "connected, {} bytes per write, {} per request",
            client.mtu(),
            client.request_limit()
        );
        Ok(client)
    }
}

/// Runs the update, and lets the session go if it fails part-way.
fn update<B: Board>(link: &mut Link<B>, bundle: &Bundle) -> Result<Outcome> {
    let mut progress = Progress::default();
    match run(link, bundle, &mut progress) {
        Ok(outcome) => Ok(outcome),
        Err(err) => {
            // Before anything is said, while a connection is still likely to
            // be there: a session left open holds the next attempt out until it
            // expires, and closing it costs one round trip.
            close_session(link, &progress);
            Err(err)
        }
    }
}

/// What has been done to the keyboard so far.
///
/// Kept as it goes rather than worked out afterwards: the ordering rule, the
/// reads at the end and the session close all turn on how far the run got.
#[derive(Debug, Default, Clone, Copy)]
struct Progress {
    /// The id the left gave the update session, for the close that ends it.
    session_id: Option<u32>,
    /// The right has been marked and restarted, which is what the left's own
    /// upload waits on.
    right_restarted: bool,
    /// The left has been marked and restarted, which drops the session with it.
    left_restarted: bool,
}

fn run<B: Board>(link: &mut Link<B>, bundle: &Bundle, progress: &mut Progress) -> Result<Outcome> {
    let force = link.options.force;

    // ---- What is there now ----------------------------------------------
    log::debug!("reading what is installed");
    link.reporter.phase(Phase::Connecting);

    let left_slots = read_state(link, Half::Left, Connect::Announced)
        .map_err(|err| in_half(Half::Left, err))?;
    link.reporter.phase(Phase::Reading);
    // Levels are only published over a live connection, so this is asked after
    // the left has been talked to rather than before. A failure here is an
    // unknown level, which the gate allows.
    let battery = link.board.battery().unwrap_or_else(|err| {
        log::debug!("could not read the battery levels: {err}");
        Battery::default()
    });
    let charging = link.board.charging();
    log::debug!("charge {}", charge(battery, charging));
    // Before the right half is touched, not after: reading the right goes
    // through the left's relay, and the relay is battery-gated too. On a flat
    // pack that read is the thing that fails, and it fails looking like a
    // radio-link fault — so the check that can actually name the battery has
    // to run first.
    check_battery(battery, charging)?;
    let right_slots = read_state(link, Half::Right, Connect::Announced).map_err(|err| {
        kind_of(
            ErrorKind::RadioLinkDown,
            Some(Half::Right),
            err.context("The halves' radio link is down"),
        )
    })?;

    // What each half would need if nobody had asked for a rewrite. Kept even
    // under `force`, because a half that is already running the bundle's image
    // cannot be marked for test however hard this end tries — see `swaps`.
    let installed = [
        (Half::Right, bundle::step_for(bundle.image(Half::Right), &right_slots)),
        (Half::Left, bundle::step_for(bundle.image(Half::Left), &left_slots)),
    ];
    let mut plan = installed;
    if force {
        plan = [(Half::Right, Step::Upload), (Half::Left, Step::Upload)];
    }

    for (half, step) in plan {
        let slots = match half {
            Half::Left => &left_slots,
            Half::Right => &right_slots,
        };
        log::debug!("{:<5}  {}", half.as_str(), describe(step, slots));
    }

    if plan.iter().all(|(_, step)| *step == Step::Installed) {
        return Ok(Outcome::UpToDate);
    }

    // Whether a half's upload is followed by a mark and a restart. See
    // `swaps_after`.
    let swaps = |wanted: Half| swaps_after(step_of(&plan, wanted), step_of(&installed, wanted));

    // The status LEDs are the keyboard's own progress while this runs, and they
    // say it whether or not anything here does.
    if swaps(Half::Right) || swaps(Half::Left) {
        log::debug!(
            "the keyboard cannot be typed on while this runs; each half shows its own \
             progress on its status LEDs — green blinking fast while its image is written, \
             green solid while it restarts and waits to be confirmed, then back to normal; \
             red solid means that half's update was refused or aborted, red blinking slowly \
             means the two halves are on different builds until this finishes; nothing is \
             kept until it has proved itself, so a keyboard interrupted half-way gets its \
             old firmware back on the next power cycle"
        );
    }

    // ---- Say what is being installed, before writing any of it -----------
    //
    // Over Bluetooth the board admits one shape of update: a host states both
    // halves' images and the build they came from, and every write after that
    // is checked against the statement. It is what turns the ordering rule
    // below from an instruction this tool follows into one the keyboard
    // enforces — without a declaration the left cannot tell a sanctioned
    // upload from a stray one, nor tell that the right is finished.
    link.reporter.phase(Phase::Declaring);
    let declaration = declare(bundle, force)?;
    let opened = open_session(link, &declaration).map_err(|err| in_half(Half::Left, err))?;
    progress.session_id = Some(opened.id);
    announce_session(&declaration, &opened);
    // Not under `force`. There the plan writes both halves whatever they are
    // carrying, so a board that says the update is already complete is right
    // and is still going to take the images — that is what the declaration's
    // rewrite flag asks for, and it is not a disagreement worth a line.
    if !force && let Some(note) = cross_check(&plan, opened.next) {
        log::warn!("{note}");
    }

    // ---- Write, right half first ----------------------------------------
    //
    // The order is forced, not a preference. The right half has no radio link
    // to this machine: every frame it sees is relayed by the left, over the
    // halves' own link, and that link only carries anything while both halves
    // speak the same protocol. A left updated first comes back speaking the new
    // one, the old right cannot follow it, and the only route to the half that
    // still needs its image is gone. Right, then left. Do not reorder this.
    for (half, step) in plan {
        if step != Step::Upload {
            continue;
        }
        write_half(link, bundle, &declaration, half, swaps(half), progress)
            .map_err(|err| in_half(half, err))?;
    }

    // ---- Read both, then keep both, or put both back ---------------------
    //
    // The left first, and only now: between the right's restart and the left's
    // the two halves are on different firmware and cannot talk to each other, so
    // the right is unreachable for that whole stretch. Which is why the right is
    // only *read* here rather than kept as it went — nothing is confirmed until
    // both halves have been read back on the images they were written, and then
    // either both are kept in one pass or neither is. This is the *same*
    // decision the wired path makes, through the one [`crate::confirm::settle`].
    //
    // What each half is carrying, seeded from the reads at the top and replaced
    // for every half that comes back from a restart; and the same kept as read,
    // for the revert to put back.
    let running = vec![
        (Half::Left, active_hash(&left_slots)),
        (Half::Right, active_hash(&right_slots)),
    ];
    let before = running.clone();

    // The halves to read (those that restarted, so their running image is in
    // doubt) and the halves to keep (everything not already installed and
    // confirmed). A half running its image on an unconfirmed boot is kept
    // without being re-read; a half that only rebooted is read but may come back
    // already confirmed, in which case it is not committed again.
    let read: Vec<Half> = [Half::Left, Half::Right]
        .into_iter()
        .filter(|half| restarted(*progress, *half))
        .collect();
    let keep = to_keep(&installed);

    link.reporter.phase(Phase::Verifying);
    {
        let mut pair = BleConfirm {
            link: &mut *link,
            bundle,
            running,
            before,
        };
        let settled = crate::confirm::settle(&mut pair, &read, &keep);
        // Whatever it came to, the line the wait owns is given back before
        // anything else is said.
        pair.link.reporter.clear();
        settled?;
    }
    link.reporter.phase(Phase::Confirmed);

    // ---- Let the session go ---------------------------------------------
    //
    // Best effort, and quiet about it. If the left restarted, its own reboot
    // already dropped the session: it lives in the left's memory and the
    // firmware writes none of it to flash, which is the design rather than
    // something that went wrong.
    close_session(link, progress);

    Ok(Outcome::Flashed)
}

/// One half's upload, and the mark and restart that follow it when there is
/// anything to swap to.
fn write_half<B: Board>(
    link: &mut Link<B>,
    bundle: &Bundle,
    declaration: &Declaration,
    half: Half,
    swaps: bool,
    progress: &mut Progress,
) -> Result<()> {
    let image = bundle.image(half);
    let hash = bundle
        .hash(half)
        .with_context(|| format!("The bundle's {half} image carries no hash to mark it by"))?;

    // The left is not writable until the right is back and reporting the
    // image it was given: the board refuses the left's own upload with "the
    // right half first" until then, and the right has only just restarted.
    if half == Half::Left && progress.right_restarted {
        wait_for_right_ready(link, declaration).map_err(|err| in_half(Half::Right, err))?;
    }

    link.reporter.phase(Phase::Writing(half));
    log::debug!("{} half — upload ({} KiB)", titled(half), image.len() / 1024);
    let (mut client, renewed) = connect_declared(link, half, declaration)?;
    progress.session_id = Some(renewed.id);

    upload(&mut client, half, image, link.reporter)?;
    // The transfer was still worth doing under `force` — it is the one part of
    // this that proves the link end to end — but there is nothing to swap to,
    // so the mark that would fail is not sent. The restart still is: an upload
    // puts the half into its update state (green, blinking fast) and only a
    // reset takes it out again, which is what the wired path does too. It
    // comes back on the image it was already running.
    link.reporter.phase(Phase::Restarting(half));
    if swaps {
        restart(&mut client, half, &hash)?;
    } else {
        log::debug!("{half} already runs this exact image — nothing to swap, restarting it as it is");
        client
            .reset()
            .with_context(|| format!("Could not restart the {half} half"))?;
    }
    match half {
        Half::Right => progress.right_restarted = true,
        Half::Left => progress.left_restarted = true,
    }
    Ok(())
}

/// What this bundle declares: both halves' images, by both of the hashes
/// mcumgr names an image with, and the build they came from.
///
/// `force` carries straight through to the declaration. Without it a board that
/// already has both images refuses every upload as finished, which is the right
/// answer to "install this" and the wrong one to "install this again" — so
/// `force` has to be part of what is declared, not just of the local plan.
fn declare(bundle: &Bundle, force: bool) -> Result<Declaration> {
    let declared = |half: Half| {
        session::Declared::of(bundle.image(half))
            .with_context(|| format!("The bundle's {half} image carries no hash to declare it by"))
    };
    Ok(Declaration {
        left: declared(Half::Left)?,
        right: declared(Half::Right)?,
        version: bundle.header.build_version.clone(),
        redo: force,
    })
}

/// Declares the update on the left half, and says what the board makes of it.
///
/// Only the left serves the session commands, and it is reachable throughout an
/// update, so this is where a session lost to a reconnect comes back. Declaring
/// the same update again renews the one already open rather than starting a
/// second, so this is called whenever a connection is made rather than only once
/// — and a board reporting no session is a board to declare to, not a failure.
fn open_session<B: Board>(link: &mut Link<B>, declaration: &Declaration) -> Result<session::Status> {
    let mut client = link.connect(Half::Left)?;
    client
        .session_open(declaration)
        .context("Could not declare the update to the left half")
}

/// Connects to a half with the update declared and live.
///
/// Both writes need it: the left checks its own upload and the restart that
/// installs it against the declaration, and the right checks the claim the left
/// pushes over the halves' link. The renewal always goes to the left, because
/// the relay channel carries no session commands of its own — the right runs no
/// session, only the left's statement repeated to it.
fn connect_declared<B: Board>(
    link: &mut Link<B>,
    half: Half,
    declaration: &Declaration,
) -> Result<(SmpClient, session::Status)> {
    if half == Half::Left {
        thread::sleep(link.options.timings.relay_settle);
    }
    let mut left = link.connect(Half::Left)?;
    let status = left
        .session_open(declaration)
        .context("Could not declare the update to the left half")?;

    match half {
        Half::Left => Ok((left, status)),
        // One more connection than a right-half upload strictly needs, and
        // worth it: the declaration has to be renewed from the left, and the
        // image has to go over the relay.
        Half::Right => {
            drop(left);
            Ok((link.connect(half)?, status))
        }
    }
}

/// Waits for the right half to come back, before the left is written.
///
/// The board's own `next` is the instrument, not `rdone`. It turns to the left
/// once the right is reachable and carrying its declared image, which is the
/// whole of the board's ordering rule — the confirm is no part of it, and comes
/// later and from here.
///
/// Asked of the left, over one connection held for the whole wait. The left is
/// up throughout, it is the half that holds the session, and STATUS is never
/// refused. Reading the right through the relay would work too and would say
/// less for more.
///
/// Re-declaring on each poll is what keeps the session's idle timer alive
/// through the wait; an identical declaration renews rather than collides.
fn wait_for_right_ready<B: Board>(link: &mut Link<B>, declaration: &Declaration) -> Result<()> {
    link.reporter.phase(Phase::WaitingForRight);
    link.reporter.wait(Wait::RightRestart);
    let ready = right_ready(link, declaration);
    link.reporter.clear();
    ready
}

/// The wait itself, with the line left to its caller.
fn right_ready<B: Board>(link: &mut Link<B>, declaration: &Declaration) -> Result<()> {
    let timings = link.options.timings;
    thread::sleep(timings.reboot_settle);

    let mut client = link.connect_quiet(Half::Left)?;
    let started = Instant::now();
    let mut noted = Instant::now();
    // Whether the right was ever reporting at all. A right that never says
    // anything is a right that did not come back, which is a different thing
    // to tell someone than one that came back on the wrong image.
    let mut seen = false;
    // Consecutive polls reporting the old image. The restart itself happens
    // inside the reboot settle, so the drop is usually never observed from
    // here; the guard against a stale pre-reboot reading is repetition — the
    // left refreshes the right's state every second, so three polls over ~9 s
    // reporting the old image are the bootloader's answer, not a cache.
    let mut on_old_image = 0u32;

    loop {
        match client.session_open(declaration) {
            Ok(status) => {
                seen |= status.right_seen;
                if matches!(status.next, Next::Left | Next::Complete) {
                    log::debug!(
                        "right: back after {} s, confirmed: {}",
                        started.elapsed().as_secs(),
                        status.right_done
                    );
                    return Ok(());
                }
                // Back from the restart but running something other than what
                // was written: the bootloader validated the image and put the
                // old one back. Waiting longer cannot change its mind.
                let declared = &declaration.right.slot_hash;
                let rejected = status
                    .right_tag
                    .as_ref()
                    .is_some_and(|tag| !declared.starts_with(tag));
                on_old_image = if rejected { on_old_image + 1 } else { 0 };
                if on_old_image >= 3 {
                    log::debug!(
                        "right: back after {} s on its old image — the bootloader rejected \
                         the written one",
                        started.elapsed().as_secs()
                    );
                    return Err(right_rejected());
                }
            }
            Err(err) => {
                log::debug!("the left stopped answering while waiting on the right: {err:#}");
                client = link.connect_quiet(Half::Left)?;
            }
        }

        if started.elapsed() >= timings.reboot_timeout {
            break;
        }
        if noted.elapsed() >= NOTE_EVERY {
            noted = Instant::now();
            log::debug!(
                "right: not back yet after {} s of {}",
                started.elapsed().as_secs(),
                timings.reboot_timeout.as_secs()
            );
        }
        thread::sleep(timings.retry_every);
    }

    let waited = timings.reboot_timeout.as_secs();
    if on_old_image > 0 {
        return Err(right_rejected());
    }
    if seen {
        bail!("The right half did not settle on the written image within {waited} s.");
    }
    bail!("The right half did not come back within {waited} s of its restart.")
}

/// The right half's bootloader refused the written image and booted the old
/// firmware instead.
fn right_rejected() -> anyhow::Error {
    kind_of(
        ErrorKind::Rejected,
        Some(Half::Right),
        anyhow!("the right half rejected the image: bad signature or corrupt file"),
    )
}

/// Ends the session, if there is one left to end.
///
/// Best effort by design, and never a reason to fail an update that worked. A
/// left half that has restarted has already dropped it, and a board that will
/// not answer here is one that has gone quiet after the last thing that mattered
/// was already done.
fn close_session<B: Board>(link: &mut Link<B>, progress: &Progress) {
    let Some(id) = progress.session_id else {
        return;
    };
    if progress.left_restarted {
        log::debug!("the left restarted, so update session {id} is already gone");
        return;
    }

    let closed = link
        .connect(Half::Left)
        .and_then(|mut client| client.session_close(Some(id)).map_err(anyhow::Error::from));
    match closed {
        Ok(status) if !status.open => log::debug!("update session {id} closed"),
        Ok(_) => log::debug!("the board still reports a session open after the close"),
        Err(err) => log::debug!("could not close update session {id}: {err:#}"),
    }
}

/// What the board said when the update was declared to it.
fn announce_session(declaration: &Declaration, status: &session::Status) {
    log::debug!(
        "declared {} to the keyboard (session {}); the board says {} next",
        named(&declaration.version),
        status.id,
        next_as(status.next)
    );
    if declaration.redo {
        log::debug!("asking for both halves to be written again, in the same order");
    }
    if !status.right_seen {
        log::debug!("the board cannot read the right half's image state at all");
    }
    if !status.guard {
        log::debug!("this board is not enforcing declared updates: a development build");
    }
}

/// What one of the board's own steps means, in the words the rest of this file
/// uses for the halves.
fn next_as(next: Next) -> String {
    match next {
        Next::None => "no update is declared".to_string(),
        Next::Right => "the right half is".to_string(),
        Next::Left => "the left half is".to_string(),
        Next::Complete => "there is nothing".to_string(),
        Next::Unknown(code) => format!("a step this zapp does not know ({code}) is"),
    }
}

/// What the board ought to say is next, given the plan this run made.
///
/// A half carrying its declared image counts as finished whether or not it has
/// confirmed it, which is the rule the board applies too: its ordering gate asks
/// that a half be carrying what was declared and reachable, not that it be
/// keeping it. So an interrupted pair left running the bundle unconfirmed is
/// already where a fresh run expects it — the run plans two confirms, writes
/// nothing, and this end and the board agree about it.
fn expected_next(plan: &[(Half, Step)]) -> Next {
    let step = |wanted: Half| {
        plan.iter()
            .find(|(half, _)| *half == wanted)
            .map(|(_, step)| *step)
    };
    let done =
        |half: Half| step(half).is_none_or(|step| step == Step::Installed || step == Step::Confirm);

    match (done(Half::Right), done(Half::Left)) {
        (false, _) => Next::Right,
        (true, false) => Next::Left,
        (true, true) => Next::Complete,
    }
}

/// A note for a board whose own sequencing disagrees with this run's plan.
///
/// The board's view is the one that decides: it can see what each half is
/// carrying, and it refuses on that. Saying so is worth a line, because a
/// disagreement here is the first sign of a half this end has read wrongly.
fn cross_check(plan: &[(Half, Step)], next: Next) -> Option<String> {
    let expected = expected_next(plan);
    if next == expected {
        return None;
    }
    Some(format!(
        "this run planned {} first; the board says otherwise.",
        match expected {
            Next::Right => "the right half",
            Next::Left => "the left half",
            Next::Complete => "nothing",
            _ => "nothing in particular",
        }
    ))
}

/// One half's step out of a plan.
///
/// Both arrays are built with both halves in them, so the fallback is
/// unreachable; `Installed` is the inert answer if it ever were.
fn step_of(steps: &[(Half, Step)], wanted: Half) -> Step {
    steps
        .iter()
        .find(|(half, _)| *half == wanted)
        .map_or(Step::Installed, |(_, step)| *step)
}

/// Whether a half's upload is followed by a mark and a restart.
///
/// `planned` is what this run will do to the half; `found` is what the half was
/// carrying when the run started.
///
/// A half already running the bundle's image, confirmed, is uploaded under
/// `force` but never marked. MCUboot refuses to mark the slot a board is
/// running from for test — image manager rc 33,
/// `IMG_MGMT_ERR_IMAGE_SETTING_TEST_TO_ACTIVE_DENIED`, from `!confirm && slot ==
/// active_slot` — and it is right to, because there is nothing to swap to. A
/// mark carries a hash, and a hash names an image rather than a slot, so a board
/// with the same build in both slots resolves it to the active one and says no.
/// Flashing the running build with `force` does that every time; a real update
/// never can, because its hash is not the one the board is running.
///
/// Only reachable under `force`. Without it such a half reads as
/// [`Step::Installed`] and the run stops before any session is opened.
fn swaps_after(planned: Step, found: Step) -> bool {
    planned == Step::Upload && found != Step::Installed
}

/// The halves the run ends by confirming: every one not already running its
/// image confirmed.
///
/// Read off what the halves were found carrying, not off the plan. `force`
/// plans an upload for both, but a half already running the bundle's image is
/// never marked (see [`swaps_after`]), and confirming it anyway is refused by
/// the board — the session is still waiting on that half's restart.
fn to_keep(installed: &[(Half, Step)]) -> Vec<Half> {
    installed
        .iter()
        .filter(|(_, step)| *step != Step::Installed)
        .map(|(half, _)| *half)
        .collect()
}

fn restarted(progress: Progress, half: Half) -> bool {
    match half {
        Half::Left => progress.left_restarted,
        Half::Right => progress.right_restarted,
    }
}

/// Sends one half's image into its spare slot, over a connection the caller has
/// already declared the update on.
fn upload(client: &mut SmpClient, half: Half, image: &[u8], reporter: &mut Reporter) -> Result<()> {
    let chip = Chip::Nrf(half);
    reporter.emit(Event::TransferStarted(chip));
    let result = {
        let reporter = RefCell::new(&mut *reporter);
        client.upload(0, image, &|progress| {
            reporter.borrow_mut().emit(Event::Transfer(chip, progress));
        })
    };
    if let Err(err) = result {
        reporter.emit(Event::TransferFailed(chip));
        return Err(err).with_context(|| format!("The {half} half's upload failed"));
    }
    log::debug!("{} bytes written into the {half} half's spare slot", image.len());
    Ok(())
}

/// Marks a half's new image for test and restarts it.
///
/// For test, never confirmed: an image that is confirmed before it has run is an
/// image with no way back. Nothing on the keyboard confirms it either, so until
/// this run does — after both halves have been seen back on what they were
/// written — every restart puts the old image back.
fn restart(client: &mut SmpClient, half: Half, hash: &[u8]) -> Result<()> {
    client
        .mark(hash, false)
        .with_context(|| format!("Could not mark the {half} half's new image for test"))?;
    log::debug!("marked {} for test on the {half} half", short(hash));
    client
        .reset()
        .with_context(|| format!("Could not restart the {half} half"))?;
    log::debug!("restarting the {half} half");
    Ok(())
}

/// The hash of whatever a half reports it is running.
fn active_hash(slots: &[ImageSlot]) -> Vec<u8> {
    slots
        .iter()
        .find(|slot| slot.active)
        .map(|slot| slot.hash.clone())
        .unwrap_or_default()
}

/// Fails unless both halves are running the bundle's image for them.
///
/// Wording matches the wired path in [`crate::wired`], which makes the same
/// check from the hashes it reads back over the cable.
fn both_on_the_bundle(bundle: &Bundle, running: &[(Half, Vec<u8>)]) -> Result<()> {
    let wrong: Vec<String> = running
        .iter()
        .filter(|(half, hash)| bundle.hash(*half).is_some_and(|wanted| wanted != *hash))
        .map(|(half, hash)| {
            format!(
                "the {half} half is carrying {}",
                match hash.is_empty() {
                    true => "nothing it would name".to_string(),
                    false => short(hash),
                }
            )
        })
        .collect();
    if wrong.is_empty() {
        return Ok(());
    }

    bail!(
        "The halves are on different builds: {}; the bundle is {}.",
        wrong.join(", "),
        named(&bundle.header.build_version)
    )
}

/// Waits for a half to come back, and says what it came back as.
///
/// None of the three answers is a reason to abort on its own: whether a half
/// that is missing or on the wrong image is fatal depends on the other half too,
/// and that decision is [`crate::confirm::decide`]'s. So this reports rather
/// than fails.
fn wait_for_boot<B: Board>(link: &mut Link<B>, half: Half, hash: &[u8]) -> Landed {
    let timings = link.options.timings;
    thread::sleep(timings.reboot_settle);

    let started = Instant::now();
    loop {
        let slots = match read_state(link, half, Connect::Quiet) {
            Ok(slots) => slots,
            Err(err) if started.elapsed() < timings.reboot_timeout => {
                log::debug!(
                    "{half} half not back yet after {} s: {err:#}",
                    started.elapsed().as_secs()
                );
                thread::sleep(timings.retry_every);
                continue;
            }
            Err(err) => {
                log::debug!("{half} half never came back: {err:#}");
                return Landed::Gone {
                    why: format!(
                        "The {half} half did not come back within {} s of its restart.",
                        timings.reboot_timeout.as_secs()
                    ),
                };
            }
        };

        let Some(active) = slots.iter().find(|slot| slot.active) else {
            return Landed::Wrong {
                active: Vec::new(),
                version: String::new(),
            };
        };
        if active.hash != hash {
            return Landed::Wrong {
                active: active.hash.clone(),
                version: active.version.clone(),
            };
        }
        log::debug!(
            "{half}: back after {} s, running {} ({})",
            started.elapsed().as_secs(),
            short(&active.hash),
            named(&active.version)
        );
        return Landed::Ok {
            confirmed: active.confirmed,
            active: active.hash.clone(),
        };
    }
}

/// Keeps the image a half is running.
///
/// This is where an image is kept: the firmware confirms nothing on its own, so
/// a half that is not confirmed from here puts its old firmware back on its next
/// reset. Which is why this only ever runs once both halves have been seen back
/// on the images they were written — there is no undoing it afterwards.
fn confirm<B: Board>(link: &mut Link<B>, half: Half, hash: &[u8], already: bool) -> Result<()> {
    if already {
        log::debug!("{half}: already confirmed");
        return Ok(());
    }

    let mut last = None;
    for attempt in 1..=CONFIRM_TRIES {
        let marked = link
            .connect_quiet(half)
            .and_then(|mut client| Ok(client.mark(hash, true)?));
        match marked {
            Ok(_) => {
                log::debug!("{half}: confirmed");
                return Ok(());
            }
            Err(err) => {
                log::debug!("could not confirm the {half} half (try {attempt}): {err:#}");
                if attempt < CONFIRM_TRIES {
                    thread::sleep(link.options.timings.retry_every);
                }
                last = Some(err);
            }
        }
    }

    Err(last.unwrap_or_else(|| anyhow!("nothing was tried")))
        .with_context(|| format!("Could not confirm the {half} half's image"))
}

/// The Bluetooth side of [`PairConfirm`].
///
/// `running` is the wait-for-reboot read the run already did; `commit` is the
/// `mark(hash, true)` that keeps an image. The right is confirmed before the
/// left — everything to the right goes through the left's relay, so if the
/// right cannot be kept the left is still unconfirmed and still revertable.
struct BleConfirm<'a, 'b, 'r, B: Board> {
    link: &'a mut Link<'b, 'r, B>,
    bundle: &'a Bundle,
    /// What each half is carrying, replaced for every half that comes back from
    /// a restart. The mismatch check reads this.
    running: Vec<(Half, Vec<u8>)>,
    /// The same, kept as it was read at the top of the run: what a revert puts
    /// back.
    before: Vec<(Half, Vec<u8>)>,
}

impl<B: Board> PairConfirm for BleConfirm<'_, '_, '_, B> {
    fn running(&mut self, half: Half) -> Landed {
        self.link.reporter.wait(Wait::HalvesRestart);
        let hash = self.bundle.hash(half).unwrap_or_default();
        let verdict = wait_for_boot(self.link, half, &hash);
        // Keep the running-hash table current for the mismatch check.
        match &verdict {
            Landed::Ok { active, .. } | Landed::Wrong { active, .. } => {
                if let Some(slot) = self.running.iter_mut().find(|(which, _)| *which == half) {
                    slot.1 = active.clone();
                }
            }
            Landed::Gone { .. } => {}
        }
        verdict
    }

    fn mismatch(&mut self, _landed: &[(Half, Landed)]) -> Option<String> {
        both_on_the_bundle(self.bundle, &self.running)
            .err()
            .map(|err| err.to_string())
    }

    fn commit(&mut self, half: Half) -> Result<()> {
        let hash = self
            .bundle
            .hash(half)
            .with_context(|| format!("The bundle's {half} image carries no hash"))
            .map_err(|err| in_half(half, err))?;
        confirm(self.link, half, &hash, false).map_err(|err| in_half(half, err))
    }

    fn revert(&mut self, half: Half) -> Result<()> {
        self.link
            .connect(half)
            .and_then(|mut client| Ok(client.reset()?))
    }

    fn keeping(&mut self) {
        self.link.reporter.phase(Phase::Confirming);
        self.link.reporter.wait(Wait::Confirming);
    }

    /// Puts both halves back on what they were running, and says so.
    ///
    /// Nothing has been confirmed at this point, so a reset is the whole of the
    /// revert: MCUboot puts the old image back on an unconfirmed one. A half
    /// that is [`Landed::Gone`] cannot be reset from here and is named instead
    /// — a power cycle reverts it just the same.
    fn revert_pair(&mut self, landed: Vec<(Half, Landed)>, reasons: Vec<String>) -> anyhow::Error {
        // Nothing was restarted, so nothing was staged and there is nothing to
        // put back — the halves are as they were found. Only reachable from a
        // pair that read as mismatched before anything was touched.
        if landed.is_empty() {
            return anyhow!("{}", reasons.join(" "));
        }

        self.link.reporter.clear();
        self.link.reporter.phase(Phase::Reverting);

        // Right then left, the same order everything else touches them in, with
        // the relay's lock given time to lapse between the two.
        for half in [Half::Right, Half::Left] {
            let Some((_, verdict)) = landed.iter().find(|(which, _)| *which == half) else {
                continue;
            };
            if matches!(verdict, Landed::Gone { .. }) {
                self.link
                    .reporter
                    .emit(Event::Reverting(half, RevertStep::Unreachable));
                continue;
            }
            let step = match self.revert(half) {
                Ok(()) => RevertStep::Restarting,
                Err(err) => RevertStep::Failed(format!("{err:#}")),
            };
            self.link.reporter.emit(Event::Reverting(half, step));
            if half == Half::Right {
                thread::sleep(self.link.options.timings.relay_settle);
            }
        }

        // The left first this time: it is reachable directly, and the right is
        // only reachable through it once both are back on the same build.
        let before = self.before.clone();
        let mut restored: Vec<(Half, bool)> = Vec::new();
        for half in [Half::Left, Half::Right] {
            if !landed.iter().any(|(which, _)| *which == half) {
                continue;
            }
            let was = before
                .iter()
                .find(|(which, _)| *which == half)
                .map(|(_, hash)| hash.clone())
                .unwrap_or_default();
            // The read itself is quiet, so the wait needs a line of its own —
            // put back before anything is said about how it went.
            self.link.reporter.wait(Wait::HalvesRestart);
            let back = wait_for_boot(self.link, half, &was);
            self.link.reporter.clear();
            restored.push((half, matches!(back, Landed::Ok { .. })));
        }

        reverted(&restored, &reasons.join(" "))
    }
}

/// Whether a connection announces itself.
///
/// One line, one wait: a connect made while the run is already saying what it
/// is waiting for has to keep quiet, or the two redraw over each other.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Connect {
    /// Nothing else is on the line, so the wait for the board is worth a line
    /// of its own — a bonded but idle board takes seconds to wake.
    Announced,
    /// The caller is already holding the line.
    Quiet,
}

/// One image-state read, over a session opened for it.
fn read_state<B: Board>(link: &mut Link<B>, half: Half, saying: Connect) -> Result<Vec<ImageSlot>> {
    let mut client = match saying {
        Connect::Announced => link.connect(half)?,
        Connect::Quiet => link.connect_quiet(half)?,
    };
    client
        .image_state()
        .with_context(|| format!("Could not read the {half} half's image state"))
}

/// Refuses an update a pack cannot see through.
///
/// A level nobody can read is allowed, deliberately: see `bundle::battery_allows`.
fn check_battery(battery: Battery, charging: bool) -> Result<()> {
    // Left, right, and the combined pack.
    let levels = [
        battery.left.percent(),
        battery.right.percent(),
        battery.keyboard.percent(),
    ];
    if levels
        .iter()
        .all(|level| bundle::battery_allows(*level, charging))
    {
        return Ok(());
    }

    Err(kind_of(
        ErrorKind::LowBattery,
        None,
        anyhow!(
            "Batteries low, please charge both halves to at least {}% to flash over Bluetooth.",
            bundle::MIN_SOC_PERCENT
        ),
    ))
}

/// Whether the pack is on the charger.
///
/// The nRF publishes no charging state over Bluetooth, so the only thing that
/// says so from here is the wire: a Wireless Voyager on USB is one with VBUS present,
/// which is exactly what the firmware's own gate tests. A second ZSA board on
/// the same machine would read as charging, which errs towards letting an
/// update run — the direction this whole check leans anyway.
fn charging() -> bool {
    device::list_devices().is_ok_and(|devices| {
        devices
            .iter()
            .any(|device| device.keyboard == Some(Keyboard::WirelessVoyager))
    })
}

fn describe(step: Step, slots: &[ImageSlot]) -> String {
    let running = slots
        .iter()
        .find(|slot| slot.active)
        .map(|slot| named(&slot.version))
        .unwrap_or_else(|| "(nothing active)".to_string());
    match step {
        Step::Installed => format!("{running} · already running this image"),
        Step::Confirm => format!("{running} · running this image, not confirmed — will be kept"),
        Step::Upload => format!("{running} · will be updated"),
    }
}

fn charge(battery: Battery, charging: bool) -> String {
    // Both halves are always named here: this runs against a split board that
    // is being updated, where a half with no level is a half worth noticing.
    let level = |level: Level| match level {
        Level::Percent(level) => format!("{level}%"),
        _ => "unknown".to_string(),
    };
    format!(
        "left {} · right {}{}",
        level(battery.left),
        level(battery.right),
        if charging { " · on the charger" } else { "" }
    )
}

// ---------------------------------------------------------------------------
// A keyboard image into the STM32
// ---------------------------------------------------------------------------

/// A keyboard image into the STM32, over ZSA's update service on the left half,
/// which relays it over the wire between the chips into the ignition bootloader.
pub fn flash_stm32(
    image: &[u8],
    options: &Options,
    events: &mut dyn FnMut(Event),
) -> std::result::Result<(), UpdateError> {
    let mut reporter = Reporter::new(events);
    flash_stm32_inner(image, options, &mut reporter).map_err(classify)
}

fn flash_stm32_inner(image: &[u8], options: &Options, reporter: &mut Reporter) -> Result<()> {
    // Refused here rather than by the keyboard: the board answers this with one
    // error byte, and the sentence that carries both figures is the one that
    // says what to do about it.
    if image.len() > dfu::MAX_IMAGE {
        bail!(
            "The image is {} bytes; the STM32's application region holds {} bytes.",
            image.len(),
            dfu::MAX_IMAGE
        );
    }

    log::debug!("image {} bytes, crc-32 {:08x}", image.len(), dfu::crc32(image));

    reporter.phase(Phase::Connecting);
    reporter.wait(Wait::Connecting(Half::Left));
    let client = dfu::open(options.device.as_deref());
    reporter.clear();
    let mut client = client.context("Could not reach the keyboard over Bluetooth")?;
    log::debug!("connected, {} bytes of image per write", client.chunk());

    reporter.wait(Wait::Bootloader);
    let ready = client.make_ready(options.timings.stm32_bootloader_wait, &report_wait);
    reporter.clear();
    ready.context("The STM32 is not in its bootloader")?;

    reporter.phase(Phase::Writing(Half::Left));
    let chip = Chip::Stm32;
    reporter.emit(Event::TransferStarted(chip));
    let started = Instant::now();
    let result = {
        let reporter = RefCell::new(&mut *reporter);
        client.flash(image, &|progress| {
            reporter.borrow_mut().emit(Event::Transfer(chip, progress));
        })
    };
    if let Err(err) = result {
        reporter.emit(Event::TransferFailed(chip));
        return Err(err).context("The flash failed");
    }

    log::debug!(
        "wrote {} bytes in {:.1} s",
        image.len(),
        started.elapsed().as_secs_f64()
    );
    Ok(())
}

/// What the readiness step is doing.
fn report_wait(wait: Readiness) {
    match wait {
        Readiness::Already => log::debug!("the STM32 is already in its bootloader"),
        Readiness::Rebooting => log::debug!("rebooting the STM32 into its bootloader"),
        Readiness::Waiting(waited) => log::debug!("not there yet after {} s", waited.as_secs()),
        Readiness::Arrived(waited) => {
            log::debug!("in the bootloader after {} s", waited.as_secs())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A signed image whose trailer carries a 64-byte hash, which is what this
    /// product's images carry: MCUboot here is built for SHA-512.
    fn signed_image(payload: u8, hash: &[u8; 64]) -> Vec<u8> {
        const MCUBOOT_MAGIC: u32 = 0x96f3_b83d;
        let body = vec![payload; 96];

        let mut out = Vec::new();
        out.extend_from_slice(&MCUBOOT_MAGIC.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // load address
        out.extend_from_slice(&32u16.to_le_bytes()); // header size
        out.extend_from_slice(&0u16.to_le_bytes()); // protected tlvs
        out.extend_from_slice(&(body.len() as u32).to_le_bytes());
        out.extend_from_slice(&[0u8; 16]);
        out.extend_from_slice(&body);

        out.extend_from_slice(&0x6907u16.to_le_bytes()); // tlv area
        out.extend_from_slice(&((4 + 4 + hash.len()) as u16).to_le_bytes());
        out.extend_from_slice(&0x0012u16.to_le_bytes()); // SHA-512
        out.extend_from_slice(&(hash.len() as u16).to_le_bytes());
        out.extend_from_slice(hash);
        out
    }

    /// A whole .zsab, laid out the way `tools/zsa_bundle.py` writes one.
    fn bundled() -> Vec<u8> {
        let left = signed_image(0x11, &[0xa1; 64]);
        let right = signed_image(0x22, &[0xb2; 64]);

        let mut out = Vec::new();
        out.extend_from_slice(b"ZSAB");
        out.extend_from_slice(&1u16.to_le_bytes()); // format version
        out.extend_from_slice(&0u16.to_le_bytes()); // flags
        out.extend_from_slice(&0x3Cu32.to_le_bytes());
        out.extend_from_slice(&(left.len() as u32).to_le_bytes());
        out.extend_from_slice(&((0x3C + left.len()) as u32).to_le_bytes());
        out.extend_from_slice(&(right.len() as u32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // key fingerprint
        let mut version = [0u8; 32];
        version[.."1.2.3-test".len()].copy_from_slice(b"1.2.3-test");
        out.extend_from_slice(&version);
        out.extend_from_slice(&left);
        out.extend_from_slice(&right);
        out
    }

    /// A plan the way `run` builds one: the right first, then the left.
    fn plan(right: Step, left: Step) -> [(Half, Step); 2] {
        [(Half::Right, right), (Half::Left, left)]
    }

    #[test]
    fn a_half_already_running_the_bundles_image_is_uploaded_but_never_marked() {
        // The bench case: the same build flashed over the wire again and again
        // leaves both slots holding it, so the hash a mark carries resolves to
        // the slot the board is running from and the firmware refuses it with
        // image manager rc 33. --force still sends the upload — it is the one
        // part of the run that proves the link end to end — and nothing
        // follows it.
        assert!(!swaps_after(Step::Upload, Step::Installed));
    }

    #[test]
    fn a_forced_half_already_running_its_image_is_not_confirmed_either() {
        // The run that failed over Bluetooth: both halves on v0.1.15 already,
        // --force, both uploaded, neither restarted — and a confirm sent for
        // the right that the board refused as "the left half is busy".
        assert_eq!(to_keep(&plan(Step::Installed, Step::Installed)), vec![]);
        assert_eq!(to_keep(&plan(Step::Installed, Step::Upload)), vec![Half::Left]);
        assert_eq!(
            to_keep(&plan(Step::Confirm, Step::Upload)),
            vec![Half::Right, Half::Left]
        );
    }

    #[test]
    fn a_half_that_really_needs_the_image_is_still_marked_and_restarted() {
        assert!(swaps_after(Step::Upload, Step::Upload));
        // A half running the bundle's image on an unconfirmed test boot is not
        // settled: it has a swap outstanding, and skipping it would leave the
        // keyboard to revert on its next restart.
        assert!(swaps_after(Step::Upload, Step::Confirm));
        // And nothing is marked where nothing was uploaded.
        assert!(!swaps_after(Step::Installed, Step::Installed));
        assert!(!swaps_after(Step::Confirm, Step::Confirm));
    }

    #[test]
    fn a_step_is_read_by_half_rather_than_by_position() {
        // The two arrays `run` keeps are both right-first; reading one by index
        // would silently swap the halves over.
        let forced = plan(Step::Upload, Step::Upload);
        let found = plan(Step::Installed, Step::Upload);
        assert_eq!(step_of(&found, Half::Right), Step::Installed);
        assert_eq!(step_of(&found, Half::Left), Step::Upload);
        // So under --force the right is skipped and the left is not.
        assert!(!swaps_after(
            step_of(&forced, Half::Right),
            step_of(&found, Half::Right)
        ));
        assert!(swaps_after(
            step_of(&forced, Half::Left),
            step_of(&found, Half::Left)
        ));
    }

    #[test]
    fn the_step_this_run_expects_is_the_one_the_board_should_name() {
        assert_eq!(
            expected_next(&plan(Step::Upload, Step::Upload)),
            Next::Right
        );
        assert_eq!(
            expected_next(&plan(Step::Installed, Step::Upload)),
            Next::Left
        );
        assert_eq!(
            expected_next(&plan(Step::Installed, Step::Installed)),
            Next::Complete
        );
    }

    #[test]
    fn a_half_running_its_image_unconfirmed_is_already_where_the_board_wants_it() {
        // The board's ordering gate asks that a half be carrying what was
        // declared, not that it be keeping it, so a pair left unconfirmed by an
        // interrupted run reads as complete on both sides. This end has to agree
        // or the cross-check fires on every resumed update — which is what makes
        // re-running the same bundle a clean two-confirm pass.
        assert_eq!(
            expected_next(&plan(Step::Confirm, Step::Installed)),
            Next::Complete
        );
        assert_eq!(
            expected_next(&plan(Step::Installed, Step::Confirm)),
            Next::Complete
        );
        assert_eq!(
            expected_next(&plan(Step::Confirm, Step::Confirm)),
            Next::Complete
        );
        // And a half that still needs its image is still what goes first.
        assert_eq!(
            expected_next(&plan(Step::Upload, Step::Confirm)),
            Next::Right
        );
        assert_eq!(
            expected_next(&plan(Step::Confirm, Step::Upload)),
            Next::Left
        );
    }

    #[test]
    fn a_board_that_agrees_with_the_plan_is_not_worth_a_line() {
        assert_eq!(
            cross_check(&plan(Step::Upload, Step::Upload), Next::Right),
            None
        );
        assert_eq!(
            cross_check(&plan(Step::Installed, Step::Upload), Next::Left),
            None
        );
    }

    #[test]
    fn a_board_that_disagrees_with_the_plan_says_so() {
        // What a right half this end read as up to date but the board can see
        // has not confirmed looks like.
        let note = cross_check(&plan(Step::Installed, Step::Upload), Next::Right).unwrap();
        assert!(note.contains("the left half"), "{note}");
        assert!(note.contains("the board says otherwise"), "{note}");

        // And a step from a firmware newer than this build is a disagreement
        // like any other rather than something to guess at.
        assert!(cross_check(&plan(Step::Upload, Step::Upload), Next::Unknown(7)).is_some());
        assert!(cross_check(&plan(Step::Upload, Step::Upload), Next::None).is_some());
    }

    #[test]
    fn a_bundles_declaration_is_two_different_hashes_for_each_half() {
        let bundle = bundle::parse(&bundled()).unwrap();
        let declaration = declare(&bundle, false).unwrap();

        assert_eq!(declaration.version, "1.2.3-test");
        for half in [Half::Left, Half::Right] {
            let declared = match half {
                Half::Left => &declaration.left,
                Half::Right => &declaration.right,
            };
            // The trailer's hash, cut to the 32 the wire field holds, and the
            // whole file's — the second is what an upload announces in `sha`.
            assert_eq!(declared.slot_hash.len(), 32);
            assert_eq!(
                declared.slot_hash,
                bundle.hash(half).unwrap()[..32],
                "{half}"
            );
            assert_eq!(
                declared.upload_hash,
                smp::image::sha256(bundle.image(half)).to_vec(),
                "{half}"
            );
            assert_ne!(declared.slot_hash, declared.upload_hash, "{half}");
        }
        // The two halves are declared as two different images.
        assert_ne!(declaration.left.slot_hash, declaration.right.slot_hash);
    }

    #[test]
    fn the_declaration_never_disturbs_the_full_hash_the_plan_compares() {
        // The truncation is the group 67 declaration's alone: `step_for` still
        // compares the whole 64-byte trailer hash against what `image list`
        // returns, which is what decides whether a half needs writing at all.
        let bundle = bundle::parse(&bundled()).unwrap();
        assert_eq!(bundle.hash(Half::Left).unwrap().len(), 64);
        assert_eq!(declare(&bundle, false).unwrap().left.slot_hash.len(), 32);
    }

    #[test]
    fn forcing_an_update_is_part_of_what_is_declared() {
        // A board with both images already refuses every upload as finished,
        // so --force that only changed the local plan would be refused by the
        // keyboard and could not rewrite anything.
        let bundle = bundle::parse(&bundled()).unwrap();
        assert!(!declare(&bundle, false).unwrap().redo);
        assert!(declare(&bundle, true).unwrap().redo);
        // And it changes nothing else, so a re-declaration is still the same
        // statement.
        let plain = declare(&bundle, false).unwrap();
        let forced = declare(&bundle, true).unwrap();
        assert_eq!(plain.left, forced.left);
        assert_eq!(plain.right, forced.right);
        assert_eq!(plain.version, forced.version);
    }

    #[test]
    fn both_halves_carrying_the_bundles_images_is_not_worth_a_word() {
        let bundle = bundle::parse(&bundled()).unwrap();
        let running = [
            (Half::Left, bundle.hash(Half::Left).unwrap()),
            (Half::Right, bundle.hash(Half::Right).unwrap()),
        ];
        assert!(both_on_the_bundle(&bundle, &running).is_ok());
    }

    #[test]
    fn a_keyboard_left_with_its_halves_on_different_builds_is_reported() {
        // What a half-finished update leaves behind: one half took the
        // bundle's image and the other is still on what it had. Each half was
        // checked against the bundle on its own and neither check saw this.
        let bundle = bundle::parse(&bundled()).unwrap();
        let running = [
            (Half::Left, bundle.hash(Half::Left).unwrap()),
            (Half::Right, vec![0x99; 64]),
        ];

        let err = both_on_the_bundle(&bundle, &running).unwrap_err().to_string();
        let err = flat(&err);
        assert!(err.contains("The halves are on different builds"), "{err}");
        assert!(err.contains("the right half is carrying"), "{err}");
        assert!(!err.contains("the left half"), "{err}");
        assert!(err.contains("1.2.3-test"), "{err}");
        assert!(!err.contains("Run the same command again"), "{err}");
    }

    #[test]
    fn a_half_that_reports_nothing_running_is_named_rather_than_given_a_hash() {
        let bundle = bundle::parse(&bundled()).unwrap();
        let running = [
            (Half::Left, Vec::new()),
            (Half::Right, bundle.hash(Half::Right).unwrap()),
        ];
        let err = flat(&both_on_the_bundle(&bundle, &running).unwrap_err().to_string());
        assert!(err.contains("the left half is carrying nothing"), "{err}");
    }

    #[test]
    fn the_hash_a_half_is_running_is_the_active_slots() {
        let slots = vec![
            ImageSlot {
                slot: 1,
                hash: vec![0x01; 4],
                ..ImageSlot::default()
            },
            ImageSlot {
                slot: 0,
                hash: vec![0x02; 4],
                active: true,
                ..ImageSlot::default()
            },
        ];
        assert_eq!(active_hash(&slots), vec![0x02; 4]);
        assert!(active_hash(&[]).is_empty());
    }

    #[test]
    fn the_charge_line_says_unknown_rather_than_a_made_up_number() {
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Unknown,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery, false), "left 36% · right unknown");
        assert_eq!(
            charge(battery, true),
            "left 36% · right unknown · on the charger"
        );
    }

    /// A wrapped paragraph as one line, since where the fold lands is not what
    /// these are checking.
    fn flat(text: &str) -> String {
        text.split_whitespace().collect::<Vec<_>>().join(" ")
    }

    #[test]
    fn the_battery_gate_is_one_line_naming_no_level() {
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        let err = check_battery(battery, false).unwrap_err().to_string();
        assert_eq!(
            err,
            "Batteries low, please charge both halves to at least 30% to flash over Bluetooth."
        );
    }

    #[test]
    fn a_board_with_one_battery_is_still_caught() {
        let battery = Battery {
            left: Level::Absent,
            right: Level::Absent,
            keyboard: Level::Percent(4),
        };
        let err = check_battery(battery, false).unwrap_err().to_string();
        assert_eq!(
            err,
            "Batteries low, please charge both halves to at least 30% to flash over Bluetooth."
        );
    }

    #[test]
    fn a_flat_half_is_a_number_rather_than_nothing() {
        // 0% is a level like any other, and the gate has to see it as one.
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        assert_eq!(charge(battery, false), "left 0% · right 27%");
        assert!(check_battery(battery, false).is_err());
        assert!(check_battery(battery, true).is_ok(), "on the charger");
    }

    // ---- The whole run, against two in-memory halves ---------------------

    use std::sync::{Arc, Mutex};

    use zapp_smp::fake::{FakeDevice, FakeSlot, Mark};
    use zapp_smp::{session as group, SmpTransport, TransportError};

    use crate::Timings;

    /// One in-memory half, shared by every connection made to it.
    #[derive(Clone)]
    struct Shared(Arc<Mutex<FakeDevice>>);

    impl SmpTransport for Shared {
        fn mtu(&self) -> usize {
            self.0.lock().unwrap().mtu()
        }

        fn write(&mut self, packet: &[u8]) -> std::result::Result<(), TransportError> {
            self.0.lock().unwrap().write(packet)
        }

        fn read(
            &mut self,
            timeout: Duration,
        ) -> std::result::Result<Option<Vec<u8>>, TransportError> {
            self.0.lock().unwrap().read(timeout)
        }
    }

    /// Two halves, and what the battery gate reads.
    struct FakeBoard {
        left: Shared,
        right: Shared,
        battery: Battery,
        charging: bool,
    }

    impl Board for FakeBoard {
        fn open(&mut self, half: Half) -> std::result::Result<SmpClient, SmpError> {
            let channel = match half {
                Half::Left => self.left.clone(),
                Half::Right => self.right.clone(),
            };
            Ok(SmpClient::new(Box::new(channel)))
        }

        fn battery(&mut self) -> std::result::Result<Battery, SmpError> {
            Ok(self.battery)
        }

        fn charging(&mut self) -> bool {
            self.charging
        }
    }

    /// The hash a half is running before the update.
    const OLD: [u8; 64] = [0x99; 64];

    /// A half running `hash`, confirmed, with nothing staged.
    fn running_on(hash: &[u8]) -> Vec<FakeSlot> {
        vec![FakeSlot::new(0, hash).active().confirmed()]
    }

    /// A half back from its restart on `hash`, on a test boot nothing kept.
    fn testing(hash: &[u8]) -> Vec<FakeSlot> {
        vec![
            FakeSlot::new(0, hash).active(),
            FakeSlot::new(1, &OLD).confirmed(),
        ]
    }

    /// Both halves, reporting `left` and `right` read by read, and recording
    /// every state write in one list, so the order across the halves shows.
    fn board(left: Vec<Vec<FakeSlot>>, right: Vec<Vec<FakeSlot>>) -> (FakeBoard, Arc<Mutex<Vec<Mark>>>) {
        let marks = Arc::new(Mutex::new(Vec::new()));
        let half = |listings: Vec<Vec<FakeSlot>>| {
            let mut fake = FakeDevice::new(256);
            fake.listings = listings;
            fake.marks = Arc::clone(&marks);
            // The right is back and carrying its image whenever the left is
            // asked, which is all the wait before the left's upload reads.
            fake.next_step = Next::Left.code();
            Shared(Arc::new(Mutex::new(fake)))
        };
        let board = FakeBoard {
            left: half(left),
            right: half(right),
            battery: Battery::default(),
            charging: false,
        };
        (board, marks)
    }

    /// No waits: the in-memory halves are back the moment they are asked.
    fn at_once(force: bool) -> Options {
        Options {
            force,
            reply_timeout: Duration::from_millis(50),
            timings: Timings {
                reboot_settle: Duration::ZERO,
                reboot_timeout: Duration::ZERO,
                retry_every: Duration::ZERO,
                relay_settle: Duration::ZERO,
                ..Timings::default()
            },
            ..Options::default()
        }
    }

    /// Runs the update against `board`, with everything it reported.
    fn update_on(board: &mut FakeBoard, force: bool) -> (Result<Outcome>, Vec<Event>) {
        let bundle = bundle::parse(&bundled()).unwrap();
        let options = at_once(force);
        let mut said = Vec::new();
        let mut record = |event: Event| said.push(event);
        let mut reporter = Reporter::new(&mut record);
        let mut link = Link {
            board,
            options: &options,
            reporter: &mut reporter,
        };
        let result = update(&mut link, &bundle);
        (result, said)
    }

    fn phases(said: &[Event]) -> Vec<Phase> {
        said.iter()
            .filter_map(|event| match event {
                Event::Phase(phase) => Some(*phase),
                _ => None,
            })
            .collect()
    }

    fn left_hash() -> Vec<u8> {
        vec![0xa1; 64]
    }

    fn right_hash() -> Vec<u8> {
        vec![0xb2; 64]
    }

    #[test]
    fn a_keyboard_already_carrying_the_bundle_is_left_alone() {
        let (mut board, marks) = board(
            vec![running_on(&left_hash())],
            vec![running_on(&right_hash())],
        );
        let (result, _said) = update_on(&mut board, false);
        assert_eq!(result.unwrap(), Outcome::UpToDate);

        // Nothing declared, nothing written, nothing marked.
        for half in [&board.left, &board.right] {
            let fake = half.0.lock().unwrap();
            assert!(fake.uploads.lock().unwrap().is_empty());
            assert!(fake.session.is_none());
            assert!(
                !fake.requests.lock().unwrap().iter().any(|(g, _)| *g == group::GROUP),
                "a session command was sent"
            );
        }
        assert!(marks.lock().unwrap().is_empty());
    }

    #[test]
    fn both_halves_are_written_right_first_then_read_and_confirmed_right_first() {
        let (mut board, marks) = board(
            vec![running_on(&OLD), testing(&left_hash())],
            vec![running_on(&OLD), testing(&right_hash())],
        );
        let (result, said) = update_on(&mut board, false);
        assert_eq!(result.unwrap(), Outcome::Flashed);

        // Marked for test right then left, as each is written; confirmed right
        // then left, once both have been read back.
        assert_eq!(
            *marks.lock().unwrap(),
            vec![
                (right_hash(), false),
                (left_hash(), false),
                (right_hash(), true),
                (left_hash(), true),
            ]
        );
        assert_eq!(*board.right.0.lock().unwrap().resets.lock().unwrap(), 1);
        assert_eq!(*board.left.0.lock().unwrap().resets.lock().unwrap(), 1);

        assert_eq!(
            phases(&said),
            vec![
                Phase::Connecting,
                Phase::Reading,
                Phase::Declaring,
                Phase::Writing(Half::Right),
                Phase::Restarting(Half::Right),
                Phase::WaitingForRight,
                Phase::Writing(Half::Left),
                Phase::Restarting(Half::Left),
                Phase::Verifying,
                Phase::Confirming,
                Phase::Confirmed,
            ]
        );
        // Every wait that went up came down again.
        let up = said.iter().filter(|event| matches!(event, Event::Wait(_))).count();
        let down = said.iter().filter(|event| matches!(event, Event::WaitOver)).count();
        assert!(up >= down && down > 0, "{up} up, {down} down");
        assert!(matches!(said.iter().rev().find(|event| matches!(event, Event::Wait(_) | Event::WaitOver)), Some(Event::WaitOver)));
    }

    #[test]
    fn a_half_that_came_back_on_its_old_image_puts_both_back_and_confirms_nothing() {
        let (mut board, marks) = board(
            // The left's swap did not take: it comes back on what it had.
            vec![running_on(&OLD), running_on(&OLD)],
            // The right took it, and is back on its old image after the revert.
            vec![running_on(&OLD), testing(&right_hash()), running_on(&OLD)],
        );
        let (result, said) = update_on(&mut board, false);

        let err = classify(result.unwrap_err());
        assert_eq!(err.kind(), &ErrorKind::Reverted);
        assert!(
            err.message().starts_with("Flashing failed and reverted to the previous firmware; "),
            "{err}"
        );
        assert!(err.message().contains("The left half came back running"), "{err}");
        // Nothing was confirmed.
        assert!(marks.lock().unwrap().iter().all(|(_, confirm)| !confirm));
        // Both halves were told to restart, right first, and said so.
        let reverts: Vec<(Half, RevertStep)> = said
            .iter()
            .filter_map(|event| match event {
                Event::Reverting(half, step) => Some((*half, step.clone())),
                _ => None,
            })
            .collect();
        assert_eq!(
            reverts,
            vec![(Half::Right, RevertStep::Restarting), (Half::Left, RevertStep::Restarting)]
        );
    }

    #[test]
    fn a_half_that_did_not_come_back_on_its_old_image_is_named() {
        let (mut board, _marks) = board(
            vec![running_on(&OLD), running_on(&OLD)],
            // Still on the written image after the revert.
            vec![running_on(&OLD), testing(&right_hash())],
        );
        let (result, _said) = update_on(&mut board, false);

        let err = classify(result.unwrap_err());
        assert_eq!(
            err.kind(),
            &ErrorKind::NotReverted {
                halves: vec![Half::Right]
            }
        );
        assert_eq!(err.half(), Some(Half::Right));
        assert!(
            err.message().starts_with("Flashing failed, the right side did not revert; "),
            "{err}"
        );
    }

    #[test]
    fn a_flat_pack_stops_the_update_before_the_right_is_touched() {
        let (mut board, marks) = board(
            vec![running_on(&OLD)],
            vec![running_on(&OLD)],
        );
        board.battery = Battery {
            left: Level::Percent(12),
            right: Level::Percent(40),
            keyboard: Level::Percent(12),
        };
        let (result, _said) = update_on(&mut board, false);

        let err = classify(result.unwrap_err());
        assert_eq!(err.kind(), &ErrorKind::LowBattery);
        assert!(board.right.0.lock().unwrap().requests.lock().unwrap().is_empty());
        assert!(marks.lock().unwrap().is_empty());
    }

    #[test]
    fn force_on_a_keyboard_already_carrying_the_bundle_writes_but_never_marks_for_test() {
        let (mut board, marks) = board(
            vec![running_on(&left_hash())],
            vec![running_on(&right_hash())],
        );
        let (result, said) = update_on(&mut board, true);
        assert_eq!(result.unwrap(), Outcome::Flashed);

        for half in [&board.left, &board.right] {
            let fake = half.0.lock().unwrap();
            assert_eq!(*fake.uploads.lock().unwrap(), vec![0]);
            // Restarted as it is, out of its update state.
            assert_eq!(*fake.resets.lock().unwrap(), 1, "a half was not restarted");
        }
        // Nothing to swap to, so nothing is marked for test, and nothing is
        // confirmed either: both were already running their images confirmed.
        assert!(marks.lock().unwrap().is_empty());
        // The left's restart is what ends the session on a real board, so no
        // close is sent after it.
        let said = phases(&said);
        assert!(said.contains(&Phase::Restarting(Half::Right)), "{said:?}");
        assert!(said.contains(&Phase::Restarting(Half::Left)), "{said:?}");
    }

    #[test]
    fn a_right_half_that_cannot_be_read_is_the_radio_link() {
        let (mut board, _marks) = board(vec![running_on(&OLD)], vec![running_on(&OLD)]);
        // Nothing comes back over the relay.
        board.right.0.lock().unwrap().silent_over = Some(0);
        let (result, _said) = update_on(&mut board, false);

        let err = classify(result.unwrap_err());
        assert_eq!(err.kind(), &ErrorKind::RadioLinkDown);
        assert_eq!(err.half(), Some(Half::Right));
        assert!(err.message().starts_with("The halves' radio link is down: "), "{err}");
    }
}
