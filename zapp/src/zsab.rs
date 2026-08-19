//! Updating a whole keyboard from one `.zsab` bundle.
//!
//! Everything under this is already here: `zapp_ble::smp` has the client, the
//! upload loop and the relay that reaches the right half. This is the order to
//! do it in, what to check first, and what to say when it goes wrong — which is
//! the part a person needs, because the keyboard is unusable for the couple of
//! minutes it takes and a failure half-way leaves the two halves on different
//! firmware.

use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use indicatif::ProgressBar;

use zapp_ble::smp::bundle::{self, Bundle, Step};
use zapp_ble::smp::session::{self, Declaration, Next};
use zapp_ble::smp::zsa_err::{self, Cause, Side};
use zapp_ble::smp::{self, Half, ImageSlot, SmpClient, SmpError};
use zapp_ble::{Battery, Level};
use zapp_core::device::{self, ids::Keyboard};
use zapp_core::flash::FlashProgress;

use crate::{smp_connect, smp_connect_quiet, LinkArgs};

/// How long to leave a half alone after telling it to restart.
///
/// It has to actually go down before there is any point knocking, and BlueZ
/// will happily hand back a link that is on its way out.
const REBOOT_SETTLE: Duration = Duration::from_secs(5);

/// How long to keep trying to reach a half that is coming back.
///
/// Generous on purpose. A reset is a reboot, an MCUboot swap and a BLE
/// re-connect, and one failed attempt can cost most of a minute on its own:
/// BlueZ waits 20 s for a link it may hand back stale, and a request on a dead
/// one waits out --timeout after that. Three attempts have to fit.
const REBOOT_TIMEOUT: Duration = Duration::from_secs(180);

/// Between attempts to reach a half that has not answered yet.
const RETRY_EVERY: Duration = Duration::from_secs(3);

/// How many times to try to confirm a half before giving up on it.
///
/// Nothing else keeps the image, so this is worth retrying: the attempts are one
/// short command each, `RETRY_EVERY` apart.
const CONFIRM_TRIES: usize = 3;

/// How often a long wait says it is still a wait.
const NOTE_EVERY: Duration = Duration::from_secs(15);

/// A keyboard that already carries the bundle, said the same way whichever way
/// the bundle was going to travel.
pub(crate) const UP_TO_DATE: &str =
    "The keyboard is up to date, skipping the flash. Pass --force to write it anyway.";

/// Updates both halves from a bundle.
pub fn update(path: &Path, force: bool, link: &LinkArgs) -> Result<()> {
    let blob = std::fs::read(path).with_context(|| format!("Failed to read {}", path.display()))?;
    let bundle = bundle::parse(&blob)
        .with_context(|| format!("{} is not a bundle this build can read", path.display()))?;
    announce(&bundle, path, blob.len());

    let mut progress = Progress::default();
    match run(&bundle, force, link, &mut progress) {
        Ok(()) => Ok(()),
        Err(err) => {
            // Before anything is printed, while a connection is still likely to
            // be there: a session left open holds the next attempt out until it
            // expires, and closing it costs one round trip.
            close_session(link, &progress);

            println!();
            println!("Update failed: {err:#}");
            if let Some(note) = refusal_note(&err, progress.battery, progress.charging) {
                println!();
                println!("{note}");
            }
            println!();
            println!("{}", progress.advice(path));
            bail!("the keyboard was not fully updated")
        }
    }
}

/// What has been done to the keyboard so far.
///
/// Kept as it goes rather than worked out afterwards, because the message a
/// failure needs is entirely a question of how far it got.
#[derive(Debug, Default, Clone, Copy)]
struct Progress {
    /// The update has been declared to the left half, so the board knows what
    /// is being installed and will admit the writes that follow.
    declared: bool,
    /// The id the left gave that session, for the close that ends it.
    session_id: Option<u32>,
    /// An image has been put into a spare slot, marked or not.
    written: bool,
    right_restarted: bool,
    /// The right came back from that restart carrying what was declared, which
    /// is what the board wants to see before the left is written.
    right_back: bool,
    left_restarted: bool,
    /// The left still needs its image — so a restarted right is a right that
    /// is talking a protocol the left does not know.
    left_pending: bool,
    /// The right still needs its image, which is what the left refuses to be
    /// written before.
    right_pending: bool,
    /// Both halves are back on their new images and only the confirm is left.
    settled: bool,
    /// The two halves came out of this on different builds.
    mismatched: bool,
    /// Nothing was kept and both halves were put back on what they were
    /// running, which was seen to work.
    reverted: bool,
    /// Putting them back was tried and at least one half was not seen back on
    /// its old image afterwards.
    revert_failed: bool,
    /// What the packs read at the last look, and whether the keyboard was on
    /// the charger. Kept here so a refusal that turns out to be about the
    /// battery can quote real numbers rather than the threshold alone; both
    /// halves stay unknown until the levels have been asked for.
    battery: Battery,
    charging: bool,
}

impl Progress {
    /// What the keyboard is, and what to do about it.
    fn advice(&self, path: &Path) -> String {
        let rerun = format!("zapp flash {}", path.display());

        if !self.written {
            if self.declared {
                return format!(
                    "The update was declared to the keyboard, but nothing was written. Both\n\
                     halves are still running the firmware they were and the keyboard works\n\
                     exactly as it did.\n\n\
                     The session the left is holding lives in its memory and is dropped on\n\
                     any restart, so there is nothing to clean up. If it simply expired, that\n\
                     is all that happened: re-run `{rerun}`, which declares the same update\n\
                     again rather than starting a second one."
                );
            }
            return "Nothing was written. The keyboard is untouched and still working."
                .to_string();
        }

        if !self.right_restarted && !self.left_restarted {
            let order = if self.right_pending {
                "\n\nThe right half has to be updated first, and the left will refuse its own\n\
                 image until the right is back and carrying what was declared.\n\
                 Check the right half is switched on, charged and near the left."
            } else {
                ""
            };
            return format!(
                "An image was written into a spare slot, but nothing was marked and nothing\n\
                 was restarted. Both halves are still running the firmware they were, and\n\
                 the keyboard works exactly as it did.\n\n\
                 If the keyboard refused because the update session expired, nothing is\n\
                 lost: re-run `{rerun}` and it picks up from what is already installed.\
                 {order}"
            );
        }

        if self.revert_failed {
            return format!(
                "The update did not work out and this could not put the keyboard back the way\n\
                 it found it. At least one half was not seen back on its old image, so the two\n\
                 may be on different builds — in that state they cannot talk to each other and\n\
                 the keyboard is half working.\n\n\
                 Power-cycle the keyboard: a half that never confirmed its new image puts the\n\
                 old one back on its next start, whether or not this could reach it. Then\n\
                 re-run `{rerun}` — it reads back what each half is carrying before it writes\n\
                 anything — before trying anything else."
            );
        }

        if self.reverted {
            return format!(
                "Nothing was kept. Both halves were put back on the firmware they were running\n\
                 before this started, and both were seen back on it, so the keyboard works\n\
                 exactly as it did.\n\n\
                 An image is kept only once this has confirmed it, and it confirms nothing\n\
                 until both halves are back and carrying what was written — so a run that\n\
                 fails anywhere before that leaves the pair as it was, together.\n\n\
                 Re-run `{rerun}` to try again."
            );
        }

        if self.mismatched {
            return format!(
                "The two halves are not on the same build. In that state they cannot talk to\n\
                 each other, so the keyboard is half working: the halves may not see each\n\
                 other at all, and the right may not type or light up.\n\n\
                 Re-run `{rerun}` to put both halves on the same build. It reads back\n\
                 what each half is carrying before it writes anything, and prints it."
            );
        }

        if self.settled {
            return format!(
                "Both halves are running the new firmware, but at least one was not confirmed:\n\
                 an image is kept only once this has confirmed it, and a half left unconfirmed\n\
                 puts its old firmware back on the next restart.\n\n\
                 Re-run `{rerun}` — a half already running its image is confirmed rather than\n\
                 uploaded again, so this finishes in one pass, and the run says which half was\n\
                 still pending."
            );
        }

        if self.right_restarted && !self.right_back && !self.left_restarted && self.left_pending {
            return format!(
                "The right half was written and restarted, and it did not come back where\n\
                 this could see it. The left was not touched, so it is still running the\n\
                 firmware it was — and the keyboard is half working until the right is\n\
                 back, because the two are on different builds.\n\n\
                 The right was only marked for test and nothing confirmed it, so it puts its\n\
                 old firmware back the next time it restarts. Re-running finishes the pair\n\
                 if the right can be reached again.\n\n\
                 Check the right half is switched on, charged and near the left, then\n\
                 power-cycle the keyboard and re-run `{rerun}`."
            );
        }

        if self.right_restarted && !self.left_restarted {
            if self.left_pending {
                return format!(
                    "The right half is running the new firmware and the left is still on the\n\
                     old one. The two are on different protocol versions, so the keyboard is\n\
                     half working: the halves may not talk to each other at all, and the right\n\
                     may not type or light up. That is what a half-finished update looks like,\n\
                     not a broken keyboard.\n\n\
                     The right was only marked for test, and it keeps that image only once\n\
                     this has confirmed it — which does not happen until both halves are\n\
                     back on the same build. As things stand it puts its old firmware back\n\
                     the next time it restarts.\n\n\
                     Either:\n  \
                     · re-run `{rerun}` to finish the update, or\n  \
                     · power-cycle the keyboard to put the right half back as it was."
                );
            }
            return format!(
                "The right half was restarted onto the new firmware and the left was already\n\
                 carrying its image, so the two are on the same build. The right keeps that\n\
                 image only once this has confirmed it, and this run did not get that far, so\n\
                 it reverts on the next restart.\n\n\
                 Re-run `{rerun}` to finish, or power-cycle the keyboard to put the right\n\
                 half back as it was."
            );
        }

        format!(
            "The halves were written, marked for test and restarted, and at least one did\n\
             not come back where this could see it. A new image is kept only once this has\n\
             confirmed it, and it confirms nothing it cannot see, so an unreachable half\n\
             puts its old firmware back on the next restart.\n\n\
             Power-cycle the keyboard to get a working pair back, then re-run `{rerun}`."
        )
    }
}

fn run(bundle: &Bundle, force: bool, link: &LinkArgs, progress: &mut Progress) -> Result<()> {
    // ---- What is there now ----------------------------------------------
    log::debug!("reading what is installed");

    let left_slots = read_state(link, Half::Left, Connect::Announced)?;
    // Levels are only published over a live connection, so this is asked after
    // the left has been talked to rather than before. A failure here is an
    // unknown level, which the gate allows.
    let battery = smp::battery(link.device.as_deref()).unwrap_or_else(|err| {
        log::debug!("could not read the battery levels: {err}");
        Battery::default()
    });
    let charging = charging();
    progress.battery = battery;
    progress.charging = charging;
    log::debug!("charge {}", charge(battery, charging));
    // Before the right half is touched, not after: reading the right goes
    // through the left's relay, and the relay is battery-gated too. On a flat
    // pack that read is the thing that fails, and it fails looking like a
    // radio-link fault — so the check that can actually name the battery has
    // to run first.
    check_battery(battery, charging)?;
    let right_slots = read_state(link, Half::Right, Connect::Announced).context(
        "The right half is reached through the left, over the halves' own radio link, and \
         that link has to be up before anything is written — updating the left first would \
         cut off the only route to the right. Check both halves are powered and linked",
    )?;

    // What each half would need if nobody had asked for a rewrite. Kept even
    // under --force, because a half that is already running the bundle's image
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
        println!("{UP_TO_DATE}");
        return Ok(());
    }
    let pending = |wanted: Half| {
        plan.iter()
            .any(|(half, step)| *half == wanted && *step != Step::Installed)
    };
    progress.left_pending = pending(Half::Left);
    progress.right_pending = pending(Half::Right);

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
    let declaration = declare(bundle, force)?;
    let opened = open_session(link, &declaration)?;
    progress.declared = true;
    progress.session_id = Some(opened.id);
    announce_session(&declaration, &opened);
    // Not under --force. There the plan writes both halves whatever they are
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
        let image = bundle.image(half);
        let hash = bundle
            .hash(half)
            .with_context(|| format!("The bundle's {half} image carries no hash to mark it by"))?;

        // The left is not writable until the right is back and reporting the
        // image it was given: the board refuses the left's own upload with "the
        // right half first" until then, and the right has only just restarted.
        if half == Half::Left && progress.right_restarted {
            wait_for_right_ready(link, &declaration)?;
            progress.right_back = true;
        }

        log::debug!("{} half — upload ({} KiB)", titled(half), image.len() / 1024);
        let (mut client, renewed) = connect_declared(link, half, &declaration)?;
        progress.session_id = Some(renewed.id);

        // Set before the upload, not after: an upload that fails part-way has
        // still put bytes in the spare slot, and the message for that is the
        // one about a slot written and nothing marked.
        progress.written = true;
        upload(&mut client, half, image)?;
        // The transfer was still worth doing under --force — it is the one part
        // of this that proves the link end to end — but there is nothing to
        // swap to, so the mark that would fail is not sent.
        if !swaps(half) {
            log::debug!("{half} already runs this exact image — nothing to swap");
            continue;
        }
        restart(&mut client, half, &hash)?;
        match half {
            Half::Right => progress.right_restarted = true,
            Half::Left => progress.left_restarted = true,
        }
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
    let keep: Vec<Half> = plan
        .iter()
        .filter(|(_, step)| *step != Step::Installed)
        .map(|(half, _)| *half)
        .collect();

    progress.settled = true;
    {
        let mut pair = BleConfirm {
            link,
            bundle,
            progress: &mut *progress,
            running,
            before,
            spinner: None,
        };
        let settled = crate::confirm::settle(&mut pair, &read, &keep);
        // Whatever it came to, the line the spinner owns is given back before
        // anything else is said.
        pair.clear();
        settled?;
    }

    // ---- Let the session go ---------------------------------------------
    //
    // Best effort, and quiet about it. If the left restarted, its own reboot
    // already dropped the session: it lives in the left's memory and the
    // firmware writes none of it to flash, which is the design rather than
    // something that went wrong.
    close_session(link, progress);

    println!("Done!");
    Ok(())
}

/// What this bundle declares: both halves' images, by both of the hashes
/// mcumgr names an image with, and the build they came from.
///
/// `force` carries straight through to the declaration. Without it a board that
/// already has both images refuses every upload as finished, which is the right
/// answer to "install this" and the wrong one to "install this again" — so
/// --force has to be part of what is declared, not just of the local plan.
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
fn open_session(link: &LinkArgs, declaration: &Declaration) -> Result<session::Status> {
    let mut client = smp_connect(link, Half::Left)?;
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
fn connect_declared(
    link: &LinkArgs,
    half: Half,
    declaration: &Declaration,
) -> Result<(SmpClient, session::Status)> {
    if half == Half::Left {
        thread::sleep(RELAY_SETTLE);
    }
    let mut left = smp_connect(link, Half::Left)?;
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
            Ok((smp_connect(link, half)?, status))
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
fn wait_for_right_ready(link: &LinkArgs, declaration: &Declaration) -> Result<()> {
    let spinner = crate::new_spinner("Waiting for the right half to restart...");
    let ready = right_ready(link, declaration);
    spinner.finish_and_clear();
    ready
}

/// The wait itself, with the spinner's line left to its caller.
fn right_ready(link: &LinkArgs, declaration: &Declaration) -> Result<()> {
    thread::sleep(REBOOT_SETTLE);

    let mut client = smp_connect_quiet(link, Half::Left)?;
    let started = Instant::now();
    let mut noted = Instant::now();
    // Whether the right was ever reporting at all. A right that never says
    // anything is a right that did not come back, which is a different thing
    // to tell someone than one that came back on the wrong image.
    let mut seen = false;

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
            }
            Err(err) => {
                log::debug!("the left stopped answering while waiting on the right: {err:#}");
                client = smp_connect_quiet(link, Half::Left)?;
            }
        }

        if started.elapsed() >= REBOOT_TIMEOUT {
            break;
        }
        if noted.elapsed() >= NOTE_EVERY {
            noted = Instant::now();
            log::debug!(
                "right: not back yet after {} s of {}",
                started.elapsed().as_secs(),
                REBOOT_TIMEOUT.as_secs()
            );
        }
        thread::sleep(RETRY_EVERY);
    }

    let waited = REBOOT_TIMEOUT.as_secs();
    if seen {
        bail!(
            "The right half came back within {waited} s but is not carrying the image that \
             was written to it, so the left half was left alone. MCUboot put its old image \
             back, which means the new one did not boot."
        )
    }
    bail!(
        "The right half did not come back within {waited} s of its restart. The left was \
         not touched and is still running the firmware it was. Check the right half is \
         switched on, charged and near the left."
    )
}

/// Ends the session, if there is one left to end.
///
/// Best effort by design, and never a reason to fail an update that worked. A
/// left half that has restarted has already dropped it, and a board that will
/// not answer here is one that has gone quiet after the last thing that mattered
/// was already done.
fn close_session(link: &LinkArgs, progress: &Progress) {
    let Some(id) = progress.session_id else {
        return;
    };
    if progress.left_restarted {
        log::debug!("the left restarted, so update session {id} is already gone");
        return;
    }

    let closed = smp_connect(link, Half::Left)
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
        "  this run planned {} first, which is not what the board says.\n  \
         The board's own view is what it enforces, so it is the one to follow.",
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
/// --force but never marked. MCUboot refuses to mark the slot a board is
/// running from for test — image manager rc 33,
/// `IMG_MGMT_ERR_IMAGE_SETTING_TEST_TO_ACTIVE_DENIED`, from `!confirm && slot ==
/// active_slot` — and it is right to, because there is nothing to swap to. A
/// mark carries a hash, and a hash names an image rather than a slot, so a board
/// with the same build in both slots resolves it to the active one and says no.
/// Flashing the running build with --force does that every time; a real update
/// never can, because its hash is not the one the board is running.
///
/// Only reachable under --force. Without it such a half reads as
/// [`Step::Installed`] and the run stops before any session is opened.
fn swaps_after(planned: Step, found: Step) -> bool {
    planned == Step::Upload && found != Step::Installed
}

fn restarted(progress: Progress, half: Half) -> bool {
    match half {
        Half::Left => progress.left_restarted,
        Half::Right => progress.right_restarted,
    }
}

/// The left holds one DFU lock for both halves, and relaying to the right takes
/// it for the whole session — which lingers a few seconds past the last frame so
/// a multi-frame exchange is not cut in half. Anything that touched the right,
/// including merely reading its image state, therefore blocks the left's own
/// upload with "busy" until that lapses. Wait it out rather than racing it.
const RELAY_SETTLE: Duration = Duration::from_secs(5);

/// Sends one half's image into its spare slot, over a connection the caller has
/// already declared the update on.
fn upload(client: &mut SmpClient, half: Half, image: &[u8]) -> Result<()> {
    let bar = crate::flash_bar(100, titled(half));
    let result = client.upload(0, image, &|progress| match progress {
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
/// Wording matches the wired path in [`crate::nrf_wired`], which makes the same
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
        "The keyboard did not come out of this on one build: {}, and the bundle is {}. Both \
         halves have to be on the same build or they cannot talk to each other — in that \
         state the keyboard is half working. Run the same command again to put both halves \
         on the bundle's build.",
        wrong.join(", "),
        named(&bundle.header.build_version)
    )
}

/// What a restarted half came back as.
///
/// `pub(crate)` because the wired path in [`crate::nrf_wired`] reaches the same
/// three answers through its own transport and hands them to the same
/// [`decide`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Landed {
    /// Reachable, and running the image that was written.
    Ok { confirmed: bool, active: Vec<u8> },
    /// Reachable, running something else — MCUboot put the old image back, or
    /// it never had anything running at all.
    Wrong { active: Vec<u8>, version: String },
    /// Not reachable inside [`REBOOT_TIMEOUT`].
    Gone { why: String },
}

impl Landed {
    /// The one sentence this verdict is worth.
    pub(crate) fn why(&self, half: Half) -> String {
        match self {
            Landed::Ok { active, .. } => {
                format!("The {half} half came back running {}.", short(active))
            }
            Landed::Wrong { active, .. } if active.is_empty() => {
                format!("The {half} half came back but reports nothing running.")
            }
            Landed::Wrong { active, version } => format!(
                "The {half} half came back running {} ({}), not the image that was written. \
                 MCUboot put the old image back, which means the new one did not boot.",
                short(active),
                named(version)
            ),
            Landed::Gone { why } => why.clone(),
        }
    }
}

/// Waits for a half to come back, and says what it came back as.
///
/// None of the three answers is a reason to abort on its own: whether a half
/// that is missing or on the wrong image is fatal depends on the other half too,
/// and that decision is [`decide`]'s. So this reports rather than fails.
fn wait_for_boot(link: &LinkArgs, half: Half, hash: &[u8]) -> Landed {
    thread::sleep(REBOOT_SETTLE);

    let started = Instant::now();
    loop {
        let slots = match read_state(link, half, Connect::Quiet) {
            Ok(slots) => slots,
            Err(err) if started.elapsed() < REBOOT_TIMEOUT => {
                log::debug!(
                    "{half} half not back yet after {} s: {err:#}",
                    started.elapsed().as_secs()
                );
                thread::sleep(RETRY_EVERY);
                continue;
            }
            Err(err) => {
                log::debug!("{half} half never came back: {err:#}");
                return Landed::Gone {
                    why: format!(
                        "The {half} half did not come back within {} s of its restart.",
                        REBOOT_TIMEOUT.as_secs()
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
fn confirm(link: &LinkArgs, half: Half, hash: &[u8], already: bool) -> Result<()> {
    if already {
        log::debug!("{half}: already confirmed");
        return Ok(());
    }

    let mut last = None;
    for attempt in 1..=CONFIRM_TRIES {
        let marked =
            smp_connect_quiet(link, half).and_then(|mut client| Ok(client.mark(hash, true)?));
        match marked {
            Ok(_) => {
                log::debug!("{half}: confirmed");
                return Ok(());
            }
            Err(err) => {
                log::debug!("could not confirm the {half} half (try {attempt}): {err:#}");
                if attempt < CONFIRM_TRIES {
                    thread::sleep(RETRY_EVERY);
                }
                last = Some(err);
            }
        }
    }

    Err(last.unwrap_or_else(|| anyhow::anyhow!("nothing was tried")))
        .with_context(|| format!("Could not confirm the {half} half's image"))
}

/// The Bluetooth side of [`crate::confirm::PairConfirm`].
///
/// `running` is the wait-for-reboot read the run already did; `commit` is the
/// `mark(hash, true)` that keeps an image. The right is confirmed before the
/// left — everything to the right goes through the left's relay, so if the
/// right cannot be kept the left is still unconfirmed and still revertable.
struct BleConfirm<'a> {
    link: &'a LinkArgs,
    bundle: &'a Bundle,
    progress: &'a mut Progress,
    /// What each half is carrying, replaced for every half that comes back from
    /// a restart. The mismatch check reads this.
    running: Vec<(Half, Vec<u8>)>,
    /// The same, kept as it was read at the top of the run: what a revert puts
    /// back.
    before: Vec<(Half, Vec<u8>)>,
    /// One spinner for the whole settle, its message swapped as the phase
    /// changes. Nothing else may print while it holds the line.
    spinner: Option<ProgressBar>,
}

impl BleConfirm<'_> {
    /// Puts the spinner on `message`, starting it if this is the first phase.
    fn saying(&mut self, message: &str) {
        match &self.spinner {
            Some(spinner) => spinner.set_message(message.to_string()),
            None => self.spinner = Some(crate::new_spinner(message)),
        }
    }

    /// Gives the line back, so something else can be said on it.
    fn clear(&mut self) {
        if let Some(spinner) = self.spinner.take() {
            spinner.finish_and_clear();
        }
    }
}

impl crate::confirm::PairConfirm for BleConfirm<'_> {
    fn running(&mut self, half: Half) -> Landed {
        self.saying("Waiting for the halves to restart...");
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
        let mismatch = both_on_the_bundle(self.bundle, &self.running)
            .err()
            .map(|err| err.to_string());
        self.progress.mismatched = mismatch.is_some();
        mismatch
    }

    fn commit(&mut self, half: Half) -> Result<()> {
        let hash = self
            .bundle
            .hash(half)
            .with_context(|| format!("The bundle's {half} image carries no hash"))?;
        confirm(self.link, half, &hash, false)
    }

    fn revert(&mut self, half: Half) -> Result<()> {
        smp_connect(self.link, half).and_then(|mut client| Ok(client.reset()?))
    }

    fn keeping(&mut self) {
        self.saying("Confirming both halves...");
    }

    /// Puts both halves back on what they were running, and says so.
    ///
    /// Nothing has been confirmed at this point, so a reset is the whole of the
    /// revert: MCUboot puts the old image back on an unconfirmed one. A half
    /// that is [`Landed::Gone`] cannot be reset from here and is named in the
    /// message instead — a power cycle reverts it just the same.
    fn revert_pair(&mut self, landed: Vec<(Half, Landed)>, reasons: Vec<String>) -> anyhow::Error {
        // Nothing was restarted, so nothing was staged and there is nothing to
        // put back — the halves are as they were found. Only reachable from a
        // pair that read as mismatched before anything was touched.
        if landed.is_empty() {
            return anyhow::anyhow!("{}", reasons.join(" "));
        }

        // Nothing below shares a line with a spinner.
        self.clear();
        println!();
        println!("Putting both halves back on the firmware they were running");
        for reason in &reasons {
            println!("  {}", wrap(reason).replace('\n', "\n  "));
        }

        // Right then left, the same order everything else touches them in, with
        // the relay's lock given time to lapse between the two.
        for half in [Half::Right, Half::Left] {
            let Some((_, verdict)) = landed.iter().find(|(which, _)| *which == half) else {
                continue;
            };
            if matches!(verdict, Landed::Gone { .. }) {
                println!("  {half:<5}: cannot be reached, so it cannot be reset from here");
                continue;
            }
            match self.revert(half) {
                Ok(()) => println!("  {half:<5}: restarting, which puts its old firmware back"),
                Err(err) => println!("  {half:<5}: could not be restarted ({err:#})"),
            }
            if half == Half::Right {
                thread::sleep(RELAY_SETTLE);
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
            self.saying("Waiting for the halves to restart...");
            let back = wait_for_boot(self.link, half, &was);
            self.clear();
            if !matches!(back, Landed::Ok { .. }) {
                println!("  {}", wrap(&back.why(half)).replace('\n', "\n  "));
            }
            restored.push((half, matches!(back, Landed::Ok { .. })));
        }

        let summary = reasons.join(" ");
        if restored.iter().all(|(_, back)| *back) {
            self.progress.reverted = true;
            return anyhow::anyhow!(
                "The update did not work out, so both halves were put back on the firmware they \
                 were running before it. The keyboard is working and is exactly as it was; \
                 nothing was kept. {summary}"
            );
        }

        self.progress.revert_failed = true;
        let where_each: Vec<String> = restored
            .iter()
            .map(|(half, back)| match back {
                true => format!("the {half} half is back on the firmware it had"),
                false => format!("the {half} half could not be seen back on it"),
            })
            .collect();
        anyhow::anyhow!(
            "The update did not work out, and putting both halves back did not finish either: {}. \
             The keyboard may be on two different builds, in which case the halves cannot talk to \
             each other. {summary}",
            where_each.join(", ")
        )
    }
}

/// Keep both halves, or put both back.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Decision {
    Keep,
    Revert { reasons: Vec<String> },
}

/// What to do with a pair that has come back.
///
/// Pure on purpose: this is the one decision the whole run turns on, and the
/// only part of it that can be checked without a keyboard. One half short of
/// perfect is a revert — a keyboard is a pair or it is nothing. `mismatch` is
/// [`both_on_the_bundle`]'s complaint, a revert case like any other, since two
/// halves on two builds cannot talk to each other.
pub(crate) fn decide(landed: &[(Half, Landed)], mismatch: Option<String>) -> Decision {
    let mut reasons: Vec<String> = landed
        .iter()
        .filter(|(_, verdict)| !matches!(verdict, Landed::Ok { .. }))
        .map(|(half, verdict)| verdict.why(*half))
        .collect();
    reasons.extend(mismatch);

    match reasons.is_empty() {
        true => Decision::Keep,
        false => Decision::Revert { reasons },
    }
}

/// Whether a connection announces itself.
///
/// One line, one spinner: a connect made while the run is already saying what
/// it is waiting for has to keep quiet, or the two redraw over each other.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Connect {
    /// Nothing else is on the line, so the wait for the board is worth a
    /// spinner of its own — a bonded but idle board takes seconds to wake.
    Announced,
    /// The caller is already holding the line.
    Quiet,
}

/// One image-state read, over a session opened for it.
fn read_state(link: &LinkArgs, half: Half, saying: Connect) -> Result<Vec<ImageSlot>> {
    let mut client = match saying {
        Connect::Announced => smp_connect(link, half)?,
        Connect::Quiet => smp_connect_quiet(link, half)?,
    };
    client
        .image_state()
        .with_context(|| format!("Could not read the {half} half's image state"))
}

/// Refuses an update a pack cannot see through.
///
/// A level nobody can read is allowed, deliberately: see `bundle::battery_allows`.
fn check_battery(battery: Battery, charging: bool) -> Result<()> {
    let halves = [
        ("left", battery.left.percent()),
        ("right", battery.right.percent()),
    ];
    let mut low: Vec<String> = halves
        .iter()
        .filter(|(_, level)| !bundle::battery_allows(*level, charging))
        .map(|(half, level)| format!("the {half} at {}%", level.unwrap_or(0)))
        .collect();
    // The combined instance is the lower of the two halves, so naming it as
    // well would report the same flat pack twice. It is only worth naming on
    // its own — which is what a board with one battery publishes.
    if low.is_empty() && !bundle::battery_allows(battery.keyboard.percent(), charging) {
        low.push(format!(
            "the keyboard at {}%",
            battery.keyboard.percent().unwrap_or(0)
        ));
    }
    if low.is_empty() {
        return Ok(());
    }

    bail!(
        "{} — under the {}% an update needs.\n\n{}",
        sentence(&low),
        bundle::MIN_SOC_PERCENT,
        wrap(
            "An update is minutes of radio and a reboot in which the firmware is rewritten, \
             and a half that runs out part-way has to be recovered over a cable. Charge the \
             keyboard, or plug it in and try again — on the charger this check does not \
             apply."
        )
    );
}

/// "the left at 0%" / "the left at 0% and the keyboard at 0%" / a comma list.
fn sentence(parts: &[String]) -> String {
    match parts {
        [] => String::new(),
        [only] => format!("The battery is low: {only}"),
        [first, second] => format!("The batteries are low: {first} and {second}"),
        [rest @ .., last] => format!("The batteries are low: {} and {last}", rest.join(", ")),
    }
}

/// The measured levels, named one by one, so a half nobody could read is not
/// passed off as a half that answered.
fn levels_line(battery: Battery, charging: bool) -> String {
    let level = |level: Level| match level {
        Level::Percent(level) => format!("{level}%"),
        Level::Unknown => "would not say".to_string(),
        Level::Absent => "did not report".to_string(),
    };
    format!(
        "The left reads {}, the right {}{}.",
        level(battery.left),
        level(battery.right),
        if charging {
            ", and the keyboard is on the charger"
        } else {
            ""
        }
    )
}

/// The firmware's own reason for saying no, in plain sentences, with the
/// measured levels spliced in when the reason is a battery.
///
/// The board answers a refusal with a ZSA code (see
/// `zapp_ble::smp::zsa_err`), and the one line the error chain gives it is a
/// summary rather than an answer. `None` for anything that is not a refusal
/// this build knows — including a code from a firmware newer than this zapp,
/// which is left to the raw group and number in the error itself rather than
/// dressed up as something it might not be.
fn refusal_note(err: &anyhow::Error, battery: Battery, charging: bool) -> Option<String> {
    let refusal = refusal(err)?;
    let advice = wrap(refusal.advice);

    let measured = match refusal.cause {
        Cause::BatteryLow(side) | Cause::SwapHeld(side) => {
            let (named, reading) = match side {
                Side::Left => ("left", battery.left),
                Side::Right => ("right", battery.right),
            };
            Some(match reading {
                Level::Percent(_) => format!(
                    "{} The threshold is {}%.",
                    levels_line(battery, charging),
                    bundle::MIN_SOC_PERCENT
                ),
                // The board refused on its own measurement and this end has
                // none, so say that rather than print a half as flat when
                // nobody here could read it.
                _ => format!(
                    "{} The threshold is {}%, and the keyboard refused on its own reading \
                     of the {named} half — this end could not read a level for it at all.",
                    levels_line(battery, charging),
                    bundle::MIN_SOC_PERCENT
                ),
            })
        }
        _ => None,
    };

    Some(match measured {
        Some(measured) => format!("{advice}\n\n{}", wrap(&measured)),
        None => advice,
    })
}

fn refusal(err: &anyhow::Error) -> Option<zsa_err::Refusal> {
    err.chain()
        .find_map(|cause| cause.downcast_ref::<SmpError>())
        .and_then(|smp| match smp {
            SmpError::Remote { group, rc, .. } => zsa_err::lookup(*group, *rc),
            _ => None,
        })
}

/// Folds a paragraph to the width the rest of this file is written at.
pub(crate) fn wrap(text: &str) -> String {
    const WIDTH: usize = 76;

    let mut out = String::new();
    let mut line = 0;
    for word in text.split_whitespace() {
        if line > 0 && line + 1 + word.chars().count() > WIDTH {
            out.push('\n');
            line = 0;
        } else if line > 0 {
            out.push(' ');
            line += 1;
        }
        out.push_str(word);
        line += word.chars().count();
    }
    out
}

/// Whether the pack is on the charger.
///
/// The nRF publishes no charging state over Bluetooth, so the only thing that
/// says so from here is the wire: a Voyager 2 on USB is one with VBUS present,
/// which is exactly what the firmware's own gate tests. A second ZSA board on
/// the same machine would read as charging, which errs towards letting an
/// update run — the direction this whole check leans anyway.
fn charging() -> bool {
    device::list_devices().is_ok_and(|devices| {
        devices
            .iter()
            .any(|device| device.keyboard == Some(Keyboard::Voyager2))
    })
}

/// Both halves' images, right first — the order they are written in.
///
/// `pub(crate)` so the wired path in [`crate::nrf_wired`] says the same thing
/// the same way.
pub(crate) fn sizes(bundle: &Bundle) -> String {
    [Half::Right, Half::Left]
        .iter()
        .map(|half| format!("{} {} KiB", half.as_str(), bundle.image(*half).len() / 1024))
        .collect::<Vec<_>>()
        .join(" + ")
}

fn announce(bundle: &Bundle, path: &Path, len: usize) {
    println!(
        "Bundle loaded: {} ({}) flashing over Bluetooth",
        named(&bundle.header.build_version),
        sizes(bundle)
    );
    log::debug!("{} · {len} bytes", path.display());
    for half in [Half::Left, Half::Right] {
        let image = bundle.image(half);
        let version = smp::image::inspect(image)
            .map(|info| info.version)
            .unwrap_or_else(|| "no MCUboot header".to_string());
        log::debug!(
            "{:<5}  v{version} · {} bytes · {}",
            half.as_str(),
            image.len(),
            bundle
                .hash(half)
                .map(|hash| short(&hash))
                .unwrap_or_else(|| "no hash".to_string())
        );
    }
    // Nothing stamps the fingerprint yet: shared/cmake/zsa_esb_key.cmake is not
    // wired into either app, so every bundle built today carries zero and it is
    // read as "unknown" rather than as a mismatch.
    log::debug!("radio key fingerprint {:08x}", bundle.header.key_fingerprint);
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

fn titled(half: Half) -> &'static str {
    match half {
        Half::Left => "Left",
        Half::Right => "Right",
    }
}

/// Enough of a hash to recognise, since a SHA-512 is 128 characters of hex.
pub(crate) fn short(hash: &[u8]) -> String {
    let hex = smp::image::hex(hash);
    match hex.len() > 20 {
        true => format!("{}…{}", &hex[..8], &hex[hex.len() - 8..]),
        false => hex,
    }
}

pub(crate) fn named(version: &str) -> String {
    match version.is_empty() {
        true => "(no version)".to_string(),
        false => version.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn advice_of(progress: Progress) -> String {
        progress.advice(Path::new("fw.zsab"))
    }

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

    #[test]
    fn a_failure_before_anything_was_written_says_the_keyboard_is_fine() {
        let advice = advice_of(Progress::default());
        assert!(advice.contains("untouched"), "{advice}");
    }

    #[test]
    fn an_upload_that_was_never_marked_leaves_the_keyboard_as_it_was() {
        let advice = advice_of(Progress {
            written: true,
            ..Progress::default()
        });
        assert!(advice.contains("still running the firmware they were"), "{advice}");
        assert!(advice.contains("zapp flash fw.zsab"), "{advice}");
    }

    #[test]
    fn a_right_that_restarted_without_the_left_says_the_halves_disagree() {
        // The case that matters: half-working keyboard, and two ways out of it.
        // The right came back from its restart — the left is only written once
        // it has — and then the left's own write did not land.
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            right_back: true,
            left_pending: true,
            ..Progress::default()
        });
        assert!(advice.contains("different protocol versions"), "{advice}");
        // And what happens to the right's image is told as the condition it
        // actually is: it keeps the image only once this run confirms it, which
        // does not happen while the left is still on the old build.
        assert!(advice.contains("keeps that image only once"), "{advice}");
        assert!(advice.contains("puts its old firmware back"), "{advice}");
        assert!(advice.contains("re-run"), "{advice}");
        assert!(advice.contains("power-cycle"), "{advice}");
    }

    #[test]
    fn a_right_that_never_came_back_from_its_restart_says_to_go_and_look_at_it() {
        // The failure mode the wait between the halves adds: the right was
        // written and restarted, the left was deliberately not touched, and
        // this end never saw the right again.
        let advice = advice_of(Progress {
            declared: true,
            written: true,
            right_restarted: true,
            right_back: false,
            left_pending: true,
            ..Progress::default()
        });
        assert!(advice.contains("did not come back"), "{advice}");
        assert!(advice.contains("left was not touched"), "{advice}");
        assert!(advice.contains("power-cycle"), "{advice}");
        // And it is not the message for a right that did come back, which is a
        // half-updated keyboard rather than a missing half.
        let back = advice_of(Progress {
            declared: true,
            written: true,
            right_restarted: true,
            right_back: true,
            left_pending: true,
            ..Progress::default()
        });
        assert!(!back.contains("did not come back"), "{back}");
        assert!(back.contains("different protocol versions"), "{back}");
    }

    #[test]
    fn a_right_that_restarted_onto_a_left_already_carrying_its_image_is_not_a_mismatch() {
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_pending: false,
            ..Progress::default()
        });
        assert!(!advice.contains("different protocol"), "{advice}");
        assert!(advice.contains("same build"), "{advice}");
        // Both halves are on the same build, and the right's image is still
        // going: nothing but this run confirms it, and this run did not get
        // there, so it says so flatly rather than leaving it open.
        assert!(advice.contains("only once this has confirmed it"), "{advice}");
        assert!(advice.contains("reverts on the next restart"), "{advice}");
    }

    #[test]
    fn a_pair_put_back_the_way_it_was_says_the_keyboard_is_fine() {
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_restarted: true,
            settled: true,
            reverted: true,
            ..Progress::default()
        });
        assert!(advice.contains("Nothing was kept"), "{advice}");
        assert!(advice.contains("works\nexactly as it did"), "{advice}");
        assert!(advice.contains("zapp flash fw.zsab"), "{advice}");
        // Not the alarming one, and not the confirm one either.
        assert!(!advice.contains("Power-cycle"), "{advice}");
        assert!(!advice.contains("half working"), "{advice}");
    }

    #[test]
    fn a_pair_that_could_not_be_put_back_asks_for_a_power_cycle_first() {
        // The worse ending: the revert was tried and at least one half was not
        // seen back on its old image, so the two may be on different builds.
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_restarted: true,
            settled: true,
            mismatched: true,
            revert_failed: true,
            ..Progress::default()
        });
        assert!(advice.contains("could not put the keyboard back"), "{advice}");
        assert!(advice.contains("different builds"), "{advice}");
        assert!(advice.contains("Power-cycle"), "{advice}");
        assert!(advice.contains("zapp flash fw.zsab"), "{advice}");
        // It outranks every other ending, including the mismatch it implies.
        assert!(!advice.contains("Nothing was kept"), "{advice}");
    }

    #[test]
    fn a_half_that_never_came_back_asks_for_a_power_cycle() {
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_restarted: true,
            left_pending: true,
            ..Progress::default()
        });
        assert!(advice.contains("did\nnot come back"), "{advice}");
        assert!(advice.contains("Power-cycle"), "{advice}");
    }

    #[test]
    fn a_failure_at_the_confirm_says_which_way_it_will_go_on_its_own() {
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_restarted: true,
            left_pending: true,
            settled: true,
            ..Progress::default()
        });
        assert!(advice.contains("not confirmed"), "{advice}");
        assert!(advice.contains("zapp flash fw.zsab"), "{advice}");
    }

    #[test]
    fn a_failure_after_the_declaration_but_before_any_write_says_nothing_is_pending() {
        let advice = advice_of(Progress {
            declared: true,
            session_id: Some(3),
            ..Progress::default()
        });
        assert!(advice.contains("nothing was written"), "{advice}");
        assert!(advice.contains("dropped on"), "{advice}");
        assert!(advice.contains("zapp flash fw.zsab"), "{advice}");
    }

    #[test]
    fn an_interrupted_write_says_an_expired_session_costs_nothing() {
        let advice = advice_of(Progress {
            declared: true,
            written: true,
            ..Progress::default()
        });
        assert!(advice.contains("session expired"), "{advice}");
        assert!(advice.contains("picks up from what is already installed"), "{advice}");
    }

    #[test]
    fn an_interrupted_write_with_the_right_still_pending_says_which_half_goes_first() {
        // The refusal a left-half write hits when the right is not finished,
        // told as a state of the keyboard rather than as an error code.
        let advice = advice_of(Progress {
            declared: true,
            written: true,
            right_pending: true,
            ..Progress::default()
        });
        assert!(advice.contains("right half has to be updated first"), "{advice}");
        // And a run whose right half was already done does not say it.
        let done = advice_of(Progress {
            declared: true,
            written: true,
            ..Progress::default()
        });
        assert!(!done.contains("has to be updated first"), "{done}");
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

    /// A pair back from its restarts, both on what was written.
    fn both_landed() -> Vec<(Half, Landed)> {
        vec![
            (
                Half::Left,
                Landed::Ok {
                    confirmed: false,
                    active: vec![0xa1; 64],
                },
            ),
            (
                Half::Right,
                Landed::Ok {
                    confirmed: false,
                    active: vec![0xb2; 64],
                },
            ),
        ]
    }

    #[test]
    fn a_pair_that_both_came_back_on_what_was_written_is_kept() {
        assert_eq!(decide(&both_landed(), None), Decision::Keep);
        // Including the resumed run, where nothing restarted at all.
        assert_eq!(decide(&[], None), Decision::Keep);
    }

    #[test]
    fn a_half_that_never_came_back_takes_the_other_half_down_with_it() {
        let mut landed = both_landed();
        landed[1].1 = Landed::Gone {
            why: "The right half did not come back within 180 s of its restart.".to_string(),
        };
        let Decision::Revert { reasons } = decide(&landed, None) else {
            panic!("a missing half is not something to keep an image over");
        };
        assert_eq!(reasons.len(), 1, "{reasons:?}");
        assert!(reasons[0].contains("right half did not come back"), "{reasons:?}");
    }

    #[test]
    fn a_half_that_came_back_on_the_wrong_image_reverts_the_pair() {
        let mut landed = both_landed();
        landed[0].1 = Landed::Wrong {
            active: vec![0x99; 64],
            version: "0.1.9".to_string(),
        };
        let Decision::Revert { reasons } = decide(&landed, None) else {
            panic!("a half on the old image is a failed update");
        };
        assert!(reasons[0].contains("The left half came back running"), "{reasons:?}");
        assert!(reasons[0].contains("0.1.9"), "{reasons:?}");
        assert!(reasons[0].contains("did not boot"), "{reasons:?}");

        // A half reporting nothing at all is named as that rather than given an
        // empty hash to print.
        landed[0].1 = Landed::Wrong {
            active: Vec::new(),
            version: String::new(),
        };
        let Decision::Revert { reasons } = decide(&landed, None) else {
            panic!("a half with nothing running is a failed update");
        };
        assert!(reasons[0].contains("reports nothing running"), "{reasons:?}");
    }

    #[test]
    fn two_halves_that_both_came_back_on_different_builds_still_revert() {
        // Each half was checked against the bundle on its own and both passed,
        // which is not the same as the pair matching.
        let Decision::Revert { reasons } =
            decide(&both_landed(), Some("they are on two builds".to_string()))
        else {
            panic!("two builds on one keyboard is not a keyboard");
        };
        assert_eq!(reasons, vec!["they are on two builds".to_string()]);
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
    fn a_board_that_disagrees_with_the_plan_says_so_and_defers_to_the_board() {
        // What a right half this end read as up to date but the board can see
        // has not confirmed looks like.
        let note = cross_check(&plan(Step::Installed, Step::Upload), Next::Right).unwrap();
        assert!(note.contains("the left half"), "{note}");
        assert!(flat(&note).contains("what it enforces"), "{note}");

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
        assert!(err.contains("did not come out of this on one build"), "{err}");
        assert!(err.contains("the right half is carrying"), "{err}");
        assert!(!err.contains("the left half"), "{err}");
        assert!(err.contains("cannot talk to each other"), "{err}");
        assert!(err.contains("Run the same command again"), "{err}");
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
    fn a_keyboard_on_two_builds_gets_advice_about_the_two_builds() {
        // Not the confirm advice: both halves may well be running the images
        // they were given, and the problem is that they are different ones.
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_restarted: true,
            settled: true,
            mismatched: true,
            ..Progress::default()
        });
        assert!(advice.contains("not on the same build"), "{advice}");
        assert!(advice.contains("cannot talk to"), "{advice}");
        assert!(advice.contains("zapp flash fw.zsab"), "{advice}");
        assert!(!advice.contains("has not confirmed"), "{advice}");
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
    fn a_long_hash_is_shortened_to_something_a_person_can_match() {
        let hash = vec![0xab; 64];
        assert_eq!(short(&hash), "abababab…abababab");
        // A short one is left whole rather than elided into nothing.
        assert_eq!(short(&[0x01, 0x02]), "0102");
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

    /// An error shaped the way one comes back from a board.
    fn refused(group: u16, rc: i64) -> anyhow::Error {
        anyhow::Error::from(SmpError::Remote {
            group,
            rc,
            meaning: "test",
        })
        .context("The left half's upload failed")
    }

    #[test]
    fn a_refusal_this_build_knows_gets_a_plain_paragraph() {
        let advice = refusal_note(&refused(zsa_err::GROUP, 1), Battery::default(), false).unwrap();
        assert!(flat(&advice).contains("battery is too low"), "{advice}");
        assert!(flat(&advice).contains("Charge the keyboard"), "{advice}");
        // Wrapped prose, not one long line, and no raw code in the text.
        // Counted in characters, which is what `wrap` folds on.
        assert!(advice.lines().count() > 1, "{advice}");
        assert!(
            advice.lines().all(|line| line.chars().count() <= 76),
            "{advice}"
        );
        assert!(!advice.contains("rc "), "{advice}");
    }

    #[test]
    fn the_three_refusals_a_user_acts_on_differently_read_differently() {
        let note = |code| refusal_note(&refused(zsa_err::GROUP, code), Battery::default(), false);
        let battery = note(2).unwrap();
        let busy = note(6).unwrap();
        let link = note(9).unwrap();
        assert!(flat(&battery).contains("Charge"), "{battery}");
        assert!(flat(&busy).contains("already talking"), "{busy}");
        assert!(flat(&link).contains("never got to the right half"), "{link}");
        assert_ne!(battery, busy);
        assert_ne!(busy, link);
    }

    #[test]
    fn a_code_from_a_newer_firmware_gets_no_paragraph_at_all() {
        // The error itself still prints "(group 66, rc 99)", which is honest.
        // Inventing a paragraph for it would not be.
        let note = |err: anyhow::Error| refusal_note(&err, Battery::default(), false);
        assert!(note(refused(zsa_err::GROUP, 99)).is_none());
        assert!(note(refused(1, 9)).is_none());
        assert!(note(anyhow::anyhow!("something else")).is_none());
    }

    #[test]
    fn a_battery_refusal_quotes_the_measured_levels_and_the_threshold() {
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        let note = refusal_note(&refused(zsa_err::GROUP, 1), battery, false).unwrap();
        assert!(flat(&note).contains("The left reads 0%"), "{note}");
        assert!(flat(&note).contains("the right 27%"), "{note}");
        assert!(flat(&note).contains("threshold is 30%"), "{note}");
        assert!(flat(&note).contains("Charge the keyboard"), "{note}");
    }

    #[test]
    fn a_battery_refusal_for_a_half_this_end_could_not_read_says_so() {
        // The board refused on its own measurement. Printing "the right is at
        // 0%" here would be a number nobody read.
        let battery = Battery {
            left: Level::Percent(64),
            right: Level::Unknown,
            keyboard: Level::Percent(64),
        };
        let note = refusal_note(&refused(zsa_err::GROUP, 2), battery, false).unwrap();
        assert!(flat(&note).contains("could not read a level for it"), "{note}");
        assert!(flat(&note).contains("the right would not say"), "{note}");
        // No invented number for the half nobody here could read.
        assert!(!flat(&note).contains("the right 0%"), "{note}");
        assert!(!flat(&note).contains("the right reads"), "{note}");
    }

    #[test]
    fn a_refusal_that_is_not_about_a_battery_carries_no_levels() {
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        let note = refusal_note(&refused(zsa_err::GROUP, 6), battery, false).unwrap();
        assert!(!flat(&note).contains("0%"), "{note}");
        assert!(flat(&note).contains("already talking"), "{note}");
    }

    #[test]
    fn the_battery_gate_names_the_halves_rather_than_the_pack_twice() {
        // left and keyboard both read 0, and they are the same cell.
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        let err = check_battery(battery, false).unwrap_err().to_string();
        assert!(flat(&err).contains("the left at 0%"), "{err}");
        assert!(flat(&err).contains("the right at 27%"), "{err}");
        assert!(!flat(&err).contains("keyboard at"), "{err}");
        assert!(flat(&err).contains("under the 30%"), "{err}");
        assert!(flat(&err).contains("Charge the keyboard"), "{err}");
    }

    #[test]
    fn a_board_with_one_battery_is_still_named() {
        let battery = Battery {
            left: Level::Absent,
            right: Level::Absent,
            keyboard: Level::Percent(4),
        };
        let err = check_battery(battery, false).unwrap_err().to_string();
        assert!(flat(&err).contains("the keyboard at 4%"), "{err}");
        // No half is named, because this board publishes none.
        assert!(!flat(&err).contains("the left"), "{err}");
        assert!(!flat(&err).contains("the right"), "{err}");
    }

    #[test]
    fn a_paragraph_is_folded_without_losing_or_breaking_a_word() {
        let text = "one two three four five six seven eight nine ten eleven twelve \
                    thirteen fourteen fifteen sixteen seventeen eighteen";
        let wrapped = wrap(text);
        assert!(wrapped.lines().count() > 1);
        assert!(wrapped.lines().all(|line| line.len() <= 76), "{wrapped}");
        assert_eq!(
            wrapped.split_whitespace().collect::<Vec<_>>(),
            text.split_whitespace().collect::<Vec<_>>()
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
}

