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
use indicatif::{ProgressBar, ProgressStyle};

use zapp_ble::smp::bundle::{self, Bundle, Step};
use zapp_ble::smp::session::{self, Declaration, Next};
use zapp_ble::smp::zsa_err::{self, Cause, Side};
use zapp_ble::smp::{self, Half, ImageSlot, SmpClient, SmpError};
use zapp_ble::{Battery, Level};
use zapp_core::device::{self, ids::Keyboard};
use zapp_core::flash::FlashProgress;

use crate::{smp_connect, LinkArgs};

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

/// How long to give a half to confirm the image it came back on.
///
/// Both halves confirm themselves once their radio has proved out — the left on
/// an encrypted BLE connection, the right once its link to the left is up — and
/// the gate in `shared/src/zsa_boot_confirm.c` waits 30 s of uptime before it
/// fires. This is that, plus room for a link that took its time.
const CONFIRM_TIMEOUT: Duration = Duration::from_secs(120);
const CONFIRM_POLL: Duration = Duration::from_secs(5);

/// How often a long wait says it is still a wait.
const NOTE_EVERY: Duration = Duration::from_secs(15);

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
            println!();
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
        let rerun = format!("zapp smp bundle {}", path.display());

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
                 image until the right is carrying what was declared and has confirmed it.\n\
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

        if self.settled {
            return format!(
                "Both halves are running the new firmware, but at least one has not confirmed\n\
                 it, so that half puts its old firmware back on the next restart.\n\n\
                 Leave the keyboard connected for a minute and re-run `{rerun}` — a half\n\
                 already running its image is confirmed rather than uploaded again.\n\
                 `zapp smp images` and `zapp smp images --half right` show which one is\n\
                 still pending."
            );
        }

        if self.right_restarted && !self.right_back && !self.left_restarted && self.left_pending {
            return format!(
                "The right half was written and restarted, and it did not come back where\n\
                 this could see it. The left was not touched, so it is still running the\n\
                 firmware it was — and the keyboard is half working until the right is\n\
                 back, because the two are on different builds.\n\n\
                 Nothing was confirmed. The right was only marked for test, so it puts its\n\
                 old firmware back by itself the next time it restarts.\n\n\
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
                     Nothing was confirmed. The right was only marked for test, so it puts its\n\
                     old firmware back by itself the next time it restarts.\n\n\
                     Either:\n  \
                     · re-run `{rerun}` to finish the update, or\n  \
                     · power-cycle the keyboard to put the right half back as it was."
                );
            }
            return format!(
                "The right half was restarted onto the new firmware and the left was already\n\
                 carrying its image, so the two are on the same build. The right has not\n\
                 confirmed, so it reverts on the next restart.\n\n\
                 Re-run `{rerun}` to finish, or power-cycle the keyboard to put the right\n\
                 half back as it was."
            );
        }

        format!(
            "The halves were written, marked for test and restarted, and at least one did\n\
             not come back where this could see it. Nothing was confirmed, so anything\n\
             that was written puts its old firmware back on the next restart.\n\n\
             Power-cycle the keyboard to get a working pair back, then re-run `{rerun}`."
        )
    }
}

fn run(bundle: &Bundle, force: bool, link: &LinkArgs, progress: &mut Progress) -> Result<()> {
    // ---- What is there now ----------------------------------------------
    println!();
    println!("Reading what is installed");

    let left_slots = read_state(link, Half::Left)?;
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
    println!("  charge {}", charge(battery, charging));
    // Before the right half is touched, not after: reading the right goes
    // through the left's relay, and the relay is battery-gated too. On a flat
    // pack that read is the thing that fails, and it fails looking like a
    // radio-link fault — so the check that can actually name the battery has
    // to run first.
    check_battery(battery, charging)?;
    let right_slots = read_state(link, Half::Right).context(
        "The right half is reached through the left, over the halves' own radio link, and \
         that link has to be up before anything is written — updating the left first would \
         cut off the only route to the right. Check both halves are powered and linked",
    )?;

    let mut plan = [
        (Half::Right, bundle::step_for(bundle.image(Half::Right), &right_slots)),
        (Half::Left, bundle::step_for(bundle.image(Half::Left), &left_slots)),
    ];
    if force {
        plan = [(Half::Right, Step::Upload), (Half::Left, Step::Upload)];
    }

    for (half, step) in plan {
        let slots = match half {
            Half::Left => &left_slots,
            Half::Right => &right_slots,
        };
        println!("  {:<5}  {}", half.as_str(), describe(step, slots));
    }

    if plan.iter().all(|(_, step)| *step == Step::Installed) {
        println!();
        println!(
            "Both halves already carry this bundle. Nothing to do — pass --force to \
             write them anyway."
        );
        return Ok(());
    }
    let pending = |wanted: Half| {
        plan.iter()
            .any(|(half, step)| *half == wanted && *step != Step::Installed)
    };
    progress.left_pending = pending(Half::Left);
    progress.right_pending = pending(Half::Right);

    let uploads = plan.iter().filter(|(_, step)| *step == Step::Upload).count();
    let restarting = uploads > 0;
    let confirming = plan.iter().any(|(_, step)| *step != Step::Installed);
    // Only when both halves are being written: with one upload there is no
    // right-half restart standing between this run and the left.
    let waiting = uploads == 2;
    let mut steps = Steps {
        total: uploads * 2
            + usize::from(waiting)
            + usize::from(restarting)
            + usize::from(confirming),
        done: 0,
    };

    if restarting {
        println!();
        println!("The keyboard cannot be typed on while this runs, and both halves go dark or");
        println!("blue — that is by design, not a fault. It takes a couple of minutes. Nothing");
        println!("is kept until it has proved itself, so a keyboard interrupted half-way gets");
        println!("its old firmware back on the next power cycle.");
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
        println!("{note}");
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
            steps.announce("Waiting for the right half to come back");
            wait_for_right_ready(link, &declaration)?;
            progress.right_back = true;
        }

        steps.announce(&format!(
            "{} half — upload ({} KiB)",
            titled(half),
            image.len() / 1024
        ));
        let (mut client, renewed) = connect_declared(link, half, &declaration)?;
        progress.session_id = Some(renewed.id);

        // Set before the upload, not after: an upload that fails part-way has
        // still put bytes in the spare slot, and the message for that is the
        // one about a slot written and nothing marked.
        progress.written = true;
        upload(&mut client, half, image)?;
        restart(&mut steps, &mut client, half, &hash)?;
        match half {
            Half::Right => progress.right_restarted = true,
            Half::Left => progress.left_restarted = true,
        }
    }

    // ---- Wait for them to come back --------------------------------------
    //
    // The left first, and only now: between the right's restart and the left's
    // the two halves are on different firmware and cannot talk, so the right is
    // unreachable and its own confirm gate — which waits on that link — cannot
    // fire. That gap is expected. Once the left is back on the same build the
    // link comes up, the right becomes reachable again, and both can confirm.
    //
    // Which is why `wait_for_right_ready` above waits on the board's `next` and
    // not on the right having confirmed: the board counts a right that carries
    // its declared image and reports a version mismatch as ready, precisely so
    // this gap does not deadlock the pair. The right's confirm lands here
    // instead, once the left is back on the matching build and the link is up.
    let mut confirmed = [(Half::Right, false), (Half::Left, false)];
    if restarting {
        steps.announce("Waiting for the halves to come back");
        for half in [Half::Left, Half::Right] {
            if !restarted(*progress, half) {
                continue;
            }
            let hash = bundle.hash(half).unwrap_or_default();
            let back = wait_for_boot(link, half, &hash)?;
            if let Some(slot) = confirmed.iter_mut().find(|(which, _)| *which == half) {
                slot.1 = back;
            }
        }
    }

    // ---- Keep it ---------------------------------------------------------
    //
    // Everything is in place from here: whatever fails now leaves both halves
    // running their new images with the confirm outstanding, which reverts on
    // the next restart rather than sticking.
    progress.settled = true;
    if confirming {
        steps.announce("Waiting for the halves to confirm");
        for (half, step) in plan {
            if step == Step::Installed {
                continue;
            }
            let hash = bundle
                .hash(half)
                .with_context(|| format!("The bundle's {half} image carries no hash"))?;
            let already = confirmed
                .iter()
                .any(|(which, done)| *which == half && *done);
            await_confirm(link, half, &hash, already)?;
        }
    }

    // ---- Let the session go ---------------------------------------------
    //
    // Best effort, and quiet about it. If the left restarted, its own reboot
    // already dropped the session: it lives in the left's memory and the
    // firmware writes none of it to flash, which is the design rather than
    // something that went wrong.
    close_session(link, progress);

    println!();
    println!(
        "Done. Both halves are running {}.",
        named(&bundle.header.build_version)
    );
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
/// once the right has done everything it can, and that is a wider condition than
/// having confirmed: a right carrying its declared image but reporting a
/// protocol mismatch counts too, because on a release that moves the halves'
/// protocol it *cannot* confirm until the left is on the matching build, and
/// insisting would deadlock the pair onto a cable. An unconfirmed right here is
/// the ordinary middle of a sequence and not a warning; the confirm pass at the
/// end is what closes it.
///
/// Asked of the left, over one connection held for the whole wait. The left is
/// up throughout, it is the half that holds the session, and STATUS is never
/// refused. Reading the right through the relay would work too and would say
/// less for more.
///
/// Re-declaring on each poll is what keeps the session's idle timer alive
/// through the wait; an identical declaration renews rather than collides.
fn wait_for_right_ready(link: &LinkArgs, declaration: &Declaration) -> Result<()> {
    println!("  right: giving it a moment to restart");
    thread::sleep(REBOOT_SETTLE);

    let mut client = smp_connect(link, Half::Left)?;
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
                    println!(
                        "  right: back after {} s{}",
                        started.elapsed().as_secs(),
                        match status.right_done {
                            true => ", confirmed",
                            false => ", not confirmed yet, which is normal here",
                        }
                    );
                    return Ok(());
                }
            }
            Err(err) => {
                log::debug!("the left stopped answering while waiting on the right: {err:#}");
                client = smp_connect(link, Half::Left)?;
            }
        }

        if started.elapsed() >= REBOOT_TIMEOUT {
            break;
        }
        if noted.elapsed() >= NOTE_EVERY {
            noted = Instant::now();
            println!(
                "  right: not back yet after {} s of {}",
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
    println!();
    println!(
        "Declared {} to the keyboard (session {})",
        named(&declaration.version),
        status.id
    );
    println!("  the board says {} next", next_as(status.next));
    if declaration.redo {
        println!("  asking for both halves to be written again, in the same order");
    }
    if !status.right_seen {
        println!("  it cannot read the right half's image state at all");
    }
    if !status.guard {
        println!("  this board is not enforcing declared updates: a development build");
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
/// A half counts as finished only when it is carrying its declared image *and*
/// has confirmed it, which is the same rule the board applies: an unconfirmed
/// image reverts on the next restart, so counting it would let the sequence
/// finish on top of firmware that is about to disappear.
fn expected_next(plan: &[(Half, Step)]) -> Next {
    let step = |wanted: Half| {
        plan.iter()
            .find(|(half, _)| *half == wanted)
            .map(|(_, step)| *step)
    };
    let done = |half: Half| step(half).is_none_or(|step| step == Step::Installed);

    match (done(Half::Right), done(Half::Left)) {
        (false, _) => Next::Right,
        (true, false) => Next::Left,
        (true, true) => Next::Complete,
    }
}

/// A note for a board whose own sequencing disagrees with this run's plan.
///
/// The board's view is the one that decides: it can see what each half is
/// carrying and whether the right has confirmed, and it refuses on that. Saying
/// so is worth a line, because a disagreement here is the first sign of a half
/// this end has read wrongly.
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
    result.with_context(|| format!("The {half} half's upload failed"))?;
    println!("  {} bytes written into the spare slot", image.len());
    Ok(())
}

/// Marks a half's new image for test and restarts it.
///
/// For test, never confirmed: an image that is confirmed before it has run is
/// an image with no way back. Both halves confirm themselves once their radio
/// has proved out, and a half that never gets that far reverts on its own.
fn restart(steps: &mut Steps, client: &mut SmpClient, half: Half, hash: &[u8]) -> Result<()> {
    steps.announce(&format!("{} half — mark for test and restart", titled(half)));
    client
        .mark(hash, false)
        .with_context(|| format!("Could not mark the {half} half's new image for test"))?;
    println!("  marked {} for test", short(hash));
    client
        .reset()
        .with_context(|| format!("Could not restart the {half} half"))?;
    println!("  restarting");
    Ok(())
}

/// Waits for a half to come back, and checks it came back on the right image.
///
/// Answers whether it had already confirmed itself by the time it was reached,
/// which a fast link sometimes manages.
fn wait_for_boot(link: &LinkArgs, half: Half, hash: &[u8]) -> Result<bool> {
    println!("  {}: giving it a moment to restart", half);
    thread::sleep(REBOOT_SETTLE);

    let started = Instant::now();
    loop {
        let slots = match read_state(link, half) {
            Ok(slots) => slots,
            Err(err) if started.elapsed() < REBOOT_TIMEOUT => {
                log::debug!("{half} half not back yet: {err:#}");
                println!(
                    "  {half}: not back yet after {} s; trying again",
                    started.elapsed().as_secs()
                );
                thread::sleep(RETRY_EVERY);
                continue;
            }
            Err(err) => {
                return Err(err).with_context(|| {
                    format!(
                        "The {half} half did not come back within {} s of its restart",
                        REBOOT_TIMEOUT.as_secs()
                    )
                })
            }
        };

        let Some(active) = slots.iter().find(|slot| slot.active) else {
            bail!("The {half} half came back but reports nothing running");
        };
        if active.hash != hash {
            bail!(
                "The {half} half came back running {} ({}), not the image that was written. \
                 MCUboot put the old image back, which means the new one did not boot.",
                short(&active.hash),
                named(&active.version)
            );
        }
        println!(
            "  {half}: back after {} s, running {} ({})",
            started.elapsed().as_secs(),
            short(&active.hash),
            named(&active.version)
        );
        return Ok(active.confirmed);
    }
}

/// Waits for a half to confirm the image it is running, and keeps it by hand if
/// it will not.
///
/// Waiting rather than confirming is the point: the firmware confirms itself
/// once its radio has proved healthy, and that gate is the only thing standing
/// between a bad image and a keyboard that stays on it. Confirming from here
/// bypasses it, so it is the fallback and it says so out loud.
fn await_confirm(link: &LinkArgs, half: Half, hash: &[u8], already: bool) -> Result<()> {
    if already {
        println!("  {half}: confirmed");
        return Ok(());
    }

    // One session held open for the whole wait: the left's own gate wants an
    // encrypted connection to fire on, and reconnecting per poll would cost
    // more than the poll.
    let mut client = smp_connect(link, half).ok();
    let started = Instant::now();
    let mut noted = Instant::now();

    while started.elapsed() < CONFIRM_TIMEOUT {
        if let Some(open) = client.as_mut() {
            match open.image_state() {
                Ok(slots) => {
                    if slots
                        .iter()
                        .any(|slot| slot.active && slot.hash == hash && slot.confirmed)
                    {
                        println!(
                            "  {half}: confirmed itself after {} s",
                            started.elapsed().as_secs()
                        );
                        return Ok(());
                    }
                }
                Err(err) => {
                    log::debug!("{half} half stopped answering while confirming: {err:#}");
                    client = None;
                }
            }
        }
        // A line now and then rather than one per poll: enough to show it is
        // still going without burying the phases either side of it.
        if noted.elapsed() >= NOTE_EVERY {
            noted = Instant::now();
            println!(
                "  {half}: still waiting for it to confirm itself ({} s of {})",
                started.elapsed().as_secs(),
                CONFIRM_TIMEOUT.as_secs()
            );
        }
        thread::sleep(CONFIRM_POLL);
        if client.is_none() {
            client = smp_connect(link, half).ok();
        }
    }

    println!(
        "  {half}: it has not confirmed itself in {} s, so confirming it from here.",
        CONFIRM_TIMEOUT.as_secs()
    );
    println!(
        "  {half}: that means the firmware's own health gate never fired. The image is"
    );
    println!("  {half}: kept, but something on that half is not happy — worth a look.");

    let mut client = match client {
        Some(client) => client,
        None => smp_connect(link, half)?,
    };
    client
        .mark(hash, true)
        .with_context(|| format!("Could not confirm the {half} half's image"))?;
    println!("  {half}: confirmed");
    Ok(())
}

/// One image-state read, over a session opened for it.
fn read_state(link: &LinkArgs, half: Half) -> Result<Vec<ImageSlot>> {
    let mut client = smp_connect(link, half)?;
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

/// The firmware's own reason for saying no, in plain sentences.
///
/// The board answers a refusal with a ZSA code (see
/// `zapp_ble::smp::zsa_err`), and the one line the error chain gives it is a
/// summary rather than an answer. `None` for anything that is not a refusal
/// this build knows — including a code from a firmware newer than this zapp,
/// which is left to the raw group and number in the error itself rather than
/// dressed up as something it might not be.
pub fn refusal_advice(err: &anyhow::Error) -> Option<String> {
    Some(wrap(refusal(err)?.advice))
}

/// The paragraph a single subcommand needs when the declared-update guard is
/// what turned it down.
///
/// `zapp smp upload`, `test`, `confirm` and `reset` write firmware on nobody's
/// authority but their own, which is exactly what shipping firmware refuses over
/// Bluetooth. They are kept because they are the debugging surface and they work
/// against a build with the guard compiled out, so the answer is to say which
/// rule was hit and where the supported path is — never to retry, since the
/// guard refuses the same request every time by design.
///
/// `None` for every other refusal, including a code newer than this build.
pub fn guard_note(err: &anyhow::Error) -> Option<String> {
    if !refusal(err)?.cause.is_guard() {
        return None;
    }
    Some(wrap(
        "Over Bluetooth this keyboard only accepts a declared update. `zapp smp bundle \
         <file.zsab>` is the supported path: it states both halves' images up front, \
         writes them in the order the keyboard insists on, and waits for each half to \
         come back. The bare `upload`, `test`, `confirm` and `reset` subcommands declare \
         nothing, so shipping firmware refuses them; they are the debugging surface and \
         they only work against a development build with the guard compiled out \
         (CONFIG_ZSA_UPDATE_GUARD=n). `zapp smp session` shows what the board currently \
         has open. Nothing was written and nothing was retried.",
    ))
}

/// The same, with the measured levels spliced in when the reason is a battery.
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
fn wrap(text: &str) -> String {
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

/// Numbered phases, so a wait of minutes reads as progress rather than a hang.
struct Steps {
    total: usize,
    done: usize,
}

impl Steps {
    fn announce(&mut self, label: &str) {
        self.done += 1;
        println!();
        println!("Step {}/{} · {label}", self.done, self.total);
    }
}

fn announce(bundle: &Bundle, path: &Path, len: usize) {
    println!(
        "Bundle {} · {} · {len} bytes",
        path.display(),
        named(&bundle.header.build_version)
    );
    for half in [Half::Left, Half::Right] {
        let image = bundle.image(half);
        let version = smp::image::inspect(image)
            .map(|info| info.version)
            .unwrap_or_else(|| "no MCUboot header".to_string());
        println!(
            "  {:<5}  v{version} · {} bytes · {}",
            half.as_str(),
            image.len(),
            bundle
                .hash(half)
                .map(|hash| short(&hash))
                .unwrap_or_else(|| "no hash".to_string())
        );
    }
    match bundle.header.key_fingerprint {
        // Nothing stamps this yet: shared/cmake/zsa_esb_key.cmake is not wired
        // into either app, so every bundle built today carries zero and it is
        // read as "unknown" rather than as a mismatch.
        0 => println!("  radio key: not stamped, so not checked"),
        fingerprint => println!("  radio key: {fingerprint:08x}"),
    }
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
fn short(hash: &[u8]) -> String {
    let hex = smp::image::hex(hash);
    match hex.len() > 20 {
        true => format!("{}…{}", &hex[..8], &hex[hex.len() - 8..]),
        false => hex,
    }
}

fn named(version: &str) -> String {
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
        assert!(advice.contains("zapp smp bundle fw.zsab"), "{advice}");
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
        assert!(advice.contains("puts its\nold firmware back"), "{advice}");
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
        assert!(advice.contains("zapp smp images"), "{advice}");
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
        assert!(advice.contains("zapp smp bundle fw.zsab"), "{advice}");
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
    fn a_half_running_its_image_unconfirmed_is_not_finished() {
        // The board counts a half done only once it has confirmed, because an
        // unconfirmed image reverts on the next restart. This end has to agree
        // or the cross-check fires on every resumed update.
        assert_eq!(
            expected_next(&plan(Step::Confirm, Step::Installed)),
            Next::Right
        );
        assert_eq!(
            expected_next(&plan(Step::Installed, Step::Confirm)),
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
    fn the_guard_sends_a_single_subcommand_to_the_bundle_and_never_to_a_retry() {
        // 11, 12 and 13 are what a bare `upload`, `test` or `confirm` gets from
        // shipping firmware; each has to name the supported path and say the
        // single-step commands need a build with the guard out.
        for code in 11..=17 {
            let note = guard_note(&refused(zsa_err::GROUP, code))
                .unwrap_or_else(|| panic!("no note for code {code}"));
            let note = flat(&note);
            assert!(note.contains("zapp smp bundle <file.zsab>"), "code {code}");
            assert!(note.contains("CONFIG_ZSA_UPDATE_GUARD=n"), "code {code}");
            assert!(note.contains("nothing was retried"), "code {code}");
        }
    }

    #[test]
    fn a_refusal_that_is_not_the_guard_gets_no_note_about_it() {
        // A flat battery or a stalled radio link is not something a bundle run
        // fixes, and saying so would send the user the wrong way.
        for code in 1..=10 {
            assert!(
                guard_note(&refused(zsa_err::GROUP, code)).is_none(),
                "code {code}"
            );
        }
        assert!(guard_note(&refused(zsa_err::GROUP, 99)).is_none());
        assert!(guard_note(&anyhow::anyhow!("something else")).is_none());
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
        let advice = refusal_advice(&refused(zsa_err::GROUP, 1)).unwrap();
        assert!(flat(&advice).contains("battery is too low"), "{advice}");
        assert!(flat(&advice).contains("Charge the keyboard"), "{advice}");
        // Wrapped prose, not one long line, and no raw code in the text.
        assert!(advice.lines().count() > 1, "{advice}");
        assert!(advice.lines().all(|line| line.len() <= 76), "{advice}");
        assert!(!advice.contains("rc "), "{advice}");
    }

    #[test]
    fn the_three_refusals_a_user_acts_on_differently_read_differently() {
        let battery = refusal_advice(&refused(zsa_err::GROUP, 2)).unwrap();
        let busy = refusal_advice(&refused(zsa_err::GROUP, 6)).unwrap();
        let link = refusal_advice(&refused(zsa_err::GROUP, 9)).unwrap();
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
        assert!(refusal_advice(&refused(zsa_err::GROUP, 99)).is_none());
        assert!(refusal_advice(&refused(1, 9)).is_none());
        assert!(refusal_advice(&anyhow::anyhow!("something else")).is_none());
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

