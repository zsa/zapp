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
use zapp_ble::smp::{self, Half, ImageSlot, SmpClient};
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
            println!();
            println!("Update failed: {err:#}");
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
    /// An image has been put into a spare slot, marked or not.
    written: bool,
    right_restarted: bool,
    left_restarted: bool,
    /// The left still needs its image — so a restarted right is a right that
    /// is talking a protocol the left does not know.
    left_pending: bool,
    /// Both halves are back on their new images and only the confirm is left.
    settled: bool,
}

impl Progress {
    /// What the keyboard is, and what to do about it.
    fn advice(&self, path: &Path) -> String {
        let rerun = format!("zapp smp bundle {}", path.display());

        if !self.written {
            return "Nothing was written. The keyboard is untouched and still working."
                .to_string();
        }

        if !self.right_restarted && !self.left_restarted {
            return format!(
                "An image was written into a spare slot, but nothing was marked and nothing\n\
                 was restarted. Both halves are still running the firmware they were, and\n\
                 the keyboard works exactly as it did.\n\n\
                 Re-run `{rerun}` to try again."
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
    let charging = charging();
    println!("  charge {}", charge(battery, charging));
    check_battery(battery, charging)?;

    if plan.iter().all(|(_, step)| *step == Step::Installed) {
        println!();
        println!(
            "Both halves already carry this bundle. Nothing to do — pass --force to \
             write them anyway."
        );
        return Ok(());
    }
    progress.left_pending = plan
        .iter()
        .any(|(half, step)| *half == Half::Left && *step != Step::Installed);

    let uploads = plan.iter().filter(|(_, step)| *step == Step::Upload).count();
    let restarting = uploads > 0;
    let confirming = plan.iter().any(|(_, step)| *step != Step::Installed);
    let mut steps = Steps {
        total: uploads * 2 + usize::from(restarting) + usize::from(confirming),
        done: 0,
    };

    if restarting {
        println!();
        println!("The keyboard cannot be typed on while this runs, and both halves go dark or");
        println!("blue — that is by design, not a fault. It takes a couple of minutes. Nothing");
        println!("is kept until it has proved itself, so a keyboard interrupted half-way gets");
        println!("its old firmware back on the next power cycle.");
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

        // Set before the upload, not after: an upload that fails part-way has
        // still put bytes in the spare slot, and the message for that is the
        // one about a slot written and nothing marked.
        progress.written = true;
        let mut client = upload(&mut steps, link, half, image)?;
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

    println!();
    println!(
        "Done. Both halves are running {}.",
        named(&bundle.header.build_version)
    );
    Ok(())
}

fn restarted(progress: Progress, half: Half) -> bool {
    match half {
        Half::Left => progress.left_restarted,
        Half::Right => progress.right_restarted,
    }
}

/// Sends one half's image into its spare slot, and leaves the session open for
/// the marking that follows.
/// The left holds one DFU lock for both halves, and relaying to the right takes
/// it for the whole session — which lingers a few seconds past the last frame so
/// a multi-frame exchange is not cut in half. Anything that touched the right,
/// including merely reading its image state, therefore blocks the left's own
/// upload with "busy" until that lapses. Wait it out rather than racing it.
const RELAY_SETTLE: Duration = Duration::from_secs(5);

fn upload(steps: &mut Steps, link: &LinkArgs, half: Half, image: &[u8]) -> Result<SmpClient> {
    steps.announce(&format!(
        "{} half — upload ({} KiB)",
        titled(half),
        image.len() / 1024
    ));
    if half == Half::Left {
        thread::sleep(RELAY_SETTLE);
    }
    let mut client = smp_connect(link, half)?;

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
    Ok(client)
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
        ("keyboard", battery.keyboard.percent()),
    ];
    for (half, level) in halves {
        if bundle::battery_allows(level, charging) {
            continue;
        }
        bail!(
            "The {half} battery is at {}%, under the {}% an update needs. An update takes \
             minutes of radio and a reboot, and a half that runs out part-way through is a \
             half that has to be recovered over the wire. Charge it, or plug the keyboard \
             in and try again — on the charger this check does not apply.",
            level.unwrap_or(0),
            bundle::MIN_SOC_PERCENT
        );
    }
    Ok(())
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
        let advice = advice_of(Progress {
            written: true,
            right_restarted: true,
            left_pending: true,
            ..Progress::default()
        });
        assert!(advice.contains("different protocol versions"), "{advice}");
        assert!(advice.contains("puts its\nold firmware back"), "{advice}");
        assert!(advice.contains("re-run"), "{advice}");
        assert!(advice.contains("power-cycle"), "{advice}");
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
        });
        assert!(advice.contains("not confirmed"), "{advice}");
        assert!(advice.contains("zapp smp images"), "{advice}");
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

