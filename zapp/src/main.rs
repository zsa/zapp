mod confirm;
mod nrf_wired;
mod zsab;

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};

use zapp_ble::dfu::{self, DfuClient, DfuError, Wait};
use zapp_ble::smp::{self, Half, SmpClient};
use zapp_ble::{Battery, BleMonitor, Level, Presence};
use zapp_core::device::ids::{self as ids, is_moonlander_revb, target_name_for_pid};
use zapp_core::device::{self, UsbDevice, WatchStatus};
use zapp_core::firmware::{self, Firmware};
use zapp_core::flash::{self, FlashProgress};
use zapp_core::hid::{
    BatteryReading, CMD_GET_TELEMETRY, ChargeState, Charging, HidConnection, HidEvent, Nav,
    Source, Telemetry,
};
use zapp_oryx::FirmwareVariant;

#[derive(Parser)]
#[command(name = "zapp", version, about = format!("⚡ Flash ZSA keyboards — v{}", env!("CARGO_PKG_VERSION")))]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Flash firmware from a local file or Oryx URL
    #[command(
        long_about = "Flash firmware from a local file or Oryx URL.\n\n\
            Supports .bin (DFU) and .hex (Intel HEX) keyboard firmware, .zsab \
            bundles carrying both wireless halves, and Oryx layout URLs. When \
            given a URL, the firmware is downloaded and flashed directly.\n\n\
            A .zsab bundle and a Voyager 2 keyboard image both go over \
            whichever way the keyboard is reachable: the cable if it is on \
            USB, Bluetooth otherwise. Over Bluetooth the STM32 is rebooted \
            into its bootloader on the way, and one already sitting there is \
            written straight away. Any other keyboard's firmware goes over the \
            cable, and this waits for the bootloader to appear rather than \
            expecting it to be there already — press the reset button on the \
            left half when asked.\n\n\
            A bundle carries both halves and they are always written together, \
            the right one first. Over the cable, a right half that does not \
            answer is waited for: make sure the TRRS cable between the halves \
            is properly seated while it asks.",
        after_long_help = "Examples:\n  \
            zapp flash firmware.bin\n  \
            zapp flash .build/zsa_voyager_tng_default.bin\n  \
            zapp flash .build/zsa_voyager_tng_default.bin --device \"Voyager\"\n  \
            zapp flash voyager2_0.1.9.zsab\n  \
            zapp flash voyager2_0.1.9.zsab --force\n  \
            zapp flash voyager2_0.1.9.zsab --device \"Voyager\"\n  \
            zapp flash https://configure.zsa.io/voyager/layouts/default/latest\n  \
            zapp flash https://configure.zsa.io/moonlander/layouts/default/abc123"
    )]
    Flash {
        /// Path to a firmware file (.bin, .hex or .zsab), or an Oryx URL.
        firmware: String,
        /// Write a half even if it already carries the image, or the bundle is
        /// older than what is installed. Only means anything for .zsab bundles:
        /// a keyboard image is always written.
        #[arg(long)]
        force: bool,
        /// Which board, by advertised name or by the handle `zapp devices`
        /// shows. Only means anything over Bluetooth, where more than one
        /// bonded board can answer; over the cable there is only ever one.
        #[arg(long, short = 'd')]
        device: Option<String>,
    },
    /// Check Oryx for updates on your layout and flash if available
    #[command(
        long_about = "Check Oryx for updates on your layout and flash if available.\n\n\
            Detects a connected ZSA keyboard, reads its layout and revision from the \
            USB serial number, and checks Oryx for a newer firmware revision. If an \
            update is available, it is downloaded and flashed automatically. \n\
            Note: This only works if your keyboard is currently flashed with a firmware \
            coming from Oryx and is not currently in bootloader mode."
    )]
    Update,
    /// List the ZSA keyboards this machine can see, wired and wireless
    #[command(long_about = "List the ZSA keyboards this machine can see.\n\n\
            Wired boards are read straight from USB. Bluetooth boards are listed \
            when they are connected to this machine, bonded to it, or advertising \
            in range. A board that has never been paired only advertises, so pass \
            --scan to hold a discovery window open long enough to catch one.")]
    Devices {
        /// Scan for boards that have never been paired to this machine
        #[arg(long)]
        scan: bool,
    },
    /// Pair the two halves by provisioning their shared ESB key over the cable
    #[command(
        long_about = "Provision the ESB pairing key onto both halves over the cable.\n\n\
            A Voyager 2's two halves talk to each other over a private radio link, \
            and they link only when both carry the same 30-byte key. `zapp bond` \
            writes one fresh key to both halves through the bootloader, reads it \
            back off each, then reboots the keyboard and waits for both halves to \
            report over USB — which is the pair having re-linked on the new key. A \
            key mismatch is silent: the link just never forms, so that telemetry is \
            the only real proof.\n\n\
            Put the keyboard in its bootloader first (reset button on the left \
            half); bond waits for it and asks. Pass --key to write a specific key \
            instead of generating one, or --show to read what the two halves carry \
            without writing anything.",
        after_long_help = "Examples:\n  \
            zapp bond\n  \
            zapp bond --show\n  \
            zapp bond --key 4a1b2c...  (60 hex characters)"
    )]
    Bond {
        /// Write this exact 30-byte key (60 hex characters) instead of generating one
        #[arg(long, value_name = "HEX")]
        key: Option<String>,
        /// Read both halves' current key and report whether they match, writing nothing
        #[arg(long)]
        show: bool,
    },
}

/// Everything about reaching a board that does not say which half.
///
/// The bundle flow talks to both halves in turn and picks the order itself, so
/// the half is never an argument. Only `device` comes off the command line;
/// the rest are the tuning the Bluetooth client needs and are left at their
/// defaults.
pub struct LinkArgs {
    /// Which board, by advertised name or by the handle `zapp devices` shows
    device: Option<String>,
    /// Bytes of payload per Bluetooth write, the ATT MTU less its 3-byte
    /// header. `None` takes what the platform reports.
    mtu: Option<usize>,
    /// Bluetooth writes to spend on one request. `None` takes 5.
    writes: Option<usize>,
    /// Largest request to send, fragmented across writes. Overrides `writes`.
    /// `None` takes the buffer the board reports, capped by `writes`.
    chunk: Option<usize>,
    /// Seconds to wait for each reply
    timeout: u64,
}

/// Seconds to wait for a reply, when nobody said otherwise.
const DEFAULT_LINK_TIMEOUT: u64 = 15;

impl Default for LinkArgs {
    /// Written out rather than derived: a derived `Default` gives `timeout`
    /// zero, and every request then times out before it has been sent.
    fn default() -> Self {
        Self {
            device: None,
            mtu: None,
            writes: None,
            chunk: None,
            timeout: DEFAULT_LINK_TIMEOUT,
        }
    }
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Flash {
            firmware,
            force,
            device,
        } => {
            let result = if zapp_oryx::is_oryx_url(&firmware) {
                cmd_flash_url(&firmware)
            } else {
                cmd_flash_file(&PathBuf::from(firmware), force, device.as_deref())
            };
            with_dfu_advice(result)
        }
        Commands::Update => cmd_update(),
        Commands::Devices { scan } => cmd_devices(scan),
        Commands::Bond { key, show } => {
            if show {
                if key.is_some() {
                    anyhow::bail!("--show reads what the halves carry; it takes no --key");
                }
                nrf_wired::bond_show()
            } else {
                nrf_wired::bond(key.as_deref())
            }
        }
    }
}

/// How long to give the radio. Without `--scan`, just long enough for the first
/// answer, which already carries every bonded board; with it, long enough for a
/// discovery window to open and close.
const BLE_SETTLE: Duration = Duration::from_millis(750);
const BLE_SCAN: Duration = Duration::from_secs(7);

/// How long to wait for a wired board to answer the telemetry query. Firmware
/// that predates the command never answers at all, so this elapses in the
/// ordinary case and must stay short enough not to be felt.
const TELEMETRY_WAIT: Duration = Duration::from_millis(500);

fn cmd_devices(scan: bool) -> Result<()> {
    let wired = device::list_devices();

    println!("USB");
    match &wired {
        Ok(devices) if devices.is_empty() => println!("  none"),
        Ok(devices) => {
            for device in devices {
                println!("  {}", describe_usb(device));
                for line in wired_report(device) {
                    println!("    {line}");
                }
            }
        }
        Err(err) => println!("  cannot scan: {err}"),
    }

    // Which models are plugged in, so a board listed twice can say which of
    // its two entries is the live one.
    let wired_models: Vec<_> = wired
        .iter()
        .flatten()
        .filter(|device| device.bootloader.is_none())
        .filter_map(|device| device.keyboard)
        .collect();

    let monitor = BleMonitor::start(&[]);
    let wait = if scan { BLE_SCAN } else { BLE_SETTLE };
    let spinner = new_spinner("Looking for Bluetooth boards...");
    thread::sleep(wait);
    spinner.finish_and_clear();

    println!();
    println!("Bluetooth");
    let boards = monitor.snapshot();
    if boards.is_empty() {
        println!("  none");
        if !scan {
            println!("  (pass --scan to look for boards that have never been paired)");
        }
    }
    for board in boards {
        let name = board
            .keyboard
            .map(|keyboard| keyboard.to_string())
            .unwrap_or_else(|| "ZSA keyboard".to_string());
        let known_as = board.name.unwrap_or_else(|| "unnamed".to_string());
        println!(
            "  {name} · {} · {known_as:?}{}{}",
            state(board.presence),
            charge(board.battery),
            also_wired(board.keyboard, &wired_models)
        );
    }

    Ok(())
}

/// Ask a wired board about itself, and say what it answered.
///
/// A board is preferred over the radio for its own state: plugged in, it can
/// read both halves directly, and the Bluetooth view of a wired board is
/// whatever the radio last carried. Firmware without the command simply never
/// answers, so an empty result here is the normal old-firmware case and says
/// nothing.
fn wired_report(device: &UsbDevice) -> Vec<String> {
    match usb_telemetry(device) {
        Some(telemetry) => describe_telemetry(&telemetry),
        None => Vec::new(),
    }
}

fn usb_telemetry(device: &UsbDevice) -> Option<Telemetry> {
    if device.bootloader.is_some() {
        return None; // a board in its bootloader runs no keyboard firmware
    }

    let conn = HidConnection::open(device.vid, device.pid).ok()?;
    conn.send(CMD_GET_TELEMETRY, &[]);

    let deadline = Instant::now() + TELEMETRY_WAIT;
    loop {
        let left = deadline.checked_duration_since(Instant::now())?;
        match conn.recv_timeout(left)? {
            HidEvent::Telemetry(telemetry) => return Some(telemetry),
            HidEvent::Disconnected => return None,
            _ => {} // the connection handshake, on its way past
        }
    }
}

/// The lines under a wired board: what each half has left, where each reading
/// came from, and what is plugged into it.
fn describe_telemetry(telemetry: &Telemetry) -> Vec<String> {
    let mut lines = Vec::new();

    let halves: Vec<String> = [("left", telemetry.left), ("right", telemetry.right)]
        .into_iter()
        .map(|(half, reading)| format!("{half} {}", read_as(reading)))
        .collect();
    lines.push(format!("battery · {}", halves.join(" · ")));

    let pointing = [
        ("build", telemetry.nav_build),
        ("left", telemetry.nav_left),
        ("right", telemetry.nav_right),
    ]
    .into_iter()
    .map(|(where_, nav)| format!("{where_} {}", named(nav)))
    .collect::<Vec<_>>();
    lines.push(format!("pointing · {}", pointing.join(" · ")));

    if let Some(line) = charging_as(telemetry.charging) {
        lines.push(line);
    }

    lines
}

/// How fast the board is charging and what decided that.
///
/// Two currents, because they are two different things and confusing them is
/// what the firmware used to do: what the board draws from the source, and
/// what reaches the cells. Nothing at all for firmware that predates the
/// fields — an empty line would read as a board that answered "none".
fn charging_as(charging: Charging) -> Option<String> {
    let state = charging.state?;
    let mut parts = vec![charging_state_as(state).to_string()];

    if let Some(ma) = charging.input_limit_ma {
        parts.push(format!("{ma} mA from the source"));
    }
    if let Some(ma) = charging.charge_current_ma {
        parts.push(format!("{ma} mA to each cell"));
    }
    // What the source said it could give, when that is not already obvious
    // from the limit we settled on.
    match charging.budget_ma {
        Some(0) | None => {}
        Some(ma) if Some(ma) == charging.input_limit_ma => {}
        Some(ma) => parts.push(format!("source offers {ma} mA")),
    }

    Some(format!("charging · {}", parts.join(" · ")))
}

fn charging_state_as(state: ChargeState) -> String {
    match state {
        ChargeState::Battery => "on battery".to_string(),
        ChargeState::Floor => "holding at the unconfigured limit".to_string(),
        ChargeState::Charging => "charging".to_string(),
        ChargeState::Full => "full".to_string(),
        ChargeState::Other(raw) => format!("a state this zapp does not know ({raw})"),
    }
}

/// One half's level, what it is doing, and which path the number took. The
/// path is worth the words: a relayed level keeps its last value when the link
/// carrying it stops, so it reads as current long after it stopped being so.
fn read_as(reading: BatteryReading) -> String {
    let level = match reading.level {
        Some(level) => format!("{level}%"),
        None => "unknown".to_string(),
    };
    let doing = match (reading.charging, reading.usb) {
        (Some(true), _) => " charging",
        (Some(false), Some(true)) => " on USB",
        _ => "",
    };
    match reading.source.filter(|_| reading.level.is_some()).and_then(came_from) {
        Some(from) => format!("{level}{doing} ({from})"),
        None => format!("{level}{doing}"),
    }
}

/// Where a reading came from, when that tells the reader something.
///
/// A half reporting its own cell is the ordinary case and saying so is noise.
/// The rest earn their words: over the cable is live, over the radio may be a
/// moment old, and a source this build does not know should not be dressed up
/// as one it does.
fn came_from(source: Source) -> Option<&'static str> {
    match source {
        Source::Nrf => None,
        Source::Wired => Some("read over the cable"),
        Source::Esb => Some("relayed over the radio"),
        Source::Unknown => Some("no source"),
        Source::Other(_) => Some("a source this zapp does not know"),
    }
}

/// What a nav mask names. An empty mask is a board that looked and found
/// nothing; no mask at all is one that could not tell, and the two must not
/// print the same.
fn named(nav: Option<Nav>) -> String {
    let Some(nav) = nav else {
        return "unknown".to_string();
    };
    let devices: Vec<&str> = [("trackball", nav.trackball), ("trackpad", nav.trackpad)]
        .into_iter()
        .filter(|&(_, present)| present)
        .map(|(name, _)| name)
        .collect();

    match devices.is_empty() {
        true => "none".to_string(),
        false => devices.join(" + "),
    }
}

/// What the halves have left.
///
/// A half the board does not publish is left out; one it publishes without a
/// level is named and called unknown, because a half that is there and quiet is
/// worth saying and a half that is not there is not.
fn charge(battery: Battery) -> String {
    let halves: Vec<String> = [("left", battery.left), ("right", battery.right)]
        .into_iter()
        .filter(|(_, level)| !level.is_absent())
        .map(|(half, level)| format!("{half} {}", said(level)))
        .collect();

    match halves.is_empty() {
        false => format!(" · {}", halves.join(" · ")),
        // A board with one battery, which reports only the whole.
        true => match battery.keyboard {
            Level::Percent(level) => format!(" · {level}%"),
            _ => String::new(),
        },
    }
}

/// Points at the wired entry for a board that is listed in both places. What
/// the radio published is what the board last had reason to publish; plugged
/// in, it answers for itself. Saying which is which is the whole difference
/// between a reading and a memory.
fn also_wired(keyboard: Option<device::Keyboard>, wired: &[device::Keyboard]) -> &'static str {
    match keyboard.is_some_and(|model| wired.contains(&model)) {
        true => " · also on USB above, which reads live",
        false => "",
    }
}

fn said(level: Level) -> String {
    match level {
        Level::Percent(level) => format!("{level}%"),
        _ => "unknown".to_string(),
    }
}

fn state(presence: Presence) -> &'static str {
    match presence {
        Presence::Connected => "connected",
        Presence::Nearby => "in range",
        Presence::Paired => "paired, not in range",
    }
}

fn describe_usb(device: &UsbDevice) -> String {
    let name = match device.bootloader {
        Some(_) => ids::friendly_name(device.vid, device.pid).to_string(),
        None => target_name_for_pid(device.pid).to_string(),
    };
    match &device.serial {
        Some(serial) if !serial.is_empty() => format!("{name} · {serial}"),
        _ => name,
    }
}

fn cmd_flash_file(path: &PathBuf, force: bool, device: Option<&str>) -> Result<()> {
    // A bundle carries both halves' wireless firmware and goes over the cable
    // through the bootloader; everything else is a keyboard image for the
    // STM32. Told apart by what the file says it is rather than by its name,
    // so a renamed bundle still does the right thing instead of failing deep
    // inside the DFU loader.
    if is_bundle(path) {
        return flash_bundle(path, force, device);
    }

    if force {
        // Said rather than ignored: a keyboard image is always written, so the
        // flag is not doing anything here and silently accepting it would
        // suggest otherwise.
        println!("--force only applies to .zsab bundles; a keyboard image is always written.");
    }

    let fw = firmware::load_firmware(path).context("Failed to load firmware")?;

    // A Voyager 2 image can go either way, so it picks the way and says which
    // one it picked; everything else is a wired update and has nothing to pick
    // between.
    if let Firmware::DfuBinary { data, pid, .. } = &fw
        && is_voyager2_image(*pid)
    {
        return flash_voyager2_image(&fw, data, device);
    }

    print_firmware_info(&fw, Transport::Usb);

    if device.is_some() {
        println!(
            "--device only applies to firmware going over Bluetooth: .zsab bundles \
             and Voyager 2 keyboard images."
        );
    }
    wait_and_flash(&fw)
}

/// Whether a DFU suffix names a Voyager 2, either revision.
fn is_voyager2_image(pid: u16) -> bool {
    matches!(pid, 0x1978 | 0x0792)
}

/// The way an update reaches the keyboard, for saying so on the loaded line.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Transport {
    Usb,
    Bluetooth,
}

impl Transport {
    fn label(self) -> &'static str {
        match self {
            Transport::Usb => "USB",
            Transport::Bluetooth => "Bluetooth",
        }
    }
}

/// Which way a Voyager 2 update goes: the cable whenever the keyboard is on it,
/// in its bootloader or running its own firmware, Bluetooth otherwise.
fn route() -> Transport {
    if nrf_wired::bootloader_present() || nrf_wired::voyager2_present() {
        Transport::Usb
    } else {
        Transport::Bluetooth
    }
}

/// A Voyager 2 keyboard image, over whichever way the keyboard is reachable.
///
/// The same choice a bundle makes, for the same reasons: the cable whenever the
/// Voyager 2 is on it, Bluetooth only for a keyboard that is not plugged in at
/// all. Over the cable `named` is ignored, as there is only ever one keyboard
/// plugged in.
fn flash_voyager2_image(fw: &Firmware, data: &[u8], named: Option<&str>) -> Result<()> {
    let over = route();
    print_firmware_info(fw, over);
    if over == Transport::Usb {
        return wait_and_flash(fw);
    }

    let result = flash_over_ble(data, named);
    if result.is_err() {
        println!();
        println!(
            "If you meant to update over the cable, press the reset button on \
             the left half to put the keyboard in its bootloader and run this again."
        );
    }
    result
}

/// A bundle, over whichever way the keyboard is reachable.
///
/// The cable wins whenever the Voyager 2 is on it, whether it is already in its
/// bootloader or still running its own firmware — the wired path waits for the
/// bootloader and asks for the reset press. Bluetooth is for a keyboard that
/// is not plugged in at all, not for one that is plugged in and simply has not
/// been put in its bootloader yet: that one is a cable update the user is part
/// way through, and sending it over the radio instead would be a surprise.
///
/// The model is what decides, not "some ZSA board is on USB": a user with an
/// original Voyager on the cable and a Voyager 2 on Bluetooth has to get the
/// radio, or the update goes down a wire the bundle can never reach the board
/// through and stops asking for a reset button that would not help.
///
/// `named` picks between bonded boards over Bluetooth. The cable ignores it:
/// there is only ever one keyboard plugged in.
fn flash_bundle(path: &Path, force: bool, named: Option<&str>) -> Result<()> {
    if route() == Transport::Usb {
        return nrf_wired::flash(path, force);
    }

    let link = LinkArgs {
        device: named.map(str::to_string),
        ..LinkArgs::default()
    };
    let result = zsab::update(path, force, &link);
    if result.is_err() {
        println!();
        println!(
            "If you meant to update over the cable, press the reset button on \
             the left half to put the keyboard in its bootloader and run this again."
        );
    }
    result
}

/// Whether the file leads with the .zsab magic.
fn is_bundle(path: &Path) -> bool {
    let mut magic = [0u8; 4];
    File::open(path)
        .and_then(|mut file| file.read_exact(&mut magic))
        .is_ok_and(|()| &magic == b"ZSAB")
}

/// Download a revision's firmware, showing a spinner while it transfers.
fn download_firmware(revision_id: &str, variant: FirmwareVariant) -> Result<Firmware> {
    let spinner = new_spinner("Downloading firmware...");
    let fw = zapp_oryx::download_firmware(revision_id, variant);
    spinner.finish_and_clear();

    fw.context("Failed to download firmware")
}

fn cmd_flash_url(url: &str) -> Result<()> {
    let (geometry, layout_id, revision_id) = zapp_oryx::parse_url(url)?;
    let revision = zapp_oryx::resolve_revision(geometry, layout_id, revision_id)
        .context("Failed to fetch latest revision")?;

    println!("Layout: {layout_id}, revision: {revision}");

    // The Moonlander's two halves ship as one collated image.
    let variant = if geometry == "moonlander" {
        FirmwareVariant::Collated
    } else {
        FirmwareVariant::Standard
    };

    let fw = download_firmware(&revision, variant)?;
    print_firmware_info(&fw, Transport::Usb);
    wait_and_flash(&fw)
}

fn cmd_update() -> Result<()> {
    let connected = device::find_keyboard().context("Failed to scan USB devices")?;
    println!("Found {} connected", connected.keyboard);

    // Parse serial number as "layoutId/revisionId"
    let Some((layout_id, revision_id)) = connected.serial.split_once('/') else {
        bail!(
            "Updates only work if your keyboard is currently flashed with a firmware coming from Oryx."
        );
    };

    if layout_id.is_empty() || revision_id.is_empty() {
        bail!(
            "Updates only work if your keyboard is currently flashed with a firmware coming from Oryx."
        );
    }

    println!("Layout: {layout_id}, revision: {revision_id}");

    let latest =
        zapp_oryx::fetch_latest_revision(layout_id).context("Failed to check for updates")?;

    if revision_id == latest {
        println!("Firmware is already up to date.");
        return Ok(());
    }

    println!("Update available: {revision_id} → {latest}");

    let geometry = zapp_oryx::geometry_for(connected.keyboard, connected.pid);
    print_revision_note("current", layout_id, revision_id, geometry);
    print_revision_note("update", layout_id, &latest, geometry);

    let variant = if is_moonlander_revb(connected.pid) {
        FirmwareVariant::Alternate
    } else {
        FirmwareVariant::Standard
    };

    let fw = download_firmware(&latest, variant)?;
    print_firmware_info(&fw, Transport::Usb);
    wait_and_flash(&fw)
}

/// Print the commit message attached to a revision. A revision may carry no
/// message, and a note is never worth failing an update over, so both cases
/// print the same placeholder.
fn print_revision_note(label: &str, layout_id: &str, revision_id: &str, geometry: &str) {
    let note = match zapp_oryx::fetch_revision_note(layout_id, revision_id, geometry) {
        Ok(note) => note,
        Err(e) => {
            log::debug!("Could not fetch note for revision {revision_id}: {e}");
            None
        }
    };

    let note = note.as_deref().unwrap_or("(no note)");
    println!("  {label:<7} {revision_id}: {note}");
}

fn print_firmware_info(fw: &Firmware, over: Transport) {
    let desc = match fw {
        Firmware::DfuBinary { data, pid, .. } => {
            format!("{} ({} bytes)", target_name_for_pid(*pid), data.len())
        }
        Firmware::IgnitionDual { primary, alternate } => {
            format!(
                "{} + {} ({} + {} bytes)",
                target_name_for_pid(primary.pid),
                target_name_for_pid(alternate.pid),
                primary.data.len(),
                alternate.data.len()
            )
        }
        Firmware::IntelHex { data } => {
            format!("Ergodox EZ ({} bytes)", data.len())
        }
    };
    println!("Firmware loaded: {desc} flashing over {}", over.label());
}

pub(crate) fn new_spinner(msg: &str) -> ProgressBar {
    let spinner = ProgressBar::new_spinner();
    spinner.set_style(
        ProgressStyle::default_spinner()
            .tick_strings(&["_", "_", "_", "-", "`", "`", "'", "´", "-", "_", "_", "_"])
            .template("{spinner:.cyan} {msg}")
            .unwrap(),
    );
    spinner.set_message(msg.to_string());
    spinner.enable_steady_tick(std::time::Duration::from_millis(100));
    spinner
}

/// The one bar every flash draws, so a bundle's two halves and a keyboard image
/// look like the same program.
///
/// `total` is what the position is measured against; every caller counts in
/// percent. `prefix` names the half on a bundle's two bars, padded so they line
/// up under each other, and is empty for a lone image.
pub(crate) fn flash_bar(total: u64, prefix: &str) -> ProgressBar {
    let bar = ProgressBar::new(total);
    let template = match prefix.is_empty() {
        true => "⚡ {bar:40.cyan/blue} {pos}% {msg}",
        false => "⚡ {prefix:<5} {bar:40.cyan/blue} {pos}% {msg}",
    };
    bar.set_style(
        ProgressStyle::default_bar()
            .template(template)
            .unwrap()
            .progress_chars("█▓░"),
    );
    bar.set_prefix(prefix.to_string());
    bar
}

/// Connects to the board these arguments name, saying so while it happens.
///
/// The spinner is the whole point of doing this in one place: a board that is
/// bonded but idle takes seconds to wake, and silence there looks like a hang.
/// Only for a connect nothing else is already covering — two spinners share one
/// line and redraw over each other, so a caller holding one of its own wants
/// [`smp_connect_quiet`].
pub fn smp_connect(args: &LinkArgs, half: Half) -> Result<SmpClient> {
    let spinner = new_spinner(match half {
        Half::Left => "Connecting over Bluetooth...",
        Half::Right => "Connecting to the right half over Bluetooth...",
    });
    let client = smp_connect_quiet(args, half);
    spinner.finish_and_clear();
    client
}

/// The same connection, for a caller that is already saying what it is waiting
/// for.
pub fn smp_connect_quiet(args: &LinkArgs, half: Half) -> Result<SmpClient> {
    let client = smp::open(args.device.as_deref(), half);

    let mut client = client.with_context(|| match half {
        Half::Left => "Could not reach the board over Bluetooth".to_string(),
        Half::Right => "Could not reach the right half over Bluetooth. It is reached \
                        through the left, so the left must be running firmware with the \
                        relay in it, and the halves' radio link must be up."
            .to_string(),
    })?;
    if let Some(mtu) = args.mtu {
        client = client.with_mtu(mtu);
    }
    let mut client = client.with_timeout(Duration::from_secs(args.timeout));

    // The timeout has to be set first: this is a request like any other, and
    // it is the first thing the board is asked.
    match args.chunk {
        Some(chunk) => client = client.with_request_limit(chunk),
        None => {
            // What the board says it can reassemble is more than it will take.
            // Reassembly across writes carries no sequence numbers and no
            // checksum — neither Zephyr's nor the left relay's — so a fragment
            // lost anywhere discards the request silently and no reply ever
            // comes; the more writes a request spans, the more of that
            // exposure it buys. Measured on a Voyager 2's left half (reports
            // 2475, 249 per write): os_mgmt echo answers up to 8 writes and
            // stops dead at 9, and an upload of a 316 KB image takes 51 s at
            // one write, 25 s at five, 22 s at eight. Five keeps margin under
            // the ceiling for most of the gain.
            //
            // The right half is not measured: its ceiling is the 1024-byte
            // frame the relay and its own reassembly buffer share, which five
            // writes already exceeds, so the reported buffer is what binds and
            // the count only matters if that frame size changes. Same default
            // either way; `--writes` and `--chunk` override it.
            const WRITES_PER_REQUEST: usize = 5;
            let writes = args.writes.unwrap_or(WRITES_PER_REQUEST).max(1);
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

/// Connects to the update service on the board `device` names.
fn dfu_connect(device: Option<&str>) -> Result<DfuClient> {
    let spinner = new_spinner("Connecting over Bluetooth...");
    let client = dfu::open(device);
    spinner.finish_and_clear();

    let client = client.context("Could not reach the keyboard over Bluetooth")?;
    log::debug!("connected, {} bytes of image per write", client.chunk());
    Ok(client)
}

/// The board's own advice about a refusal, wherever in the chain it is.
fn dfu_advice(err: &anyhow::Error) -> Option<&'static str> {
    err.chain()
        .find_map(|cause| cause.downcast_ref::<DfuError>())
        .and_then(DfuError::advice)
}

/// Passes a result through, saying what to do about a refusal the board itself
/// raised. The board's own reasons come back as a code and a sentence; the
/// paragraph that says what to do about one is worth more than the line the
/// error chain gives it.
fn with_dfu_advice(result: Result<()>) -> Result<()> {
    if let Err(err) = &result
        && let Some(advice) = dfu_advice(err)
    {
        println!();
        println!("{advice}");
    }
    result
}

/// Seconds to give the STM32 to reach its bootloader, when nobody said
/// otherwise.
const BLE_BOOTLOADER_WAIT: Duration = Duration::from_secs(30);

/// A keyboard image into the STM32, over ZSA's update service on the left half,
/// which relays it over the wire between the chips into the ignition bootloader.
fn flash_over_ble(image: &[u8], device: Option<&str>) -> Result<()> {
    // Refused here rather than by the keyboard: the board answers this with one
    // error byte, and the sentence that carries both figures is the one that
    // says what to do about it.
    if image.len() > dfu::MAX_IMAGE {
        bail!(
            "This image is {} bytes, and the STM32's application region holds {} ({} KiB). \
             This is a firmware built for another keyboard, or one built to run \
             without the ignition bootloader under it.",
            image.len(),
            dfu::MAX_IMAGE,
            dfu::MAX_IMAGE / 1024
        );
    }

    log::debug!("image {} bytes, crc-32 {:08x}", image.len(), dfu::crc32(image));

    let mut client = dfu_connect(device)?;

    let spinner = new_spinner("Waiting for keyboard in bootloader mode...");
    let ready = client.make_ready(BLE_BOOTLOADER_WAIT, &report_wait);
    spinner.finish_and_clear();
    ready.context("The STM32 is not in its bootloader, so nothing was written")?;

    let pb = flash_bar(100, "");
    let started = Instant::now();
    let result = client.flash(image, &|progress| match progress {
        FlashProgress::Erasing {
            bytes_erased,
            total_bytes,
        } => {
            pb.set_position(((bytes_erased * 100) / total_bytes) as u64);
            pb.set_message("Erasing...");
        }
        FlashProgress::Writing {
            bytes_written,
            total_bytes,
        } => {
            pb.set_position(((bytes_written * 100) / total_bytes) as u64);
            pb.set_message("Writing...");
        }
        FlashProgress::Resetting => {
            pb.set_position(100);
            pb.set_message("Resetting...");
        }
        FlashProgress::Complete => pb.finish_with_message("Done!"),
    });
    if let Err(err) = result {
        // The bar owns its line, so it goes before anything is said about why.
        pb.finish_and_clear();
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
fn report_wait(wait: Wait) {
    match wait {
        Wait::Already => log::debug!("the STM32 is already in its bootloader"),
        Wait::Rebooting => log::debug!("rebooting the STM32 into its bootloader"),
        Wait::Waiting(waited) => log::debug!("not there yet after {} s", waited.as_secs()),
        Wait::Arrived(waited) => log::debug!("in the bootloader after {} s", waited.as_secs()),
    }
}

fn wait_and_flash(fw: &Firmware) -> Result<()> {
    let spinner = new_spinner("Waiting for keyboard in bootloader mode...");

    let device = device::wait_for_bootloader(None, |status| match status {
        WatchStatus::Waiting => {}
        WatchStatus::Found { .. } => {
            spinner.finish_and_clear();
        }
    })
    .context("Failed to detect bootloader")?;

    let pb = flash_bar(100, "");

    flash::flash_device(&device, fw, &|progress| match progress {
        FlashProgress::Erasing {
            bytes_erased,
            total_bytes,
        } => {
            let pct = (bytes_erased * 100) / total_bytes;
            pb.set_position(pct as u64);
            pb.set_message("Erasing...");
        }
        FlashProgress::Writing {
            bytes_written,
            total_bytes,
        } => {
            let pct = (bytes_written * 100) / total_bytes;
            pb.set_position(pct as u64);
            pb.set_message("Writing...");
        }
        FlashProgress::Resetting => {
            pb.set_position(100);
            pb.set_message("Resetting...");
        }
        FlashProgress::Complete => {
            pb.finish_with_message("Done!");
        }
    })
    .context("Flash failed")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_halves_of_a_split_board_are_named() {
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        assert_eq!(charge(battery), " · left 0% · right 27%");
    }

    #[test]
    fn a_half_that_is_there_and_would_not_say_is_called_unknown() {
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Unknown,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery), " · left 36% · right unknown");
    }

    #[test]
    fn a_half_the_board_does_not_have_is_left_out() {
        // Not the same as one that would not say: naming it would invent a half.
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Absent,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery), " · left 36%");
    }

    #[test]
    fn a_board_with_one_battery_reports_the_whole() {
        let battery = Battery {
            keyboard: Level::Percent(80),
            ..Battery::default()
        };
        assert_eq!(charge(battery), " · 80%");
    }

    fn reading(level: Option<u8>, source: Source) -> BatteryReading {
        BatteryReading {
            level,
            charging: Some(false),
            usb: Some(false),
            source: Some(source),
        }
    }

    #[test]
    fn a_wired_board_says_where_each_level_came_from() {
        let telemetry = Telemetry {
            version: 1,
            left: reading(Some(87), Source::Nrf),
            right: reading(Some(64), Source::Wired),
            nav_build: Some(Nav {
                trackball: true,
                trackpad: false,
            }),
            nav_left: Some(Nav::default()),
            nav_right: Some(Nav {
                trackball: true,
                trackpad: false,
            }),
            charging: Charging {
                budget_ma: Some(1500),
                input_limit_ma: Some(700),
                charge_current_ma: Some(550),
                state: Some(ChargeState::Charging),
            },
        };
        assert_eq!(
            describe_telemetry(&telemetry),
            vec![
                "battery · left 87% · right 64% (read over the cable)",
                "pointing · build trackball · left none · right trackball",
                "charging · charging · 700 mA from the source · 550 mA to each cell · source offers 1500 mA",
            ]
        );
    }

    #[test]
    fn the_two_currents_are_kept_apart() {
        // What the board draws and what reaches the cells are different
        // numbers, and reporting one as the other is the bug this line exists
        // to make visible.
        assert_eq!(
            charging_as(Charging {
                budget_ma: Some(500),
                input_limit_ma: Some(200),
                charge_current_ma: Some(50),
                state: Some(ChargeState::Charging),
            }),
            Some(
                "charging · charging · 200 mA from the source · 50 mA to each cell · source offers 500 mA"
                    .to_string()
            )
        );
    }

    #[test]
    fn a_source_that_has_not_said_what_it_is_reads_as_held_back() {
        assert_eq!(
            charging_as(Charging {
                budget_ma: Some(0),
                input_limit_ma: Some(100),
                charge_current_ma: Some(32),
                state: Some(ChargeState::Floor),
            }),
            Some(
                "charging · holding at the unconfigured limit · 100 mA from the source · 32 mA to each cell"
                    .to_string()
            )
        );
    }

    #[test]
    fn a_source_offering_more_than_we_take_says_so() {
        // A 3 A charger against a board whose input register stops at 1.5 A:
        // the difference is real and the line has to show it.
        assert_eq!(
            charging_as(Charging {
                budget_ma: Some(3000),
                input_limit_ma: Some(1500),
                charge_current_ma: Some(700),
                state: Some(ChargeState::Charging),
            }),
            Some(
                "charging · charging · 1500 mA from the source · 700 mA to each cell · source offers 3000 mA"
                    .to_string()
            )
        );
    }

    #[test]
    fn firmware_without_the_charging_fields_says_nothing_at_all() {
        // Not an empty line and not zeroes: an old board did not answer this
        // question, which is different from answering "none".
        assert_eq!(charging_as(Charging::default()), None);
    }

    #[test]
    fn a_relayed_level_is_not_dressed_up_as_a_live_one() {
        // The distinction this whole payload exists for: same number, and the
        // line has to say which of the two it is.
        assert_eq!(
            read_as(reading(Some(64), Source::Wired)),
            "64% (read over the cable)"
        );
        assert_eq!(
            read_as(reading(Some(64), Source::Esb)),
            "64% (relayed over the radio)"
        );
    }

    #[test]
    fn a_level_that_was_not_read_is_not_a_flat_one() {
        assert_eq!(read_as(reading(None, Source::Wired)), "unknown");
        assert_eq!(
            read_as(reading(Some(0), Source::Wired)),
            "0% (read over the cable)"
        );
    }

    #[test]
    fn unknown_flags_do_not_become_a_confident_no() {
        let unsure = BatteryReading {
            level: Some(50),
            charging: None,
            usb: None,
            source: Some(Source::Nrf),
        };
        assert_eq!(read_as(unsure), "50%");

        let charging = BatteryReading {
            charging: Some(true),
            ..unsure
        };
        assert_eq!(read_as(charging), "50% charging");
    }

    #[test]
    fn a_half_we_could_not_read_is_not_a_half_with_nothing_in_it() {
        assert_eq!(named(None), "unknown");
        assert_eq!(named(Some(Nav::default())), "none");
        assert_eq!(
            named(Some(Nav {
                trackball: true,
                trackpad: true
            })),
            "trackball + trackpad"
        );
    }

    #[test]
    fn a_board_listed_twice_points_at_its_live_entry() {
        let wired = [device::Keyboard::Voyager2];
        assert_eq!(
            also_wired(Some(device::Keyboard::Voyager2), &wired),
            " · also on USB above, which reads live"
        );
        assert_eq!(also_wired(Some(device::Keyboard::Voyager), &wired), "");
        assert_eq!(also_wired(None, &wired), "");
        assert_eq!(also_wired(Some(device::Keyboard::Voyager2), &[]), "");
    }

    #[test]
    fn a_board_that_said_nothing_says_nothing() {
        assert_eq!(charge(Battery::default()), "");
        // Including one connected but unreadable, which must not read as flat.
        let battery = Battery {
            keyboard: Level::Unknown,
            ..Battery::default()
        };
        assert_eq!(charge(battery), "");
    }
}
