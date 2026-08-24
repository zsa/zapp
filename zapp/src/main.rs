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

use zapp_ble::dfu::{self, DfuClient, Wait};
use zapp_ble::smp::{self, Half, SmpClient};
use zapp_ble::{Battery, BleMonitor, Level, Presence};
use zapp_core::device::ids::{self as ids, is_moonlander_revb, target_name_for_pid};
use zapp_core::device::{self, UsbDevice, WatchStatus};
use zapp_core::firmware::{self, Firmware};
use zapp_core::flash::{self, FlashProgress};
use zapp_core::hid::{
    BatteryReading, CMD_GET_TELEMETRY, HidConnection, HidEvent, Nav, Source, Telemetry,
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
            report over USB — which shows both came back and answer, not that the \
            radio link formed. On USB the left switches its radio off, so the link \
            can only be checked with no cables: unplug USB and the cable between \
            the halves and run `zapp devices`, which shows both halves' batteries \
            over Bluetooth when they are linked.\n\n\
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

/// Runs the command the arguments name, and says one line about a failure.
fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();

    let cli = Cli::parse();

    if let Err(err) = dispatch(cli.command) {
        eprintln!("Error: {err:#}");
        std::process::exit(1);
    }
}

/// One command, run to whatever it comes to.
fn dispatch(command: Commands) -> Result<()> {
    match command {
        Commands::Flash {
            firmware,
            force,
            device,
        } => {
            if zapp_oryx::is_oryx_url(&firmware) {
                cmd_flash_url(&firmware)
            } else {
                cmd_flash_file(&PathBuf::from(firmware), force, device.as_deref())
            }
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
            "  {name} · {} · {known_as:?}{}",
            state(board.presence),
            charge(board.battery),
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

/// The lines under a wired board: what each half has left and how fast it is
/// filling, and which pointing devices the build carries and answer.
fn describe_telemetry(telemetry: &Telemetry) -> Vec<String> {
    let speed = charge_speed(telemetry.charging.budget_ma);
    let halves: Vec<String> = [("left", telemetry.left), ("right", telemetry.right)]
        .into_iter()
        .map(|(half, reading)| format!("{half} {}", read_as(reading, speed)))
        .collect();

    vec![
        format!("{:<8} · {}", "battery", halves.join(" · ")),
        format!(
            "{:<8} · {}",
            "features",
            features(telemetry.nav_build, telemetry.nav_left, telemetry.nav_right)
        ),
    ]
}

/// The port's declared capacity, not what is presently drawn — a source that
/// only offered the default 500 mA can never charge fast no matter what
/// gets negotiated after. Mirrors ZSA_PWR_BUDGET_HIGH_MA in the firmware's
/// zsa_uart_proto.h.
const FAST_CHARGE_BUDGET_MA: u16 = 1500;

fn charge_speed(budget_ma: Option<u16>) -> Option<&'static str> {
    match budget_ma {
        Some(0) | None => None, // no source identified, or firmware predates the field
        Some(ma) if ma >= FAST_CHARGE_BUDGET_MA => Some("fast"),
        Some(_) => Some("slow"),
    }
}

/// One half's level, what it is doing, and which path the number took. The
/// path is worth the words: a relayed level keeps its last value when the link
/// carrying it stops, so it reads as current long after it stopped being so.
fn read_as(reading: BatteryReading, speed: Option<&str>) -> String {
    let level = match reading.level {
        Some(level) => format!("{level}%"),
        None => "unknown".to_string(),
    };
    let doing = match (reading.charging, reading.usb) {
        (Some(true), _) => match speed {
            Some(speed) => format!(" {speed} charging"),
            None => " charging".to_string(),
        },
        (Some(false), Some(true)) => " on USB".to_string(),
        _ => String::new(),
    };
    match reading.source.filter(|_| reading.level.is_some()).and_then(came_from) {
        Some(from) => format!("{level}{doing} ({from})"),
        None => format!("{level}{doing}"),
    }
}

/// Where a reading came from, when that tells the reader something.
///
/// A half reporting its own cell and a half read over the cable are both the
/// ordinary case here and saying so is noise. The rest earn their words: over
/// the radio may be a moment old, and a source this build does not know should
/// not be dressed up as one it does.
fn came_from(source: Source) -> Option<&'static str> {
    match source {
        Source::Nrf => None,
        Source::Wired => None,
        Source::Esb => Some("relayed over the radio"),
        Source::Unknown => Some("no source"),
        Source::Other(_) => Some("a source this zapp does not know"),
    }
}

/// One entry per pointing device this build knows about: not supported when
/// the build was never compiled to drive it, connected when either half
/// currently reports it present, disconnected when the build supports it but
/// neither half does.
fn features(build: Option<Nav>, left: Option<Nav>, right: Option<Nav>) -> String {
    let devices: [(&str, fn(Nav) -> bool); 2] = [
        ("Trackball", |nav| nav.trackball),
        ("Trackpad", |nav| nav.trackpad),
    ];
    devices
        .into_iter()
        .map(|(name, has)| {
            let state = match build.filter(|&build| has(build)) {
                None => "not supported",
                Some(_) => match left.is_some_and(has) || right.is_some_and(has) {
                    true => "connected",
                    false => "disconnected",
                },
            };
            format!("{name} ({state})")
        })
        .collect::<Vec<_>>()
        .join(", ")
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
    if let Err(err) = &result {
        // Not for a keyboard that refused the image: the cable changes nothing
        // about a signature its bootloader does not trust.
        if err.downcast_ref::<zsab::RightRejected>().is_none() {
            println!();
            println!(
                "If you meant to update over the cable, press the reset button on \
                 the left half to put the keyboard in its bootloader and run this again."
            );
        }
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
    if let Err(err) = &result {
        if err.downcast_ref::<zsab::RightRejected>().is_none() {
            println!();
            println!(
                "If you meant to update over the cable, press the reset button on \
                 the left half to put the keyboard in its bootloader and run this again."
            );
        }
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
    use zapp_core::hid::Charging;

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

    fn charging_reading(level: Option<u8>, source: Source) -> BatteryReading {
        BatteryReading {
            charging: Some(true),
            ..reading(level, source)
        }
    }

    #[test]
    fn a_wired_board_says_what_each_half_is_doing() {
        let telemetry = Telemetry {
            version: 1,
            left: reading(Some(87), Source::Nrf),
            right: charging_reading(Some(64), Source::Wired),
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
                ..Charging::default()
            },
        };
        assert_eq!(
            describe_telemetry(&telemetry),
            vec![
                "battery  · left 87% · right 64% fast charging",
                "features · Trackball (connected), Trackpad (not supported)",
            ]
        );
    }

    #[test]
    fn only_a_high_power_source_charges_fast() {
        // The port's declared capacity decides this, and a source that never
        // said what it is decides nothing.
        assert_eq!(charge_speed(Some(1500)), Some("fast"));
        assert_eq!(charge_speed(Some(500)), Some("slow"));
        assert_eq!(charge_speed(Some(0)), None);
        assert_eq!(charge_speed(None), None);
    }

    #[test]
    fn a_relayed_level_is_not_dressed_up_as_a_live_one() {
        // The distinction this whole payload exists for: same number, and the
        // line has to say which of the two it is.
        assert_eq!(read_as(reading(Some(64), Source::Wired), None), "64%");
        assert_eq!(
            read_as(reading(Some(64), Source::Esb), None),
            "64% (relayed over the radio)"
        );
    }

    #[test]
    fn a_level_that_was_not_read_is_not_a_flat_one() {
        assert_eq!(read_as(reading(None, Source::Wired), None), "unknown");
        assert_eq!(read_as(reading(Some(0), Source::Wired), None), "0%");
    }

    #[test]
    fn unknown_flags_do_not_become_a_confident_no() {
        let unsure = BatteryReading {
            level: Some(50),
            charging: None,
            usb: None,
            source: Some(Source::Nrf),
        };
        assert_eq!(read_as(unsure, None), "50%");
        // A speed says nothing about a half that is not charging.
        assert_eq!(read_as(unsure, Some("fast")), "50%");

        let charging = BatteryReading {
            charging: Some(true),
            ..unsure
        };
        assert_eq!(read_as(charging, None), "50% charging");
        assert_eq!(read_as(charging, Some("fast")), "50% fast charging");
    }

    #[test]
    fn a_device_the_build_cannot_drive_is_not_a_missing_one() {
        let ball = Nav {
            trackball: true,
            trackpad: false,
        };
        let both = Nav {
            trackball: true,
            trackpad: true,
        };
        assert_eq!(
            features(Some(ball), Some(Nav::default()), Some(ball)),
            "Trackball (connected), Trackpad (not supported)"
        );
        assert_eq!(
            features(Some(both), Some(Nav::default()), Some(Nav::default())),
            "Trackball (disconnected), Trackpad (disconnected)"
        );
        assert_eq!(
            features(None, Some(ball), None),
            "Trackball (not supported), Trackpad (not supported)"
        );
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
