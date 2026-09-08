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
    BatteryDetail, BatteryReading, BleIdentity, CMD_GET_CHARGE_TELEMETRY, CMD_GET_PMIC_RAILS,
    CMD_GET_TELEMETRY, ChargeTelemetry, ChargerFault, ChargerStatus, Charging, HidConnection,
    HidEvent, Nav, PmicRails, PmicRailsReport, Telemetry,
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
            bundles carrying both wireless halves, and Oryx layout URLs. A \
            .zsab bundle and a Voyager 2 keyboard image go over the cable when \
            the keyboard is on USB and over Bluetooth otherwise; any other \
            keyboard's firmware goes over the cable. A bundle's two halves are \
            written together, the right one first.",
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
        /// older than what is installed. Applies to .zsab bundles only.
        #[arg(long)]
        force: bool,
        /// Which board, by advertised name or by the handle the device list
        /// shows. Applies over Bluetooth only.
        #[arg(long, short = 'd')]
        device: Option<String>,
    },
    /// Check Oryx for updates on your layout and flash if available
    #[command(
        long_about = "Check Oryx for updates on your layout and flash if available.\n\n\
            Reads a connected ZSA keyboard's layout and revision from its USB serial \
            number, and downloads and flashes the newest revision Oryx has for it."
    )]
    Update,
    /// List the ZSA keyboards this machine can see, wired and wireless
    #[command(long_about = "List the ZSA keyboards this machine can see.\n\n\
            Wired boards are read straight from USB. Bluetooth boards are listed \
            when they are connected to this machine, bonded to it, or advertising \
            in range, and --scan holds a discovery window open for boards that have \
            never been paired.")]
    Devices {
        /// Scan for boards that have never been paired to this machine
        #[arg(long)]
        scan: bool,
        /// Also ask each half what its cell and its charger are reading
        #[arg(long, short = 'v')]
        verbose: bool,
    },
    /// Pair the two halves by provisioning their shared ESB key over the cable
    #[command(
        long_about = "Provision the ESB pairing key onto both halves over the cable.\n\n\
            Writes one fresh 30-byte key to both halves through the bootloader, reads \
            it back off each, then reboots the keyboard and waits for both halves to \
            report over USB. --key writes a specific key instead of generating one, \
            and --show reads what the two halves carry without writing anything.",
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
        Commands::Devices { scan, verbose } => cmd_devices(scan, verbose),
        Commands::Bond { key, show } => {
            if show {
                if key.is_some() {
                    anyhow::bail!("--show and --key cannot be combined.");
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

fn cmd_devices(scan: bool, verbose: bool) -> Result<()> {
    let wired = device::list_devices();

    println!("USB");
    match &wired {
        Ok(devices) if devices.is_empty() => println!("  none"),
        Ok(devices) => {
            for device in devices {
                println!("  {}", describe_usb(device));
                for line in wired_report(device, verbose) {
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
    }
    for board in boards {
        let name = board
            .keyboard
            .map(|keyboard| keyboard.to_string())
            .unwrap_or_else(|| "ZSA keyboard".to_string());
        let known_as = board.name.clone().unwrap_or_else(|| "unnamed".to_string());
        // The address only when the platform handle spells one out.
        let address = match &board.address {
            Some(address) => format!(" · {address}"),
            None => String::new(),
        };
        println!(
            "  {name} · {} · {known_as:?}{address}{}",
            state(board.presence),
            charge(board.battery),
        );
        for line in ble_report(&board, verbose) {
            println!("    {line}");
        }
    }

    Ok(())
}

/// Ask a wired board about itself, and say what it answered.
///
/// A board is preferred over the radio for its own state: plugged in, it can
/// read both halves directly, and the Bluetooth view of a wired board is
/// whatever the radio last carried. Firmware without the command simply never
/// answers, so an empty result here is the normal old-firmware case and says
/// nothing. Only Voyager 2 has a battery or pointing-device header to report
/// on; the rest have neither, so there is nothing worth asking them.
fn wired_report(device: &UsbDevice, verbose: bool) -> Vec<String> {
    if device.keyboard != Some(ids::Keyboard::Voyager2) {
        return Vec::new();
    }
    let Some((telemetry, detail, rails)) = usb_telemetry(device, verbose) else {
        return Vec::new();
    };

    let mut lines = describe_telemetry(&telemetry, detail.as_ref());
    if let Some(detail) = detail {
        lines.extend(describe_charge(&telemetry, &detail, rails));
    }
    lines
}

/// What the board answers, on one connection: the summary always, and the
/// detail behind it only when it was asked for. A board whose firmware predates
/// any of these commands never answers it, which is why each wait can elapse.
type UsbTelemetry = (Telemetry, Option<ChargeTelemetry>, Option<PmicRailsReport>);

fn usb_telemetry(device: &UsbDevice, detail: bool) -> Option<UsbTelemetry> {
    if device.bootloader.is_some() {
        return None; // a board in its bootloader runs no keyboard firmware
    }

    let conn = HidConnection::open(device.vid, device.pid).ok()?;
    let telemetry = ask(&conn, CMD_GET_TELEMETRY, |event| match event {
        HidEvent::Telemetry(telemetry) => Some(telemetry),
        _ => None,
    })?;
    let charge = detail
        .then(|| {
            ask(&conn, CMD_GET_CHARGE_TELEMETRY, |event| match event {
                HidEvent::ChargeTelemetry(telemetry) => Some(telemetry),
                _ => None,
            })
        })
        .flatten();
    let rails = detail
        .then(|| {
            ask(&conn, CMD_GET_PMIC_RAILS, |event| match event {
                HidEvent::PmicRails(report) => Some(report),
                _ => None,
            })
        })
        .flatten();

    Some((telemetry, charge, rails))
}

/// Send one query and wait out the answer `take` is looking for.
///
/// Everything else on the way past is the connection's own handshake, which
/// runs whether anything wants it or not.
fn ask<T>(conn: &HidConnection, cmd: u8, take: impl Fn(HidEvent) -> Option<T>) -> Option<T> {
    conn.send(cmd, &[]);

    let deadline = Instant::now() + TELEMETRY_WAIT;
    loop {
        let left = deadline.checked_duration_since(Instant::now())?;
        match conn.recv_timeout(left)? {
            HidEvent::Disconnected => return None,
            event => {
                if let Some(answer) = take(event) {
                    return Some(answer);
                }
            }
        }
    }
}

/// The lines under a wired board: what each half has left and how fast it is
/// filling, and which pointing devices the build carries and answer.
///
/// The charger phase from the detail read words each half when it arrived, so
/// this line and the one under it say the same thing; the summary's charging
/// flag words it only when no detail came.
fn describe_telemetry(telemetry: &Telemetry, detail: Option<&ChargeTelemetry>) -> Vec<String> {
    let speed = charge_speed(telemetry.charging.budget_ma);
    let halves: Vec<String> = [
        (
            "left",
            telemetry.left,
            detail.and_then(|detail| detail.left.detail.charger),
        ),
        (
            "right",
            telemetry.right,
            detail.and_then(|detail| detail.right.detail.charger),
        ),
    ]
    .into_iter()
    .map(|(half, reading, charger)| {
        let said = match charger {
            Some(status) => read_in_phase(reading, status, speed),
            None => read_as(reading, speed),
        };
        format!("{half} {said}")
    })
    .collect();

    let mut lines = vec![format!("{:<8} · {}", "battery", halves.join(" · "))];
    if let Some(ble) = &telemetry.ble {
        lines.push(format!("{:<8} · {}", "bluetooth", ble_identity(ble)));
    }
    lines.push(format!(
        "{:<8} · {}",
        "features",
        features(telemetry.nav_build, telemetry.nav_left, telemetry.nav_right)
    ));
    lines
}

/// The address the board is using and which of its host slots it is.
///
/// Slots are counted from zero on the wire and from one for a reader, and a
/// board that did not say how many it keeps gets the address alone.
fn ble_identity(ble: &BleIdentity) -> String {
    match ble.slot_count {
        0 => ble.address.clone(),
        count => format!("{} (host {} of {count})", ble.address, ble.slot + 1),
    }
}

/// The lines under a Bluetooth board, when the detail was asked for.
///
/// Only a connected board can be read at all, and only one whose firmware
/// serves the characteristic answers; anything else says nothing, which is the
/// same silence an old wired firmware gives.
fn ble_report(board: &zapp_ble::BleBoard, verbose: bool) -> Vec<String> {
    if !verbose || board.presence != Presence::Connected {
        return Vec::new();
    }
    let Some(detail) = zapp_ble::read_charge_detail(&board.key) else {
        return Vec::new();
    };

    // A board on the radio is a board off the cable, so what it is heading for
    // is empty. The radio read carries no VBUS flag either.
    [
        ("left", detail.left, detail.left_rails),
        ("right", detail.right, detail.right_rails),
    ]
    .into_iter()
    .flat_map(|(half, detail, rails)| {
        let line = detail_line(half, &detail, Estimate::Empty, None);
        // The rails go under the half they belong to, and only when that half
        // measured them: a firmware that predates the block says nothing.
        [Some(line), rails.map(rails_line)]
    })
    .flatten()
    .collect()
}

/// One half's rails: what its input and system rails measure, what the input
/// regulator is doing and whether the charger is switched on.
///
/// Only printed for a half that took a reading, so unlike the detail line
/// nothing here says "unknown" for want of a measurement — only the charger
/// register, which the firmware can decline to read on its own. The boost
/// converter is left off the line entirely by firmware that does not report it.
fn rails_line(rails: PmicRails) -> String {
    let names = rails.vbusin.names();
    let vbusin = match names.is_empty() {
        true => "none".to_string(),
        false => names.join(", "),
    };
    let charger = match rails.charger_enabled {
        Some(true) => "charger enabled",
        Some(false) => "charger disabled",
        None => "charger unknown",
    };

    let boost = match rails.boost_off {
        Some(true) => " · boost off",
        Some(false) => " · boost on",
        None => "",
    };

    // Only the half holding back says so; charging at the commanded rate is
    // the ordinary case and reads as silence.
    let reduced = match rails.charge_reduced {
        true => " · charge reduced",
        false => "",
    };

    format!(
        "{:<8} · VBUS {} · VSYS {} · in: {vbusin} · {charger}{boost}{reduced}",
        "rails",
        volts(rails.vbus_mv),
        volts(rails.vsys_mv),
    )
}

/// The lines under a wired board once its charge telemetry has been read: what
/// the source allows, then what each half's cell and charger are doing.
///
/// The summary read comes along for each half's VBUS flag, which the detail
/// block does not carry and the charger phase needs. The rails go under the
/// half they belong to, and only for a half that measured them.
fn describe_charge(
    summary: &Telemetry,
    telemetry: &ChargeTelemetry,
    rails: Option<PmicRailsReport>,
) -> Vec<String> {
    let mut lines = Vec::new();
    if let Some(rates) = charge_rates(&summary.charging) {
        lines.push(format!("{:<8} · {}", "charging", rates));
    }
    // On the cable the halves are filling, so what they are heading for is full.
    for (half, detail, usb, rails) in [
        (
            "left",
            telemetry.left,
            summary.left.usb,
            rails.and_then(|rails| rails.left),
        ),
        (
            "right",
            telemetry.right,
            summary.right.usb,
            rails.and_then(|rails| rails.right),
        ),
    ] {
        lines.push(detail_line(half, &detail.detail, Estimate::Full, usb));
        lines.extend(rails.map(rails_line));
    }
    lines
}

/// What the source offers, what the board draws from it and what goes into the
/// cells. A figure the board would not give is left out, and a board that gave
/// none of the three has no line.
fn charge_rates(charging: &Charging) -> Option<String> {
    let rates: Vec<String> = [
        ("source", charging.budget_ma),
        ("limit", charging.input_limit_ma),
        ("charge", charging.charge_current_ma),
    ]
    .into_iter()
    .filter_map(|(what, ma)| ma.map(|ma| format!("{what} {ma} mA")))
    .collect();

    (!rates.is_empty()).then(|| rates.join(" · "))
}

/// Which estimate a line leads with: the one the half is heading for on the way
/// it was read. A half read on the radio falls back to the other one, since it
/// can be charging from a wall charger; a half read on the cable does not, since
/// time-to-empty says nothing about a board that is plugged in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Estimate {
    Full,
    Empty,
}

/// One half's line: volts, current, temperature, how long it has, what the
/// charger is doing and whether anything latched.
///
/// Every piece the board would not give says "unknown" rather than dropping
/// out, so the columns stay under each other.
fn detail_line(
    half: &str,
    detail: &BatteryDetail,
    heading_for: Estimate,
    usb: Option<bool>,
) -> String {
    let parts = [
        detail.voltage_mv.map_or_else(unknown, volts),
        detail.current_ma.map_or_else(unknown, milliamps),
        detail.temp_cdeg.map_or_else(unknown, degrees),
        time_left(detail, heading_for),
        detail
            .charger
            .map_or_else(unknown, |status| phase(status, usb)),
        detail.fault.map_or_else(unknown, fault),
    ];

    format!("{half:<8} · {}", parts.join(" · "))
}

fn unknown() -> String {
    "unknown".to_string()
}

fn volts(millivolts: u16) -> String {
    format!("{:.2} V", f64::from(millivolts) / 1000.0)
}

/// Signed, because the sign is the whole of what it says: into the cell or out
/// of it.
fn milliamps(current: i16) -> String {
    format!("{current:+} mA")
}

fn degrees(centidegrees: i16) -> String {
    format!("{:.1} °C", f64::from(centidegrees) / 100.0)
}

/// How long the half has, whichever way it is going. On the cable only
/// time-to-full is said, since a plugged-in board is not heading for empty.
fn time_left(detail: &BatteryDetail, heading_for: Estimate) -> String {
    let estimates: &[(Estimate, Option<u16>)] = match heading_for {
        Estimate::Full => &[(Estimate::Full, detail.ttf_min)],
        Estimate::Empty => &[
            (Estimate::Empty, detail.tte_min),
            (Estimate::Full, detail.ttf_min),
        ],
    };

    match estimates
        .iter()
        .find_map(|&(which, minutes)| minutes.map(|minutes| (which, minutes)))
    {
        Some((Estimate::Full, minutes)) => format!("full in {}", duration(minutes)),
        Some((Estimate::Empty, minutes)) => format!("empty in {}", duration(minutes)),
        None => unknown(),
    }
}

/// Hours and minutes, or minutes on their own under an hour.
fn duration(minutes: u16) -> String {
    match (minutes / 60, minutes % 60) {
        (0, minutes) => format!("{minutes} min"),
        (hours, minutes) => format!("{hours} h {minutes:02} min"),
    }
}

/// What the charger is doing, in one phrase. More than one phase bit at a time
/// is the register's business; the first of them is what the charge is at.
///
/// The no-cell bit only means anything while the half has USB: on battery the
/// firmware reads it clear with a healthy cell in place. So it is said only
/// when the caller knows a source is attached, and otherwise the phase bits
/// are read as they stand.
fn phase(status: ChargerStatus, usb: Option<bool>) -> String {
    match status {
        _ if usb == Some(true) && !status.battery_detected => "no battery detected",
        _ if status.complete => "charge complete",
        _ if status.trickle => "trickle charging",
        _ if status.constant_current => "constant current",
        _ if status.constant_voltage => "constant voltage",
        _ if status.recharge => "recharging",
        _ if status.paused_die_temp => "paused (die temperature)",
        _ => "charger idle",
    }
    .to_string()
}

/// Whether anything latched, and what the sensors read when it did.
///
/// The firmware clears these when a source is attached, so a fault named here
/// is one that happened since.
fn fault(fault: ChargerFault) -> String {
    if fault.is_clear() {
        return "no fault".to_string();
    }

    let reasons = fault.reasons();
    let named = match reasons.is_empty() {
        // A bit this host has no name for: firmware, or hardware, newer than it.
        true => format!("0x{:02x}", fault.reason),
        false => reasons.join(", "),
    };
    let sensors = fault.sensors();
    match sensors.is_empty() {
        true => format!("fault: {named}"),
        false => format!("fault: {named} ({})", sensors.join(", ")),
    }
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

/// One half's level and, if it is charging, how fast.
///
/// The fallback wording, for a board that gave no charge detail: its charging
/// flag is true for a trickle and for a topped-up cell alike.
fn read_as(reading: BatteryReading, speed: Option<&str>) -> String {
    let level = level_as(reading.level);
    match reading.charging {
        Some(true) => format!("{level} {}", charging_as(speed)),
        _ => level,
    }
}

/// One half's level and what its charger is doing, out of the same phase the
/// detail line names. The phase bits are read in the order `phase` reads them,
/// so the two lines never disagree.
fn read_in_phase(reading: BatteryReading, status: ChargerStatus, speed: Option<&str>) -> String {
    let level = level_as(reading.level);
    match status {
        _ if reading.usb == Some(true) && !status.battery_detected => {
            format!("{level} no battery detected")
        }
        _ if status.complete => format!("{level} full"),
        _ if status.trickle => format!("{level} trickle charging"),
        _ if status.constant_current || status.constant_voltage => {
            format!("{level} {}", charging_as(speed))
        }
        _ if status.recharge => format!("{level} charging"),
        _ if status.paused_die_temp => format!("{level} charging paused (hot)"),
        // No phase at all: idle on a source, and nothing to say off one.
        _ if reading.usb == Some(true) => format!("{level} not charging"),
        _ => level,
    }
}

fn level_as(level: Option<u8>) -> String {
    match level {
        Some(level) => format!("{level}%"),
        None => "unknown".to_string(),
    }
}

fn charging_as(speed: Option<&str>) -> String {
    match speed {
        Some(speed) => format!("{speed} charging"),
        None => "charging".to_string(),
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
        println!("--force is ignored for a keyboard image.");
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
        println!("--device is ignored over the cable.");
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

    flash_over_ble(data, named)
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
    zsab::update(path, force, &link)
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
        bail!("The keyboard is not running an Oryx firmware.");
    };

    if layout_id.is_empty() || revision_id.is_empty() {
        bail!("The keyboard is not running an Oryx firmware.");
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
        Half::Right => "Could not reach the right half over Bluetooth".to_string(),
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
            "The image is {} bytes; the STM32's application region holds {} bytes.",
            image.len(),
            dfu::MAX_IMAGE
        );
    }

    log::debug!("image {} bytes, crc-32 {:08x}", image.len(), dfu::crc32(image));

    let mut client = dfu_connect(device)?;

    let spinner = new_spinner("Waiting for keyboard in bootloader mode...");
    let ready = client.make_ready(BLE_BOOTLOADER_WAIT, &report_wait);
    spinner.finish_and_clear();
    ready.context("The STM32 is not in its bootloader")?;

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
    use zapp_core::hid::{HalfDetail, Source, VbusInStatus};

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
            ble: None,
        };
        // No detail read, so the summary's own charging flag words it. The
        // board named no Bluetooth identity, so nothing is said about one.
        assert_eq!(
            describe_telemetry(&telemetry, None),
            vec![
                "battery  · left 87% · right 64% fast charging",
                "features · Trackball (connected), Trackpad (not supported)",
            ]
        );
    }

    #[test]
    fn a_wired_board_says_which_bluetooth_identity_it_is_on() {
        let telemetry = Telemetry {
            version: 1,
            ble: Some(BleIdentity {
                address: "FE:DB:E9:A8:B1:36".into(),
                random: true,
                slot: 2,
                slot_count: 6,
            }),
            ..Telemetry::default()
        };
        // Between the battery and the features, and the slot counted from one.
        assert_eq!(
            describe_telemetry(&telemetry, None)[1],
            "bluetooth · FE:DB:E9:A8:B1:36 (host 3 of 6)"
        );
    }

    #[test]
    fn a_board_that_did_not_say_how_many_hosts_it_keeps_gives_the_address_alone() {
        let telemetry = Telemetry {
            version: 1,
            ble: Some(BleIdentity {
                address: "FE:DB:E9:A8:B1:36".into(),
                random: false,
                slot: 0,
                slot_count: 0,
            }),
            ..Telemetry::default()
        };
        assert_eq!(
            describe_telemetry(&telemetry, None)[1],
            "bluetooth · FE:DB:E9:A8:B1:36"
        );
    }

    #[test]
    fn a_detail_read_words_the_summary_line() {
        // The flag says both halves are charging; the phase says one of them
        // finished and the other is only trickling.
        let telemetry = Telemetry {
            version: 1,
            left: charging_reading(Some(57), Source::Nrf),
            right: charging_reading(Some(99), Source::Wired),
            charging: Charging {
                budget_ma: Some(1500),
                ..Charging::default()
            },
            ..Telemetry::default()
        };
        let detail = ChargeTelemetry {
            version: 1,
            left: half_in(0x01 | 0x04),
            right: half_in(0x01 | 0x02),
            ..ChargeTelemetry::default()
        };

        assert_eq!(
            describe_telemetry(&telemetry, Some(&detail))[0],
            "battery  · left 57% trickle charging · right 99% full"
        );
    }

    /// One half whose charger reads `raw`, on USB.
    fn half_in(raw: u8) -> HalfDetail {
        HalfDetail {
            source: Some(Source::Nrf),
            detail: BatteryDetail {
                charger: Some(ChargerStatus::from_bits(raw)),
                ..BatteryDetail::default()
            },
        }
    }

    /// One half on USB, read at `level`.
    fn on_usb(level: u8) -> BatteryReading {
        BatteryReading {
            usb: Some(true),
            ..charging_reading(Some(level), Source::Nrf)
        }
    }

    #[test]
    fn the_phase_words_each_half_of_the_summary() {
        let said = |raw: u8, speed| read_in_phase(on_usb(57), ChargerStatus::from_bits(raw), speed);

        assert_eq!(said(0x01 | 0x02, Some("fast")), "57% full");
        assert_eq!(said(0x01 | 0x08, Some("fast")), "57% fast charging");
        assert_eq!(said(0x01 | 0x10, Some("slow")), "57% slow charging");
        assert_eq!(said(0x01 | 0x08, None), "57% charging");
        // A trickle is never fast, whatever the source offered.
        assert_eq!(said(0x01 | 0x04, Some("fast")), "57% trickle charging");
        assert_eq!(said(0x01 | 0x20, Some("fast")), "57% charging");
        assert_eq!(said(0x01 | 0x40, Some("fast")), "57% charging paused (hot)");
        assert_eq!(said(0x00, Some("fast")), "57% no battery detected");
        assert_eq!(said(0x01, Some("fast")), "57% not charging");
    }

    #[test]
    fn a_half_off_the_cable_is_only_a_level() {
        // Nothing is filling it, so no charger word belongs on the line, and
        // the no-cell bit means nothing off a source.
        let idle = ChargerStatus::from_bits(0x00);
        for usb in [Some(false), None] {
            let reading = BatteryReading { usb, ..on_usb(57) };
            assert_eq!(read_in_phase(reading, idle, Some("fast")), "57%");
        }
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
    fn a_board_that_is_not_a_voyager_2_is_never_asked_for_telemetry() {
        // The original Voyager has no battery and no nav header at all, so
        // there is nothing to report and nothing worth asking it for.
        let device = UsbDevice {
            keyboard: Some(ids::Keyboard::Voyager),
            bootloader: None,
            serial: None,
            vid: 0,
            pid: 0,
        };
        assert_eq!(wired_report(&device, true), Vec::<String>::new());
    }

    /// One half filling on the cable, with nothing latched.
    fn filling(voltage_mv: u16, current_ma: i16, temp_cdeg: i16, ttf_min: u16) -> BatteryDetail {
        BatteryDetail {
            voltage_mv: Some(voltage_mv),
            current_ma: Some(current_ma),
            temp_cdeg: Some(temp_cdeg),
            tte_min: None,
            ttf_min: Some(ttf_min),
            charger: Some(ChargerStatus::from_bits(0x01 | 0x08)),
            fault: Some(ChargerFault::default()),
        }
    }

    #[test]
    fn a_wired_board_says_what_each_half_is_filling_at() {
        let telemetry = ChargeTelemetry {
            version: 1,
            left: HalfDetail {
                source: Some(Source::Nrf),
                detail: filling(4020, 512, 2850, 80),
            },
            right: HalfDetail {
                source: Some(Source::Wired),
                detail: filling(3980, 298, 2790, 125),
            },
        };
        let summary = Telemetry {
            charging: Charging {
                budget_ma: Some(1500),
                input_limit_ma: Some(700),
                charge_current_ma: Some(550),
                state: None,
            },
            ..Telemetry::default()
        };

        assert_eq!(
            describe_charge(&summary, &telemetry, None),
            vec![
                "charging · source 1500 mA · limit 700 mA · charge 550 mA",
                "left     · 4.02 V · +512 mA · 28.5 °C · full in 1 h 20 min · \
                 constant current · no fault",
                "right    · 3.98 V · +298 mA · 27.9 °C · full in 2 h 05 min · \
                 constant current · no fault",
            ]
        );
    }

    /// Both halves filling on the cable, as the two per-half reads see them.
    fn wired_pair() -> (Telemetry, ChargeTelemetry) {
        let telemetry = ChargeTelemetry {
            version: 1,
            left: HalfDetail {
                source: Some(Source::Nrf),
                detail: filling(4020, 512, 2850, 80),
            },
            right: HalfDetail {
                source: Some(Source::Wired),
                detail: filling(3980, 298, 2790, 125),
            },
        };
        let summary = Telemetry {
            charging: Charging {
                budget_ma: Some(1500),
                ..Charging::default()
            },
            ..Telemetry::default()
        };
        (summary, telemetry)
    }

    #[test]
    fn a_wired_board_puts_each_half_s_rails_under_it() {
        let (summary, telemetry) = wired_pair();
        let report = PmicRailsReport {
            version: 1,
            left: Some(PmicRails {
                vbus_mv: 4620,
                vsys_mv: 4550,
                vbusin: VbusInStatus::from_bits(0x21),
                charger_enabled: Some(true),
                boost_off: None,
                charge_reduced: false,
            }),
            right: Some(PmicRails {
                vbus_mv: 4580,
                vsys_mv: 4510,
                vbusin: VbusInStatus::from_bits(0x01),
                charger_enabled: Some(true),
                boost_off: None,
                charge_reduced: false,
            }),
        };

        assert_eq!(
            describe_charge(&summary, &telemetry, Some(report)),
            vec![
                "charging · source 1500 mA",
                "left     · 4.02 V · +512 mA · 28.5 °C · full in 1 h 20 min · \
                 constant current · no fault",
                "rails    · VBUS 4.62 V · VSYS 4.55 V · in: present, out active · charger enabled",
                "right    · 3.98 V · +298 mA · 27.9 °C · full in 2 h 05 min · \
                 constant current · no fault",
                "rails    · VBUS 4.58 V · VSYS 4.51 V · in: present · charger enabled",
            ]
        );
    }

    #[test]
    fn a_half_with_no_rails_keeps_its_detail_line() {
        // An older firmware answers the rails command with nothing for a half,
        // and that half's line stays exactly as it was.
        let (summary, telemetry) = wired_pair();
        let report = PmicRailsReport {
            version: 1,
            left: None,
            right: Some(PmicRails {
                vbus_mv: 4580,
                vsys_mv: 4510,
                vbusin: VbusInStatus::from_bits(0x01),
                charger_enabled: Some(true),
                boost_off: None,
                charge_reduced: false,
            }),
        };

        assert_eq!(
            describe_charge(&summary, &telemetry, Some(report)),
            vec![
                "charging · source 1500 mA",
                "left     · 4.02 V · +512 mA · 28.5 °C · full in 1 h 20 min · \
                 constant current · no fault",
                "right    · 3.98 V · +298 mA · 27.9 °C · full in 2 h 05 min · \
                 constant current · no fault",
                "rails    · VBUS 4.58 V · VSYS 4.51 V · in: present · charger enabled",
            ]
        );

        // A board that never answered the command at all is the same silence.
        assert_eq!(describe_charge(&summary, &telemetry, None).len(), 3);
    }

    #[test]
    fn a_half_on_the_radio_says_how_long_it_has_left() {
        let detail = BatteryDetail {
            voltage_mv: Some(4020),
            current_ma: Some(-45),
            temp_cdeg: Some(2850),
            tte_min: Some(750),
            ttf_min: None,
            charger: Some(ChargerStatus::from_bits(0x01)),
            fault: Some(ChargerFault::default()),
        };
        assert_eq!(
            detail_line("left", &detail, Estimate::Empty, None),
            "left     · 4.02 V · -45 mA · 28.5 °C · empty in 12 h 30 min · \
             charger idle · no fault"
        );
    }

    #[test]
    fn a_half_on_the_cable_never_says_how_long_until_empty() {
        // Plugged in, so time-to-empty means nothing even when the half has it.
        let mut detail = filling(4020, -45, 2850, 80);
        detail.ttf_min = None;
        detail.tte_min = Some(45);
        assert_eq!(time_left(&detail, Estimate::Full), "unknown");

        // On the radio it is the answer, and time-to-full is the fallback.
        assert_eq!(time_left(&detail, Estimate::Empty), "empty in 45 min");
        detail.tte_min = None;
        detail.ttf_min = Some(80);
        assert_eq!(time_left(&detail, Estimate::Empty), "full in 1 h 20 min");
    }

    #[test]
    fn a_half_with_no_cell_says_so_only_on_a_source() {
        // The bit reads clear on battery power with a healthy cell, so it is
        // only worth saying when the half is known to have USB.
        let no_phase = ChargerStatus::from_bits(0x00);
        assert_eq!(phase(no_phase, Some(true)), "no battery detected");
        assert_eq!(phase(no_phase, None), "charger idle");
        assert_eq!(phase(no_phase, Some(false)), "charger idle");

        // Off a source the phase bits are read as they stand.
        let charging = ChargerStatus::from_bits(0x08);
        assert_eq!(phase(charging, Some(true)), "no battery detected");
        assert_eq!(phase(charging, None), "constant current");
        assert_eq!(phase(charging, Some(false)), "constant current");
    }

    #[test]
    fn a_reading_the_board_did_not_take_keeps_its_column() {
        // Every piece unknown, so the columns still line up under a half that
        // answered — and a flat cell is never invented from a missing one.
        assert_eq!(
            detail_line("right", &BatteryDetail::default(), Estimate::Full, None),
            "right    · unknown · unknown · unknown · unknown · unknown · unknown"
        );
    }

    #[test]
    fn hours_only_appear_once_there_is_one() {
        assert_eq!(duration(0), "0 min");
        assert_eq!(duration(59), "59 min");
        assert_eq!(duration(60), "1 h 00 min");
        assert_eq!(duration(125), "2 h 05 min");
        assert_eq!(duration(1439), "23 h 59 min");
    }

    #[test]
    fn the_charger_phase_is_named() {
        let phases = [
            (0x02, "charge complete"),
            (0x04, "trickle charging"),
            (0x08, "constant current"),
            (0x10, "constant voltage"),
            (0x20, "recharging"),
            (0x40, "paused (die temperature)"),
            (0x00, "charger idle"),
            (0x80, "charger idle"), // supplementing is not a phase
        ];
        for (raw, expected) in phases {
            assert_eq!(
                phase(ChargerStatus::from_bits(raw | 0x01), Some(true)),
                expected
            );
        }
    }

    #[test]
    fn a_latched_fault_says_what_it_was() {
        assert_eq!(fault(ChargerFault::default()), "no fault");
        assert_eq!(
            fault(ChargerFault {
                reason: 0x40,
                sensor: 0x00
            }),
            "fault: trickle timeout"
        );
        assert_eq!(
            fault(ChargerFault {
                reason: 0x20,
                sensor: 0x08
            }),
            "fault: charge timeout (NTC hot)"
        );
        // A bit with no name is still a fault, and says which one it was.
        assert_eq!(
            fault(ChargerFault {
                reason: 0x80,
                sensor: 0x00
            }),
            "fault: 0x80"
        );
    }

    #[test]
    fn a_figure_the_board_would_not_give_is_left_out() {
        let charging = Charging {
            budget_ma: Some(1500),
            input_limit_ma: None,
            charge_current_ma: Some(300),
            state: None,
        };
        assert_eq!(
            charge_rates(&charging).as_deref(),
            Some("source 1500 mA · charge 300 mA")
        );
        // A board that gave none of the three has no line at all.
        assert_eq!(charge_rates(&Charging::default()), None);
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

    #[test]
    fn a_half_on_the_radio_says_what_its_rails_measure() {
        let rails = PmicRails {
            vbus_mv: 4620,
            vsys_mv: 4550,
            vbusin: VbusInStatus::from_bits(0x21),
            charger_enabled: Some(true),
            boost_off: None,
            charge_reduced: false,
        };
        assert_eq!(
            rails_line(rails),
            "rails    · VBUS 4.62 V · VSYS 4.55 V · in: present, out active · charger enabled"
        );
    }

    #[test]
    fn rails_with_nothing_attached_say_so_rather_than_stopping_short() {
        let rails = PmicRails {
            vbus_mv: 0,
            vsys_mv: 3980,
            vbusin: VbusInStatus::from_bits(0),
            charger_enabled: Some(false),
            boost_off: None,
            charge_reduced: false,
        };
        assert_eq!(
            rails_line(rails),
            "rails    · VBUS 0.00 V · VSYS 3.98 V · in: none · charger disabled"
        );
    }

    #[test]
    fn a_charger_register_that_was_not_read_is_not_a_charger_that_is_off() {
        let rails = PmicRails {
            vbus_mv: 5040,
            vsys_mv: 4600,
            vbusin: VbusInStatus::from_bits(0x12),
            charger_enabled: None,
            boost_off: None,
            charge_reduced: false,
        };
        assert_eq!(
            rails_line(rails),
            "rails    · VBUS 5.04 V · VSYS 4.60 V · in: current limit, suspended · charger unknown"
        );
    }

    #[test]
    fn a_half_that_reports_its_boost_says_which_way_it_is() {
        let base = PmicRails {
            vbus_mv: 4620,
            vsys_mv: 4550,
            vbusin: VbusInStatus::from_bits(0x21),
            charger_enabled: Some(true),
            boost_off: None,
            charge_reduced: false,
        };

        let on = PmicRails {
            boost_off: Some(false),
            ..base
        };
        assert_eq!(
            rails_line(on),
            "rails    · VBUS 4.62 V · VSYS 4.55 V · in: present, out active · \
             charger enabled · boost on"
        );

        let off = PmicRails {
            boost_off: Some(true),
            ..base
        };
        assert_eq!(
            rails_line(off),
            "rails    · VBUS 4.62 V · VSYS 4.55 V · in: present, out active · \
             charger enabled · boost off"
        );

        // A half that does not report it says nothing rather than guessing.
        assert!(!rails_line(base).contains("boost"));
    }

    #[test]
    fn a_half_holding_its_charge_current_back_says_so() {
        let base = PmicRails {
            vbus_mv: 4620,
            vsys_mv: 4550,
            vbusin: VbusInStatus::from_bits(0x21),
            charger_enabled: Some(true),
            boost_off: None,
            charge_reduced: false,
        };

        assert_eq!(
            rails_line(PmicRails {
                charge_reduced: true,
                ..base
            }),
            "rails    · VBUS 4.62 V · VSYS 4.55 V · in: present, out active · \
             charger enabled · charge reduced"
        );

        // A half charging at the rate it was told leaves the line as it was.
        assert!(!rails_line(base).contains("charge reduced"));
    }
}
