mod confirm;
mod flash_log;
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
use zapp_ble::{Battery, BleMonitor, ChargeDetail, Level, Presence};
use zapp_core::device::ids::{self as ids, is_moonlander_revb, target_name_for_pid};
use zapp_core::device::{self, UsbDevice, WatchStatus};
use zapp_core::firmware::{self, Firmware};
use zapp_core::flash::{self, FlashProgress};
use zapp_core::hid::{
    BatteryDetail, BatteryReading, BleIdentity, CMD_GET_CHARGE_TELEMETRY, CMD_GET_PMIC_RAILS,
    CMD_GET_TELEMETRY, ChargeTelemetry, ChargerFault, ChargerStatus, HidConnection, HidEvent, Nav,
    PmicRails, PmicRailsReport, Telemetry,
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
    let cli = Cli::parse();
    flash_log::init(matches!(cli.command, Commands::Flash { .. } | Commands::Update));

    let result = dispatch(cli.command);
    flash_log::finish(&result);
    if let Err(err) = result {
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

    // The Bluetooth identities the boards on the cable named, which the roster
    // below matches its bonded entries against.
    let mut on_usb: Vec<String> = Vec::new();

    println!("USB");
    match &wired {
        Ok(devices) if devices.is_empty() => println!("  none"),
        Ok(devices) => {
            for device in devices {
                let report = wired_report(device, verbose);
                println!("  {}{}", describe_usb(device), report.charge);
                for line in &report.lines {
                    println!("    {line}");
                }
                on_usb.extend(report.ble_address);
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
        // Read once: the same detail words the summary line and fills the
        // lines under it.
        let detail = ble_charge_detail(&board, verbose);
        println!(
            "  {name} · {} · {known_as:?}{address}{}",
            state(board.presence, board.address.as_deref(), &on_usb),
            charge(board.battery, detail.as_ref()),
        );
        for line in ble_report(detail.as_ref()) {
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
fn wired_report(device: &UsbDevice, verbose: bool) -> WiredReport {
    if device.keyboard != Some(ids::Keyboard::Voyager2) {
        return WiredReport::default();
    }
    let Some((telemetry, detail, rails)) = usb_telemetry(device, verbose) else {
        return WiredReport::default();
    };

    let mut lines = Vec::new();
    if verbose {
        lines.extend(describe_telemetry(&telemetry));
        if let Some(detail) = &detail {
            lines.extend(describe_charge(&telemetry, detail, rails));
        }
    }
    WiredReport {
        charge: wired_charge(&telemetry, detail.as_ref()),
        lines,
        ble_address: telemetry.ble.map(|ble| ble.address),
    }
}

/// What a wired board answered: what its halves have left, which goes on the
/// board's own line; the lines to print under it, which only `--verbose` asks
/// for; and the Bluetooth address it is using, which the roster matches its
/// bonded entries against.
#[derive(Debug, Default, PartialEq, Eq)]
struct WiredReport {
    charge: String,
    lines: Vec<String>,
    ble_address: Option<String>,
}

/// What the board answers, on one connection: the summary always, and the
/// detail behind it only when it was asked for. A board whose firmware predates
/// any of these commands never answers it, which is why each wait can elapse.
type UsbTelemetry = (Telemetry, Option<ChargeTelemetry>, Option<PmicRailsReport>);

fn usb_telemetry(device: &UsbDevice, rails: bool) -> Option<UsbTelemetry> {
    if device.bootloader.is_some() {
        return None; // a board in its bootloader runs no keyboard firmware
    }

    let conn = HidConnection::open(device.vid, device.pid).ok()?;
    let telemetry = ask(&conn, CMD_GET_TELEMETRY, |event| match event {
        HidEvent::Telemetry(telemetry) => Some(telemetry),
        _ => None,
    })?;
    // Asked for every listing, verbose or not: the charger phase words the
    // board's own line, and the two wordings must not differ.
    let charge = ask(&conn, CMD_GET_CHARGE_TELEMETRY, |event| match event {
        HidEvent::ChargeTelemetry(telemetry) => Some(telemetry),
        _ => None,
    });
    let rails = rails
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

/// What each half of a wired board has left, in the shape and the wording the
/// Bluetooth roster line uses, ready to append to the board's own line.
///
/// The charger phase from the detail read words each half when it arrived, so
/// this and the table under it say the same thing; the summary's charging flag
/// words it only for firmware that answers no detail at all.
fn wired_charge(telemetry: &Telemetry, detail: Option<&ChargeTelemetry>) -> String {
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

    format!(" · {}", halves.join(" · "))
}

/// The lines under a wired board that stand on their own: the Bluetooth
/// identity it is using, and which pointing devices the build carries and
/// which of them answer. Only `--verbose` asks for these.
fn describe_telemetry(telemetry: &Telemetry) -> Vec<String> {
    let mut lines = Vec::new();
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

/// The address the board is using. The slot fields come off the wire with it
/// and are not printed.
fn ble_identity(ble: &BleIdentity) -> String {
    ble.address.clone()
}

/// What a Bluetooth board's charge-detail characteristic says, when the detail
/// was asked for.
///
/// Only a connected board can be read at all, and only one whose firmware
/// serves the characteristic answers; anything else says nothing, which is the
/// same silence an old wired firmware gives.
fn ble_charge_detail(board: &zapp_ble::BleBoard, verbose: bool) -> Option<ChargeDetail> {
    if !verbose || board.presence != Presence::Connected {
        return None;
    }
    zapp_ble::read_charge_detail(&board.key)
}

/// The lines under a Bluetooth board, out of the detail that was read.
fn ble_report(detail: Option<&ChargeDetail>) -> Vec<String> {
    let Some(detail) = detail else {
        return Vec::new();
    };

    // A board on the radio is a board off the cable, so what it is heading for
    // is empty. The radio read carries no VBUS flag either.
    // The radio read carries no VBUS flag, so the halves' own rails and charger
    // say which way the board is going.
    let going = heading_for([
        (&detail.left, detail.left_rails),
        (&detail.right, detail.right_rails),
    ]);
    table(&[
        detail_header(going),
        detail_row("left", &detail.left, going, None, detail.left_rails),
        detail_row("right", &detail.right, going, None, detail.right_rails),
    ])
}

/// The lines under a wired board once its charge telemetry has been read: what
/// each half's cell and charger are doing.
///
/// The summary read comes along for each half's VBUS flag, which the detail
/// block does not carry and the charger phase needs.
fn describe_charge(
    summary: &Telemetry,
    telemetry: &ChargeTelemetry,
    rails: Option<PmicRailsReport>,
) -> Vec<String> {
    let left_rails = rails.and_then(|rails| rails.left);
    let right_rails = rails.and_then(|rails| rails.right);
    let going = heading_for([
        (&telemetry.left.detail, left_rails),
        (&telemetry.right.detail, right_rails),
    ]);
    table(&[
        detail_header(going),
        detail_row(
            "left",
            &telemetry.left.detail,
            going,
            summary.left.usb,
            left_rails,
        ),
        detail_row(
            "right",
            &telemetry.right.detail,
            going,
            summary.right.usb,
            right_rails,
        ),
    ])
}

/// Which way the board is going, and so what its time column counts down to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Estimate {
    Full,
    Empty,
}

impl Estimate {
    fn heading(self) -> &'static str {
        match self {
            Self::Full => "full in",
            Self::Empty => "empty in",
        }
    }

    /// What one half has left the way the board is going, or nothing when the
    /// gauge did not say or the half is not going that way. A charge that has
    /// terminated is not filling, whatever estimate the gauge still carries.
    fn minutes(self, detail: &BatteryDetail) -> Option<u16> {
        match self {
            Self::Full => detail
                .ttf_min
                .filter(|_| !detail.charger.is_some_and(|status| status.complete)),
            Self::Empty => detail.tte_min,
        }
    }
}

/// Which way a board is going: filling while anything says it is on a source,
/// running down otherwise. Read off both halves together, so the time column is
/// headed once for the board rather than once per half.
fn heading_for(halves: [(&BatteryDetail, Option<PmicRails>); 2]) -> Estimate {
    let on_source = halves.iter().any(|(detail, rails)| {
        rails.is_some_and(|rails| rails.vbusin.present)
            || detail.charger.is_some_and(charging_phase)
    });
    match on_source {
        true => Estimate::Full,
        false => Estimate::Empty,
    }
}

/// Whether the charger names a phase at all, which it only does on a source.
fn charging_phase(status: ChargerStatus) -> bool {
    status.complete
        || status.trickle
        || status.constant_current
        || status.constant_voltage
        || status.recharge
        || status.paused_die_temp
}

/// The headings the halves' rows sit under. The half's own name has none.
fn detail_header(going: Estimate) -> Vec<String> {
    [
        "",
        "cell",
        "current",
        "temp",
        "VBUS",
        "VSYS",
        going.heading(),
        "charger",
        "fault",
    ]
    .iter()
    .map(|heading| (*heading).to_string())
    .collect()
}

/// One half's row: its cell, its rails, how long it has, what the charger is
/// doing and whether anything latched.
///
/// A column the board would not fill in says "n/a" rather than dropping out, so
/// the rows stay under their headings. The input rail is "n/a" for a half with
/// nothing attached, since what it reads then is not an input.
fn detail_row(
    half: &str,
    detail: &BatteryDetail,
    going: Estimate,
    usb: Option<bool>,
    rails: Option<PmicRails>,
) -> Vec<String> {
    let vbus = rails
        .filter(|rails| rails.vbusin.present)
        .map(|rails| volts(rails.vbus_mv));

    vec![
        half.to_string(),
        detail.voltage_mv.map_or_else(not_available, volts),
        detail.current_ma.map_or_else(not_available, milliamps),
        detail.temp_cdeg.map_or_else(not_available, degrees),
        vbus.unwrap_or_else(not_available),
        rails.map_or_else(not_available, |rails| volts(rails.vsys_mv)),
        going.minutes(detail).map_or_else(not_available, duration),
        detail
            .charger
            .map_or_else(not_available, |status| phase(status, usb)),
        detail.fault.map_or_else(not_available, fault),
    ]
}

/// Rows laid out in columns: each as wide as the widest thing in it, two spaces
/// between, left-aligned, and nothing trailing the last one.
fn table(rows: &[Vec<String>]) -> Vec<String> {
    let columns = rows.iter().map(Vec::len).max().unwrap_or(0);
    let widths: Vec<usize> = (0..columns)
        .map(|column| {
            rows.iter()
                .filter_map(|row| row.get(column))
                .map(|cell| cell.chars().count())
                .max()
                .unwrap_or(0)
        })
        .collect();

    rows.iter()
        .map(|row| {
            let mut line = String::new();
            for (column, cell) in row.iter().enumerate() {
                line.push_str(cell);
                if column + 1 < row.len() {
                    let pad = widths[column] - cell.chars().count();
                    line.push_str(&" ".repeat(pad + 2));
                }
            }
            line
        })
        .collect()
}

fn not_available() -> String {
    "n/a".to_string()
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

/// Hours and minutes, or minutes on their own under an hour. An estimate that
/// has run down to nothing is under a minute rather than none.
fn duration(minutes: u16) -> String {
    match (minutes / 60, minutes % 60) {
        (0, 0) => "under a minute".to_string(),
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
        return "none".to_string();
    }

    let reasons = fault.reasons();
    let named = match reasons.is_empty() {
        // A bit this host has no name for: firmware, or hardware, newer than it.
        true => format!("0x{:02x}", fault.reason),
        false => reasons.join(", "),
    };
    let sensors = fault.sensors();
    match sensors.is_empty() {
        true => named,
        false => format!("{named} ({})", sensors.join(", ")),
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
/// detail line names.
fn read_in_phase(reading: BatteryReading, status: ChargerStatus, speed: Option<&str>) -> String {
    let level = level_as(reading.level);
    match phase_as(status, reading.usb, speed) {
        Some(said) => format!("{level} {said}"),
        None => level,
    }
}

/// A summary line's word for one charger phase, or `None` when there is nothing
/// to say: no phase at all, off a source.
///
/// The bits are read in the order `phase` reads them, so the summary and the
/// detail line never disagree. Only constant current is a speed: constant
/// voltage is the tail of the charge, whatever the source offered.
fn phase_as(status: ChargerStatus, usb: Option<bool>, speed: Option<&str>) -> Option<String> {
    let said = match status {
        _ if usb == Some(true) && !status.battery_detected => "no battery detected".to_string(),
        _ if status.complete => "full".to_string(),
        _ if status.trickle => "trickle charging".to_string(),
        _ if status.constant_current => charging_as(speed),
        _ if status.constant_voltage => "topping up".to_string(),
        _ if status.recharge => "charging".to_string(),
        _ if status.paused_die_temp => "charging paused (hot)".to_string(),
        // No phase at all: idle on a source, and nothing to say off one.
        _ if usb == Some(true) => "not charging".to_string(),
        _ => return None,
    };
    Some(said)
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

/// What the halves have left, and what each half's charger is doing when the
/// detail was read.
///
/// A half the board does not publish is left out; one it publishes without a
/// level is named and called unknown, because a half that is there and quiet is
/// worth saying and a half that is not there is not.
///
/// The phase words this line the same way the wired summary is worded, off the
/// same bits, so the two paths say the same thing for the same state. The radio
/// read carries no VBUS flag and no source budget, so a half with no phase is
/// only a level and a charge is never called fast or slow here.
fn charge(battery: Battery, detail: Option<&ChargeDetail>) -> String {
    let halves: Vec<String> = [
        ("left", battery.left, detail.map(|detail| detail.left)),
        ("right", battery.right, detail.map(|detail| detail.right)),
    ]
    .into_iter()
    .filter(|(_, level, _)| !level.is_absent())
    .map(|(half, level, detail)| {
        let phase = detail
            .and_then(|detail| detail.charger)
            .and_then(|status| phase_as(status, None, None));
        match phase {
            Some(phase) => format!("{half} {} {phase}", said(level)),
            None => format!("{half} {}", said(level)),
        }
    })
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

/// How reachable a bonded board is. A bonded entry whose address is one a board
/// on the cable named is that board, so it is not out of range.
fn state(presence: Presence, address: Option<&str>, on_usb: &[String]) -> &'static str {
    match presence {
        Presence::Connected => "connected",
        Presence::Nearby => "in range",
        Presence::Paired => {
            match address.is_some_and(|address| on_usb.iter().any(|wired| wired == address)) {
                true => "on USB, Bluetooth off",
                false => "paired, not in range",
            }
        }
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
    use zapp_core::hid::{Charging, HalfDetail, Source, VbusInStatus};

    #[test]
    fn both_halves_of_a_split_board_are_named() {
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        assert_eq!(charge(battery, None), " · left 0% · right 27%");
    }

    #[test]
    fn a_half_that_is_there_and_would_not_say_is_called_unknown() {
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Unknown,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery, None), " · left 36% · right unknown");
    }

    #[test]
    fn a_half_the_board_does_not_have_is_left_out() {
        // Not the same as one that would not say: naming it would invent a half.
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Absent,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery, None), " · left 36%");
    }

    #[test]
    fn a_board_with_one_battery_reports_the_whole() {
        let battery = Battery {
            keyboard: Level::Percent(80),
            ..Battery::default()
        };
        assert_eq!(charge(battery, None), " · 80%");
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
        // Firmware that answers no detail at all, so the summary's own
        // charging flag words the board's line.
        assert_eq!(
            wired_charge(&telemetry, None),
            " · left 87% · right 64% fast charging"
        );
        // The board named no Bluetooth identity, so nothing is said about one.
        assert_eq!(
            describe_telemetry(&telemetry),
            vec!["features · Trackball (connected), Trackpad (not supported)"]
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
        // Above the features, and the address alone: the slot the board named
        // is not printed.
        assert_eq!(
            describe_telemetry(&telemetry)[0],
            "bluetooth · FE:DB:E9:A8:B1:36"
        );
    }

    #[test]
    fn a_board_that_named_no_slot_gives_the_same_line() {
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
            describe_telemetry(&telemetry)[0],
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
            wired_charge(&telemetry, Some(&detail)),
            " · left 57% trickle charging · right 99% full"
        );
    }

    #[test]
    fn a_half_holding_at_voltage_is_topping_up_rather_than_charging_fast() {
        let telemetry = Telemetry {
            version: 1,
            left: charging_reading(Some(99), Source::Nrf),
            right: charging_reading(Some(56), Source::Wired),
            charging: Charging {
                budget_ma: Some(1500),
                ..Charging::default()
            },
            ..Telemetry::default()
        };
        let detail = ChargeTelemetry {
            version: 1,
            left: half_in(0x01 | 0x10),
            right: half_in(0x01 | 0x08),
        };

        assert_eq!(
            wired_charge(&telemetry, Some(&detail)),
            " · left 99% topping up · right 56% fast charging"
        );
    }

    /// One half's charge detail, with its charger reading `raw`.
    fn ble_half(raw: u8) -> BatteryDetail {
        BatteryDetail {
            charger: Some(ChargerStatus::from_bits(raw)),
            ..BatteryDetail::default()
        }
    }

    #[test]
    fn a_bluetooth_summary_says_what_the_detail_read_said() {
        let battery = Battery {
            left: Level::Percent(99),
            right: Level::Percent(54),
            keyboard: Level::Percent(54),
        };
        let detail = ChargeDetail {
            left: ble_half(0x01 | 0x02),
            right: ble_half(0x01 | 0x08),
            ..ChargeDetail::default()
        };
        // The same words the cable gives for the same bits. The radio carries
        // no source budget, so a charge is neither fast nor slow here.
        assert_eq!(
            charge(battery, Some(&detail)),
            " · left 99% full · right 54% charging"
        );

        let topping = ChargeDetail {
            left: ble_half(0x01 | 0x10),
            right: ble_half(0x01 | 0x04),
            ..ChargeDetail::default()
        };
        assert_eq!(
            charge(battery, Some(&topping)),
            " · left 99% topping up · right 54% trickle charging"
        );
    }

    #[test]
    fn a_bluetooth_half_on_battery_is_only_a_level() {
        let battery = Battery {
            left: Level::Percent(99),
            right: Level::Percent(54),
            keyboard: Level::Percent(54),
        };
        // No phase and no VBUS flag on the radio: nothing to say about a
        // charger, and a half the read did not reach is left as it was.
        let detail = ChargeDetail {
            left: ble_half(0x01),
            right: BatteryDetail::default(),
            ..ChargeDetail::default()
        };
        assert_eq!(charge(battery, Some(&detail)), " · left 99% · right 54%");

        // And with no detail read at all the line is the level alone.
        assert_eq!(charge(battery, None), " · left 99% · right 54%");
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
        assert_eq!(said(0x01 | 0x08, Some("slow")), "57% slow charging");
        assert_eq!(said(0x01 | 0x08, None), "57% charging");
        // The tail of the charge is never fast, whatever the source offered.
        assert_eq!(said(0x01 | 0x10, Some("fast")), "57% topping up");
        assert_eq!(said(0x01 | 0x10, None), "57% topping up");
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
    fn a_bonded_entry_that_is_the_board_on_the_cable_is_not_out_of_range() {
        let on_usb = vec!["C6:14:15:90:63:EC".to_string()];

        assert_eq!(
            state(Presence::Paired, Some("C6:14:15:90:63:EC"), &on_usb),
            "on USB, Bluetooth off"
        );
        // Another bonded board, and one whose handle spells out no address.
        assert_eq!(
            state(Presence::Paired, Some("FE:DB:E9:A8:B1:36"), &on_usb),
            "paired, not in range"
        );
        assert_eq!(
            state(Presence::Paired, None, &on_usb),
            "paired, not in range"
        );
        // Nothing on the cable named an identity.
        assert_eq!(
            state(Presence::Paired, Some("C6:14:15:90:63:EC"), &[]),
            "paired, not in range"
        );
    }

    #[test]
    fn a_reachable_board_is_worded_the_same_whatever_is_on_the_cable() {
        let on_usb = vec!["C6:14:15:90:63:EC".to_string()];
        for presence in [Presence::Connected, Presence::Nearby] {
            assert_eq!(
                state(presence, Some("C6:14:15:90:63:EC"), &on_usb),
                state(presence, Some("C6:14:15:90:63:EC"), &[])
            );
        }
        assert_eq!(state(Presence::Connected, None, &on_usb), "connected");
        assert_eq!(state(Presence::Nearby, None, &on_usb), "in range");
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
        assert_eq!(wired_report(&device, true), WiredReport::default());
    }

    #[test]
    fn a_board_with_nothing_to_report_keeps_its_single_line() {
        // Nothing to append and nothing to print under it, so the board keeps
        // the one line `describe_usb` gives it, verbose or not.
        let report = WiredReport::default();
        assert_eq!(report.charge, "");
        assert!(report.lines.is_empty());
        assert_eq!(report.ble_address, None);
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
    fn a_wired_board_lays_the_two_halves_out_under_headings() {
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

        // The rates the summary carries are decoded and not printed. No rails
        // were read, so both rail columns are n/a for want of a measurement.
        assert_eq!(
            describe_charge(&summary, &telemetry, None),
            vec![
                "       cell    current  temp     VBUS  VSYS  full in     charger           fault",
                "left   4.02 V  +512 mA  28.5 °C  n/a   n/a   1 h 20 min  constant current  none",
                "right  3.98 V  +298 mA  27.9 °C  n/a   n/a   2 h 05 min  constant current  none",
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

    /// One half's rails on a cable. The flags come off the wire with the two
    /// rails and are not printed.
    fn rails() -> PmicRails {
        PmicRails {
            vbus_mv: 5030,
            vsys_mv: 4960,
            vbusin: VbusInStatus::from_bits(0x21),
            charger_enabled: Some(true),
            boost_off: Some(false),
            charge_reduced: true,
        }
    }

    #[test]
    fn a_wired_board_puts_each_half_s_rails_in_its_own_row() {
        let (summary, telemetry) = wired_pair();
        let report = PmicRailsReport {
            version: 1,
            left: Some(PmicRails {
                vbus_mv: 4620,
                vsys_mv: 4550,
                ..rails()
            }),
            right: Some(PmicRails {
                vbus_mv: 4580,
                vsys_mv: 4510,
                vbusin: VbusInStatus::from_bits(0x01),
                ..rails()
            }),
        };

        assert_eq!(
            describe_charge(&summary, &telemetry, Some(report)),
            vec![
                "       cell    current  temp     VBUS    VSYS    full in     charger           fault",
                "left   4.02 V  +512 mA  28.5 °C  4.62 V  4.55 V  1 h 20 min  constant current  none",
                "right  3.98 V  +298 mA  27.9 °C  4.58 V  4.51 V  2 h 05 min  constant current  none",
            ]
        );
    }

    #[test]
    fn a_half_with_no_rails_has_n_a_in_both_rail_columns() {
        // An older firmware answers the rails command with nothing for a half,
        // and that half's two rail columns say so rather than dropping out.
        let (summary, telemetry) = wired_pair();
        let report = PmicRailsReport {
            version: 1,
            left: None,
            right: Some(PmicRails {
                vbus_mv: 4580,
                vsys_mv: 4510,
                vbusin: VbusInStatus::from_bits(0x01),
                ..rails()
            }),
        };

        assert_eq!(
            describe_charge(&summary, &telemetry, Some(report)),
            vec![
                "       cell    current  temp     VBUS    VSYS    full in     charger           fault",
                "left   4.02 V  +512 mA  28.5 °C  n/a     n/a     1 h 20 min  constant current  none",
                "right  3.98 V  +298 mA  27.9 °C  4.58 V  4.51 V  2 h 05 min  constant current  none",
            ]
        );
    }

    #[test]
    fn a_board_on_the_cable_counts_down_to_full() {
        // One half finished and one still filling: the column is headed once
        // for the board, and the finished half has no time to name.
        let (summary, mut telemetry) = wired_pair();
        telemetry.left.detail = BatteryDetail {
            voltage_mv: Some(4170),
            current_ma: Some(0),
            temp_cdeg: Some(3440),
            ttf_min: None,
            charger: Some(ChargerStatus::from_bits(0x01 | 0x02)),
            ..filling(4170, 0, 3440, 0)
        };
        telemetry.right.detail = filling(3990, 161, 3620, 386);
        let report = PmicRailsReport {
            version: 1,
            left: Some(rails()),
            right: Some(PmicRails {
                vbus_mv: 4680,
                vsys_mv: 4680,
                vbusin: VbusInStatus::from_bits(0x01),
                ..rails()
            }),
        };

        assert_eq!(
            describe_charge(&summary, &telemetry, Some(report)),
            vec![
                "       cell    current  temp     VBUS    VSYS    full in     charger           fault",
                "left   4.17 V  +0 mA    34.4 °C  5.03 V  4.96 V  n/a         charge complete   none",
                "right  3.99 V  +161 mA  36.2 °C  4.68 V  4.68 V  6 h 26 min  constant current  none",
            ]
        );
    }

    /// One half running down on battery, with nothing latched.
    fn running_down(voltage_mv: u16, current_ma: i16, temp_cdeg: i16, tte_min: u16) -> BatteryDetail {
        BatteryDetail {
            voltage_mv: Some(voltage_mv),
            current_ma: Some(current_ma),
            temp_cdeg: Some(temp_cdeg),
            tte_min: Some(tte_min),
            ttf_min: None,
            charger: Some(ChargerStatus::from_bits(0x01)),
            fault: Some(ChargerFault::default()),
        }
    }

    /// One half's rails on battery: nothing attached, so the input rail reads
    /// zero because there is no input.
    fn off_a_source(vsys_mv: u16) -> PmicRails {
        PmicRails {
            vbus_mv: 0,
            vsys_mv,
            vbusin: VbusInStatus::from_bits(0x00),
            charger_enabled: Some(false),
            ..rails()
        }
    }

    #[test]
    fn a_board_on_battery_counts_down_to_empty() {
        // No half on a source and no charger phase anywhere: the column is
        // headed the other way, and the input rail is n/a on both rows.
        let detail = ChargeDetail {
            left: running_down(4140, -40, 3120, 369),
            right: running_down(3880, -31, 3050, 130),
            left_rails: Some(off_a_source(4140)),
            right_rails: Some(off_a_source(3880)),
        };

        assert_eq!(
            ble_report(Some(&detail)),
            vec![
                "       cell    current  temp     VBUS  VSYS    empty in    charger       fault",
                "left   4.14 V  -40 mA   31.2 °C  n/a   4.14 V  6 h 09 min  charger idle  none",
                "right  3.88 V  -31 mA   30.5 °C  n/a   3.88 V  2 h 10 min  charger idle  none",
            ]
        );
    }

    #[test]
    fn a_half_with_no_estimate_the_column_asks_for_says_n_a() {
        // Heading for full, and the half only knows how long until empty.
        let mut detail = filling(4020, 512, 2850, 80);
        detail.ttf_min = None;
        detail.tte_min = Some(45);
        assert_eq!(Estimate::Full.minutes(&detail), None);
        assert_eq!(Estimate::Empty.minutes(&detail), Some(45));

        // And a finished charge is not filling, whatever the gauge carries.
        let mut done = filling(4170, 0, 3520, 30);
        done.charger = Some(ChargerStatus::from_bits(0x01 | 0x02));
        assert_eq!(Estimate::Full.minutes(&done), None);
    }

    #[test]
    fn an_estimate_under_a_minute_is_not_zero_minutes() {
        assert_eq!(duration(0), "under a minute");

        let detail = filling(4200, 131, 3500, 0);
        assert_eq!(
            detail_row("left", &detail, Estimate::Full, Some(true), None)[6],
            "under a minute"
        );
    }

    #[test]
    fn a_board_is_filling_while_anything_says_it_is_on_a_source() {
        let idle = running_down(4020, -45, 2850, 750);
        let attached = off_a_source(4020);

        // Nothing attached and no phase anywhere.
        assert_eq!(
            heading_for([(&idle, Some(attached)), (&idle, Some(attached))]),
            Estimate::Empty
        );
        // One half's rails say an input is attached.
        assert_eq!(
            heading_for([(&idle, Some(attached)), (&idle, Some(rails()))]),
            Estimate::Full
        );
        // Or one half's charger names a phase, with no rails read at all.
        let charging = filling(4020, 512, 2850, 80);
        assert_eq!(
            heading_for([(&idle, None), (&charging, None)]),
            Estimate::Full
        );
        assert_eq!(heading_for([(&idle, None), (&idle, None)]), Estimate::Empty);
        // A completed charge is a board on a source too.
        let mut done = charging;
        done.charger = Some(ChargerStatus::from_bits(0x01 | 0x02));
        assert_eq!(heading_for([(&idle, None), (&done, None)]), Estimate::Full);
    }

    #[test]
    fn a_half_names_only_the_two_rails_it_measured() {
        let detail = filling(3990, 161, 3620, 386);
        let row = detail_row("right", &detail, Estimate::Full, Some(true), Some(rails()));
        assert_eq!(row[4], "5.03 V");
        assert_eq!(row[5], "4.96 V");

        // What the input regulator, the charger register, the boost and the
        // reduced-charge flag say is decoded and never printed.
        let line = table(&[row]).remove(0);
        for word in ["in:", "charger enabled", "boost", "charge reduced"] {
            assert!(!line.contains(word), "{word}");
        }
    }

    #[test]
    fn a_half_with_nothing_attached_names_only_its_system_rail() {
        // On battery the input rail reads zero because there is no input, so
        // its column is n/a and the system rail stays.
        let detail = running_down(3990, -45, 3620, 386);
        let row = detail_row("left", &detail, Estimate::Empty, None, Some(off_a_source(4150)));
        assert_eq!(row[4], "n/a");
        assert_eq!(row[5], "4.15 V");
    }

    #[test]
    fn a_half_on_an_input_that_is_not_carrying_still_names_it() {
        // Attached and suspended, or attached and held at its limit: there is
        // an input to measure, so the column carries it.
        let detail = filling(3990, 161, 3620, 386);
        for raw in [0x01, 0x11, 0x03] {
            let attached = PmicRails {
                vbusin: VbusInStatus::from_bits(raw),
                ..rails()
            };
            let row = detail_row("right", &detail, Estimate::Full, Some(true), Some(attached));
            assert_eq!(row[4], "5.03 V", "{raw:#04x}");
        }
    }

    #[test]
    fn a_column_is_as_wide_as_the_widest_thing_in_it() {
        let rows = [
            vec!["".into(), "cell".into(), "fault".into()],
            vec!["left".into(), "4.02 V".into(), "none".into()],
            vec!["right".into(), "n/a".into(), "trickle timeout".into()],
        ];
        assert_eq!(
            table(&rows),
            vec![
                "       cell    fault",
                "left   4.02 V  none",
                "right  n/a     trickle timeout",
            ]
        );
        // Nothing trails the last column, whatever is under it.
        for line in table(&rows) {
            assert_eq!(line.trim_end(), line);
        }
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
        // Every column n/a, so the row still lines up under a half that
        // answered — and a flat cell is never invented from a missing one.
        assert_eq!(
            detail_row("right", &BatteryDetail::default(), Estimate::Full, None, None),
            vec!["right", "n/a", "n/a", "n/a", "n/a", "n/a", "n/a", "n/a", "n/a"]
        );
    }

    #[test]
    fn hours_only_appear_once_there_is_one() {
        assert_eq!(duration(0), "under a minute");
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
        // Under a heading of its own, so a clear register is "none" rather
        // than a sentence.
        assert_eq!(fault(ChargerFault::default()), "none");
        assert_eq!(
            fault(ChargerFault {
                reason: 0x40,
                sensor: 0x00
            }),
            "trickle timeout"
        );
        assert_eq!(
            fault(ChargerFault {
                reason: 0x20,
                sensor: 0x08
            }),
            "charge timeout (NTC hot)"
        );
        // A bit with no name is still a fault, and says which one it was.
        assert_eq!(
            fault(ChargerFault {
                reason: 0x80,
                sensor: 0x00
            }),
            "0x80"
        );
    }

    #[test]
    fn a_board_that_said_nothing_says_nothing() {
        assert_eq!(charge(Battery::default(), None), "");
        // Including one connected but unreadable, which must not read as flat.
        let battery = Battery {
            keyboard: Level::Unknown,
            ..Battery::default()
        };
        assert_eq!(charge(battery, None), "");
    }
}
