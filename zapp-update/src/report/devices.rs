//! The `zapp devices` lines for a wired board, shared with `zapp report` and
//! front ends that write the same report.

use zapp_core::device::UsbDevice;
use zapp_core::device::ids::{self as ids, target_name_for_pid};
use zapp_core::hid::{
    BatteryDetail, BatteryReading, BleIdentity, ChargeIssue, ChargeTelemetry, ChargerFault,
    ChargerStatus, EsbLink, HostSlot, Nav, NavReport, PmicRails, PmicRailsReport, RadioInfo,
    Telemetry,
};

use crate::probe::UsbTelemetry;

/// What [`crate::probe::report_on`] answers.
pub type UsbAnswer = (UsbTelemetry, Option<RadioInfo>, Vec<HostSlot>);

/// The [`WiredReport`] for what [`usb_report`](crate::probe::usb_report) or
/// [`report_on`](crate::probe::report_on) answered.
pub fn wired_report_from(answer: Option<UsbAnswer>, verbose: bool) -> WiredReport {
    let Some(((telemetry, detail, rails), radio, slots)) = answer else {
        return WiredReport::default();
    };

    let mut lines = Vec::new();
    if verbose {
        lines.extend(describe_telemetry(&telemetry));
        lines.extend(host_slot_lines(&slots));
        if let Some(detail) = &detail {
            lines.extend(describe_charge(&telemetry, detail, rails));
        }
    }
    WiredReport {
        charge: wired_charge(&telemetry, detail.as_ref()),
        lines,
        ble_address: telemetry.ble.map(|ble| ble.address),
        radio,
    }
}

/// "radio    · left 0.1.20 · 0a1b2c3d · right 4e5f6a7b", with full hashes,
/// build number and confirm state under `--verbose`.
pub fn radio_line(radio: &RadioInfo, verbose: bool) -> String {
    if !radio.is_known() {
        return format!("{:<8} · unknown", "radio");
    }
    let short = if verbose { 8 } else { 4 };
    let confirmed = |confirmed: bool| match (verbose, confirmed) {
        (true, true) => " · confirmed",
        (_, false) => " · not confirmed",
        (false, true) => "",
    };
    let left = match &radio.left {
        Some(left) => {
            let build = match left.version.build {
                build if verbose && build != 0 => format!("+{build}"),
                _ => String::new(),
            };
            format!(
                "left {}{build} · {}{}",
                left.version,
                hex(&left.hash[..short]),
                confirmed(left.confirmed)
            )
        }
        None => "left unknown".to_string(),
    };
    let right = match &radio.right {
        Some(right) => format!(
            "right {}{}",
            hex(&right.tag[..short]),
            confirmed(right.confirmed)
        ),
        None => "right unknown".to_string(),
    };
    format!("{:<8} · {left} · {right}", "radio")
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// What a wired board answered: what its halves have left, which goes on the
/// board's own line; the lines to print under it, which only `--verbose` asks
/// for; and the Bluetooth address it is using, which the roster matches its
/// bonded entries against; and the radio firmware it runs, for firmware that
/// can say.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct WiredReport {
    pub charge: String,
    pub lines: Vec<String>,
    pub ble_address: Option<String>,
    pub radio: Option<RadioInfo>,
}

/// What each half of a wired board has left, in the shape and the wording the
/// Bluetooth roster line uses, ready to append to the board's own line.
///
/// The charger phase from the detail read words each half when it arrived, so
/// this and the table under it say the same thing; the summary's charging flag
/// words it only for firmware that answers no detail at all.
pub fn wired_charge(telemetry: &Telemetry, detail: Option<&ChargeTelemetry>) -> String {
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

    match telemetry.charge_issue.is_some_and(ChargeIssue::is_unstable) {
        true => format!(
            " · {} · charging unstable on this USB port, try another port",
            halves.join(" · ")
        ),
        false => format!(" · {}", halves.join(" · ")),
    }
}

/// The lines under a wired board that stand on their own: the Bluetooth
/// identity it is using, how the halves are linked, what the board did about
/// unstable charging on its USB port, and which pointing devices the build
/// carries and which of them answer. Only `--verbose` asks for these.
pub fn describe_telemetry(telemetry: &Telemetry) -> Vec<String> {
    let mut lines = Vec::new();
    if let Some(ble) = &telemetry.ble {
        lines.push(format!("{:<8} · {}", "bluetooth", ble_identity(ble)));
    }
    lines.extend(halves_line(telemetry));
    lines.extend(charge_issue_line(telemetry));
    lines.push(features_line(&telemetry.nav()));
    lines
}

/// What the board did about unstable charging on its USB port, or nothing
/// while charging is stable or the board did not say.
pub fn charge_issue_line(telemetry: &Telemetry) -> Option<String> {
    let said = match telemetry.charge_issue? {
        ChargeIssue::Reduced => "charging power lowered",
        ChargeIssue::AtFloor => "at lowest charging power",
        ChargeIssue::None | ChargeIssue::Other(_) => return None,
    };
    Some(format!("{:<8} · {said}", "usb"))
}

/// How the halves are linked: the cable, or the radio between them. The radio
/// is left out with the cable plugged, where the right half stops it.
pub fn halves_line(telemetry: &Telemetry) -> Option<String> {
    let radio = telemetry.esb_link.map(|link| match link {
        EsbLink::Linked => "radio linked",
        EsbLink::Absent => "radio not linked",
        EsbLink::Mismatch => "radio key mismatch",
    });
    let link = match (telemetry.trrs, radio) {
        (Some(true), _) => "cable plugged".to_string(),
        (Some(false), Some(radio)) => format!("cable unplugged, {radio}"),
        (Some(false), None) => "cable unplugged".to_string(),
        (None, Some(radio)) => radio.to_string(),
        (None, None) => return None,
    };
    Some(format!("{:<8} · {link}", "halves"))
}

/// One line per Bluetooth host slot, counted from one:
/// "slot 1: C4:12:9A:33:0E:7B  paired, connected".
pub fn host_slot_lines(slots: &[HostSlot]) -> Vec<String> {
    slots
        .iter()
        .map(|slot| {
            let number = u16::from(slot.slot) + 1;
            let Some(address) = &slot.address else {
                return format!("slot {number}: unknown");
            };
            let state = match (slot.paired, slot.connected) {
                (true, true) => "paired, connected",
                (true, false) => "paired",
                (false, true) => "connected",
                (false, false) => "free",
            };
            format!("slot {number}: {address}  {state}")
        })
        .collect()
}

/// The address the board is using. The slot fields come off the wire with it
/// and are not printed.
pub fn ble_identity(ble: &BleIdentity) -> String {
    ble.address.clone()
}

/// The lines under a wired board once its charge telemetry has been read: what
/// each half's cell and charger are doing.
///
/// The summary read comes along for each half's VBUS flag, which the detail
/// block does not carry and the charger phase needs.
pub fn describe_charge(
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
pub enum Estimate {
    Full,
    Empty,
}

impl Estimate {
    pub fn heading(self) -> &'static str {
        match self {
            Self::Full => "full in",
            Self::Empty => "empty in",
        }
    }

    /// What one half has left the way the board is going, or nothing when the
    /// gauge did not say or the half is not going that way. A charge that has
    /// terminated is not filling, whatever estimate the gauge still carries.
    pub fn minutes(self, detail: &BatteryDetail) -> Option<u16> {
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
pub fn heading_for(halves: [(&BatteryDetail, Option<PmicRails>); 2]) -> Estimate {
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
pub fn charging_phase(status: ChargerStatus) -> bool {
    status.complete
        || status.trickle
        || status.constant_current
        || status.constant_voltage
        || status.recharge
        || status.paused_die_temp
}

/// The headings the halves' rows sit under. The half's own name has none.
pub fn detail_header(going: Estimate) -> Vec<String> {
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
pub fn detail_row(
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
pub fn table(rows: &[Vec<String>]) -> Vec<String> {
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

pub fn not_available() -> String {
    "n/a".to_string()
}

pub fn volts(millivolts: u16) -> String {
    format!("{:.2} V", f64::from(millivolts) / 1000.0)
}

/// Signed, because the sign is the whole of what it says: into the cell or out
/// of it.
pub fn milliamps(current: i16) -> String {
    format!("{current:+} mA")
}

pub fn degrees(centidegrees: i16) -> String {
    format!("{:.1} °C", f64::from(centidegrees) / 100.0)
}

/// Hours and minutes, or minutes on their own under an hour. An estimate that
/// has run down to nothing is under a minute rather than none.
pub fn duration(minutes: u16) -> String {
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
pub fn phase(status: ChargerStatus, usb: Option<bool>) -> String {
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
pub fn fault(fault: ChargerFault) -> String {
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
pub const FAST_CHARGE_BUDGET_MA: u16 = 1500;

pub fn charge_speed(budget_ma: Option<u16>) -> Option<&'static str> {
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
pub fn read_as(reading: BatteryReading, speed: Option<&str>) -> String {
    let level = level_as(reading.level);
    match reading.charging {
        Some(true) => format!("{level} {}", charging_as(speed)),
        _ => level,
    }
}

/// One half's level and what its charger is doing, out of the same phase the
/// detail line names.
pub fn read_in_phase(reading: BatteryReading, status: ChargerStatus, speed: Option<&str>) -> String {
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
pub fn phase_as(status: ChargerStatus, usb: Option<bool>, speed: Option<&str>) -> Option<String> {
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

pub fn level_as(level: Option<u8>) -> String {
    match level {
        Some(level) => format!("{level}%"),
        None => "unknown".to_string(),
    }
}

pub fn charging_as(speed: Option<&str>) -> String {
    match speed {
        Some(speed) => format!("{speed} charging"),
        None => "charging".to_string(),
    }
}

/// One entry per pointing device: unknown when the build is not known, not
/// supported when the build cannot drive it, connected with the side it is on
/// (or with no side when only the shared bus found it), disconnected when both
/// halves answered without it, and unknown otherwise.
pub fn features(nav: &NavReport) -> String {
    let devices: [(&str, fn(Nav) -> bool); 2] = [
        ("Trackball", |nav| nav.trackball),
        ("Trackpad", |nav| nav.trackpad),
    ];
    devices
        .into_iter()
        .map(|(name, has)| {
            let found = |side: Option<Nav>| side.is_some_and(has);
            let state = match nav.build {
                None => "unknown",
                Some(build) if !has(build) => "not supported",
                Some(_) => match (found(nav.left), found(nav.right)) {
                    (true, true) => "connected, left and right",
                    (true, false) => "connected, left",
                    (false, true) => "connected, right",
                    (false, false) if found(nav.bus) => "connected",
                    _ if nav.left.is_some() && nav.right.is_some() => "disconnected",
                    // A non-zero bus mask comes from a bus that reaches both halves.
                    _ if nav.bus.is_some_and(|bus| bus != Nav::default()) => "disconnected",
                    _ => "unknown",
                },
            };
            format!("{name} ({state})")
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The features line, as wired and Bluetooth boards print it.
pub fn features_line(nav: &NavReport) -> String {
    format!("{:<8} · {}", "features", features(nav))
}

pub fn describe_usb(device: &UsbDevice) -> String {
    let name = match device.bootloader {
        Some(_) => ids::friendly_name(device.vid, device.pid).to_string(),
        None => target_name_for_pid(device.pid).to_string(),
    };
    match &device.serial {
        Some(serial) if !serial.is_empty() => format!("{name} · {serial}"),
        _ => name,
    }
}
