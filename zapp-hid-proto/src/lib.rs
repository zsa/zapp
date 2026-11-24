//! Wire format of the ZSA keyboard raw-HID protocol: live key events, layer
//! changes, pairing.
//!
//! ZSA boards expose a raw HID interface on usage page 0xFF60 (the QMK raw
//! HID page). A report is 32 bytes whose first byte is a command/event opcode
//! and whose remaining bytes are parameters terminated by STOP_BIT (0xFE).
//!
//! This crate is the codec only; the transport lives with the consumer (e.g.
//! hidapi in `zapp-core`). It has no transport dependencies and builds for any
//! target. Report IDs are the transport's business: the codec produces the
//! bare 32-byte report and hidapi, for one, prepends a `0x00` byte.

/// HID usage page of the ZSA/QMK raw interface.
pub const USAGE_PAGE: u16 = 0xFF60;
/// Report size, excluding any transport-specific report ID byte.
pub const REPORT_SIZE: usize = 32;
/// Terminates the parameter list of incoming packets.
pub const STOP_BIT: u8 = 0xFE;

// Host → device commands
pub const CMD_GET_FW_VERSION: u8 = 0x00;
pub const CMD_INIT_PAIRING: u8 = 0x01;
pub const CMD_VALIDATE_PAIRING: u8 = 0x02;
pub const CMD_SET_LAYER: u8 = 0x04;
pub const CMD_RGB_CONTROL: u8 = 0x05;
pub const CMD_SET_RGB_LED: u8 = 0x06;
pub const CMD_SET_STATUS_LED: u8 = 0x07;
pub const CMD_UPDATE_BRIGHTNESS: u8 = 0x08;
pub const CMD_SET_RGB_ALL_LED: u8 = 0x09;
pub const CMD_STATUS_LED_CONTROL: u8 = 0x0A;
// 0x0B/0x0C are the firmware's ORYX_SET_AUTOMOUSE/ORYX_GET_AUTOMOUSE, unused by
// zapp. Take opcodes from the firmware enum, never by counting down this list.
/// Ask the board about itself: battery per half, what it can drive, what is
/// plugged in. Answered with `EVT_TELEMETRY`.
pub const CMD_GET_TELEMETRY: u8 = 0x0D;
/// Ask the board what the charger and the fuel gauges read, per half: the
/// volts, milliamps and degrees behind the percentage `CMD_GET_TELEMETRY`
/// answers with. Answered with `EVT_CHARGE_TELEMETRY`.
pub const CMD_GET_CHARGE_TELEMETRY: u8 = 0x0E;
/// Ask the board what each half's PMIC reads on its input and system rails,
/// and what its input regulator and charger are set to. Answered with
/// `EVT_PMIC_RAILS`.
pub const CMD_GET_PMIC_RAILS: u8 = 0x0F;
// 0x10 is the firmware's ORYX_CMD_GET_SCAN_DIAG, unused by zapp.
/// Ask the board for one page of a diagnostic record: params are record,
/// page and flags (`DIAG_FETCH`). Answered with `EVT_DIAG`.
pub const CMD_GET_DIAG: u8 = 0x11;
/// Ask the board which radio firmware each half runs. Only for boards at
/// `RADIO_INFO_PROTOCOL` or later. Answered with `EVT_RADIO_INFO`.
pub const CMD_GET_RADIO_INFO: u8 = 0x12;
/// Ask the board for one of its Bluetooth host slots: the param is the slot,
/// counted from zero. Answered with `EVT_HOST_SLOTS`; firmware before it does
/// not answer.
pub const CMD_GET_HOST_SLOTS: u8 = 0x13;
/// Ask the board for one chunk of a left nRF blob: params are the blob
/// (`BLOB_FREEZE`, `BLOB_EVENTS`) and the offset, little-endian. Answered with
/// `EVT_BLOB`; firmware before it answers `EVT_ERROR`.
pub const CMD_GET_BLOB: u8 = 0x14;
/// Clear the left nRF freeze record's unread flag. Answered with
/// `EVT_FREEZE_SEEN`.
pub const CMD_FREEZE_SEEN: u8 = 0x15;
pub const CMD_GET_PROTOCOL_VERSION: u8 = 0xFE;

// Device → host events
pub const EVT_GET_FIRMWARE_VERSION: u8 = 0x00;
pub const EVT_PAIRING_INIT: u8 = 0x01;
pub const EVT_PAIRING_KEY_INPUT: u8 = 0x02;
pub const EVT_PAIRING_FAIL: u8 = 0x03;
pub const EVT_PAIRING_SUCCESS: u8 = 0x04;
pub const EVT_LAYER_CHANGE: u8 = 0x05;
pub const EVT_KEY_PRESSED: u8 = 0x06;
pub const EVT_KEY_RELEASED: u8 = 0x07;
pub const EVT_TOGGLE_SMARTLAYER: u8 = 0x09;
pub const EVT_TRIGGER_SMARTLAYER: u8 = 0x0A;
// 0x0C is the firmware's ORYX_EVT_AUTOMOUSE, unused by zapp. Take opcodes from
// the firmware enum, never by counting down this list.
/// Answer to `CMD_GET_TELEMETRY`. Its payload is binary and fills the report,
/// so it carries no STOP_BIT: a 0xFE inside it is data.
pub const EVT_TELEMETRY: u8 = 0x0D;
/// Answer to `CMD_GET_CHARGE_TELEMETRY`. Binary and STOP_BIT-free for the same
/// reason as `EVT_TELEMETRY`: it fills the report and a 0xFE in it is data.
pub const EVT_CHARGE_TELEMETRY: u8 = 0x0E;
/// Answer to `CMD_GET_PMIC_RAILS`. Binary and STOP_BIT-free for the same
/// reason as `EVT_TELEMETRY`: a 0xFE among the millivolts is a value.
pub const EVT_PMIC_RAILS: u8 = 0x0F;
// 0x10 is the firmware's ORYX_EVT_GET_SCAN_DIAG, unused by zapp.
/// Answer to `CMD_GET_DIAG`. Binary and STOP_BIT-free: a 0xFE in it is data.
pub const EVT_DIAG: u8 = 0x11;
/// Answer to `CMD_GET_RADIO_INFO`. Binary and STOP_BIT-free: a 0xFE in a hash
/// is data.
pub const EVT_RADIO_INFO: u8 = 0x12;
/// Answer to `CMD_GET_HOST_SLOTS`. Binary and STOP_BIT-free: a 0xFE in an
/// address is data.
pub const EVT_HOST_SLOTS: u8 = 0x13;
/// Answer to `CMD_GET_BLOB`. Binary and STOP_BIT-free.
pub const EVT_BLOB: u8 = 0x14;
/// Answer to `CMD_FREEZE_SEEN`: one status byte, `BLOB_OK` or
/// `BLOB_NO_LINK`.
pub const EVT_FREEZE_SEEN: u8 = 0x15;
/// The board refused a command; the param is the error code.
pub const EVT_ERROR: u8 = 0xFF;
/// `EVT_ERROR` code for a command the firmware does not know.
pub const ERR_UNKNOWN_COMMAND: u8 = 0xFF;
pub const EVT_GET_PROTOCOL_VERSION: u8 = 0xFD;

/// Telemetry format this host understands. Only bumped by firmware when a
/// field that has already shipped changes meaning, which is exactly the case a
/// host cannot guess its way through — so a payload carrying anything else is
/// refused whole rather than misread. Fields *added* do not touch this: they
/// raise the payload's own length byte, and a short one simply leaves the
/// fields we did not get unknown.
pub const TELEMETRY_VERSION: u8 = 1;

/// Byte offsets inside the telemetry payload, which begins at the byte after
/// the opcode. `version` and `len` are at 0 and 1 in every version; `len`
/// counts the defined bytes after itself, so a field at offset `n` is present
/// only while `n < len + 2`.
mod telemetry_offset {
    pub const VERSION: usize = 0;
    pub const LEN: usize = 1;
    pub const BATTERY_LEFT: usize = 2;
    pub const BATTERY_LEFT_FLAGS: usize = 3;
    pub const BATTERY_LEFT_SOURCE: usize = 4;
    pub const BATTERY_RIGHT: usize = 5;
    pub const BATTERY_RIGHT_FLAGS: usize = 6;
    pub const BATTERY_RIGHT_SOURCE: usize = 7;
    pub const NAV_BUILD: usize = 8;
    pub const NAV_LEFT: usize = 9;
    pub const NAV_RIGHT: usize = 10;
    /// Milliamps, little-endian pairs — see [`telemetry_field_le16`].
    pub const POWER_BUDGET_MA: usize = 11;
    pub const CHARGE_LIMIT_MA: usize = 13;
    pub const CHARGE_CURRENT_MA: usize = 15;
    pub const CHARGE_STATE: usize = 17;
    pub const BLE_ADDR_TYPE: usize = 18;
    /// Six bytes, `bt_addr_t` order — least significant first.
    pub const BLE_ADDR: usize = 19;
    pub const BLE_SLOT: usize = 25;
    pub const BLE_SLOT_COUNT: usize = 26;
    pub const NAV_BUS: usize = 27;
    pub const TRRS: usize = 28;
    pub const ESB_LINK: usize = 29;
    pub const CHARGE_ISSUE: usize = 30;
}

/// Bytes in the address the identity block carries.
const BLE_ADDR_LEN: usize = 6;

/// The board has no Bluetooth identity in the active slot yet, so the address
/// beside it is meaningless rather than zero.
const TELEMETRY_BLE_ADDR_UNKNOWN: u8 = 0xFF;
/// `addr_type` for an identity whose address is a random one.
const TELEMETRY_BLE_ADDR_RANDOM: u8 = 0x01;

/// A battery level the firmware could not read. Never 0: that is a flat one.
const TELEMETRY_LEVEL_UNKNOWN: u8 = 0xFF;
/// A half whose pointing device could not be determined, as against 0, which
/// means nothing is plugged into it.
const TELEMETRY_NAV_UNKNOWN: u8 = 0xFF;
const TELEMETRY_NAV_TRACKBALL: u8 = 0x01;
const TELEMETRY_NAV_TRACKPAD: u8 = 0x02;
const TELEMETRY_BAT_CHARGING: u8 = 0x01;
const TELEMETRY_BAT_USB: u8 = 0x02;
/// Clear means the flag bits are unknown, not false.
const TELEMETRY_BAT_KNOWN: u8 = 0x80;
/// A current the board could not report. Zero, because no limit or rate it can
/// be set to is zero — the input floor alone is 100 mA.
const TELEMETRY_CURRENT_UNKNOWN: u16 = 0;
/// The board has nothing to say about what its charging policy is doing.
const TELEMETRY_CHG_UNKNOWN: u8 = 0xFF;
/// TRRS cable between the halves.
const TELEMETRY_TRRS_UNPLUGGED: u8 = 0;
const TELEMETRY_TRRS_PLUGGED: u8 = 1;
#[cfg(test)]
const TELEMETRY_TRRS_UNKNOWN: u8 = 0xFF;
/// The left half's radio link to the right half.
const TELEMETRY_ESB_ABSENT: u8 = 0;
const TELEMETRY_ESB_LINKED: u8 = 1;
const TELEMETRY_ESB_MISMATCH: u8 = 2;
#[cfg(test)]
const TELEMETRY_ESB_UNKNOWN: u8 = 0xFF;
/// Charging trouble on the USB port, wired only.
const TELEMETRY_CHARGE_ISSUE_NONE: u8 = 0;
const TELEMETRY_CHARGE_ISSUE_REDUCED: u8 = 1;
const TELEMETRY_CHARGE_ISSUE_AT_FLOOR: u8 = 2;
const TELEMETRY_CHARGE_ISSUE_UNKNOWN: u8 = 0xFF;

/// Charge-telemetry format this host understands, and refused whole when it is
/// anything else — same rule, and for the same reason, as [`TELEMETRY_VERSION`].
pub const CHARGE_TELEMETRY_VERSION: u8 = 1;

/// Byte offsets inside the charge-telemetry payload, the bytes after the
/// `EVT_CHARGE_TELEMETRY` opcode. Same header as the telemetry payload:
/// `version` and `len` first, and `len` counts the defined bytes after itself.
mod charge_offset {
    pub const VERSION: usize = 0;
    pub const LEN: usize = 1;
    pub const LEFT_SOURCE: usize = 2;
    /// The left half's 13-byte detail block — see [`super::detail_offset`].
    pub const LEFT: usize = 3;
    pub const RIGHT_SOURCE: usize = 16;
    pub const RIGHT: usize = 17;
}

/// Byte offsets inside one detail block, which is the same 13 bytes wherever it
/// travels: inside a HID report, and inside the board's Bluetooth read.
mod detail_offset {
    pub const VOLTAGE_MV: usize = 0;
    pub const CURRENT_MA: usize = 2;
    pub const TEMP_CDEG: usize = 4;
    pub const TTE_MIN: usize = 6;
    pub const TTF_MIN: usize = 8;
    pub const CHG_STATUS: usize = 10;
    pub const CHG_ERROR: usize = 11;
    pub const CHG_SENSOR: usize = 12;
}

/// Bytes in one detail block.
pub const BATTERY_DETAIL_SIZE: usize = 13;

/// Byte offsets inside one PMIC rails block.
mod rails_offset {
    pub const FLAGS: usize = 0;
    pub const VBUS_MV: usize = 1;
    pub const VSYS_MV: usize = 3;
    pub const VBUSIN_STATUS: usize = 5;
    pub const CHG_ENABLE: usize = 6;
}

/// Bytes in one PMIC rails block.
pub const PMIC_RAILS_SIZE: usize = 7;

/// Rails-report format this host understands, and refused whole when it is
/// anything else — same rule, and for the same reason, as [`TELEMETRY_VERSION`].
pub const PMIC_RAILS_REPORT_VERSION: u8 = 1;

/// Byte offsets inside the rails payload, the bytes after the `EVT_PMIC_RAILS`
/// opcode. Same header as the other payloads: `version` and `len` first, and
/// `len` counts the defined bytes after itself.
mod rails_report_offset {
    pub const VERSION: usize = 0;
    pub const LEN: usize = 1;
    /// The left half's 7-byte rails block — see [`super::rails_offset`].
    pub const LEFT: usize = 2;
    pub const RIGHT: usize = 9;
}

/// `flags` bit 0: the block carries a reading. Clear means the half has not
/// taken one yet, and the millivolts beside it are zeroes rather than volts.
const PMIC_RAILS_PRESENT: u8 = 0x01;

/// `flags` bit 1: the boost bit beside it is meaningful. Clear on firmware that
/// does not report the boost converter at all.
const PMIC_RAILS_BOOST_KNOWN: u8 = 0x02;

/// `flags` bit 2: the firmware is holding the boost converter off. Only worth
/// reading while [`PMIC_RAILS_BOOST_KNOWN`] is set.
const PMIC_RAILS_BOOST_OFF: u8 = 0x04;

/// `flags` bit 3: the half is holding its charge current below the rate it was
/// commanded, because the charger kept restarting its cycle at that rate.
const PMIC_RAILS_CHARGE_REDUCED: u8 = 0x08;

/// A PMIC register the firmware did not read this tick. Not zero: every bit
/// clear is a real register value.
const PMIC_REG_UNKNOWN: u8 = 0xFF;

/// A cell voltage the board could not read. Zero, because a cell at zero volts
/// is not a cell anything could have reported it from.
const DETAIL_MV_UNKNOWN: u16 = 0;
/// A current or a temperature the board could not read. The one value of the
/// range that is not a plausible reading, since zero is a real one for both.
const DETAIL_I16_UNKNOWN: i16 = i16::MIN;
/// An estimate the fuel gauge would not make. Not zero: zero minutes left is a
/// real, and rather important, answer.
const DETAIL_MIN_UNKNOWN: u16 = 0xFFFF;
/// The top of the minutes field, where the firmware saturates: an estimate this
/// long is the cap rather than a reading, and carries no information. Unknown
/// too, so nothing renders a board as having 45 days left.
const DETAIL_MIN_SATURATED: u16 = 0xFFFE;
/// A charger register that was not read this tick. Not zero: zero means every
/// bit clear, which is a charger sitting idle with nothing wrong.
const DETAIL_REG_UNKNOWN: u8 = 0xFF;

/// Where a battery reading came from. Worth surfacing rather than folding
/// away: a relayed level keeps its last value when the link carrying it stops,
/// so it can look exactly like a fresh one while being days old.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// Nothing produced this reading.
    Unknown,
    /// The left nRF, over the wire between it and the keyboard's MCU.
    Nrf,
    /// The right half over the halves' radio link, relayed by the left.
    Esb,
    /// The right half itself, read over the cable between the halves.
    Wired,
    /// A source this host has no name for: firmware newer than it.
    Other(u8),
}

impl Source {
    fn from_raw(raw: u8) -> Self {
        match raw {
            0 => Source::Unknown,
            1 => Source::Nrf,
            2 => Source::Esb,
            3 => Source::Wired,
            other => Source::Other(other),
        }
    }
}

/// Pointing devices named by one telemetry mask. Both false is a real answer —
/// nothing is plugged in. "We could not tell" is the absence of a `Nav`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Nav {
    pub trackball: bool,
    pub trackpad: bool,
}

impl Nav {
    /// One nav mask byte, as both the HID telemetry and the Bluetooth detail
    /// read carry it. `None` for the unknown sentinel.
    pub fn from_mask(mask: u8) -> Option<Self> {
        (mask != TELEMETRY_NAV_UNKNOWN).then_some(Nav {
            trackball: mask & TELEMETRY_NAV_TRACKBALL != 0,
            trackpad: mask & TELEMETRY_NAV_TRACKPAD != 0,
        })
    }

    /// True when the board looked and found nothing.
    pub fn is_empty(&self) -> bool {
        !self.trackball && !self.trackpad
    }
}

/// One half's battery. Every field is optional because "we could not read it"
/// and "it is zero" are different answers and the board distinguishes them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BatteryReading {
    /// Percent, or `None` when the board could not read it.
    pub level: Option<u8>,
    /// `None` is unknown, not "not charging".
    pub charging: Option<bool>,
    /// `None` is unknown, not "no USB".
    pub usb: Option<bool>,
    pub source: Option<Source>,
}

/// What the board's charging policy is doing, as against what the cells are
/// doing — [`BatteryReading::charging`] already says whether current flows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChargeState {
    /// No source attached: running off the cells.
    Battery,
    /// A source is attached but has not identified itself, so the board is
    /// still holding at the current a USB device may draw unconfigured.
    Floor,
    /// Charging at the rate the source allows.
    Charging,
    /// Source attached, charger has terminated.
    Full,
    /// A state this host has no name for: firmware newer than it.
    Other(u8),
}

impl ChargeState {
    fn from_raw(raw: u8) -> Option<Self> {
        match raw {
            TELEMETRY_CHG_UNKNOWN => None,
            0 => Some(ChargeState::Battery),
            1 => Some(ChargeState::Floor),
            2 => Some(ChargeState::Charging),
            3 => Some(ChargeState::Full),
            other => Some(ChargeState::Other(other)),
        }
    }
}

/// How fast the board is charging and why. Every field is optional for the
/// usual reason: a firmware that predates them says nothing rather than zero,
/// and the board itself distinguishes "no source" from "did not find out".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Charging {
    /// Milliamps the attached source advertised. `Some(0)` is a real answer —
    /// nothing attached, or attached and not yet identified.
    pub budget_ma: Option<u16>,
    /// Milliamps the board is allowed to draw from the source.
    pub input_limit_ma: Option<u16>,
    /// Milliamps going into each cell.
    pub charge_current_ma: Option<u16>,
    pub state: Option<ChargeState>,
}

/// The Bluetooth identity the board is using right now: the one the current
/// connection is on, or the one it would advertise as next.
///
/// A board keeps several, one per host it pairs with, and each has an address
/// of its own — so the address alone does not name the board, and the slot it
/// sits in is part of the answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BleIdentity {
    /// `FE:DB:E9:A8:B1:36` — most significant byte first, the way an address is
    /// written everywhere a person reads one, though it travels the other way.
    pub address: String,
    /// A random address rather than a public one, which is what a board that
    /// was never assigned an address block uses.
    pub random: bool,
    /// Which host slot this identity is, counted from zero.
    pub slot: u8,
    /// How many slots the board keeps. Zero when the firmware did not say.
    pub slot_count: u8,
}

/// Pointing devices a board reports: what the build drives, what each half
/// has, and what the shared bus has while the cable is plugged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NavReport {
    pub build: Option<Nav>,
    pub left: Option<Nav>,
    pub right: Option<Nav>,
    pub bus: Option<Nav>,
}

impl NavReport {
    /// Bytes in the build, left, right, bus block.
    pub const SIZE: usize = 4;

    /// Decode a build, left, right, bus block. `None` when it is short.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let [build, left, right, bus] = *bytes.get(..Self::SIZE)? else {
            return None;
        };
        Some(Self {
            build: Nav::from_mask(build),
            left: Nav::from_mask(left),
            right: Nav::from_mask(right),
            bus: Nav::from_mask(bus),
        })
    }
}

/// The left half's radio (ESB) link to the right half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EsbLink {
    /// Both halves hear each other and agree on the protocol.
    Linked,
    /// No right half heard.
    Absent,
    /// Both halves alive, running different protocol versions.
    Mismatch,
}

impl EsbLink {
    fn from_raw(raw: u8) -> Option<Self> {
        match raw {
            TELEMETRY_ESB_LINKED => Some(Self::Linked),
            TELEMETRY_ESB_ABSENT => Some(Self::Absent),
            TELEMETRY_ESB_MISMATCH => Some(Self::Mismatch),
            _ => None,
        }
    }
}

/// Charging trouble the board saw on its USB port. Reset to [`Self::None`]
/// when USB goes away, so it only means something while plugged in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChargeIssue {
    None,
    /// Charging was unstable; the board lowered its charging power.
    Reduced,
    /// Still unstable at the lowest charging power.
    AtFloor,
    /// An issue this host has no name for: firmware newer than it.
    Other(u8),
}

impl ChargeIssue {
    fn from_raw(raw: u8) -> Option<Self> {
        match raw {
            TELEMETRY_CHARGE_ISSUE_UNKNOWN => None,
            TELEMETRY_CHARGE_ISSUE_NONE => Some(Self::None),
            TELEMETRY_CHARGE_ISSUE_REDUCED => Some(Self::Reduced),
            TELEMETRY_CHARGE_ISSUE_AT_FLOOR => Some(Self::AtFloor),
            other => Some(Self::Other(other)),
        }
    }

    /// Charging on this port is unstable, whatever the board did about it.
    pub fn is_unstable(self) -> bool {
        matches!(self, Self::Reduced | Self::AtFloor)
    }
}

/// What a board says about itself.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Telemetry {
    pub version: u8,
    pub left: BatteryReading,
    pub right: BatteryReading,
    pub charging: Charging,
    /// Pointing devices this firmware was built to drive. Compile-time, so the
    /// board always knows it — `None` only if the field predates this host.
    pub nav_build: Option<Nav>,
    pub nav_left: Option<Nav>,
    pub nav_right: Option<Nav>,
    /// Pointing devices found on the bus the halves share while the cable is
    /// plugged, side unknown. Empty with the cable unplugged; `None` from a
    /// firmware that predates the field.
    pub nav_bus: Option<Nav>,
    /// Whether the TRRS cable between the halves is plugged. `None` when the
    /// board cannot tell, or from a firmware that predates the field.
    pub trrs: Option<bool>,
    /// The radio link between the halves. `None` when the board cannot tell,
    /// or from a firmware that predates the field.
    pub esb_link: Option<EsbLink>,
    /// `None` when the board cannot tell, or from a firmware that predates the
    /// field.
    pub charge_issue: Option<ChargeIssue>,
    /// `None` from a firmware that predates the field, and from a board that
    /// has no identity in its active slot yet.
    pub ble: Option<BleIdentity>,
}

impl Telemetry {
    pub fn nav(&self) -> NavReport {
        NavReport {
            build: self.nav_build,
            left: self.nav_left,
            right: self.nav_right,
            bus: self.nav_bus,
        }
    }
}

/// What the charger is doing, out of the nPM1300's `BCHGCHARGESTATUS`.
///
/// Every bit stands on its own: `battery_detected` and `supplement` say nothing
/// about which phase the charge is in, and more than one phase bit at a time is
/// the register's business rather than something to resolve here.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChargerStatus {
    pub battery_detected: bool,
    /// Charged, and the charger has terminated.
    pub complete: bool,
    /// Below the trickle threshold: filling slowly until the cell is fit for
    /// the full rate.
    pub trickle: bool,
    pub constant_current: bool,
    pub constant_voltage: bool,
    /// Fell back under the recharge threshold after completing.
    pub recharge: bool,
    /// Stopped because the die is too hot; resumes on its own.
    pub paused_die_temp: bool,
    /// The cell is helping the source carry the load.
    pub supplement: bool,
}

impl ChargerStatus {
    pub fn from_bits(raw: u8) -> Self {
        Self {
            battery_detected: raw & 0x01 != 0,
            complete: raw & 0x02 != 0,
            trickle: raw & 0x04 != 0,
            constant_current: raw & 0x08 != 0,
            constant_voltage: raw & 0x10 != 0,
            recharge: raw & 0x20 != 0,
            paused_die_temp: raw & 0x40 != 0,
            supplement: raw & 0x80 != 0,
        }
    }

    /// Whether the charger is in no phase at all: nothing to charge, nothing
    /// attached, or done long enough ago that even `complete` has been cleared.
    pub fn is_idle(&self) -> bool {
        !(self.complete
            || self.trickle
            || self.constant_current
            || self.constant_voltage
            || self.recharge
            || self.paused_die_temp)
    }
}

/// Why the charger stopped, and what the sensors read when it did:
/// `BCHGERRREASON` and `BCHGERRSENSOR`, raw.
///
/// Latched by the hardware until something clears it, so a fault reported here
/// is one that happened at some point in this session rather than one happening
/// now — the firmware clears these when a source is attached.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChargerFault {
    pub reason: u8,
    pub sensor: u8,
}

/// `BCHGERRREASON` bits, in register order.
const FAULT_REASONS: [(u8, &str); 7] = [
    (0x01, "NTC sensor error"),
    (0x02, "VBAT sensor error"),
    (0x04, "VBAT low"),
    (0x08, "trickle voltage"),
    (0x10, "measurement timeout"),
    (0x20, "charge timeout"),
    (0x40, "trickle timeout"),
];

/// `BCHGERRSENSOR` bits, in register order.
const FAULT_SENSORS: [(u8, &str); 8] = [
    (0x01, "NTC cold"),
    (0x02, "NTC cool"),
    (0x04, "NTC warm"),
    (0x08, "NTC hot"),
    (0x10, "Vterm"),
    (0x20, "recharge"),
    (0x40, "Vtrickle"),
    (0x80, "VBAT low"),
];

fn bit_names(bits: u8, table: &[(u8, &'static str)]) -> Vec<&'static str> {
    table
        .iter()
        .filter(|(bit, _)| bits & bit != 0)
        .map(|(_, name)| *name)
        .collect()
}

impl ChargerFault {
    /// Nothing has latched. The usual answer, and not the same as never having
    /// looked — that is a `None` where this sits.
    pub fn is_clear(&self) -> bool {
        self.reason == 0
    }

    pub fn ntc_sensor_error(&self) -> bool {
        self.reason & 0x01 != 0
    }
    pub fn vbat_sensor_error(&self) -> bool {
        self.reason & 0x02 != 0
    }
    pub fn vbat_low(&self) -> bool {
        self.reason & 0x04 != 0
    }
    pub fn trickle_voltage(&self) -> bool {
        self.reason & 0x08 != 0
    }
    pub fn measurement_timeout(&self) -> bool {
        self.reason & 0x10 != 0
    }
    pub fn charge_timeout(&self) -> bool {
        self.reason & 0x20 != 0
    }
    pub fn trickle_timeout(&self) -> bool {
        self.reason & 0x40 != 0
    }

    // The sensor register names the state the hardware was in, not a reason, so
    // its bits carry the prefix — two of them share a name with a reason bit.
    pub fn sensor_ntc_cold(&self) -> bool {
        self.sensor & 0x01 != 0
    }
    pub fn sensor_ntc_cool(&self) -> bool {
        self.sensor & 0x02 != 0
    }
    pub fn sensor_ntc_warm(&self) -> bool {
        self.sensor & 0x04 != 0
    }
    pub fn sensor_ntc_hot(&self) -> bool {
        self.sensor & 0x08 != 0
    }
    pub fn sensor_vterm(&self) -> bool {
        self.sensor & 0x10 != 0
    }
    pub fn sensor_recharge(&self) -> bool {
        self.sensor & 0x20 != 0
    }
    pub fn sensor_vtrickle(&self) -> bool {
        self.sensor & 0x40 != 0
    }
    pub fn sensor_vbat_low(&self) -> bool {
        self.sensor & 0x80 != 0
    }

    /// The reason bits that are set, named.
    pub fn reasons(&self) -> Vec<&'static str> {
        bit_names(self.reason, &FAULT_REASONS)
    }

    /// What the sensors read when it latched, named.
    pub fn sensors(&self) -> Vec<&'static str> {
        bit_names(self.sensor, &FAULT_SENSORS)
    }
}

/// One half's cell, in detail: what the gauge and the charger read rather than
/// the percentage they come to.
///
/// Optional throughout, as everywhere else here, because the board sends an
/// explicit "did not read it" for each field and none of them can be told from
/// a real reading by its value: 0 mA is a cell at rest, 0 minutes left is a
/// cell about to go flat.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BatteryDetail {
    /// Cell voltage, millivolts.
    pub voltage_mv: Option<u16>,
    /// Milliamps, positive into the cell and negative out of it.
    pub current_ma: Option<i16>,
    /// What the NTC beside the cell reads, hundredths of a degree Celsius.
    pub temp_cdeg: Option<i16>,
    /// Minutes the fuel gauge thinks are left at the present draw.
    pub tte_min: Option<u16>,
    /// Minutes the fuel gauge thinks are left until full.
    pub ttf_min: Option<u16>,
    pub charger: Option<ChargerStatus>,
    pub fault: Option<ChargerFault>,
}

/// What the nPM1300's `VBUSIN.VBUSINSTATUS` says about the input: whether
/// there is one, and what the input regulator is doing with it.
///
/// Every bit stands on its own, as in the charger's status register, so the raw
/// byte comes along: a firmware newer than this host can set a bit it has no
/// name for and the number still says so.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VbusInStatus {
    pub raw: u8,
    /// An input is attached.
    pub present: bool,
    /// The input regulator is holding at its current limit.
    pub current_limit: bool,
    /// The input is above the overvoltage threshold and is being ignored.
    pub overvoltage: bool,
    /// The input is below the undervoltage threshold.
    pub undervoltage: bool,
    /// The input is suspended: attached and deliberately not drawn from.
    pub suspended: bool,
    /// The PMIC is feeding its VBUS output.
    pub out_active: bool,
}

/// `VBUSINSTATUS` bits, in register order.
const VBUSIN_BITS: [(u8, &str); 6] = [
    (0x01, "present"),
    (0x02, "current limit"),
    (0x04, "overvoltage"),
    (0x08, "undervoltage"),
    (0x10, "suspended"),
    (0x20, "out active"),
];

impl VbusInStatus {
    pub fn from_bits(raw: u8) -> Self {
        Self {
            raw,
            present: raw & 0x01 != 0,
            current_limit: raw & 0x02 != 0,
            overvoltage: raw & 0x04 != 0,
            undervoltage: raw & 0x08 != 0,
            suspended: raw & 0x10 != 0,
            out_active: raw & 0x20 != 0,
        }
    }

    /// The bits that are set, named, in register order. Empty is a real
    /// answer: nothing attached and the output off.
    pub fn names(&self) -> Vec<&'static str> {
        bit_names(self.raw, &VBUSIN_BITS)
    }
}

/// One half's PMIC rails: what the input and the system rail measure, and what
/// the input regulator and the charger are set to.
///
/// A whole block or nothing — see [`decode_pmic_rails`] — because the flag that
/// says the half took a reading sits inside it, so a partial one cannot be told
/// from a half that has not measured yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PmicRails {
    /// The input rail, millivolts, as the PMIC's ADC read it.
    pub vbus_mv: u16,
    /// The system rail, millivolts.
    pub vsys_mv: u16,
    pub vbusin: VbusInStatus,
    /// Whether the charger is switched on at all, read back out of
    /// `BCHGENABLESET`. `None` when the firmware did not read the register —
    /// which is not the same as a charger that is off.
    pub charger_enabled: Option<bool>,
    /// Whether the firmware is holding the boost converter off. `None` on
    /// firmware that does not report the boost at all.
    pub boost_off: Option<bool>,
    /// Whether the half is holding its charge current below the rate it was
    /// commanded, because the charger kept restarting its cycle at that rate.
    pub charge_reduced: bool,
}

/// Decode one PMIC rails block: 7 bytes, all of them there and the block
/// marked as carrying a reading.
///
/// `None` for anything else, the two cases being a board whose firmware
/// predates the block (so the bytes are absent or zeroed) and one that has not
/// run a measurement tick yet. Neither has rails to report.
pub fn decode_pmic_rails(bytes: &[u8]) -> Option<PmicRails> {
    use rails_offset as offset;

    if bytes.len() != PMIC_RAILS_SIZE || bytes[offset::FLAGS] & PMIC_RAILS_PRESENT == 0 {
        return None;
    }
    let le16 = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);

    Some(PmicRails {
        vbus_mv: le16(offset::VBUS_MV),
        vsys_mv: le16(offset::VSYS_MV),
        vbusin: VbusInStatus::from_bits(bytes[offset::VBUSIN_STATUS]),
        charger_enabled: match bytes[offset::CHG_ENABLE] {
            PMIC_REG_UNKNOWN => None,
            raw => Some(raw & 0x01 != 0),
        },
        boost_off: (bytes[offset::FLAGS] & PMIC_RAILS_BOOST_KNOWN != 0)
            .then_some(bytes[offset::FLAGS] & PMIC_RAILS_BOOST_OFF != 0),
        charge_reduced: bytes[offset::FLAGS] & PMIC_RAILS_CHARGE_REDUCED != 0,
    })
}

/// What a board says both its PMICs are seeing.
///
/// A half is `None` when it has no rails to report, which covers the three
/// cases that look alike from here: firmware that predates the block, a half
/// that has not measured yet, and a right half the left could not reach.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PmicRailsReport {
    pub version: u8,
    pub left: Option<PmicRails>,
    pub right: Option<PmicRails>,
}

/// Decode a rails payload, the bytes after the `EVT_PMIC_RAILS` opcode.
///
/// Same two rules as [`parse_telemetry`]: a format version this host was not
/// written against is refused whole, and the `len` byte decides the rest. A
/// block is whole or it is nothing — see [`decode_pmic_rails`].
pub fn parse_pmic_rails_report(params: &[u8]) -> Option<PmicRailsReport> {
    use rails_report_offset as at;

    let version = *params.get(at::VERSION)?;
    if version != PMIC_RAILS_REPORT_VERSION {
        return None;
    }
    let len = *params.get(at::LEN)? as usize;
    let block = |at: usize| {
        (at + PMIC_RAILS_SIZE <= len + 2)
            .then(|| params.get(at..at + PMIC_RAILS_SIZE))
            .flatten()
            .and_then(decode_pmic_rails)
    };

    Some(PmicRailsReport {
        version,
        left: block(at::LEFT),
        right: block(at::RIGHT),
    })
}

/// One half's detail and where it came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HalfDetail {
    pub source: Option<Source>,
    pub detail: BatteryDetail,
}

/// What a board says its cells and chargers are doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChargeTelemetry {
    pub version: u8,
    pub left: HalfDetail,
    pub right: HalfDetail,
}

/// `EVT_DIAG` page format this host understands. A board answering 0 has the
/// command but no diagnostics.
pub const DIAG_PAGE_VERSION: u8 = 1;
/// Layout version of the STM32 and left nRF records this host understands.
pub const DIAG_RECORD_VERSION: u8 = 1;
/// `EVT_DIAG` payload size: the bytes after the opcode.
pub const DIAG_PAGE_SIZE: usize = 31;
/// Record bytes carried per page.
pub const DIAG_PAGE_DATA: usize = 24;
/// `CMD_GET_DIAG` flag: re-fetch both left nRF records from the nRF now.
pub const DIAG_FETCH: u8 = 0x01;
/// `DiagPage::age_ms` for a page 65.5 s old or older, or of unknown age.
pub const DIAG_AGE_UNKNOWN: u16 = 0xFFFF;
/// A record's age field that never started counting.
pub const DIAG_NEVER: u32 = 0xFFFF_FFFF;
/// STM32 record size, layout version 1.
pub const STM32_DIAG_SIZE: usize = 48;
/// STM32 record size with the left nRF rescue counters appended.
pub const STM32_DIAG_SIZE_RESCUE: usize = 50;
/// Left nRF record size, layout version 1.
pub const LEFT_DIAG_SIZE: usize = 90;

/// Byte offsets inside the `EVT_DIAG` payload.
mod diag_page_offset {
    pub const VERSION: usize = 0;
    pub const RECORD: usize = 1;
    pub const PAGE: usize = 2;
    pub const STATUS: usize = 3;
    pub const TOTAL_LEN: usize = 4;
    pub const AGE_MS: usize = 5;
    pub const DATA: usize = 7;
}

/// The four records `CMD_GET_DIAG` serves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DiagRecord {
    Stm32 = 0,
    Stm32Previous = 1,
    Left = 2,
    LeftPrevious = 3,
}

impl DiagRecord {
    pub const ALL: [DiagRecord; 4] = [
        DiagRecord::Stm32,
        DiagRecord::Stm32Previous,
        DiagRecord::Left,
        DiagRecord::LeftPrevious,
    ];

    pub fn id(self) -> u8 {
        self as u8
    }

    /// Records the STM32 caches from the left nRF.
    pub fn is_left(self) -> bool {
        matches!(self, DiagRecord::Left | DiagRecord::LeftPrevious)
    }
}

/// Request params for `CMD_GET_DIAG`.
pub fn diag_request(record: DiagRecord, page: u8, flags: u8) -> [u8; 3] {
    [record.id(), page, flags]
}

/// Whether the board had the page asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagStatus {
    Ok,
    /// No such record, or never received from the nRF.
    NoRecord,
    /// Unknown record, or a page past the end.
    OutOfRange,
    Other(u8),
}

impl DiagStatus {
    fn from_wire(raw: u8) -> Self {
        match raw {
            0 => DiagStatus::Ok,
            1 => DiagStatus::NoRecord,
            2 => DiagStatus::OutOfRange,
            other => DiagStatus::Other(other),
        }
    }
}

/// One `EVT_DIAG` answer: a slice of a record and its header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagPage {
    pub version: u8,
    pub record: u8,
    pub page: u8,
    pub status: DiagStatus,
    /// Record size in bytes; 0 unless `status` is `Ok`.
    pub total_len: u8,
    /// Left records: ms since the page arrived from the nRF. 0 for STM32.
    pub age_ms: u16,
    pub data: [u8; DIAG_PAGE_DATA],
}

/// Decode an `EVT_DIAG` payload, the bytes after the opcode. `None` when short.
pub fn parse_diag_page(params: &[u8]) -> Option<DiagPage> {
    use diag_page_offset as at;

    let params = params.get(..DIAG_PAGE_SIZE)?;
    let mut data = [0u8; DIAG_PAGE_DATA];
    data.copy_from_slice(&params[at::DATA..at::DATA + DIAG_PAGE_DATA]);
    Some(DiagPage {
        version: params[at::VERSION],
        record: params[at::RECORD],
        page: params[at::PAGE],
        status: DiagStatus::from_wire(params[at::STATUS]),
        total_len: params[at::TOTAL_LEN],
        age_ms: le16(params, at::AGE_MS),
        data,
    })
}

/// Pages a record of `total_len` bytes spans.
pub fn diag_page_count(total_len: u8) -> u8 {
    total_len.div_ceil(DIAG_PAGE_DATA as u8)
}

/// Joins a record's pages, given in order from page 0, into its bytes.
///
/// `None` unless every page is `Ok`, of the same record and size, and all of
/// them are there.
pub fn assemble_diag_record(pages: &[DiagPage]) -> Option<Vec<u8>> {
    let first = pages.first()?;
    if pages.len() != usize::from(diag_page_count(first.total_len).max(1)) {
        return None;
    }
    let consistent = pages.iter().enumerate().all(|(index, page)| {
        page.status == DiagStatus::Ok
            && usize::from(page.page) == index
            && page.record == first.record
            && page.total_len == first.total_len
    });
    if !consistent {
        return None;
    }
    let mut bytes: Vec<u8> = pages.iter().flat_map(|page| page.data).collect();
    bytes.truncate(usize::from(first.total_len));
    Some(bytes)
}

fn le16(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

fn le32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

/// The first `size` bytes of a record whose version this host knows.
///
/// A record shorter than `size`, by its `len` byte or by the bytes present, is
/// refused whole; a longer one decodes its known prefix.
fn diag_record(bytes: &[u8], size: usize) -> Option<&[u8]> {
    let version = *bytes.first()?;
    let len = usize::from(*bytes.get(1)?);
    if version != DIAG_RECORD_VERSION || len + 2 < size {
        return None;
    }
    bytes.get(..size)
}

// STM32 record `flags` bits.
pub const STM32_DIAG_LINK_UP: u8 = 0x01;
pub const STM32_DIAG_NRF_VBUS: u8 = 0x02;
pub const STM32_DIAG_USB_HOST: u8 = 0x04;
pub const STM32_DIAG_BLE_HOST: u8 = 0x08;
pub const STM32_DIAG_TRRS: u8 = 0x10;
pub const STM32_DIAG_RIGHT_LEDS: u8 = 0x20;
pub const STM32_DIAG_VBUS_SEEN: u8 = 0x40;

// STM32 record `reset_flags` bits (RCC_CSR[31:24]).
pub const STM32_RESET_IWDG: u8 = 0x20;
pub const STM32_RESET_WWDG: u8 = 0x40;

const STM32_RESET_CAUSES: &[(u8, &str)] = &[
    (0x02, "option-byte load"),
    (0x04, "reset pin"),
    (0x08, "power-on"),
    (0x10, "software reset"),
    (STM32_RESET_IWDG, "watchdog"),
    (STM32_RESET_WWDG, "window watchdog"),
    (0x80, "low-power reset"),
];

/// Names of the causes set in an STM32 record's `reset_flags`.
pub fn stm32_reset_causes(flags: u8) -> Vec<&'static str> {
    bit_names(flags, STM32_RESET_CAUSES)
}

/// The STM32's own record of a boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Stm32Diag {
    pub boot_count: u16,
    pub uptime_s: u32,
    pub reset_flags: u8,
    pub flags: u8,
    /// 0 pending, 1 Bluetooth, 2 USB.
    pub transport: u8,
    /// 0 active, otherwise asleep or staged.
    pub idle_step: u8,
    /// 0 absent, 1 OK, 2 version mismatch, 0xFF unknown.
    pub right_link: u8,
    /// 0 waiting for a verdict, 1 ok, 2 critical battery, 3 done/shipping.
    pub power_gate: u8,
    pub link_ups: u16,
    pub sd_overruns: u16,
    pub sd_framing: u16,
    pub sd_noise: u16,
    pub sd_parity: u16,
    pub sd_queue_full: u16,
    pub parser_resyncs: u16,
    pub crc_errors: u16,
    pub frame_aborts: u16,
    pub frames_rx: u32,
    pub frames_tx: u32,
    /// `DIAG_NEVER` when no frame has arrived.
    pub rx_age_ms: u32,
    pub max_rx_gap_ms: u32,
    /// MCUboot probes sent to a silent left nRF; saturates. `None` from
    /// firmware whose record stops at [`STM32_DIAG_SIZE`].
    pub left_probes: Option<u8>,
    /// Left nRF resets after MCUboot answered a probe; saturates.
    pub left_resets: Option<u8>,
}

/// Decode an STM32 record (see [`diag_record`] for the length rule).
pub fn parse_stm32_diag(bytes: &[u8]) -> Option<Stm32Diag> {
    let b = diag_record(bytes, STM32_DIAG_SIZE)?;
    // The rescue counters, when the record's `len` and its bytes both reach them.
    let rescue = diag_record(bytes, STM32_DIAG_SIZE_RESCUE);
    Some(Stm32Diag {
        boot_count: le16(b, 2),
        uptime_s: le32(b, 4),
        reset_flags: b[8],
        flags: b[9],
        transport: b[10],
        idle_step: b[11],
        right_link: b[12],
        power_gate: b[13],
        link_ups: le16(b, 14),
        sd_overruns: le16(b, 16),
        sd_framing: le16(b, 18),
        sd_noise: le16(b, 20),
        sd_parity: le16(b, 22),
        sd_queue_full: le16(b, 24),
        parser_resyncs: le16(b, 26),
        crc_errors: le16(b, 28),
        frame_aborts: le16(b, 30),
        frames_rx: le32(b, 32),
        frames_tx: le32(b, 36),
        rx_age_ms: le32(b, 40),
        max_rx_gap_ms: le32(b, 44),
        left_probes: rescue.map(|r| r[48]),
        left_resets: rescue.map(|r| r[49]),
    })
}

// Left nRF record `flags` bits.
pub const LEFT_DIAG_STM32_LINK: u8 = 0x01;
pub const LEFT_DIAG_STM32_SLEEP: u8 = 0x02;
pub const LEFT_DIAG_STM32_USB: u8 = 0x04;
pub const LEFT_DIAG_STM32_TRRS: u8 = 0x08;
pub const LEFT_DIAG_BLE_HOST: u8 = 0x10;
pub const LEFT_DIAG_UART_RX_OFF: u8 = 0x20;
pub const LEFT_DIAG_VBUS: u8 = 0x40;

// nRF54L15 RESET.RESETREAS bits.
pub const NRF_RESET_DOG0: u32 = 1 << 1;
pub const NRF_RESET_DOG1: u32 = 1 << 2;
pub const NRF_RESET_SREQ: u32 = 1 << 6;
pub const NRF_RESET_LOCKUP: u32 = 1 << 7;

const NRF_RESET_CAUSES: &[(u32, &str)] = &[
    (1 << 0, "reset pin"),
    (NRF_RESET_DOG0, "watchdog 0"),
    (NRF_RESET_DOG1, "watchdog 1"),
    (1 << 3, "debugger soft reset"),
    (1 << 4, "debugger hard reset"),
    (1 << 5, "debugger reset pin"),
    (NRF_RESET_SREQ, "software reset"),
    (NRF_RESET_LOCKUP, "CPU lockup"),
    (1 << 8, "wake from System OFF"),
    (1 << 9, "LPCOMP wake"),
    (1 << 10, "debug interface wake"),
    (1 << 11, "GRTC wake"),
    (1 << 12, "NFC wake"),
    (1 << 13, "tamper detector"),
];

/// Names of the causes set in a RESETREAS word; 0 is power-on or brownout.
pub fn nrf_reset_causes(resetreas: u32) -> Vec<&'static str> {
    if resetreas == 0 {
        return vec!["power-on or brownout"];
    }
    NRF_RESET_CAUSES
        .iter()
        .filter(|(bit, _)| resetreas & bit != 0)
        .map(|(_, name)| *name)
        .collect()
}

/// One left nRF thread's wait: current and longest this boot, in ms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ThreadStall {
    pub stall_ms: u32,
    pub max_stall_ms: u32,
}

/// Names of `LeftDiag::threads`, in order.
pub const LEFT_DIAG_THREADS: [&str; 4] = ["main loop", "HID queue", "system workqueue", "UART RX"];

/// The left nRF's record of a boot. Its u16 counters wrap.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LeftDiag {
    pub boot_count: u16,
    pub uptime_s: u32,
    pub resetreas: u32,
    pub flags: u8,
    /// 0xFF none.
    pub ble_slot: u8,
    /// 0 absent, 1 OK, 2 mismatch.
    pub esb_link: u8,
    pub stm32_idle_step: u8,
    /// 1.25 ms units, 0 = no host.
    pub conn_interval: u16,
    pub conn_latency: u16,
    /// 10 ms units.
    pub conn_timeout: u16,
    pub uart_frames: u16,
    pub uart_bad_frames: u16,
    pub uart_rx_errors: u16,
    pub uart_rx_overruns: u16,
    pub uart_rx_line_errors: u16,
    pub uart_ring_drops: u16,
    pub uart_rx_restarts: u16,
    pub kbd_reports: u16,
    pub kbd_dropped: u16,
    pub ble_send_errors: u16,
    pub notify_errors: u16,
    pub esb_hops: u16,
    pub esb_timeouts: u16,
    pub hid_send_max_ms: u16,
    /// `DIAG_NEVER` when no byte has arrived.
    pub uart_rx_age_ms: u32,
    /// `DIAG_NEVER` when no packet has arrived.
    pub esb_rx_age_ms: u32,
    /// In `LEFT_DIAG_THREADS` order.
    pub threads: [ThreadStall; 4],
}

/// Decode a left nRF record (see [`diag_record`] for the length rule).
pub fn parse_left_diag(bytes: &[u8]) -> Option<LeftDiag> {
    let b = diag_record(bytes, LEFT_DIAG_SIZE)?;
    let thread = |index: usize| ThreadStall {
        stall_ms: le32(b, 58 + index * 8),
        max_stall_ms: le32(b, 62 + index * 8),
    };
    Some(LeftDiag {
        boot_count: le16(b, 2),
        uptime_s: le32(b, 4),
        resetreas: le32(b, 8),
        flags: b[12],
        ble_slot: b[13],
        esb_link: b[14],
        stm32_idle_step: b[15],
        conn_interval: le16(b, 16),
        conn_latency: le16(b, 18),
        conn_timeout: le16(b, 20),
        uart_frames: le16(b, 22),
        uart_bad_frames: le16(b, 24),
        uart_rx_errors: le16(b, 26),
        uart_rx_overruns: le16(b, 28),
        uart_rx_line_errors: le16(b, 30),
        uart_ring_drops: le16(b, 32),
        uart_rx_restarts: le16(b, 34),
        kbd_reports: le16(b, 36),
        kbd_dropped: le16(b, 38),
        ble_send_errors: le16(b, 40),
        notify_errors: le16(b, 42),
        esb_hops: le16(b, 44),
        esb_timeouts: le16(b, 46),
        hid_send_max_ms: le16(b, 48),
        uart_rx_age_ms: le32(b, 50),
        esb_rx_age_ms: le32(b, 54),
        threads: [thread(0), thread(1), thread(2), thread(3)],
    })
}

/// First Oryx protocol version that answers `CMD_GET_RADIO_INFO`.
pub const RADIO_INFO_PROTOCOL: u8 = 0x06;
/// `EVT_RADIO_INFO` payload format this host understands.
pub const RADIO_INFO_FORMAT: u8 = 1;
/// `EVT_RADIO_INFO` payload size: the bytes after the opcode.
pub const RADIO_INFO_SIZE: usize = 26;
/// Bytes of an image hash the answer carries.
pub const RADIO_HASH_PREFIX: usize = 8;

// `EVT_RADIO_INFO` flags.
pub const RADIO_LEFT_VALID: u8 = 0x01;
pub const RADIO_LEFT_CONFIRMED: u8 = 0x02;
pub const RADIO_RIGHT_VALID: u8 = 0x04;
pub const RADIO_RIGHT_CONFIRMED: u8 = 0x08;

/// Byte offsets inside the `EVT_RADIO_INFO` payload.
mod radio_info_offset {
    pub const FORMAT: usize = 0;
    pub const FLAGS: usize = 1;
    pub const MAJOR: usize = 2;
    pub const MINOR: usize = 3;
    pub const REVISION: usize = 4;
    pub const BUILD: usize = 6;
    pub const LEFT_HASH: usize = 10;
    pub const RIGHT_TAG: usize = 18;
}

/// Whether a board at Oryx protocol `version` answers `CMD_GET_RADIO_INFO`.
pub fn has_radio_info(version: u8) -> bool {
    version >= RADIO_INFO_PROTOCOL
}

/// An MCUboot image version.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RadioVersion {
    pub major: u8,
    pub minor: u8,
    pub revision: u16,
    pub build: u32,
}

impl std::fmt::Display for RadioVersion {
    /// `major.minor.revision`; the build number is left out.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.revision)
    }
}

/// The running left image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeftRadio {
    pub version: RadioVersion,
    /// First bytes of the image hash.
    pub hash: [u8; RADIO_HASH_PREFIX],
    pub confirmed: bool,
}

/// The running right image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RightRadio {
    /// First bytes of the image hash.
    pub tag: [u8; RADIO_HASH_PREFIX],
    pub confirmed: bool,
}

/// What `EVT_RADIO_INFO` says about each half. `None` for a half the board
/// could not tell about.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RadioInfo {
    pub left: Option<LeftRadio>,
    pub right: Option<RightRadio>,
}

impl RadioInfo {
    /// Whether the board told anything at all.
    pub fn is_known(&self) -> bool {
        self.left.is_some() || self.right.is_some()
    }
}

/// Decode an `EVT_RADIO_INFO` payload, the bytes after the opcode. `None` when
/// short; an unknown format reads as nothing known.
pub fn parse_radio_info(params: &[u8]) -> Option<RadioInfo> {
    use radio_info_offset as at;

    let params = params.get(..RADIO_INFO_SIZE)?;
    if params[at::FORMAT] != RADIO_INFO_FORMAT {
        return Some(RadioInfo::default());
    }
    let flags = params[at::FLAGS];
    let prefix = |at: usize| {
        let mut hash = [0u8; RADIO_HASH_PREFIX];
        hash.copy_from_slice(&params[at..at + RADIO_HASH_PREFIX]);
        hash
    };
    let left = (flags & RADIO_LEFT_VALID != 0).then(|| LeftRadio {
        version: RadioVersion {
            major: params[at::MAJOR],
            minor: params[at::MINOR],
            revision: le16(params, at::REVISION),
            build: le32(params, at::BUILD),
        },
        hash: prefix(at::LEFT_HASH),
        confirmed: flags & RADIO_LEFT_CONFIRMED != 0,
    });
    let right = (flags & RADIO_RIGHT_VALID != 0).then(|| RightRadio {
        tag: prefix(at::RIGHT_TAG),
        confirmed: flags & RADIO_RIGHT_CONFIRMED != 0,
    });
    Some(RadioInfo { left, right })
}

/// Host slots a board keeps; `CMD_GET_HOST_SLOTS` is asked at most this often.
pub const HOST_SLOTS_MAX: u8 = 5;
/// `EVT_HOST_SLOTS` payload size: the bytes after the opcode.
pub const HOST_SLOT_SIZE: usize = 10;

// `EVT_HOST_SLOTS` flags.
pub const HOST_SLOT_PAIRED: u8 = 0x01;
pub const HOST_SLOT_CONNECTED: u8 = 0x02;

/// Byte offsets inside the `EVT_HOST_SLOTS` payload.
mod host_slot_offset {
    pub const SLOT: usize = 0;
    pub const SLOT_COUNT: usize = 1;
    pub const FLAGS: usize = 2;
    pub const ADDR_TYPE: usize = 3;
    pub const ADDR: usize = 4;
}

/// One Bluetooth host slot: the board's own address on it, and whether a host
/// is paired and connected there.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HostSlot {
    /// The slot, counted from zero.
    pub slot: u8,
    /// How many slots the board keeps. Zero when the board knows nothing.
    pub slot_count: u8,
    pub paired: bool,
    pub connected: bool,
    /// Most significant byte first, as `BleIdentity::address`. `None` when
    /// the board does not know it.
    pub address: Option<String>,
    pub random: bool,
}

/// Decode an `EVT_HOST_SLOTS` payload, the bytes after the opcode. `None` when
/// short.
pub fn parse_host_slot(params: &[u8]) -> Option<HostSlot> {
    use host_slot_offset as at;

    let params = params.get(..HOST_SLOT_SIZE)?;
    let flags = params[at::FLAGS];
    let addr_type = params[at::ADDR_TYPE];
    let mut addr = [0u8; BLE_ADDR_LEN];
    addr.copy_from_slice(&params[at::ADDR..at::ADDR + BLE_ADDR_LEN]);
    Some(HostSlot {
        slot: params[at::SLOT],
        slot_count: params[at::SLOT_COUNT],
        paired: flags & HOST_SLOT_PAIRED != 0,
        connected: flags & HOST_SLOT_CONNECTED != 0,
        address: (addr_type != TELEMETRY_BLE_ADDR_UNKNOWN).then(|| ble_address(&addr)),
        random: addr_type == TELEMETRY_BLE_ADDR_RANDOM,
    })
}

// `CMD_GET_BLOB` blobs.
pub const BLOB_FREEZE: u8 = 0;
pub const BLOB_EVENTS: u8 = 1;
/// Data bytes per `EVT_BLOB`.
pub const BLOB_CHUNK: usize = 24;
/// `EVT_BLOB` payload size: the bytes after the opcode.
pub const BLOB_SIZE: usize = 7 + BLOB_CHUNK;

// `EVT_BLOB` and `EVT_FREEZE_SEEN` status.
pub const BLOB_OK: u8 = 0;
pub const BLOB_NONE: u8 = 1;
pub const BLOB_BAD_OFFSET: u8 = 2;
pub const BLOB_TIMEOUT: u8 = 0x80;
pub const BLOB_NO_LINK: u8 = 0x81;
pub const BLOB_BUSY: u8 = 0x82;

/// Byte offsets inside the `EVT_BLOB` payload.
mod blob_offset {
    pub const WHICH: usize = 0;
    pub const STATUS: usize = 1;
    pub const TOTAL_LEN: usize = 2;
    pub const OFFSET: usize = 4;
    pub const LEN: usize = 6;
    pub const DATA: usize = 7;
}

/// One `EVT_BLOB` answer: `data` is the chunk at `offset`, `total_len` the
/// whole blob.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlobChunk {
    pub which: u8,
    pub status: u8,
    pub total_len: u16,
    pub offset: u16,
    pub data: Vec<u8>,
}

/// Params of `CMD_GET_BLOB`.
pub fn blob_request(which: u8, offset: u16) -> [u8; 3] {
    let [lo, hi] = offset.to_le_bytes();
    [which, lo, hi]
}

/// Decode an `EVT_BLOB` payload, the bytes after the opcode. `None` when
/// short or when `len` is past a chunk.
pub fn parse_blob_chunk(params: &[u8]) -> Option<BlobChunk> {
    use blob_offset as at;

    let params = params.get(..BLOB_SIZE)?;
    let len = usize::from(params[at::LEN]);
    if len > BLOB_CHUNK {
        return None;
    }
    Some(BlobChunk {
        which: params[at::WHICH],
        status: params[at::STATUS],
        total_len: le16(params, at::TOTAL_LEN),
        offset: le16(params, at::OFFSET),
        data: params[at::DATA..at::DATA + len].to_vec(),
    })
}

/// How a blob read ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BlobRead {
    Data(Vec<u8>),
    /// The left nRF has no such blob.
    None,
    /// The firmware does not know `CMD_GET_BLOB`, or never answered it.
    Unsupported,
    /// A chunk came back with this status (`BLOB_*`).
    Failed(u8),
    /// The chunks did not join up.
    Inconsistent,
}

/// Whole-blob restarts when `total_len` changes during a read.
const BLOB_RESTARTS: usize = 2;

/// Reads a blob chunk by chunk from offset 0, at most `limit` bytes of it.
/// `ask` returns the answer for one offset: `None` for no answer,
/// `Some(Err(()))` for an unknown-command error.
pub fn read_blob(
    which: u8,
    limit: usize,
    mut ask: impl FnMut(u16) -> Option<Result<BlobChunk, ()>>,
) -> BlobRead {
    'restart: for _ in 0..=BLOB_RESTARTS {
        let mut bytes: Vec<u8> = Vec::new();
        let mut total: Option<u16> = None;
        loop {
            let offset = bytes.len();
            if let Some(total) = total
                && offset >= usize::from(total).min(limit)
            {
                bytes.truncate(limit);
                return BlobRead::Data(bytes);
            }
            let Ok(at) = u16::try_from(offset) else {
                return BlobRead::Inconsistent;
            };
            let chunk = match ask(at) {
                None if offset == 0 => return BlobRead::Unsupported,
                None => return BlobRead::Failed(BLOB_TIMEOUT),
                Some(Err(())) => return BlobRead::Unsupported,
                Some(Ok(chunk)) => chunk,
            };
            if chunk.which != which || chunk.offset != at {
                return BlobRead::Inconsistent;
            }
            match chunk.status {
                BLOB_OK => {}
                BLOB_NONE if offset == 0 => return BlobRead::None,
                status => return BlobRead::Failed(status),
            }
            match total {
                None => total = Some(chunk.total_len),
                Some(total) if total != chunk.total_len => continue 'restart,
                Some(_) => {}
            }
            if chunk.data.is_empty() && offset < usize::from(chunk.total_len) {
                return BlobRead::Inconsistent;
            }
            bytes.extend_from_slice(&chunk.data);
        }
    }
    BlobRead::Inconsistent
}

/// "ZFRZ", little-endian: the freeze record's first four bytes.
pub const FREEZE_MAGIC: u32 = 0x5A52_465A;
/// Bytes in the freeze record's fixed header.
pub const FREEZE_HEADER_SIZE: usize = 36;

// `FreezeHeader::stalled` bits.
pub const FREEZE_STALL_SYSTEM_WQ: u8 = 0x01;
pub const FREEZE_STALL_HID_WQ: u8 = 0x02;
pub const FREEZE_STALL_SLOW_WQ: u8 = 0x04;
pub const FREEZE_STALL_MAIN: u8 = 0x08;

// `FreezeHeader::flags` bits.
pub const FREEZE_UNREAD: u8 = 0x01;
pub const FREEZE_RESTORED: u8 = 0x04;

/// What the freeze record caught.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreezeKind {
    /// A task queue stopped making progress.
    Freeze,
    /// A fatal error.
    Fatal,
    Other(u8),
}

/// The freeze record's fixed header; the body after it is not decoded here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FreezeHeader {
    pub layout: u16,
    pub total_len: u16,
    pub kind: FreezeKind,
    /// `FREEZE_STALL_*` bits.
    pub stalled: u8,
    /// `FREEZE_*` flag bits.
    pub flags: u8,
    /// Freezes since this record was taken, this one included.
    pub freezes: u8,
    pub uptime_ms: u32,
    /// Zephyr fatal reason; 0 for a freeze.
    pub fatal_reason: u32,
    pub version: [u8; 3],
    pub build: u32,
    pub hash: [u8; 8],
}

impl FreezeHeader {
    pub fn unread(&self) -> bool {
        self.flags & FREEZE_UNREAD != 0
    }

    /// Names of the stalled queues, in bit order.
    pub fn stalled_names(&self) -> Vec<&'static str> {
        freeze_bit_names(
            self.stalled,
            &[
                (FREEZE_STALL_SYSTEM_WQ, "system workqueue"),
                (FREEZE_STALL_HID_WQ, "hid_wq"),
                (FREEZE_STALL_SLOW_WQ, "slow_wq"),
                (FREEZE_STALL_MAIN, "main loop"),
            ],
        )
    }

    /// Names of the set flags, in bit order.
    pub fn flag_names(&self) -> Vec<&'static str> {
        freeze_bit_names(
            self.flags,
            &[
                (FREEZE_UNREAD, "unread"),
                (FREEZE_RESTORED, "restored from flash"),
            ],
        )
    }
}

fn freeze_bit_names(bits: u8, names: &[(u8, &'static str)]) -> Vec<&'static str> {
    names
        .iter()
        .filter(|(bit, _)| bits & bit != 0)
        .map(|&(_, name)| name)
        .collect()
}

/// Decode the freeze record's fixed header from its first bytes. `None` when
/// short or without the magic.
pub fn parse_freeze_header(bytes: &[u8]) -> Option<FreezeHeader> {
    let bytes = bytes.get(..FREEZE_HEADER_SIZE)?;
    if le32(bytes, 0) != FREEZE_MAGIC {
        return None;
    }
    let mut hash = [0u8; 8];
    hash.copy_from_slice(&bytes[28..36]);
    Some(FreezeHeader {
        layout: le16(bytes, 4),
        total_len: le16(bytes, 6),
        kind: match bytes[8] {
            1 => FreezeKind::Freeze,
            2 => FreezeKind::Fatal,
            other => FreezeKind::Other(other),
        },
        stalled: bytes[9],
        flags: bytes[10],
        freezes: bytes[11],
        uptime_ms: le32(bytes, 12),
        fatal_reason: le32(bytes, 16),
        version: [bytes[20], bytes[21], bytes[22]],
        build: le32(bytes, 24),
        hash,
    })
}

/// Events from a connected keyboard's raw HID interface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HidEvent {
    /// Device reported its layout: serial is "<layoutId>/<revisionId>".
    FirmwareVersion {
        layout_id: String,
        revision_id: String,
    },
    /// Feature-gating protocol version (0x03 = smart layer, 0x04 = RGB,
    /// 0x06 = radio info).
    ProtocolVersion(u8),
    /// The active layer changed.
    LayerChanged(u8),
    /// A physical key went down or up, in matrix (column, row) coordinates.
    Key {
        column: u8,
        row: u8,
        pressed: bool,
    },
    /// The device opened a pairing handshake; answered automatically.
    PairingInit,
    /// The device is collecting the pairing key from the user.
    PairingKeyInput,
    PairingSuccess,
    PairingFail,
    /// The user asked to toggle smart layers on or off.
    ToggleSmartLayer,
    /// The user asked to re-evaluate the smart layer binding.
    TriggerSmartLayer,
    /// The board answered `CMD_GET_TELEMETRY`.
    Telemetry(Telemetry),
    /// The board answered `CMD_GET_CHARGE_TELEMETRY`.
    ChargeTelemetry(ChargeTelemetry),
    /// The board answered `CMD_GET_PMIC_RAILS`.
    PmicRails(PmicRailsReport),
    /// The board answered `CMD_GET_DIAG` with one page of a record.
    Diag(DiagPage),
    /// The board answered `CMD_GET_RADIO_INFO`.
    RadioInfo(RadioInfo),
    /// The board answered `CMD_GET_HOST_SLOTS` for one slot.
    HostSlot(HostSlot),
    /// The board answered `CMD_GET_BLOB` with one chunk.
    Blob(BlobChunk),
    /// The board answered `CMD_FREEZE_SEEN` with this status.
    FreezeSeen(u8),
    /// The board answered `EVT_ERROR` with this code.
    Error(u8),
    /// The read loop ended (device unplugged or errored).
    Disconnected,
}

/// An outgoing command: opcode plus parameter bytes.
pub type Packet = (u8, Vec<u8>);

/// Build the outgoing report: opcode, then zero-padded params.
pub fn encode_packet(cmd: u8, params: &[u8]) -> [u8; REPORT_SIZE] {
    let mut buf = [0u8; REPORT_SIZE];
    buf[0] = cmd;
    let n = params.len().min(REPORT_SIZE - 1);
    buf[1..1 + n].copy_from_slice(&params[..n]);
    buf
}

/// Split an incoming report into (event, params). Params run until STOP_BIT.
/// Quirk: some firmware sends STOP_BIT as the first byte to announce the
/// protocol version.
pub fn parse_packet(data: &[u8]) -> Option<(u8, &[u8])> {
    let &cmd = data.first()?;
    if cmd == STOP_BIT {
        return Some((EVT_GET_PROTOCOL_VERSION, &data[1..data.len().min(2)]));
    }
    if matches!(
        cmd,
        EVT_TELEMETRY
            | EVT_CHARGE_TELEMETRY
            | EVT_PMIC_RAILS
            | EVT_DIAG
            | EVT_RADIO_INFO
            | EVT_HOST_SLOTS
            | EVT_BLOB
            | EVT_FREEZE_SEEN
            | EVT_ERROR
    ) {
        // Binary payload carrying its own length: 0xFE in it is a value, not
        // a terminator, and truncating there would eat a 254% battery or a
        // reserved byte the board deliberately set.
        return Some((cmd, &data[1..]));
    }
    let end = data
        .iter()
        .position(|&b| b == STOP_BIT)
        .unwrap_or(data.len());
    Some((cmd, &data[1..end]))
}

/// The byte at `offset`, if the payload both reaches it and claims it.
///
/// Two ways it may not: the report was short, or the board's `len` stops
/// before this field because its firmware predates it. Either way the answer
/// is "unknown", which is what `None` says here — never a zero.
fn telemetry_field(params: &[u8], offset: usize) -> Option<u8> {
    let len = *params.get(telemetry_offset::LEN)? as usize;
    (offset < len + 2).then(|| params.get(offset).copied())?
}

/// A little-endian `u16` at `offset`, if the payload claims both of its bytes.
///
/// Both or neither: half a number is not a smaller number, it is a wrong one,
/// and a board whose `len` stops between them has not sent this field.
fn telemetry_field_le16(params: &[u8], offset: usize) -> Option<u16> {
    let lo = telemetry_field(params, offset)?;
    let hi = telemetry_field(params, offset + 1)?;
    Some(u16::from_le_bytes([lo, hi]))
}

fn telemetry_charging(params: &[u8]) -> Charging {
    use telemetry_offset as at;

    Charging {
        budget_ma: telemetry_field_le16(params, at::POWER_BUDGET_MA),
        input_limit_ma: telemetry_field_le16(params, at::CHARGE_LIMIT_MA)
            .filter(|&ma| ma != TELEMETRY_CURRENT_UNKNOWN),
        charge_current_ma: telemetry_field_le16(params, at::CHARGE_CURRENT_MA)
            .filter(|&ma| ma != TELEMETRY_CURRENT_UNKNOWN),
        state: telemetry_field(params, at::CHARGE_STATE).and_then(ChargeState::from_raw),
    }
}

/// The active Bluetooth identity, or `None`.
///
/// All nine bytes or nothing, for the same reason half a `u16` is nothing: an
/// address missing its last byte is a different address, not a shorter one.
fn telemetry_ble(params: &[u8]) -> Option<BleIdentity> {
    use telemetry_offset as at;

    let addr_type = telemetry_field(params, at::BLE_ADDR_TYPE)?;
    let mut addr = [0u8; BLE_ADDR_LEN];
    for (i, byte) in addr.iter_mut().enumerate() {
        *byte = telemetry_field(params, at::BLE_ADDR + i)?;
    }
    let slot = telemetry_field(params, at::BLE_SLOT)?;
    let slot_count = telemetry_field(params, at::BLE_SLOT_COUNT)?;

    if addr_type == TELEMETRY_BLE_ADDR_UNKNOWN {
        return None;
    }

    Some(BleIdentity {
        address: ble_address(&addr),
        random: addr_type == TELEMETRY_BLE_ADDR_RANDOM,
        slot,
        slot_count,
    })
}

/// `FE:DB:E9:A8:B1:36` from an address as it travels, least significant byte
/// first.
fn ble_address(addr: &[u8; BLE_ADDR_LEN]) -> String {
    addr.iter()
        .rev()
        .map(|byte| format!("{byte:02X}"))
        .collect::<Vec<_>>()
        .join(":")
}

fn telemetry_battery(params: &[u8], level: usize, flags: usize, source: usize) -> BatteryReading {
    let raw_flags = telemetry_field(params, flags).filter(|f| f & TELEMETRY_BAT_KNOWN != 0);
    BatteryReading {
        level: telemetry_field(params, level).filter(|&l| l != TELEMETRY_LEVEL_UNKNOWN && l <= 100),
        charging: raw_flags.map(|f| f & TELEMETRY_BAT_CHARGING != 0),
        usb: raw_flags.map(|f| f & TELEMETRY_BAT_USB != 0),
        source: telemetry_field(params, source).map(Source::from_raw),
    }
}

/// Decode a telemetry payload, the bytes after the `EVT_TELEMETRY` opcode.
///
/// Returns `None` for a payload whose format version is not the one this host
/// was written against — see [`TELEMETRY_VERSION`]. Growth is the `len` byte's
/// job, and this handles it in both directions: fields past what the board
/// sent come back unknown, and fields past what this host knows are ignored.
pub fn parse_telemetry(params: &[u8]) -> Option<Telemetry> {
    use telemetry_offset as at;

    let version = *params.get(at::VERSION)?;
    if version != TELEMETRY_VERSION {
        return None;
    }
    // Both header bytes or nothing: a payload that cannot say how far it
    // reaches is not one we can read fields out of.
    params.get(at::LEN)?;

    Some(Telemetry {
        version,
        left: telemetry_battery(
            params,
            at::BATTERY_LEFT,
            at::BATTERY_LEFT_FLAGS,
            at::BATTERY_LEFT_SOURCE,
        ),
        right: telemetry_battery(
            params,
            at::BATTERY_RIGHT,
            at::BATTERY_RIGHT_FLAGS,
            at::BATTERY_RIGHT_SOURCE,
        ),
        charging: telemetry_charging(params),
        nav_build: telemetry_field(params, at::NAV_BUILD).and_then(Nav::from_mask),
        nav_left: telemetry_field(params, at::NAV_LEFT).and_then(Nav::from_mask),
        nav_right: telemetry_field(params, at::NAV_RIGHT).and_then(Nav::from_mask),
        nav_bus: telemetry_field(params, at::NAV_BUS).and_then(Nav::from_mask),
        trrs: telemetry_field(params, at::TRRS).and_then(|raw| match raw {
            TELEMETRY_TRRS_UNPLUGGED => Some(false),
            TELEMETRY_TRRS_PLUGGED => Some(true),
            _ => None,
        }),
        esb_link: telemetry_field(params, at::ESB_LINK).and_then(EsbLink::from_raw),
        charge_issue: telemetry_field(params, at::CHARGE_ISSUE).and_then(ChargeIssue::from_raw),
        ble: telemetry_ble(params),
    })
}

/// Decode one 13-byte detail block, reading it through `field`.
///
/// The block travels two ways — inside a HID payload, where a board's `len` may
/// stop part way through it, and as raw bytes over Bluetooth, where it is
/// either all there or not there at all — so what a byte's absence means is the
/// caller's to define and this only says whether it got one.
/// Whether a minutes field says anything: neither the "no estimate" sentinel
/// nor the saturation cap does.
fn known_minutes(minutes: &u16) -> bool {
    !matches!(*minutes, DETAIL_MIN_UNKNOWN | DETAIL_MIN_SATURATED)
}

fn battery_detail(field: impl Fn(usize) -> Option<u8>) -> BatteryDetail {
    use detail_offset as offset;

    // Both bytes or neither, as everywhere else: half a number is a wrong one.
    let le16 =
        |at: usize| -> Option<u16> { Some(u16::from_le_bytes([field(at)?, field(at + 1)?])) };
    let reg = |at: usize| field(at).filter(|&raw| raw != DETAIL_REG_UNKNOWN);

    BatteryDetail {
        voltage_mv: le16(offset::VOLTAGE_MV).filter(|&mv| mv != DETAIL_MV_UNKNOWN),
        current_ma: le16(offset::CURRENT_MA)
            .map(|raw| raw as i16)
            .filter(|&ma| ma != DETAIL_I16_UNKNOWN),
        temp_cdeg: le16(offset::TEMP_CDEG)
            .map(|raw| raw as i16)
            .filter(|&cdeg| cdeg != DETAIL_I16_UNKNOWN),
        tte_min: le16(offset::TTE_MIN).filter(known_minutes),
        ttf_min: le16(offset::TTF_MIN).filter(known_minutes),
        charger: reg(offset::CHG_STATUS).map(ChargerStatus::from_bits),
        // A fault is what latched; the sensor register only qualifies it, so a
        // sensor byte that did not arrive leaves the fault itself standing.
        fault: reg(offset::CHG_ERROR).map(|reason| ChargerFault {
            reason,
            sensor: reg(offset::CHG_SENSOR).unwrap_or(0),
        }),
    }
}

/// Decode a bare detail block: 13 bytes, all of them there.
///
/// What the board's Bluetooth read carries, twice over. A short slice leaves
/// the fields it does not reach unknown rather than inventing them.
pub fn decode_battery_detail(bytes: &[u8]) -> BatteryDetail {
    battery_detail(|offset| bytes.get(offset).copied())
}

fn half_detail(params: &[u8], source: usize, block: usize) -> HalfDetail {
    HalfDetail {
        source: telemetry_field(params, source).map(Source::from_raw),
        detail: battery_detail(|offset| telemetry_field(params, block + offset)),
    }
}

/// Decode a charge-telemetry payload, the bytes after the
/// `EVT_CHARGE_TELEMETRY` opcode.
///
/// Same two rules as [`parse_telemetry`]: a format version this host was not
/// written against is refused whole, and everything else is the `len` byte's
/// business — a board that stops short of a field has not sent it, and this
/// says unknown rather than reading whatever happens to be at that offset.
pub fn parse_charge_telemetry(params: &[u8]) -> Option<ChargeTelemetry> {
    use charge_offset as at;

    let version = *params.get(at::VERSION)?;
    if version != CHARGE_TELEMETRY_VERSION {
        return None;
    }
    params.get(at::LEN)?;

    Some(ChargeTelemetry {
        version,
        left: half_detail(params, at::LEFT_SOURCE, at::LEFT),
        right: half_detail(params, at::RIGHT_SOURCE, at::RIGHT),
    })
}

/// Decode a report into the event to emit, plus any packets the protocol
/// requires the host to send back.
pub fn process_packet(data: &[u8]) -> (Option<HidEvent>, Vec<Packet>) {
    let Some((evt, params)) = parse_packet(data) else {
        return (None, Vec::new());
    };

    let event = match evt {
        EVT_GET_FIRMWARE_VERSION => {
            let serial = String::from_utf8_lossy(params);
            let Some((layout, revision)) = serial.split_once('/') else {
                return (None, Vec::new());
            };
            // Continue the handshake: pair (zero key), then protocol version.
            return (
                Some(HidEvent::FirmwareVersion {
                    layout_id: layout.to_string(),
                    revision_id: revision.to_string(),
                }),
                vec![
                    (CMD_INIT_PAIRING, vec![0u8; 6]),
                    (CMD_GET_PROTOCOL_VERSION, Vec::new()),
                ],
            );
        }
        EVT_PAIRING_INIT => {
            // Echo the 6-byte pairing key back to validate.
            let replies = match params.get(..6) {
                Some(key) => vec![(CMD_VALIDATE_PAIRING, key.to_vec())],
                None => Vec::new(),
            };
            return (Some(HidEvent::PairingInit), replies);
        }
        EVT_PAIRING_KEY_INPUT => HidEvent::PairingKeyInput,
        EVT_PAIRING_FAIL => HidEvent::PairingFail,
        EVT_PAIRING_SUCCESS => HidEvent::PairingSuccess,
        EVT_LAYER_CHANGE => match params.first() {
            Some(&layer) => HidEvent::LayerChanged(layer),
            None => return (None, Vec::new()),
        },
        EVT_KEY_PRESSED | EVT_KEY_RELEASED => {
            if params.len() < 2 {
                return (None, Vec::new());
            }
            HidEvent::Key {
                column: params[0],
                row: params[1],
                pressed: evt == EVT_KEY_PRESSED,
            }
        }
        EVT_TOGGLE_SMARTLAYER => HidEvent::ToggleSmartLayer,
        EVT_TRIGGER_SMARTLAYER => HidEvent::TriggerSmartLayer,
        EVT_TELEMETRY => match parse_telemetry(params) {
            Some(telemetry) => HidEvent::Telemetry(telemetry),
            None => return (None, Vec::new()),
        },
        EVT_CHARGE_TELEMETRY => match parse_charge_telemetry(params) {
            Some(telemetry) => HidEvent::ChargeTelemetry(telemetry),
            None => return (None, Vec::new()),
        },
        EVT_PMIC_RAILS => match parse_pmic_rails_report(params) {
            Some(report) => HidEvent::PmicRails(report),
            None => return (None, Vec::new()),
        },
        EVT_DIAG => match parse_diag_page(params) {
            Some(page) => HidEvent::Diag(page),
            None => return (None, Vec::new()),
        },
        EVT_RADIO_INFO => match parse_radio_info(params) {
            Some(info) => HidEvent::RadioInfo(info),
            None => return (None, Vec::new()),
        },
        EVT_HOST_SLOTS => match parse_host_slot(params) {
            Some(slot) => HidEvent::HostSlot(slot),
            None => return (None, Vec::new()),
        },
        EVT_BLOB => match parse_blob_chunk(params) {
            Some(chunk) => HidEvent::Blob(chunk),
            None => return (None, Vec::new()),
        },
        EVT_FREEZE_SEEN => match params.first() {
            Some(&status) => HidEvent::FreezeSeen(status),
            None => return (None, Vec::new()),
        },
        EVT_ERROR => match params.first() {
            Some(&code) => HidEvent::Error(code),
            None => return (None, Vec::new()),
        },
        EVT_GET_PROTOCOL_VERSION => match params.first() {
            Some(&version) => HidEvent::ProtocolVersion(version),
            None => return (None, Vec::new()),
        },
        _ => return (None, Vec::new()),
    };

    (Some(event), Vec::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 32-byte incoming report from an opcode and params.
    fn report(cmd: u8, params: &[u8]) -> [u8; REPORT_SIZE] {
        let mut data = [0u8; REPORT_SIZE];
        data[0] = cmd;
        data[1..1 + params.len()].copy_from_slice(params);
        data[1 + params.len()] = STOP_BIT;
        data
    }

    #[test]
    fn encode_frames_command_first() {
        let packet = encode_packet(CMD_SET_LAYER, &[1, 3]);
        assert_eq!(packet.len(), REPORT_SIZE);
        assert_eq!(&packet[..3], &[CMD_SET_LAYER, 1, 3]);
        assert!(packet[3..].iter().all(|&b| b == 0));
    }

    #[test]
    fn encode_every_command() {
        let cases: &[(u8, &[u8])] = &[
            (CMD_GET_FW_VERSION, &[]),
            (CMD_INIT_PAIRING, &[0, 0, 0, 0, 0, 0]),
            (CMD_VALIDATE_PAIRING, &[1, 2, 3, 4, 5, 6]),
            (CMD_SET_LAYER, &[1, 2]),
            (CMD_RGB_CONTROL, &[1]),
            (CMD_SET_RGB_LED, &[7, 0xFF, 0x80, 0x00]),
            (CMD_SET_STATUS_LED, &[1, 1]),
            (CMD_UPDATE_BRIGHTNESS, &[1]),
            (CMD_SET_RGB_ALL_LED, &[0x10, 0x20, 0x30]),
            (CMD_STATUS_LED_CONTROL, &[0]),
            (CMD_GET_PROTOCOL_VERSION, &[]),
        ];

        for &(cmd, params) in cases {
            let packet = encode_packet(cmd, params);
            assert_eq!(packet[0], cmd);
            assert_eq!(&packet[1..1 + params.len()], params);
            assert!(packet[1 + params.len()..].iter().all(|&b| b == 0));
        }
    }

    #[test]
    fn encode_truncates_oversized_params() {
        let packet = encode_packet(CMD_SET_RGB_ALL_LED, &[0xAB; 64]);
        assert_eq!(packet[0], CMD_SET_RGB_ALL_LED);
        assert!(packet[1..].iter().all(|&b| b == 0xAB));
    }

    #[test]
    fn parse_key_event() {
        let data = report(EVT_KEY_PRESSED, &[3, 5]);
        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_KEY_PRESSED);
        assert_eq!(params, &[3, 5]);
    }

    #[test]
    fn parse_protocol_version_quirk() {
        // Some firmware sends STOP_BIT first, then the version byte.
        let data = [STOP_BIT, 0x04, 0, 0];
        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_GET_PROTOCOL_VERSION);
        assert_eq!(params, &[0x04]);
    }

    #[test]
    fn parse_firmware_version_string() {
        let serial = b"AbCdE/XyZ12";
        let data = report(EVT_GET_FIRMWARE_VERSION, serial);
        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_GET_FIRMWARE_VERSION);
        assert_eq!(params, serial);
    }

    #[test]
    fn parse_no_stop_bit_uses_full_packet() {
        let mut data = [0u8; REPORT_SIZE];
        data[0] = EVT_LAYER_CHANGE;
        data[1] = 2;
        // no STOP_BIT anywhere: params run to the end
        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_LAYER_CHANGE);
        assert_eq!(params.len(), 31);
        assert_eq!(params[0], 2);
    }

    #[test]
    fn parse_empty_report() {
        assert!(parse_packet(&[]).is_none());
    }

    #[test]
    fn decode_firmware_version_continues_handshake() {
        let data = report(EVT_GET_FIRMWARE_VERSION, b"tPqXo/eAyRr");
        let (event, replies) = process_packet(&data);
        assert_eq!(
            event,
            Some(HidEvent::FirmwareVersion {
                layout_id: "tPqXo".into(),
                revision_id: "eAyRr".into(),
            })
        );
        assert_eq!(
            replies,
            vec![
                (CMD_INIT_PAIRING, vec![0u8; 6]),
                (CMD_GET_PROTOCOL_VERSION, Vec::new()),
            ]
        );
    }

    #[test]
    fn decode_firmware_version_without_separator_is_ignored() {
        let data = report(EVT_GET_FIRMWARE_VERSION, b"nosep");
        assert_eq!(process_packet(&data), (None, Vec::new()));
    }

    #[test]
    fn decode_pairing_init_echoes_key() {
        let key = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02];
        let data = report(EVT_PAIRING_INIT, &key);
        let (event, replies) = process_packet(&data);
        assert_eq!(event, Some(HidEvent::PairingInit));
        assert_eq!(replies, vec![(CMD_VALIDATE_PAIRING, key.to_vec())]);
        // The reply goes out as [0x02, key...]
        let packet = encode_packet(replies[0].0, &replies[0].1);
        assert_eq!(
            &packet[..7],
            &[CMD_VALIDATE_PAIRING, 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02]
        );
    }

    #[test]
    fn decode_layer_change() {
        let data = report(EVT_LAYER_CHANGE, &[4]);
        assert_eq!(process_packet(&data).0, Some(HidEvent::LayerChanged(4)));
    }

    #[test]
    fn decode_key_press_and_release() {
        let pressed = report(EVT_KEY_PRESSED, &[3, 5]);
        assert_eq!(
            process_packet(&pressed).0,
            Some(HidEvent::Key {
                column: 3,
                row: 5,
                pressed: true
            })
        );

        let released = report(EVT_KEY_RELEASED, &[3, 5]);
        assert_eq!(
            process_packet(&released).0,
            Some(HidEvent::Key {
                column: 3,
                row: 5,
                pressed: false
            })
        );
    }

    #[test]
    fn decode_key_event_needs_column_and_row() {
        let data = report(EVT_KEY_PRESSED, &[3]);
        assert_eq!(process_packet(&data), (None, Vec::new()));
    }

    #[test]
    fn decode_protocol_version() {
        let data = report(EVT_GET_PROTOCOL_VERSION, &[0x04]);
        assert_eq!(process_packet(&data).0, Some(HidEvent::ProtocolVersion(4)));
    }

    #[test]
    fn decode_protocol_version_stray_stop_bit() {
        let mut data = [0u8; REPORT_SIZE];
        data[0] = STOP_BIT;
        data[1] = 0x03;
        let (event, replies) = process_packet(&data);
        assert_eq!(event, Some(HidEvent::ProtocolVersion(3)));
        assert!(replies.is_empty());
    }

    #[test]
    fn decode_smart_layer_and_pairing_status() {
        let cases = [
            (EVT_PAIRING_KEY_INPUT, HidEvent::PairingKeyInput),
            (EVT_PAIRING_FAIL, HidEvent::PairingFail),
            (EVT_PAIRING_SUCCESS, HidEvent::PairingSuccess),
            (EVT_TOGGLE_SMARTLAYER, HidEvent::ToggleSmartLayer),
            (EVT_TRIGGER_SMARTLAYER, HidEvent::TriggerSmartLayer),
        ];
        for (opcode, expected) in cases {
            let data = report(opcode, &[]);
            assert_eq!(process_packet(&data).0, Some(expected));
        }
    }

    #[test]
    fn decode_unknown_opcode_is_ignored() {
        let data = report(0x42, &[1, 2, 3]);
        assert_eq!(process_packet(&data), (None, Vec::new()));
    }

    /// A full v1 payload: both halves read, a trackball build with the ball on
    /// the left and nothing on the right.
    fn telemetry_payload() -> Vec<u8> {
        vec![
            TELEMETRY_VERSION,
            9, // len
            87,
            TELEMETRY_BAT_KNOWN | TELEMETRY_BAT_USB,
            1, // left source: Nrf
            64,
            TELEMETRY_BAT_KNOWN | TELEMETRY_BAT_CHARGING,
            3, // right source: Wired
            TELEMETRY_NAV_TRACKBALL,
            TELEMETRY_NAV_TRACKBALL,
            0,
        ]
    }

    #[test]
    fn parse_telemetry_reads_both_halves() {
        let telemetry = parse_telemetry(&telemetry_payload()).unwrap();

        assert_eq!(telemetry.version, TELEMETRY_VERSION);
        assert_eq!(telemetry.left.level, Some(87));
        assert_eq!(telemetry.left.charging, Some(false));
        assert_eq!(telemetry.left.usb, Some(true));
        assert_eq!(telemetry.left.source, Some(Source::Nrf));
        assert_eq!(telemetry.right.level, Some(64));
        assert_eq!(telemetry.right.charging, Some(true));
        assert_eq!(telemetry.right.usb, Some(false));
        assert_eq!(telemetry.right.source, Some(Source::Wired));
        assert_eq!(
            telemetry.nav_build,
            Some(Nav {
                trackball: true,
                trackpad: false
            })
        );
        assert_eq!(telemetry.nav_right, Some(Nav::default()));
        assert!(telemetry.nav_right.unwrap().is_empty());
    }

    #[test]
    fn telemetry_rides_a_report_with_no_stop_bit() {
        // 0xFE means "reserved byte the board set", not "params end here".
        let mut data = [0u8; REPORT_SIZE];
        data[0] = EVT_TELEMETRY;
        data[1..1 + telemetry_payload().len()].copy_from_slice(&telemetry_payload());
        data[1 + telemetry_payload().len()] = STOP_BIT;

        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_TELEMETRY);
        assert_eq!(params.len(), REPORT_SIZE - 1);

        let (event, replies) = process_packet(&data);
        assert!(replies.is_empty());
        let HidEvent::Telemetry(telemetry) = event.unwrap() else {
            panic!("expected telemetry");
        };
        assert_eq!(telemetry.left.level, Some(87));
    }

    #[test]
    fn an_unread_level_is_not_a_flat_one() {
        let mut payload = telemetry_payload();
        payload[2] = TELEMETRY_LEVEL_UNKNOWN;
        payload[5] = 0; // genuinely flat

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left.level, None);
        assert_eq!(telemetry.right.level, Some(0));
    }

    #[test]
    fn an_impossible_level_is_unknown() {
        let mut payload = telemetry_payload();
        payload[2] = 101;
        assert_eq!(parse_telemetry(&payload).unwrap().left.level, None);
    }

    #[test]
    fn flags_without_the_known_bit_are_unknown_not_false() {
        let mut payload = telemetry_payload();
        payload[3] = 0; // no KNOWN bit: the board never found out
        payload[6] = TELEMETRY_BAT_KNOWN; // KNOWN, nothing set: a confident no

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left.charging, None);
        assert_eq!(telemetry.left.usb, None);
        assert_eq!(telemetry.right.charging, Some(false));
        assert_eq!(telemetry.right.usb, Some(false));
    }

    #[test]
    fn a_half_whose_pointing_device_is_undecidable_is_unknown() {
        let mut payload = telemetry_payload();
        payload[9] = TELEMETRY_NAV_UNKNOWN;
        payload[10] = 0;

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.nav_left, None);
        // Not the same as an empty one, which is a board that looked.
        assert_eq!(telemetry.nav_right, Some(Nav::default()));
    }

    #[test]
    fn a_newer_firmware_with_more_fields_still_parses() {
        // Same version, longer len, two fields this host knows nothing about.
        let mut payload = telemetry_payload();
        payload[1] = 11;
        payload.push(0x5A);
        payload.push(0xFE);

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left.level, Some(87));
        assert_eq!(telemetry.right.source, Some(Source::Wired));
        assert_eq!(
            telemetry.nav_build,
            Some(Nav {
                trackball: true,
                trackpad: false
            })
        );
    }

    #[test]
    fn a_newer_firmware_with_a_new_source_keeps_the_number() {
        let mut payload = telemetry_payload();
        payload[7] = 9;
        assert_eq!(
            parse_telemetry(&payload).unwrap().right.source,
            Some(Source::Other(9))
        );
    }

    #[test]
    fn an_older_firmware_reports_unknown_rather_than_zero() {
        // A board from before the nav fields: len covers the batteries only.
        let mut payload = telemetry_payload();
        payload[1] = 6;

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.right.source, Some(Source::Wired));
        assert_eq!(telemetry.nav_build, None);
        assert_eq!(telemetry.nav_left, None);
        assert_eq!(telemetry.nav_right, None);

        // Older still: nothing past the two header bytes.
        payload[1] = 0;
        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left, BatteryReading::default());
        assert_eq!(telemetry.right, BatteryReading::default());
    }

    #[test]
    fn a_version_this_host_does_not_know_is_refused_whole() {
        // len degrades; the version does not. A version bump means a field
        // that shipped changed meaning, so guessing would be a misread.
        let mut payload = telemetry_payload();
        payload[0] = TELEMETRY_VERSION + 1;
        assert_eq!(parse_telemetry(&payload), None);

        let mut data = [0u8; REPORT_SIZE];
        data[0] = EVT_TELEMETRY;
        data[1..1 + payload.len()].copy_from_slice(&payload);
        assert_eq!(process_packet(&data), (None, Vec::new()));
    }

    /// The same board once it also reports what its charging policy is doing.
    /// `telemetry_payload()` deliberately stays as it was: it is now a real
    /// case, a board from before these fields existed.
    fn telemetry_payload_with_charging() -> Vec<u8> {
        let mut payload = telemetry_payload();
        payload[1] = 16; // len grew with the fields; the version did not
        payload.extend_from_slice(&1500u16.to_le_bytes()); // source offered
        payload.extend_from_slice(&1500u16.to_le_bytes()); // drawn from it
        payload.extend_from_slice(&700u16.to_le_bytes()); // into each cell
        payload.push(2); // charging
        payload
    }

    #[test]
    fn the_two_charging_currents_are_read_apart() {
        let telemetry = parse_telemetry(&telemetry_payload_with_charging()).unwrap();

        assert_eq!(telemetry.charging.budget_ma, Some(1500));
        assert_eq!(telemetry.charging.input_limit_ma, Some(1500));
        assert_eq!(telemetry.charging.charge_current_ma, Some(700));
        assert_eq!(telemetry.charging.state, Some(ChargeState::Charging));
    }

    /// The same, grown by the nine bytes of the active Bluetooth identity. The
    /// address travels least significant byte first.
    fn telemetry_payload_with_ble(addr_type: u8) -> Vec<u8> {
        let mut payload = telemetry_payload_with_charging();
        payload[1] = 25;
        payload.push(addr_type);
        payload.extend_from_slice(&[0x36, 0xB1, 0xA8, 0xE9, 0xDB, 0xFE]);
        payload.push(2); // third host slot, counted from zero
        payload.push(6);
        payload
    }

    #[test]
    fn the_active_bluetooth_identity_is_read_back_the_way_it_is_written() {
        let telemetry = parse_telemetry(&telemetry_payload_with_ble(1)).unwrap();
        let ble = telemetry.ble.expect("an identity");

        assert_eq!(ble.address, "FE:DB:E9:A8:B1:36");
        assert!(ble.random);
        assert_eq!(ble.slot, 2);
        assert_eq!(ble.slot_count, 6);
    }

    #[test]
    fn a_public_address_is_not_a_random_one() {
        let ble = parse_telemetry(&telemetry_payload_with_ble(0))
            .unwrap()
            .ble
            .expect("an identity");
        assert!(!ble.random);
    }

    #[test]
    fn a_board_with_no_identity_in_the_active_slot_reports_none() {
        let telemetry = parse_telemetry(&telemetry_payload_with_ble(0xFF)).unwrap();
        assert_eq!(telemetry.ble, None);
    }

    #[test]
    fn an_identity_the_firmware_stops_short_of_is_not_half_read() {
        // Everything up to the address, and one byte of it missing: nothing to
        // read, rather than an address five bytes long.
        let mut payload = telemetry_payload_with_ble(1);
        payload[1] = 23;
        assert_eq!(parse_telemetry(&payload).unwrap().ble, None);

        // A firmware that predates the field entirely.
        assert_eq!(
            parse_telemetry(&telemetry_payload_with_charging())
                .unwrap()
                .ble,
            None
        );
    }

    /// A whole 32-byte report as the board sends it: opcode, payload, then
    /// whatever the rest of the buffer holds.
    fn telemetry_report(payload: &[u8], fill: u8) -> [u8; REPORT_SIZE] {
        let mut data = [fill; REPORT_SIZE];
        data[0] = EVT_TELEMETRY;
        data[1..1 + payload.len()].copy_from_slice(payload);
        data
    }

    fn telemetry_from_report(data: &[u8]) -> Telemetry {
        let (event, replies) = process_packet(data);
        assert!(replies.is_empty());
        let Some(HidEvent::Telemetry(telemetry)) = event else {
            panic!("expected telemetry");
        };
        telemetry
    }

    #[test]
    fn a_board_from_before_the_bus_field_says_nothing_about_the_bus() {
        // len 25 ends at the slot count. The byte after it is not the bus
        // field, whatever it holds.
        let payload = telemetry_payload_with_ble(1);
        assert_eq!(payload[1], 25);
        let telemetry = telemetry_from_report(&telemetry_report(&payload, TELEMETRY_NAV_TRACKPAD));

        assert_eq!(telemetry.nav_bus, None);
        assert_eq!(
            telemetry.nav_left,
            Some(Nav {
                trackball: true,
                trackpad: false
            })
        );
        assert_eq!(telemetry.ble.unwrap().slot_count, 6);
    }

    #[test]
    fn the_bus_field_follows_the_slot_count() {
        // Cable plugged: both halves 0xFF, a trackball and a trackpad on the
        // shared bus.
        let mut payload = telemetry_payload_with_ble(1);
        payload[1] = 26;
        payload[8] = TELEMETRY_NAV_TRACKBALL | TELEMETRY_NAV_TRACKPAD;
        payload[9] = TELEMETRY_NAV_UNKNOWN;
        payload[10] = TELEMETRY_NAV_UNKNOWN;
        payload.push(TELEMETRY_NAV_TRACKBALL | TELEMETRY_NAV_TRACKPAD);
        assert_eq!(payload.len(), 28);
        let data = telemetry_report(&payload, 0);
        assert_eq!(data[28], 0x03, "report byte 28");

        let telemetry = telemetry_from_report(&data);
        let both = Nav {
            trackball: true,
            trackpad: true,
        };
        assert_eq!(
            telemetry.nav(),
            NavReport {
                build: Some(both),
                left: None,
                right: None,
                bus: Some(both),
            }
        );
        assert_eq!(telemetry.ble.unwrap().address, "FE:DB:E9:A8:B1:36");

        // Cable unplugged: per side, bus empty, right half not linked.
        payload[9] = TELEMETRY_NAV_TRACKPAD;
        payload[27] = 0;
        let telemetry = telemetry_from_report(&telemetry_report(&payload, 0xFE));
        assert_eq!(
            telemetry.nav(),
            NavReport {
                build: Some(both),
                left: Some(Nav {
                    trackball: false,
                    trackpad: true
                }),
                right: None,
                bus: Some(Nav::default()),
            }
        );

        // Unknown on the bus.
        payload[27] = TELEMETRY_NAV_UNKNOWN;
        let telemetry = telemetry_from_report(&telemetry_report(&payload, 0));
        assert_eq!(telemetry.nav_bus, None);
    }

    /// A len 26 payload: everything up to and including the bus field.
    fn telemetry_payload_with_bus() -> Vec<u8> {
        let mut payload = telemetry_payload_with_ble(1);
        payload[1] = 26;
        payload.push(0);
        payload
    }

    #[test]
    fn a_board_from_before_the_cable_field_says_nothing_about_the_cable() {
        // len 26 ends at the bus field. The byte after it is not the cable
        // field, whatever it holds.
        let payload = telemetry_payload_with_bus();
        let telemetry = telemetry_from_report(&telemetry_report(&payload, TELEMETRY_TRRS_PLUGGED));

        assert_eq!(telemetry.trrs, None);
        assert_eq!(telemetry.nav_bus, Some(Nav::default()));
    }

    #[test]
    fn the_cable_field_follows_the_bus_field() {
        let mut payload = telemetry_payload_with_bus();
        payload[1] = 27;
        payload.push(TELEMETRY_TRRS_PLUGGED);
        assert_eq!(payload.len(), 29);
        let data = telemetry_report(&payload, 0);
        assert_eq!(data[29], TELEMETRY_TRRS_PLUGGED, "report byte 29");
        let telemetry = telemetry_from_report(&data);
        assert_eq!(telemetry.trrs, Some(true));
        assert_eq!(telemetry.nav_bus, Some(Nav::default()));

        payload[28] = TELEMETRY_TRRS_UNPLUGGED;
        let telemetry = telemetry_from_report(&telemetry_report(&payload, 0xFE));
        assert_eq!(telemetry.trrs, Some(false));

        payload[28] = TELEMETRY_TRRS_UNKNOWN;
        let telemetry = telemetry_from_report(&telemetry_report(&payload, 0));
        assert_eq!(telemetry.trrs, None);
    }

    /// A len 27 payload: everything up to and including the cable field.
    fn telemetry_payload_with_trrs() -> Vec<u8> {
        let mut payload = telemetry_payload_with_bus();
        payload[1] = 27;
        payload.push(TELEMETRY_TRRS_UNPLUGGED);
        payload
    }

    #[test]
    fn a_board_from_before_the_radio_field_says_nothing_about_the_radio() {
        // len 27 ends at the cable field. The byte after it is not the radio
        // field, whatever it holds.
        let payload = telemetry_payload_with_trrs();
        let telemetry = telemetry_from_report(&telemetry_report(&payload, TELEMETRY_ESB_LINKED));

        assert_eq!(telemetry.esb_link, None);
        assert_eq!(telemetry.trrs, Some(false));
    }

    #[test]
    fn the_radio_field_follows_the_cable_field() {
        let mut payload = telemetry_payload_with_trrs();
        payload[1] = 28;
        payload.push(TELEMETRY_ESB_LINKED);
        assert_eq!(payload.len(), 30);
        let data = telemetry_report(&payload, 0);
        assert_eq!(data[30], TELEMETRY_ESB_LINKED, "report byte 30");
        let telemetry = telemetry_from_report(&data);
        assert_eq!(telemetry.esb_link, Some(EsbLink::Linked));
        assert_eq!(telemetry.trrs, Some(false));

        for (raw, expected) in [
            (TELEMETRY_ESB_ABSENT, Some(EsbLink::Absent)),
            (TELEMETRY_ESB_MISMATCH, Some(EsbLink::Mismatch)),
            (TELEMETRY_ESB_UNKNOWN, None),
            (3, None),
        ] {
            payload[29] = raw;
            let telemetry = telemetry_from_report(&telemetry_report(&payload, 0));
            assert_eq!(telemetry.esb_link, expected, "raw {raw:#04x}");
        }
    }

    /// A len 28 payload: everything up to and including the radio field.
    fn telemetry_payload_with_esb() -> Vec<u8> {
        let mut payload = telemetry_payload_with_trrs();
        payload[1] = 28;
        payload.push(TELEMETRY_ESB_LINKED);
        payload
    }

    #[test]
    fn a_board_from_before_the_charge_issue_field_says_nothing_about_it() {
        // len 28 ends at the radio field. The byte after it is not the charge
        // issue field, whatever it holds.
        let payload = telemetry_payload_with_esb();
        let telemetry =
            telemetry_from_report(&telemetry_report(&payload, TELEMETRY_CHARGE_ISSUE_REDUCED));

        assert_eq!(telemetry.charge_issue, None);
        assert_eq!(telemetry.esb_link, Some(EsbLink::Linked));
    }

    #[test]
    fn the_charge_issue_field_follows_the_radio_field() {
        let mut payload = telemetry_payload_with_esb();
        payload[1] = 29;
        payload.push(TELEMETRY_CHARGE_ISSUE_REDUCED);
        assert_eq!(payload.len(), 31);
        let data = telemetry_report(&payload, 0);
        assert_eq!(data[31], TELEMETRY_CHARGE_ISSUE_REDUCED, "report byte 31");
        let telemetry = telemetry_from_report(&data);
        assert_eq!(telemetry.charge_issue, Some(ChargeIssue::Reduced));
        assert_eq!(telemetry.esb_link, Some(EsbLink::Linked));

        for (raw, expected) in [
            (TELEMETRY_CHARGE_ISSUE_NONE, Some(ChargeIssue::None)),
            (TELEMETRY_CHARGE_ISSUE_AT_FLOOR, Some(ChargeIssue::AtFloor)),
            (TELEMETRY_CHARGE_ISSUE_UNKNOWN, None),
            (3, Some(ChargeIssue::Other(3))),
        ] {
            payload[30] = raw;
            let telemetry = telemetry_from_report(&telemetry_report(&payload, 0));
            assert_eq!(telemetry.charge_issue, expected, "raw {raw:#04x}");
        }
    }

    #[test]
    fn a_nav_block_is_build_left_right_bus() {
        assert_eq!(
            NavReport::decode(&[0x01, 0x00, 0xFF, 0x02]),
            Some(NavReport {
                build: Some(Nav {
                    trackball: true,
                    trackpad: false
                }),
                left: Some(Nav::default()),
                right: None,
                bus: Some(Nav {
                    trackball: false,
                    trackpad: true
                }),
            })
        );
        assert_eq!(NavReport::decode(&[0x01, 0x00, 0xFF]), None);
    }

    #[test]
    fn a_currentless_reading_is_unknown_not_zero() {
        // The board has not commanded a rate yet. Zero is not a rate any of
        // this hardware can be set to, so it can only mean "did not find out".
        let mut payload = telemetry_payload_with_charging();
        payload[13] = 0;
        payload[14] = 0;
        payload[15] = 0;
        payload[16] = 0;
        payload[17] = 0xFF; // unknown state

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.charging.input_limit_ma, None);
        assert_eq!(telemetry.charging.charge_current_ma, None);
        assert_eq!(telemetry.charging.state, None);
        // A zero budget IS an answer: no source, or one not yet identified.
        assert_eq!(telemetry.charging.budget_ma, Some(1500));
    }

    #[test]
    fn a_board_from_before_the_charging_fields_says_nothing_about_them() {
        // The whole point of `len`: this host asks for fields the board never
        // heard of and gets "unknown" rather than whatever was at that offset.
        let telemetry = parse_telemetry(&telemetry_payload()).unwrap();

        assert_eq!(telemetry.charging, Charging::default());
        // and everything the old board did send still reads correctly
        assert_eq!(telemetry.left.level, Some(87));
        assert_eq!(telemetry.right.source, Some(Source::Wired));
    }

    #[test]
    fn half_a_current_is_not_a_smaller_one() {
        // A board whose len stops between the two bytes of a 16-bit field has
        // not sent that field, and reading the low byte alone would report
        // 1500 mA as 220 mA.
        let mut payload = telemetry_payload_with_charging();
        payload[1] = 10; // covers POWER_BUDGET_MA's low byte only

        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.charging.budget_ma, None);
    }

    #[test]
    fn a_charge_state_this_host_does_not_know_keeps_its_number() {
        let mut payload = telemetry_payload_with_charging();
        payload[17] = 9;
        assert_eq!(
            parse_telemetry(&payload).unwrap().charging.state,
            Some(ChargeState::Other(9))
        );
    }

    /// One detail block as a board fills it in.
    fn detail_block(voltage_mv: u16, current_ma: i16, ttf_min: u16, status: u8) -> Vec<u8> {
        let mut block = Vec::new();
        block.extend_from_slice(&voltage_mv.to_le_bytes());
        block.extend_from_slice(&current_ma.to_le_bytes());
        block.extend_from_slice(&2850i16.to_le_bytes()); // 28.50 °C
        block.extend_from_slice(&DETAIL_MIN_UNKNOWN.to_le_bytes()); // charging: no tte
        block.extend_from_slice(&ttf_min.to_le_bytes());
        block.extend_from_slice(&[status, 0, 0]); // nothing latched
        assert_eq!(block.len(), BATTERY_DETAIL_SIZE);
        block
    }

    /// A charger detecting a cell and pushing it at the full rate.
    const CHARGING_CC: u8 = 0x01 | 0x08;

    /// A board on a charger: both halves filling, the left over the nRF wire
    /// and the right over the cable between the halves.
    fn charge_payload() -> Vec<u8> {
        let mut payload = vec![CHARGE_TELEMETRY_VERSION, 28];
        payload.push(1); // left source: Nrf
        payload.extend_from_slice(&detail_block(4020, 512, 80, CHARGING_CC));
        payload.push(3); // right source: Wired
        payload.extend_from_slice(&detail_block(3980, 298, 125, CHARGING_CC));
        payload.push(0); // reserved
        assert_eq!(payload.len(), 31);
        payload
    }

    #[test]
    fn parse_charge_telemetry_reads_both_halves() {
        let telemetry = parse_charge_telemetry(&charge_payload()).unwrap();

        assert_eq!(telemetry.version, CHARGE_TELEMETRY_VERSION);
        assert_eq!(telemetry.left.source, Some(Source::Nrf));
        assert_eq!(telemetry.left.detail.voltage_mv, Some(4020));
        assert_eq!(telemetry.left.detail.current_ma, Some(512));
        assert_eq!(telemetry.left.detail.temp_cdeg, Some(2850));
        assert_eq!(telemetry.left.detail.tte_min, None);
        assert_eq!(telemetry.left.detail.ttf_min, Some(80));
        assert!(telemetry.left.detail.charger.unwrap().constant_current);
        assert!(telemetry.left.detail.fault.unwrap().is_clear());

        assert_eq!(telemetry.right.source, Some(Source::Wired));
        assert_eq!(telemetry.right.detail.voltage_mv, Some(3980));
        assert_eq!(telemetry.right.detail.ttf_min, Some(125));
    }

    #[test]
    fn charge_telemetry_rides_a_report_with_no_stop_bit() {
        // 0xFE is data here too: an unknown estimate is 0xFFFF, whose bytes
        // would otherwise be read as the end of the params.
        let payload = charge_payload();
        let mut data = [0u8; REPORT_SIZE];
        data[0] = EVT_CHARGE_TELEMETRY;
        data[1..1 + payload.len()].copy_from_slice(&payload);

        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_CHARGE_TELEMETRY);
        assert_eq!(params.len(), REPORT_SIZE - 1);

        let (event, replies) = process_packet(&data);
        assert!(replies.is_empty());
        let HidEvent::ChargeTelemetry(telemetry) = event.unwrap() else {
            panic!("expected charge telemetry");
        };
        assert_eq!(telemetry.left.detail.voltage_mv, Some(4020));
    }

    #[test]
    fn a_reading_the_board_did_not_take_is_not_a_zero_one() {
        let mut payload = charge_payload();
        // Every sentinel at once, on the left half.
        payload[3..5].copy_from_slice(&DETAIL_MV_UNKNOWN.to_le_bytes());
        payload[5..7].copy_from_slice(&DETAIL_I16_UNKNOWN.to_le_bytes());
        payload[7..9].copy_from_slice(&DETAIL_I16_UNKNOWN.to_le_bytes());
        payload[9..11].copy_from_slice(&DETAIL_MIN_UNKNOWN.to_le_bytes());
        payload[11..13].copy_from_slice(&DETAIL_MIN_UNKNOWN.to_le_bytes());
        payload[13] = DETAIL_REG_UNKNOWN;
        payload[14] = DETAIL_REG_UNKNOWN;
        payload[15] = DETAIL_REG_UNKNOWN;

        let telemetry = parse_charge_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left.detail, BatteryDetail::default());
        // A half the board did read is untouched by the one it did not.
        assert_eq!(telemetry.right.detail.voltage_mv, Some(3980));
    }

    #[test]
    fn a_cell_at_rest_reads_zero_rather_than_unknown() {
        // The one reading of each pair that a sentinel must not be able to
        // stand in for: no current, and no time left.
        let mut payload = charge_payload();
        payload[5..7].copy_from_slice(&0i16.to_le_bytes());
        payload[7..9].copy_from_slice(&0i16.to_le_bytes());
        payload[9..11].copy_from_slice(&0u16.to_le_bytes());
        payload[13] = 0;
        payload[14] = 0;

        let detail = parse_charge_telemetry(&payload).unwrap().left.detail;
        assert_eq!(detail.current_ma, Some(0));
        assert_eq!(detail.temp_cdeg, Some(0));
        assert_eq!(detail.tte_min, Some(0));
        assert!(detail.charger.unwrap().is_idle());
        assert!(detail.fault.unwrap().is_clear());
    }

    #[test]
    fn an_estimate_at_the_cap_is_no_estimate() {
        // The firmware saturates the minutes field, so the top value is where
        // an estimate stopped counting rather than one it made.
        let mut block = detail_block(4020, -45, 80, CHARGING_CC);
        block[6..8].copy_from_slice(&DETAIL_MIN_SATURATED.to_le_bytes());
        block[8..10].copy_from_slice(&DETAIL_MIN_SATURATED.to_le_bytes());

        let detail = decode_battery_detail(&block);
        assert_eq!(detail.tte_min, None);
        assert_eq!(detail.ttf_min, None);

        // One minute under the cap is still a reading.
        block[6..8].copy_from_slice(&(DETAIL_MIN_SATURATED - 1).to_le_bytes());
        assert_eq!(decode_battery_detail(&block).tte_min, Some(0xFFFD));
    }

    #[test]
    fn a_discharging_half_reads_negative() {
        let mut payload = charge_payload();
        payload[5..7].copy_from_slice(&(-45i16).to_le_bytes());
        payload[7..9].copy_from_slice(&(-320i16).to_le_bytes()); // a cold room
        payload[13] = 0x01; // a cell, and no phase

        let detail = parse_charge_telemetry(&payload).unwrap().left.detail;
        assert_eq!(detail.current_ma, Some(-45));
        assert_eq!(detail.temp_cdeg, Some(-320));
        assert!(detail.charger.unwrap().battery_detected);
        assert!(detail.charger.unwrap().is_idle());
    }

    #[test]
    fn a_board_that_stops_part_way_leaves_the_rest_unknown() {
        // len covers the left half's voltage, current and temperature, and
        // stops between the two bytes of its time-to-empty.
        let mut payload = charge_payload();
        payload[1] = 8;

        let telemetry = parse_charge_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left.source, Some(Source::Nrf));
        assert_eq!(telemetry.left.detail.voltage_mv, Some(4020));
        assert_eq!(telemetry.left.detail.current_ma, Some(512));
        assert_eq!(telemetry.left.detail.temp_cdeg, Some(2850));
        assert_eq!(telemetry.left.detail.tte_min, None, "one byte of two");
        assert_eq!(telemetry.left.detail.ttf_min, None);
        assert_eq!(telemetry.left.detail.charger, None);
        assert_eq!(telemetry.left.detail.fault, None);
        // Nothing of the right half was sent at all, source included.
        assert_eq!(telemetry.right, HalfDetail::default());
    }

    #[test]
    fn a_charge_version_this_host_does_not_know_is_refused_whole() {
        let mut payload = charge_payload();
        payload[0] = CHARGE_TELEMETRY_VERSION + 1;
        assert_eq!(parse_charge_telemetry(&payload), None);

        let mut data = [0u8; REPORT_SIZE];
        data[0] = EVT_CHARGE_TELEMETRY;
        data[1..1 + payload.len()].copy_from_slice(&payload);
        assert_eq!(process_packet(&data), (None, Vec::new()));

        // And a payload with no header at all is not a payload.
        assert_eq!(parse_charge_telemetry(&[]), None);
        assert_eq!(parse_charge_telemetry(&[CHARGE_TELEMETRY_VERSION]), None);
    }

    #[test]
    fn a_bare_block_is_read_as_far_as_it_goes() {
        let block = detail_block(4020, -45, 80, CHARGING_CC);
        let detail = decode_battery_detail(&block);
        assert_eq!(detail.voltage_mv, Some(4020));
        assert_eq!(detail.current_ma, Some(-45));

        // Three bytes: a whole voltage and half a current.
        let short = decode_battery_detail(&block[..3]);
        assert_eq!(short.voltage_mv, Some(4020));
        assert_eq!(short.current_ma, None);
        assert_eq!(decode_battery_detail(&[]), BatteryDetail::default());
    }

    #[test]
    fn the_charger_phases_are_read_apart() {
        let cases = [
            (0x02, "complete"),
            (0x04, "trickle"),
            (0x08, "constant current"),
            (0x10, "constant voltage"),
            (0x20, "recharge"),
            (0x40, "paused"),
        ];
        for (raw, what) in cases {
            let status = ChargerStatus::from_bits(raw | 0x01);
            assert!(status.battery_detected);
            assert!(!status.is_idle(), "{what} is a phase");
        }
        // A cell being helped along is not a phase either way round.
        assert!(ChargerStatus::from_bits(0x81).supplement);
        assert!(ChargerStatus::from_bits(0x81).is_idle());
        assert!(ChargerStatus::from_bits(0).is_idle());
    }

    #[test]
    fn a_latched_fault_names_its_bits() {
        let fault = ChargerFault {
            reason: 0x40 | 0x08,
            sensor: 0x08,
        };
        assert!(!fault.is_clear());
        assert!(fault.trickle_timeout());
        assert!(fault.trickle_voltage());
        assert!(!fault.charge_timeout());
        assert!(fault.sensor_ntc_hot());
        assert_eq!(fault.reasons(), vec!["trickle voltage", "trickle timeout"]);
        assert_eq!(fault.sensors(), vec!["NTC hot"]);

        let clear = ChargerFault::default();
        assert!(clear.is_clear());
        assert!(clear.reasons().is_empty());
    }

    #[test]
    fn a_fault_whose_sensor_byte_is_missing_still_stands() {
        let mut payload = charge_payload();
        payload[14] = 0x20; // charge timeout
        payload[15] = DETAIL_REG_UNKNOWN;

        let fault = parse_charge_telemetry(&payload).unwrap().left.detail.fault;
        assert_eq!(
            fault,
            Some(ChargerFault {
                reason: 0x20,
                sensor: 0
            })
        );
        assert!(fault.unwrap().sensors().is_empty());
    }

    #[test]
    fn a_truncated_telemetry_report_is_not_invented() {
        // The board claims nine bytes but only three arrived.
        let payload = [TELEMETRY_VERSION, 9, 42];
        let telemetry = parse_telemetry(&payload).unwrap();
        assert_eq!(telemetry.left.level, Some(42));
        assert_eq!(telemetry.left.charging, None);
        assert_eq!(telemetry.right.level, None);
        assert_eq!(telemetry.nav_build, None);

        assert_eq!(parse_telemetry(&[]), None);
        assert_eq!(parse_telemetry(&[TELEMETRY_VERSION]), None);
    }

    /// One rails block as a half on the cable fills it in.
    fn rails_block(vbus_mv: u16, vsys_mv: u16, vbusin: u8, chg_enable: u8) -> Vec<u8> {
        let mut block = vec![PMIC_RAILS_PRESENT];
        block.extend_from_slice(&vbus_mv.to_le_bytes());
        block.extend_from_slice(&vsys_mv.to_le_bytes());
        block.push(vbusin);
        block.push(chg_enable);
        block
    }

    #[test]
    fn a_rails_block_says_what_the_two_rails_measure() {
        let rails = decode_pmic_rails(&rails_block(4620, 4550, 0x21, 0x01)).unwrap();
        assert_eq!(rails.vbus_mv, 4620);
        assert_eq!(rails.vsys_mv, 4550);
        assert_eq!(rails.charger_enabled, Some(true));
        assert!(rails.vbusin.present);
        assert!(rails.vbusin.out_active);
        assert!(!rails.vbusin.current_limit);
        assert_eq!(rails.vbusin.raw, 0x21);
        assert_eq!(rails.vbusin.names(), vec!["present", "out active"]);
    }

    #[test]
    fn every_vbusin_bit_has_a_name_in_register_order() {
        let all = VbusInStatus::from_bits(0x3F);
        assert!(
            all.present
                && all.current_limit
                && all.overvoltage
                && all.undervoltage
                && all.suspended
                && all.out_active
        );
        assert_eq!(
            all.names(),
            vec![
                "present",
                "current limit",
                "overvoltage",
                "undervoltage",
                "suspended",
                "out active"
            ]
        );

        // Nothing attached and the output off is a reading, not an absence.
        let none = VbusInStatus::from_bits(0);
        assert!(none.names().is_empty());
        assert_eq!(none, VbusInStatus::default());
    }

    #[test]
    fn a_half_that_has_not_measured_yet_has_no_rails() {
        let mut block = rails_block(4620, 4550, 0x21, 0x01);
        block[0] = 0;
        assert_eq!(decode_pmic_rails(&block), None);
    }

    #[test]
    fn a_rails_block_is_whole_or_it_is_nothing() {
        let block = rails_block(4620, 4550, 0x21, 0x01);
        assert_eq!(decode_pmic_rails(&block[..6]), None);
        assert_eq!(decode_pmic_rails(&[]), None);
        // Longer is no more readable than shorter: it is not this block.
        assert_eq!(decode_pmic_rails(&[block.as_slice(), &[0]].concat()), None);
    }

    #[test]
    fn a_charger_register_that_was_not_read_is_not_a_charger_that_is_off() {
        let unread = decode_pmic_rails(&rails_block(4620, 4550, 0x21, PMIC_REG_UNKNOWN)).unwrap();
        assert_eq!(unread.charger_enabled, None);

        let off = decode_pmic_rails(&rails_block(4620, 4550, 0x21, 0x00)).unwrap();
        assert_eq!(off.charger_enabled, Some(false));
    }

    #[test]
    fn a_boost_state_is_only_read_when_the_half_says_it_knows_it() {
        // Firmware that does not report the boost leaves both flag bits clear.
        let silent = decode_pmic_rails(&rails_block(4620, 4550, 0x21, 0x01)).unwrap();
        assert_eq!(silent.boost_off, None);

        let mut block = rails_block(4620, 4550, 0x21, 0x01);
        block[0] |= PMIC_RAILS_BOOST_KNOWN;
        assert_eq!(decode_pmic_rails(&block).unwrap().boost_off, Some(false));

        block[0] |= PMIC_RAILS_BOOST_OFF;
        assert_eq!(decode_pmic_rails(&block).unwrap().boost_off, Some(true));

        // The off bit on its own is a claim the sender did not make.
        let mut lone = rails_block(4620, 4550, 0x21, 0x01);
        lone[0] |= PMIC_RAILS_BOOST_OFF;
        assert_eq!(decode_pmic_rails(&lone).unwrap().boost_off, None);
    }

    #[test]
    fn a_held_back_charge_current_is_read_from_the_flag() {
        // A half charging at the rate it was told reports nothing.
        let full = decode_pmic_rails(&rails_block(4620, 4550, 0x21, 0x01)).unwrap();
        assert!(!full.charge_reduced);

        let mut block = rails_block(4620, 4550, 0x21, 0x01);
        block[0] |= PMIC_RAILS_CHARGE_REDUCED;
        assert!(decode_pmic_rails(&block).unwrap().charge_reduced);
    }

    /// A board on a charger answering the rails command: both halves measured.
    fn rails_payload() -> Vec<u8> {
        let mut payload = vec![PMIC_RAILS_REPORT_VERSION, 14];
        payload.extend_from_slice(&rails_block(4620, 4550, 0x21, 0x01));
        payload.extend_from_slice(&rails_block(4580, 4510, 0x01, 0x01));
        payload
    }

    #[test]
    fn parse_pmic_rails_report_reads_both_halves() {
        let report = parse_pmic_rails_report(&rails_payload()).unwrap();

        assert_eq!(report.version, PMIC_RAILS_REPORT_VERSION);
        let left = report.left.unwrap();
        assert_eq!(left.vbus_mv, 4620);
        assert_eq!(left.vsys_mv, 4550);
        assert_eq!(left.charger_enabled, Some(true));
        assert!(left.vbusin.present && left.vbusin.out_active);

        let right = report.right.unwrap();
        assert_eq!(right.vbus_mv, 4580);
        assert_eq!(right.vsys_mv, 4510);
        assert!(right.vbusin.present && !right.vbusin.out_active);
    }

    #[test]
    fn a_rails_report_rides_a_report_with_no_stop_bit() {
        // 0xFE is data here too: 4606 mV is 0xFE 0x11 on the wire, and
        // truncating there would drop the rest of the half's block.
        let mut payload = vec![PMIC_RAILS_REPORT_VERSION, 14];
        payload.extend_from_slice(&rails_block(4606, 4550, 0x21, 0x01));
        payload.extend_from_slice(&rails_block(4580, 4510, 0x01, 0x01));
        let mut data = [0u8; REPORT_SIZE];
        data[0] = EVT_PMIC_RAILS;
        data[1..1 + payload.len()].copy_from_slice(&payload);

        let (evt, params) = parse_packet(&data).unwrap();
        assert_eq!(evt, EVT_PMIC_RAILS);
        assert_eq!(params.len(), REPORT_SIZE - 1);

        let (event, replies) = process_packet(&data);
        assert!(replies.is_empty());
        let HidEvent::PmicRails(report) = event.unwrap() else {
            panic!("expected a rails report");
        };
        assert_eq!(report.left.unwrap().vbus_mv, 4606);
        assert_eq!(report.right.unwrap().vbus_mv, 4580);
    }

    #[test]
    fn a_firmware_that_predates_the_rails_has_none_to_report() {
        // The bytes are in the report, but the board's own len says it never
        // wrote them, so they are padding rather than a reading of zero.
        let mut payload = rails_payload();
        payload[1] = 0;
        let report = parse_pmic_rails_report(&payload).unwrap();
        assert_eq!(report.left, None);
        assert_eq!(report.right, None);

        // A len stopping inside the left block leaves it unread too.
        payload[1] = 6;
        assert_eq!(parse_pmic_rails_report(&payload).unwrap().left, None);

        // A format this host was not written against is refused whole.
        payload[0] = PMIC_RAILS_REPORT_VERSION + 1;
        assert_eq!(parse_pmic_rails_report(&payload), None);

        assert_eq!(parse_pmic_rails_report(&[]), None);
        assert_eq!(parse_pmic_rails_report(&[PMIC_RAILS_REPORT_VERSION]), None);
    }

    #[test]
    fn a_left_that_has_not_measured_does_not_hide_the_right() {
        // Each block carries its own present flag, so a half still waiting on
        // its first tick says nothing while the other one reports.
        let mut payload = rails_payload();
        payload[rails_report_offset::LEFT] = 0;

        let report = parse_pmic_rails_report(&payload).unwrap();
        assert_eq!(report.left, None);
        assert_eq!(report.right.unwrap().vbus_mv, 4580);
    }

    /// An STM32 record built field by field, in wire order.
    fn stm32_record() -> Vec<u8> {
        let mut b = vec![DIAG_RECORD_VERSION, (STM32_DIAG_SIZE - 2) as u8];
        b.extend_from_slice(&7u16.to_le_bytes()); // boot_count
        b.extend_from_slice(&8040u32.to_le_bytes()); // uptime_s
        b.push(STM32_RESET_IWDG | 0x04); // reset_flags
        b.push(STM32_DIAG_LINK_UP | STM32_DIAG_USB_HOST | STM32_DIAG_TRRS); // flags
        b.push(2); // transport
        b.push(0); // idle_step
        b.push(1); // right_link
        b.push(1); // power_gate
        for value in [3u16, 1, 2, 3, 4, 5, 6, 7, 0xFE] {
            // link_ups, sd_*, parser_resyncs, crc_errors, frame_aborts
            b.extend_from_slice(&value.to_le_bytes());
        }
        for value in [123_456u32, 23_456, 12, 0x0001_FEFE] {
            // frames_rx, frames_tx, rx_age_ms, max_rx_gap_ms
            b.extend_from_slice(&value.to_le_bytes());
        }
        assert_eq!(b.len(), STM32_DIAG_SIZE);
        b
    }

    /// A left nRF record built field by field, in wire order.
    fn left_record() -> Vec<u8> {
        let mut b = vec![DIAG_RECORD_VERSION, (LEFT_DIAG_SIZE - 2) as u8];
        b.extend_from_slice(&2u16.to_le_bytes()); // boot_count
        b.extend_from_slice(&300u32.to_le_bytes()); // uptime_s
        b.extend_from_slice(&NRF_RESET_SREQ.to_le_bytes()); // resetreas
        b.push(LEFT_DIAG_STM32_LINK | LEFT_DIAG_BLE_HOST); // flags
        b.push(0xFF); // ble_slot
        b.push(2); // esb_link
        b.push(1); // stm32_idle_step
        for value in [12u16, 0, 400] {
            // conn_interval, conn_latency, conn_timeout
            b.extend_from_slice(&value.to_le_bytes());
        }
        for value in 1u16..=14 {
            // uart_frames .. hid_send_max_ms
            b.extend_from_slice(&value.to_le_bytes());
        }
        b.extend_from_slice(&DIAG_NEVER.to_le_bytes()); // uart_rx_age_ms
        b.extend_from_slice(&5u32.to_le_bytes()); // esb_rx_age_ms
        for (stall, max) in [(0u32, 47_200u32), (0, 3), (0, 1), (900, 900)] {
            b.extend_from_slice(&stall.to_le_bytes());
            b.extend_from_slice(&max.to_le_bytes());
        }
        assert_eq!(b.len(), LEFT_DIAG_SIZE);
        b
    }

    /// An `EVT_DIAG` report carrying `data` as page `page` of `record`.
    fn diag_report(record: u8, page: u8, total_len: u8, age_ms: u16, data: &[u8]) -> Vec<u8> {
        let mut r = vec![EVT_DIAG, DIAG_PAGE_VERSION, record, page, 0, total_len];
        r.extend_from_slice(&age_ms.to_le_bytes());
        r.extend_from_slice(data);
        r.resize(REPORT_SIZE, 0);
        r
    }

    /// The pages a record is served as.
    fn paged(record: u8, bytes: &[u8]) -> Vec<DiagPage> {
        let total = bytes.len() as u8;
        (0..diag_page_count(total))
            .map(|page| {
                let start = usize::from(page) * DIAG_PAGE_DATA;
                let end = (start + DIAG_PAGE_DATA).min(bytes.len());
                let report = diag_report(record, page, total, 0, &bytes[start..end]);
                match process_packet(&report).0 {
                    Some(HidEvent::Diag(page)) => page,
                    other => panic!("not a diag page: {other:?}"),
                }
            })
            .collect()
    }

    #[test]
    fn a_diag_reply_is_not_cut_at_a_0xfe_byte() {
        let report = diag_report(0, 1, 48, 0, &[0xFE; DIAG_PAGE_DATA]);
        let (evt, params) = parse_packet(&report).unwrap();
        assert_eq!(evt, EVT_DIAG);
        assert_eq!(params.len(), DIAG_PAGE_SIZE);
        let page = parse_diag_page(params).unwrap();
        assert_eq!(page.data, [0xFE; DIAG_PAGE_DATA]);
    }

    #[test]
    fn a_diag_page_decodes_its_header_and_data() {
        let report = diag_report(3, 2, 90, 0x1234, &[9, 8, 7]);
        let (event, replies) = process_packet(&report);
        assert!(replies.is_empty());
        let Some(HidEvent::Diag(page)) = event else {
            panic!("not a diag page: {event:?}");
        };
        assert_eq!(page.version, DIAG_PAGE_VERSION);
        assert_eq!((page.record, page.page, page.total_len), (3, 2, 90));
        assert_eq!(page.status, DiagStatus::Ok);
        assert_eq!(page.age_ms, 0x1234);
        assert_eq!(&page.data[..4], &[9, 8, 7, 0]);

        let mut none = diag_report(2, 0, 0, DIAG_AGE_UNKNOWN, &[]);
        let status = |raw: u8, none: &mut Vec<u8>| {
            none[4] = raw;
            parse_diag_page(&none[1..]).unwrap().status
        };
        assert_eq!(status(1, &mut none), DiagStatus::NoRecord);
        assert_eq!(status(2, &mut none), DiagStatus::OutOfRange);
        assert_eq!(status(9, &mut none), DiagStatus::Other(9));

        // Short replies are dropped.
        assert_eq!(parse_diag_page(&report[1..DIAG_PAGE_SIZE]), None);
        assert_eq!(process_packet(&report[..8]).0, None);
    }

    #[test]
    fn a_diag_request_is_record_page_flags() {
        let packet = encode_packet(CMD_GET_DIAG, &diag_request(DiagRecord::Left, 0, DIAG_FETCH));
        assert_eq!(&packet[..5], &[0x11, 2, 0, 0x01, 0]);
        assert_eq!(DiagRecord::LeftPrevious.id(), 3);
        assert!(DiagRecord::Left.is_left() && !DiagRecord::Stm32Previous.is_left());
    }

    /// An `EVT_RADIO_INFO` report: left 0.2.3 build 7, both halves known.
    fn radio_report(format: u8, flags: u8) -> Vec<u8> {
        let mut r = vec![EVT_RADIO_INFO, format, flags, 0, 2];
        r.extend_from_slice(&3u16.to_le_bytes());
        r.extend_from_slice(&7u32.to_le_bytes());
        r.extend_from_slice(&[0xFE, 1, 2, 3, 4, 5, 6, 7]);
        r.extend_from_slice(&[0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xFE]);
        assert_eq!(r.len(), 1 + RADIO_INFO_SIZE);
        r.resize(REPORT_SIZE, 0);
        r
    }

    #[test]
    fn radio_info_decodes_both_halves() {
        let flags = RADIO_LEFT_VALID | RADIO_LEFT_CONFIRMED | RADIO_RIGHT_VALID;
        let (event, replies) = process_packet(&radio_report(RADIO_INFO_FORMAT, flags));
        assert!(replies.is_empty());
        let Some(HidEvent::RadioInfo(info)) = event else {
            panic!("not radio info: {event:?}");
        };
        let left = info.left.unwrap();
        assert_eq!(
            left.version,
            RadioVersion {
                major: 0,
                minor: 2,
                revision: 3,
                build: 7
            }
        );
        assert_eq!(left.version.to_string(), "0.2.3");
        // A 0xFE in a hash is data, not a terminator.
        assert_eq!(left.hash, [0xFE, 1, 2, 3, 4, 5, 6, 7]);
        assert!(left.confirmed);
        let right = info.right.unwrap();
        assert_eq!(right.tag, [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xFE]);
        assert!(!right.confirmed);
        assert!(info.is_known());
    }

    #[test]
    fn radio_info_reads_each_half_by_its_flag() {
        let report = radio_report(RADIO_INFO_FORMAT, RADIO_LEFT_VALID | RADIO_RIGHT_CONFIRMED);
        let info = parse_radio_info(&report[1..]).unwrap();
        assert!(info.left.is_some());
        assert_eq!(info.right, None);

        let report = radio_report(RADIO_INFO_FORMAT, RADIO_RIGHT_VALID | RADIO_RIGHT_CONFIRMED);
        let info = parse_radio_info(&report[1..]).unwrap();
        assert_eq!(info.left, None);
        assert!(info.right.unwrap().confirmed);
    }

    #[test]
    fn radio_info_flags_zero_is_unknown() {
        let info = parse_radio_info(&radio_report(RADIO_INFO_FORMAT, 0)[1..]).unwrap();
        assert_eq!(info, RadioInfo::default());
        assert!(!info.is_known());
    }

    #[test]
    fn radio_info_in_another_format_is_unknown() {
        let all =
            RADIO_LEFT_VALID | RADIO_LEFT_CONFIRMED | RADIO_RIGHT_VALID | RADIO_RIGHT_CONFIRMED;
        for format in [0, 2, 0xFF] {
            let (event, _) = process_packet(&radio_report(format, all));
            assert_eq!(event, Some(HidEvent::RadioInfo(RadioInfo::default())));
        }
    }

    #[test]
    fn short_radio_info_is_dropped() {
        let report = radio_report(RADIO_INFO_FORMAT, RADIO_LEFT_VALID);
        assert_eq!(parse_radio_info(&report[1..RADIO_INFO_SIZE]), None);
        assert_eq!(parse_radio_info(&[]), None);
        assert_eq!(process_packet(&report[..RADIO_INFO_SIZE]).0, None);
    }

    /// An `EVT_HOST_SLOTS` report for slot 1 of 5, address C4:12:9A:33:0E:FE.
    fn host_slot_report(flags: u8, addr_type: u8) -> Vec<u8> {
        let mut r = vec![EVT_HOST_SLOTS, 1, 5, flags, addr_type];
        r.extend_from_slice(&[0xFE, 0x0E, 0x33, 0x9A, 0x12, 0xC4]);
        assert_eq!(r.len(), 1 + HOST_SLOT_SIZE);
        r.resize(REPORT_SIZE, 0);
        r
    }

    #[test]
    fn host_slot_decodes_address_and_state() {
        let report = host_slot_report(HOST_SLOT_PAIRED | HOST_SLOT_CONNECTED, 0x01);
        let (event, replies) = process_packet(&report);
        assert!(replies.is_empty());
        let Some(HidEvent::HostSlot(slot)) = event else {
            panic!("not a host slot: {event:?}");
        };
        assert_eq!(slot.slot, 1);
        assert_eq!(slot.slot_count, 5);
        assert!(slot.paired && slot.connected && slot.random);
        // Most significant byte first; the 0xFE is data, not a terminator.
        assert_eq!(slot.address.as_deref(), Some("C4:12:9A:33:0E:FE"));

        let free = parse_host_slot(&host_slot_report(0, 0x00)[1..]).unwrap();
        assert!(!free.paired && !free.connected && !free.random);
        assert_eq!(free.address.as_deref(), Some("C4:12:9A:33:0E:FE"));
    }

    #[test]
    fn host_slot_unknown_address_is_none() {
        let slot = parse_host_slot(&host_slot_report(HOST_SLOT_PAIRED, 0xFF)[1..]).unwrap();
        assert_eq!(slot.address, None);
        assert!(slot.paired);
    }

    #[test]
    fn short_host_slot_is_dropped() {
        let report = host_slot_report(HOST_SLOT_PAIRED, 0x00);
        assert_eq!(parse_host_slot(&report[1..HOST_SLOT_SIZE]), None);
        assert_eq!(process_packet(&report[..HOST_SLOT_SIZE]).0, None);
        assert_eq!(&encode_packet(CMD_GET_HOST_SLOTS, &[3])[..3], &[0x13, 3, 0]);
    }

    #[test]
    fn radio_info_needs_protocol_6() {
        assert!(!has_radio_info(0x05));
        assert!(has_radio_info(0x06));
        assert!(has_radio_info(0x07));
        assert_eq!(&encode_packet(CMD_GET_RADIO_INFO, &[])[..2], &[0x12, 0]);
    }

    #[test]
    fn page_count_rounds_up() {
        assert_eq!(diag_page_count(0), 0);
        assert_eq!(diag_page_count(24), 1);
        assert_eq!(diag_page_count(25), 2);
        assert_eq!(diag_page_count(STM32_DIAG_SIZE as u8), 2);
        assert_eq!(diag_page_count(LEFT_DIAG_SIZE as u8), 4);
    }

    #[test]
    fn pages_reassemble_into_the_record() {
        let left = left_record();
        let pages = paged(2, &left);
        assert_eq!(assemble_diag_record(&pages).unwrap(), left);

        // Missing, reordered or mismatched pages are refused.
        assert_eq!(assemble_diag_record(&pages[..3]), None);
        let mut swapped = pages.clone();
        swapped.swap(1, 2);
        assert_eq!(assemble_diag_record(&swapped), None);
        let mut mixed = pages.clone();
        mixed[1].record = 3;
        assert_eq!(assemble_diag_record(&mixed), None);
        let mut failed = pages;
        failed[3].status = DiagStatus::NoRecord;
        assert_eq!(assemble_diag_record(&failed), None);
        assert_eq!(assemble_diag_record(&[]), None);
    }

    #[test]
    fn the_stm32_record_decodes_field_by_field() {
        let diag = parse_stm32_diag(&stm32_record()).unwrap();
        assert_eq!(
            diag,
            Stm32Diag {
                boot_count: 7,
                uptime_s: 8040,
                reset_flags: 0x24,
                flags: 0x15,
                transport: 2,
                idle_step: 0,
                right_link: 1,
                power_gate: 1,
                link_ups: 3,
                sd_overruns: 1,
                sd_framing: 2,
                sd_noise: 3,
                sd_parity: 4,
                sd_queue_full: 5,
                parser_resyncs: 6,
                crc_errors: 7,
                frame_aborts: 0xFE,
                frames_rx: 123_456,
                frames_tx: 23_456,
                rx_age_ms: 12,
                max_rx_gap_ms: 0x0001_FEFE,
                left_probes: None,
                left_resets: None,
            }
        );
    }

    #[test]
    fn the_stm32_rescue_counters_decode_only_from_the_longer_record() {
        // Older firmware: 48 bytes, len 46.
        let short = parse_stm32_diag(&stm32_record()).unwrap();
        assert_eq!((short.left_probes, short.left_resets), (None, None));

        // Newer firmware: 50 bytes, len 48.
        let mut long = stm32_record();
        long[1] = (STM32_DIAG_SIZE_RESCUE - 2) as u8;
        long.extend_from_slice(&[3, 1]);
        let diag = parse_stm32_diag(&long).unwrap();
        assert_eq!((diag.left_probes, diag.left_resets), (Some(3), Some(1)));
        assert_eq!(diag.max_rx_gap_ms, short.max_rx_gap_ms);

        // A len that claims them without the bytes, or bytes without the len.
        let mut cut = long.clone();
        cut.truncate(STM32_DIAG_SIZE_RESCUE - 1);
        assert_eq!(parse_stm32_diag(&cut).unwrap().left_probes, None);
        let mut unclaimed = long;
        unclaimed[1] = (STM32_DIAG_SIZE - 2) as u8;
        assert_eq!(parse_stm32_diag(&unclaimed).unwrap().left_probes, None);
    }

    #[test]
    fn the_left_record_decodes_field_by_field() {
        let diag = parse_left_diag(&left_record()).unwrap();
        assert_eq!(diag.boot_count, 2);
        assert_eq!(diag.uptime_s, 300);
        assert_eq!(diag.resetreas, NRF_RESET_SREQ);
        assert_eq!(diag.flags, LEFT_DIAG_STM32_LINK | LEFT_DIAG_BLE_HOST);
        assert_eq!(diag.ble_slot, 0xFF);
        assert_eq!(diag.esb_link, 2);
        assert_eq!(diag.stm32_idle_step, 1);
        assert_eq!(diag.conn_interval, 12);
        assert_eq!(diag.conn_latency, 0);
        assert_eq!(diag.conn_timeout, 400);
        let counters = [
            diag.uart_frames,
            diag.uart_bad_frames,
            diag.uart_rx_errors,
            diag.uart_rx_overruns,
            diag.uart_rx_line_errors,
            diag.uart_ring_drops,
            diag.uart_rx_restarts,
            diag.kbd_reports,
            diag.kbd_dropped,
            diag.ble_send_errors,
            diag.notify_errors,
            diag.esb_hops,
            diag.esb_timeouts,
            diag.hid_send_max_ms,
        ];
        assert_eq!(counters, core::array::from_fn(|i| i as u16 + 1));
        assert_eq!(diag.uart_rx_age_ms, DIAG_NEVER);
        assert_eq!(diag.esb_rx_age_ms, 5);
        let stalls = diag.threads.map(|t| (t.stall_ms, t.max_stall_ms));
        assert_eq!(stalls, [(0, 47_200), (0, 3), (0, 1), (900, 900)]);
    }

    #[test]
    fn a_record_of_another_version_is_refused() {
        let mut stm32 = stm32_record();
        stm32[0] = DIAG_RECORD_VERSION + 1;
        assert_eq!(parse_stm32_diag(&stm32), None);
        let mut left = left_record();
        left[0] = 0;
        assert_eq!(parse_left_diag(&left), None);
    }

    #[test]
    fn a_short_record_is_refused_and_a_longer_one_decodes() {
        let stm32 = stm32_record();
        assert_eq!(parse_stm32_diag(&stm32[..STM32_DIAG_SIZE - 1]), None);
        assert_eq!(parse_stm32_diag(&stm32[..2]), None);
        assert_eq!(parse_stm32_diag(&[]), None);

        // A len byte that stops short refuses it even when the bytes are there.
        let mut short_len = stm32.clone();
        short_len[1] -= 1;
        assert_eq!(parse_stm32_diag(&short_len), None);

        let mut longer = left_record();
        longer[1] += 2;
        longer.extend_from_slice(&[0xAB, 0xCD]);
        assert_eq!(parse_left_diag(&longer), parse_left_diag(&left_record()));
    }

    #[test]
    fn reset_causes_are_named() {
        assert_eq!(nrf_reset_causes(0), ["power-on or brownout"]);
        assert_eq!(nrf_reset_causes(NRF_RESET_SREQ), ["software reset"]);
        assert_eq!(
            nrf_reset_causes(1 | NRF_RESET_DOG1 | NRF_RESET_LOCKUP | 1 << 13),
            ["reset pin", "watchdog 1", "CPU lockup", "tamper detector"]
        );
        assert!(nrf_reset_causes(1 << 20).is_empty());
        assert_eq!(stm32_reset_causes(0x0C), ["reset pin", "power-on"]);
        assert_eq!(
            stm32_reset_causes(STM32_RESET_IWDG | 0x04),
            ["reset pin", "watchdog"]
        );
        assert!(stm32_reset_causes(0x01).is_empty());
    }

    /// An `EVT_BLOB` report.
    fn blob_report(which: u8, status: u8, total: u16, offset: u16, data: &[u8]) -> Vec<u8> {
        let mut r = vec![EVT_BLOB, which, status];
        r.extend_from_slice(&total.to_le_bytes());
        r.extend_from_slice(&offset.to_le_bytes());
        r.push(data.len() as u8);
        r.extend_from_slice(data);
        r.resize(REPORT_SIZE, 0);
        r
    }

    #[test]
    fn blob_chunk_decodes() {
        let data: Vec<u8> = (0xE8..=0xFF).collect();
        let (event, replies) = process_packet(&blob_report(BLOB_EVENTS, BLOB_OK, 300, 48, &data));
        assert!(replies.is_empty());
        assert_eq!(
            event,
            Some(HidEvent::Blob(BlobChunk {
                which: BLOB_EVENTS,
                status: BLOB_OK,
                total_len: 300,
                offset: 48,
                // 0xFE in it is data.
                data,
            }))
        );
        assert_eq!(
            &encode_packet(CMD_GET_BLOB, &blob_request(1, 0x0130))[..4],
            &[0x14, 1, 0x30, 0x01]
        );
    }

    #[test]
    fn blob_chunk_rejects_short_and_overlong() {
        let report = blob_report(BLOB_FREEZE, BLOB_OK, 10, 0, &[1, 2]);
        assert_eq!(parse_blob_chunk(&report[1..BLOB_SIZE]), None);
        let mut bad = blob_report(BLOB_FREEZE, BLOB_OK, 10, 0, &[]);
        bad[7] = 25;
        assert_eq!(parse_blob_chunk(&bad[1..]), None);
    }

    #[test]
    fn freeze_seen_and_error_events() {
        let mut r = vec![EVT_FREEZE_SEEN, BLOB_NO_LINK];
        r.resize(REPORT_SIZE, 0);
        assert_eq!(
            process_packet(&r).0,
            Some(HidEvent::FreezeSeen(BLOB_NO_LINK))
        );
        let mut r = vec![EVT_ERROR, ERR_UNKNOWN_COMMAND];
        r.resize(REPORT_SIZE, 0);
        assert_eq!(
            process_packet(&r).0,
            Some(HidEvent::Error(ERR_UNKNOWN_COMMAND))
        );
    }

    /// Answers `read_blob` from `blob`, `BLOB_CHUNK` bytes at a time.
    fn serve(which: u8, blob: &[u8]) -> impl FnMut(u16) -> Option<Result<BlobChunk, ()>> + '_ {
        move |offset| {
            let at = usize::from(offset).min(blob.len());
            let end = (at + BLOB_CHUNK).min(blob.len());
            Some(Ok(BlobChunk {
                which,
                status: BLOB_OK,
                total_len: blob.len() as u16,
                offset,
                data: blob[at..end].to_vec(),
            }))
        }
    }

    #[test]
    fn read_blob_joins_chunks() {
        let blob: Vec<u8> = (0..100u8).collect();
        let mut asked = Vec::new();
        let mut inner = serve(BLOB_EVENTS, &blob);
        let read = read_blob(BLOB_EVENTS, usize::MAX, |offset| {
            asked.push(offset);
            inner(offset)
        });
        assert_eq!(read, BlobRead::Data(blob.clone()));
        assert_eq!(asked, [0, 24, 48, 72, 96]);

        // A limit stops early: the 36-byte header is two chunks.
        let mut count = 0;
        let mut inner = serve(BLOB_EVENTS, &blob);
        let read = read_blob(BLOB_EVENTS, FREEZE_HEADER_SIZE, |offset| {
            count += 1;
            inner(offset)
        });
        assert_eq!(read, BlobRead::Data(blob[..FREEZE_HEADER_SIZE].to_vec()));
        assert_eq!(count, 2);

        // An empty blob is one chunk.
        assert_eq!(
            read_blob(BLOB_FREEZE, usize::MAX, serve(BLOB_FREEZE, &[])),
            BlobRead::Data(Vec::new())
        );
    }

    #[test]
    fn read_blob_outcomes() {
        let none = |offset| {
            Some(Ok(BlobChunk {
                which: 0,
                status: BLOB_NONE,
                total_len: 0,
                offset,
                data: Vec::new(),
            }))
        };
        assert_eq!(read_blob(BLOB_FREEZE, usize::MAX, none), BlobRead::None);
        assert_eq!(
            read_blob(BLOB_FREEZE, usize::MAX, |_| None),
            BlobRead::Unsupported
        );
        assert_eq!(
            read_blob(BLOB_FREEZE, usize::MAX, |_| Some(Err(()))),
            BlobRead::Unsupported
        );
        let no_link = |offset| {
            Some(Ok(BlobChunk {
                which: 0,
                status: BLOB_NO_LINK,
                total_len: 0,
                offset,
                data: Vec::new(),
            }))
        };
        assert_eq!(
            read_blob(BLOB_FREEZE, usize::MAX, no_link),
            BlobRead::Failed(BLOB_NO_LINK)
        );

        // Silence after the first chunk is a timeout.
        let blob = [7u8; 40];
        let mut inner = serve(BLOB_FREEZE, &blob);
        let read = read_blob(BLOB_FREEZE, usize::MAX, |offset| {
            (offset == 0).then(|| inner(offset)).flatten()
        });
        assert_eq!(read, BlobRead::Failed(BLOB_TIMEOUT));

        // The wrong blob back.
        assert_eq!(
            read_blob(BLOB_FREEZE, usize::MAX, serve(BLOB_EVENTS, &blob)),
            BlobRead::Inconsistent
        );
    }

    #[test]
    fn read_blob_restarts_when_the_size_changes() {
        let short: Vec<u8> = vec![1; 30];
        let long: Vec<u8> = vec![2; 50];
        let mut calls = 0;
        let read = read_blob(BLOB_EVENTS, usize::MAX, |offset| {
            calls += 1;
            // The first read sees 30 bytes, then the log grows to 50.
            if calls == 1 {
                serve(BLOB_EVENTS, &short)(offset)
            } else {
                serve(BLOB_EVENTS, &long)(offset)
            }
        });
        assert_eq!(read, BlobRead::Data(long));
    }

    /// A freeze header: hid_wq and slow_wq stalled, unread and restored.
    fn freeze_header() -> Vec<u8> {
        let mut h = Vec::new();
        h.extend_from_slice(&FREEZE_MAGIC.to_le_bytes());
        h.extend_from_slice(&1u16.to_le_bytes());
        h.extend_from_slice(&200u16.to_le_bytes());
        h.extend_from_slice(&[1, 0x06, 0x05, 2]);
        h.extend_from_slice(&1_213_000u32.to_le_bytes());
        h.extend_from_slice(&0u32.to_le_bytes());
        h.extend_from_slice(&[0, 2, 3, 0]);
        h.extend_from_slice(&41u32.to_le_bytes());
        h.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04]);
        assert_eq!(h.len(), FREEZE_HEADER_SIZE);
        h.extend_from_slice(&[0xAA; 8]);
        h
    }

    #[test]
    fn freeze_header_decodes() {
        let h = parse_freeze_header(&freeze_header()).unwrap();
        assert_eq!(h.layout, 1);
        assert_eq!(h.total_len, 200);
        assert_eq!(h.kind, FreezeKind::Freeze);
        assert_eq!(h.stalled_names(), ["hid_wq", "slow_wq"]);
        assert!(h.unread());
        assert_eq!(h.flag_names(), ["unread", "restored from flash"]);
        assert_eq!(h.freezes, 2);
        // Bit 1 is unused.
        let unused = FreezeHeader { flags: 0x07, ..h.clone() };
        assert_eq!(unused.flag_names(), ["unread", "restored from flash"]);
        assert_eq!(h.uptime_ms, 1_213_000);
        assert_eq!(h.fatal_reason, 0);
        assert_eq!(h.version, [0, 2, 3]);
        assert_eq!(h.build, 41);
        assert_eq!(h.hash, [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04]);
    }

    #[test]
    fn freeze_header_rejects_short_and_bad_magic() {
        let h = freeze_header();
        assert_eq!(parse_freeze_header(&h[..FREEZE_HEADER_SIZE - 1]), None);
        let mut bad = h.clone();
        bad[0] ^= 1;
        assert_eq!(parse_freeze_header(&bad), None);
        let mut fatal = h;
        fatal[8] = 2;
        fatal[9] = 0;
        fatal[16] = 4;
        let fatal = parse_freeze_header(&fatal).unwrap();
        assert_eq!(fatal.kind, FreezeKind::Fatal);
        assert!(fatal.stalled_names().is_empty());
        assert_eq!(fatal.fatal_reason, 4);
    }
}
