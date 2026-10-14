//! Charge left in a board and, on a split one, in each half.

use zapp_core::hid::{
    decode_battery_detail, decode_pmic_rails, BatteryDetail, NavReport, PmicRails,
    BATTERY_DETAIL_SIZE, PMIC_RAILS_SIZE,
};

/// Battery Service, its level characteristic, and the descriptor saying which
/// half an instance speaks for.
pub(crate) const BATTERY_SERVICE: u128 = 0x0000180f_0000_1000_8000_00805f9b34fb;
pub(crate) const BATTERY_LEVEL: u128 = 0x00002a19_0000_1000_8000_00805f9b34fb;
pub(crate) const PRESENTATION_FORMAT: u128 = 0x00002904_0000_1000_8000_00805f9b34fb;

/// The left half's battery telemetry service, read-only characteristic: what
/// the two halves' gauges and chargers read, built on demand.
pub(crate) const CHARGE_DETAIL: u128 = 0x12345678_1234_5678_1234_56789abcdef2;

/// The level a board publishes when it has no reading: outside the 0..100 the
/// spec allows, so a host shows "unknown" rather than a made-up number.
const UNKNOWN: u8 = 0xFF;

/// GATT Namespace Descriptors (SIG namespace) naming what a characteristic
/// describes. A split keyboard publishes one Battery Service instance per half
/// plus a combined one, told apart by these.
const MAIN: u16 = 0x0106;
const LEFT: u16 = 0x010D;
const RIGHT: u16 = 0x010E;

/// Which half a Battery Service instance speaks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Half {
    /// The board as a whole, which is the lower of the two halves.
    Keyboard,
    Left,
    Right,
}

/// What is known about one battery.
///
/// Three states, not two, because "the board has no such battery" and "the
/// board has one and would not say" are different answers and a host that
/// prints them the same way makes a missing half look like a flat one.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Level {
    /// No such Battery Service instance. A board with one battery has no left
    /// and no right, and a board nothing is connected to publishes none at all.
    #[default]
    Absent,
    /// The instance is there and its level is not: the board published the
    /// not-known sentinel, or it could not be read this time round.
    Unknown,
    Percent(u8),
}

impl Level {
    /// What a raw characteristic byte says. The sentinel and anything outside
    /// the range the spec allows are a battery that would not say.
    pub fn from_raw(raw: u8) -> Self {
        match raw != UNKNOWN && raw <= 100 {
            true => Self::Percent(raw),
            false => Self::Unknown,
        }
    }

    /// The percentage, for the callers that only care whether there is a
    /// number — the update gate treats absent and unknown alike.
    pub fn percent(self) -> Option<u8> {
        match self {
            Self::Percent(level) => Some(level),
            _ => None,
        }
    }

    pub fn is_absent(self) -> bool {
        self == Self::Absent
    }
}

/// Charge left, per half.
///
/// A board with one battery reports only `keyboard`; a split one reports all
/// three, and leaves the right unknown while the halves are not linked.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Battery {
    pub keyboard: Level,
    pub left: Level,
    pub right: Level,
}

impl Battery {
    /// Whether anything at all was reported.
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    pub fn set(&mut self, half: Half, level: Level) {
        let slot = match half {
            Half::Keyboard => &mut self.keyboard,
            Half::Left => &mut self.left,
            Half::Right => &mut self.right,
        };
        *slot = level;
    }
}

/// Charge-detail format this host understands. Refused whole when it is
/// anything else, for the reason the HID payloads are: a version bump means a
/// field that already shipped changed meaning.
pub const CHARGE_DETAIL_VERSION: u8 = 1;

/// Byte offsets in the charge-detail characteristic: `version`, `len`, one
/// detail block per half, then one rails block per half. Same header as the HID
/// payloads carry, so a board that grows the value says so in `len` rather than
/// in the version — which is how the rails blocks arrived.
const DETAIL_LEFT: usize = 2;
const DETAIL_RIGHT: usize = DETAIL_LEFT + BATTERY_DETAIL_SIZE;
const RAILS_LEFT: usize = DETAIL_RIGHT + BATTERY_DETAIL_SIZE;
const RAILS_RIGHT: usize = RAILS_LEFT + PMIC_RAILS_SIZE;
/// Build, left, right, bus nav masks, after the scan and nRF diag blocks.
const NAV: usize = 86;

/// What one board's charge-detail characteristic says: the volts, milliamps
/// and degrees behind the two Battery Service levels.
///
/// A half the board did not fill in comes back with every field unknown, which
/// is the ordinary case for the right one while the halves are not linked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChargeDetail {
    pub left: BatteryDetail,
    pub right: BatteryDetail,
    /// What the left half's PMIC reads on its input and system rails. `None`
    /// from a board whose firmware predates the block, and from a half that has
    /// not taken a reading yet.
    pub left_rails: Option<PmicRails>,
    pub right_rails: Option<PmicRails>,
    /// Pointing devices as the STM32 reports them. `None` from a board whose
    /// firmware predates the block.
    pub nav: Option<NavReport>,
}

impl ChargeDetail {
    /// Decode the characteristic's value, or `None` if it is not one this host
    /// can read.
    pub fn decode(value: &[u8]) -> Option<Self> {
        if *value.first()? != CHARGE_DETAIL_VERSION {
            return None;
        }
        let len = *value.get(1)? as usize;

        // A block is whole or it is nothing: half a block would be read field
        // by field off bytes the board never sent. Both the board's `len` and
        // what actually arrived have to reach past its end.
        let whole = |at: usize, size: usize| {
            (at + size <= len + 2)
                .then(|| value.get(at..at + size))
                .flatten()
        };
        let block = |at: usize| match whole(at, BATTERY_DETAIL_SIZE) {
            Some(bytes) => decode_battery_detail(bytes),
            None => BatteryDetail::default(),
        };
        let rails = |at: usize| whole(at, PMIC_RAILS_SIZE).and_then(decode_pmic_rails);

        Some(Self {
            left: block(DETAIL_LEFT),
            right: block(DETAIL_RIGHT),
            left_rails: rails(RAILS_LEFT),
            right_rails: rails(RAILS_RIGHT),
            nav: whole(NAV, NavReport::SIZE).and_then(NavReport::decode),
        })
    }
}

/// The half a Characteristic Presentation Format descriptor names.
///
/// Seven bytes: format, exponent, unit, namespace, then the description at
/// offset 5, little-endian. The description is what distinguishes the
/// instances; the user description string beside it says the same thing in
/// words, which is a worse thing to match on.
pub fn half_from_format(descriptor: &[u8]) -> Option<Half> {
    let description = u16::from_le_bytes([*descriptor.get(5)?, *descriptor.get(6)?]);
    match description {
        MAIN => Some(Half::Keyboard),
        LEFT => Some(Half::Left),
        RIGHT => Some(Half::Right),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::hid::Nav;

    #[test]
    fn the_format_descriptor_names_the_half() {
        // Straight off the wire: percent (0x27AD), SIG namespace, then the id.
        assert_eq!(
            half_from_format(&[0x04, 0x00, 0xAD, 0x27, 0x01, 0x06, 0x01]),
            Some(Half::Keyboard)
        );
        assert_eq!(
            half_from_format(&[0x04, 0x00, 0xAD, 0x27, 0x01, 0x0D, 0x01]),
            Some(Half::Left)
        );
        assert_eq!(
            half_from_format(&[0x04, 0x00, 0xAD, 0x27, 0x01, 0x0E, 0x01]),
            Some(Half::Right)
        );
    }

    #[test]
    fn a_format_descriptor_naming_something_else_is_no_half() {
        assert_eq!(
            half_from_format(&[0x04, 0x00, 0xAD, 0x27, 0x01, 0x00, 0x00]),
            None
        );
        assert_eq!(half_from_format(&[0x04, 0x00]), None, "too short to read");
        assert_eq!(half_from_format(&[]), None);
    }

    #[test]
    fn the_unknown_sentinel_is_not_a_level() {
        assert_eq!(Level::from_raw(0), Level::Percent(0));
        assert_eq!(Level::from_raw(70), Level::Percent(70));
        assert_eq!(Level::from_raw(100), Level::Percent(100));
        assert_eq!(Level::from_raw(UNKNOWN), Level::Unknown);
        // Anything else out of range is equally not a percentage.
        assert_eq!(Level::from_raw(101), Level::Unknown);
    }

    #[test]
    fn a_half_that_would_not_say_is_not_a_half_that_is_not_there() {
        let mut battery = Battery::default();
        battery.set(Half::Left, Level::from_raw(70));
        battery.set(Half::Right, Level::from_raw(UNKNOWN));
        battery.set(Half::Keyboard, Level::from_raw(70));

        assert_eq!(battery.left, Level::Percent(70));
        // The right half published an instance, so it exists and is unknown —
        // where a one-battery board would have left it absent.
        assert_eq!(battery.right, Level::Unknown);
        assert!(!battery.right.is_absent());
        assert_eq!(battery.keyboard, Level::Percent(70));
        assert!(!battery.is_empty());
    }

    #[test]
    fn a_board_nothing_is_connected_to_reports_nothing_at_all() {
        let battery = Battery::default();
        assert!(battery.is_empty());
        assert!(battery.left.is_absent());
        assert!(battery.right.is_absent());
        assert!(battery.keyboard.is_absent());
        assert_eq!(battery.left.percent(), None);
    }

    /// One detail block as the board fills it in: volts, milliamps,
    /// hundredths of a degree, the two estimates, then the three registers.
    fn block(voltage_mv: u16, current_ma: i16, tte_min: u16) -> Vec<u8> {
        let mut block = Vec::new();
        block.extend_from_slice(&voltage_mv.to_le_bytes());
        block.extend_from_slice(&current_ma.to_le_bytes());
        block.extend_from_slice(&2790i16.to_le_bytes());
        block.extend_from_slice(&tte_min.to_le_bytes());
        block.extend_from_slice(&0xFFFFu16.to_le_bytes()); // not charging: no ttf
        block.extend_from_slice(&[0x01, 0x00, 0x00]);
        block
    }

    /// One rails block: the present flag, the two rails, then the two
    /// registers read back.
    fn rails(vbus_mv: u16, vsys_mv: u16, vbusin: u8, chg_enable: u8) -> Vec<u8> {
        let mut block = vec![0x01];
        block.extend_from_slice(&vbus_mv.to_le_bytes());
        block.extend_from_slice(&vsys_mv.to_le_bytes());
        block.push(vbusin);
        block.push(chg_enable);
        block
    }

    /// The characteristic's value, as a linked board on battery serves it.
    fn charge_value() -> Vec<u8> {
        let mut value = vec![CHARGE_DETAIL_VERSION, 26];
        value.extend_from_slice(&block(4020, -45, 750));
        value.extend_from_slice(&block(3980, -38, 845));
        value
    }

    /// The same, from a board new enough to append both rails blocks.
    fn charge_value_with_rails() -> Vec<u8> {
        let mut value = charge_value();
        value[1] = 40;
        value.extend_from_slice(&rails(4620, 4550, 0x21, 0x01));
        value.extend_from_slice(&rails(4480, 4400, 0x01, 0x00));
        value
    }

    #[test]
    fn the_charge_characteristic_carries_both_halves() {
        let detail = ChargeDetail::decode(&charge_value()).unwrap();
        assert_eq!(detail.left.voltage_mv, Some(4020));
        assert_eq!(detail.left.current_ma, Some(-45));
        assert_eq!(detail.left.tte_min, Some(750));
        assert_eq!(detail.left.ttf_min, None, "not charging");
        assert_eq!(detail.right.voltage_mv, Some(3980));
        assert_eq!(detail.right.tte_min, Some(845));
    }

    #[test]
    fn a_charge_version_this_host_does_not_know_is_refused_whole() {
        let mut value = charge_value();
        value[0] = CHARGE_DETAIL_VERSION + 1;
        assert_eq!(ChargeDetail::decode(&value), None);
        assert_eq!(ChargeDetail::decode(&[]), None);
        assert_eq!(ChargeDetail::decode(&[CHARGE_DETAIL_VERSION]), None);
    }

    #[test]
    fn a_half_the_board_did_not_send_is_unknown_rather_than_flat() {
        // The board says it filled in the left block only, which is what an
        // unlinked right half comes to.
        let mut value = charge_value();
        value[1] = 13;

        let detail = ChargeDetail::decode(&value).unwrap();
        assert_eq!(detail.left.voltage_mv, Some(4020));
        assert_eq!(detail.right, BatteryDetail::default());
    }

    #[test]
    fn a_value_that_arrived_short_is_not_read_off_the_end() {
        // The board claims both blocks and only the left one arrived.
        let value = charge_value();
        let detail = ChargeDetail::decode(&value[..20]).unwrap();
        assert_eq!(detail.left.voltage_mv, Some(4020));
        assert_eq!(detail.right, BatteryDetail::default());
    }

    #[test]
    fn the_charge_characteristic_carries_both_halves_rails() {
        let detail = ChargeDetail::decode(&charge_value_with_rails()).unwrap();

        let left = detail.left_rails.unwrap();
        assert_eq!(left.vbus_mv, 4620);
        assert_eq!(left.vsys_mv, 4550);
        assert_eq!(left.vbusin.names(), vec!["present", "out active"]);
        assert_eq!(left.charger_enabled, Some(true));

        let right = detail.right_rails.unwrap();
        assert_eq!(right.vbus_mv, 4480);
        assert_eq!(right.vbusin.names(), vec!["present"]);
        assert_eq!(right.charger_enabled, Some(false));
        // The detail blocks are read the same way as before.
        assert_eq!(detail.left.voltage_mv, Some(4020));
        assert_eq!(detail.right.voltage_mv, Some(3980));
    }

    #[test]
    fn a_board_that_sends_no_rails_has_none() {
        // An older firmware: `len` stops at the two detail blocks.
        let detail = ChargeDetail::decode(&charge_value()).unwrap();
        assert_eq!(detail.left_rails, None);
        assert_eq!(detail.right_rails, None);

        // And one that claims them but arrived short is not read off the end.
        let value = charge_value_with_rails();
        let cut = ChargeDetail::decode(&value[..value.len() - 4]).unwrap();
        assert_eq!(cut.left_rails.unwrap().vbus_mv, 4620);
        assert_eq!(cut.right_rails, None);
    }

    #[test]
    fn a_half_that_has_not_measured_its_rails_is_not_a_half_at_zero_volts() {
        let mut value = charge_value_with_rails();
        // The right half's present flag, clear: the block is there and empty.
        value[35] = 0;

        let detail = ChargeDetail::decode(&value).unwrap();
        assert!(detail.left_rails.is_some());
        assert_eq!(detail.right_rails, None);
    }

    /// The value as a board of the 84-byte generation serves it: rails, then
    /// scan and nRF diag blocks this host does not read, filled with markers.
    fn charge_value_with_diag() -> Vec<u8> {
        let mut value = charge_value_with_rails();
        value[1] = 84;
        value.extend((42..86).map(|i| i as u8));
        assert_eq!(value.len(), 86);
        value
    }

    /// The same, grown by the build, left, right, bus nav block.
    fn charge_value_with_nav(nav: [u8; 4]) -> Vec<u8> {
        let mut value = charge_value_with_diag();
        value[1] = 88;
        value.extend_from_slice(&nav);
        assert_eq!(value.len(), 90);
        value
    }

    #[test]
    fn a_board_from_before_the_nav_block_has_none() {
        let detail = ChargeDetail::decode(&charge_value_with_diag()).unwrap();
        assert_eq!(detail.nav, None);
        assert_eq!(detail.left.voltage_mv, Some(4020));
        assert_eq!(detail.right_rails.unwrap().vbus_mv, 4480);
    }

    #[test]
    fn the_nav_block_is_read_after_the_diag_blocks() {
        let ball = Nav {
            trackball: true,
            trackpad: false,
        };
        let pad = Nav {
            trackball: false,
            trackpad: true,
        };
        let both = Nav {
            trackball: true,
            trackpad: true,
        };

        // Cable unplugged: ball on the left, right half not linked.
        let detail = ChargeDetail::decode(&charge_value_with_nav([0x03, 0x01, 0xFF, 0x00])).unwrap();
        assert_eq!(
            detail.nav,
            Some(NavReport {
                build: Some(both),
                left: Some(ball),
                right: None,
                bus: Some(Nav::default()),
            })
        );

        // Cable plugged: a pad on the shared bus, side unknown.
        let detail = ChargeDetail::decode(&charge_value_with_nav([0x03, 0xFF, 0xFF, 0x02])).unwrap();
        assert_eq!(
            detail.nav,
            Some(NavReport {
                build: Some(both),
                left: None,
                right: None,
                bus: Some(pad),
            })
        );
        // Everything before it reads as before.
        assert_eq!(detail.left.voltage_mv, Some(4020));
        assert_eq!(detail.right_rails.unwrap().vbus_mv, 4480);
    }

    #[test]
    fn a_nav_block_that_arrived_short_is_not_read() {
        let value = charge_value_with_nav([0x01, 0x01, 0x00, 0x00]);
        assert_eq!(ChargeDetail::decode(&value[..89]).unwrap().nav, None);
    }

    #[test]
    fn only_a_percentage_is_a_number() {
        assert_eq!(Level::Percent(36).percent(), Some(36));
        assert_eq!(Level::Unknown.percent(), None);
        assert_eq!(Level::Absent.percent(), None);
    }
}
