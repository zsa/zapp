//! Charge left in a board and, on a split one, in each half.

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

    #[test]
    fn only_a_percentage_is_a_number() {
        assert_eq!(Level::Percent(36).percent(), Some(36));
        assert_eq!(Level::Unknown.percent(), None);
        assert_eq!(Level::Absent.percent(), None);
    }
}
