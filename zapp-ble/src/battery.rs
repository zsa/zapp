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

/// Charge left, per half, as a percentage.
///
/// `None` is "not said": a board with one battery reports only `keyboard`, and
/// the right half is unknown while the halves are not linked.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Battery {
    pub keyboard: Option<u8>,
    pub left: Option<u8>,
    pub right: Option<u8>,
}

impl Battery {
    /// Whether anything at all was reported.
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    pub fn set(&mut self, half: Half, level: u8) {
        let slot = match half {
            Half::Keyboard => &mut self.keyboard,
            Half::Left => &mut self.left,
            Half::Right => &mut self.right,
        };
        *slot = level_from(level);
    }
}

/// A raw battery level, or `None` for the not-yet-known sentinel and anything
/// outside the range the spec allows.
pub fn level_from(raw: u8) -> Option<u8> {
    (raw != UNKNOWN && raw <= 100).then_some(raw)
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
        assert_eq!(level_from(0), Some(0));
        assert_eq!(level_from(70), Some(70));
        assert_eq!(level_from(100), Some(100));
        assert_eq!(level_from(UNKNOWN), None);
        // Anything else out of range is equally not a percentage.
        assert_eq!(level_from(101), None);
    }

    #[test]
    fn an_unlinked_right_half_leaves_its_level_unsaid() {
        let mut battery = Battery::default();
        battery.set(Half::Left, 70);
        battery.set(Half::Right, UNKNOWN);
        battery.set(Half::Keyboard, 70);

        assert_eq!(battery.left, Some(70));
        assert_eq!(battery.right, None);
        assert_eq!(battery.keyboard, Some(70));
        assert!(!battery.is_empty());
    }
}
