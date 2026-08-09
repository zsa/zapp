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
/// Ask the board about itself: battery per half, what it can drive, what is
/// plugged in. Answered with `EVT_TELEMETRY`.
pub const CMD_GET_TELEMETRY: u8 = 0x0B;
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
/// Answer to `CMD_GET_TELEMETRY`. Its payload is binary and fills the report,
/// so it carries no STOP_BIT: a 0xFE inside it is data.
pub const EVT_TELEMETRY: u8 = 0x0C;
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
}

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
    fn from_mask(mask: u8) -> Option<Self> {
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

/// What a board says about itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
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
}

/// Events from a connected keyboard's raw HID interface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HidEvent {
    /// Device reported its layout: serial is "<layoutId>/<revisionId>".
    FirmwareVersion {
        layout_id: String,
        revision_id: String,
    },
    /// Feature-gating protocol version (0x03 = smart layer, 0x04 = RGB).
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
    if cmd == EVT_TELEMETRY {
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
}
