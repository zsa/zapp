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
pub const EVT_GET_PROTOCOL_VERSION: u8 = 0xFD;

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
    Key { column: u8, row: u8, pressed: bool },
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
    let end = data
        .iter()
        .position(|&b| b == STOP_BIT)
        .unwrap_or(data.len());
    Some((cmd, &data[1..end]))
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
}
