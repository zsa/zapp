//! ZSA keyboard raw-HID protocol: live key events, layer changes, pairing.
//!
//! ZSA boards expose a raw HID interface on usage page 0xFF60 (the QMK raw
//! HID page). Packets are 33 bytes: a report ID byte followed by a 32-byte
//! report whose first byte is a command/event opcode and whose remaining
//! bytes are parameters terminated by STOP_BIT (0xFE). This mirrors the Go
//! implementation in zsausb/device.go.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use crate::ZappError;

/// HID usage page of the ZSA/QMK raw interface.
pub const USAGE_PAGE: u16 = 0xFF60;
/// Write packet size: report ID byte + 32-byte report.
const PACKET_SIZE: usize = 33;
/// Terminates the parameter list of incoming packets.
const STOP_BIT: u8 = 0xFE;
const REPORT_ID: u8 = 0x00;
/// Poll interval when the non-blocking read returns nothing.
const IDLE_POLL: Duration = Duration::from_millis(20);

// Host → device commands
const CMD_GET_FW_VERSION: u8 = 0x00;
const CMD_INIT_PAIRING: u8 = 0x01;
const CMD_VALIDATE_PAIRING: u8 = 0x02;
pub const CMD_SET_LAYER: u8 = 0x04;
pub const CMD_RGB_CONTROL: u8 = 0x05;
pub const CMD_SET_RGB_LED: u8 = 0x06;
pub const CMD_SET_STATUS_LED: u8 = 0x07;
pub const CMD_UPDATE_BRIGHTNESS: u8 = 0x08;
pub const CMD_SET_RGB_ALL_LED: u8 = 0x09;
pub const CMD_STATUS_LED_CONTROL: u8 = 0x0A;
const CMD_GET_PROTOCOL_VERSION: u8 = 0xFE;

// Device → host events
const EVT_GET_FIRMWARE_VERSION: u8 = 0x00;
const EVT_PAIRING_INIT: u8 = 0x01;
const EVT_PAIRING_KEY_INPUT: u8 = 0x02;
const EVT_PAIRING_FAIL: u8 = 0x03;
const EVT_PAIRING_SUCCESS: u8 = 0x04;
const EVT_LAYER_CHANGE: u8 = 0x05;
const EVT_KEY_PRESSED: u8 = 0x06;
const EVT_KEY_RELEASED: u8 = 0x07;
const EVT_TOGGLE_SMARTLAYER: u8 = 0x09;
const EVT_TRIGGER_SMARTLAYER: u8 = 0x0A;
const EVT_GET_PROTOCOL_VERSION: u8 = 0xFD;

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
type Packet = (u8, Vec<u8>);

/// A live connection to a keyboard's raw HID interface.
///
/// Spawns a reader thread that performs the handshake (firmware version,
/// pairing, protocol version) and streams events until dropped.
pub struct HidConnection {
    rx: mpsc::Receiver<HidEvent>,
    tx_cmd: mpsc::Sender<Packet>,
    running: Arc<AtomicBool>,
}

impl HidConnection {
    /// Open the raw HID interface (usage page 0xFF60) of the device with the
    /// given VID/PID.
    pub fn open(vid: u16, pid: u16) -> Result<Self, ZappError> {
        let api = hidapi::HidApi::new()?;

        let path = api
            .device_list()
            .find(|d| {
                d.vendor_id() == vid && d.product_id() == pid && d.usage_page() == USAGE_PAGE
            })
            .map(|d| d.path().to_owned())
            .ok_or(ZappError::NoKeyboardFound)?;

        let device = api.open_path(&path)?;
        device.set_blocking_mode(false)?;

        let (tx, rx) = mpsc::channel();
        let (tx_cmd, rx_cmd) = mpsc::channel::<Packet>();
        let running = Arc::new(AtomicBool::new(true));

        let thread_running = running.clone();
        thread::spawn(move || {
            read_loop(device, tx, rx_cmd, thread_running);
        });

        // Kick off the handshake; the read loop continues it on responses.
        let conn = Self {
            rx,
            tx_cmd,
            running,
        };
        conn.send(CMD_GET_FW_VERSION, &[]);
        Ok(conn)
    }

    /// Queue a command packet for the device.
    pub fn send(&self, cmd: u8, params: &[u8]) {
        let _ = self.tx_cmd.send((cmd, params.to_vec()));
    }

    /// Next pending event, if any.
    pub fn try_recv(&self) -> Option<HidEvent> {
        self.rx.try_recv().ok()
    }

    /// Block until the next event is available.
    pub fn recv(&self) -> Option<HidEvent> {
        self.rx.recv().ok()
    }

    /// Block until the next event is available or `timeout` elapses.
    pub fn recv_timeout(&self, timeout: Duration) -> Option<HidEvent> {
        self.rx.recv_timeout(timeout).ok()
    }
}

impl Drop for HidConnection {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

/// Build the outgoing report: report ID, opcode, then zero-padded params.
fn encode_packet(cmd: u8, params: &[u8]) -> [u8; PACKET_SIZE] {
    let mut buf = [0u8; PACKET_SIZE];
    buf[0] = REPORT_ID;
    buf[1] = cmd;
    let n = params.len().min(PACKET_SIZE - 2);
    buf[2..2 + n].copy_from_slice(&params[..n]);
    buf
}

fn write_packet(device: &hidapi::HidDevice, cmd: u8, params: &[u8]) -> Result<(), ZappError> {
    device.write(&encode_packet(cmd, params))?;
    Ok(())
}

/// Split an incoming report into (event, params). Params run until STOP_BIT.
/// Quirk: some firmware sends STOP_BIT as the first byte to announce the
/// protocol version.
fn parse_packet(data: &[u8]) -> Option<(u8, &[u8])> {
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
fn process_packet(data: &[u8]) -> (Option<HidEvent>, Vec<Packet>) {
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

fn read_loop(
    device: hidapi::HidDevice,
    tx: mpsc::Sender<HidEvent>,
    rx_cmd: mpsc::Receiver<Packet>,
    running: Arc<AtomicBool>,
) {
    let mut buf = [0u8; PACKET_SIZE];

    while running.load(Ordering::Relaxed) {
        // Flush queued outgoing commands.
        while let Ok((cmd, params)) = rx_cmd.try_recv() {
            if write_packet(&device, cmd, &params).is_err() {
                let _ = tx.send(HidEvent::Disconnected);
                return;
            }
        }

        match device.read(&mut buf) {
            Ok(0) => thread::sleep(IDLE_POLL),
            Ok(n) => {
                let (event, replies) = process_packet(&buf[..n]);
                for (cmd, params) in replies {
                    if write_packet(&device, cmd, &params).is_err() {
                        let _ = tx.send(HidEvent::Disconnected);
                        return;
                    }
                }
                if let Some(event) = event
                    && tx.send(event).is_err()
                {
                    return; // receiver gone
                }
            }
            Err(_) => {
                let _ = tx.send(HidEvent::Disconnected);
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 32-byte incoming report from an opcode and params.
    fn report(cmd: u8, params: &[u8]) -> [u8; 32] {
        let mut data = [0u8; 32];
        data[0] = cmd;
        data[1..1 + params.len()].copy_from_slice(params);
        data[1 + params.len()] = STOP_BIT;
        data
    }

    #[test]
    fn encode_frames_report_id_and_command() {
        let packet = encode_packet(CMD_SET_LAYER, &[1, 3]);
        assert_eq!(packet.len(), PACKET_SIZE);
        assert_eq!(&packet[..4], &[REPORT_ID, CMD_SET_LAYER, 1, 3]);
        assert!(packet[4..].iter().all(|&b| b == 0));
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
            assert_eq!(packet[0], REPORT_ID);
            assert_eq!(packet[1], cmd);
            assert_eq!(&packet[2..2 + params.len()], params);
            assert!(packet[2 + params.len()..].iter().all(|&b| b == 0));
        }
    }

    #[test]
    fn encode_truncates_oversized_params() {
        let packet = encode_packet(CMD_SET_RGB_ALL_LED, &[0xAB; 64]);
        assert_eq!(packet[1], CMD_SET_RGB_ALL_LED);
        assert!(packet[2..].iter().all(|&b| b == 0xAB));
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
        let mut data = [0u8; 32];
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
        // The reply goes out as [report id, 0x02, key...]
        let packet = encode_packet(replies[0].0, &replies[0].1);
        assert_eq!(
            &packet[..8],
            &[REPORT_ID, CMD_VALIDATE_PAIRING, 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02]
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
        let mut data = [0u8; 32];
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
