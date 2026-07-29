//! ZSA keyboard raw-HID protocol: live key events, layer changes, pairing.
//!
//! ZSA boards expose a raw HID interface on usage page 0xFF60 (the QMK raw
//! HID page). Packets are 33 bytes: a report ID byte followed by a 32-byte
//! report whose first byte is a command/event opcode and whose remaining
//! bytes are parameters terminated by STOP_BIT (0xFE). This mirrors the Go
//! implementation in zsausb/device.go.
//!
//! The wire format itself lives in `zapp-hid-proto`, shared with transports
//! that cannot link hidapi; this module is the hidapi transport around it.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use zapp_hid_proto::{CMD_GET_FW_VERSION, Packet, REPORT_SIZE, encode_packet, process_packet};

use crate::ZappError;

pub use zapp_hid_proto::{
    CMD_RGB_CONTROL, CMD_SET_LAYER, CMD_SET_RGB_ALL_LED, CMD_SET_RGB_LED, CMD_SET_STATUS_LED,
    CMD_STATUS_LED_CONTROL, CMD_UPDATE_BRIGHTNESS, HidEvent, USAGE_PAGE,
};

/// Write packet size: report ID byte + 32-byte report.
const PACKET_SIZE: usize = REPORT_SIZE + 1;
const REPORT_ID: u8 = 0x00;
/// Poll interval when the non-blocking read returns nothing.
const IDLE_POLL: Duration = Duration::from_millis(20);

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

/// Frame a report for hidapi: report ID byte, then the 32-byte report.
fn hid_buffer(cmd: u8, params: &[u8]) -> [u8; PACKET_SIZE] {
    let mut buf = [0u8; PACKET_SIZE];
    buf[0] = REPORT_ID;
    buf[1..].copy_from_slice(&encode_packet(cmd, params));
    buf
}

fn write_packet(device: &hidapi::HidDevice, cmd: u8, params: &[u8]) -> Result<(), ZappError> {
    device.write(&hid_buffer(cmd, params))?;
    Ok(())
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

    #[test]
    fn hid_buffer_prepends_report_id() {
        let buf = hid_buffer(CMD_SET_LAYER, &[1, 3]);
        assert_eq!(buf.len(), PACKET_SIZE);
        assert_eq!(&buf[..4], &[REPORT_ID, CMD_SET_LAYER, 1, 3]);
        assert!(buf[4..].iter().all(|&b| b == 0));
    }
}
