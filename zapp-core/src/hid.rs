//! ZSA keyboard raw-HID protocol: live key events, layer changes, pairing.
//!
//! ZSA boards expose a raw HID interface on usage page 0xFF60 (the QMK raw
//! HID page). Packets are 33 bytes: a report ID byte followed by a 32-byte
//! report whose first byte is a command/event opcode and whose remaining
//! bytes are parameters terminated by STOP_BIT (0xFE). This mirrors the Go
//! implementation in zsausb/device.go.
//!
//! The wire format itself lives in `zapp-hid-proto`, shared with transports
//! that cannot link hidapi. This module runs the handshake and the read loop
//! over any [`ReportLink`]; hidapi is the one it opens itself.

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use zapp_hid_proto::{CMD_GET_FW_VERSION, Packet, REPORT_SIZE, encode_packet, process_packet};

use crate::ZappError;

pub use zapp_hid_proto::{
    BATTERY_DETAIL_SIZE, BatteryDetail, BatteryReading, BleIdentity, CMD_GET_CHARGE_TELEMETRY,
    CMD_GET_PMIC_RAILS, CMD_GET_TELEMETRY, CMD_RGB_CONTROL, CMD_SET_LAYER, CMD_SET_RGB_ALL_LED,
    CMD_SET_RGB_LED, CMD_SET_STATUS_LED, CMD_STATUS_LED_CONTROL, CMD_UPDATE_BRIGHTNESS,
    ChargeState, ChargeTelemetry, ChargerFault, ChargerStatus, Charging, EsbLink, HalfDetail,
    HidEvent, Nav, NavReport,
    PMIC_RAILS_REPORT_VERSION, PMIC_RAILS_SIZE, PmicRails, PmicRailsReport, Source, Telemetry,
    USAGE_PAGE, VbusInStatus, decode_battery_detail, decode_pmic_rails, parse_pmic_rails_report,
};
pub use zapp_hid_proto::{
    CMD_GET_DIAG, DIAG_AGE_UNKNOWN, DIAG_FETCH, DIAG_NEVER, DIAG_PAGE_VERSION, DiagPage,
    DiagRecord, DiagStatus, LEFT_DIAG_BLE_HOST, LEFT_DIAG_STM32_LINK, LEFT_DIAG_STM32_SLEEP,
    LEFT_DIAG_STM32_TRRS, LEFT_DIAG_STM32_USB, LEFT_DIAG_THREADS, LEFT_DIAG_UART_RX_OFF,
    LEFT_DIAG_VBUS, LeftDiag, NRF_RESET_DOG0, NRF_RESET_DOG1, NRF_RESET_LOCKUP, NRF_RESET_SREQ,
    STM32_DIAG_BLE_HOST, STM32_DIAG_LINK_UP, STM32_DIAG_NRF_VBUS, STM32_DIAG_RIGHT_LEDS,
    STM32_DIAG_TRRS, STM32_DIAG_USB_HOST, STM32_DIAG_VBUS_SEEN, STM32_RESET_IWDG, STM32_RESET_WWDG,
    Stm32Diag, ThreadStall, assemble_diag_record, diag_page_count, diag_request, nrf_reset_causes,
    parse_left_diag, parse_stm32_diag, stm32_reset_causes,
};
pub use zapp_hid_proto::{
    CMD_GET_RADIO_INFO, LeftRadio, RADIO_INFO_PROTOCOL, RadioInfo, RadioVersion, RightRadio,
    has_radio_info,
};
pub use zapp_hid_proto::{CMD_GET_HOST_SLOTS, HOST_SLOTS_MAX, HostSlot, parse_host_slot};
pub use zapp_hid_proto::{
    BLOB_BUSY, BLOB_EVENTS, BLOB_FREEZE, BLOB_NO_LINK, BLOB_OK, BLOB_TIMEOUT, BlobChunk, BlobRead,
    CMD_FREEZE_SEEN, CMD_GET_BLOB, ERR_UNKNOWN_COMMAND, FREEZE_HEADER_SIZE, FreezeHeader,
    FreezeKind, blob_request, parse_freeze_header, read_blob,
};

/// Write packet size: report ID byte + 32-byte report.
const PACKET_SIZE: usize = REPORT_SIZE + 1;
const REPORT_ID: u8 = 0x00;
/// Poll interval when the non-blocking read returns nothing.
const IDLE_POLL: Duration = Duration::from_millis(20);

/// One 32-byte report each way, over whatever carries the protocol.
///
/// The reports are the same on every carrier: hidapi's raw HID interface, or
/// a GATT service that moves them one per write and one per notification.
pub trait ReportLink: Send + 'static {
    /// Sends one report.
    fn send(&mut self, report: &[u8; REPORT_SIZE]) -> io::Result<()>;

    /// The next report, or `None` once `timeout` elapses.
    fn receive(&mut self, timeout: Duration) -> io::Result<Option<Vec<u8>>>;
}

/// A live connection to a keyboard's raw HID protocol.
///
/// Spawns a reader thread that performs the handshake (firmware version,
/// pairing, protocol version) and streams events until dropped.
pub struct HidConnection {
    rx: mpsc::Receiver<HidEvent>,
    tx_cmd: mpsc::Sender<Packet>,
    running: Arc<AtomicBool>,
    reader: Option<thread::JoinHandle<()>>,
}

impl HidConnection {
    /// Open the raw HID interface (usage page 0xFF60) of the device with the
    /// given VID/PID.
    pub fn open(vid: u16, pid: u16) -> Result<Self, ZappError> {
        let api = hidapi::HidApi::new()?;

        let path = api
            .device_list()
            .find(|d| d.vendor_id() == vid && d.product_id() == pid && d.usage_page() == USAGE_PAGE)
            .map(|d| d.path().to_owned())
            .ok_or(ZappError::NoKeyboardFound)?;

        let device = api.open_path(&path)?;
        device.set_blocking_mode(false)?;

        Ok(Self::over(HidLink(device)))
    }

    /// Runs the protocol over `link`: starts the reader thread and kicks off
    /// the handshake.
    pub fn over(link: impl ReportLink) -> Self {
        let (tx, rx) = mpsc::channel();
        let (tx_cmd, rx_cmd) = mpsc::channel::<Packet>();
        let running = Arc::new(AtomicBool::new(true));

        let thread_running = running.clone();
        let reader = thread::spawn(move || {
            read_loop(link, tx, rx_cmd, thread_running);
        });

        // Kick off the handshake; the read loop continues it on responses.
        let conn = Self {
            rx,
            tx_cmd,
            running,
            reader: Some(reader),
        };
        conn.send(CMD_GET_FW_VERSION, &[]);
        conn
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

    /// Stops the reader thread and waits for it, so the link is dropped
    /// before this returns.
    pub fn close(mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

impl Drop for HidConnection {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

/// Frame a report for hidapi: report ID byte, then the 32-byte report.
fn hid_buffer(report: &[u8; REPORT_SIZE]) -> [u8; PACKET_SIZE] {
    let mut buf = [0u8; PACKET_SIZE];
    buf[0] = REPORT_ID;
    buf[1..].copy_from_slice(report);
    buf
}

/// hidapi's raw HID interface, opened non-blocking.
struct HidLink(hidapi::HidDevice);

impl ReportLink for HidLink {
    fn send(&mut self, report: &[u8; REPORT_SIZE]) -> io::Result<()> {
        self.0.write(&hid_buffer(report)).map_err(io::Error::other)?;
        Ok(())
    }

    fn receive(&mut self, timeout: Duration) -> io::Result<Option<Vec<u8>>> {
        let mut buf = [0u8; PACKET_SIZE];
        match self.0.read(&mut buf).map_err(io::Error::other)? {
            0 => {
                thread::sleep(timeout);
                Ok(None)
            }
            n => Ok(Some(buf[..n].to_vec())),
        }
    }
}

fn read_loop(
    mut link: impl ReportLink,
    tx: mpsc::Sender<HidEvent>,
    rx_cmd: mpsc::Receiver<Packet>,
    running: Arc<AtomicBool>,
) {
    while running.load(Ordering::Relaxed) {
        // Flush queued outgoing commands.
        while let Ok((cmd, params)) = rx_cmd.try_recv() {
            if link.send(&encode_packet(cmd, &params)).is_err() {
                let _ = tx.send(HidEvent::Disconnected);
                return;
            }
        }

        match link.receive(IDLE_POLL) {
            Ok(None) => {}
            Ok(Some(report)) => {
                let (event, replies) = process_packet(&report);
                for (cmd, params) in replies {
                    if link.send(&encode_packet(cmd, &params)).is_err() {
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
            Err(err) => {
                log::debug!("report link closed: {err}");
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
        let buf = hid_buffer(&encode_packet(CMD_SET_LAYER, &[1, 3]));
        assert_eq!(buf.len(), PACKET_SIZE);
        assert_eq!(&buf[..4], &[REPORT_ID, CMD_SET_LAYER, 1, 3]);
        assert!(buf[4..].iter().all(|&b| b == 0));
    }

    /// Answers the firmware-version request with a serial, and records every
    /// report written.
    struct Scripted {
        sent: mpsc::Sender<[u8; REPORT_SIZE]>,
        pending: Vec<Vec<u8>>,
    }

    impl ReportLink for Scripted {
        fn send(&mut self, report: &[u8; REPORT_SIZE]) -> io::Result<()> {
            if report[0] == CMD_GET_FW_VERSION {
                let mut answer = b"\x00abc/def".to_vec();
                answer.push(zapp_hid_proto::STOP_BIT);
                self.pending.push(answer);
            }
            let _ = self.sent.send(*report);
            Ok(())
        }

        fn receive(&mut self, timeout: Duration) -> io::Result<Option<Vec<u8>>> {
            if self.pending.is_empty() {
                thread::sleep(timeout);
                return Ok(None);
            }
            Ok(Some(self.pending.remove(0)))
        }
    }

    #[test]
    fn any_link_carries_the_same_handshake() {
        let (sent, written) = mpsc::channel();
        let connection = HidConnection::over(Scripted {
            sent,
            pending: Vec::new(),
        });

        assert_eq!(
            connection.recv_timeout(Duration::from_secs(2)),
            Some(HidEvent::FirmwareVersion {
                layout_id: "abc".into(),
                revision_id: "def".into(),
            })
        );
        let opcodes: Vec<u8> = (0..3)
            .map(|_| written.recv_timeout(Duration::from_secs(2)).unwrap()[0])
            .collect();
        assert_eq!(
            opcodes,
            [
                CMD_GET_FW_VERSION,
                zapp_hid_proto::CMD_INIT_PAIRING,
                zapp_hid_proto::CMD_GET_PROTOCOL_VERSION
            ]
        );
    }

    /// Idle until dropped, then says so.
    struct Watched(mpsc::Sender<()>);

    impl ReportLink for Watched {
        fn send(&mut self, _: &[u8; REPORT_SIZE]) -> io::Result<()> {
            Ok(())
        }

        fn receive(&mut self, timeout: Duration) -> io::Result<Option<Vec<u8>>> {
            thread::sleep(timeout);
            Ok(None)
        }
    }

    impl Drop for Watched {
        fn drop(&mut self) {
            let _ = self.0.send(());
        }
    }

    #[test]
    fn close_drops_the_link_before_returning() {
        let (dropped, watch) = mpsc::channel();
        HidConnection::over(Watched(dropped)).close();
        assert_eq!(watch.try_recv(), Ok(()));
    }

    struct Broken;

    impl ReportLink for Broken {
        fn send(&mut self, _: &[u8; REPORT_SIZE]) -> io::Result<()> {
            Err(io::Error::other("gone"))
        }

        fn receive(&mut self, _: Duration) -> io::Result<Option<Vec<u8>>> {
            Err(io::Error::other("gone"))
        }
    }

    #[test]
    fn a_link_that_fails_ends_the_connection() {
        let connection = HidConnection::over(Broken);
        assert_eq!(
            connection.recv_timeout(Duration::from_secs(2)),
            Some(HidEvent::Disconnected)
        );
    }
}
