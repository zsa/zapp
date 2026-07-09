use std::time::Duration;

use hidapi::{HidApi, HidDevice};

use crate::ZappError;
use crate::device::BootloaderDevice;
use crate::device::ids::{HALFKAY_PID, HALFKAY_VID};
use crate::firmware::Firmware;

use super::FlashProgress;

const ERGODOX_MEM_SIZE: usize = 32256;
const ERGODOX_SECTOR_SIZE: usize = 128;
const PACKET_RETRIES: usize = 5;

/// Flash a device using the HALFKAY protocol (legacy Ergodox EZ).
///
/// Uses hidapi instead of the device's nusb handle so the transport works on
/// Windows, where the HALFKAY bootloader enumerates as HID and the kernel HID
/// driver prevents libusb-style access.
pub fn flash_halfkay(
    _device: &BootloaderDevice,
    firmware: &Firmware,
    on_progress: &dyn Fn(FlashProgress),
) -> Result<(), ZappError> {
    let firmware_data = match firmware {
        Firmware::IntelHex { data } => data.as_slice(),
        _ => {
            return Err(ZappError::InvalidFirmware(
                "HALFKAY requires Intel HEX firmware (.hex file)".into(),
            ));
        }
    };

    let api = HidApi::new()?;
    let hid = api.open(HALFKAY_VID, HALFKAY_PID)?;

    // Write firmware in 128-byte sectors
    let mut addr: u32 = 0;
    while addr < ERGODOX_MEM_SIZE as u32 {
        // HID output report: [report_id=0, addr_lo, addr_hi, ...128 firmware bytes]
        let mut buf = vec![0u8; ERGODOX_SECTOR_SIZE + 3];
        buf[0] = 0; // report ID (unnumbered)
        buf[1] = (addr & 0xFF) as u8;
        buf[2] = ((addr >> 8) & 0xFF) as u8;

        // Fill sector data from firmware (pad with 0xFF if beyond firmware end)
        let start = addr as usize;
        for i in 0..ERGODOX_SECTOR_SIZE {
            buf[i + 3] = if start + i < firmware_data.len() {
                firmware_data[start + i]
            } else {
                0xFF
            };
        }

        log::debug!("HALFKAY: writing sector at {:#06x}", addr);

        send_with_retries(&hid, &buf, false)?;

        // First block needs extra time for erase
        if addr == 0 {
            std::thread::sleep(Duration::from_secs(2));
        } else {
            std::thread::sleep(Duration::from_millis(10));
        }

        on_progress(FlashProgress::Writing {
            bytes_written: addr as usize + ERGODOX_SECTOR_SIZE,
            total_bytes: ERGODOX_MEM_SIZE,
        });

        addr += ERGODOX_SECTOR_SIZE as u32;
    }

    std::thread::sleep(Duration::from_secs(1));

    // Send reboot packet (addr = 0xFFFF)
    on_progress(FlashProgress::Resetting);
    let mut reboot_buf = vec![0u8; ERGODOX_SECTOR_SIZE + 3];
    reboot_buf[0] = 0;
    reboot_buf[1] = 0xFF;
    reboot_buf[2] = 0xFF;
    // Reboot errors are non-fatal (device disconnects mid-transfer)
    let _ = send_with_retries(&hid, &reboot_buf, true);

    on_progress(FlashProgress::Complete);
    Ok(())
}

fn send_with_retries(hid: &HidDevice, buf: &[u8], silent: bool) -> Result<(), ZappError> {
    for attempt in 0..PACKET_RETRIES {
        match hid.write(buf) {
            Ok(_) => return Ok(()),
            Err(e) => {
                if !silent {
                    log::warn!(
                        "HALFKAY: send failed (attempt {}/{}): {}",
                        attempt + 1,
                        PACKET_RETRIES,
                        e
                    );
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    }

    if silent {
        Ok(())
    } else {
        Err(ZappError::Dfu(format!(
            "failed to send HID packet after {} retries",
            PACKET_RETRIES
        )))
    }
}
