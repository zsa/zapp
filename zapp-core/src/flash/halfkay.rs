use std::time::Duration;

use nusb::transfer::ControlOut;
use nusb::transfer::ControlType;
use nusb::transfer::Recipient;
use nusb::MaybeFuture;

use crate::device::BootloaderDevice;
use crate::firmware::Firmware;
use crate::ZappError;

const USB_TIMEOUT: Duration = Duration::from_secs(5);

use super::FlashProgress;

const ERGODOX_MEM_SIZE: usize = 32256;
const ERGODOX_SECTOR_SIZE: usize = 128;
const PACKET_RETRIES: usize = 5;

/// Flash a device using the HALFKAY protocol (legacy Ergodox EZ).
pub fn flash_halfkay(
    device: &BootloaderDevice,
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

    let interface = device.device.detach_and_claim_interface(0).wait()?;

    // Write firmware in 128-byte sectors
    let mut addr: u32 = 0;
    while addr < ERGODOX_MEM_SIZE as u32 {
        // Build packet: [addr_lo, addr_hi, ...128 bytes of firmware...]
        // Report ID 0 is already encoded in wValue (0x0200) of the control transfer.
        let mut buf = vec![0u8; ERGODOX_SECTOR_SIZE + 2];
        buf[0] = (addr & 0xFF) as u8;
        buf[1] = ((addr >> 8) & 0xFF) as u8;

        // Fill sector data from firmware (pad with 0xFF if beyond firmware end)
        let start = addr as usize;
        for i in 0..ERGODOX_SECTOR_SIZE {
            buf[i + 2] = if start + i < firmware_data.len() {
                firmware_data[start + i]
            } else {
                0xFF
            };
        }

        log::debug!("HALFKAY: writing sector at {:#06x}", addr);

        // Send with retries via HID SET_REPORT control transfer
        send_with_retries(&interface, &buf, false)?;

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
    let mut reboot_buf = vec![0u8; ERGODOX_SECTOR_SIZE + 2];
    reboot_buf[0] = 0xFF;
    reboot_buf[1] = 0xFF;
    // Reboot errors are non-fatal (device disconnects)
    let _ = send_with_retries(&interface, &reboot_buf, true);

    on_progress(FlashProgress::Complete);
    Ok(())
}

fn send_with_retries(
    interface: &nusb::Interface,
    buf: &[u8],
    silent: bool,
) -> Result<(), ZappError> {
    for attempt in 0..PACKET_RETRIES {
        // HID SET_REPORT: bmRequestType=0x21 (class, host-to-device, interface)
        // bRequest=0x09 (SET_REPORT), wValue=0x0200 (output report, report ID 0)
        let result = interface
            .control_out(ControlOut {
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
                request: 0x09, // HID SET_REPORT
                value: 0x0200, // Output report, report ID 0
                index: 0,
                data: buf,
            }, USB_TIMEOUT)
            .wait();

        match result {
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
