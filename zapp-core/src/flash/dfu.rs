use std::time::Duration;

use nusb::transfer::ControlIn;
use nusb::transfer::ControlOut;
use nusb::transfer::ControlType;
use nusb::transfer::Recipient;
use nusb::MaybeFuture;

use crate::device::BootloaderDevice;
use crate::firmware::Firmware;
use crate::ZappError;

use super::FlashProgress;

const USB_TIMEOUT: Duration = Duration::from_secs(5);

// DFU constants
const BLOCK_SIZE: usize = 2048;

const STM32_START_ADDRESS: u32 = 0x0800_0000;
const STM32_END_ADDRESS: u32 = 0x0804_0000;
const IGNITION_START_ADDRESS: u32 = 0x0800_2000;
const IGNITION_END_ADDRESS: u32 = 0x0804_2000;

// DFU class requests
const DFU_DNLOAD: u8 = 0x01;
const DFU_GETSTATUS: u8 = 0x03;
const DFU_CLRSTATUS: u8 = 0x04;

// DFU states
const DFU_DNBUSY: u8 = 0x04;
const DFU_DNIDLE: u8 = 0x05;
const DFU_MANIFEST: u8 = 0x07;
const DFU_ERROR: u8 = 0x0A;

// DfuSe commands (sent as data in DFU_DNLOAD with wValue=0)
const DFU_SET_ADDRESS: u8 = 0x21;
const DFU_ERASE_SECTOR: u8 = 0x41;

#[derive(Debug, Default)]
struct DfuStatus {
    b_status: u8,
    bw_poll_timeout: u32,
    b_state: u8,
}

/// Perform DFU flashing on a device.
///
/// If `ignition` is true, uses Ignition address offsets (0x0800_2000).
pub fn flash_dfu(
    device: &BootloaderDevice,
    firmware: &Firmware,
    ignition: bool,
    on_progress: &dyn Fn(FlashProgress),
) -> Result<(), ZappError> {
    let firmware_data = match firmware {
        Firmware::DfuBinary { data, .. } => data.as_slice(),
        Firmware::IgnitionDual { primary, alternate } => {
            // Select the firmware matching this device's PID
            if device.pid == primary.pid {
                primary.data.as_slice()
            } else if device.pid == alternate.pid {
                alternate.data.as_slice()
            } else {
                // Fall back to primary
                primary.data.as_slice()
            }
        }
        Firmware::IntelHex { .. } => {
            return Err(ZappError::InvalidFirmware(
                "cannot flash Intel HEX via DFU; expected a .bin file".into(),
            ));
        }
    };

    let start_address = if ignition {
        IGNITION_START_ADDRESS
    } else {
        STM32_START_ADDRESS
    };
    let max_address = if ignition {
        IGNITION_END_ADDRESS
    } else {
        STM32_END_ADDRESS
    };

    let end_address = start_address + firmware_data.len() as u32 + BLOCK_SIZE as u32;
    if end_address > max_address {
        return Err(ZappError::InvalidFirmware(format!(
            "firmware too large: {} bytes exceeds flash capacity",
            firmware_data.len()
        )));
    }

    let interface = device.device.detach_and_claim_interface(0).wait()?;

    let mut status = DfuStatus::default();

    // Get initial status
    dfu_get_status(&interface, &mut status)?;

    // Clear error state if needed
    if status.b_status == DFU_ERROR {
        log::debug!("DFU: clearing error status");
        dfu_clear_status(&interface)?;
        dfu_get_status(&interface, &mut status)?;

        if status.b_status == DFU_ERROR {
            return Err(ZappError::Dfu(
                "failed to clear error status — try unplugging and replugging the device".into(),
            ));
        }
    }

    let erase_size = (end_address - start_address) as usize;
    let total_progress = erase_size + firmware_data.len();

    // Erase flash
    log::info!("DFU: erasing flash");
    let mut progress = 0usize;
    let mut addr = start_address;
    while addr < end_address {
        log::debug!("DFU: erasing sector at {:#010x}", addr);
        dfu_command(&interface, &mut status, DFU_ERASE_SECTOR, addr)?;
        progress += BLOCK_SIZE;
        on_progress(FlashProgress::Erasing {
            bytes_erased: progress.min(erase_size),
            total_bytes: total_progress,
        });
        addr += BLOCK_SIZE as u32;
    }

    // Write firmware
    log::info!(
        "DFU: writing {} bytes of firmware",
        firmware_data.len()
    );
    let mut written = 0usize;
    while written < firmware_data.len() {
        let page_addr = start_address + written as u32;
        let chunk_size = BLOCK_SIZE.min(firmware_data.len() - written);

        log::debug!("DFU: writing block at {:#010x}", page_addr);

        // Set address pointer
        dfu_command(&interface, &mut status, DFU_SET_ADDRESS, page_addr)?;

        // Download data block (wValue=2)
        dfu_download(&interface, &mut status, &firmware_data[written..written + chunk_size])?;

        written += chunk_size;
        on_progress(FlashProgress::Writing {
            bytes_written: erase_size + written,
            total_bytes: total_progress,
        });
    }

    // Reboot
    on_progress(FlashProgress::Resetting);
    std::thread::sleep(Duration::from_secs(1));

    // Set address back to start
    dfu_command(&interface, &mut status, DFU_SET_ADDRESS, start_address)?;

    // Send empty download to trigger reboot
    log::info!("DFU: rebooting device");
    dfu_reboot(&interface, &mut status)?;

    on_progress(FlashProgress::Complete);
    Ok(())
}

fn dfu_get_status(
    interface: &nusb::Interface,
    status: &mut DfuStatus,
) -> Result<(), ZappError> {
    let buf = interface
        .control_in(ControlIn {
            control_type: ControlType::Class,
            recipient: Recipient::Interface,
            request: DFU_GETSTATUS,
            value: 0,
            index: 0,
            length: 6,
        }, USB_TIMEOUT)
        .wait()?;

    if buf.len() < 6 {
        return Err(ZappError::Dfu("GETSTATUS returned fewer than 6 bytes".into()));
    }

    status.b_status = buf[0];
    status.bw_poll_timeout =
        buf[1] as u32 | (buf[2] as u32) << 8 | (buf[3] as u32) << 16;
    status.b_state = buf[4];

    Ok(())
}

fn dfu_clear_status(interface: &nusb::Interface) -> Result<(), ZappError> {
    interface
        .control_out(ControlOut {
            control_type: ControlType::Class,
            recipient: Recipient::Interface,
            request: DFU_CLRSTATUS,
            value: 0,
            index: 0,
            data: &[],
        }, USB_TIMEOUT)
        .wait()?;
    Ok(())
}

fn dfu_poll_timeout(
    interface: &nusb::Interface,
    status: &mut DfuStatus,
    predicate: impl Fn(&DfuStatus) -> bool,
) -> Result<(), ZappError> {
    while predicate(status) {
        let timeout = status.bw_poll_timeout.max(100);
        log::debug!(
            "DFU: polling — state={:#04x}, status={:#04x}, timeout={}ms",
            status.b_state,
            status.b_status,
            timeout
        );
        std::thread::sleep(Duration::from_millis(timeout as u64));
        dfu_get_status(interface, status)?;
    }
    Ok(())
}

fn dfu_command(
    interface: &nusb::Interface,
    status: &mut DfuStatus,
    cmd: u8,
    addr: u32,
) -> Result<(), ZappError> {
    let mut buf = Vec::with_capacity(5);
    buf.push(cmd);
    if cmd == DFU_ERASE_SECTOR && addr == 0 {
        // Mass erase — single byte command
    } else {
        buf.extend_from_slice(&addr.to_le_bytes());
    }

    interface
        .control_out(ControlOut {
            control_type: ControlType::Class,
            recipient: Recipient::Interface,
            request: DFU_DNLOAD,
            value: 0,
            index: 0,
            data: &buf,
        }, USB_TIMEOUT)
        .wait()?;

    dfu_get_status(interface, status)?;
    dfu_poll_timeout(interface, status, |s| s.b_state == DFU_DNBUSY)?;

    Ok(())
}

fn dfu_download(
    interface: &nusb::Interface,
    status: &mut DfuStatus,
    data: &[u8],
) -> Result<(), ZappError> {
    interface
        .control_out(ControlOut {
            control_type: ControlType::Class,
            recipient: Recipient::Interface,
            request: DFU_DNLOAD,
            value: 2,
            index: 0,
            data,
        }, USB_TIMEOUT)
        .wait()?;

    dfu_get_status(interface, status)?;
    dfu_poll_timeout(interface, status, |s| s.b_state != DFU_DNIDLE)?;

    Ok(())
}

fn dfu_reboot(
    interface: &nusb::Interface,
    status: &mut DfuStatus,
) -> Result<(), ZappError> {
    // Send empty download to trigger manifest/reboot
    interface
        .control_out(ControlOut {
            control_type: ControlType::Class,
            recipient: Recipient::Interface,
            request: DFU_DNLOAD,
            value: 2,
            index: 0,
            data: &[],
        }, USB_TIMEOUT)
        .wait()?;

    dfu_get_status(interface, status)?;
    // Some devices disconnect immediately during manifest, which is fine
    let _ = dfu_poll_timeout(interface, status, |s| s.b_state != DFU_MANIFEST);

    Ok(())
}
