pub mod dfu;
pub mod halfkay;

use crate::ZappError;
use crate::device::ids::{self, Keyboard};
use crate::device::{BootloaderDevice, BootloaderKind};
use crate::firmware::Firmware;

/// Progress updates emitted during flashing.
#[derive(Debug, Clone)]
pub enum FlashProgress {
    Erasing {
        bytes_erased: usize,
        total_bytes: usize,
    },
    Writing {
        bytes_written: usize,
        total_bytes: usize,
    },
    Resetting,
    Complete,
}

/// Flash a device with the given firmware, reporting progress via callback.
pub fn flash_device(
    device: &BootloaderDevice,
    firmware: &Firmware,
    on_progress: &dyn Fn(FlashProgress),
) -> Result<(), ZappError> {
    validate_firmware_compatibility(device, firmware)?;

    match device.kind {
        BootloaderKind::Stm32Dfu => dfu::flash_dfu(device, firmware, false, on_progress),
        BootloaderKind::IgnitionStm32 | BootloaderKind::IgnitionGd32 => {
            dfu::flash_dfu(device, firmware, true, on_progress)
        }
        BootloaderKind::Halfkay => halfkay::flash_halfkay(device, firmware, on_progress),
    }
}

/// Validate that the firmware is compatible with the target bootloader device.
///
/// Prevents flashing firmware built for one bootloader protocol onto a device
/// using a different protocol (e.g. STM32 DFU firmware onto an Ignition device),
/// which would brick the keyboard due to mismatched base addresses.
fn validate_firmware_compatibility(
    device: &BootloaderDevice,
    firmware: &Firmware,
) -> Result<(), ZappError> {
    let is_ignition = matches!(
        device.kind,
        BootloaderKind::IgnitionStm32 | BootloaderKind::IgnitionGd32
    );

    match firmware {
        Firmware::IgnitionDual { primary, alternate } => {
            // A dual-image firmware contains two images (e.g. Moonlander revA + revB).
            // The device's bootloader PID must match one of them so the correct image
            // is selected in flash_dfu. Reject if neither image matches.
            if device.pid != primary.pid && device.pid != alternate.pid {
                return Err(ZappError::IncompatibleFirmware {
                    firmware_desc: format!(
                        "dual-image firmware ({} + {})",
                        ids::target_name_for_pid(primary.pid),
                        ids::target_name_for_pid(alternate.pid),
                    ),
                    device_desc: ids::friendly_name(device.vid, device.pid).into(),
                });
            }
        }
        Firmware::DfuBinary { vid, pid, .. } => {
            // Determine which bootloader protocol the firmware expects based on
            // the VID/PID embedded in its DFU suffix.
            let fw_bootloader = ids::identify_bootloader(*vid, *pid);
            let fw_is_ignition = matches!(
                fw_bootloader,
                Some(BootloaderKind::IgnitionStm32 | BootloaderKind::IgnitionGd32)
            );
            let fw_is_stm32_dfu = matches!(fw_bootloader, Some(BootloaderKind::Stm32Dfu));

            if fw_is_stm32_dfu && is_ignition {
                // STM32 DFU firmware (linked at 0x0800_0000) on Ignition device (0x0800_2000).
                return Err(ZappError::IncompatibleFirmware {
                    firmware_desc: format!(
                        "{} firmware (STM32 DFU)",
                        ids::target_name_for_pid(*pid)
                    ),
                    device_desc: ids::friendly_name(device.vid, device.pid).into(),
                });
            }

            if fw_is_ignition && !is_ignition {
                // Ignition firmware (linked at 0x0800_2000) on STM32 DFU device (0x0800_0000).
                return Err(ZappError::IncompatibleFirmware {
                    firmware_desc: format!(
                        "{} firmware (Ignition)",
                        ids::target_name_for_pid(*pid)
                    ),
                    device_desc: ids::friendly_name(device.vid, device.pid).into(),
                });
            }

            // Also check normal-mode PIDs for Moonlander cross-revision mismatch,
            // in case the firmware suffix uses the keyboard PID rather than bootloader PID.
            if *vid == ids::ZSA_VID {
                let fw_keyboard = ids::identify_keyboard(*vid, *pid);
                if fw_keyboard == Some(Keyboard::Moonlander) {
                    let fw_is_revb = ids::is_moonlander_revb(*pid);
                    if fw_is_revb && !is_ignition {
                        return Err(ZappError::IncompatibleFirmware {
                            firmware_desc: "Moonlander rev B (Ignition) firmware".into(),
                            device_desc: ids::friendly_name(device.vid, device.pid).into(),
                        });
                    }
                    if !fw_is_revb && is_ignition {
                        return Err(ZappError::IncompatibleFirmware {
                            firmware_desc: "Moonlander rev A (STM32 DFU) firmware".into(),
                            device_desc: ids::friendly_name(device.vid, device.pid).into(),
                        });
                    }
                }
            }
        }
        Firmware::IntelHex { .. } => {
            // Intel HEX is only valid for HALFKAY devices.
            if device.kind != BootloaderKind::Halfkay {
                return Err(ZappError::IncompatibleFirmware {
                    firmware_desc: "Intel HEX firmware (HALFKAY)".into(),
                    device_desc: ids::friendly_name(device.vid, device.pid).into(),
                });
            }
        }
    }

    Ok(())
}
