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
        BootloaderKind::Stm32Dfu
        | BootloaderKind::IgnitionStm32
        | BootloaderKind::IgnitionGd32
        | BootloaderKind::IgnitionV2 => dfu::flash_dfu(device, firmware, on_progress),
        BootloaderKind::Halfkay => halfkay::flash_halfkay(device, firmware, on_progress),
    }
}

/// Validate that the firmware is compatible with the target bootloader device.
fn validate_firmware_compatibility(
    device: &BootloaderDevice,
    firmware: &Firmware,
) -> Result<(), ZappError> {
    check_compatibility(device.kind, device.vid, device.pid, firmware)
}

/// Prevents flashing firmware built for one bootloader onto a device using
/// another, which would brick the keyboard: each bootloader writes at its own
/// flash base (STM32 DFU 0x0800_0000, Ignition 0x0800_2000, Ignition v2
/// 0x0800_4000) and the image is linked for exactly one of them.
fn check_compatibility(
    kind: BootloaderKind,
    vid: u16,
    pid: u16,
    firmware: &Firmware,
) -> Result<(), ZappError> {
    let is_ignition = matches!(
        kind,
        BootloaderKind::IgnitionStm32 | BootloaderKind::IgnitionGd32 | BootloaderKind::IgnitionV2
    );
    let (device_base, _) = dfu::flash_region(kind);

    match firmware {
        Firmware::IgnitionDual { primary, alternate } => {
            // A dual-image firmware contains two images (e.g. Moonlander revA + revB).
            // The device's bootloader PID must match one of them so the correct image
            // is selected in flash_dfu. Reject if neither image matches.
            if pid != primary.pid && pid != alternate.pid {
                return Err(ZappError::IncompatibleFirmware {
                    firmware_desc: format!(
                        "dual-image firmware ({} + {})",
                        ids::target_name_for_pid(primary.pid),
                        ids::target_name_for_pid(alternate.pid),
                    ),
                    device_desc: ids::friendly_name(vid, pid).into(),
                });
            }
        }
        Firmware::DfuBinary {
            vid: fw_vid,
            pid: fw_pid,
            ..
        } => {
            // The VID/PID in the DFU suffix says which bootloader the image was
            // linked for; its base must match the device's.
            if let Some(fw_kind) = ids::identify_bootloader(*fw_vid, *fw_pid) {
                let (fw_base, _) = dfu::flash_region(fw_kind);
                if fw_base != device_base {
                    return Err(ZappError::IncompatibleFirmware {
                        firmware_desc: format!(
                            "{} firmware (flash base {:#010x})",
                            ids::target_name_for_pid(*fw_pid),
                            fw_base
                        ),
                        device_desc: ids::friendly_name(vid, pid).into(),
                    });
                }
            }

            // Also check normal-mode PIDs for Moonlander cross-revision mismatch,
            // in case the firmware suffix uses the keyboard PID rather than bootloader PID.
            if *fw_vid == ids::ZSA_VID
                && ids::identify_keyboard(*fw_vid, *fw_pid) == Some(Keyboard::Moonlander)
            {
                let fw_is_revb = ids::is_moonlander_revb(*fw_pid);
                if fw_is_revb && !is_ignition {
                    return Err(ZappError::IncompatibleFirmware {
                        firmware_desc: "Moonlander rev B (Ignition) firmware".into(),
                        device_desc: ids::friendly_name(vid, pid).into(),
                    });
                }
                if !fw_is_revb && is_ignition {
                    return Err(ZappError::IncompatibleFirmware {
                        firmware_desc: "Moonlander rev A (STM32 DFU) firmware".into(),
                        device_desc: ids::friendly_name(vid, pid).into(),
                    });
                }
            }
        }
        Firmware::IntelHex { .. } => {
            // Intel HEX is only valid for HALFKAY devices.
            if kind != BootloaderKind::Halfkay {
                return Err(ZappError::IncompatibleFirmware {
                    firmware_desc: "Intel HEX firmware (HALFKAY)".into(),
                    device_desc: ids::friendly_name(vid, pid).into(),
                });
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::firmware::DfuImage;

    fn dfu_binary(vid: u16, pid: u16) -> Firmware {
        Firmware::DfuBinary {
            data: vec![0; 64],
            vid,
            pid,
        }
    }

    fn check(kind: BootloaderKind, pid: u16, firmware: &Firmware) -> Result<(), ZappError> {
        check_compatibility(kind, ids::ZSA_VID, pid, firmware)
    }

    #[test]
    fn matching_base_is_accepted() {
        // Wireless Voyager image on a Wireless Voyager bootloader
        assert!(
            check(
                BootloaderKind::IgnitionV2,
                0x0792,
                &dfu_binary(ids::ZSA_VID, 0x0792)
            )
            .is_ok()
        );
        // Voyager image on a Voyager Ignition bootloader
        assert!(
            check(
                BootloaderKind::IgnitionStm32,
                0x0791,
                &dfu_binary(ids::ZSA_VID, 0x0791)
            )
            .is_ok()
        );
    }

    #[test]
    fn voyager2_image_is_rejected_on_other_bases() {
        let fw = dfu_binary(ids::ZSA_VID, 0x0792);
        assert!(check(BootloaderKind::IgnitionStm32, 0x0791, &fw).is_err());
        assert!(check(BootloaderKind::IgnitionGd32, 0x1791, &fw).is_err());
        assert!(
            check_compatibility(
                BootloaderKind::Stm32Dfu,
                ids::STM32_VID,
                ids::STM32_DFU_PID,
                &fw
            )
            .is_err()
        );
    }

    #[test]
    fn other_images_are_rejected_on_voyager2() {
        for (vid, pid) in [
            (ids::ZSA_VID, 0x0791),
            (ids::ZSA_VID, 0x1791),
            (ids::STM32_VID, ids::STM32_DFU_PID),
        ] {
            assert!(
                check(BootloaderKind::IgnitionV2, 0x0792, &dfu_binary(vid, pid)).is_err(),
                "{vid:#06x}:{pid:#06x} should not flash onto Ignition v2"
            );
        }
    }

    #[test]
    fn intel_hex_only_flashes_halfkay() {
        let hex = Firmware::IntelHex { data: vec![0; 64] };
        assert!(
            check_compatibility(
                BootloaderKind::Halfkay,
                ids::HALFKAY_VID,
                ids::HALFKAY_PID,
                &hex
            )
            .is_ok()
        );
        assert!(check(BootloaderKind::IgnitionV2, 0x0792, &hex).is_err());
    }

    #[test]
    fn dual_image_requires_a_matching_pid() {
        let dual = Firmware::IgnitionDual {
            primary: DfuImage {
                data: vec![0; 64],
                vid: ids::ZSA_VID,
                pid: 0x0791,
            },
            alternate: DfuImage {
                data: vec![0; 64],
                vid: ids::ZSA_VID,
                pid: 0x1791,
            },
        };
        assert!(check(BootloaderKind::IgnitionStm32, 0x0791, &dual).is_ok());
        assert!(check(BootloaderKind::IgnitionV2, 0x0792, &dual).is_err());
    }
}
