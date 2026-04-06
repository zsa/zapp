pub mod dfu;
pub mod halfkay;

use crate::device::{BootloaderDevice, BootloaderKind};
use crate::firmware::Firmware;
use crate::ZappError;

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
    match device.kind {
        BootloaderKind::Stm32Dfu => dfu::flash_dfu(device, firmware, false, on_progress),
        BootloaderKind::IgnitionStm32 | BootloaderKind::IgnitionGd32 => {
            dfu::flash_dfu(device, firmware, true, on_progress)
        }
        BootloaderKind::Halfkay => halfkay::flash_halfkay(device, firmware, on_progress),
    }
}
