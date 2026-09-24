use std::time::{Duration, Instant};

use futures_lite::StreamExt;
use nusb::hotplug::HotplugEvent;
use nusb::{Device, DeviceInfo, MaybeFuture};

use super::access::{ACCESS_RETRY, AccessGrace, permission_denied};
use super::ids::{
    BootloaderDevice, BootloaderKind, Keyboard, identify_bootloader, identify_keyboard,
    keyboard_for_bootloader,
};
use crate::ZappError;

/// A ZSA keyboard detected in normal (non-bootloader) mode.
pub struct ConnectedKeyboard {
    pub keyboard: Keyboard,
    pub serial: String,
    pub pid: u16,
}

/// A ZSA device attached right now, in either mode.
pub struct UsbDevice {
    /// `None` for a bootloader that names no keyboard, such as the generic
    /// STM32 DFU any STM32 board can present.
    pub keyboard: Option<Keyboard>,
    /// The mode it is in: `None` means it is running keyboard firmware.
    pub bootloader: Option<BootloaderKind>,
    /// `layoutId/revisionId` for a keyboard; bootloaders carry none.
    pub serial: Option<String>,
    pub vid: u16,
    pub pid: u16,
}

/// Every ZSA device attached right now, keyboards and bootloaders alike.
pub fn list_devices() -> Result<Vec<UsbDevice>, ZappError> {
    let mut devices = Vec::new();

    for dev_info in nusb::list_devices().wait()? {
        let vid = dev_info.vendor_id();
        let pid = dev_info.product_id();

        if let Some(keyboard) = identify_keyboard(vid, pid) {
            devices.push(UsbDevice {
                keyboard: Some(keyboard),
                bootloader: None,
                serial: dev_info.serial_number().map(str::to_string),
                vid,
                pid,
            });
        } else if let Some(kind) = identify_bootloader(vid, pid) {
            devices.push(UsbDevice {
                keyboard: keyboard_for_bootloader(vid, pid),
                bootloader: Some(kind),
                serial: None,
                vid,
                pid,
            });
        }
    }

    Ok(devices)
}

/// Scan USB for the first ZSA keyboard in normal mode.
pub fn find_keyboard() -> Result<ConnectedKeyboard, ZappError> {
    for dev_info in nusb::list_devices().wait()? {
        let vid = dev_info.vendor_id();
        let pid = dev_info.product_id();

        if let Some(keyboard) = identify_keyboard(vid, pid) {
            let serial = dev_info.serial_number().unwrap_or_default().to_string();
            return Ok(ConnectedKeyboard {
                keyboard,
                serial,
                pid,
            });
        }
    }
    Err(ZappError::NoKeyboardFound)
}

/// Status updates from the bootloader watcher.
#[derive(Debug, Clone)]
pub enum WatchStatus {
    /// Waiting for a bootloader device to appear.
    Waiting,
    /// A compatible bootloader was found.
    Found {
        keyboard: Option<Keyboard>,
        kind: BootloaderKind,
        name: &'static str,
    },
}

/// Block until a supported bootloader device appears on USB.
///
/// Checks already-connected devices first, then watches for hotplug events.
/// Returns as soon as a supported bootloader is detected.
pub fn wait_for_bootloader(
    timeout: Option<Duration>,
    on_status: impl Fn(WatchStatus),
) -> Result<BootloaderDevice, ZappError> {
    // Start watching before enumeration to avoid race conditions
    let watch = nusb::watch_devices()?;

    // Check already-connected devices first
    if let Some(dev) = try_find_bootloader_in_list()? {
        let name = super::ids::friendly_name(dev.vid, dev.pid);
        on_status(WatchStatus::Found {
            keyboard: dev.keyboard,
            kind: dev.kind,
            name,
        });
        return Ok(dev);
    }

    on_status(WatchStatus::Waiting);

    let deadline = timeout.map(|t| std::time::Instant::now() + t);

    futures_lite::future::block_on(async {
        let mut watch = std::pin::pin!(watch);

        loop {
            if let Some(deadline) = deadline {
                let remaining = deadline.saturating_duration_since(std::time::Instant::now());
                if remaining.is_zero() {
                    return Err(ZappError::Timeout);
                }
            }

            // Wait for the next hotplug event
            let event = match watch.next().await {
                Some(event) => event,
                None => return Err(ZappError::Timeout),
            };

            if let HotplugEvent::Connected(dev_info) = event {
                let vid = dev_info.vendor_id();
                let pid = dev_info.product_id();

                if let Some(kind) = identify_bootloader(vid, pid) {
                    let keyboard = keyboard_for_bootloader(vid, pid);
                    let name = super::ids::friendly_name(vid, pid);

                    // Small delay for device to be ready (especially on Windows)
                    std::thread::sleep(Duration::from_millis(500));

                    let device = open_bootloader(&dev_info)?;
                    on_status(WatchStatus::Found {
                        keyboard,
                        kind,
                        name,
                    });

                    return Ok(BootloaderDevice {
                        device,
                        vid,
                        pid,
                        kind,
                        keyboard,
                    });
                }
            }
        }
    })
}

fn try_find_bootloader_in_list() -> Result<Option<BootloaderDevice>, ZappError> {
    for dev_info in nusb::list_devices().wait()? {
        let vid = dev_info.vendor_id();
        let pid = dev_info.product_id();

        if let Some(kind) = identify_bootloader(vid, pid) {
            let keyboard = keyboard_for_bootloader(vid, pid);
            let device = open_bootloader(&dev_info)?;

            return Ok(Some(BootloaderDevice {
                device,
                vid,
                pid,
                kind,
                keyboard,
            }));
        }
    }
    Ok(None)
}

/// Opens a bootloader, giving a node that has just appeared a moment for the
/// udev rule to open it up.
fn open_bootloader(dev_info: &DeviceInfo) -> Result<Device, ZappError> {
    let mut grace = AccessGrace::default();
    loop {
        match dev_info.open().wait() {
            Err(err) if permission_denied(&err) => {
                if !grace.retry(Instant::now()) {
                    return Err(ZappError::AccessDenied(err));
                }
                log::debug!("the bootloader refused to open ({err}), trying again");
                std::thread::sleep(ACCESS_RETRY);
            }
            result => return Ok(result?),
        }
    }
}
