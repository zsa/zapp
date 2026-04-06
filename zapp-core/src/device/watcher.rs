use std::time::Duration;

use futures_lite::StreamExt;
use nusb::hotplug::HotplugEvent;
use nusb::MaybeFuture;

use super::ids::{identify_bootloader, keyboard_for_bootloader, BootloaderDevice, BootloaderKind, Keyboard};
use crate::ZappError;

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

                    let device = dev_info.open().wait()?;
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
            let device = dev_info.open().wait()?;

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
