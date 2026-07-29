//! Continuous device monitoring via USB hotplug events.
//!
//! Unlike `watcher` which blocks until a single bootloader appears,
//! `DeviceMonitor` provides a stream of connect/disconnect events
//! for both normal keyboards and bootloader devices.

use std::sync::mpsc;
use std::thread;

use futures_lite::StreamExt;
use nusb::hotplug::HotplugEvent;
use nusb::{DeviceId, MaybeFuture};

use super::ids::{
    BootloaderKind, Keyboard, identify_bootloader, identify_keyboard, keyboard_for_bootloader,
};
use crate::ZappError;

/// Events emitted by the device monitor.
///
/// Every variant carries the `nusb::DeviceId` of the device it refers to, so a
/// consumer tracking several devices can match a disconnect to a connect.
/// Connect variants also carry the physical port (`bus_id` + `port_chain`),
/// which stays stable across a reset into the bootloader.
#[derive(Debug, Clone)]
pub enum DeviceEvent {
    /// A ZSA keyboard was connected in normal mode.
    KeyboardConnected {
        id: DeviceId,
        keyboard: Keyboard,
        serial: String,
        pid: u16,
        bus_id: String,
        port_chain: Vec<u8>,
    },
    /// A ZSA keyboard was disconnected.
    KeyboardDisconnected { id: DeviceId },
    /// A bootloader device appeared.
    BootloaderConnected {
        id: DeviceId,
        keyboard: Option<Keyboard>,
        kind: BootloaderKind,
        vid: u16,
        pid: u16,
        bus_id: String,
        port_chain: Vec<u8>,
    },
    /// A bootloader device was disconnected.
    BootloaderDisconnected { id: DeviceId },
}

enum TrackedDevice {
    Keyboard,
    Bootloader,
}

/// Monitors USB for ZSA device connect/disconnect events using hotplug.
pub struct DeviceMonitor {
    rx: mpsc::Receiver<DeviceEvent>,
}

impl DeviceMonitor {
    /// Start monitoring. Performs an initial scan, then watches for hotplug events.
    pub fn start() -> Result<Self, ZappError> {
        let (tx, rx) = mpsc::channel();

        // Start hotplug watcher before initial scan to avoid missing events
        let watch = nusb::watch_devices()?;

        thread::spawn(move || {
            // Track connected ZSA devices by their DeviceId so we can identify disconnects
            let mut tracked: Vec<(DeviceId, TrackedDevice)> = Vec::new();

            // Single initial scan: seeds `tracked` and emits the events for
            // devices that were already attached
            if let Ok(devices) = nusb::list_devices().wait() {
                for dev_info in devices {
                    if let Some((event, kind)) = classify(&dev_info) {
                        tracked.push((dev_info.id(), kind));
                        if tx.send(event).is_err() {
                            return;
                        }
                    }
                }
            }

            futures_lite::future::block_on(async {
                let mut watch = std::pin::pin!(watch);

                while let Some(event) = watch.next().await {
                    let event = match event {
                        HotplugEvent::Connected(dev_info) => {
                            let Some((event, kind)) = classify(&dev_info) else {
                                continue;
                            };
                            tracked.push((dev_info.id(), kind));
                            event
                        }
                        HotplugEvent::Disconnected(id) => {
                            let Some(pos) = tracked.iter().position(|(dev, _)| *dev == id) else {
                                continue;
                            };
                            match tracked.remove(pos).1 {
                                TrackedDevice::Keyboard => DeviceEvent::KeyboardDisconnected { id },
                                TrackedDevice::Bootloader => {
                                    DeviceEvent::BootloaderDisconnected { id }
                                }
                            }
                        }
                    };

                    if tx.send(event).is_err() {
                        break;
                    }
                }
            });
        });

        Ok(Self { rx })
    }

    /// Try to receive the next event without blocking.
    pub fn try_recv(&self) -> Option<DeviceEvent> {
        self.rx.try_recv().ok()
    }

    /// Block until the next event is available.
    pub fn recv(&self) -> Option<DeviceEvent> {
        self.rx.recv().ok()
    }
}

/// Build the connect event for a device, or `None` if it is not a ZSA device.
fn classify(dev_info: &nusb::DeviceInfo) -> Option<(DeviceEvent, TrackedDevice)> {
    let vid = dev_info.vendor_id();
    let pid = dev_info.product_id();
    let id = dev_info.id();
    let bus_id = dev_info.bus_id().to_string();
    let port_chain = dev_info.port_chain().to_vec();

    if let Some(keyboard) = identify_keyboard(vid, pid) {
        let serial = dev_info.serial_number().unwrap_or_default().to_string();
        return Some((
            DeviceEvent::KeyboardConnected {
                id,
                keyboard,
                serial,
                pid,
                bus_id,
                port_chain,
            },
            TrackedDevice::Keyboard,
        ));
    }

    let kind = identify_bootloader(vid, pid)?;
    Some((
        DeviceEvent::BootloaderConnected {
            id,
            keyboard: keyboard_for_bootloader(vid, pid),
            kind,
            vid,
            pid,
            bus_id,
            port_chain,
        },
        TrackedDevice::Bootloader,
    ))
}
