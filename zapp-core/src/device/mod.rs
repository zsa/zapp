pub mod access;
pub mod ids;
pub mod monitor;
pub mod watcher;

pub use access::AccessGrace;
pub use ids::{BootloaderDevice, BootloaderKind, Keyboard};
pub use monitor::{DeviceEvent, DeviceMonitor};
pub use watcher::{
    ConnectedKeyboard, UsbDevice, WatchStatus, find_keyboard, list_devices, wait_for_bootloader,
};

/// Opaque USB device identity, re-exported so consumers of `DeviceEvent` do not
/// need their own `nusb` dependency.
pub use nusb::DeviceId;
