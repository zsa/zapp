pub mod ids;
pub mod monitor;
pub mod watcher;

pub use ids::{BootloaderDevice, BootloaderKind, Keyboard};
pub use monitor::{DeviceEvent, DeviceMonitor};
pub use watcher::{ConnectedKeyboard, WatchStatus, find_keyboard, wait_for_bootloader};

/// Opaque USB device identity, re-exported so consumers of `DeviceEvent` do not
/// need their own `nusb` dependency.
pub use nusb::DeviceId;
