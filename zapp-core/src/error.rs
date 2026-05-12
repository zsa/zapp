use std::path::PathBuf;

#[derive(Debug, thiserror::Error)]
pub enum ZappError {
    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),

    #[error("USB transfer error: {0}")]
    Transfer(#[from] nusb::transfer::TransferError),

    #[error("HID error: {0}")]
    Hid(#[from] hidapi::HidError),

    #[error("DFU error: {0}")]
    Dfu(String),

    #[error("Invalid firmware: {0}")]
    InvalidFirmware(String),

    #[error("Incompatible firmware: {firmware_desc} cannot be flashed on {device_desc}")]
    IncompatibleFirmware {
        firmware_desc: String,
        device_desc: String,
    },

    #[error("Unsupported device: VID={vid:#06x} PID={pid:#06x}")]
    UnsupportedDevice { vid: u16, pid: u16 },

    #[error("Timeout waiting for bootloader")]
    Timeout,

    #[error("No ZSA keyboard found")]
    NoKeyboardFound,

    #[error("Network error: {0}")]
    Network(String),

    #[error("IO error reading {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
}
