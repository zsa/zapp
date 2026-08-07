//! BLE discovery for ZSA keyboards.
//!
//! A board shows up if it is connected to this machine, advertising in range,
//! or bonded to it — the same three states on every platform, assembled from
//! whatever each one is willing to answer. See `docs/ble-discovery.md` in
//! keymapp-tng for why each platform needs its own backend.

pub mod backend;
pub mod battery;
mod board;
pub mod identity;
mod monitor;
mod roster;

pub use battery::{Battery, Half};
pub use board::{BleBoard, BleEvent, BleSighting, Presence};
pub use monitor::BleMonitor;
pub use roster::Roster;

/// What can go wrong reaching the radio.
#[derive(Debug, thiserror::Error)]
pub enum BleError {
    #[error("no Bluetooth adapter")]
    NoAdapter,
    #[error("{0}")]
    Platform(String),
}
