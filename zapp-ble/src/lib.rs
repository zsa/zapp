//! BLE discovery for ZSA keyboards.
//!
//! A board shows up if it is connected to this machine, advertising in range,
//! or bonded to it — the same three states on every platform, assembled from
//! whatever each one is willing to answer. See `docs/ble-discovery.md` in
//! keymapp-tng for why each platform needs its own backend.

pub mod backend;
pub mod battery;
mod board;
pub mod dfu;
pub mod identity;
mod monitor;
pub mod pick;
mod roster;
pub mod smp;

pub use battery::{Battery, Half, Level};
pub use board::{BleBoard, BleEvent, BleSighting, Presence};
pub use monitor::BleMonitor;
pub use roster::Roster;

/// What can go wrong reaching the radio.
#[derive(Debug, thiserror::Error)]
pub enum BleError {
    #[error("no Bluetooth adapter")]
    NoAdapter,
    /// A platform that answers about presence but cannot open a GATT link.
    #[error("{0} is not supported on this platform")]
    Unsupported(&'static str),
    /// A handle the platform no longer knows, or a service the board does not
    /// carry.
    #[error("{0}")]
    NotFound(String),
    /// Reachable, but not bonded — and an SMP link needs an encrypted one.
    #[error("{0} is not bonded to this machine; pair it first")]
    NotBonded(String),
    #[error("{0}")]
    Platform(String),
}
