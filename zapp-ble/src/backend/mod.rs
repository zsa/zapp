//! Platform BLE backends.
//!
//! Presence today. The GATT half — connect, discover, read, write, subscribe —
//! joins this trait when flashing lands, on the same handle a sighting carries,
//! so the backends do not have to be reshaped to reach it.

use std::time::Duration;

use crate::{BleError, BleSighting};

#[cfg(target_os = "linux")]
mod bluez;

pub trait Backend: Send {
    /// Every device the platform can see right now, ZSA or not.
    fn sightings(&mut self) -> Result<Vec<BleSighting>, BleError>;

    /// Blocks until something may have changed, or `timeout` elapses.
    ///
    /// Sleeping is the honest default, and makes the monitor a poller. A
    /// backend with a change signal of its own — BlueZ has D-Bus ones —
    /// overrides this so snapshots follow the radio instead of the clock.
    fn wait_for_change(&mut self, timeout: Duration) {
        std::thread::sleep(timeout);
    }
}

/// The backend for this platform, or `None` where there is not one yet.
///
/// `known` seeds the platforms that cannot enumerate their own bonds; the rest
/// ignore it.
pub fn for_platform(known: &[String]) -> Option<Box<dyn Backend>> {
    let _ = known;

    #[cfg(target_os = "linux")]
    match bluez::Bluez::connect() {
        Ok(backend) => return Some(Box::new(backend)),
        // No system bus, or no BlueZ on it. Nothing else to fall back to.
        Err(err) => log::warn!("no BlueZ: {err}"),
    }

    None
}
