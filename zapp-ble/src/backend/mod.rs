//! Platform BLE backends.
//!
//! Presence, and — since flashing has landed — the GATT half: connect,
//! discover, read, write, subscribe, reached on the same handle a sighting
//! carries, so the backends did not have to be reshaped to get at it.

use std::time::Duration;

use crate::{BleError, BleSighting};

#[cfg(target_os = "linux")]
mod bluez;
#[cfg(target_os = "linux")]
mod bluez_gatt;

/// An open link to one characteristic on one board.
///
/// Deliberately packet-shaped rather than stream-shaped: `write` puts exactly
/// one ATT packet on the wire and `notification` returns exactly one, because a
/// protocol carried over GATT has to do its own fragmenting against the MTU and
/// cannot do that through an interface that hides the boundaries.
pub trait GattChannel: Send {
    /// The most bytes one write or one notification can carry.
    fn mtu(&self) -> usize;

    /// Reads the characteristic's current value.
    fn read(&mut self) -> Result<Vec<u8>, BleError>;

    /// Sends one packet. Must be no longer than `mtu`.
    fn write(&mut self, value: &[u8]) -> Result<(), BleError>;

    /// The next notification, or `None` once `timeout` elapses.
    fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError>;
}

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

    /// Closes any discovery window this backend opened.
    ///
    /// A scanning radio is a radio not spending its airtime on the link, which
    /// a caller about to push a firmware image over that link wants back.
    fn stop_discovery(&mut self) {}

    /// Connects to a board and subscribes to one of its characteristics.
    ///
    /// `key` is the handle a sighting carries. The board must already be
    /// bonded: the services worth writing to are behind an encrypted link.
    fn open(
        &mut self,
        key: &str,
        service: u128,
        characteristic: u128,
    ) -> Result<Box<dyn GattChannel>, BleError> {
        let _ = (key, service, characteristic);
        Err(BleError::Unsupported("GATT"))
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
