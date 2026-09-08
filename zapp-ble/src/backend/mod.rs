//! Platform BLE backends.
//!
//! Presence, and — since flashing has landed — the GATT half: connect,
//! discover, read, write, subscribe, reached on the same handle a sighting
//! carries, so the backends did not have to be reshaped to get at it.

use std::time::Duration;

use crate::battery::ChargeDetail;
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

/// How a characteristic is going to be driven.
///
/// GATT characteristics are not interchangeable and the platform has to be told
/// which kind it is opening: subscribing to one that does not notify fails, and
/// a control opcode written as a command is a write nothing ever answers. One
/// service can need all three shapes at once — ZSA's update service has a
/// control characteristic that wants the acknowledgement, a data characteristic
/// that must not pay for one, and a status characteristic that only notifies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Usage {
    /// Subscribe, so the board can speak first.
    pub notify: bool,
    /// Anything is ever written to it.
    pub written: bool,
    /// Write with a response, rather than fire-and-forget.
    pub acknowledged: bool,
}

impl Usage {
    /// Notifies, and takes command writes: the SMP characteristic.
    pub const DUPLEX: Usage = Usage {
        notify: true,
        written: true,
        acknowledged: false,
    };

    /// Write without response, nothing coming back. What bulk data wants: an
    /// acknowledgement per packet would cost a connection interval each.
    pub const STREAM: Usage = Usage {
        notify: false,
        written: true,
        acknowledged: false,
    };

    /// Write with a response, nothing coming back. What a control opcode wants,
    /// since the response is the only sign the board took it.
    pub const COMMAND: Usage = Usage {
        notify: false,
        written: true,
        acknowledged: true,
    };

    /// Read and subscribed to, never written.
    pub const LISTEN: Usage = Usage {
        notify: true,
        written: false,
        acknowledged: false,
    };
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
        self.open_as(key, service, characteristic, Usage::DUPLEX)
    }

    /// The same, for a characteristic that is not the duplex kind.
    fn open_as(
        &mut self,
        key: &str,
        service: u128,
        characteristic: u128,
        usage: Usage,
    ) -> Result<Box<dyn GattChannel>, BleError> {
        let _ = (key, service, characteristic, usage);
        Err(BleError::Unsupported("GATT"))
    }
}

/// What a connected board's cells and chargers are doing, read once.
///
/// `key` is the handle a sighting carries. Outside a board that is connected
/// and carries the characteristic there is nothing to read, and on a platform
/// without a GATT backend there is nowhere to read it from: all of those are
/// `None`, which a caller shows by saying nothing.
pub fn read_charge_detail(key: &str) -> Option<ChargeDetail> {
    #[cfg(target_os = "linux")]
    {
        bluez::read_charge_detail(key)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = key;
        None
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
