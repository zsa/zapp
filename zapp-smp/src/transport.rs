//! The one thing SMP needs from whatever carries it.
//!
//! Deliberately packet-shaped rather than stream-shaped: `write` puts exactly
//! one packet on the wire and `read` returns exactly one, because SMP has to
//! do its own fragmenting against the MTU and cannot do that through an
//! interface that hides the boundaries.

use std::time::Duration;

pub trait SmpTransport: Send {
    /// Bytes one write puts on the wire.
    fn mtu(&self) -> usize;

    /// Sends one packet. Must be no longer than `mtu`.
    fn write(&mut self, packet: &[u8]) -> Result<(), TransportError>;

    /// The next packet, or `None` once `timeout` elapses.
    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, TransportError>;
}

/// What can go wrong on the wire, whichever wire it is.
#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error("{0}")]
    Io(String),
    #[error("the link is closed")]
    Closed,
    #[error("the link timed out")]
    Timeout,
}
