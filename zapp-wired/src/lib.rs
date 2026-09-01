//! Updating the nRFs over the wire, through ignition.
//!
//! The keyboard's two nRF halves both run MCUboot, and MCUboot's serial
//! recovery speaks SMP over a line-based codec — base64 with a length and a
//! checksum, cut into lines short enough for its receive buffer. Getting those
//! lines to a half is ignition's job: it bridges the left half over UART and
//! the right half over the shared TRRS I2C bus, and the host reaches both
//! through one vendor interface on its USB device.
//!
//! So this crate is four layers, of which only the third touches hardware:
//!
//! * [`nlip`] — the line codec, pure functions.
//! * [`pipe`] — the framing over ignition's bulk endpoint, a state machine.
//! * [`ignition`] — the vendor requests, behind [`ignition::IgnitionLink`].
//! * [`transport`] — the two [`zapp_smp::SmpTransport`]s built out of those.
//!
//! With the link as a trait, the whole stack runs against an in-memory device:
//! see [`loopback`], which is how everything above it gets written without a
//! keyboard on the desk.

use std::time::Duration;

pub mod ignition;
#[cfg(any(test, feature = "loopback"))]
pub mod loopback;
pub mod nlip;
pub mod pipe;
pub mod transport;

pub use ignition::{Ignition, IgnitionLink, Retrying, Status};
pub use transport::{LeftTransport, RightTransport};

/// What can go wrong on the way to a half.
#[derive(Debug, thiserror::Error)]
pub enum WiredError {
    #[error(transparent)]
    Nlip(#[from] nlip::NlipError),

    #[error(transparent)]
    Pipe(#[from] pipe::PipeError),

    #[error("no ZSA bootloader is plugged in")]
    NoBootloader,

    #[error("{0}")]
    Usb(String),

    #[error("vendor request {request:#04x} failed: {reason}")]
    Request { request: u8, reason: String },

    /// The device answered a control request with a STALL.
    ///
    /// Its own variant rather than a [`Self::Request`] with a string in it,
    /// because it is the one control failure worth trying again: see
    /// [`Retrying`]. A request the bootloader genuinely refuses stalls every
    /// time and reaches a caller as this, once the retries are spent.
    #[error("the bootloader refused vendor request {request:#04x}: it stalled")]
    Stalled { request: u8 },

    /// A control transfer failed below the protocol — the host controller
    /// reported a hardware fault or protocol violation (EPROTO, EILSEQ and the
    /// rest), not an answer from the device.
    ///
    /// Its own variant rather than a [`Self::Request`], because on a read it is
    /// the second control failure worth trying again: see [`Retrying`]. Reads
    /// the same as a [`Self::Request`] to a caller, since a fault the retries
    /// do not clear is a failed request like any other.
    #[error("vendor request {request:#04x} failed: {reason}")]
    Faulted { request: u8, reason: String },

    #[error("a status response needs {expected} bytes, got {got}", expected = ignition::STATUS_LEN)]
    ShortStatus { got: usize },

    #[error(
        "this build reads version {expected} of ignition's status and the board sent \
         version {got}",
        expected = ignition::STATUS_VERSION
    )]
    StatusVersion { got: u8 },

    #[error(
        "a confirm result needs {expected} bytes, got {got}",
        expected = ignition::CONFIRM_RESULT_LEN
    )]
    ShortConfirm { got: usize },

    #[error(
        "a confirm names an image by {expected} bytes of hash and this one has {got}",
        expected = ignition::CONFIRM_HASH_LEN
    )]
    ShortHash { got: usize },

    #[error("an encoded frame of {len} bytes is past the board's {budget}-byte message limit")]
    FrameTooLong { len: usize, budget: usize },

    #[error("the bridge reported {0}")]
    Event(pipe::Event),

    /// A message the bridge would not take onto its bulk endpoint, for the
    /// whole time it was given.
    ///
    /// Its own variant rather than a [`Self::Usb`] string, because it is not a
    /// USB fault: the host cancels the transfer when the timeout runs out, and
    /// the cancellation is what surfaces. Nothing of the message reached the
    /// target and it cannot be sent again, so the link is finished — which is
    /// why it reads as [`zapp_smp::TransportError::Closed`].
    #[error("the bootloader stopped taking data for {} seconds", .after.as_secs())]
    WriteStalled { after: Duration },
}

/// A link that is gone says so as [`zapp_smp::TransportError::Closed`] — a
/// shape a caller can match on. Two things end a link: a fatal event from the
/// bridge, and a write the bridge never took, since a bootloader that has not
/// drained a message in half a minute is not coming back mid-stream.
/// Everything else flattens to its sentence, which is all a caller does with
/// it.
impl From<WiredError> for zapp_smp::TransportError {
    fn from(error: WiredError) -> Self {
        match &error {
            WiredError::Event(event) if event.code.is_fatal() => Self::Closed,
            WiredError::WriteStalled { .. } => Self::Closed,
            _ => Self::Io(error.to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_smp::TransportError;

    #[test]
    fn a_write_the_bridge_never_took_reads_as_a_closed_link() {
        // Same shape as a fatal event, so a caller telling "the half stopped
        // responding" from every other failure does not have to know which of
        // the two ways the cable went.
        let err = TransportError::from(WiredError::WriteStalled {
            after: Duration::from_secs(30),
        });
        assert!(matches!(err, TransportError::Closed), "{err}");
    }

    #[test]
    fn every_other_failure_keeps_its_sentence() {
        let err = TransportError::from(WiredError::NoBootloader);
        assert!(
            matches!(&err, TransportError::Io(text) if text.contains("no ZSA bootloader")),
            "{err}"
        );
    }
}
