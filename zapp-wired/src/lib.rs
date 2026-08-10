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

    #[error("a status response needs {expected} bytes, got {got}", expected = ignition::STATUS_LEN)]
    ShortStatus { got: usize },

    #[error(
        "this build reads version {expected} of ignition's status and the board sent \
         version {got}; the bootloader and zapp are not from the same release",
        expected = ignition::STATUS_VERSION
    )]
    StatusVersion { got: u8 },

    #[error("an encoded frame of {len} bytes is past the board's {budget}-byte message limit")]
    FrameTooLong { len: usize, budget: usize },

    #[error("the bridge reported {0}")]
    Event(pipe::Event),
}

impl From<WiredError> for zapp_smp::TransportError {
    fn from(error: WiredError) -> Self {
        Self::Io(error.to_string())
    }
}
