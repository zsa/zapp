//! Updating the nRFs over the wire, through ignition.
//!
//! The keyboard's two nRF halves both run MCUboot, and MCUboot's serial
//! recovery speaks SMP over a line-based codec — base64 with a length and a
//! checksum, cut into lines short enough for its receive buffer. Getting those
//! lines to a half is ignition's job: it bridges the left half over UART and
//! the right half over the shared TRRS I2C bus, and the host reaches both
//! through one vendor interface on its USB device.
//!
//! The two codecs that describes are here, and neither one does I/O:
//!
//! * [`nlip`] — the line codec, pure functions.
//! * [`pipe`] — the framing over ignition's bulk endpoint, a state machine.

pub mod nlip;
pub mod pipe;

/// What can go wrong on the way to a half.
#[derive(Debug, thiserror::Error)]
pub enum WiredError {
    #[error(transparent)]
    Nlip(#[from] nlip::NlipError),

    #[error(transparent)]
    Pipe(#[from] pipe::PipeError),
}

impl From<WiredError> for zapp_smp::TransportError {
    fn from(error: WiredError) -> Self {
        Self::Io(error.to_string())
    }
}
