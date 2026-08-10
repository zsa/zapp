//! mcumgr's Simple Management Protocol, over any transport.
//!
//! This is the standard Zephyr SMP service, the one nRF Connect Device Manager
//! speaks, and it is what updates the firmware on the nRF itself.
//!
//! Everything here is transport-agnostic: framing in `proto`, payloads in
//! `img`, and a client in `client` that is driven through an `SmpTransport` —
//! a GATT characteristic, a USB pipe, or a fake device with no wire in sight.
//! Which service, which characteristic, which half: that belongs to the crate
//! that owns the transport.

pub mod bundle;
pub mod cbor;
pub mod client;
#[cfg(any(test, feature = "test-double"))]
pub mod fake;
pub mod image;
pub mod img;
pub mod proto;
pub mod session;
pub mod transport;
pub mod zsa_err;

use std::fmt;
use std::str::FromStr;
use std::time::Duration;

pub use bundle::Bundle;
pub use client::SmpClient;
pub use img::ImageSlot;
pub use transport::{SmpTransport, TransportError};

/// Which half of the keyboard a session talks to.
///
/// Both halves run the same SMP server and take the same commands; only the
/// way the frames get there differs, which is the transport's business.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Half {
    #[default]
    Left,
    Right,
}

impl Half {
    pub fn as_str(self) -> &'static str {
        match self {
            Half::Left => "left",
            Half::Right => "right",
        }
    }
}

impl fmt::Display for Half {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for Half {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text.to_ascii_lowercase().as_str() {
            "left" | "l" => Ok(Half::Left),
            "right" | "r" => Ok(Half::Right),
            other => Err(format!("{other:?} is not a half; expected left or right")),
        }
    }
}

/// What can go wrong driving an update.
#[derive(Debug, thiserror::Error)]
pub enum SmpError {
    #[error(transparent)]
    Transport(#[from] TransportError),

    #[error("no bonded ZSA board to talk to{}", match .0 {
        Some(hint) => format!(" matching {hint:?}"),
        None => String::new(),
    })]
    NoBoard(Option<String>),

    #[error("more than one board matches: {}; name one with --device", .0.join(", "))]
    Ambiguous(Vec<String>),

    #[error("a header needs 8 bytes, got {0}")]
    ShortHeader(usize),

    #[error("{0} is not an SMP op")]
    BadOp(u8),

    #[error("a payload of {0} bytes will not fit the 16-bit length field")]
    PayloadTooLong(usize),

    #[error("an MTU of {0} cannot carry anything")]
    MtuTooSmall(usize),

    #[error("{0} bytes per request leaves no room for image data")]
    ChunkTooSmall(usize),

    #[error("the reply is about group {}, command {} — this asked group {}, command {}",
        .got.0, .got.1, .expected.0, .expected.1)]
    Mismatch {
        expected: (u16, u8),
        got: (u16, u8),
    },

    #[error("no reply within {0:?}")]
    Timeout(Duration),

    #[error("bad CBOR from the board: {0}")]
    Cbor(String),

    #[error("{0}")]
    Malformed(String),

    /// The device said no. `rc` 0 is success and never reaches here.
    #[error("the board refused: {meaning} (group {group}, rc {rc})")]
    Remote {
        group: u16,
        rc: i64,
        meaning: &'static str,
    },

    #[error("the board asked for offset {off} of a {total}-byte image")]
    BadOffset { off: u32, total: u32 },

    #[error("the board stopped taking the image at offset {0}")]
    Stalled(u32),

    #[error("an image of {0} bytes is too large to announce")]
    ImageTooLarge(usize),

    #[error("the image is empty")]
    EmptyImage,
}
