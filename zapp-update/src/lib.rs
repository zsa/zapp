//! Updating a Voyager 2, as a library.
//!
//! The order an update is done in, what is checked first, and how a pair of
//! halves is kept or put back, for any front end: the `zapp` CLI draws it on a
//! terminal, and a desktop app can draw it however it likes. Nothing in here
//! prints, prompts or exits. Progress goes to an `&mut dyn FnMut(Event)` (see
//! [`event`]) and failures come back as an [`UpdateError`].
//!
//! # Blocking
//!
//! Everything here blocks, because everything under it does: the Bluetooth
//! stack is driven through blocking calls (zbus's blocking API on Linux) and
//! the cable through synchronous USB transfers, with sleeps between polls. An
//! update takes minutes. An async app runs it on a thread of its own, e.g. in
//! Tauri:
//!
//! ```ignore
//! let handle = app.clone();
//! tauri::async_runtime::spawn_blocking(move || {
//!     let bundle = zapp_update::load_bundle(&path)?;
//!     let options = zapp_update::Options::default();
//!     let mut events = |event| { let _ = handle.emit("update", format!("{event:?}")); };
//!     match zapp_update::route() {
//!         zapp_update::Transport::Usb => zapp_update::wired::update_bundle(&bundle, &options, &mut events),
//!         zapp_update::Transport::Bluetooth => zapp_update::ble::update_bundle(&bundle, &options, &mut events),
//!     }
//! });
//! ```
//!
//! There is no cancellation: an update stopped half-way is what the confirm
//! step exists to recover from, not something to offer as a button.

use std::path::Path;
use std::time::Duration;

use anyhow::Context;

pub mod ble;
pub mod confirm;
mod error;
pub mod event;
pub mod guard;
pub mod probe;
pub mod text;
pub mod wired;

pub use error::{ErrorKind, UpdateError};
pub use event::{Chip, Event, Phase, RevertStep, Wait};
pub use probe::{route, Transport};
pub use zapp_smp::{Bundle, Half};

/// What an update did to the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// At least one half was written or confirmed, and the pair was kept.
    Flashed,
    /// Both halves already carried the bundle, so nothing was written.
    UpToDate,
}

/// Everything an update can be told, with the CLI's defaults.
#[derive(Debug, Clone)]
pub struct Options {
    /// Write a half even if it already carries the bundle's image, or (over the
    /// cable) the bundle is older than what is installed.
    pub force: bool,
    /// Which board, by advertised name or by the platform handle a listing
    /// shows. Bluetooth only; the cable only ever has one keyboard on it.
    pub device: Option<String>,
    /// How long to wait for each reply from a half.
    pub reply_timeout: Duration,
    /// Leave a failed wired run exactly as it is, both halves in their
    /// bootloaders, for reading their counters over SWD.
    pub keep_state_on_failure: bool,
    /// Leave the STM32 in Ignition after a successful wired run instead of
    /// restarting it, for reading a `make DIAG=1` Ignition's record. The
    /// keyboard is then not checked.
    pub keep_ignition: bool,
    /// How requests are cut up over Bluetooth.
    pub link: BleLink,
    /// How long each wait may take.
    pub timings: Timings,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            force: false,
            device: None,
            reply_timeout: Duration::from_secs(15),
            keep_state_on_failure: false,
            keep_ignition: false,
            link: BleLink::default(),
            timings: Timings::default(),
        }
    }
}

/// The Bluetooth client's tuning. `None` everywhere is what the CLI uses.
#[derive(Debug, Clone, Copy, Default)]
pub struct BleLink {
    /// Bytes of payload per Bluetooth write, the ATT MTU less its 3-byte
    /// header. `None` takes what the platform reports.
    pub mtu: Option<usize>,
    /// Bluetooth writes to spend on one request. `None` takes 5.
    pub writes: Option<usize>,
    /// Largest request to send, fragmented across writes. Overrides `writes`.
    /// `None` takes the buffer the board reports, capped by `writes`.
    pub chunk: Option<usize>,
}

/// How long each wait may take. The defaults are the CLI's.
#[derive(Debug, Clone, Copy)]
pub struct Timings {
    /// Cable: how long to wait for the bootloader to show up.
    pub bootloader_wait: Duration,
    /// Cable: how long to keep asking for a session while the right half is
    /// missing.
    pub right_half_wait: Duration,
    /// Cable: how long both halves get to swap and answer the confirm.
    pub settle: Duration,
    /// Cable: how long the keyboard gets to come back on USB afterwards.
    pub keyboard_restart: Duration,
    /// Bluetooth: how long to leave a half alone after telling it to restart.
    pub reboot_settle: Duration,
    /// Bluetooth: how long to keep trying to reach a half that is coming back.
    pub reboot_timeout: Duration,
    /// Bluetooth: between attempts to reach a half that has not answered yet.
    pub retry_every: Duration,
    /// Bluetooth: how long the left's relay lock lingers after the right was
    /// touched.
    pub relay_settle: Duration,
    /// Bluetooth: how long the STM32 gets to reach its bootloader.
    pub stm32_bootloader_wait: Duration,
}

impl Default for Timings {
    fn default() -> Self {
        Self {
            bootloader_wait: wired::WAIT_FOR_BOOTLOADER,
            right_half_wait: wired::WAIT_FOR_RIGHT,
            settle: wired::SETTLE_TIMEOUT,
            keyboard_restart: wired::CONFIRM_WINDOW,
            reboot_settle: ble::REBOOT_SETTLE,
            reboot_timeout: ble::REBOOT_TIMEOUT,
            retry_every: ble::RETRY_EVERY,
            relay_settle: ble::RELAY_SETTLE,
            stm32_bootloader_wait: ble::BLE_BOOTLOADER_WAIT,
        }
    }
}

/// Reads and parses a `.zsab` bundle.
pub fn load_bundle(path: &Path) -> Result<Bundle, UpdateError> {
    let parsed = std::fs::read(path)
        .with_context(|| format!("Failed to read {}", path.display()))
        .and_then(|blob| {
            zapp_smp::bundle::parse(&blob)
                .with_context(|| format!("{} is not a bundle this build can read", path.display()))
        });
    parsed.map_err(|err| error::classify(error::kind_of(ErrorKind::Bundle, None, err)))
}
