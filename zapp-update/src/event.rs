//! What an update says while it runs.
//!
//! Every update takes an `&mut dyn FnMut(Event)` and calls it as things happen.
//! Two kinds of event are in here. [`Event::Phase`] names the step the update
//! is on, for a front end that lists them. The rest describe the one line a
//! terminal shows: a wait with a message, a transfer with a bar, and the few
//! lines a revert or a bond prints. The CLI draws exactly those; a GUI can use
//! either kind or both.

use zapp_core::flash::FlashProgress;
use zapp_smp::Half;

/// One thing an update has to say.
#[derive(Debug, Clone)]
pub enum Event {
    /// The update has moved on to another step.
    Phase(Phase),
    /// A wait starts, or the wait already running now waits on something else.
    Wait(Wait),
    /// The wait that was running is over.
    WaitOver,
    /// An image transfer to `chip` starts.
    TransferStarted(Chip),
    /// How far the transfer to `chip` has got.
    Transfer(Chip, FlashProgress),
    /// The transfer to `chip` stopped short. The failure is the update's error.
    TransferFailed(Chip),
    /// A half is being put back on its previous firmware, over Bluetooth.
    Reverting(Half, RevertStep),
    /// A failed wired run left both halves as they were, for reading their
    /// counters over SWD.
    KeptStateOnFailure,
    /// A successful wired run left the STM32 in Ignition, and the keyboard
    /// was not checked.
    KeptIgnition,
    /// One half took the pairing key and read `bytes` of it back.
    Provisioned { half: Half, bytes: usize },
}

/// The steps of an update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Looking for the keyboard: its bootloader on USB, or the board over
    /// Bluetooth.
    Connecting,
    /// Reading what each half is running.
    Reading,
    /// Opening ignition's session on both halves (cable only).
    OpeningSession,
    /// Declaring the update to the left half (Bluetooth only).
    Declaring,
    /// One half's turn: its image is written, if it needs one.
    Writing(Half),
    /// A half is restarting: onto its new image when it was marked for test,
    /// on the one it runs when it was only written under `force`.
    Restarting(Half),
    /// Waiting for the right half to come back before the left is written
    /// (Bluetooth only).
    WaitingForRight,
    /// Reading both halves back on what they came up running.
    Verifying,
    /// Both halves matched, and both are being confirmed.
    Confirming,
    /// Both halves are kept.
    Confirmed,
    /// The pair did not match, and both halves are being put back.
    Reverting,
    /// Waiting for the keyboard to come back on USB with both halves (cable
    /// only).
    WaitingForKeyboard,
}

/// What a wait is waiting on. [`Wait::message`] is the line the CLI shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wait {
    /// Connecting to one half over Bluetooth.
    Connecting(Half),
    /// For the right half to come back from its restart.
    RightRestart,
    /// For both halves to come back from their restarts.
    HalvesRestart,
    /// For both halves to take their confirms.
    Confirming,
    /// For the keyboard's bootloader to show up.
    Bootloader,
    /// For ignition to find the right half on the TRRS bus.
    RightHalf,
    /// The same wait, once it has gone on long enough to ask for the cable.
    RightHalfCable,
    /// For the keyboard to come back on USB after the update.
    KeyboardRestart,
}

impl Wait {
    /// The line this wait is shown as.
    pub fn message(self) -> &'static str {
        match self {
            Wait::Connecting(Half::Left) => "Connecting over Bluetooth...",
            Wait::Connecting(Half::Right) => "Connecting to the right half over Bluetooth...",
            Wait::RightRestart => "Waiting for the right half to restart...",
            Wait::HalvesRestart => "Waiting for the halves to restart...",
            Wait::Confirming => "Confirming both halves...",
            Wait::Bootloader => "Waiting for keyboard in bootloader mode...",
            Wait::RightHalf => "Waiting for the right half...",
            Wait::RightHalfCable => {
                "Waiting for the right half. Check the cable between the halves..."
            }
            Wait::KeyboardRestart => "Waiting for the keyboard to restart...",
        }
    }
}

/// What an image is being written to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// The STM32, over ZSA's update service on the left half.
    Stm32,
    /// One half's nRF.
    Nrf(Half),
}

/// How putting one half back went.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RevertStep {
    /// The half could not be reached, so nothing was sent to it.
    Unreachable,
    /// The half was told to restart.
    Restarting,
    /// The restart could not be sent, and why.
    Failed(String),
}

/// The sink every update reports to, with the one wait it has up.
///
/// A wait is one line: a new one replaces the message of the one already up,
/// and clearing only says so when something was up.
pub(crate) struct Reporter<'a> {
    sink: &'a mut dyn FnMut(Event),
    waiting: bool,
}

impl<'a> Reporter<'a> {
    pub(crate) fn new(sink: &'a mut dyn FnMut(Event)) -> Self {
        Self {
            sink,
            waiting: false,
        }
    }

    pub(crate) fn emit(&mut self, event: Event) {
        (self.sink)(event);
    }

    pub(crate) fn phase(&mut self, phase: Phase) {
        self.emit(Event::Phase(phase));
    }

    /// Puts a wait up, or changes the one that is up.
    pub(crate) fn wait(&mut self, wait: Wait) {
        self.waiting = true;
        self.emit(Event::Wait(wait));
    }

    /// Changes the wait that is up, and puts none up if none is.
    pub(crate) fn rewait(&mut self, wait: Wait) {
        if self.waiting {
            self.emit(Event::Wait(wait));
        }
    }

    /// Takes the wait down, if one is up.
    pub(crate) fn clear(&mut self) {
        if self.waiting {
            self.waiting = false;
            self.emit(Event::WaitOver);
        }
    }
}
