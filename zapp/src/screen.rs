//! Drawing an update's events on the terminal: one spinner for whatever is
//! being waited on, one bar for whatever is being written, and the few lines a
//! revert or a bond prints.

use indicatif::ProgressBar;

use zapp_core::flash::FlashProgress;
use zapp_update::text::titled;
use zapp_update::{Chip, Event, RevertStep};

use crate::{flash_bar, new_spinner};

/// The line an update owns while it runs.
#[derive(Default)]
pub struct Screen {
    spinner: Option<ProgressBar>,
    bar: Option<ProgressBar>,
}

impl Screen {
    /// Draws one event.
    pub fn show(&mut self, event: Event) {
        match event {
            Event::Phase(_) => {}
            Event::Wait(wait) => match &self.spinner {
                Some(spinner) => spinner.set_message(wait.message()),
                None => self.spinner = Some(new_spinner(wait.message())),
            },
            Event::WaitOver => {
                if let Some(spinner) = self.spinner.take() {
                    spinner.finish_and_clear();
                }
            }
            Event::TransferStarted(chip) => {
                let prefix = match chip {
                    Chip::Stm32 => "",
                    Chip::Nrf(half) => titled(half),
                };
                self.bar = Some(flash_bar(100, prefix));
            }
            Event::Transfer(_, progress) => {
                if let Some(bar) = &self.bar {
                    advance(bar, progress);
                }
            }
            Event::TransferFailed(_) => {
                // The bar owns its line until something finishes it.
                if let Some(bar) = self.bar.take() {
                    bar.finish_and_clear();
                }
            }
            Event::Reverting(half, step) => match step {
                RevertStep::Unreachable => println!("  {half:<5}: cannot be reached"),
                RevertStep::Restarting => println!("  {half:<5}: restarting"),
                RevertStep::Failed(err) => println!("  {half:<5}: could not be restarted ({err})"),
            },
            Event::KeptStateOnFailure => {
                println!();
                println!(
                    "ZAPP_KEEP_STATE_ON_FAILURE: both halves left in their bootloaders, \
                     nothing reset."
                );
            }
            Event::KeptIgnition => {
                println!(
                    "ZAPP_KEEP_IGNITION: left in Ignition, not restarted; the keyboard was \
                     not checked."
                );
            }
            Event::Provisioned { half, bytes } => {
                let label = format!("{} half", titled(half));
                println!("  {label:<10}: wrote and read back {bytes} bytes.");
            }
        }
    }
}

/// Moves the bar on.
pub fn advance(bar: &ProgressBar, progress: FlashProgress) {
    match progress {
        FlashProgress::Erasing {
            bytes_erased,
            total_bytes,
        } => {
            bar.set_position(((bytes_erased * 100) / total_bytes) as u64);
            bar.set_message("Erasing...");
        }
        FlashProgress::Writing {
            bytes_written,
            total_bytes,
        } => {
            bar.set_position(((bytes_written * 100) / total_bytes) as u64);
            bar.set_message("Writing...");
        }
        FlashProgress::Resetting => {
            bar.set_position(100);
            bar.set_message("Resetting...");
        }
        FlashProgress::Complete => bar.finish_with_message("Done!"),
    }
}
