//! The monitor: one thread per process, turning backend snapshots into events.

use std::sync::mpsc;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

use crate::backend;
use crate::roster::Roster;
use crate::{BleBoard, BleEvent, BleSighting};

/// How long to wait between snapshots when a backend has no change signal.
const POLL: Duration = Duration::from_secs(3);

/// Watches BLE for ZSA boards.
///
/// Mirrors `zapp_core::device::DeviceMonitor`: a thread behind a channel, so a
/// caller sees the same shape whichever transport it is watching, and neither
/// needs an async runtime of its own.
pub struct BleMonitor {
    rx: mpsc::Receiver<BleEvent>,
    boards: Arc<RwLock<Vec<BleBoard>>>,
}

impl BleMonitor {
    /// Starts watching.
    ///
    /// `known` seeds the platforms that cannot enumerate their own bonds —
    /// macOS, which can only retrieve peripherals whose identifiers it is
    /// given. Persisting them is the caller's business.
    ///
    /// Infallible: no backend, no adapter, or Bluetooth switched off all give a
    /// monitor that reports nothing, so a caller never fails to start for want
    /// of a radio.
    pub fn start(known: &[String]) -> Self {
        let (tx, rx) = mpsc::channel();
        let boards = Arc::new(RwLock::new(Vec::new()));

        let Some(mut backend) = backend::for_platform(known) else {
            log::debug!("no BLE backend on this platform");
            return Self { rx, boards };
        };

        let published = Arc::clone(&boards);
        let spawned = thread::Builder::new()
            .name("zapp-ble".into())
            .spawn(move || {
                let mut roster = Roster::default();
                let mut quiet = false;
                loop {
                    match backend.sightings() {
                        Ok(sightings) => {
                            quiet = false;
                            let events = roster.reconcile(boards_among(&sightings));
                            *published.write().unwrap() = roster.boards();
                            for event in events {
                                if tx.send(event).is_err() {
                                    return;
                                }
                            }
                        }
                        // Bluetooth off, or the adapter went away. Say so once,
                        // then keep trying: switching it back on should not
                        // need a restart.
                        Err(err) => {
                            if !quiet {
                                log::warn!("BLE scan failed: {err}");
                                quiet = true;
                            }
                        }
                    }
                    backend.wait_for_change(POLL);
                }
            });

        if let Err(err) = spawned {
            log::warn!("could not start the BLE monitor: {err}");
        }
        Self { rx, boards }
    }

    /// Blocks for the next event. `None` once the monitor has stopped.
    pub fn recv(&self) -> Option<BleEvent> {
        self.rx.recv().ok()
    }

    /// The next event, if one is waiting.
    pub fn try_recv(&self) -> Option<BleEvent> {
        self.rx.try_recv().ok()
    }

    /// Every ZSA board visible right now.
    pub fn snapshot(&self) -> Vec<BleBoard> {
        self.boards.read().unwrap().clone()
    }
}

/// The ZSA boards among everything the platform saw.
fn boards_among(sightings: &[BleSighting]) -> Vec<BleBoard> {
    sightings
        .iter()
        .filter_map(BleBoard::from_sighting)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE;

    #[test]
    fn only_zsa_boards_come_out_of_a_snapshot() {
        let sightings = vec![
            BleSighting {
                key: "ours".into(),
                services: vec![ZSA_BLE_DFU_SERVICE],
                connected: true,
                ..BleSighting::default()
            },
            BleSighting {
                key: "some other keyboard".into(),
                services: vec![0x00001812_0000_1000_8000_00805f9b34fb],
                connected: true,
                ..BleSighting::default()
            },
        ];
        let boards = boards_among(&sightings);
        assert_eq!(boards.len(), 1);
        assert_eq!(boards[0].key, "ours");
    }

    #[test]
    fn a_monitor_without_a_backend_reports_nothing() {
        let monitor = BleMonitor::start(&[]);
        assert_eq!(monitor.try_recv(), None);
        assert!(monitor.snapshot().is_empty());
        // The channel is closed rather than left hanging, so a caller blocking
        // on recv returns instead of waiting for a radio that will never speak.
        assert_eq!(monitor.recv(), None);
    }
}
