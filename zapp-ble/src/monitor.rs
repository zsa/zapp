//! The monitor: one thread per process, turning backend snapshots into events.

use std::sync::mpsc;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

use crate::backend::{self, Backend};
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
        Self::watching(backend::for_platform(known))
    }

    /// Starts on a given backend, or on nothing.
    fn watching(backend: Option<Box<dyn Backend>>) -> Self {
        let (tx, rx) = mpsc::channel();
        let boards = Arc::new(RwLock::new(Vec::new()));

        let Some(mut backend) = backend else {
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
    use crate::Presence;
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
        let monitor = BleMonitor::watching(None);
        assert_eq!(monitor.try_recv(), None);
        assert!(monitor.snapshot().is_empty());
        // The channel is closed rather than left hanging, so a caller blocking
        // on recv returns instead of waiting for a radio that will never speak.
        assert_eq!(monitor.recv(), None);
    }

    /// Replays scripted snapshots, then keeps reporting the last one.
    struct Scripted(std::sync::Mutex<std::collections::VecDeque<Vec<BleSighting>>>);

    impl Backend for Scripted {
        fn sightings(&mut self) -> Result<Vec<BleSighting>, crate::BleError> {
            let mut rounds = self.0.lock().unwrap();
            Ok(rounds.pop_front().unwrap_or_default())
        }

        fn wait_for_change(&mut self, _timeout: Duration) {
            thread::sleep(Duration::from_millis(1));
        }
    }

    fn scripted(rounds: Vec<Vec<BleSighting>>) -> BleMonitor {
        BleMonitor::watching(Some(Box::new(Scripted(std::sync::Mutex::new(
            rounds.into_iter().collect(),
        )))))
    }

    fn ours() -> BleSighting {
        BleSighting {
            key: "dev_E8_6B_FC_47_E2_64".into(),
            services: vec![ZSA_BLE_DFU_SERVICE],
            modalias: Some("usb:v3297p1978d0100".into()),
            connected: true,
            paired: true,
            ..BleSighting::default()
        }
    }

    #[test]
    fn a_board_appearing_and_going_away_reaches_the_caller() {
        let away = BleSighting {
            connected: false,
            ..ours()
        };
        // Connected, then merely bonded, then gone from the platform entirely.
        let monitor = scripted(vec![vec![ours()], vec![away], vec![]]);

        let found = monitor.recv().expect("the board arrives");
        assert!(
            matches!(&found, BleEvent::Found(board) if board.presence == Presence::Connected),
            "{found:?}"
        );
        let changed = monitor.recv().expect("it stops being connected");
        assert!(
            matches!(&changed, BleEvent::Changed(board) if board.presence == Presence::Paired),
            "{changed:?}"
        );
        // Two empty rounds, since one miss is not a loss.
        let lost = monitor.recv().expect("it leaves");
        assert!(matches!(lost, BleEvent::Lost { .. }), "{lost:?}");
    }

    #[test]
    fn the_snapshot_follows_the_events() {
        let monitor = scripted(vec![vec![ours()]]);
        monitor.recv().expect("the board arrives");
        let snapshot = monitor.snapshot();
        assert_eq!(snapshot.len(), 1);
        assert_eq!(
            snapshot[0].keyboard,
            Some(zapp_core::device::ids::Keyboard::Voyager2)
        );
    }
}
