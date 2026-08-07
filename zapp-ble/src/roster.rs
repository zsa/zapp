//! Turning snapshots of what is visible into presence changes.

use std::collections::{BTreeMap, BTreeSet};

use crate::{BleBoard, BleEvent};

/// Snapshots a board may be missing from before it is declared gone.
///
/// One dropped scan window is ordinary, and a loss is not free downstream: it
/// costs its consumer a reconnect decision.
const MISSES_BEFORE_LOST: u8 = 2;

/// The boards seen so far, and what changes between snapshots.
#[derive(Debug, Default)]
pub struct Roster {
    boards: BTreeMap<String, BleBoard>,
    misses: BTreeMap<String, u8>,
}

impl Roster {
    /// Folds one snapshot in and reports what changed.
    pub fn reconcile(&mut self, seen: Vec<BleBoard>) -> Vec<BleEvent> {
        let mut events = Vec::new();
        let mut present = BTreeSet::new();

        for board in seen {
            present.insert(board.key.clone());
            self.misses.remove(&board.key);
            match self.boards.get(&board.key) {
                Some(known) if *known == board => continue,
                Some(_) => events.push(BleEvent::Changed(board.clone())),
                None => events.push(BleEvent::Found(board.clone())),
            }
            self.boards.insert(board.key.clone(), board);
        }

        let absent: Vec<String> = self
            .boards
            .keys()
            .filter(|key| !present.contains(*key))
            .cloned()
            .collect();
        for key in absent {
            let misses = self.misses.entry(key.clone()).or_insert(0);
            *misses += 1;
            if *misses >= MISSES_BEFORE_LOST {
                self.boards.remove(&key);
                self.misses.remove(&key);
                events.push(BleEvent::Lost { key });
            }
        }

        events
    }

    /// Every board currently visible.
    pub fn boards(&self) -> Vec<BleBoard> {
        self.boards.values().cloned().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Presence;
    use zapp_core::device::ids::Keyboard;

    fn board(presence: Presence) -> BleBoard {
        BleBoard {
            key: "dev_E8_6B_FC_47_E2_64".into(),
            name: Some("Voyager".into()),
            keyboard: Some(Keyboard::Voyager2),
            presence,
        }
    }

    #[test]
    fn a_new_board_is_found_once() {
        let mut roster = Roster::default();
        assert_eq!(
            roster.reconcile(vec![board(Presence::Connected)]),
            vec![BleEvent::Found(board(Presence::Connected))]
        );
        // Seeing the same board again says nothing.
        assert_eq!(roster.reconcile(vec![board(Presence::Connected)]), vec![]);
    }

    #[test]
    fn a_change_of_presence_is_reported() {
        let mut roster = Roster::default();
        roster.reconcile(vec![board(Presence::Connected)]);
        assert_eq!(
            roster.reconcile(vec![board(Presence::Paired)]),
            vec![BleEvent::Changed(board(Presence::Paired))]
        );
        assert_eq!(roster.boards(), vec![board(Presence::Paired)]);
    }

    #[test]
    fn a_board_is_only_lost_after_two_misses() {
        let mut roster = Roster::default();
        roster.reconcile(vec![board(Presence::Nearby)]);

        assert_eq!(roster.reconcile(vec![]), vec![], "one miss is not a loss");
        assert_eq!(
            roster.reconcile(vec![]),
            vec![BleEvent::Lost {
                key: "dev_E8_6B_FC_47_E2_64".into()
            }]
        );
        assert!(roster.boards().is_empty());
        // And it stays gone rather than being reported lost again.
        assert_eq!(roster.reconcile(vec![]), vec![]);
    }

    #[test]
    fn a_board_that_comes_back_before_the_second_miss_is_not_lost() {
        let mut roster = Roster::default();
        roster.reconcile(vec![board(Presence::Nearby)]);
        roster.reconcile(vec![]);
        assert_eq!(roster.reconcile(vec![board(Presence::Nearby)]), vec![]);
        // The miss counter reset, so a single miss again is still not a loss.
        assert_eq!(roster.reconcile(vec![]), vec![]);
        assert_eq!(roster.boards(), vec![board(Presence::Nearby)]);
    }

    #[test]
    fn boards_are_tracked_apart() {
        let mut roster = Roster::default();
        let other = BleBoard {
            key: "dev_F8_DE_F9_BF_5F_5F".into(),
            ..board(Presence::Paired)
        };
        let events = roster.reconcile(vec![board(Presence::Connected), other.clone()]);
        assert_eq!(events.len(), 2);

        // One leaves, the other does not.
        roster.reconcile(vec![board(Presence::Connected)]);
        assert_eq!(
            roster.reconcile(vec![board(Presence::Connected)]),
            vec![BleEvent::Lost { key: other.key }]
        );
        assert_eq!(roster.boards(), vec![board(Presence::Connected)]);
    }
}
