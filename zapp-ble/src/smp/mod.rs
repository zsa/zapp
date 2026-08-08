//! mcumgr's Simple Management Protocol, over BLE.
//!
//! This is the standard Zephyr SMP service, the one nRF Connect Device Manager
//! speaks, and it is what updates the firmware on the nRF itself. It is not
//! `zapp_core::device::ids::ZSA_BLE_DFU_SERVICE`, which is ZSA's own service
//! for relaying an STM32 image through the left half — different service,
//! different protocol, different target chip.
//!
//! Everything below the transport is pure: framing in `proto`, payloads in
//! `img`, and a client in `client` that is driven through a `GattChannel` and
//! so can be run against a fake device with no radio in sight.

pub mod cbor;
pub mod client;
pub mod image;
pub mod img;
pub mod proto;

use std::time::Duration;

pub use client::SmpClient;
pub use img::ImageSlot;

use crate::backend;
use crate::{identity, BleError, BleSighting};

/// The Zephyr SMP service, and the one characteristic everything rides on.
pub const SMP_SERVICE: u128 = 0x8d53dc1d_1db7_4cd3_868b_8a527460aa84;
pub const SMP_CHARACTERISTIC: u128 = 0xda2e7828_fbce_4e01_ae9e_261174997c48;

/// What can go wrong driving an update.
#[derive(Debug, thiserror::Error)]
pub enum SmpError {
    #[error(transparent)]
    Ble(#[from] BleError),

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

/// Opens an SMP session to a bonded board.
///
/// `hint` picks one out when more than one is bonded, by advertised name or by
/// the platform handle a listing shows.
pub fn open(hint: Option<&str>) -> Result<SmpClient, SmpError> {
    let mut backend = backend::for_platform(&[]).ok_or(BleError::NoAdapter)?;
    let sightings = backend.sightings()?;
    let key = select(&sightings, hint)?.key.clone();
    // The image is going over this link; the radio should not be spending its
    // airtime looking for other boards while it does.
    backend.stop_discovery();

    let channel = backend.open(&key, SMP_SERVICE, SMP_CHARACTERISTIC)?;
    Ok(SmpClient::new(channel))
}

/// The one board a hint names, out of everything the platform can see.
///
/// Pure, so the awkward cases — nothing, several, one that was never bonded —
/// are settled without a radio.
pub fn select<'a>(
    sightings: &'a [BleSighting],
    hint: Option<&str>,
) -> Result<&'a BleSighting, SmpError> {
    let named = |sighting: &BleSighting| match hint {
        None => true,
        Some(hint) => {
            sighting.key.eq_ignore_ascii_case(hint)
                || sighting
                    .name
                    .as_deref()
                    .is_some_and(|name| name.to_lowercase().contains(&hint.to_lowercase()))
        }
    };

    let matches: Vec<&BleSighting> = sightings
        .iter()
        .filter(|sighting| speaks_smp(sighting) && named(sighting))
        .collect();

    // A board already connected is the one a caller means, and is also the only
    // one that will answer promptly.
    let connected: Vec<&BleSighting> = matches
        .iter()
        .copied()
        .filter(|sighting| sighting.connected)
        .collect();
    let shortlist = if connected.is_empty() {
        matches
    } else {
        connected
    };

    match shortlist.as_slice() {
        [] => Err(SmpError::NoBoard(hint.map(str::to_string))),
        [only] => bonded(only),
        several => Err(SmpError::Ambiguous(
            several.iter().map(|sighting| describe(sighting)).collect(),
        )),
    }
}

/// Whether a sighting could be carrying the SMP service.
///
/// The resolved service list settles it when there is one. Otherwise being a
/// ZSA board is enough to try: BlueZ only lists the services it has resolved,
/// and a bonded board it has not connected to since boot lists none.
fn speaks_smp(sighting: &BleSighting) -> bool {
    sighting.services.contains(&SMP_SERVICE) || identity::is_zsa_board(sighting)
}

fn bonded(sighting: &BleSighting) -> Result<&BleSighting, SmpError> {
    if sighting.paired {
        Ok(sighting)
    } else {
        Err(BleError::NotBonded(describe(sighting)).into())
    }
}

fn describe(sighting: &BleSighting) -> String {
    match (&sighting.name, identity::keyboard_for(sighting)) {
        (Some(name), Some(keyboard)) => format!("{keyboard} {name:?}"),
        (Some(name), None) => format!("{name:?}"),
        (None, Some(keyboard)) => format!("{keyboard} ({})", sighting.key),
        (None, None) => sighting.key.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE;

    fn board(key: &str, name: &str) -> BleSighting {
        BleSighting {
            key: key.into(),
            name: Some(name.into()),
            services: vec![ZSA_BLE_DFU_SERVICE, SMP_SERVICE],
            modalias: Some("usb:v3297p1978d0100".into()),
            paired: true,
            ..BleSighting::default()
        }
    }

    #[test]
    fn the_smp_uuids_are_the_zephyr_ones_and_not_zsas_own() {
        // Written out the way they appear in a scanner, since a transposed
        // digit here would fail as "no such characteristic" and look like a
        // firmware problem.
        assert_eq!(
            identity::parse_uuid("8d53dc1d-1db7-4cd3-868b-8a527460aa84"),
            Some(SMP_SERVICE)
        );
        assert_eq!(
            identity::parse_uuid("da2e7828-fbce-4e01-ae9e-261174997c48"),
            Some(SMP_CHARACTERISTIC)
        );
        assert_ne!(SMP_SERVICE, ZSA_BLE_DFU_SERVICE);
    }

    #[test]
    fn one_bonded_board_needs_no_naming() {
        let boards = vec![board("dev_A", "Voyager")];
        assert_eq!(select(&boards, None).unwrap().key, "dev_A");
    }

    #[test]
    fn anything_that_is_not_a_board_is_passed_over() {
        let boards = vec![
            BleSighting {
                key: "some mouse".into(),
                services: vec![0x00001812_0000_1000_8000_00805f9b34fb],
                paired: true,
                ..BleSighting::default()
            },
            board("dev_A", "Voyager"),
        ];
        assert_eq!(select(&boards, None).unwrap().key, "dev_A");
    }

    #[test]
    fn a_hint_picks_one_out_by_name_or_by_handle() {
        let boards = vec![board("dev_A", "Voyager"), board("dev_B", "Bench unit")];
        assert_eq!(select(&boards, Some("bench")).unwrap().key, "dev_B");
        assert_eq!(select(&boards, Some("BENCH")).unwrap().key, "dev_B");
        assert_eq!(select(&boards, Some("dev_A")).unwrap().key, "dev_A");
    }

    #[test]
    fn two_boards_and_no_hint_is_a_question_rather_than_a_guess() {
        let boards = vec![board("dev_A", "Voyager"), board("dev_B", "Bench unit")];
        let err = select(&boards, None).unwrap_err();
        assert!(matches!(err, SmpError::Ambiguous(ref names) if names.len() == 2), "{err:?}");
        assert!(err.to_string().contains("--device"));
    }

    #[test]
    fn the_connected_board_wins_over_ones_that_are_merely_bonded() {
        let boards = vec![
            board("dev_A", "Voyager"),
            BleSighting {
                connected: true,
                ..board("dev_B", "Voyager")
            },
        ];
        assert_eq!(select(&boards, None).unwrap().key, "dev_B");
    }

    #[test]
    fn a_hint_that_matches_nothing_says_so() {
        let boards = vec![board("dev_A", "Voyager")];
        let err = select(&boards, Some("nowhere")).unwrap_err();
        assert!(matches!(err, SmpError::NoBoard(Some(ref hint)) if hint == "nowhere"));
        assert!(select(&[], None).is_err());
    }

    #[test]
    fn a_board_that_was_never_bonded_is_refused_with_a_reason() {
        // Advertising in range, never paired: pairing is the desktop's job,
        // not this tool's, so the message has to say so rather than time out
        // somewhere inside BlueZ.
        let nearby = BleSighting {
            paired: false,
            rssi: Some(-60),
            ..board("dev_A", "Voyager")
        };
        let err = select(&[nearby], None).unwrap_err();
        assert!(
            matches!(err, SmpError::Ble(BleError::NotBonded(_))),
            "{err:?}"
        );
        assert!(err.to_string().contains("pair it first"), "{err}");
    }

    #[test]
    fn a_bonded_board_whose_services_are_not_resolved_is_still_a_candidate() {
        // BlueZ lists no services for a board it has not connected to since it
        // started; refusing on that would make the tool unusable from cold.
        let cold = BleSighting {
            services: vec![],
            advertised: vec![0x32970101_e5aa_4f42_a596_1ea5a0200001],
            ..board("dev_A", "Voyager")
        };
        assert_eq!(select(&[cold], None).unwrap().key, "dev_A");
    }
}
