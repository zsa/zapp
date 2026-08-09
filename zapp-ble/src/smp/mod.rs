//! mcumgr's Simple Management Protocol, over BLE.
//!
//! This is the standard Zephyr SMP service, the one nRF Connect Device Manager
//! speaks, and it is what updates the firmware on the nRF itself. It is not
//! `zapp_core::device::ids::ZSA_BLE_DFU_SERVICE`, which is ZSA's own service
//! for relaying an STM32 image through the left half — different service,
//! different protocol, different target chip.
//!
//! The right half runs the same SMP server behind a second ZSA service on the
//! left, which relays whole frames over the ESB radio link without reading
//! them. Same framing, same commands, same client: only the pair of UUIDs the
//! channel is opened on changes, which is what `Half` picks.
//!
//! Everything below the transport is pure: framing in `proto`, payloads in
//! `img`, and a client in `client` that is driven through a `GattChannel` and
//! so can be run against a fake device with no radio in sight.

pub mod bundle;
pub mod cbor;
pub mod client;
pub mod image;
pub mod img;
pub mod proto;
pub mod session;
pub mod zsa_err;

use std::fmt;
use std::str::FromStr;
use std::time::Duration;

pub use bundle::Bundle;
pub use client::SmpClient;
pub use img::ImageSlot;

use crate::backend;
use crate::{identity, Battery, BleError, BleSighting};

/// The Zephyr SMP service, and the one characteristic everything rides on.
pub const SMP_SERVICE: u128 = 0x8d53dc1d_1db7_4cd3_868b_8a527460aa84;
pub const SMP_CHARACTERISTIC: u128 = 0xda2e7828_fbce_4e01_ae9e_261174997c48;

/// ZSA's relay service on the left half, carrying the right half's SMP frames.
///
/// `32970201`/`32970202` in ZSA's UUID space (VID `0x3297`), a sub-range of its
/// own: `32970001`-`32970004` is the STM32 update service and `32970101` up is
/// product identity.
pub const RELAY_SERVICE: u128 = 0x32970201_e5aa_4f42_a596_1ea5a0200001;
pub const RELAY_CHARACTERISTIC: u128 = 0x32970202_e5aa_4f42_a596_1ea5a0200001;

/// Which half of the keyboard a session talks to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Half {
    /// The left nRF's own SMP service, served by the chip the link ends at.
    #[default]
    Left,
    /// The right nRF, reached through the left's relay and the ESB link.
    Right,
}

impl Half {
    /// The service and characteristic a session on this half is opened on.
    pub fn channel(self) -> (u128, u128) {
        match self {
            Half::Left => (SMP_SERVICE, SMP_CHARACTERISTIC),
            Half::Right => (RELAY_SERVICE, RELAY_CHARACTERISTIC),
        }
    }

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

/// Opens an SMP session to a bonded board, on one half or the other.
///
/// `hint` picks one out when more than one is bonded, by advertised name or by
/// the platform handle a listing shows. Either half is reached over the same
/// link to the left: the right's frames are relayed on from there.
pub fn open(hint: Option<&str>, half: Half) -> Result<SmpClient, SmpError> {
    let mut backend = backend::for_platform(&[]).ok_or(BleError::NoAdapter)?;
    let sightings = backend.sightings()?;
    let key = select(&sightings, hint)?.key.clone();
    // The image is going over this link; the radio should not be spending its
    // airtime looking for other boards while it does.
    backend.stop_discovery();

    let (service, characteristic) = half.channel();
    let channel = backend.open(&key, service, characteristic)?;
    Ok(SmpClient::new(channel))
}

/// What the halves have left, in one look, without starting a monitor.
///
/// A `BleMonitor` would keep a discovery window opening behind the update, and
/// a scanning radio is airtime the link does not get — over a transfer measured
/// in minutes that is worth avoiding.
///
/// A board only publishes its levels while it is connected, so this says
/// nothing at all about one that is not; the battery gate reads that as unknown
/// and allows it. Calling this straight after a session is open is what makes
/// it answer.
pub fn battery(hint: Option<&str>) -> Result<Battery, SmpError> {
    let mut backend = backend::for_platform(&[]).ok_or(BleError::NoAdapter)?;
    let sightings = backend.sightings()?;
    let battery = select(&sightings, hint)?.battery;
    backend.stop_discovery();
    Ok(battery)
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

/// Whether a sighting could be carrying either SMP service.
///
/// The resolved service list settles it when there is one. Otherwise being a
/// ZSA board is enough to try: BlueZ only lists the services it has resolved,
/// and a bonded board it has not connected to since boot lists none. Both
/// halves live behind the same board, so this does not narrow by half — a
/// board that turns out not to carry the relay says so when the channel is
/// opened, which is a clearer error than "no board".
fn speaks_smp(sighting: &BleSighting) -> bool {
    sighting.services.contains(&SMP_SERVICE)
        || sighting.services.contains(&RELAY_SERVICE)
        || identity::is_zsa_board(sighting)
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
    fn the_relay_uuids_are_in_zsas_space_and_clear_of_the_ranges_already_taken() {
        assert_eq!(
            identity::parse_uuid("32970201-e5aa-4f42-a596-1ea5a0200001"),
            Some(RELAY_SERVICE)
        );
        assert_eq!(
            identity::parse_uuid("32970202-e5aa-4f42-a596-1ea5a0200001"),
            Some(RELAY_CHARACTERISTIC)
        );
        // 32970001-32970004 is the STM32 update service, 32970101 up is
        // product identity, and neither may be shadowed.
        assert_ne!(RELAY_SERVICE, ZSA_BLE_DFU_SERVICE);
        assert_ne!(RELAY_SERVICE, 0x32970101_e5aa_4f42_a596_1ea5a0200001);
        assert_ne!(RELAY_SERVICE, SMP_SERVICE);
    }

    #[test]
    fn each_half_opens_its_own_channel() {
        assert_eq!(Half::Left.channel(), (SMP_SERVICE, SMP_CHARACTERISTIC));
        assert_eq!(Half::Right.channel(), (RELAY_SERVICE, RELAY_CHARACTERISTIC));
        assert_ne!(Half::Left.channel(), Half::Right.channel());
        assert_eq!(Half::default(), Half::Left);
    }

    #[test]
    fn a_half_is_named_the_way_it_is_typed() {
        assert_eq!("left".parse(), Ok(Half::Left));
        assert_eq!("RIGHT".parse(), Ok(Half::Right));
        assert_eq!("r".parse(), Ok(Half::Right));
        assert_eq!(Half::Right.to_string(), "right");
        assert!("middle".parse::<Half>().is_err());
    }

    #[test]
    fn a_board_listing_only_the_relay_service_is_still_a_candidate() {
        let relay_only = BleSighting {
            services: vec![RELAY_SERVICE],
            modalias: None,
            advertised: vec![],
            ..board("dev_A", "Voyager")
        };
        assert_eq!(select(&[relay_only], None).unwrap().key, "dev_A");
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
