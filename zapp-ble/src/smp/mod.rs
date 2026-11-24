//! mcumgr's Simple Management Protocol, over BLE.
//!
//! The protocol itself lives in `zapp_smp`, which knows nothing about radios;
//! this is the GATT transport under it, and is re-exported whole so a caller
//! reaches both through one path.
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
//! channel is opened on changes, which is what `HalfChannel` picks.

use std::time::Duration;

pub use zapp_smp::*;

use crate::backend::{self, GattChannel};
use crate::pick::{self, Picked};
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

/// Where a half's frames go on this transport.
///
/// The left nRF serves its own SMP; the right is reached through the left's
/// relay and the ESB link. Which pair of UUIDs that is is BLE's business, not
/// the protocol's, so it hangs off `Half` from out here.
pub trait HalfChannel {
    fn channel(self) -> (u128, u128);
}

impl HalfChannel for Half {
    fn channel(self) -> (u128, u128) {
        match self {
            Half::Left => (SMP_SERVICE, SMP_CHARACTERISTIC),
            Half::Right => (RELAY_SERVICE, RELAY_CHARACTERISTIC),
        }
    }
}

/// A GATT characteristic, driven as an SMP transport.
///
/// `GattChannel` has a `read` of its own that reads the characteristic's
/// current value; SMP never wants that, only the next notification.
impl SmpTransport for Box<dyn GattChannel> {
    fn mtu(&self) -> usize {
        (**self).mtu()
    }

    fn write(&mut self, packet: &[u8]) -> Result<(), TransportError> {
        (**self).write(packet).map_err(TransportError::from)
    }

    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, TransportError> {
        (**self).notification(timeout).map_err(TransportError::from)
    }
}

impl From<BleError> for TransportError {
    fn from(err: BleError) -> Self {
        TransportError::Io(err.to_string())
    }
}

impl From<BleError> for SmpError {
    fn from(err: BleError) -> Self {
        SmpError::Transport(err.into())
    }
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
    Ok(SmpClient::new(Box::new(channel)))
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
pub fn select<'a>(
    sightings: &'a [BleSighting],
    hint: Option<&str>,
) -> Result<&'a BleSighting, SmpError> {
    match pick::pick(sightings, hint, &speaks_smp) {
        Picked::One(sighting) => Ok(sighting),
        Picked::None => Err(SmpError::NoBoard(hint.map(str::to_string))),
        Picked::Several(names) => Err(SmpError::Ambiguous(names)),
        Picked::Unbonded(name) => Err(BleError::NotBonded(name).into()),
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
        assert!(err.to_string().contains("more than one board matches"));
        assert!(!err.to_string().contains("--device"));
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
        // not this tool's, so the message names that rather than timing out
        // somewhere inside BlueZ.
        let nearby = BleSighting {
            paired: false,
            rssi: Some(-60),
            ..board("dev_A", "Voyager")
        };
        let err = select(&[nearby], None).unwrap_err();
        assert!(
            matches!(err, SmpError::Transport(TransportError::Io(_))),
            "{err:?}"
        );
        assert!(err.to_string().contains("is not bonded to this machine"), "{err}");
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
