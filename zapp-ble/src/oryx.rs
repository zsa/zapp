//! The Oryx live protocol, over Bluetooth.
//!
//! The same 0xFF60 raw-HID reports the cable carries, on a GATT service of the
//! left half: one report per write to the command characteristic, one per
//! notification from the event characteristic, no report ID either way. The
//! handshake and the read loop are `zapp_core::hid`'s; this is only the
//! carrier under them.
//!
//! | | | |
//! |---|---|---|
//! | `32970302` | command | written, with or without a response: one OUT report |
//! | `32970303` | event   | notify: one 32-byte IN report each |

use std::io;
use std::time::Duration;

use zapp_core::hid::{HidConnection, ReportLink};
use zapp_hid_proto::REPORT_SIZE;

use crate::backend::{self, GattChannel, Usage};
use crate::pick::{self, Picked};
use crate::{identity, BleError, BleSighting};

pub use zapp_core::device::ids::ZSA_BLE_ORYX_SERVICE as ORYX_SERVICE;

/// The two characteristics, in ZSA's UUID space.
pub const ORYX_COMMAND: u128 = 0x32970302_e5aa_4f42_a596_1ea5a0200001;
pub const ORYX_EVENT: u128 = 0x32970303_e5aa_4f42_a596_1ea5a0200001;

/// What can go wrong opening a live session over Bluetooth.
#[derive(Debug, thiserror::Error)]
pub enum OryxError {
    #[error(transparent)]
    Ble(#[from] BleError),

    #[error("no bonded ZSA board to talk to{}", match .0 {
        Some(hint) => format!(" matching {hint:?}"),
        None => String::new(),
    })]
    NoBoard(Option<String>),

    #[error("more than one board matches: {}", .0.join(", "))]
    Ambiguous(Vec<String>),

    /// Connected, and the firmware does not serve the Oryx service.
    #[error("the keyboard's firmware has no live connection over Bluetooth")]
    NoService,
}

/// Opens the Oryx service on a bonded board and starts the handshake.
///
/// `hint` picks one out by advertised name or by the platform handle a
/// listing shows. Events are subscribed before the first command is written,
/// so the firmware-version answer cannot be missed.
pub fn open(hint: Option<&str>) -> Result<HidConnection, OryxError> {
    let mut backend = backend::for_platform(&[]).ok_or(BleError::NoAdapter)?;
    let sightings = backend.sightings()?;
    let key = select(&sightings, hint)?.key.clone();
    backend.stop_discovery();

    let events = backend
        .open_as(&key, ORYX_SERVICE, ORYX_EVENT, Usage::LISTEN)
        .map_err(missing_service)?;
    let commands = backend
        .open_as(&key, ORYX_SERVICE, ORYX_COMMAND, Usage::STREAM)
        .map_err(missing_service)?;

    Ok(HidConnection::over(GattLink { commands, events }))
}

/// A characteristic the board lacks is firmware without the service.
fn missing_service(err: BleError) -> OryxError {
    match err {
        BleError::Missing { .. } => OryxError::NoService,
        other => OryxError::Ble(other),
    }
}

/// The one board a hint names, out of everything the platform can see.
pub fn select<'a>(
    sightings: &'a [BleSighting],
    hint: Option<&str>,
) -> Result<&'a BleSighting, OryxError> {
    match pick::pick(sightings, hint, &speaks_oryx) {
        Picked::One(sighting) => Ok(sighting),
        Picked::None => Err(OryxError::NoBoard(hint.map(str::to_string))),
        Picked::Several(names) => Err(OryxError::Ambiguous(names)),
        Picked::Unbonded(name) => Err(BleError::NotBonded(name).into()),
    }
}

/// Whether a sighting could be carrying the service.
///
/// Being a ZSA board is enough to try, as for SMP: BlueZ lists no services for
/// a bonded board it has not connected to since boot. A board without the
/// service says so when the channel is opened.
fn speaks_oryx(sighting: &BleSighting) -> bool {
    sighting.services.contains(&ORYX_SERVICE) || identity::is_zsa_board(sighting)
}

/// The command and event characteristics as one report link.
struct GattLink {
    commands: Box<dyn GattChannel>,
    events: Box<dyn GattChannel>,
}

impl ReportLink for GattLink {
    fn send(&mut self, report: &[u8; REPORT_SIZE]) -> io::Result<()> {
        self.commands
            .write(trimmed(report))
            .map_err(|err| io::Error::other(err.to_string()))
    }

    fn receive(&mut self, timeout: Duration) -> io::Result<Option<Vec<u8>>> {
        self.events
            .notification(timeout)
            .map_err(|err| io::Error::other(err.to_string()))
    }
}

/// The report without its trailing zeros, at least the opcode.
///
/// The board zero-pads every write to 32 bytes, so the short form is the same
/// report and fits an ATT payload smaller than 32.
fn trimmed(report: &[u8; REPORT_SIZE]) -> &[u8] {
    let len = report
        .iter()
        .rposition(|&b| b != 0)
        .map_or(1, |last| last + 1);
    &report[..len]
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE;
    use zapp_hid_proto::{encode_packet, CMD_GET_FW_VERSION, CMD_INIT_PAIRING, CMD_SET_LAYER};

    #[test]
    fn the_oryx_uuids_are_in_zsas_space_and_clear_of_the_ranges_already_taken() {
        assert_eq!(
            identity::parse_uuid("32970301-e5aa-4f42-a596-1ea5a0200001"),
            Some(ORYX_SERVICE)
        );
        assert_eq!(
            identity::parse_uuid("32970302-e5aa-4f42-a596-1ea5a0200001"),
            Some(ORYX_COMMAND)
        );
        assert_eq!(
            identity::parse_uuid("32970303-e5aa-4f42-a596-1ea5a0200001"),
            Some(ORYX_EVENT)
        );
        assert_ne!(ORYX_SERVICE, ZSA_BLE_DFU_SERVICE);
        assert_ne!(ORYX_SERVICE, crate::smp::RELAY_SERVICE);
    }

    #[test]
    fn a_report_is_written_without_its_zero_padding() {
        assert_eq!(
            trimmed(&encode_packet(CMD_SET_LAYER, &[1, 3])),
            &[CMD_SET_LAYER, 1, 3]
        );
        // A zero key is all padding.
        assert_eq!(
            trimmed(&encode_packet(CMD_INIT_PAIRING, &[0; 6])),
            &[CMD_INIT_PAIRING]
        );
        // Opcode 0x00 is still one byte on the wire.
        assert_eq!(
            trimmed(&encode_packet(CMD_GET_FW_VERSION, &[])),
            &[CMD_GET_FW_VERSION]
        );
        let full = [0xAB; REPORT_SIZE];
        assert_eq!(trimmed(&full).len(), REPORT_SIZE);
    }

    #[test]
    fn a_missing_characteristic_is_firmware_without_the_service() {
        let err = missing_service(BleError::Missing {
            device: "dev_A".into(),
            characteristic: ORYX_EVENT,
        });
        assert!(matches!(err, OryxError::NoService), "{err:?}");
        assert_eq!(
            err.to_string(),
            "the keyboard's firmware has no live connection over Bluetooth"
        );
        // Anything else stays what the platform said.
        let err = missing_service(BleError::NotFound("dev_A went away".into()));
        assert!(matches!(err, OryxError::Ble(_)), "{err:?}");
    }

    #[test]
    fn a_bonded_zsa_board_is_a_candidate_before_its_services_resolve() {
        let cold = BleSighting {
            key: "dev_A".into(),
            name: Some("Voyager".into()),
            advertised: vec![0x32970101_e5aa_4f42_a596_1ea5a0200001],
            paired: true,
            ..BleSighting::default()
        };
        assert_eq!(select(&[cold], None).unwrap().key, "dev_A");
        assert!(matches!(select(&[], None), Err(OryxError::NoBoard(None))));
    }
}
