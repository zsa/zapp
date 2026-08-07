//! Whether a sighting is a ZSA board, and which one.
//!
//! Every rule here is pure, so the awkward cases are testable without a radio.

use zapp_core::device::ids::{self as ids, Keyboard};

use crate::BleSighting;

/// Whether this is a ZSA board at all.
///
/// The firmware-update service is the discriminator for a board the platform
/// already knows: a generic BLE keyboard exposes only HID and battery. A board
/// seen advertising has no service list to check, so its product identity UUID
/// answers instead.
pub fn is_zsa_board(sighting: &BleSighting) -> bool {
    sighting.services.contains(&ids::ZSA_BLE_DFU_SERVICE) || advertised_keyboard(sighting).is_some()
}

/// Which product this is, when anything the platform offered says so.
///
/// Two routes, neither needing a connection: the advertised identity UUID, and
/// the modalias, which carries the same USB product id the board enumerates
/// with over the wire. Reading the DIS model number would settle it for a
/// bonded board too, but that needs a GATT read and so belongs to a caller
/// holding a connection.
pub fn keyboard_for(sighting: &BleSighting) -> Option<Keyboard> {
    advertised_keyboard(sighting).or_else(|| {
        sighting
            .modalias
            .as_deref()
            .and_then(keyboard_from_modalias)
    })
}

fn advertised_keyboard(sighting: &BleSighting) -> Option<Keyboard> {
    sighting
        .advertised
        .iter()
        .copied()
        .find_map(ids::keyboard_for_ble_identity)
}

/// The keyboard a `usb:vVVVVpPPPPdDDDD` modalias names.
///
/// BlueZ publishes this for a bonded device without connecting: it read the
/// DIS PnP id during service discovery and kept it.
pub fn keyboard_from_modalias(modalias: &str) -> Option<Keyboard> {
    let tail = modalias.strip_prefix("usb:v")?;
    let (vid, tail) = tail.split_once('p')?;
    let pid = tail.split_once('d').map_or(tail, |(pid, _)| pid);
    ids::identify_keyboard(
        u16::from_str_radix(vid, 16).ok()?,
        u16::from_str_radix(pid, 16).ok()?,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE;

    const VOYAGER2_IDENTITY: u128 = 0x32970101_e5aa_4f42_a596_1ea5a0200001;
    const HID: u128 = 0x00001812_0000_1000_8000_00805f9b34fb;
    const BATTERY: u128 = 0x0000180f_0000_1000_8000_00805f9b34fb;

    #[test]
    fn a_bonded_board_is_ours_by_its_update_service() {
        let sighting = BleSighting {
            services: vec![HID, BATTERY, ZSA_BLE_DFU_SERVICE],
            ..BleSighting::default()
        };
        assert!(is_zsa_board(&sighting));
    }

    #[test]
    fn another_bluetooth_keyboard_is_not_ours() {
        // What the DIERYA-M1 on the bench exposes: the generic services only.
        let sighting = BleSighting {
            services: vec![HID, BATTERY],
            ..BleSighting::default()
        };
        assert!(!is_zsa_board(&sighting));
        assert_eq!(keyboard_for(&sighting), None);
    }

    #[test]
    fn an_advertising_board_is_ours_by_its_identity_uuid() {
        // Nothing has connected, so there is no service list at all.
        let sighting = BleSighting {
            advertised: vec![HID, BATTERY, VOYAGER2_IDENTITY],
            ..BleSighting::default()
        };
        assert!(is_zsa_board(&sighting));
        assert_eq!(keyboard_for(&sighting), Some(Keyboard::Voyager2));
    }

    #[test]
    fn a_bonded_board_is_named_by_its_modalias() {
        let sighting = BleSighting {
            services: vec![ZSA_BLE_DFU_SERVICE],
            modalias: Some("usb:v3297p1978d0100".into()),
            ..BleSighting::default()
        };
        assert_eq!(keyboard_for(&sighting), Some(Keyboard::Voyager2));
    }

    #[test]
    fn a_board_the_platform_cannot_name_is_still_ours() {
        // macOS and Windows offer no modalias, and a bonded board advertises
        // nothing: it is a ZSA board of unknown model until something reads DIS.
        let sighting = BleSighting {
            services: vec![ZSA_BLE_DFU_SERVICE],
            ..BleSighting::default()
        };
        assert!(is_zsa_board(&sighting));
        assert_eq!(keyboard_for(&sighting), None);
    }

    #[test]
    fn the_advertised_identity_wins_over_the_modalias() {
        let sighting = BleSighting {
            advertised: vec![VOYAGER2_IDENTITY],
            modalias: Some("usb:v3297p1977d0100".into()),
            ..BleSighting::default()
        };
        assert_eq!(keyboard_for(&sighting), Some(Keyboard::Voyager2));
    }

    #[test]
    fn modalias_forms_that_name_nothing() {
        assert_eq!(
            keyboard_from_modalias("usb:v3297p1978d0100"),
            Some(Keyboard::Voyager2)
        );
        // The version suffix is optional.
        assert_eq!(
            keyboard_from_modalias("usb:v3297p1977"),
            Some(Keyboard::Voyager)
        );
        // Nordic's ids, which is what the board reported before feat/zsa-uuid.
        assert_eq!(keyboard_from_modalias("usb:v1915pEEEFd0100"), None);
        assert_eq!(keyboard_from_modalias("bluetooth:v3297p1978"), None);
        assert_eq!(keyboard_from_modalias("usb:v3297"), None);
        assert_eq!(keyboard_from_modalias("usb:vZZZZpYYYY"), None);
        assert_eq!(keyboard_from_modalias(""), None);
    }
}
