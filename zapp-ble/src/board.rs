//! What a backend reports, and what the monitor publishes.

use zapp_core::device::ids::Keyboard;

use crate::{identity, Battery};

/// How reachable a board is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Presence {
    /// Connected to this machine, by us or by anything else.
    Connected,
    /// Advertising in range, not connected here.
    Nearby,
    /// Bonded to this machine, not reachable right now.
    Paired,
}

/// A ZSA board the platform can see.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BleBoard {
    /// Platform-stable handle: a BlueZ object path, a CoreBluetooth peripheral
    /// identifier, a WinRT device id. Not stable across a re-bond.
    pub key: String,
    pub name: Option<String>,
    /// The Bluetooth address of the identity this machine is bonded to, when
    /// the platform handle carries one. `None` off Linux, and for any handle
    /// that is not a BlueZ device path.
    pub address: Option<String>,
    /// `None` when nothing the platform offered names the product.
    pub keyboard: Option<Keyboard>,
    pub presence: Presence,
    pub battery: Battery,
}

/// A change in what the platform can see.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BleEvent {
    Found(BleBoard),
    Changed(BleBoard),
    /// Gone from the platform entirely — unbonded, or removed. A board that is
    /// merely out of range becomes `Presence::Paired` instead.
    Lost {
        key: String,
    },
}

/// One device as a backend saw it, before identification.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BleSighting {
    pub key: String,
    pub name: Option<String>,
    /// GATT services, resolved or from the platform's cache.
    pub services: Vec<u128>,
    /// Service UUIDs carried in the advertising payload.
    pub advertised: Vec<u128>,
    /// BlueZ's `Device1.Modalias`, `usb:vVVVVpPPPPdDDDD`. Linux only.
    pub modalias: Option<String>,
    pub connected: bool,
    pub paired: bool,
    pub rssi: Option<i16>,
    pub battery: Battery,
}

impl BleSighting {
    /// How reachable this is, or `None` for a device that is none of the three
    /// — a stray seen in an earlier scan that is neither bonded nor in range.
    pub fn presence(&self) -> Option<Presence> {
        if self.connected {
            Some(Presence::Connected)
        } else if self.rssi.is_some() {
            Some(Presence::Nearby)
        } else if self.paired {
            Some(Presence::Paired)
        } else {
            None
        }
    }
}

impl BleBoard {
    /// The board a sighting describes, or `None` if it is not a reachable ZSA
    /// board.
    pub fn from_sighting(sighting: &BleSighting) -> Option<Self> {
        if !identity::is_zsa_board(sighting) {
            return None;
        }
        Some(Self {
            key: sighting.key.clone(),
            name: sighting.name.clone(),
            address: address_from_key(&sighting.key),
            keyboard: identity::keyboard_for(sighting),
            presence: sighting.presence()?,
            battery: sighting.battery,
        })
    }
}

/// The Bluetooth address a platform handle spells out, or `None`.
///
/// BlueZ names a device by the address it is bonded to — `dev_FE_DB_E9_A8_B1_36`
/// at the end of the object path — and Windows by the adapter's address and the
/// device's — `BluetoothLE#BluetoothLEb4:8c:9d:35:f7:86-fe:db:e9:a8:b1:36` — so
/// the handle already carries the identity and no round trip to the board is
/// needed for it. CoreBluetooth's handles carry no address to read.
fn address_from_key(key: &str) -> Option<String> {
    let tail = match key.strip_prefix("BluetoothLE#") {
        Some(windows) => windows.rsplit('-').next()?,
        None => key.rsplit('/').next()?.strip_prefix("dev_")?,
    };

    let octets: Vec<String> = tail
        .split(['_', ':'])
        .map(|o| o.to_ascii_uppercase())
        .collect();
    let well_formed = octets.len() == 6
        && octets
            .iter()
            .all(|o| o.len() == 2 && o.bytes().all(|b| b.is_ascii_hexdigit()));

    well_formed.then(|| octets.join(":"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE;

    fn bonded() -> BleSighting {
        BleSighting {
            key: "dev_E8_6B_FC_47_E2_64".into(),
            name: Some("Voyager".into()),
            services: vec![ZSA_BLE_DFU_SERVICE],
            modalias: Some("usb:v3297p1978d0100".into()),
            paired: true,
            ..BleSighting::default()
        }
    }

    #[test]
    fn a_connected_board_is_present_without_an_rssi() {
        // It stopped advertising when it connected, so there is no rssi to have.
        let sighting = BleSighting {
            connected: true,
            ..bonded()
        };
        assert_eq!(sighting.presence(), Some(Presence::Connected));
    }

    #[test]
    fn a_bonded_board_out_of_range_is_paired() {
        assert_eq!(bonded().presence(), Some(Presence::Paired));
    }

    #[test]
    fn a_stray_that_is_neither_bonded_nor_in_range_is_not_present() {
        let sighting = BleSighting {
            paired: false,
            ..bonded()
        };
        assert_eq!(sighting.presence(), None);
    }

    #[test]
    fn a_board_carries_its_product_through_from_the_sighting() {
        let board = BleBoard::from_sighting(&bonded()).expect("a bonded ZSA board");
        assert_eq!(board.keyboard, Some(Keyboard::WirelessVoyager));
        assert_eq!(board.presence, Presence::Paired);
        assert_eq!(board.name.as_deref(), Some("Voyager"));
    }

    #[test]
    fn a_board_carries_the_address_out_of_its_object_path() {
        let sighting = BleSighting {
            key: "/org/bluez/hci0/dev_FE_DB_E9_A8_B1_36".into(),
            ..bonded()
        };
        let board = BleBoard::from_sighting(&sighting).expect("a bonded ZSA board");
        assert_eq!(board.address.as_deref(), Some("FE:DB:E9:A8:B1:36"));
    }

    #[test]
    fn a_bluez_address_is_read_off_the_path_tail() {
        assert_eq!(
            address_from_key("/org/bluez/hci0/dev_FE_DB_E9_A8_B1_36").as_deref(),
            Some("FE:DB:E9:A8:B1:36")
        );
        // Lower case is BlueZ's business; the address we print is not.
        assert_eq!(
            address_from_key("dev_fe_db_e9_a8_b1_36").as_deref(),
            Some("FE:DB:E9:A8:B1:36")
        );
    }

    #[test]
    fn a_windows_handle_carries_the_device_address_after_the_adapters() {
        assert_eq!(
            address_from_key("BluetoothLE#BluetoothLEb4:8c:9d:35:f7:86-fe:db:e9:a8:b1:36")
                .as_deref(),
            Some("FE:DB:E9:A8:B1:36")
        );
        // The adapter's own address is not the board's.
        assert_eq!(
            address_from_key("BluetoothLE#BluetoothLEb4:8c:9d:35:f7:86"),
            None
        );
    }

    #[test]
    fn a_handle_that_is_not_a_bluez_device_path_has_no_address() {
        // Another platform's handle, a BlueZ adapter rather than a device, and
        // device paths whose tail is the wrong length or not hex.
        for key in [
            "8D2F1C4A-0B33-4E7A-9F21-6C5D0E3B7A18",
            "/org/bluez/hci0",
            "/org/bluez/hci0/dev_FE_DB_E9_A8_B1",
            "/org/bluez/hci0/dev_FE_DB_E9_A8_B1_36_7C",
            "/org/bluez/hci0/dev_FE_DB_E9_A8_B1_3G",
            "/org/bluez/hci0/dev_FEDBE9A8B136",
        ] {
            assert_eq!(address_from_key(key), None, "{key}");
        }
    }

    #[test]
    fn a_board_that_is_not_present_yields_nothing() {
        let sighting = BleSighting {
            paired: false,
            ..bonded()
        };
        assert_eq!(BleBoard::from_sighting(&sighting), None);
    }
}
