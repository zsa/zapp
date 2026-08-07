//! BlueZ, over D-Bus.
//!
//! BlueZ keeps every bonded device in its object tree with the services it
//! resolved over the bond, so a board that is connected — and therefore silent —
//! is still there to be read. That is what makes Linux the one platform where
//! all three presence states come from a single call.

use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

use zbus::blocking::{fdo::ObjectManagerProxy, Connection, Proxy};
use zbus::names::OwnedInterfaceName;
use zbus::zvariant::{OwnedObjectPath, OwnedValue};

use super::Backend;
use crate::battery::{self, Battery};
use crate::{identity, BleError, BleSighting};

const BLUEZ: &str = "org.bluez";
const DEVICE: &str = "org.bluez.Device1";
const ADAPTER: &str = "org.bluez.Adapter1";
const SERVICE: &str = "org.bluez.GattService1";
const CHARACTERISTIC: &str = "org.bluez.GattCharacteristic1";
const DESCRIPTOR: &str = "org.bluez.GattDescriptor1";

/// Battery Service, its level characteristic, and the descriptor saying which
/// half an instance speaks for.
const BATTERY_SERVICE: u128 = 0x0000180f_0000_1000_8000_00805f9b34fb;
const BATTERY_LEVEL: u128 = 0x00002a19_0000_1000_8000_00805f9b34fb;
const PRESENTATION_FORMAT: u128 = 0x00002904_0000_1000_8000_00805f9b34fb;

/// How long each discovery window stays open, and how often one opens.
///
/// Discovery is only needed to find a board nobody has bonded yet: a bonded one
/// is in the object tree whether or not anything is scanning. Leaving the radio
/// scanning costs power and airtime, and contends with the very links it is
/// scanning past, so it runs in short windows.
const SCAN_WINDOW: Duration = Duration::from_secs(4);
const SCAN_EVERY: Duration = Duration::from_secs(30);

type Properties = HashMap<String, OwnedValue>;
type Objects = HashMap<OwnedObjectPath, HashMap<OwnedInterfaceName, Properties>>;

pub struct Bluez {
    connection: Connection,
    objects: ObjectManagerProxy<'static>,
    scanning_since: Option<Instant>,
    scanned_at: Option<Instant>,
    /// Which half each level characteristic speaks for, by object path. The
    /// descriptor saying so never changes, so it is read once.
    halves: HashMap<String, battery::Half>,
    /// Level characteristics already subscribed to. BlueZ then keeps their
    /// `Value` fresh, and later snapshots cost nothing.
    notifying: HashSet<String>,
}

impl Bluez {
    pub fn connect() -> Result<Self, BleError> {
        let connection = Connection::system().map_err(platform)?;
        let objects = ObjectManagerProxy::new(&connection, BLUEZ, "/").map_err(platform)?;
        Ok(Self {
            connection,
            objects,
            scanning_since: None,
            scanned_at: None,
            halves: HashMap::new(),
            notifying: HashSet::new(),
        })
    }

    /// Opens and closes the discovery window.
    ///
    /// Failures are logged rather than returned: an adapter that is off or busy
    /// stops nothing, since the bonded devices are readable either way.
    fn pump_discovery(&mut self, objects: &Objects) {
        match self.scanning_since {
            Some(started) if started.elapsed() >= SCAN_WINDOW => {
                self.for_each_adapter(objects, "StopDiscovery");
                self.scanning_since = None;
                self.scanned_at = Some(Instant::now());
            }
            Some(_) => {}
            None => {
                let due = self
                    .scanned_at
                    .is_none_or(|ended| ended.elapsed() >= SCAN_EVERY);
                if due {
                    self.for_each_adapter(objects, "StartDiscovery");
                    self.scanning_since = Some(Instant::now());
                }
            }
        }
    }

    fn for_each_adapter(&self, objects: &Objects, method: &str) {
        for path in objects
            .iter()
            .filter(|(_, ifaces)| ifaces.keys().any(|iface| iface.as_str() == ADAPTER))
            .map(|(path, _)| path)
        {
            let proxy = Proxy::new(&self.connection, BLUEZ, path.as_str(), ADAPTER);
            let call = proxy.and_then(|proxy| proxy.call::<_, _, ()>(method, &()));
            if let Err(err) = call {
                log::debug!("{method} on {path} failed: {err}");
            }
        }
    }
}

impl Backend for Bluez {
    fn sightings(&mut self) -> Result<Vec<BleSighting>, BleError> {
        let objects = self.objects.get_managed_objects().map_err(platform)?;
        self.pump_discovery(&objects);
        let devices: Vec<(String, BleSighting)> = objects
            .iter()
            .filter_map(|(path, ifaces)| {
                let properties = ifaces
                    .iter()
                    .find(|(iface, _)| iface.as_str() == DEVICE)
                    .map(|(_, properties)| properties)?;
                Some((path.to_string(), sighting(path.as_str(), properties)))
            })
            .collect();

        // A subscription dies with the connection that carried it, and BlueZ
        // hands the same object paths back on reconnect, so anything under a
        // board that is no longer connected is forgotten and asked for again.
        let live: Vec<&str> = devices
            .iter()
            .filter(|(_, sighting)| sighting.connected)
            .map(|(path, _)| path.as_str())
            .collect();
        self.notifying
            .retain(|path| live.iter().any(|device| path.starts_with(device)));

        Ok(devices
            .iter()
            .map(|(path, sighting)| {
                let mut sighting = sighting.clone();
                // Only ours, and only while connected: reading a battery means
                // talking to the board.
                if sighting.connected && identity::is_zsa_board(&sighting) {
                    sighting.battery = self.battery(&objects, path);
                }
                sighting
            })
            .collect())
    }
}

impl Bluez {
    /// What a board's Battery Service instances say.
    ///
    /// A split board publishes one per half plus a combined one, so the levels
    /// are read from the GATT tree rather than `org.bluez.Battery1`, which
    /// surfaces only the first.
    fn battery(&mut self, objects: &Objects, device: &str) -> Battery {
        let mut battery = Battery::default();

        let services: Vec<&str> = objects
            .iter()
            .filter(|(path, _)| path.as_str().starts_with(device))
            .filter(|(_, ifaces)| {
                properties(ifaces, SERVICE)
                    .and_then(|service| get::<String>(service, "UUID"))
                    .and_then(|uuid| identity::parse_uuid(&uuid))
                    == Some(BATTERY_SERVICE)
            })
            .map(|(path, _)| path.as_str())
            .collect();

        for service in services {
            let Some((path, level)) = objects
                .iter()
                .filter(|(path, _)| path.as_str().starts_with(service))
                .find_map(|(path, ifaces)| {
                    let characteristic = properties(ifaces, CHARACTERISTIC)?;
                    let uuid = identity::parse_uuid(&get::<String>(characteristic, "UUID")?)?;
                    (uuid == BATTERY_LEVEL).then(|| (path.as_str(), characteristic))
                })
            else {
                continue;
            };

            let Some(half) = self.half_of(objects, path) else {
                continue;
            };
            // The cached value, which notifications keep fresh; a read is only
            // needed the first time, before the subscription exists.
            let level = get::<Vec<u8>>(level, "Value")
                .filter(|value| !value.is_empty())
                .or_else(|| read(&self.connection, path, CHARACTERISTIC));
            if let Some(level) = level.and_then(|value| value.first().copied()) {
                battery.set(half, level);
            }
            self.subscribe(path);
        }

        battery
    }

    /// Which half a level characteristic speaks for, from the presentation
    /// format descriptor beside it. Read once and remembered.
    fn half_of(&mut self, objects: &Objects, characteristic: &str) -> Option<battery::Half> {
        if let Some(half) = self.halves.get(characteristic) {
            return Some(*half);
        }

        let descriptor = objects
            .iter()
            .filter(|(path, _)| path.as_str().starts_with(characteristic))
            .find_map(|(path, ifaces)| {
                let descriptor = properties(ifaces, DESCRIPTOR)?;
                let uuid = identity::parse_uuid(&get::<String>(descriptor, "UUID")?)?;
                (uuid == PRESENTATION_FORMAT).then(|| path.as_str())
            })?;

        let value = read(&self.connection, descriptor, DESCRIPTOR)?;
        let half = battery::half_from_format(&value)?;
        self.halves.insert(characteristic.to_string(), half);
        Some(half)
    }

    /// Asks BlueZ to keep this characteristic's value fresh. Once each: a
    /// second call is an error, and the subscription outlives the poll.
    fn subscribe(&mut self, characteristic: &str) {
        if !self.notifying.insert(characteristic.to_string()) {
            return;
        }
        let proxy = Proxy::new(&self.connection, BLUEZ, characteristic, CHARACTERISTIC);
        let call = proxy.and_then(|proxy| proxy.call::<_, _, ()>("StartNotify", &()));
        if let Err(err) = call {
            log::debug!("StartNotify on {characteristic} failed: {err}");
            self.notifying.remove(characteristic);
        }
    }
}

/// One interface's properties, if an object carries it.
fn properties<'a>(
    ifaces: &'a HashMap<OwnedInterfaceName, Properties>,
    wanted: &str,
) -> Option<&'a Properties> {
    ifaces
        .iter()
        .find(|(iface, _)| iface.as_str() == wanted)
        .map(|(_, properties)| properties)
}

/// Reads a characteristic or descriptor, which means talking to the board.
fn read(connection: &Connection, path: &str, iface: &str) -> Option<Vec<u8>> {
    let proxy = Proxy::new(connection, BLUEZ, path, iface).ok()?;
    let options: HashMap<String, OwnedValue> = HashMap::new();
    match proxy.call::<_, _, Vec<u8>>("ReadValue", &(options,)) {
        Ok(value) => Some(value),
        Err(err) => {
            log::debug!("ReadValue on {path} failed: {err}");
            None
        }
    }
}

/// One `org.bluez.Device1` as a sighting.
fn sighting(path: &str, device: &Properties) -> BleSighting {
    // BlueZ merges the UUIDs a device advertises with the ones it resolved over
    // a connection into this single property, so both of ours come from it.
    let uuids: Vec<u128> = get::<Vec<String>>(device, "UUIDs")
        .unwrap_or_default()
        .iter()
        .filter_map(|uuid| identity::parse_uuid(uuid))
        .collect();

    BleSighting {
        key: path.to_string(),
        name: get::<String>(device, "Alias").or_else(|| get::<String>(device, "Name")),
        advertised: uuids.clone(),
        services: uuids,
        modalias: get::<String>(device, "Modalias"),
        connected: get::<bool>(device, "Connected").unwrap_or(false),
        paired: get::<bool>(device, "Paired").unwrap_or(false),
        // Present only while the device has been heard from recently, which is
        // what makes it the "in range" signal rather than a signal strength.
        rssi: get::<i16>(device, "RSSI"),
        // Filled in afterwards, and only for a board worth reading.
        battery: Battery::default(),
    }
}

fn get<T>(properties: &Properties, key: &str) -> Option<T>
where
    T: TryFrom<OwnedValue>,
{
    T::try_from(properties.get(key)?.try_clone().ok()?).ok()
}

fn platform(err: impl std::fmt::Display) -> BleError {
    BleError::Platform(err.to_string())
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    /// The rule that decides which subscriptions survive a snapshot, lifted out
    /// of the D-Bus call so it can be exercised without a bus.
    fn keep(notifying: &mut HashSet<String>, connected: &[&str]) {
        notifying.retain(|path| connected.iter().any(|device| path.starts_with(device)));
    }

    #[test]
    fn a_subscription_is_forgotten_when_its_board_disconnects() {
        let device = "/org/bluez/hci0/dev_E8_6B_FC_47_E2_64";
        let level = format!("{device}/service0016/char0017");
        let mut notifying = HashSet::from([level.clone()]);

        keep(&mut notifying, &[device]);
        assert!(
            notifying.contains(&level),
            "still connected, still subscribed"
        );

        // BlueZ tore the notify session down with the connection, and hands the
        // same path back on reconnect: without forgetting, it is never renewed.
        keep(&mut notifying, &[]);
        assert!(notifying.is_empty());
    }

    #[test]
    fn one_board_disconnecting_leaves_anothers_subscription_alone() {
        let ours = "/org/bluez/hci0/dev_E8_6B_FC_47_E2_64";
        let other = "/org/bluez/hci0/dev_F8_DE_F9_BF_5F_5F";
        let mut notifying = HashSet::from([
            format!("{ours}/service0016/char0017"),
            format!("{other}/service0016/char0017"),
        ]);

        keep(&mut notifying, &[ours]);
        assert_eq!(notifying.len(), 1);
        assert!(notifying.iter().all(|path| path.starts_with(ours)));
    }
}
