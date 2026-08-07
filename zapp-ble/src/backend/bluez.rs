//! BlueZ, over D-Bus.
//!
//! BlueZ keeps every bonded device in its object tree with the services it
//! resolved over the bond, so a board that is connected — and therefore silent —
//! is still there to be read. That is what makes Linux the one platform where
//! all three presence states come from a single call.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use zbus::blocking::{fdo::ObjectManagerProxy, Connection, Proxy};
use zbus::names::OwnedInterfaceName;
use zbus::zvariant::{OwnedObjectPath, OwnedValue};

use super::Backend;
use crate::{identity, BleError, BleSighting};

const BLUEZ: &str = "org.bluez";
const DEVICE: &str = "org.bluez.Device1";
const ADAPTER: &str = "org.bluez.Adapter1";

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
        Ok(objects
            .iter()
            .filter_map(|(path, ifaces)| {
                let device = ifaces
                    .iter()
                    .find(|(iface, _)| iface.as_str() == DEVICE)
                    .map(|(_, properties)| properties)?;
                Some(sighting(path.as_str(), device))
            })
            .collect())
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
