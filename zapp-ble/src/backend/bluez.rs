//! BlueZ, over D-Bus.
//!
//! BlueZ keeps every bonded device in its object tree with the services it
//! resolved over the bond, so a board that is connected — and therefore silent —
//! is still there to be read. That is what makes Linux the one platform where
//! all three presence states come from a single call.

use std::collections::{HashMap, HashSet};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use zbus::blocking::{fdo::ObjectManagerProxy, Connection, MessageIterator, Proxy};
use zbus::names::OwnedInterfaceName;
use zbus::zvariant::{OwnedObjectPath, OwnedValue};
use zbus::MatchRule;

use super::{bluez_gatt, Backend, GattChannel, Usage};
use crate::battery::{
    self, Battery, ChargeDetail, BATTERY_LEVEL, BATTERY_SERVICE, CHARGE_DETAIL, PRESENTATION_FORMAT,
};
use crate::{identity, BleError, BleSighting};

pub(super) const BLUEZ: &str = "org.bluez";
pub(super) const DEVICE: &str = "org.bluez.Device1";
const ADAPTER: &str = "org.bluez.Adapter1";
pub(super) const SERVICE: &str = "org.bluez.GattService1";
pub(super) const CHARACTERISTIC: &str = "org.bluez.GattCharacteristic1";
const DESCRIPTOR: &str = "org.bluez.GattDescriptor1";

/// How long each discovery window stays open, and how often one opens.
///
/// Discovery is only needed to find a board nobody has bonded yet: a bonded one
/// is in the object tree whether or not anything is scanning. Leaving the radio
/// scanning costs power and airtime, and contends with the very links it is
/// scanning past, so it runs in short windows.
const SCAN_WINDOW: Duration = Duration::from_secs(4);
const SCAN_EVERY: Duration = Duration::from_secs(30);

/// How long to let a burst of signals finish before reading the tree.
///
/// One change rarely arrives alone — a connection brings a `Connected`
/// property, a pile of `InterfacesAdded` for the GATT tree, then the first
/// notifications — and reading after each would mean reading a dozen times for
/// one event.
const SETTLE: Duration = Duration::from_millis(250);

pub(super) type Properties = HashMap<String, OwnedValue>;
pub(super) type Objects = HashMap<OwnedObjectPath, HashMap<OwnedInterfaceName, Properties>>;

pub struct Bluez {
    connection: Connection,
    objects: ObjectManagerProxy<'static>,
    scanning_since: Option<Instant>,
    scanned_at: Option<Instant>,
    /// Which half each level characteristic speaks for, by object path. The
    /// descriptor saying so never changes, so it is read once — unlike a
    /// subscription, which dies with the connection that carried it.
    halves: HashMap<String, battery::Half>,
    /// Level characteristics whose current notify session we are in step with:
    /// read once since it came up, and fed by BlueZ ever since. Their cached
    /// `Value` is the board's value, so later snapshots cost nothing.
    synced: HashSet<String>,
    /// Ticks when BlueZ says something changed.
    changes: Receiver<()>,
}

impl Bluez {
    pub fn connect() -> Result<Self, BleError> {
        let connection = Connection::system().map_err(platform)?;
        let objects = ObjectManagerProxy::new(&connection, BLUEZ, "/").map_err(platform)?;
        let changes = watch_signals(&connection)?;
        Ok(Self {
            connection,
            objects,
            scanning_since: None,
            scanned_at: None,
            halves: HashMap::new(),
            synced: HashSet::new(),
            changes,
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
    /// Waits for BlueZ to say something changed, rather than for the clock.
    ///
    /// Everything this backend reports — presence, connection state, battery —
    /// reaches BlueZ as a signal, so the timeout is only the safety net for one
    /// that was missed.
    fn wait_for_change(&mut self, timeout: Duration) {
        match self.changes.recv_timeout(timeout) {
            Ok(()) => log::trace!("woken by bluez"),
            // The watcher thread is gone, so nothing will ever tick again.
            // Falling back to the clock keeps the monitor working.
            Err(RecvTimeoutError::Disconnected) => thread::sleep(timeout),
            Err(RecvTimeoutError::Timeout) => {
                log::trace!("nothing from bluez in {timeout:?}");
                return;
            }
        }

        thread::sleep(SETTLE);
        // Everything that arrived during the burst is answered by the one
        // snapshot about to be taken.
        while self.changes.try_recv().is_ok() {}
    }

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

        // Housekeeping: a board that is no longer connected has no GATT tree to
        // hold paths for, so its entries are dropped rather than kept for a
        // reconnect that may never come. Renewing a subscription does not
        // depend on this — `plan` reads that off BlueZ.
        let live: Vec<&str> = devices
            .iter()
            .filter(|(_, sighting)| sighting.connected)
            .map(|(path, _)| path.as_str())
            .collect();
        keep(&mut self.synced, &live);

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

    /// Shuts the discovery window, and keeps it shut until the next snapshot
    /// decides otherwise.
    fn stop_discovery(&mut self) {
        if self.scanning_since.is_none() {
            return;
        }
        let objects = match self.objects.get_managed_objects() {
            Ok(objects) => objects,
            Err(err) => {
                log::debug!("cannot read the object tree to stop discovery: {err}");
                return;
            }
        };
        self.for_each_adapter(&objects, "StopDiscovery");
        self.scanning_since = None;
        self.scanned_at = Some(Instant::now());
    }

    fn open_as(
        &mut self,
        key: &str,
        service: u128,
        characteristic: u128,
        usage: Usage,
    ) -> Result<Box<dyn GattChannel>, BleError> {
        bluez_gatt::open(
            &self.connection,
            &self.objects,
            key,
            service,
            characteristic,
            usage,
        )
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

            // The instance is there, so the half is there. Whatever comes of
            // reading it, this is a half whose level is unknown rather than a
            // half the board does not have.
            battery.set(half, battery::Level::Unknown);

            let plan = plan(get::<bool>(level, "Notifying"), self.synced.contains(path));
            // Armed before read, so a change in between arrives as a
            // notification instead of falling in the gap.
            let listening = !plan.start_notify || self.start_notify(path);
            let value = match plan.read {
                true => read(&self.connection, path, CHARACTERISTIC),
                // In step with a live session: what BlueZ holds is the last
                // thing the board sent, which is what the board holds.
                false => get::<Vec<u8>>(level, "Value"),
            }
            .filter(|value| !value.is_empty());
            if let Some(raw) = value.as_deref().and_then(|value| value.first()) {
                battery.set(half, battery::Level::from_raw(*raw));
            }
            log::trace!(
                "{half:?} level {value:?} from {}",
                if plan.read { "the board" } else { "bluez" }
            );

            // In step only once there is a session and a value read under it.
            // A StartNotify or a read that failed leaves it out, so the next
            // snapshot tries again instead of settling for a stale value.
            match listening && (!plan.read || value.is_some()) {
                true => self.synced.insert(path.to_string()),
                false => self.synced.remove(path),
            };
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

    /// Asks BlueZ to keep this characteristic's value fresh, and says whether
    /// it will.
    fn start_notify(&self, characteristic: &str) -> bool {
        let proxy = Proxy::new(&self.connection, BLUEZ, characteristic, CHARACTERISTIC);
        match proxy.and_then(|proxy| proxy.call::<_, _, ()>("StartNotify", &())) {
            Ok(()) => true,
            Err(err) => {
                log::debug!("StartNotify on {characteristic} failed: {err}");
                false
            }
        }
    }
}

/// What one board's charge-detail characteristic says, read now.
///
/// A one-shot on a connection of its own rather than something the monitor
/// keeps fresh: nothing asks for it until a user does, and the answer costs a
/// round trip to the board. `key` is the object path a sighting carries.
///
/// `None` covers every way this comes to nothing — no bus, a board that is not
/// connected, firmware without the characteristic, a read that failed — because
/// none of them is worth a line under a board in a listing. A value that
/// arrives short is not nothing: what it did not reach is left unknown.
pub(super) fn read_charge_detail(key: &str) -> Option<ChargeDetail> {
    let connection = Connection::system().ok()?;
    let objects = ObjectManagerProxy::new(&connection, BLUEZ, "/").ok()?;
    let objects = objects.get_managed_objects().ok()?;

    let path = objects
        .iter()
        .filter(|(path, _)| path.as_str().starts_with(key))
        .find_map(|(path, ifaces)| {
            let characteristic = properties(ifaces, CHARACTERISTIC)?;
            let uuid = identity::parse_uuid(&get::<String>(characteristic, "UUID")?)?;
            (uuid == CHARGE_DETAIL).then(|| path.to_string())
        });

    let Some(path) = path else {
        log::debug!("{key} publishes no charge detail characteristic");
        return None;
    };
    ChargeDetail::decode(&read(&connection, &path, CHARACTERISTIC)?)
}

/// What one battery level characteristic needs before its value can be
/// believed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Plan {
    /// Ask BlueZ for a notify session.
    start_notify: bool,
    /// Go to the board, rather than believing what BlueZ has cached.
    read: bool,
}

/// The rule, from BlueZ's `Notifying` property and whether this characteristic
/// has been read since the session behind it came up.
///
/// Asking BlueZ what it is subscribed to, rather than remembering what we asked
/// it to subscribe to, is what survives a reconnect: a session dies with its
/// connection whether or not anyone was watching, and `Notifying` back at false
/// is BlueZ saying so. Missing the disconnect costs nothing, because the next
/// snapshot arms it again.
fn plan(notifying: Option<bool>, synced: bool) -> Plan {
    match (notifying == Some(true), synced) {
        // Live, and in step with it. Every change since the read arrived as a
        // notification, so the cached value is the board's.
        (true, true) => Plan {
            start_notify: false,
            read: false,
        },
        // Live, but nothing was listening when it came up — it may be someone
        // else's session, or ours from a moment ago. One read catches up.
        (true, false) => Plan {
            start_notify: false,
            read: true,
        },
        // No session, so the cached value has been frozen since the last one
        // died. Arm one, then read what was missed while it was down.
        (false, _) => Plan {
            start_notify: true,
            read: true,
        },
    }
}

/// Which characteristics stay tracked across a snapshot: only those under a
/// board that is still connected, since the rest have no GATT tree left.
fn keep(tracked: &mut HashSet<String>, connected: &[&str]) {
    tracked.retain(|path| connected.iter().any(|device| path.starts_with(device)));
}

/// Ticks once per signal from BlueZ.
///
/// Anything it emits is worth a look: `InterfacesAdded` and `InterfacesRemoved`
/// for devices and their GATT trees, `PropertiesChanged` for connection state,
/// rssi and the battery values notifications refresh. Which one it was does not
/// matter, since the answer is always to read the tree again.
fn watch_signals(connection: &Connection) -> Result<Receiver<()>, BleError> {
    let rule = MatchRule::builder()
        .msg_type(zbus::message::Type::Signal)
        .sender(BLUEZ)
        .map_err(platform)?
        .build();
    let signals = MessageIterator::for_match_rule(rule, connection, None).map_err(platform)?;

    let (tx, rx) = mpsc::channel();
    thread::Builder::new()
        .name("zapp-ble-dbus".into())
        .spawn(move || {
            for message in signals {
                if message.is_err() {
                    continue;
                }
                if tx.send(()).is_err() {
                    break;
                }
            }
        })
        .map_err(platform)?;

    Ok(rx)
}

/// One interface's properties, if an object carries it.
pub(super) fn properties<'a>(
    ifaces: &'a HashMap<OwnedInterfaceName, Properties>,
    wanted: &str,
) -> Option<&'a Properties> {
    ifaces
        .iter()
        .find(|(iface, _)| iface.as_str() == wanted)
        .map(|(_, properties)| properties)
}

/// Reads a characteristic or descriptor, which means talking to the board.
pub(super) fn read(connection: &Connection, path: &str, iface: &str) -> Option<Vec<u8>> {
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

pub(super) fn get<T>(properties: &Properties, key: &str) -> Option<T>
where
    T: TryFrom<OwnedValue>,
{
    T::try_from(properties.get(key)?.try_clone().ok()?).ok()
}

pub(super) fn platform(err: impl std::fmt::Display) -> BleError {
    BleError::Platform(err.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One poll, as the backend runs it: plan, then record what came of it.
    /// `notifying` is what BlueZ said, `armed` and `got` whether the
    /// StartNotify and the read that followed worked.
    fn poll(synced: &mut HashSet<String>, notifying: Option<bool>, armed: bool, got: bool) -> Plan {
        let path = "char0017";
        let plan = plan(notifying, synced.contains(path));
        let listening = !plan.start_notify || armed;
        match listening && (!plan.read || got) {
            true => synced.insert(path.to_string()),
            false => synced.remove(path),
        };
        plan
    }

    /// The case that was wrong on hardware: one snapshot, no subscription yet,
    /// and BlueZ holding a value from who knows when.
    #[test]
    fn the_first_snapshot_goes_to_the_board_rather_than_to_the_cache() {
        let mut synced = HashSet::new();
        assert_eq!(
            poll(&mut synced, Some(false), true, true),
            Plan {
                start_notify: true,
                read: true
            }
        );
    }

    #[test]
    fn a_session_we_are_in_step_with_costs_nothing() {
        let mut synced = HashSet::new();
        poll(&mut synced, Some(false), true, true);
        // Armed and read last time, and BlueZ has been feeding it since.
        assert_eq!(
            poll(&mut synced, Some(true), true, true),
            Plan {
                start_notify: false,
                read: false
            }
        );
        assert_eq!(
            poll(&mut synced, Some(true), true, true),
            Plan {
                start_notify: false,
                read: false
            },
            "and stays free for as long as the session lasts"
        );
    }

    /// The reconnect. BlueZ tears the session down with the connection and
    /// hands the same object path back, so remembering that we once subscribed
    /// is what froze the value for the life of the process.
    #[test]
    fn a_reconnect_arms_the_subscription_again() {
        let mut synced = HashSet::new();
        poll(&mut synced, Some(false), true, true);
        poll(&mut synced, Some(true), true, true);

        // Down, and back up on the same path: BlueZ says Notifying is false.
        let plan = poll(&mut synced, Some(false), true, true);
        assert!(plan.start_notify, "the session has to be renewed");
        assert!(plan.read, "and what changed while it was down, read");
    }

    /// The disconnect may fall inside a settle window, between two snapshots —
    /// then nothing ever sees `connected` go false, and a backend tracking the
    /// transition would never renew. `Notifying` still says the session died.
    #[test]
    fn a_disconnect_nobody_saw_still_renews_the_subscription() {
        let mut synced = HashSet::new();
        poll(&mut synced, Some(false), true, true);
        poll(&mut synced, Some(true), true, true);

        assert!(poll(&mut synced, Some(false), true, true).start_notify);
    }

    #[test]
    fn a_session_somebody_else_armed_is_read_once_before_it_is_trusted() {
        let mut synced = HashSet::new();
        // Nothing in this process subscribed, yet BlueZ is notifying: whatever
        // it holds is from before we were listening.
        assert_eq!(
            poll(&mut synced, Some(true), true, true),
            Plan {
                start_notify: false,
                read: true
            }
        );
        assert!(!poll(&mut synced, Some(true), true, true).read);
    }

    #[test]
    fn a_characteristic_that_will_not_notify_is_read_every_time() {
        let mut synced = HashSet::new();
        for _ in 0..3 {
            let plan = poll(&mut synced, Some(false), false, true);
            assert!(plan.read, "the value still has to be right");
        }
    }

    #[test]
    fn a_read_that_failed_is_not_taken_for_a_value() {
        let mut synced = HashSet::new();
        poll(&mut synced, Some(false), true, false);
        // Nothing was learnt, so the next snapshot must not fall through to the
        // cache on the strength of a subscription that told it nothing.
        assert!(poll(&mut synced, Some(true), true, true).read);
    }

    #[test]
    fn a_characteristic_bluez_says_nothing_about_is_read() {
        let mut synced = HashSet::new();
        assert_eq!(
            poll(&mut synced, None, true, true),
            Plan {
                start_notify: true,
                read: true
            }
        );
    }

    #[test]
    fn a_disconnected_board_stops_being_tracked() {
        let device = "/org/bluez/hci0/dev_E8_6B_FC_47_E2_64";
        let level = format!("{device}/service0016/char0017");
        let mut synced = HashSet::from([level.clone()]);

        keep(&mut synced, &[device]);
        assert!(synced.contains(&level), "still connected, still tracked");

        keep(&mut synced, &[]);
        assert!(synced.is_empty());
    }

    #[test]
    fn one_board_disconnecting_leaves_another_alone() {
        let ours = "/org/bluez/hci0/dev_E8_6B_FC_47_E2_64";
        let other = "/org/bluez/hci0/dev_F8_DE_F9_BF_5F_5F";
        let mut synced = HashSet::from([
            format!("{ours}/service0016/char0017"),
            format!("{other}/service0016/char0017"),
        ]);

        keep(&mut synced, &[ours]);
        assert_eq!(synced.len(), 1);
        assert!(synced.iter().all(|path| path.starts_with(ours)));
    }
}
