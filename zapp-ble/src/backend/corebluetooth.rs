//! CoreBluetooth.
//!
//! macOS answers presence in three pieces rather than one call. A board the
//! system is connected to — which is a board being typed on — comes from
//! `retrieveConnectedPeripheralsWithServices`. One advertising in range comes
//! from a scan. One that is bonded but neither cannot be enumerated at all: it
//! is only reachable by an identifier somebody kept, which is what `known` and
//! the peripherals remembered for the life of the process are for.
//!
//! CoreBluetooth is callback-driven, and this crate is blocking, so the two
//! meet in one place. A single central per process runs on a serial dispatch
//! queue of its own; every call into CoreBluetooth is made on that queue with
//! `exec_sync`, and every delegate callback runs there too and turns what it
//! was handed into plain Rust data under a mutex. A caller waits on the condvar
//! beside that mutex for the answer it is after. One central rather than one
//! per backend, because an update opens and drops links all the way through and
//! each new central would cost a power-on handshake and lose every peripheral
//! the last one had retrieved.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

use dispatch2::{DispatchQueue, DispatchQueueAttr, DispatchRetained};
use objc2::rc::Retained;
use objc2::runtime::{AnyObject, NSObject, ProtocolObject};
use objc2::{define_class, msg_send, AllocAnyThread, DefinedClass, Message};
use objc2_core_bluetooth::{
    CBAdvertisementDataLocalNameKey, CBAdvertisementDataServiceUUIDsKey, CBCentralManager,
    CBCentralManagerDelegate, CBCharacteristic, CBCharacteristicWriteType, CBDescriptor, CBManagerState,
    CBPeripheral, CBPeripheralDelegate, CBPeripheralState, CBService, CBUUID,
};
use objc2_foundation::{
    NSArray, NSData, NSDictionary, NSError, NSNumber, NSObjectProtocol, NSString, NSUUID,
};

use super::{Backend, GattChannel, Usage};
use crate::battery::{
    self, Battery, ChargeDetail, Level, BATTERY_LEVEL, BATTERY_SERVICE, CHARGE_DETAIL,
    PRESENTATION_FORMAT,
};
use crate::smp::{RELAY_SERVICE, SMP_SERVICE};
use crate::{identity, BleError, BleSighting};
use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE;

/// How long a fresh central has to say whether Bluetooth is on.
///
/// The first answer comes as a callback shortly after the central is made, and
/// is the one that carries a permission refusal as well as a switched-off
/// radio.
const POWER_TIMEOUT: Duration = Duration::from_secs(5);

/// How long to wait for a link.
///
/// CoreBluetooth never gives up on a connection by itself: the request stays
/// pending until the board shows up, so the deadline has to be ours. Matches
/// BlueZ's, which waits out a bonded board's advertising interval the same way.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);

/// How long one round of service or characteristic discovery may take.
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(10);

/// How long a read, an acknowledged write or a subscription may go unanswered.
///
/// The ceiling is set by the slowest of them: the update service erases the
/// STM32 inside the START write's own handler, so that acknowledgement can be
/// seconds coming. The same 25 seconds D-Bus allows a method call.
const REPLY_TIMEOUT: Duration = Duration::from_secs(25);

/// The discovery window, the same rhythm as BlueZ's and for the same reason: a
/// scanning radio is airtime the links do not get.
const SCAN_WINDOW: Duration = Duration::from_secs(4);
const SCAN_EVERY: Duration = Duration::from_secs(30);

/// How long an advertisement counts as "in range".
///
/// CoreBluetooth reports a device once per scan and says nothing when it goes
/// quiet, so freshness is ours to decide. Long enough to bridge the gap between
/// two windows, so a board that has not moved does not flicker out of range.
const ADVERT_FRESH: Duration = Duration::from_secs(SCAN_EVERY.as_secs() + 2 * SCAN_WINDOW.as_secs());

/// Device Information, and its PnP ID: the USB vendor and product a board
/// enumerates with, which is how BlueZ names the product in its modalias.
const DEVICE_INFORMATION: u128 = 0x0000180a_0000_1000_8000_00805f9b34fb;
const PNP_ID: u128 = 0x00002a50_0000_1000_8000_00805f9b34fb;

/// How long a board's profile — model and battery — may take to read.
const PROFILE_TIMEOUT: Duration = Duration::from_secs(5);

/// The payload of the 23-byte ATT MTU every link starts on, and how long to
/// give a fresh link to negotiate past it.
const DEFAULT_PAYLOAD: usize = 20;
const MTU_SETTLE: Duration = Duration::from_secs(2);

/// How long to let a burst of callbacks finish before reading the state again.
const SETTLE: Duration = Duration::from_millis(250);

/// Services that make a system-connected peripheral worth looking at.
///
/// HID would be the obvious one, but CoreBluetooth hides it from apps. Each is
/// asked about on its own, so the answer also says which of them a board
/// carries.
const RETRIEVE_BY: [u128; 3] = [ZSA_BLE_DFU_SERVICE, SMP_SERVICE, RELAY_SERVICE];

/// The backend: the process's one central, plus this backend's scan rhythm.
pub struct CoreBluetooth {
    central: Result<&'static Central, BleError>,
    known: Vec<String>,
    scanning_since: Option<Instant>,
    scanned_at: Option<Instant>,
    /// The generation `wait_for_change` last saw.
    seen: u64,
}

impl CoreBluetooth {
    /// Never fails: a central that cannot be had — Bluetooth off, permission
    /// refused — is reported by `sightings` instead, so the reason reaches the
    /// person rather than being flattened into "no adapter".
    pub fn new(known: &[String]) -> Self {
        Self {
            central: Central::get(),
            known: known.to_vec(),
            scanning_since: None,
            scanned_at: None,
            seen: 0,
        }
    }

    fn central(&self) -> Result<&'static Central, BleError> {
        match &self.central {
            Ok(central) => {
                central.ready()?;
                Ok(central)
            }
            Err(err) => Err(BleError::Platform(err.to_string())),
        }
    }

    fn pump_discovery(&mut self, central: &Central) {
        match self.scanning_since {
            Some(started) if started.elapsed() >= SCAN_WINDOW => {
                central.scan(false);
                self.scanning_since = None;
                self.scanned_at = Some(Instant::now());
            }
            Some(_) => {}
            None => {
                if self
                    .scanned_at
                    .is_none_or(|ended| ended.elapsed() >= SCAN_EVERY)
                {
                    central.scan(true);
                    self.scanning_since = Some(Instant::now());
                }
            }
        }
    }
}

impl Backend for CoreBluetooth {
    fn sightings(&mut self) -> Result<Vec<BleSighting>, BleError> {
        let central = self.central()?;
        self.pump_discovery(central);
        let sightings = central.sightings(&self.known);

        // What BlueZ keeps for a bonded board — its model and its levels —
        // CoreBluetooth keeps to itself, so a connected board is asked once per
        // connection. After that its levels arrive as notifications.
        let fresh: Vec<&str> = sightings
            .iter()
            .filter(|sighting| sighting.connected && identity::is_zsa_board(sighting))
            .map(|sighting| sighting.key.as_str())
            .filter(|key| !central.profiled(key))
            .collect();
        if fresh.is_empty() {
            return Ok(sightings);
        }
        for key in fresh {
            central.profile(key);
        }
        Ok(central.sightings(&self.known))
    }

    fn wait_for_change(&mut self, timeout: Duration) {
        let Ok(central) = &self.central else {
            thread::sleep(timeout);
            return;
        };
        let seen = self.seen;
        let changed = central.wait(timeout, |events| {
            (events.generation != seen).then_some(events.generation)
        });
        if let Some(generation) = changed {
            thread::sleep(SETTLE);
            self.seen = central.events().generation.max(generation);
        }
    }

    fn stop_discovery(&mut self) {
        if self.scanning_since.take().is_none() {
            return;
        }
        if let Ok(central) = &self.central {
            central.scan(false);
        }
        self.scanned_at = Some(Instant::now());
    }

    fn open_as(
        &mut self,
        key: &str,
        service: u128,
        characteristic: u128,
        usage: Usage,
    ) -> Result<Box<dyn GattChannel>, BleError> {
        let central = self.central()?;
        let session = central.connect(key)?;
        central.discover(key, service, characteristic)?;
        let mtu = central.mtu(key)?;
        log::debug!("characteristic {characteristic:032x} on {key}, {mtu} bytes per write");

        let channel = Channel {
            central,
            key: key.to_string(),
            attribute: (service, characteristic),
            usage,
            mtu,
            session,
        };
        if usage.notify {
            central.subscribe(&channel)?;
        }
        Ok(Box::new(channel))
    }
}

/// What one board's charge-detail characteristic says, read now.
///
/// The same one-shot as BlueZ's: nothing keeps it fresh, and every way it can
/// come to nothing — Bluetooth off, a board that is not connected, firmware
/// without the characteristic, a read that failed — is `None`, since none of
/// them is worth a line under a board in a listing.
pub(super) fn read_charge_detail(key: &str) -> Option<ChargeDetail> {
    let central = Central::get().ok()?;
    central.ready().ok()?;
    let session = central.connect(key).ok()?;
    if let Err(err) = central.discover_everything(key, &[]) {
        log::debug!("cannot look for {key}'s charge detail: {err}");
        return None;
    }

    let service = central.on_queue(|objects| unsafe {
        let peripheral = objects.delegate.peripheral(key)?;
        let service = peripheral
            .services()?
            .iter()
            .find(|service| find_characteristic(service, CHARGE_DETAIL).is_some())?;
        from_cbuuid(&service.UUID())
    });
    let Some(service) = service else {
        log::debug!("{key} publishes no charge detail characteristic");
        return None;
    };

    let channel = Channel {
        central,
        key: key.to_string(),
        attribute: (service, CHARGE_DETAIL),
        usage: Usage {
            notify: false,
            written: false,
            acknowledged: false,
        },
        mtu: 0,
        session,
    };
    match central.read(&channel) {
        Ok(value) => ChargeDetail::decode(&value),
        Err(err) => {
            log::debug!("cannot read {key}'s charge detail: {err}");
            None
        }
    }
}

/// One characteristic on one board.
///
/// Holds no CoreBluetooth object: those may only be touched on the queue, and
/// a characteristic object is only good for the connection that discovered it.
/// What it holds instead is enough to look it up again, and the session it was
/// opened under — a channel that outlives its connection says so rather than
/// writing into a characteristic CoreBluetooth has already thrown away.
struct Channel {
    central: &'static Central,
    key: String,
    attribute: Attribute,
    usage: Usage,
    mtu: usize,
    session: u64,
}

impl GattChannel for Channel {
    fn mtu(&self) -> usize {
        self.mtu
    }

    fn read(&mut self) -> Result<Vec<u8>, BleError> {
        self.central.read(self)
    }

    fn write(&mut self, value: &[u8]) -> Result<(), BleError> {
        // CoreBluetooth would split a long acknowledged write into a long write
        // a characteristic like SMP's does not take, and drops an oversized
        // unacknowledged one without a word. Neither is a write that happened.
        if value.len() > self.mtu {
            return Err(BleError::Platform(format!(
                "a {}-byte write will not fit the {}-byte ATT payload",
                value.len(),
                self.mtu
            )));
        }
        match self.usage.acknowledged {
            true => self.central.write_acknowledged(self, value),
            false => self.central.write_unacknowledged(self, value),
        }
    }

    fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        if !self.usage.notify {
            thread::sleep(timeout);
            return Ok(None);
        }
        self.central.notification(self, timeout)
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        if self.usage.notify {
            self.central.unsubscribe(self);
        }
    }
}

/// A characteristic, by the service it is under and its own UUID.
type Attribute = (u128, u128);

/// Everything the delegate has learnt, as plain data.
#[derive(Default)]
struct Events {
    /// `CBManagerState`, once the central has said.
    power: Option<isize>,
    /// Bumped on anything that could change a sighting.
    generation: u64,
    adverts: HashMap<String, Advert>,
    links: HashMap<String, Link>,
    /// Every service each peripheral has been seen to carry. Kept across
    /// disconnects, the way BlueZ keeps a bonded device's: CoreBluetooth
    /// forgets a peripheral's services the moment its link drops.
    services: HashMap<String, HashSet<u128>>,
    /// Peripherals known to be bonded: connected to the system, connected to by
    /// us, or handed in as known. Nothing else in CoreBluetooth says.
    bonded: HashSet<String>,
    /// What each board's PnP ID read as, in BlueZ's modalias form. Kept across
    /// disconnects: it names the product, which does not change.
    modalias: HashMap<String, String>,
}

#[derive(Debug, Clone)]
struct Advert {
    name: Option<String>,
    services: Vec<u128>,
    rssi: i16,
    seen: Instant,
}

/// One peripheral's link, as far as this process's central is concerned.
#[derive(Default)]
struct Link {
    connected: bool,
    /// Counts connections, so a channel can tell its own from a later one.
    session: u64,
    /// Why the last attempt failed or the last link went down.
    failed: Option<String>,
    /// Set by each discovery round as it answers.
    discovered: Option<Result<(), String>>,
    /// Write-without-response has room again.
    ready_to_write: bool,
    attributes: HashMap<Attribute, Attr>,
    /// Counts the answers a profile waits on: characteristic and descriptor
    /// discoveries, and reads of what it watches. Only ever grows, so a caller
    /// waits for it to pass the figure it started from plus what it asked.
    answers: u64,
    /// Every service has been discovered on this connection, not only the
    /// ones a channel asked for.
    all_services: bool,
    /// The profile has been read on this connection.
    profiled: bool,
    /// Which half each Battery Level characteristic speaks for, and the last
    /// level each said, by characteristic. A split board publishes several
    /// Battery Service instances under the one UUID, so the object is the
    /// only thing that tells them apart.
    halves: HashMap<usize, battery::Half>,
    levels: HashMap<usize, u8>,
}

impl Link {
    /// What the halves have left, as last read or notified. Nothing while the
    /// link is down: a board publishes its levels only while connected.
    fn battery(&self) -> Battery {
        let mut battery = Battery::default();
        if !self.connected {
            return battery;
        }
        for (characteristic, half) in &self.halves {
            let level = self
                .levels
                .get(characteristic)
                .map_or(Level::Unknown, |raw| Level::from_raw(*raw));
            battery.set(*half, level);
        }
        battery
    }

    /// Back to the state of a link nothing has been asked of, keeping only
    /// what outlives a connection.
    fn reset(&mut self, connected: bool) {
        *self = Link {
            connected,
            session: self.session + u64::from(connected),
            ready_to_write: connected,
            ..Link::default()
        };
    }
}

/// One characteristic's traffic.
#[derive(Default)]
struct Attr {
    subscribers: usize,
    subscribed: Option<Result<(), String>>,
    /// Every notify on or off asked for and not yet answered, oldest first.
    ///
    /// CoreBluetooth answers both through one callback, in the order they
    /// were asked, and an update drops one channel and opens the next straight
    /// after — so an "off" for the old channel is routinely still in flight
    /// when the new one's "on" goes out. Matching each answer to its request
    /// is what keeps the first from being read as the second's refusal.
    requested: VecDeque<bool>,
    notifications: VecDeque<Vec<u8>>,
    reading: bool,
    read: Option<Result<Vec<u8>, String>>,
    written: Option<Result<(), String>>,
}

struct Shared {
    events: Mutex<Events>,
    changed: Condvar,
}

impl Shared {
    fn update(&self, f: impl FnOnce(&mut Events)) {
        let mut events = self.events.lock().unwrap_or_else(|poison| poison.into_inner());
        f(&mut events);
        drop(events);
        self.changed.notify_all();
    }
}

/// What may only be touched on the central's queue.
///
/// CoreBluetooth objects are neither `Send` nor `Sync`, and the bindings say
/// so. They are kept behind this and reached only from inside `exec_sync`,
/// which runs on the one serial queue every delegate callback also runs on, so
/// nothing is ever touched from two threads at once.
struct QueueOnly<T>(T);

// SAFETY: see above; `Central::on_queue` is the only way in.
unsafe impl<T> Send for QueueOnly<T> {}
// SAFETY: see above.
unsafe impl<T> Sync for QueueOnly<T> {}

struct Objects {
    manager: Retained<CBCentralManager>,
    delegate: Retained<Delegate>,
}

struct Central {
    queue: DispatchRetained<DispatchQueue>,
    objects: QueueOnly<Objects>,
    shared: Arc<Shared>,
}

static CENTRAL: OnceLock<Result<Central, String>> = OnceLock::new();

impl Central {
    fn get() -> Result<&'static Central, BleError> {
        CENTRAL
            .get_or_init(Central::start)
            .as_ref()
            .map_err(|err| BleError::Platform(err.clone()))
    }

    fn start() -> Result<Central, String> {
        let queue = DispatchQueue::new("com.zsa.zapp.ble", DispatchQueueAttr::SERIAL);
        let shared = Arc::new(Shared {
            events: Mutex::new(Events::default()),
            changed: Condvar::new(),
        });
        let delegate = Delegate::new(Arc::clone(&shared));
        // SAFETY: the queue is serial and outlives the manager (both live in
        // the process-wide `CENTRAL`); every later call is made on it.
        let manager = unsafe {
            CBCentralManager::initWithDelegate_queue(
                CBCentralManager::alloc(),
                Some(ProtocolObject::from_ref(&*delegate)),
                Some(&queue),
            )
        };
        Ok(Central {
            queue,
            objects: QueueOnly(Objects { manager, delegate }),
            shared,
        })
    }

    /// Runs `f` on the queue, with the CoreBluetooth objects in reach.
    fn on_queue<R: Send>(&self, f: impl FnOnce(&Objects) -> R + Send) -> R {
        let objects = &self.objects;
        let mut out = None;
        let slot = QueueOnly(&mut out);
        self.queue.exec_sync(move || {
            let slot = slot;
            *slot.0 = Some(f(&objects.0));
        });
        out.expect("exec_sync ran the block")
    }

    fn events(&self) -> MutexGuard<'_, Events> {
        self.shared
            .events
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Waits until `check` finds what it is after, or `timeout` elapses.
    fn wait<T>(&self, timeout: Duration, mut check: impl FnMut(&mut Events) -> Option<T>) -> Option<T> {
        let deadline = Instant::now() + timeout;
        let mut events = self.events();
        loop {
            if let Some(found) = check(&mut events) {
                return Some(found);
            }
            let left = deadline.checked_duration_since(Instant::now())?;
            events = self
                .shared
                .changed
                .wait_timeout(events, left)
                .unwrap_or_else(|poison| poison.into_inner())
                .0;
        }
    }

    /// Waits for Bluetooth to be on, and says why when it is not.
    fn ready(&self) -> Result<(), BleError> {
        let power = self.wait(POWER_TIMEOUT, |events| {
            events.power.filter(|power| {
                // Unknown and resetting are on the way to an answer.
                *power != CBManagerState::Unknown.0 && *power != CBManagerState::Resetting.0
            })
        });
        match power.map(CBManagerState) {
            Some(CBManagerState::PoweredOn) => Ok(()),
            Some(CBManagerState::PoweredOff) => Err(BleError::Platform("Bluetooth is off".into())),
            Some(CBManagerState::Unauthorized) => Err(BleError::Platform(
                "no Bluetooth permission: allow it under System Settings > Privacy & Security > \
                 Bluetooth (for zapp, allow your terminal)"
                    .into(),
            )),
            Some(CBManagerState::Unsupported) => Err(BleError::NoAdapter),
            _ => Err(BleError::Platform("Bluetooth is not available".into())),
        }
    }

    fn scan(&self, on: bool) {
        self.on_queue(|objects| unsafe {
            match on {
                // Every service, so an advertising board shows up whichever of
                // its UUIDs made it into the payload.
                true => objects
                    .manager
                    .scanForPeripheralsWithServices_options(None, None),
                false => objects.manager.stopScan(),
            }
        });
    }

    /// Everything this central can see, ZSA or not.
    fn sightings(&self, known: &[String]) -> Vec<BleSighting> {
        // What the system is connected to, asked one service at a time so the
        // answer also says which services each one carries.
        let found: Vec<Found> = self.on_queue(|objects| unsafe {
            let mut connected: HashMap<String, Vec<u128>> = HashMap::new();
            for service in RETRIEVE_BY {
                let uuids = NSArray::from_retained_slice(&[cbuuid(service)]);
                for peripheral in objects
                    .manager
                    .retrieveConnectedPeripheralsWithServices(&uuids)
                    .iter()
                {
                    let key = objects.delegate.remember(&peripheral);
                    connected.entry(key).or_default().push(service);
                }
            }

            // Bonded boards the caller kept the identifiers of. Only those not
            // already remembered: retrieving costs nothing, but it hands back a
            // fresh object each time.
            let missing: Vec<Retained<NSUUID>> = known
                .iter()
                .filter(|key| !objects.delegate.knows(key))
                .filter_map(|key| NSUUID::from_string(&NSString::from_str(key)))
                .collect();
            if !missing.is_empty() {
                let ids = NSArray::from_retained_slice(&missing);
                for peripheral in objects.manager.retrievePeripheralsWithIdentifiers(&ids).iter() {
                    objects.delegate.remember(&peripheral);
                }
            }

            objects
                .delegate
                .ivars()
                .peripherals
                .borrow()
                .iter()
                .map(|(key, peripheral)| Found {
                    key: key.clone(),
                    name: peripheral.name().map(|name| name.to_string()),
                    connected: connected.contains_key(key)
                        || peripheral.state() == CBPeripheralState::Connected,
                    services: connected.remove(key).unwrap_or_default(),
                })
                .collect()
        });

        let mut events = self.events();
        let events = &mut *events;
        events.bonded.extend(known.iter().cloned());
        found
            .into_iter()
            .map(|found| {
                if found.connected {
                    // Connected to this machine, so bonded to it: a ZSA board
                    // only offers its services over an encrypted link.
                    events.bonded.insert(found.key.clone());
                }
                let services = events.services.entry(found.key.clone()).or_default();
                services.extend(found.services);
                let advert = events
                    .adverts
                    .get(&found.key)
                    .filter(|advert| advert.seen.elapsed() < ADVERT_FRESH);

                BleSighting {
                    name: found.name.or_else(|| advert.and_then(|advert| advert.name.clone())),
                    services: services.iter().copied().collect(),
                    advertised: advert.map(|advert| advert.services.clone()).unwrap_or_default(),
                    // Read off the board's PnP ID, since macOS keeps its own
                    // copy to itself.
                    modalias: events.modalias.get(&found.key).cloned(),
                    connected: found.connected,
                    paired: events.bonded.contains(&found.key),
                    rssi: advert.map(|advert| advert.rssi),
                    battery: events
                        .links
                        .get(&found.key)
                        .map(Link::battery)
                        .unwrap_or_default(),
                    key: found.key,
                }
            })
            .collect()
    }

    /// Brings our side of the link up, and returns the session it is.
    ///
    /// A board the system is already connected to still has to be connected to
    /// by this central before it can be talked to; that costs no radio time,
    /// since the link is already there.
    fn connect(&self, key: &str) -> Result<u64, BleError> {
        let started = self.on_queue(|objects| unsafe {
            let peripheral = objects.delegate.peripheral(key).or_else(|| {
                let id = NSUUID::from_string(&NSString::from_str(key))?;
                let ids = NSArray::from_retained_slice(&[id]);
                let peripheral = objects
                    .manager
                    .retrievePeripheralsWithIdentifiers(&ids)
                    .firstObject()?;
                objects.delegate.remember(&peripheral);
                Some(peripheral)
            })?;
            peripheral.setDelegate(Some(ProtocolObject::from_ref(&*objects.delegate)));
            if peripheral.state() == CBPeripheralState::Connected {
                return Some(true);
            }
            log::info!("connecting to {key}");
            objects.delegate.ivars().shared.update(|events| {
                events.links.entry(key.to_string()).or_default().failed = None;
            });
            objects.manager.connectPeripheral_options(&peripheral, None);
            Some(false)
        });
        let Some(already) = started else {
            return Err(BleError::NotFound(format!(
                "{key} is not a peripheral this Mac knows"
            )));
        };

        let outcome = self.wait(CONNECT_TIMEOUT, |events| {
            let link = events.links.entry(key.to_string()).or_default();
            if link.connected {
                return Some(Ok(link.session));
            }
            link.failed.take().map(Err)
        });
        match outcome {
            Some(Ok(session)) => {
                self.events().bonded.insert(key.to_string());
                Ok(session)
            }
            Some(Err(why)) => Err(BleError::Platform(format!("cannot connect to {key}: {why}"))),
            None if already => Err(BleError::Platform(format!(
                "{key} is connected, but CoreBluetooth never said so"
            ))),
            None => {
                // The request would otherwise stay pending for as long as the
                // process lives, and connect the board whenever it next showed.
                self.cancel(key);
                Err(BleError::Platform(format!(
                    "cannot connect to {key}: no answer in {}s",
                    CONNECT_TIMEOUT.as_secs()
                )))
            }
        }
    }

    fn profiled(&self, key: &str) -> bool {
        self.events()
            .links
            .get(key)
            .is_some_and(|link| link.connected && link.profiled)
    }

    /// Reads a connected board's model and battery levels, and subscribes to
    /// the levels so they stay current.
    ///
    /// Best effort: whatever does not answer is left unknown, and the attempt
    /// is not repeated until the next connection — a board that will not say
    /// is not asked again every few seconds.
    fn profile(&self, key: &str) {
        if let Err(err) = self.connect(key) {
            log::debug!("cannot profile {key}: {err}");
            return;
        }
        let started = Instant::now();
        if let Err(err) = self.discover_everything(key, &[BATTERY_SERVICE, DEVICE_INFORMATION]) {
            log::debug!("cannot profile {key}: {err}");
        }

        // Which characteristics to watch, and the descriptors beside each
        // level that say which half it is.
        let target = self.on_queue(|objects| unsafe {
            let peripheral = objects.delegate.peripheral(key)?;
            // Read on the queue, before anything is asked: the answers arrive
            // on this queue too, so none can have been counted yet.
            let mut target = objects.delegate.answers(key);
            for service in peripheral.services()?.iter() {
                log::trace!(
                    "{key} service {} with {:?}",
                    service.UUID().UUIDString(),
                    service.characteristics().map(|characteristics| characteristics
                        .iter()
                        .map(|characteristic| characteristic.UUID().UUIDString().to_string())
                        .collect::<Vec<_>>())
                );
                match from_cbuuid(&service.UUID()) {
                    Some(BATTERY_SERVICE) => {
                        if let Some(level) = find_characteristic(&service, BATTERY_LEVEL) {
                            objects.delegate.watch(key, &level, What::Level);
                            peripheral.discoverDescriptorsForCharacteristic(&level);
                            target += 1;
                        }
                    }
                    Some(DEVICE_INFORMATION) => {
                        if let Some(pnp) = find_characteristic(&service, PNP_ID) {
                            objects.delegate.watch(key, &pnp, What::PnpId);
                        }
                    }
                    _ => {}
                }
            }
            Some(target)
        });
        self.answered(key, target, |_| {});

        // Which half each level is for, the model, the levels themselves, and
        // a subscription to keep the levels coming.
        let known_model = self.events().modalias.contains_key(key);
        let target = self.on_queue(|objects| unsafe {
            let peripheral = objects.delegate.peripheral(key)?;
            let mut target = objects.delegate.answers(key);
            for (what, characteristic) in objects.delegate.watched(key) {
                match what {
                    What::Level => {
                        let format = characteristic.descriptors().and_then(|descriptors| {
                            descriptors.iter().find(|descriptor| {
                                from_cbuuid(&descriptor.UUID()) == Some(PRESENTATION_FORMAT)
                            })
                        });
                        if let Some(format) = format {
                            peripheral.readValueForDescriptor(&format);
                            target += 1;
                        }
                        peripheral.readValueForCharacteristic(&characteristic);
                        peripheral.setNotifyValue_forCharacteristic(true, &characteristic);
                        target += 1;
                    }
                    What::PnpId if !known_model => {
                        peripheral.readValueForCharacteristic(&characteristic);
                        target += 1;
                    }
                    What::PnpId => {}
                }
            }
            Some(target)
        });
        self.answered(key, target, |_| {});
        // Asked once per connection whatever came of it.
        if let Some(link) = self.events().links.get_mut(key) {
            link.profiled = link.connected;
        }
        log::debug!("profiled {key} in {} ms", started.elapsed().as_millis());
    }

    /// Waits for a board's answer counter to reach `target`, then runs `done`
    /// on its link. Gives up quietly at the deadline or when the link drops:
    /// whatever did not answer stays unknown.
    fn answered(&self, key: &str, target: Option<u64>, done: impl FnOnce(&mut Link)) {
        let mut done = Some(done);
        let Some(target) = target else {
            return;
        };
        let finished = self.wait(PROFILE_TIMEOUT, |events| {
            let link = events.links.get_mut(key)?;
            if link.connected && link.answers < target {
                return None;
            }
            done.take().map(|done| done(link))
        });
        if finished.is_none() {
            let answers = self.events().links.get(key).map_or(0, |link| link.answers);
            log::debug!("{key}: {answers} of {target} answers before the deadline");
        }
    }

    /// Discovers every service — or every one of `wanted`, when given — and
    /// all their characteristics.
    fn discover_everything(&self, key: &str, wanted: &[u128]) -> Result<(), BleError> {
        let started = self.on_queue(|objects| unsafe {
            let peripheral = objects.delegate.peripheral(key)?;
            let mut all = false;
            objects.delegate.ivars().shared.update(|events| {
                let link = events.links.entry(key.to_string()).or_default();
                all = link.all_services;
                if !all {
                    link.discovered = None;
                }
            });
            if !all {
                peripheral.discoverServices(None);
            }
            Some(!all)
        });
        match started {
            None => return Err(BleError::NotFound(format!("{key} went away"))),
            Some(true) => {
                let answer = self.wait(DISCOVER_TIMEOUT, |events| {
                    let link = events.links.get_mut(key)?;
                    if !link.connected {
                        return Some(Err("the board closed the link".to_string()));
                    }
                    link.discovered.take()
                });
                match answer {
                    Some(Ok(())) => {
                        if let Some(link) = self.events().links.get_mut(key) {
                            link.all_services = true;
                        }
                    }
                    Some(Err(why)) => {
                        return Err(BleError::Platform(format!(
                            "cannot discover {key}'s services: {why}"
                        )))
                    }
                    None => {
                        return Err(BleError::Platform(format!(
                            "{key} never listed its services"
                        )))
                    }
                }
            }
            Some(false) => {}
        }

        let target = self.on_queue(|objects| unsafe {
            let peripheral = objects.delegate.peripheral(key)?;
            let mut target = objects.delegate.answers(key);
            for service in peripheral.services()?.iter() {
                let uuid = from_cbuuid(&service.UUID());
                let wanted = wanted.is_empty() || uuid.is_some_and(|uuid| wanted.contains(&uuid));
                if wanted && service.characteristics().is_none() {
                    peripheral.discoverCharacteristics_forService(None, &service);
                    target += 1;
                }
            }
            Some(target)
        });
        self.answered(key, target, |_| {});
        Ok(())
    }

    fn cancel(&self, key: &str) {
        self.on_queue(|objects| unsafe {
            if let Some(peripheral) = objects.delegate.peripheral(key) {
                objects.manager.cancelPeripheralConnection(&peripheral);
            }
        });
    }

    /// Walks as much of the GATT tree as it takes to reach one characteristic.
    ///
    /// Asks only for what is missing: the service if it has not been
    /// discovered, its characteristics if they have not. A second channel on
    /// the same service costs nothing.
    fn discover(&self, key: &str, service: u128, characteristic: u128) -> Result<(), BleError> {
        #[derive(Clone, Copy, PartialEq)]
        enum Step {
            Found,
            Services,
            Characteristics,
        }

        let missing = || BleError::Missing {
            device: key.to_string(),
            characteristic,
        };

        let mut asked = Vec::new();
        loop {
            let step = self.on_queue(|objects| unsafe {
                let peripheral = objects.delegate.peripheral(key)?;
                let reset = || {
                    objects.delegate.ivars().shared.update(|events| {
                        events.links.entry(key.to_string()).or_default().discovered = None;
                    })
                };
                let Some(found) = find_service(&peripheral, service) else {
                    reset();
                    // A filtered discovery may leave CoreBluetooth listing
                    // only what it asked for.
                    objects.delegate.ivars().shared.update(|events| {
                        events.links.entry(key.to_string()).or_default().all_services = false;
                    });
                    let uuids = NSArray::from_retained_slice(&[cbuuid(service)]);
                    peripheral.discoverServices(Some(&uuids));
                    return Some(Step::Services);
                };
                if find_characteristic(&found, characteristic).is_some() {
                    return Some(Step::Found);
                }
                reset();
                peripheral.discoverCharacteristics_forService(None, &found);
                Some(Step::Characteristics)
            });

            match step {
                None => return Err(BleError::NotFound(format!("{key} went away"))),
                Some(Step::Found) => return Ok(()),
                // Asked once and answered without it: the board does not have it.
                Some(step) if asked.contains(&step) => return Err(missing()),
                Some(step) => asked.push(step),
            }

            let answer = self.wait(DISCOVER_TIMEOUT, |events| {
                let link = events.links.get_mut(key)?;
                if !link.connected {
                    return Some(Err("the board closed the link".to_string()));
                }
                link.discovered.take()
            });
            match answer {
                Some(Ok(())) => {}
                Some(Err(why)) => {
                    return Err(BleError::Platform(format!(
                        "cannot discover {key}'s services: {why}"
                    )))
                }
                None => {
                    return Err(BleError::Platform(format!(
                        "{key} connected but never listed its services"
                    )))
                }
            }
        }
    }

    /// Bytes of payload one write can carry.
    ///
    /// CoreBluetooth answers this directly — the ATT MTU less its header, which
    /// BlueZ leaves to the caller. The without-response figure for every kind
    /// of write: the acknowledged figure is what a long write could carry,
    /// which nothing here wants.
    fn mtu(&self, key: &str) -> Result<usize, BleError> {
        let deadline = Instant::now() + MTU_SETTLE;
        loop {
            let mtu = self.on_queue(|objects| unsafe {
                objects.delegate.peripheral(key).map(|peripheral| {
                    peripheral
                        .maximumWriteValueLengthForType(CBCharacteristicWriteType::WithoutResponse)
                })
            });
            match mtu {
                // The exchange may still be under way on a link this central
                // just brought up, and until it lands CoreBluetooth reports
                // the 23-byte default. A system-held link settled long ago.
                Some(mtu) if mtu <= DEFAULT_PAYLOAD && Instant::now() < deadline => {
                    thread::sleep(Duration::from_millis(100));
                }
                Some(mtu) if mtu > 0 => return Ok(mtu),
                Some(mtu) => {
                    return Err(BleError::Platform(format!(
                        "CoreBluetooth reported {mtu} bytes per write, which cannot carry anything"
                    )))
                }
                None => return Err(BleError::NotFound(format!("{key} went away"))),
            }
        }
    }

    /// Runs `f` on the queue with the channel's characteristic, or says why
    /// it is not there.
    fn with_characteristic<R: Send>(
        &self,
        channel: &Channel,
        f: impl FnOnce(&CBPeripheral, &CBCharacteristic, &Shared) -> R + Send,
    ) -> Result<R, BleError> {
        self.alive(channel)?;
        let (service, characteristic) = channel.attribute;
        let key = channel.key.as_str();
        self.on_queue(|objects| unsafe {
            let peripheral = objects.delegate.peripheral(key)?;
            let found = find_characteristic(&*find_service(&peripheral, service)?, characteristic)?;
            Some(f(&peripheral, &found, &objects.delegate.ivars().shared))
        })
        .ok_or_else(|| BleError::Platform("the board closed the link".into()))
    }

    /// Whether the connection a channel was opened on is still up.
    fn alive(&self, channel: &Channel) -> Result<(), BleError> {
        let events = self.events();
        match events.links.get(&channel.key) {
            Some(link) if link.connected && link.session == channel.session => Ok(()),
            _ => Err(BleError::Platform("the board closed the link".into())),
        }
    }

    /// Waits on one characteristic's traffic, giving up early if the link goes.
    fn wait_on<T>(
        &self,
        channel: &Channel,
        timeout: Duration,
        mut check: impl FnMut(&mut Attr) -> Option<T>,
    ) -> Result<Option<T>, BleError> {
        let lost = self.wait(timeout, |events| {
            let link = events.links.get_mut(&channel.key)?;
            if !link.connected || link.session != channel.session {
                return Some(Err(()));
            }
            check(link.attributes.entry(channel.attribute).or_default()).map(Ok)
        });
        match lost {
            Some(Ok(found)) => Ok(Some(found)),
            Some(Err(())) => Err(BleError::Platform("the board closed the link".into())),
            None => Ok(None),
        }
    }

    fn subscribe(&self, channel: &Channel) -> Result<(), BleError> {
        let attribute = channel.attribute;
        let key = channel.key.clone();
        self.with_characteristic(channel, move |peripheral, characteristic, shared| unsafe {
            let mut first = false;
            shared.update(|events| {
                let link = events.links.entry(key).or_default();
                let attr = link.attributes.entry(attribute).or_default();
                attr.subscribers += 1;
                if attr.subscribers == 1 {
                    first = true;
                    attr.subscribed = None;
                    attr.notifications.clear();
                    attr.requested.push_back(true);
                }
            });
            // A second channel on the same characteristic shares the first's
            // subscription, and the first's answer: CoreBluetooth keeps one
            // per characteristic.
            if first {
                peripheral.setNotifyValue_forCharacteristic(true, characteristic);
            }
        })?;

        let answer = self.wait_on(channel, REPLY_TIMEOUT, |attr| attr.subscribed.clone());
        let failed = match answer {
            Ok(Some(Ok(()))) => return Ok(()),
            Ok(Some(Err(why))) => BleError::Platform(format!(
                "cannot subscribe to {:032x}: {why}",
                channel.attribute.1
            )),
            Ok(None) => BleError::Platform(format!(
                "subscribing to {:032x} went unanswered",
                channel.attribute.1
            )),
            Err(err) => err,
        };
        self.unsubscribe(channel);
        Err(failed)
    }

    fn unsubscribe(&self, channel: &Channel) {
        let attribute = channel.attribute;
        let key = channel.key.clone();
        let session = channel.session;
        // The count and the request go together on the queue, so a subscribe
        // cannot slip between them and be turned off behind its back.
        let _ = self.with_characteristic(channel, move |peripheral, characteristic, shared| unsafe {
            let mut last = false;
            shared.update(|events| {
                let Some(link) = events.links.get_mut(&key) else {
                    return;
                };
                // A later connection has its own subscriptions, and none of
                // them are this channel's to end.
                if link.session != session || !link.connected {
                    return;
                }
                let attr = link.attributes.entry(attribute).or_default();
                attr.subscribers = attr.subscribers.saturating_sub(1);
                if attr.subscribers == 0 {
                    last = true;
                    attr.requested.push_back(false);
                }
            });
            if last {
                peripheral.setNotifyValue_forCharacteristic(false, characteristic);
            }
        });
    }

    fn read(&self, channel: &Channel) -> Result<Vec<u8>, BleError> {
        let attribute = channel.attribute;
        let key = channel.key.clone();
        self.with_characteristic(channel, move |peripheral, characteristic, shared| unsafe {
            shared.update(|events| {
                let attr = events
                    .links
                    .entry(key)
                    .or_default()
                    .attributes
                    .entry(attribute)
                    .or_default();
                attr.reading = true;
                attr.read = None;
            });
            peripheral.readValueForCharacteristic(characteristic);
        })?;

        match self.wait_on(channel, REPLY_TIMEOUT, |attr| attr.read.take())? {
            Some(Ok(value)) => Ok(value),
            Some(Err(why)) => Err(BleError::Platform(format!("read failed: {why}"))),
            None => Err(BleError::Platform("the read went unanswered".into())),
        }
    }

    fn write_acknowledged(&self, channel: &Channel, value: &[u8]) -> Result<(), BleError> {
        let attribute = channel.attribute;
        let key = channel.key.clone();
        let data = value.to_vec();
        self.with_characteristic(channel, move |peripheral, characteristic, shared| unsafe {
            shared.update(|events| {
                events
                    .links
                    .entry(key)
                    .or_default()
                    .attributes
                    .entry(attribute)
                    .or_default()
                    .written = None;
            });
            peripheral.writeValue_forCharacteristic_type(
                &NSData::from_vec(data),
                characteristic,
                CBCharacteristicWriteType::WithResponse,
            );
        })?;

        match self.wait_on(channel, REPLY_TIMEOUT, |attr| attr.written.take())? {
            Some(Ok(())) => Ok(()),
            Some(Err(why)) => Err(BleError::Platform(format!("write failed: {why}"))),
            None => Err(BleError::Platform("the write went unacknowledged".into())),
        }
    }

    /// One write without response, once CoreBluetooth has room for it.
    ///
    /// CoreBluetooth drops a write-without-response it has no buffer for, with
    /// no error anywhere, so asking first is the only back pressure there is.
    /// A firmware image is thousands of these in a row, and the buffer is a
    /// handful of packets deep.
    fn write_unacknowledged(&self, channel: &Channel, value: &[u8]) -> Result<(), BleError> {
        loop {
            let key = channel.key.clone();
            let data = value.to_vec();
            let sent = self.with_characteristic(channel, move |peripheral, characteristic, shared| unsafe {
                if peripheral.canSendWriteWithoutResponse() {
                    peripheral.writeValue_forCharacteristic_type(
                        &NSData::from_vec(data),
                        characteristic,
                        CBCharacteristicWriteType::WithoutResponse,
                    );
                    return true;
                }
                // Cleared on the queue, so the callback that sets it again
                // cannot have run in between.
                shared.update(|events| {
                    events.links.entry(key).or_default().ready_to_write = false;
                });
                false
            })?;
            if sent {
                return Ok(());
            }

            let ready = self.wait(REPLY_TIMEOUT, |events| {
                let link = events.links.get(&channel.key)?;
                (link.ready_to_write || !link.connected || link.session != channel.session)
                    .then_some(())
            });
            if ready.is_none() {
                return Err(BleError::Platform(
                    "CoreBluetooth stopped taking writes".into(),
                ));
            }
        }
    }

    fn notification(&self, channel: &Channel, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        self.wait_on(channel, timeout, |attr| attr.notifications.pop_front())
    }
}

/// One peripheral, as `sightings` found it on the queue.
struct Found {
    key: String,
    name: Option<String>,
    connected: bool,
    services: Vec<u128>,
}

unsafe fn find_service(peripheral: &CBPeripheral, uuid: u128) -> Option<Retained<CBService>> {
    peripheral
        .services()?
        .iter()
        .find(|service| from_cbuuid(&service.UUID()) == Some(uuid))
}

unsafe fn find_characteristic(service: &CBService, uuid: u128) -> Option<Retained<CBCharacteristic>> {
    service
        .characteristics()?
        .iter()
        .find(|characteristic| from_cbuuid(&characteristic.UUID()) == Some(uuid))
}

/// The handle a sighting carries: the peripheral's identifier.
///
/// Stable for as long as the bond is, and local to this Mac — CoreBluetooth
/// never hands out a Bluetooth address.
fn key_of(peripheral: &CBPeripheral) -> String {
    unsafe { peripheral.identifier() }.UUIDString().to_string()
}

/// Which characteristic a callback is about.
unsafe fn attribute_of(characteristic: &CBCharacteristic) -> Option<Attribute> {
    let service = characteristic.service()?;
    Some((
        from_cbuuid(&service.UUID())?,
        from_cbuuid(&characteristic.UUID())?,
    ))
}

fn cbuuid(uuid: u128) -> Retained<CBUUID> {
    let text = format!(
        "{:08x}-{:04x}-{:04x}-{:04x}-{:012x}",
        uuid >> 96,
        (uuid >> 80) & 0xffff,
        (uuid >> 64) & 0xffff,
        (uuid >> 48) & 0xffff,
        uuid & 0xffff_ffff_ffff
    );
    unsafe { CBUUID::UUIDWithString(&NSString::from_str(&text)) }
}

fn from_cbuuid(uuid: &CBUUID) -> Option<u128> {
    expand_uuid(&unsafe { uuid.UUIDString() }.to_string())
}

/// A UUID as CoreBluetooth spells it, as the integer the id tables use.
///
/// CoreBluetooth shortens anything in the Bluetooth base range to its 16- or
/// 32-bit alias — `180F`, not `0000180F-0000-1000-8000-00805F9B34FB` — so
/// those are put back on the base before parsing.
fn expand_uuid(text: &str) -> Option<u128> {
    const BASE: &str = "-0000-1000-8000-00805f9b34fb";
    match text.len() {
        4 => identity::parse_uuid(&format!("0000{text}{BASE}")),
        8 => identity::parse_uuid(&format!("{text}{BASE}")),
        _ => identity::parse_uuid(text),
    }
}

/// A characteristic's identity, for telling apart ones that share a UUID.
///
/// Takes the object itself rather than anything generic: given a
/// `&Retained<_>`, a generic version would happily return the address of the
/// handle on the stack, the same for every characteristic watched in a loop.
fn object_id(characteristic: &CBCharacteristic) -> usize {
    characteristic as *const CBCharacteristic as usize
}

/// A PnP ID in the modalias form BlueZ derives from the same bytes:
/// `usb:vVVVVpPPPPdDDDD` for a USB vendor id, `bluetooth:…` for a SIG one.
fn modalias_from_pnp(value: &[u8]) -> Option<String> {
    let [source, vid_lo, vid_hi, pid_lo, pid_hi, ver_lo, ver_hi, ..] = *value else {
        return None;
    };
    let kind = match source {
        1 => "bluetooth",
        2 => "usb",
        _ => return None,
    };
    Some(format!(
        "{kind}:v{:04X}p{:04X}d{:04X}",
        u16::from_le_bytes([vid_lo, vid_hi]),
        u16::from_le_bytes([pid_lo, pid_hi]),
        u16::from_le_bytes([ver_lo, ver_hi]),
    ))
}

fn describe(error: Option<&NSError>) -> Option<String> {
    error.map(|error| error.localizedDescription().to_string())
}

struct DelegateIvars {
    shared: Arc<Shared>,
    /// Every peripheral CoreBluetooth has handed over, kept so it can be
    /// connected to: CoreBluetooth cancels a connection to a peripheral nobody
    /// holds a reference to.
    peripherals: RefCell<HashMap<String, Retained<CBPeripheral>>>,
    /// The characteristics a profile reads, by object: battery levels and the
    /// PnP ID. Their values go to the profile rather than to a channel. Held,
    /// so the address an entry is keyed by cannot be handed to another object.
    watched: RefCell<HashMap<usize, Watched>>,
}

struct Watched {
    key: String,
    what: What,
    characteristic: Retained<CBCharacteristic>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum What {
    Level,
    PnpId,
}

define_class!(
    // SAFETY: NSObject has no subclassing requirements, and `Delegate` does
    // not implement `Drop`.
    #[unsafe(super(NSObject))]
    #[name = "ZappBleDelegate"]
    #[ivars = DelegateIvars]
    struct Delegate;

    unsafe impl NSObjectProtocol for Delegate {}

    unsafe impl CBCentralManagerDelegate for Delegate {
        #[unsafe(method(centralManagerDidUpdateState:))]
        fn did_update_state(&self, central: &CBCentralManager) {
            let state = unsafe { central.state() };
            log::debug!("CoreBluetooth central state {}", state.0);
            // Leaving the powered-on state takes every link down with it, but
            // CoreBluetooth sends no didDisconnectPeripheral for them. Left
            // marked connected, the next connect would hand out the dead
            // session and lose its channel to the didConnect that follows.
            let down = state != CBManagerState::PoweredOn;
            if down {
                self.ivars().watched.borrow_mut().clear();
            }
            self.ivars().shared.update(|events| {
                events.power = Some(state.0);
                if down {
                    for link in events.links.values_mut().filter(|link| link.connected) {
                        link.reset(false);
                    }
                }
                events.generation += 1;
            });
        }

        #[unsafe(method(centralManager:didDiscoverPeripheral:advertisementData:RSSI:))]
        fn did_discover(
            &self,
            _central: &CBCentralManager,
            peripheral: &CBPeripheral,
            advertisement: &NSDictionary<NSString, AnyObject>,
            rssi: &NSNumber,
        ) {
            let key = self.remember(peripheral);
            let name = advertisement
                .objectForKey(unsafe { CBAdvertisementDataLocalNameKey })
                .and_then(|name| name.downcast::<NSString>().ok())
                .map(|name| name.to_string());
            let services = advertisement
                .objectForKey(unsafe { CBAdvertisementDataServiceUUIDsKey })
                .and_then(|uuids| uuids.downcast::<NSArray>().ok())
                .map(|uuids| {
                    uuids
                        .iter()
                        .filter_map(|uuid| uuid.downcast::<CBUUID>().ok())
                        .filter_map(|uuid| from_cbuuid(&uuid))
                        .collect()
                })
                .unwrap_or_default();
            // 127 is CoreBluetooth's "not available".
            let rssi = rssi.integerValue();
            log::trace!(
                "advertisement from {key} {name:?} (known as {:?}) rssi {rssi}: {services:032x?}",
                unsafe { peripheral.name() }.map(|name| name.to_string())
            );
            if rssi == 127 {
                return;
            }
            let advert = Advert {
                name,
                services,
                rssi: rssi.clamp(i16::MIN as isize, i16::MAX as isize) as i16,
                seen: Instant::now(),
            };
            self.ivars().shared.update(|events| {
                let changed = events
                    .adverts
                    .get(&key)
                    .is_none_or(|old| old.name != advert.name || old.services != advert.services);
                events.adverts.insert(key, advert);
                if changed {
                    events.generation += 1;
                }
            });
        }

        #[unsafe(method(centralManager:didConnectPeripheral:))]
        fn did_connect(&self, _central: &CBCentralManager, peripheral: &CBPeripheral) {
            let key = key_of(peripheral);
            log::debug!("connected to {key}");
            self.forget_watched(&key);
            self.ivars().shared.update(|events| {
                events.links.entry(key).or_default().reset(true);
                events.generation += 1;
            });
        }

        #[unsafe(method(centralManager:didFailToConnectPeripheral:error:))]
        fn did_fail_to_connect(
            &self,
            _central: &CBCentralManager,
            peripheral: &CBPeripheral,
            error: Option<&NSError>,
        ) {
            let why = describe(error).unwrap_or_else(|| "CoreBluetooth gave no reason".into());
            self.ivars().shared.update(|events| {
                events.links.entry(key_of(peripheral)).or_default().failed = Some(why);
            });
        }

        #[unsafe(method(centralManager:didDisconnectPeripheral:error:))]
        fn did_disconnect(
            &self,
            _central: &CBCentralManager,
            peripheral: &CBPeripheral,
            error: Option<&NSError>,
        ) {
            let key = key_of(peripheral);
            let why = describe(error).unwrap_or_else(|| "disconnected".into());
            log::debug!("{key} disconnected: {why}");
            self.forget_watched(&key);
            self.ivars().shared.update(|events| {
                events.links.entry(key).or_default().reset(false);
                events.generation += 1;
            });
        }
    }

    unsafe impl CBPeripheralDelegate for Delegate {
        #[unsafe(method(peripheral:didDiscoverServices:))]
        fn did_discover_services(&self, peripheral: &CBPeripheral, error: Option<&NSError>) {
            let key = key_of(peripheral);
            let services: Vec<u128> = unsafe { peripheral.services() }
                .map(|services| {
                    services
                        .iter()
                        .filter_map(|service| from_cbuuid(&*unsafe { service.UUID() }))
                        .collect()
                })
                .unwrap_or_default();
            self.ivars().shared.update(|events| {
                events.services.entry(key.clone()).or_default().extend(services);
                events.links.entry(key).or_default().discovered = Some(match describe(error) {
                    Some(why) => Err(why),
                    None => Ok(()),
                });
            });
        }

        #[unsafe(method(peripheral:didDiscoverCharacteristicsForService:error:))]
        fn did_discover_characteristics(
            &self,
            peripheral: &CBPeripheral,
            _service: &CBService,
            error: Option<&NSError>,
        ) {
            self.ivars().shared.update(|events| {
                let link = events.links.entry(key_of(peripheral)).or_default();
                link.answers += 1;
                link.discovered = Some(match describe(error) {
                    Some(why) => Err(why),
                    None => Ok(()),
                });
            });
        }

        #[unsafe(method(peripheral:didDiscoverDescriptorsForCharacteristic:error:))]
        fn did_discover_descriptors(
            &self,
            peripheral: &CBPeripheral,
            characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            log::trace!(
                "descriptors of {:x}: {:?} {:?}",
                object_id(characteristic),
                unsafe { characteristic.descriptors() }.map(|descriptors| descriptors
                    .iter()
                    .map(|descriptor| unsafe { descriptor.UUID().UUIDString() }.to_string())
                    .collect::<Vec<_>>()),
                describe(error)
            );
            self.ivars().shared.update(|events| {
                events.links.entry(key_of(peripheral)).or_default().answers += 1;
            });
        }

        /// Only the presentation format beside a battery level is ever read:
        /// it says which half that instance speaks for.
        #[unsafe(method(peripheral:didUpdateValueForDescriptor:error:))]
        fn did_update_descriptor(
            &self,
            peripheral: &CBPeripheral,
            descriptor: &CBDescriptor,
            error: Option<&NSError>,
        ) {
            let half = (|| unsafe {
                if error.is_some() || from_cbuuid(&descriptor.UUID()) != Some(PRESENTATION_FORMAT) {
                    return None;
                }
                let value = descriptor.value()?.downcast::<NSData>().ok()?.to_vec();
                let half = battery::half_from_format(&value)?;
                Some((object_id(&*descriptor.characteristic()?), half))
            })();
            log::trace!(
                "descriptor {:?} answered: {half:?} {:?}",
                unsafe { descriptor.UUID().UUIDString() },
                describe(error)
            );
            self.ivars().shared.update(|events| {
                let link = events.links.entry(key_of(peripheral)).or_default();
                link.answers += 1;
                if let Some((characteristic, half)) = half {
                    link.halves.insert(characteristic, half);
                }
            });
        }

        /// A read's answer and a notification arrive through this one
        /// callback, and CoreBluetooth gives no way to tell them apart. While a
        /// read is outstanding the next value is taken as its answer — which
        /// is right unless the board notifies in the moment between the read
        /// going out and its response coming back. The update service only
        /// notifies at the end of a step it was told to take, so nothing it
        /// does lands in that gap.
        #[unsafe(method(peripheral:didUpdateValueForCharacteristic:error:))]
        fn did_update_value(
            &self,
            peripheral: &CBPeripheral,
            characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            let value = unsafe { characteristic.value() }
                .map(|data| data.to_vec())
                .unwrap_or_default();
            if self.profile_value(peripheral, characteristic, &value, error.is_some()) {
                return;
            }
            let Some(attribute) = (unsafe { attribute_of(characteristic) }) else {
                return;
            };
            let error = describe(error);
            self.ivars().shared.update(|events| {
                let attr = events
                    .links
                    .entry(key_of(peripheral))
                    .or_default()
                    .attributes
                    .entry(attribute)
                    .or_default();
                if attr.reading {
                    attr.reading = false;
                    attr.read = Some(match error {
                        Some(why) => Err(why),
                        None => Ok(value),
                    });
                } else if let Some(why) = error {
                    log::debug!("notification on {:032x} failed: {why}", attribute.1);
                } else if attr.subscribers > 0 {
                    attr.notifications.push_back(value);
                }
            });
        }

        #[unsafe(method(peripheral:didWriteValueForCharacteristic:error:))]
        fn did_write_value(
            &self,
            peripheral: &CBPeripheral,
            characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            let Some(attribute) = (unsafe { attribute_of(characteristic) }) else {
                return;
            };
            self.ivars().shared.update(|events| {
                events
                    .links
                    .entry(key_of(peripheral))
                    .or_default()
                    .attributes
                    .entry(attribute)
                    .or_default()
                    .written = Some(match describe(error) {
                    Some(why) => Err(why),
                    None => Ok(()),
                });
            });
        }

        #[unsafe(method(peripheral:didUpdateNotificationStateForCharacteristic:error:))]
        fn did_update_notification_state(
            &self,
            peripheral: &CBPeripheral,
            characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            let Some(attribute) = (unsafe { attribute_of(characteristic) }) else {
                return;
            };
            let notifying = unsafe { characteristic.isNotifying() };
            self.ivars().shared.update(|events| {
                let attr = events
                    .links
                    .entry(key_of(peripheral))
                    .or_default()
                    .attributes
                    .entry(attribute)
                    .or_default();
                // The answer to an unsubscribe is nobody's business, and one
                // nothing here asked for even less so.
                if attr.requested.pop_front() != Some(true) {
                    return;
                }
                attr.subscribed = Some(match (describe(error), notifying) {
                    (Some(why), _) => Err(why),
                    (None, true) => Ok(()),
                    (None, false) => Err("the board did not turn notifications on".into()),
                });
            });
        }

        #[unsafe(method(peripheralIsReadyToSendWriteWithoutResponse:))]
        fn ready_to_write(&self, peripheral: &CBPeripheral) {
            self.ivars().shared.update(|events| {
                events.links.entry(key_of(peripheral)).or_default().ready_to_write = true;
            });
        }
    }
);

impl Delegate {
    fn new(shared: Arc<Shared>) -> Retained<Self> {
        let this = Self::alloc().set_ivars(DelegateIvars {
            shared,
            peripherals: RefCell::new(HashMap::new()),
            watched: RefCell::new(HashMap::new()),
        });
        // SAFETY: NSObject's designated initialiser.
        unsafe { msg_send![super(this), init] }
    }

    /// Holds on to a peripheral, and returns its key.
    fn remember(&self, peripheral: &CBPeripheral) -> String {
        let key = key_of(peripheral);
        self.ivars()
            .peripherals
            .borrow_mut()
            .entry(key.clone())
            .or_insert_with(|| peripheral.retain());
        key
    }

    fn watch(&self, key: &str, characteristic: &Retained<CBCharacteristic>, what: What) {
        self.ivars().watched.borrow_mut().insert(
            object_id(characteristic),
            Watched {
                key: key.to_string(),
                what,
                characteristic: characteristic.clone(),
            },
        );
    }

    /// A connection's characteristic objects die with it.
    fn forget_watched(&self, key: &str) {
        self.ivars().watched.borrow_mut().retain(|_, watched| watched.key != key);
    }

    /// What a profile watches on one board.
    fn watched(&self, key: &str) -> Vec<(What, Retained<CBCharacteristic>)> {
        self.ivars()
            .watched
            .borrow()
            .values()
            .filter(|watched| watched.key == key)
            .map(|watched| (watched.what, watched.characteristic.clone()))
            .collect()
    }

    /// Takes a value meant for the profile, and says whether it was one.
    ///
    /// A battery level arrives both as the answer to the profile's read and,
    /// from then on, as a notification; either way it is the latest level.
    fn profile_value(
        &self,
        peripheral: &CBPeripheral,
        characteristic: &CBCharacteristic,
        value: &[u8],
        failed: bool,
    ) -> bool {
        let id = object_id(characteristic);
        let Some(what) = self.ivars().watched.borrow().get(&id).map(|watched| watched.what) else {
            return false;
        };
        let key = key_of(peripheral);
        log::trace!("{key} profile value {id:x}: {value:02x?} (failed: {failed})");
        self.ivars().shared.update(|events| {
            match (what, failed) {
                (_, true) => {}
                (What::Level, false) => {
                    let link = events.links.entry(key.clone()).or_default();
                    if let Some(raw) = value.first() {
                        if link.levels.insert(id, *raw) != Some(*raw) {
                            events.generation += 1;
                        }
                    }
                }
                (What::PnpId, false) => {
                    if let Some(modalias) = modalias_from_pnp(value) {
                        events.modalias.insert(key.clone(), modalias);
                        events.generation += 1;
                    }
                }
            }
            events.links.entry(key).or_default().answers += 1;
        });
        true
    }

    /// A board's answer counter, read on the queue.
    fn answers(&self, key: &str) -> u64 {
        self.ivars()
            .shared
            .events
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .links
            .get(key)
            .map_or(0, |link| link.answers)
    }

    fn knows(&self, key: &str) -> bool {
        self.ivars().peripherals.borrow().contains_key(key)
    }

    fn peripheral(&self, key: &str) -> Option<Retained<CBPeripheral>> {
        self.ivars().peripherals.borrow().get(key).cloned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pnp_id_reads_as_the_modalias_bluez_would_have_made_of_it() {
        // USB vendor source, 0x3297:0x1978, version 0x0100: a Wireless Voyager.
        let pnp = [0x02, 0x97, 0x32, 0x78, 0x19, 0x00, 0x01];
        let modalias = modalias_from_pnp(&pnp).unwrap();
        assert_eq!(modalias, "usb:v3297p1978d0100");
        assert_eq!(
            identity::keyboard_from_modalias(&modalias),
            Some(zapp_core::device::ids::Keyboard::WirelessVoyager)
        );
        assert_eq!(
            modalias_from_pnp(&[0x01, 0x97, 0x32, 0x78, 0x19, 0x00, 0x01]).as_deref(),
            Some("bluetooth:v3297p1978d0100")
        );
        assert_eq!(modalias_from_pnp(&pnp[..6]), None, "short");
        assert_eq!(modalias_from_pnp(&[0x03, 0, 0, 0, 0, 0, 0]), None, "unknown source");
    }

    #[test]
    fn a_short_uuid_is_put_back_on_the_bluetooth_base() {
        assert_eq!(
            expand_uuid("180F"),
            Some(0x0000180f_0000_1000_8000_00805f9b34fb)
        );
        assert_eq!(
            expand_uuid("0000180F"),
            Some(0x0000180f_0000_1000_8000_00805f9b34fb)
        );
    }

    #[test]
    fn a_full_uuid_is_taken_as_it_is() {
        assert_eq!(
            expand_uuid("32970001-E5AA-4F42-A596-1EA5A0200001"),
            Some(ZSA_BLE_DFU_SERVICE)
        );
        assert_eq!(expand_uuid("not a uuid"), None);
    }

    #[test]
    fn a_uuid_survives_the_trip_through_corebluetooth() {
        for uuid in [ZSA_BLE_DFU_SERVICE, SMP_SERVICE, RELAY_SERVICE, 0x0000180f_0000_1000_8000_00805f9b34fb] {
            assert_eq!(from_cbuuid(&cbuuid(uuid)), Some(uuid), "{uuid:032x}");
        }
    }
}
