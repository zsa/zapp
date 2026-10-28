//! WinRT.
//!
//! Windows answers presence from its device database, which already knows
//! everything about a bonded board without the radio being asked. Every GATT
//! service a board carried when it was paired is a device interface of its own,
//! present whether or not the board is in range, and each one carries the
//! board's address and the PnP ID it read from Device Information — so one
//! query names every bonded ZSA board, lists its services and says which
//! product it is, without connecting to anything. The pairing selector adds
//! the names and the handles, a `BluetoothLEDevice` held per board says whether
//! it is connected, and a passive advertisement watcher finds the boards in
//! range that were never bonded.
//!
//! WinRT is asynchronous and this crate is blocking, so every operation is
//! waited on where it is made, with a deadline of ours: a GATT call to a board
//! out of range would otherwise sit for as long as Windows feels like. The
//! callbacks — advertisements, connection changes, notifications — run on the
//! thread pool and turn what they were handed into plain Rust data under one
//! mutex, which a caller waits on through the condvar beside it.
//!
//! WinRT needs an apartment on every thread that calls it. The calling threads
//! here are whoever the caller is — a monitor thread, a session pump, a tokio
//! worker — so rather than initialise each one, the process keeps the
//! multithreaded apartment alive once, and every thread that has not joined an
//! apartment of its own is in it.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock, Weak};
use std::thread;
use std::time::{Duration, Instant};

use windows::core::{IInspectable, Interface, GUID, HSTRING};
use windows::Devices::Bluetooth::Advertisement::{
    BluetoothLEAdvertisementReceivedEventArgs, BluetoothLEAdvertisementWatcher,
    BluetoothLEScanningMode,
};
use windows::Devices::Bluetooth::GenericAttributeProfile::{
    GattCharacteristic, GattClientCharacteristicConfigurationDescriptorValue,
    GattCommunicationStatus, GattDeviceService, GattSession, GattSharingMode,
    GattValueChangedEventArgs, GattWriteOption,
};
use windows::Devices::Bluetooth::{
    BluetoothAdapter, BluetoothCacheMode, BluetoothConnectionStatus, BluetoothLEDevice,
};
use windows::Devices::Enumeration::{DeviceInformation, DeviceInformationKind};
use windows::Devices::Radios::RadioState;
use windows::Foundation::{IReference, TypedEventHandler};
use windows::Storage::Streams::{DataReader, DataWriter, IBuffer};
use windows::Win32::System::Com::CoIncrementMTAUsage;
use windows_collections::IIterable;
use windows_future::IAsyncOperation;

use super::{Backend, GattChannel, Usage};
use crate::battery::{
    self, Battery, ChargeDetail, Half, Level, BATTERY_LEVEL, BATTERY_SERVICE, CHARGE_DETAIL,
    PRESENTATION_FORMAT,
};
use crate::{identity, BleError, BleSighting};

/// How long the radio has to answer a query of the device database. Local, so
/// anything near this is a radio stack that has stopped answering.
const QUERY_TIMEOUT: Duration = Duration::from_secs(10);

/// How long to wait for a link, the same as BlueZ's and CoreBluetooth's: long
/// enough to wait out a bonded board's advertising interval.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);

/// How long a read, an acknowledged write or a subscription may go unanswered.
///
/// The update service erases the STM32 inside the START write's own handler, so
/// that acknowledgement can be seconds coming. The same 25 seconds D-Bus allows
/// a method call.
const REPLY_TIMEOUT: Duration = Duration::from_secs(25);

/// The discovery window, the same rhythm as the other backends and for the same
/// reason: a scanning radio is airtime the links do not get.
const SCAN_WINDOW: Duration = Duration::from_secs(4);
const SCAN_EVERY: Duration = Duration::from_secs(30);

/// How long an advertisement counts as "in range". Windows says nothing when a
/// device goes quiet, so freshness is ours to decide, and long enough to
/// bridge two windows so a board that has not moved does not flicker.
const ADVERT_FRESH: Duration =
    Duration::from_secs(SCAN_EVERY.as_secs() + 2 * SCAN_WINDOW.as_secs());

/// The payload of the 23-byte ATT MTU every link starts on, and how long to
/// give a fresh link to negotiate past it.
const DEFAULT_PAYLOAD: usize = 20;
const MTU_SETTLE: Duration = Duration::from_secs(2);

/// What an ATT write costs on top of its payload: opcode and handle.
const ATT_HEADER: usize = 3;

/// How long to let a burst of callbacks finish before reading the state again.
const SETTLE: Duration = Duration::from_millis(250);

/// `GUID_BLUETOOTH_GATT_SERVICE_DEVICE_INTERFACE`: one interface per service
/// per bonded device, kept while the device is out of range.
const GATT_SERVICE_INTERFACE: &str = "{6e3bb679-4372-40c8-9eaa-4509df260cd8}";

/// What each GATT service interface says about the board it belongs to.
const SERVICE_ADDRESS: &str = "System.DeviceInterface.Bluetooth.DeviceAddress";
const SERVICE_UUID: &str = "System.DeviceInterface.Bluetooth.ServiceGuid";
const VENDOR_SOURCE: &str = "System.DeviceInterface.Bluetooth.VendorIdSource";
const VENDOR: &str = "System.DeviceInterface.Bluetooth.VendorId";
const PRODUCT: &str = "System.DeviceInterface.Bluetooth.ProductId";
const PRODUCT_VERSION: &str = "System.DeviceInterface.Bluetooth.ProductVersion";

/// The address an association endpoint — what the pairing selector returns —
/// is for.
const ENDPOINT_ADDRESS: &str = "System.Devices.Aep.DeviceAddress";

/// The backend: the process's one radio, plus this backend's scan rhythm.
pub struct WinRt {
    radio: Result<&'static Radio, BleError>,
    scanning_since: Option<Instant>,
    scanned_at: Option<Instant>,
    /// The generation `wait_for_change` last saw.
    seen: u64,
}

impl WinRt {
    /// Never fails: a radio that cannot be had — no adapter, Bluetooth off — is
    /// reported by `sightings` instead, so the reason reaches the person.
    pub fn new() -> Self {
        Self {
            radio: Radio::get(),
            scanning_since: None,
            scanned_at: None,
            seen: 0,
        }
    }

    fn radio(&self) -> Result<&'static Radio, BleError> {
        let radio = self.radio.as_ref().map_err(clone_error)?;
        radio.ready()?;
        Ok(radio)
    }

    fn pump_discovery(&mut self, radio: &Radio) {
        match self.scanning_since {
            Some(started) if started.elapsed() >= SCAN_WINDOW => {
                radio.scan(false);
                self.scanning_since = None;
                self.scanned_at = Some(Instant::now());
            }
            Some(_) => {}
            None => {
                if self
                    .scanned_at
                    .is_none_or(|ended| ended.elapsed() >= SCAN_EVERY)
                {
                    radio.scan(true);
                    self.scanning_since = Some(Instant::now());
                }
            }
        }
    }
}

impl Backend for WinRt {
    fn sightings(&mut self) -> Result<Vec<BleSighting>, BleError> {
        let radio = self.radio()?;
        self.pump_discovery(radio);
        let sightings = radio.sightings()?;

        // What BlueZ keeps for a bonded board's levels Windows keeps to its own
        // battery client, so a connected board is asked once per connection.
        // After that its levels arrive as notifications.
        let fresh: Vec<&str> = sightings
            .iter()
            .filter(|sighting| sighting.connected && identity::is_zsa_board(sighting))
            .map(|sighting| sighting.key.as_str())
            .filter(|key| !radio.profiled(key))
            .collect();
        if fresh.is_empty() {
            return Ok(sightings);
        }
        for key in fresh {
            radio.profile(key);
        }
        radio.sightings()
    }

    fn wait_for_change(&mut self, timeout: Duration) {
        let Ok(radio) = &self.radio else {
            thread::sleep(timeout);
            return;
        };
        let seen = self.seen;
        let changed = radio.shared.wait(timeout, |state| {
            (state.generation != seen).then_some(state.generation)
        });
        if let Some(generation) = changed {
            thread::sleep(SETTLE);
            self.seen = radio.shared.state().generation.max(generation);
        }
    }

    fn stop_discovery(&mut self) {
        if self.scanning_since.take().is_none() {
            return;
        }
        if let Ok(radio) = &self.radio {
            radio.scan(false);
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
        let radio = self.radio()?;
        let link = radio.link(key)?;
        let found = link.characteristic(service, characteristic)?;
        let mtu = link.mtu()?;
        log::debug!("characteristic {characteristic:032x} on {key}, {mtu} bytes per write");

        let session = link.session_id();
        let mut channel = Channel {
            link,
            characteristic: found,
            usage,
            mtu,
            session,
            notifications: None,
        };
        if usage.notify {
            channel.subscribe()?;
        }
        Ok(Box::new(channel))
    }
}

/// What one board's charge-detail characteristic says, read now.
///
/// The same one-shot as the other backends': every way it can come to nothing —
/// Bluetooth off, a board that is not connected, firmware without the
/// characteristic, a read that failed — is `None`.
pub(super) fn read_charge_detail(key: &str) -> Option<ChargeDetail> {
    let radio = Radio::get().ok()?;
    radio.ready().ok()?;
    if !radio.connected(key) {
        return None;
    }
    let link = radio.link(key).ok()?;
    let characteristic = match link.find_anywhere(CHARGE_DETAIL) {
        Ok(Some(characteristic)) => characteristic,
        Ok(None) => {
            log::debug!("{key} publishes no charge detail characteristic");
            return None;
        }
        Err(err) => {
            log::debug!("cannot look for {key}'s charge detail: {err}");
            return None;
        }
    };
    match read_value(&characteristic) {
        Ok(value) => ChargeDetail::decode(&value),
        Err(err) => {
            log::debug!("cannot read {key}'s charge detail: {err}");
            None
        }
    }
}

/// One characteristic on one board.
///
/// Holds the link it was opened under, which keeps the connection up for as
/// long as any channel on it is open, and the connection it was opened on —
/// a channel that outlives its connection says so rather than writing into a
/// characteristic Windows has already thrown away.
struct Channel {
    link: Arc<Link>,
    characteristic: GattCharacteristic,
    usage: Usage,
    mtu: usize,
    session: u64,
    /// The notifications, and the registration that delivers them.
    notifications: Option<(Receiver<Vec<u8>>, i64)>,
}

impl Channel {
    /// Delivers notifications to this channel, then asks the board for them.
    ///
    /// The handler goes on first so that nothing the board says straight after
    /// the descriptor write can be missed.
    fn subscribe(&mut self) -> Result<(), BleError> {
        let (tx, rx) = mpsc::channel();
        let shared = Arc::clone(&self.link.shared);
        let token = self
            .characteristic
            .ValueChanged(&TypedEventHandler::<
                GattCharacteristic,
                GattValueChangedEventArgs,
            >::new(move |_, args| {
                if let Some(args) = args.as_ref() {
                    let value = bytes(&args.CharacteristicValue()?)?;
                    log::trace!("notified {value:02x?}");
                    let _ = tx.send(value);
                    shared.notify();
                }
                Ok(())
            }))
            .map_err(platform)?;
        self.notifications = Some((rx, token));

        let value = if self
            .characteristic
            .CharacteristicProperties()
            .map_err(platform)?
            .contains(windows::Devices::Bluetooth::GenericAttributeProfile::GattCharacteristicProperties::Notify)
        {
            GattClientCharacteristicConfigurationDescriptorValue::Notify
        } else {
            GattClientCharacteristicConfigurationDescriptorValue::Indicate
        };
        let written = wait(
            self.characteristic
                .WriteClientCharacteristicConfigurationDescriptorWithResultAsync(value)
                .map_err(platform)?,
            REPLY_TIMEOUT,
            "subscribing",
        )?;
        status(written.Status().map_err(platform)?, "subscribing")
    }

    fn alive(&self) -> Result<(), BleError> {
        match self.link.connected.load(Ordering::SeqCst) && self.link.session_id() == self.session {
            true => Ok(()),
            false => Err(BleError::Platform("the board closed the link".into())),
        }
    }
}

impl GattChannel for Channel {
    fn mtu(&self) -> usize {
        self.mtu
    }

    fn read(&mut self) -> Result<Vec<u8>, BleError> {
        self.alive()?;
        read_value(&self.characteristic)
    }

    fn write(&mut self, value: &[u8]) -> Result<(), BleError> {
        // Windows would turn a long acknowledged write into a prepared write a
        // characteristic like SMP's does not take, and refuses an oversized
        // unacknowledged one. Neither is a write that happened.
        if value.len() > self.mtu {
            return Err(BleError::Platform(format!(
                "a {}-byte write will not fit the {}-byte ATT payload",
                value.len(),
                self.mtu
            )));
        }
        self.alive()?;
        let option = match self.usage.acknowledged {
            true => GattWriteOption::WriteWithResponse,
            false => GattWriteOption::WriteWithoutResponse,
        };
        log::trace!("writing {value:02x?}");
        let written = wait(
            self.characteristic
                .WriteValueWithResultAndOptionAsync(&buffer(value)?, option)
                .map_err(platform)?,
            REPLY_TIMEOUT,
            "writing",
        )?;
        status(written.Status().map_err(platform)?, "writing")
    }

    fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        let Some((notifications, _)) = &self.notifications else {
            thread::sleep(timeout);
            return Ok(None);
        };
        // Whatever arrived before the link went is still the board's to say.
        if let Ok(value) = notifications.try_recv() {
            return Ok(Some(value));
        }
        self.alive()?;
        // Woken early by a link that goes down, as well as by a value.
        let deadline = Instant::now() + timeout;
        loop {
            let step = deadline
                .saturating_duration_since(Instant::now())
                .min(Duration::from_millis(100));
            match notifications.recv_timeout(step) {
                Ok(value) => return Ok(Some(value)),
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(BleError::Platform("the board closed the link".into()))
                }
                Err(RecvTimeoutError::Timeout) if Instant::now() >= deadline => return Ok(None),
                Err(RecvTimeoutError::Timeout) => self.alive()?,
            }
        }
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        let Some((_, token)) = self.notifications.take() else {
            return;
        };
        let _ = self.characteristic.RemoveValueChanged(token);
        if !self.link.connected.load(Ordering::SeqCst) {
            return;
        }
        // Best effort: the board forgets the subscription at the next
        // connection anyway, and a channel being dropped is no place to fail.
        if let Ok(op) = self
            .characteristic
            .WriteClientCharacteristicConfigurationDescriptorWithResultAsync(
                GattClientCharacteristicConfigurationDescriptorValue::None,
            )
        {
            let _ = wait(op, REPLY_TIMEOUT, "unsubscribing");
        }
    }
}

/// An open GATT link to one board, shared by every channel on it.
///
/// The session is asked to maintain the connection, which is what keeps a
/// board Windows would otherwise drop between two writes connected for the
/// length of an update. Dropping the last channel drops the link, and the
/// connection goes back to whoever else holds it — the HID driver, typically.
struct Link {
    key: String,
    shared: Arc<Shared>,
    device: BluetoothLEDevice,
    session: GattSession,
    /// The services opened on this link, so each is opened once.
    services: Mutex<HashMap<u128, GattDeviceService>>,
    connected: AtomicBool,
    /// Counts connections, so a channel can tell its own from a later one.
    sessions: AtomicU64,
    token: i64,
}

impl Link {
    fn session_id(&self) -> u64 {
        self.sessions.load(Ordering::SeqCst)
    }

    /// The service, opened for shared use, or `Missing` for a board without
    /// it.
    fn service(&self, uuid: u128, characteristic: u128) -> Result<GattDeviceService, BleError> {
        if let Some(service) = self.services.lock().unwrap().get(&uuid) {
            return Ok(service.clone());
        }
        // Uncached: this is the call that brings the link up, and the one that
        // finds out whether the firmware on the other end carries the service.
        let result = wait(
            self.device
                .GetGattServicesForUuidWithCacheModeAsync(
                    GUID::from_u128(uuid),
                    BluetoothCacheMode::Uncached,
                )
                .map_err(platform)?,
            CONNECT_TIMEOUT,
            "connecting",
        )?;
        status(result.Status().map_err(platform)?, "connecting")?;
        let service = result
            .Services()
            .map_err(platform)?
            .into_iter()
            .next()
            .ok_or_else(|| BleError::Missing {
                device: self.key.clone(),
                characteristic,
            })?;
        let opened = wait(
            service
                .OpenAsync(GattSharingMode::SharedReadAndWrite)
                .map_err(platform)?,
            REPLY_TIMEOUT,
            "opening the service",
        )?;
        log::debug!("service {uuid:032x} on {}: {opened:?}", self.key);
        self.services.lock().unwrap().insert(uuid, service.clone());
        Ok(service)
    }

    fn characteristic(
        &self,
        service: u128,
        characteristic: u128,
    ) -> Result<GattCharacteristic, BleError> {
        let opened = self.service(service, characteristic)?;
        let result = wait(
            opened
                .GetCharacteristicsForUuidWithCacheModeAsync(
                    GUID::from_u128(characteristic),
                    BluetoothCacheMode::Uncached,
                )
                .map_err(platform)?,
            REPLY_TIMEOUT,
            "discovering characteristics",
        )?;
        status(
            result.Status().map_err(platform)?,
            "discovering characteristics",
        )?;
        let found = result
            .Characteristics()
            .map_err(platform)?
            .into_iter()
            .next()
            .ok_or_else(|| BleError::Missing {
                device: self.key.clone(),
                characteristic,
            })?;
        // The link is up now, whatever the connection callback has or has not
        // said yet.
        if !self.connected.swap(true, Ordering::SeqCst) {
            self.sessions.fetch_add(1, Ordering::SeqCst);
        }
        Ok(found)
    }

    /// The first characteristic of this UUID under any service — for one like
    /// the charge detail, whose service UUID is the firmware's business.
    fn find_anywhere(&self, characteristic: u128) -> Result<Option<GattCharacteristic>, BleError> {
        let result = wait(
            self.device
                .GetGattServicesWithCacheModeAsync(BluetoothCacheMode::Uncached)
                .map_err(platform)?,
            CONNECT_TIMEOUT,
            "connecting",
        )?;
        status(result.Status().map_err(platform)?, "connecting")?;
        for service in result.Services().map_err(platform)? {
            let found = wait(
                service
                    .GetCharacteristicsForUuidWithCacheModeAsync(
                        GUID::from_u128(characteristic),
                        BluetoothCacheMode::Cached,
                    )
                    .map_err(platform)?,
                REPLY_TIMEOUT,
                "discovering characteristics",
            )?;
            if found.Status().map_err(platform)? != GattCommunicationStatus::Success {
                continue;
            }
            if let Some(found) = found
                .Characteristics()
                .map_err(platform)?
                .into_iter()
                .next()
            {
                return Ok(Some(found));
            }
        }
        Ok(None)
    }

    /// The ATT payload one write can carry.
    fn mtu(&self) -> Result<usize, BleError> {
        let deadline = Instant::now() + MTU_SETTLE;
        loop {
            let pdu = self.session.MaxPduSize().map_err(platform)? as usize;
            let payload = pdu.saturating_sub(ATT_HEADER);
            match payload {
                // The exchange may still be under way on a link this process
                // just brought up, and until it lands Windows reports the
                // 23-byte default. A system-held link settled long ago.
                payload if payload <= DEFAULT_PAYLOAD && Instant::now() < deadline => {
                    thread::sleep(Duration::from_millis(100));
                }
                0 => {
                    return Err(BleError::Platform(format!(
                        "Windows reported a {pdu}-byte PDU, which cannot carry anything"
                    )))
                }
                payload => return Ok(payload),
            }
        }
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        let _ = self.device.RemoveConnectionStatusChanged(self.token);
        let _ = self.session.SetMaintainConnection(false);
        for service in self.services.get_mut().unwrap().values() {
            let _ = service.Close();
        }
        let _ = self.session.Close();
    }
}

/// Everything the callbacks have learnt, as plain data.
#[derive(Default)]
struct State {
    /// Bumped on anything that could change a sighting.
    generation: u64,
    /// The last advertisement from each address, ZSA or not: an advert carries
    /// no bond, so which of them is a board is only known later.
    adverts: HashMap<u64, Advert>,
    /// Each bonded board's battery, as profiled on its current connection.
    profiles: HashMap<String, Profile>,
}

#[derive(Debug, Clone)]
struct Advert {
    name: Option<String>,
    services: Vec<u128>,
    rssi: i16,
    seen: Instant,
}

/// What a connected board's Battery Service instances say.
///
/// Holds the characteristics it subscribed to: dropping them would drop the
/// subscriptions with them.
struct Profile {
    /// The connection it was read on. A board that reconnects is profiled
    /// again, since its levels went with the old link.
    session: u64,
    halves: Vec<(GattCharacteristic, Half, i64)>,
    levels: HashMap<Half, u8>,
}

impl Profile {
    fn battery(&self) -> Battery {
        let mut battery = Battery::default();
        for (_, half, _) in &self.halves {
            let level = self
                .levels
                .get(half)
                .map_or(Level::Unknown, |raw| Level::from_raw(*raw));
            battery.set(*half, level);
        }
        battery
    }
}

impl Drop for Profile {
    fn drop(&mut self) {
        for (characteristic, _, token) in &self.halves {
            let _ = characteristic.RemoveValueChanged(*token);
        }
    }
}

struct Shared {
    state: Mutex<State>,
    changed: Condvar,
}

impl Shared {
    fn state(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn update(&self, f: impl FnOnce(&mut State)) {
        let mut state = self.state();
        f(&mut state);
        state.generation += 1;
        drop(state);
        self.changed.notify_all();
    }

    /// Wakes whoever waits on a channel, without anything having changed.
    fn notify(&self) {
        self.changed.notify_all();
    }

    fn wait<T>(
        &self,
        timeout: Duration,
        mut check: impl FnMut(&mut State) -> Option<T>,
    ) -> Option<T> {
        let deadline = Instant::now() + timeout;
        let mut state = self.state();
        loop {
            if let Some(found) = check(&mut state) {
                return Some(found);
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return None;
            }
            state = self
                .changed
                .wait_timeout(state, left)
                .unwrap_or_else(|poison| poison.into_inner())
                .0;
        }
    }
}

/// A bonded board, held so its connection state can be read and followed.
struct Tracked {
    device: BluetoothLEDevice,
    token: i64,
    /// Counts connections, the way a link does.
    sessions: Arc<AtomicU64>,
}

impl Drop for Tracked {
    fn drop(&mut self) {
        let _ = self.device.RemoveConnectionStatusChanged(self.token);
    }
}

/// The process's view of the radio: one per process, because the watcher and
/// the links outlive any one backend — an update opens a backend per step, and
/// each would otherwise lose every link the last one had up.
struct Radio {
    shared: Arc<Shared>,
    adapter: BluetoothAdapter,
    watcher: BluetoothLEAdvertisementWatcher,
    /// The bonded ZSA boards, by key.
    tracked: Mutex<HashMap<String, Tracked>>,
    /// The open links, by key. Weak: a link lives as long as its channels.
    links: Mutex<HashMap<String, Weak<Link>>>,
}

static RADIO: OnceLock<Result<Radio, String>> = OnceLock::new();

impl Radio {
    fn get() -> Result<&'static Radio, BleError> {
        match RADIO.get_or_init(Radio::start) {
            Ok(radio) => Ok(radio),
            Err(err) => Err(BleError::Platform(err.clone())),
        }
    }

    fn start() -> Result<Radio, String> {
        apartment();
        let adapter = wait(
            BluetoothAdapter::GetDefaultAsync().map_err(|err| err.message())?,
            QUERY_TIMEOUT,
            "finding the adapter",
        )
        .map_err(|_| "no Bluetooth adapter".to_string())?;

        let shared = Arc::new(Shared {
            state: Mutex::new(State::default()),
            changed: Condvar::new(),
        });

        // Passive: the product identity sits in the advertising payload
        // itself, not the scan response, so nothing needs to be asked of a
        // board to recognise it.
        let watcher = BluetoothLEAdvertisementWatcher::new().map_err(|err| err.message())?;
        watcher
            .SetScanningMode(BluetoothLEScanningMode::Passive)
            .map_err(|err| err.message())?;
        let received = Arc::clone(&shared);
        watcher
            .Received(&TypedEventHandler::<
                BluetoothLEAdvertisementWatcher,
                BluetoothLEAdvertisementReceivedEventArgs,
            >::new(move |_, args| {
                if let Some(args) = args.as_ref() {
                    on_advert(&received, args);
                }
                Ok(())
            }))
            .map_err(|err| err.message())?;

        Ok(Radio {
            shared,
            adapter,
            watcher,
            tracked: Mutex::new(HashMap::new()),
            links: Mutex::new(HashMap::new()),
        })
    }

    /// Whether the radio is there and on.
    fn ready(&self) -> Result<(), BleError> {
        apartment();
        let radio = wait(
            self.adapter.GetRadioAsync().map_err(platform)?,
            QUERY_TIMEOUT,
            "reading the radio",
        )?;
        match radio.State().map_err(platform)? {
            RadioState::On => Ok(()),
            RadioState::Off => Err(BleError::Platform("Bluetooth is switched off".into())),
            RadioState::Disabled => Err(BleError::Platform("Bluetooth is disabled".into())),
            _ => Err(BleError::Platform(
                "Bluetooth is in an unknown state".into(),
            )),
        }
    }

    fn scan(&self, on: bool) {
        let result = match on {
            true => self.watcher.Start(),
            false => self.watcher.Stop(),
        };
        if let Err(err) = result {
            log::debug!(
                "cannot {} the advertisement watcher: {err}",
                if on { "start" } else { "stop" }
            );
        }
    }

    /// Every bonded board and every fresh advertisement, as sightings.
    fn sightings(&self) -> Result<Vec<BleSighting>, BleError> {
        apartment();
        let services = service_interfaces()?;
        let bonded = bonded_devices()?;
        let adapter = self.adapter.BluetoothAddress().map_err(platform)?;

        let mut state_adverts = {
            let mut state = self.shared.state();
            state
                .adverts
                .retain(|_, advert| advert.seen.elapsed() < ADVERT_FRESH);
            state.adverts.clone()
        };

        let mut sightings = Vec::new();
        let mut keys = HashSet::new();
        for Bonded { key, name, address } in bonded {
            let interfaces = address.and_then(|address| services.get(&address));
            let advert = address.and_then(|address| state_adverts.remove(&address));
            let mut sighting = BleSighting {
                key: key.clone(),
                name: name.or_else(|| advert.as_ref().and_then(|advert| advert.name.clone())),
                services: interfaces
                    .map(|found| found.services.clone())
                    .unwrap_or_default(),
                advertised: advert
                    .as_ref()
                    .map(|advert| advert.services.clone())
                    .unwrap_or_default(),
                modalias: interfaces.and_then(|found| found.modalias.clone()),
                paired: true,
                rssi: advert.map(|advert| advert.rssi),
                ..BleSighting::default()
            };
            // Only a board is worth holding a device for: Windows bonds with
            // everything from headphones to the mouse.
            if identity::is_zsa_board(&sighting) {
                sighting.connected = self.track(&key);
                sighting.battery = self.battery(&key, sighting.connected);
                keys.insert(key);
            }
            sightings.push(sighting);
        }

        // Boards no longer bonded go, and take their profile with them.
        self.tracked
            .lock()
            .unwrap()
            .retain(|key, _| keys.contains(key));
        self.shared
            .state()
            .profiles
            .retain(|key, _| keys.contains(key));

        // Advertising but never bonded: in range, and nothing more to know.
        for (address, advert) in state_adverts {
            sightings.push(BleSighting {
                key: device_key(adapter, address),
                name: advert.name,
                advertised: advert.services,
                rssi: Some(advert.rssi),
                ..BleSighting::default()
            });
        }
        Ok(sightings)
    }

    /// Holds a bonded board's device and says whether it is connected.
    fn track(&self, key: &str) -> bool {
        let mut tracked = self.tracked.lock().unwrap();
        if !tracked.contains_key(key) {
            match self.start_tracking(key) {
                Ok(device) => {
                    tracked.insert(key.to_string(), device);
                }
                Err(err) => {
                    log::debug!("cannot follow {key}: {err}");
                    return false;
                }
            }
        }
        tracked[key]
            .device
            .ConnectionStatus()
            .is_ok_and(|status| status == BluetoothConnectionStatus::Connected)
    }

    fn start_tracking(&self, key: &str) -> Result<Tracked, BleError> {
        let device = device(key)?;
        let sessions = Arc::new(AtomicU64::new(0));
        let changed = Arc::clone(&self.shared);
        let counted = Arc::clone(&sessions);
        let owner = key.to_string();
        let token = device
            .ConnectionStatusChanged(&TypedEventHandler::<BluetoothLEDevice, IInspectable>::new(
                move |device, _| {
                    let connected = device
                        .as_ref()
                        .and_then(|device| device.ConnectionStatus().ok())
                        == Some(BluetoothConnectionStatus::Connected);
                    log::debug!(
                        "{owner} {}",
                        if connected {
                            "connected"
                        } else {
                            "disconnected"
                        }
                    );
                    if connected {
                        counted.fetch_add(1, Ordering::SeqCst);
                    }
                    changed.update(|state| {
                        if !connected {
                            state.profiles.remove(&owner);
                        }
                    });
                    Ok(())
                },
            ))
            .map_err(platform)?;
        Ok(Tracked {
            device,
            token,
            sessions,
        })
    }

    fn connected(&self, key: &str) -> bool {
        self.track(key)
    }

    /// The battery a board's profile holds, if it is connected and has one.
    fn battery(&self, key: &str, connected: bool) -> Battery {
        if !connected {
            return Battery::default();
        }
        self.shared
            .state()
            .profiles
            .get(key)
            .map(Profile::battery)
            .unwrap_or_default()
    }

    fn session(&self, key: &str) -> u64 {
        self.tracked
            .lock()
            .unwrap()
            .get(key)
            .map_or(0, |tracked| tracked.sessions.load(Ordering::SeqCst))
    }

    fn profiled(&self, key: &str) -> bool {
        let session = self.session(key);
        self.shared
            .state()
            .profiles
            .get(key)
            .is_some_and(|profile| profile.session == session)
    }

    /// Reads a connected board's battery levels, and subscribes to them so they
    /// stay current.
    ///
    /// Best effort: whatever does not answer is left unknown, and the attempt
    /// is not repeated until the next connection — a board that will not say is
    /// not asked again every few seconds.
    fn profile(&self, key: &str) {
        let started = Instant::now();
        let session = self.session(key);
        let halves = match self.read_halves(key) {
            Ok(halves) => halves,
            Err(err) => {
                log::debug!("cannot profile {key}: {err}");
                Vec::new()
            }
        };
        let levels = halves
            .iter()
            .filter_map(|(characteristic, half)| {
                let level = read_value(characteristic).ok()?;
                Some((*half, *level.first()?))
            })
            .collect();

        let mut subscribed = Vec::new();
        for (characteristic, half) in halves {
            match self.follow_level(key, &characteristic, half) {
                Ok(token) => subscribed.push((characteristic, half, token)),
                Err(err) => log::debug!("cannot follow {key}'s {half:?} level: {err}"),
            }
        }
        self.shared.update(|state| {
            state.profiles.insert(
                key.to_string(),
                Profile {
                    session,
                    halves: subscribed,
                    levels,
                },
            );
        });
        log::debug!("profiled {key} in {} ms", started.elapsed().as_millis());
    }

    /// Each Battery Level characteristic a board publishes, and which half it
    /// speaks for. A split board publishes several Battery Service instances
    /// under the one UUID, and the presentation format descriptor beside each
    /// level is the only thing that tells them apart.
    fn read_halves(&self, key: &str) -> Result<Vec<(GattCharacteristic, Half)>, BleError> {
        let device = device(key)?;
        let result = wait(
            device
                .GetGattServicesForUuidWithCacheModeAsync(
                    GUID::from_u128(BATTERY_SERVICE),
                    BluetoothCacheMode::Cached,
                )
                .map_err(platform)?,
            REPLY_TIMEOUT,
            "finding the battery",
        )?;
        status(result.Status().map_err(platform)?, "finding the battery")?;

        let mut halves = Vec::new();
        for service in result.Services().map_err(platform)? {
            let found = wait(
                service
                    .GetCharacteristicsForUuidWithCacheModeAsync(
                        GUID::from_u128(BATTERY_LEVEL),
                        BluetoothCacheMode::Cached,
                    )
                    .map_err(platform)?,
                REPLY_TIMEOUT,
                "finding the battery level",
            )?;
            if found.Status().map_err(platform)? != GattCommunicationStatus::Success {
                continue;
            }
            for level in found.Characteristics().map_err(platform)? {
                if let Some(half) = half_of(&level) {
                    halves.push((level, half));
                }
            }
        }
        Ok(halves)
    }

    fn follow_level(
        &self,
        key: &str,
        level: &GattCharacteristic,
        half: Half,
    ) -> Result<i64, BleError> {
        let shared = Arc::clone(&self.shared);
        let owner = key.to_string();
        let token = level
            .ValueChanged(&TypedEventHandler::<
                GattCharacteristic,
                GattValueChangedEventArgs,
            >::new(move |_, args| {
                let Some(args) = args.as_ref() else {
                    return Ok(());
                };
                let value = bytes(&args.CharacteristicValue()?)?;
                if let Some(raw) = value.first().copied() {
                    shared.update(|state| {
                        if let Some(profile) = state.profiles.get_mut(&owner) {
                            profile.levels.insert(half, raw);
                        }
                    });
                }
                Ok(())
            }))
            .map_err(platform)?;
        let written = wait(
            level
                .WriteClientCharacteristicConfigurationDescriptorWithResultAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::Notify,
                )
                .map_err(platform)?,
            REPLY_TIMEOUT,
            "subscribing to the battery level",
        );
        match written.and_then(|written| status(written.Status().map_err(platform)?, "subscribing"))
        {
            Ok(()) => Ok(token),
            Err(err) => {
                let _ = level.RemoveValueChanged(token);
                Err(err)
            }
        }
    }

    /// The open link to a board, or a new one.
    fn link(&self, key: &str) -> Result<Arc<Link>, BleError> {
        let mut links = self.links.lock().unwrap();
        if let Some(link) = links.get(key).and_then(Weak::upgrade) {
            if link.connected.load(Ordering::SeqCst) || link.session_id() == 0 {
                return Ok(link);
            }
        }
        let link = self.open_link(key)?;
        links.insert(key.to_string(), Arc::downgrade(&link));
        Ok(link)
    }

    fn open_link(&self, key: &str) -> Result<Arc<Link>, BleError> {
        apartment();
        let device = device(key)?;
        let session = wait(
            GattSession::FromDeviceIdAsync(&device.BluetoothDeviceId().map_err(platform)?)
                .map_err(platform)?,
            QUERY_TIMEOUT,
            "opening a session",
        )?;
        session.SetMaintainConnection(true).map_err(platform)?;

        let connected = device
            .ConnectionStatus()
            .is_ok_and(|status| status == BluetoothConnectionStatus::Connected);
        Ok(Arc::new_cyclic(|this: &Weak<Link>| {
            let this = this.clone();
            let token = device
                .ConnectionStatusChanged(
                    &TypedEventHandler::<BluetoothLEDevice, IInspectable>::new(move |device, _| {
                        let connected = device
                            .as_ref()
                            .and_then(|device| device.ConnectionStatus().ok())
                            == Some(BluetoothConnectionStatus::Connected);
                        if let Some(link) = this.upgrade() {
                            let was = link.connected.swap(connected, Ordering::SeqCst);
                            if connected && !was {
                                link.sessions.fetch_add(1, Ordering::SeqCst);
                            }
                            link.shared.notify();
                        }
                        Ok(())
                    }),
                )
                .unwrap_or_default();
            Link {
                key: key.to_string(),
                shared: Arc::clone(&self.shared),
                device,
                session,
                services: Mutex::new(HashMap::new()),
                connected: AtomicBool::new(connected),
                sessions: AtomicU64::new(u64::from(connected)),
                token,
            }
        }))
    }
}

/// What every GATT service interface on this machine says, by board address:
/// the services each bonded board carries, and the product its PnP ID names.
struct Interfaces {
    services: Vec<u128>,
    modalias: Option<String>,
}

fn service_interfaces() -> Result<HashMap<u64, Interfaces>, BleError> {
    let filter = HSTRING::from(format!(
        "System.Devices.InterfaceClassGuid:=\"{GATT_SERVICE_INTERFACE}\""
    ));
    let found = wait(
        DeviceInformation::FindAllAsyncWithKindAqsFilterAndAdditionalProperties(
            &filter,
            &properties(&[
                SERVICE_ADDRESS,
                SERVICE_UUID,
                VENDOR_SOURCE,
                VENDOR,
                PRODUCT,
                PRODUCT_VERSION,
            ])?,
            DeviceInformationKind::DeviceInterface,
        )
        .map_err(platform)?,
        QUERY_TIMEOUT,
        "listing GATT services",
    )?;

    let mut boards: HashMap<u64, Interfaces> = HashMap::new();
    for info in found {
        let Ok(properties) = info.Properties() else {
            continue;
        };
        let lookup = |name: &str| properties.Lookup(&HSTRING::from(name)).ok();
        let Some(address) = lookup(SERVICE_ADDRESS)
            .and_then(|value| string(&value))
            .and_then(|text| parse_address(&text))
        else {
            continue;
        };
        let board = boards.entry(address).or_insert(Interfaces {
            services: Vec::new(),
            modalias: None,
        });
        if let Some(uuid) = lookup(SERVICE_UUID).and_then(|value| guid(&value)) {
            if !board.services.contains(&uuid) {
                board.services.push(uuid);
            }
        }
        if board.modalias.is_none() {
            board.modalias = modalias(
                lookup(VENDOR_SOURCE).and_then(|value| number(&value)),
                lookup(VENDOR).and_then(|value| number(&value)),
                lookup(PRODUCT).and_then(|value| number(&value)),
                lookup(PRODUCT_VERSION).and_then(|value| number(&value)),
            );
        }
    }
    Ok(boards)
}

/// A bonded Bluetooth LE device, as the pairing selector lists it.
struct Bonded {
    key: String,
    name: Option<String>,
    address: Option<u64>,
}

/// Every bonded Bluetooth LE device.
fn bonded_devices() -> Result<Vec<Bonded>, BleError> {
    let selector = BluetoothLEDevice::GetDeviceSelectorFromPairingState(true).map_err(platform)?;
    let found = wait(
        DeviceInformation::FindAllAsyncAqsFilterAndAdditionalProperties(
            &selector,
            &properties(&[ENDPOINT_ADDRESS])?,
        )
        .map_err(platform)?,
        QUERY_TIMEOUT,
        "listing bonded devices",
    )?;
    let mut devices = Vec::new();
    for info in found {
        let Ok(key) = info.Id() else {
            continue;
        };
        let name = info
            .Name()
            .ok()
            .map(|name| name.to_string())
            .filter(|name| !name.is_empty());
        let address = info
            .Properties()
            .ok()
            .and_then(|properties| properties.Lookup(&HSTRING::from(ENDPOINT_ADDRESS)).ok())
            .and_then(|value| string(&value))
            .and_then(|text| parse_address(&text))
            .or_else(|| address_of_key(&key.to_string()));
        devices.push(Bonded {
            key: key.to_string(),
            name,
            address,
        });
    }
    Ok(devices)
}

fn on_advert(shared: &Shared, args: &BluetoothLEAdvertisementReceivedEventArgs) {
    let Ok(address) = args.BluetoothAddress() else {
        return;
    };
    let rssi = args.RawSignalStrengthInDBm().unwrap_or_default();
    let (name, services) = match args.Advertisement() {
        Ok(advert) => (
            advert
                .LocalName()
                .ok()
                .map(|name| name.to_string())
                .filter(|name| !name.is_empty()),
            advert
                .ServiceUuids()
                .map(|uuids| uuids.into_iter().map(|uuid| uuid.to_u128()).collect())
                .unwrap_or_default(),
        ),
        Err(_) => (None, Vec::new()),
    };

    let mut state = shared.state();
    // A scan response arrives as an advert of its own, often with only the
    // name; it adds to the last one rather than replacing it.
    let advert = state.adverts.entry(address).or_insert_with(|| Advert {
        name: None,
        services: Vec::new(),
        rssi,
        seen: Instant::now(),
    });
    let ours = services.iter().any(|uuid: &u128| zsa_uuid(*uuid));
    let new = advert.services.is_empty() && !services.is_empty();
    if name.is_some() {
        advert.name = name;
    }
    for uuid in services {
        if !advert.services.contains(&uuid) {
            advert.services.push(uuid);
        }
    }
    advert.rssi = rssi;
    advert.seen = Instant::now();
    // Only a board is worth waking the monitor for: a busy room advertises
    // dozens of times a second.
    if ours && new {
        state.generation += 1;
        drop(state);
        shared.notify();
    }
}

/// Whether a UUID is in ZSA's own space, which every board identity and
/// service is.
fn zsa_uuid(uuid: u128) -> bool {
    uuid >> 112 == 0x3297
}

/// The half a Battery Level characteristic speaks for, by the presentation
/// format descriptor beside it.
fn half_of(level: &GattCharacteristic) -> Option<Half> {
    let found = wait(
        level
            .GetDescriptorsForUuidWithCacheModeAsync(
                GUID::from_u128(PRESENTATION_FORMAT),
                BluetoothCacheMode::Cached,
            )
            .ok()?,
        REPLY_TIMEOUT,
        "finding the presentation format",
    )
    .ok()?;
    let descriptor = found.Descriptors().ok()?.into_iter().next()?;
    let read = wait(
        descriptor
            .ReadValueWithCacheModeAsync(BluetoothCacheMode::Uncached)
            .ok()?,
        REPLY_TIMEOUT,
        "reading the presentation format",
    )
    .ok()?;
    if read.Status().ok()? != GattCommunicationStatus::Success {
        return None;
    }
    battery::half_from_format(&bytes(&read.Value().ok()?).ok()?)
}

fn read_value(characteristic: &GattCharacteristic) -> Result<Vec<u8>, BleError> {
    let read = wait(
        characteristic
            .ReadValueWithCacheModeAsync(BluetoothCacheMode::Uncached)
            .map_err(platform)?,
        REPLY_TIMEOUT,
        "reading",
    )?;
    status(read.Status().map_err(platform)?, "reading")?;
    bytes(&read.Value().map_err(platform)?).map_err(platform)
}

/// A bonded device by its handle. Does not connect.
fn device(key: &str) -> Result<BluetoothLEDevice, BleError> {
    wait(
        BluetoothLEDevice::FromIdAsync(&HSTRING::from(key)).map_err(platform)?,
        QUERY_TIMEOUT,
        "finding the device",
    )
    .map_err(|_| BleError::NotFound(format!("{key} is not a device Windows knows")))
}

/// Waits on an operation, cancelling it at the deadline.
fn wait<T>(op: IAsyncOperation<T>, timeout: Duration, what: &str) -> Result<T, BleError>
where
    T: windows::core::RuntimeType + Send + 'static,
{
    let (tx, rx) = mpsc::sync_channel(1);
    op.when(move |result| {
        let _ = tx.send(result);
    })
    .map_err(platform)?;
    match rx.recv_timeout(timeout) {
        Ok(result) => result.map_err(platform),
        Err(_) => {
            let _ = op.Cancel();
            Err(BleError::Platform(format!(
                "{what} took longer than {} s",
                timeout.as_secs()
            )))
        }
    }
}

/// A GATT status as a result. Everything but success is the board's refusal or
/// its absence, said in words.
fn status(status: GattCommunicationStatus, what: &str) -> Result<(), BleError> {
    match status {
        GattCommunicationStatus::Success => Ok(()),
        GattCommunicationStatus::Unreachable => Err(BleError::Platform(format!(
            "{what}: the board is out of reach"
        ))),
        GattCommunicationStatus::AccessDenied => Err(BleError::Platform(format!(
            "{what}: Windows refused access"
        ))),
        GattCommunicationStatus::ProtocolError => {
            Err(BleError::Platform(format!("{what}: the board refused")))
        }
        other => Err(BleError::Platform(format!("{what}: status {}", other.0))),
    }
}

/// The handle Windows gives a device: the adapter's address and the device's.
fn device_key(adapter: u64, device: u64) -> String {
    format!(
        "BluetoothLE#BluetoothLE{}-{}",
        format_address(adapter),
        format_address(device)
    )
}

/// An address as Windows writes it in a handle: lowercase, colon-separated.
fn format_address(address: u64) -> String {
    let bytes = address.to_be_bytes();
    bytes[2..]
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join(":")
}

/// An address in either form Windows hands over — `db:dc:7d:6c:33:3a` from an
/// endpoint, `dbdc7d6c333a` from an interface — as the integer adverts carry.
fn parse_address(text: &str) -> Option<u64> {
    let digits: String = text.chars().filter(|c| *c != ':' && *c != '-').collect();
    (digits.len() == 12)
        .then(|| u64::from_str_radix(&digits, 16).ok())
        .flatten()
}

/// The device's address out of its handle, the part after the adapter's.
fn address_of_key(key: &str) -> Option<u64> {
    parse_address(key.rsplit('-').next()?)
}

/// A PnP ID in the modalias form BlueZ derives from the same fields.
fn modalias(
    source: Option<u32>,
    vendor: Option<u32>,
    product: Option<u32>,
    version: Option<u32>,
) -> Option<String> {
    let kind = match source? {
        1 => "bluetooth",
        2 => "usb",
        _ => return None,
    };
    Some(format!(
        "{kind}:v{:04X}p{:04X}d{:04X}",
        vendor?,
        product?,
        version.unwrap_or(0)
    ))
}

fn properties(names: &[&str]) -> Result<IIterable<HSTRING>, BleError> {
    let names: Vec<HSTRING> = names.iter().map(|name| HSTRING::from(*name)).collect();
    Ok(IIterable::from(names))
}

fn string(value: &IInspectable) -> Option<String> {
    value
        .cast::<IReference<HSTRING>>()
        .ok()?
        .Value()
        .ok()
        .map(|text| text.to_string())
}

fn guid(value: &IInspectable) -> Option<u128> {
    Some(
        value
            .cast::<IReference<GUID>>()
            .ok()?
            .Value()
            .ok()?
            .to_u128(),
    )
}

/// A property that is a number of any width Windows chose to box it in.
fn number(value: &IInspectable) -> Option<u32> {
    if let Ok(value) = value.cast::<IReference<u16>>() {
        return value.Value().ok().map(u32::from);
    }
    if let Ok(value) = value.cast::<IReference<u8>>() {
        return value.Value().ok().map(u32::from);
    }
    value.cast::<IReference<u32>>().ok()?.Value().ok()
}

fn buffer(value: &[u8]) -> Result<IBuffer, BleError> {
    let writer = DataWriter::new().map_err(platform)?;
    writer.WriteBytes(value).map_err(platform)?;
    writer.DetachBuffer().map_err(platform)
}

fn bytes(buffer: &IBuffer) -> windows::core::Result<Vec<u8>> {
    let reader = DataReader::FromBuffer(buffer)?;
    let mut value = vec![0u8; buffer.Length()? as usize];
    reader.ReadBytes(&mut value)?;
    Ok(value)
}

/// Keeps the multithreaded apartment alive for the life of the process, so
/// every thread that has not joined one of its own is in it.
fn apartment() {
    static MTA: OnceLock<()> = OnceLock::new();
    MTA.get_or_init(|| {
        // The cookie would only be handed back to end it, which never happens.
        if let Err(err) = unsafe { CoIncrementMTAUsage() } {
            log::warn!("cannot join the multithreaded apartment: {err}");
        }
    });
}

fn platform(err: windows::core::Error) -> BleError {
    BleError::Platform(err.message().to_string())
}

fn clone_error(err: &BleError) -> BleError {
    BleError::Platform(err.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_address_reads_in_both_forms_windows_writes_it() {
        assert_eq!(parse_address("db:dc:7d:6c:33:3a"), Some(0xdbdc7d6c333a));
        assert_eq!(parse_address("DBDC7D6C333A"), Some(0xdbdc7d6c333a));
        assert_eq!(parse_address("dbdc7d6c33"), None);
        assert_eq!(parse_address("not an address"), None);
    }

    #[test]
    fn a_handle_round_trips_through_its_addresses() {
        let key = device_key(0xb48c9d35f786, 0xdbdc7d6c333a);
        assert_eq!(
            key,
            "BluetoothLE#BluetoothLEb4:8c:9d:35:f7:86-db:dc:7d:6c:33:3a"
        );
        assert_eq!(address_of_key(&key), Some(0xdbdc7d6c333a));
    }

    #[test]
    fn a_pnp_id_reads_as_the_modalias_bluez_would_have_made_of_it() {
        // What the GATT service interfaces of a bonded Voyager 2 carry.
        assert_eq!(
            modalias(Some(2), Some(0x3297), Some(0x1978), Some(0x0100)).as_deref(),
            Some("usb:v3297p1978d0100")
        );
        assert_eq!(
            identity::keyboard_from_modalias("usb:v3297p1978d0100"),
            Some(zapp_core::device::ids::Keyboard::Voyager2)
        );
        assert_eq!(
            modalias(Some(1), Some(0x0059), Some(0x0001), None).as_deref(),
            Some("bluetooth:v0059p0001d0000")
        );
        // No source, or one the spec does not define, names nothing.
        assert_eq!(modalias(None, Some(0x3297), Some(0x1978), None), None);
        assert_eq!(modalias(Some(3), Some(0x3297), Some(0x1978), None), None);
    }

    #[test]
    fn only_zsas_own_uuids_wake_the_monitor() {
        assert!(zsa_uuid(0x32970101_e5aa_4f42_a596_1ea5a0200001));
        assert!(zsa_uuid(0x32970001_e5aa_4f42_a596_1ea5a0200001));
        assert!(!zsa_uuid(0x00001812_0000_1000_8000_00805f9b34fb));
    }
}
