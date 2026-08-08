//! One GATT characteristic on one board, over BlueZ.
//!
//! BlueZ offers two ways to move bytes through a characteristic, and this
//! module prefers the faster one and keeps the other as a fallback.
//!
//! `AcquireWrite`/`AcquireNotify` hand back a socket pair per direction. Every
//! packet after that is a plain `write`/`read` on a file descriptor: no D-Bus
//! round trip, no message marshalling, and back pressure comes free from the
//! socket buffer filling up. They also answer the question nothing else does —
//! `AcquireWrite` returns the negotiated MTU, which is what decides how much
//! image fits in one request. A firmware image is tens of thousands of packets,
//! so this is the path that matters.
//!
//! `WriteValue` plus `PropertiesChanged` needs no kernel tricks and works on
//! every BlueZ, but pays a bus round trip per packet in each direction and
//! never says what the MTU is. It is what runs when the sockets are refused —
//! BlueZ before 5.46, or a characteristic without write-without-response.

use std::collections::HashMap;
use std::io::{ErrorKind, Read, Write};
use std::os::unix::net::UnixStream;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use zbus::blocking::{fdo::ObjectManagerProxy, Connection, MessageIterator, Proxy};
use zbus::zvariant::{ObjectPath, OwnedValue, Value};
use zbus::MatchRule;

use super::bluez::{self, Objects, CHARACTERISTIC, DEVICE, SERVICE};
use super::GattChannel;
use crate::{identity, BleError};

const PROPERTIES: &str = "org.freedesktop.DBus.Properties";

/// How long to wait for a link and for BlueZ to walk the board's GATT tree.
///
/// A board that is bonded but idle has to be woken, which means waiting out its
/// advertising interval before the connection even starts.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
const RESOLVE_POLL: Duration = Duration::from_millis(200);

/// What one write carries when nothing will say otherwise.
///
/// The socket path reports the real figure and this is never used; it is the
/// fallback path's guess, matching the 247-byte ATT MTU a Zephyr peripheral
/// asks for, less the three bytes of ATT header.
const DEFAULT_MTU: usize = 244;

/// Connects if need be, then opens a characteristic.
pub fn open(
    connection: &Connection,
    objects: &ObjectManagerProxy<'static>,
    device: &str,
    service: u128,
    characteristic: u128,
) -> Result<Box<dyn GattChannel>, BleError> {
    let tree = objects.get_managed_objects().map_err(bluez::platform)?;
    let properties = tree
        .iter()
        .find(|(path, _)| path.as_str() == device)
        .and_then(|(_, ifaces)| bluez::properties(ifaces, DEVICE))
        .ok_or_else(|| BleError::NotFound(format!("{device} is not a device BlueZ knows")))?;

    // An SMP link is only offered over an encrypted one, and this tool does not
    // pair: a board that is not bonded is a job for the desktop's own pairing.
    if !bluez::get::<bool>(properties, "Paired").unwrap_or(false) {
        let name = bluez::get::<String>(properties, "Alias")
            .or_else(|| bluez::get::<String>(properties, "Name"))
            .unwrap_or_else(|| device.to_string());
        return Err(BleError::NotBonded(name));
    }

    connect(connection, device)?;
    // The tree read above predates the connection, so the services are only in
    // it now.
    let tree = objects.get_managed_objects().map_err(bluez::platform)?;
    let path = find_characteristic(&tree, device, service, characteristic).ok_or_else(|| {
        BleError::NotFound(format!(
            "{device} does not carry characteristic {characteristic:032x}"
        ))
    })?;
    log::debug!("SMP characteristic at {path}");

    match FdChannel::acquire(connection, &path) {
        Ok(channel) => Ok(Box::new(channel)),
        Err(err) => {
            log::info!("no socket path on this characteristic ({err}); falling back to D-Bus");
            Ok(Box::new(DbusChannel::start(connection, &path)?))
        }
    }
}

/// Brings the link up, and waits for BlueZ to finish walking the GATT tree.
///
/// Both halves matter: `Connect` returns once the link is up, which is before
/// the characteristics exist as objects, and asking for one too early finds
/// nothing.
fn connect(connection: &Connection, device: &str) -> Result<(), BleError> {
    let proxy = Proxy::new(connection, bluez::BLUEZ, device, DEVICE).map_err(bluez::platform)?;

    if !proxy.get_property::<bool>("Connected").unwrap_or(false) {
        log::info!("connecting to {device}");
        proxy
            .call::<_, _, ()>("Connect", &())
            .map_err(|err| BleError::Platform(format!("cannot connect to {device}: {err}")))?;
    }

    let deadline = Instant::now() + CONNECT_TIMEOUT;
    loop {
        if proxy.get_property::<bool>("ServicesResolved").unwrap_or(false) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(BleError::Platform(format!(
                "{device} connected but BlueZ never resolved its services"
            )));
        }
        thread::sleep(RESOLVE_POLL);
    }
}

/// The object path of one characteristic under one service under one device.
fn find_characteristic(
    tree: &Objects,
    device: &str,
    service: u128,
    characteristic: u128,
) -> Option<String> {
    let uuid_of = |path: &str, iface: &str| -> Option<u128> {
        let ifaces = tree.iter().find(|(other, _)| other.as_str() == path)?.1;
        let properties = bluez::properties(ifaces, iface)?;
        identity::parse_uuid(&bluez::get::<String>(properties, "UUID")?)
    };

    let service = tree
        .keys()
        .map(|path| path.as_str())
        .find(|path| path.starts_with(device) && uuid_of(path, SERVICE) == Some(service))?;

    tree.keys()
        .map(|path| path.as_str())
        .find(|path| {
            path.starts_with(service) && uuid_of(path, CHARACTERISTIC) == Some(characteristic)
        })
        .map(str::to_string)
}

/// The socket path: one file descriptor per direction.
struct FdChannel {
    writer: UnixStream,
    notifier: UnixStream,
    mtu: usize,
    path: String,
    /// Held open for as long as the sockets are: BlueZ ties what it acquired to
    /// the client that asked, and drops it when that client leaves the bus.
    connection: Connection,
    /// Set once, since a socket read timeout survives between calls.
    timeout: Option<Duration>,
}

impl FdChannel {
    fn acquire(connection: &Connection, path: &str) -> Result<Self, BleError> {
        let proxy =
            Proxy::new(connection, bluez::BLUEZ, path, CHARACTERISTIC).map_err(bluez::platform)?;
        let options: HashMap<String, OwnedValue> = HashMap::new();

        // Notify first. Acquiring the write side locks WriteValue, so a failure
        // after it would leave the characteristic worse off than not trying.
        let (notify_fd, notify_mtu) = proxy
            .call::<_, _, (zbus::zvariant::OwnedFd, u16)>("AcquireNotify", &(&options,))
            .map_err(bluez::platform)?;
        let (write_fd, write_mtu) = proxy
            .call::<_, _, (zbus::zvariant::OwnedFd, u16)>("AcquireWrite", &(&options,))
            .map_err(bluez::platform)?;

        let notifier = socket(notify_fd)?;
        let writer = socket(write_fd)?;

        // Both are ATT payload sizes, already less the three bytes of ATT
        // header, and a request cannot outgrow what a notification can answer.
        let mtu = usize::from(write_mtu.min(notify_mtu));
        log::debug!("acquired sockets on {path}, mtu {mtu} (write {write_mtu}, notify {notify_mtu})");
        if mtu == 0 {
            return Err(BleError::Platform("BlueZ reported a zero MTU".into()));
        }

        Ok(Self {
            writer,
            notifier,
            mtu,
            path: path.to_string(),
            connection: connection.clone(),
            timeout: None,
        })
    }
}

/// One acquired descriptor as a blocking socket.
///
/// BlueZ creates the pair non-blocking and hands over its end as it made it, so
/// clearing that is what turns reads into ones that can be waited on and writes
/// into ones that push back when the buffer fills rather than failing.
fn socket(fd: zbus::zvariant::OwnedFd) -> Result<UnixStream, BleError> {
    let stream = UnixStream::from(std::os::fd::OwnedFd::from(fd));
    stream
        .set_nonblocking(false)
        .map_err(|err| BleError::Platform(format!("cannot block on a BlueZ socket: {err}")))?;
    Ok(stream)
}

impl GattChannel for FdChannel {
    fn mtu(&self) -> usize {
        self.mtu
    }

    fn read(&mut self) -> Result<Vec<u8>, BleError> {
        // Acquiring the write and notify sockets does not lock ReadValue.
        bluez::read(&self.connection, &self.path, CHARACTERISTIC)
            .ok_or_else(|| BleError::Platform(format!("cannot read {}", self.path)))
    }

    fn write(&mut self, value: &[u8]) -> Result<(), BleError> {
        // A sequenced-packet socket sends one packet per write, which is
        // exactly the framing GATT wants; write_all would be wrong here, since
        // a short write would become a second packet.
        let sent = self
            .writer
            .write(value)
            .map_err(|err| BleError::Platform(format!("write failed: {err}")))?;
        if sent != value.len() {
            return Err(BleError::Platform(format!(
                "BlueZ took {sent} of {} bytes",
                value.len()
            )));
        }
        Ok(())
    }

    fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        if self.timeout != Some(timeout) {
            self.notifier
                .set_read_timeout(Some(timeout))
                .map_err(|err| BleError::Platform(format!("cannot set a read timeout: {err}")))?;
            self.timeout = Some(timeout);
        }

        let mut buffer = vec![0u8; self.mtu.max(512)];
        match self.notifier.read(&mut buffer) {
            Ok(0) => Err(BleError::Platform("the board closed the link".into())),
            Ok(read) => {
                buffer.truncate(read);
                Ok(Some(buffer))
            }
            Err(err) if timed_out(&err) => Ok(None),
            Err(err) => Err(BleError::Platform(format!("notification read failed: {err}"))),
        }
    }
}

fn timed_out(err: &std::io::Error) -> bool {
    matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut)
}

/// The D-Bus path: a method call out, a signal back.
struct DbusChannel {
    connection: Connection,
    path: String,
    mtu: usize,
    notifications: Receiver<Vec<u8>>,
}

impl DbusChannel {
    fn start(connection: &Connection, path: &str) -> Result<Self, BleError> {
        // Subscribed before StartNotify, or the first notification — which for
        // a request already in flight is the one that matters — is missed.
        let notifications = watch_value(connection, path)?;

        let proxy =
            Proxy::new(connection, bluez::BLUEZ, path, CHARACTERISTIC).map_err(bluez::platform)?;
        proxy
            .call::<_, _, ()>("StartNotify", &())
            .map_err(|err| BleError::Platform(format!("StartNotify on {path} failed: {err}")))?;

        // BlueZ 5.62 and later publish the ATT MTU here, header included.
        let mtu = match proxy.get_property::<u16>("MTU") {
            Ok(mtu) if mtu > 3 => usize::from(mtu) - 3,
            _ => {
                log::warn!("BlueZ will not say what the MTU is; assuming {DEFAULT_MTU}");
                DEFAULT_MTU
            }
        };
        log::debug!("notifying on {path} over D-Bus, mtu {mtu}");

        Ok(Self {
            connection: connection.clone(),
            path: path.to_string(),
            mtu,
            notifications,
        })
    }
}

/// Ticks with every new value BlueZ publishes for one characteristic.
fn watch_value(connection: &Connection, path: &str) -> Result<Receiver<Vec<u8>>, BleError> {
    let rule = MatchRule::builder()
        .msg_type(zbus::message::Type::Signal)
        .sender(bluez::BLUEZ)
        .map_err(bluez::platform)?
        .path(ObjectPath::try_from(path.to_string()).map_err(bluez::platform)?)
        .map_err(bluez::platform)?
        .interface(PROPERTIES)
        .map_err(bluez::platform)?
        .member("PropertiesChanged")
        .map_err(bluez::platform)?
        .build();
    let signals = MessageIterator::for_match_rule(rule, connection, None).map_err(bluez::platform)?;

    let (tx, rx) = mpsc::channel();
    thread::Builder::new()
        .name("zapp-ble-notify".into())
        .spawn(move || {
            for message in signals.flatten() {
                let body = message.body();
                let Ok((_, changed, _)) =
                    body.deserialize::<(String, HashMap<String, OwnedValue>, Vec<String>)>()
                else {
                    continue;
                };
                // Anything else about the characteristic changing is not ours.
                let Some(value) = bluez::get::<Vec<u8>>(&changed, "Value") else {
                    continue;
                };
                if tx.send(value).is_err() {
                    break;
                }
            }
        })
        .map_err(bluez::platform)?;

    Ok(rx)
}

impl GattChannel for DbusChannel {
    fn mtu(&self) -> usize {
        self.mtu
    }

    fn read(&mut self) -> Result<Vec<u8>, BleError> {
        bluez::read(&self.connection, &self.path, CHARACTERISTIC)
            .ok_or_else(|| BleError::Platform(format!("cannot read {}", self.path)))
    }

    fn write(&mut self, value: &[u8]) -> Result<(), BleError> {
        let proxy = Proxy::new(&self.connection, bluez::BLUEZ, self.path.as_str(), CHARACTERISTIC)
            .map_err(bluez::platform)?;
        // "command" is write-without-response, which is the only kind the SMP
        // characteristic accepts; the default would be a request write, and
        // BlueZ would turn one longer than the MTU into a long write, which the
        // characteristic does not accept either.
        let options = HashMap::from([("type", Value::from("command"))]);
        proxy
            .call::<_, _, ()>("WriteValue", &(value, options))
            .map_err(|err| BleError::Platform(format!("WriteValue failed: {err}")))
    }

    fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        match self.notifications.recv_timeout(timeout) {
            Ok(value) => Ok(Some(value)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => {
                Err(BleError::Platform("the notification watcher stopped".into()))
            }
        }
    }
}

impl Drop for DbusChannel {
    fn drop(&mut self) {
        let proxy = Proxy::new(&self.connection, bluez::BLUEZ, self.path.as_str(), CHARACTERISTIC);
        let call = proxy.and_then(|proxy| proxy.call::<_, _, ()>("StopNotify", &()));
        if let Err(err) = call {
            log::debug!("StopNotify on {} failed: {err}", self.path);
        }
    }
}
