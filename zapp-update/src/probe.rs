//! What is plugged in, and which way an update goes.

use std::time::{Duration, Instant};

use zapp_core::device::{self, Keyboard, UsbDevice};
use zapp_core::hid::{
    ChargeTelemetry, HidConnection, HidEvent, HostSlot, PmicRailsReport, RadioInfo, Telemetry,
    CMD_GET_CHARGE_TELEMETRY, CMD_GET_HOST_SLOTS, CMD_GET_PMIC_RAILS, CMD_GET_RADIO_INFO,
    CMD_GET_TELEMETRY, HOST_SLOTS_MAX, has_radio_info,
};

use zapp_wired::ignition::usb::UsbLink;
use zapp_wired::WiredError;

/// The way an update reaches the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    Usb,
    Bluetooth,
}

impl Transport {
    pub fn label(self) -> &'static str {
        match self {
            Transport::Usb => "USB",
            Transport::Bluetooth => "Bluetooth",
        }
    }
}

/// Which way a Wireless Voyager update goes: the cable whenever the keyboard is on it,
/// in its bootloader or running its own firmware, Bluetooth otherwise.
///
/// Bluetooth is for a keyboard that is not plugged in at all, not for one that
/// is plugged in and simply has not been put in its bootloader yet: that one is
/// a cable update the user is part way through, and sending it over the radio
/// instead would be a surprise. The model is what decides, not "some ZSA board
/// is on USB": a user with an original Voyager on the cable and a Wireless Voyager on
/// Bluetooth has to get the radio.
pub fn route() -> Transport {
    if bootloader_present() || voyager2_present() {
        Transport::Usb
    } else {
        Transport::Bluetooth
    }
}

/// Whether a bootloader is plugged in right now, without waiting for one.
///
/// For deciding which way to send a bundle. Waiting would defeat the point:
/// the answer "not here" is what routes it over Bluetooth instead.
pub fn bootloader_present() -> bool {
    match UsbLink::open() {
        Ok(_) => true,
        Err(WiredError::NoBootloader) => false,
        Err(WiredError::AccessDenied(err)) => {
            // Plugged in, and either not opened up by udev yet or never will
            // be. Both are the cable: the wired path waits out the first and
            // names the second.
            log::debug!("a bootloader is present but refused to open: {err}");
            true
        }
        Err(err) => {
            // Something is plugged in that cannot be opened. Say so rather
            // than silently going over the radio, where the same board would
            // work and the cause would stay hidden.
            log::warn!("a bootloader is present but could not be opened: {err}");
            false
        }
    }
}

/// Whether a Wireless Voyager running its own firmware is plugged in right now.
///
/// For deciding which way to send a bundle, alongside [`bootloader_present`]:
/// between them they are every USB state a bundle can be written from.
pub fn voyager2_present() -> bool {
    attached().is_some_and(|attached| voyager2_among(&attached).is_some())
}

/// What is plugged in, or nothing when the bus cannot be read.
pub(crate) fn attached() -> Option<Vec<UsbDevice>> {
    device::list_devices()
        .inspect_err(|err| log::debug!("could not look at what is plugged in: {err}"))
        .ok()
}

/// The attached Wireless Voyager running its own firmware, if one is there.
///
/// The model is checked, not just "a ZSA keyboard": an original Voyager on the
/// same cable is a board no bundle can be written to, and counting it sends the
/// update down the wire at something that will never answer.
///
/// A board still in a bootloader is not it either. For the restart wait that is
/// the whole point — the STM32 has to have left one — and for routing it is
/// [`bootloader_present`]'s to find.
pub(crate) fn voyager2_among(attached: &[UsbDevice]) -> Option<&UsbDevice> {
    attached
        .iter()
        .find(|device| device.bootloader.is_none() && device.keyboard == Some(Keyboard::Voyager2))
}

/// How long to wait for a wired board to answer the telemetry query. Firmware
/// that predates the command never answers at all, so this elapses in the
/// ordinary case and must stay short enough not to be felt.
const TELEMETRY_WAIT: Duration = Duration::from_millis(500);

/// How long a board gets to answer `CMD_GET_RADIO_INFO`, which it passes on to
/// the left radio first. The wait `zapp diag` gives a diagnostic page.
pub const RADIO_INFO_WAIT: Duration = Duration::from_millis(1500);
/// How long a Bluetooth board gets to finish the handshake that says its
/// protocol version.
const BLE_HANDSHAKE_WAIT: Duration = Duration::from_secs(3);

/// What the board answers, on one connection: the summary always, and the
/// detail behind it only when it was asked for. A board whose firmware predates
/// any of these commands never answers it, which is why each wait can elapse.
pub type UsbTelemetry = (Telemetry, Option<ChargeTelemetry>, Option<PmicRailsReport>);

/// Asks a wired board about itself: the telemetry summary, the charge detail,
/// and the PMIC rails when `rails` is set.
pub fn usb_telemetry(device: &UsbDevice, rails: bool) -> Option<UsbTelemetry> {
    usb_report(device, rails, false, false).map(|(telemetry, _, _)| telemetry)
}

/// [`usb_telemetry`], plus the radio firmware each half runs when `radio` is
/// set, and the Bluetooth host slots when `slots` is set. The radio answer is
/// `None` for firmware before [`zapp_core::hid::RADIO_INFO_PROTOCOL`], and
/// nothing known when the board did not answer. The slots are empty when the
/// board did not answer.
pub fn usb_report(
    device: &UsbDevice,
    rails: bool,
    radio: bool,
    slots: bool,
) -> Option<(UsbTelemetry, Option<RadioInfo>, Vec<HostSlot>)> {
    if device.bootloader.is_some() {
        return None; // a board in its bootloader runs no keyboard firmware
    }

    let mut probe = Probe::new(HidConnection::open(device.vid, device.pid).ok()?);
    let telemetry = probe.ask(CMD_GET_TELEMETRY, |event| match event {
        HidEvent::Telemetry(telemetry) => Some(telemetry),
        _ => None,
    })?;
    // Asked for every listing, verbose or not: the charger phase words the
    // board's own line, and the two wordings must not differ.
    let charge = probe.ask(CMD_GET_CHARGE_TELEMETRY, |event| match event {
        HidEvent::ChargeTelemetry(telemetry) => Some(telemetry),
        _ => None,
    });
    let rails = rails
        .then(|| {
            probe.ask(CMD_GET_PMIC_RAILS, |event| match event {
                HidEvent::PmicRails(report) => Some(report),
                _ => None,
            })
        })
        .flatten();
    let radio = radio.then(|| probe.radio_info(TELEMETRY_WAIT)).flatten();
    let slots = if slots {
        probe.host_slots(TELEMETRY_WAIT)
    } else {
        Vec::new()
    };

    Some(((telemetry, charge, rails), radio, slots))
}

/// Asks a bonded Bluetooth board which radio firmware each half runs, and its
/// telemetry summary and Bluetooth host slots when `telemetry` is set, on one
/// connection to its Oryx service. `key` is the platform handle. Same answers
/// as [`usb_report`]; nothing when the service cannot be opened.
pub fn ble_report(
    key: &str,
    telemetry: bool,
) -> (Option<RadioInfo>, Option<Telemetry>, Vec<HostSlot>) {
    let connection = match zapp_ble::oryx::open(Some(key)) {
        Ok(connection) => connection,
        Err(err) => {
            log::debug!("could not open the Oryx service on {key}: {err}");
            return (None, None, Vec::new());
        }
    };
    let mut probe = Probe::new(connection);
    let radio = probe.radio_info(BLE_HANDSHAKE_WAIT);
    let telemetry = telemetry
        .then(|| {
            probe.ask(CMD_GET_TELEMETRY, |event| match event {
                HidEvent::Telemetry(telemetry) => Some(telemetry),
                _ => None,
            })
        })
        .flatten();
    let slots = if telemetry.is_some() {
        probe.host_slots(BLE_HANDSHAKE_WAIT)
    } else {
        Vec::new()
    };
    probe.conn.close();
    (radio, telemetry, slots)
}

/// One connection, and the protocol version its handshake reported.
struct Probe {
    conn: HidConnection,
    protocol: Option<u8>,
}

impl Probe {
    fn new(conn: HidConnection) -> Self {
        Self {
            conn,
            protocol: None,
        }
    }

    /// Send one query and wait out the answer `take` is looking for.
    fn ask<T>(&mut self, cmd: u8, take: impl Fn(HidEvent) -> Option<T>) -> Option<T> {
        self.conn.send(cmd, &[]);
        self.wait(TELEMETRY_WAIT, take)
    }

    /// Wait up to `wait` for the event `take` is looking for. Handshake events
    /// on the way past are recorded.
    fn wait<T>(&mut self, wait: Duration, take: impl Fn(HidEvent) -> Option<T>) -> Option<T> {
        let deadline = Instant::now() + wait;
        loop {
            let left = deadline.checked_duration_since(Instant::now())?;
            match self.conn.recv_timeout(left)? {
                HidEvent::Disconnected => return None,
                HidEvent::ProtocolVersion(version) => self.protocol = Some(version),
                event => {
                    if let Some(answer) = take(event) {
                        return Some(answer);
                    }
                }
            }
        }
    }

    /// The protocol version, waiting up to `wait` for it if it has not come.
    fn protocol(&mut self, wait: Duration) -> Option<u8> {
        if self.protocol.is_none() {
            self.wait(wait, |_| None::<()>);
        }
        self.protocol
    }

    /// Asks for the radio firmware when the protocol has the command.
    fn radio_info(&mut self, protocol_wait: Duration) -> Option<RadioInfo> {
        let protocol = self.protocol(protocol_wait);
        if !protocol.is_some_and(has_radio_info) {
            log::debug!("no radio info at protocol {protocol:?}");
            return None;
        }
        self.conn.send(CMD_GET_RADIO_INFO, &[]);
        let answer = self.wait(RADIO_INFO_WAIT, |event| match event {
            HidEvent::RadioInfo(info) => Some(info),
            _ => None,
        });
        if answer.is_none() {
            log::debug!("the board did not answer the radio info request");
        }
        Some(answer.unwrap_or_default())
    }

    /// Asks for each Bluetooth host slot in turn, when the protocol has radio
    /// info. Empty when the board does not answer the first, which is what
    /// firmware before the command does, or knows no slots.
    fn host_slots(&mut self, protocol_wait: Duration) -> Vec<HostSlot> {
        let protocol = self.protocol(protocol_wait);
        if !protocol.is_some_and(has_radio_info) {
            return Vec::new();
        }
        let mut slots = Vec::new();
        let mut count = HOST_SLOTS_MAX;
        let mut slot = 0;
        while slot < count {
            self.conn.send(CMD_GET_HOST_SLOTS, &[slot]);
            let answer = self.wait(TELEMETRY_WAIT, |event| match event {
                HidEvent::HostSlot(answer) if answer.slot == slot => Some(answer),
                _ => None,
            });
            let Some(answer) = answer else {
                log::debug!("the board did not answer host slot {slot}");
                break;
            };
            count = count.min(answer.slot_count);
            if slot < count {
                slots.push(answer);
            }
            slot += 1;
        }
        slots
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::BootloaderKind;

    /// A ZSA board on USB, the way `list_devices` reports one.
    fn plugged_in(keyboard: Option<Keyboard>, pid: u16) -> UsbDevice {
        UsbDevice {
            keyboard,
            bootloader: None,
            serial: Some("layout/revision".to_string()),
            vid: 0x3297,
            pid,
        }
    }

    /// The other ZSA keyboards, none of which a bundle can be written to.
    fn the_other_models() -> Vec<UsbDevice> {
        vec![
            plugged_in(Some(Keyboard::Voyager), 0x1977),
            plugged_in(Some(Keyboard::Moonlander), 0x1969),
            plugged_in(Some(Keyboard::ErgodoxEz), 0x1307),
            plugged_in(Some(Keyboard::PlanckEz), 0x6060),
        ]
    }

    #[test]
    fn another_zsa_keyboard_on_the_cable_is_not_a_voyager_2() {
        // The bug this guards: a wired Voyager plugged in while the Wireless Voyager
        // is on Bluetooth used to count as "a keyboard on USB", so the bundle
        // went down the cable and sat waiting for a reset button on a board it
        // could never be written to.
        for other in the_other_models() {
            let pid = other.pid;
            assert!(
                voyager2_among(&[other]).is_none(),
                "{pid:#06x} was taken for a Wireless Voyager"
            );
        }
        assert!(voyager2_among(&the_other_models()).is_none());
    }

    #[test]
    fn a_voyager_2_among_other_boards_is_still_found() {
        // Whichever order the bus happens to report them in: a user with two
        // ZSA keyboards plugged in must still get the cable.
        let mut mixed = the_other_models();
        mixed.push(plugged_in(Some(Keyboard::Voyager2), 0x1978));
        assert_eq!(voyager2_among(&mixed).map(|found| found.pid), Some(0x1978));
        mixed.reverse();
        assert_eq!(voyager2_among(&mixed).map(|found| found.pid), Some(0x1978));
    }

    #[test]
    fn a_voyager_2_in_a_bootloader_is_not_one_running_its_firmware() {
        // `bootloader_present` finds that one. Counting it here would report a
        // board as back from its restart while it is still sitting in MCUboot.
        let mut booting = plugged_in(Some(Keyboard::Voyager2), 0x0792);
        booting.bootloader = Some(BootloaderKind::IgnitionV2);
        assert!(voyager2_among(&[booting]).is_none());
        // A bootloader that names no keyboard at all is not it either.
        let mut generic = plugged_in(None, 0xDF11);
        generic.vid = 0x0483;
        generic.bootloader = Some(BootloaderKind::Stm32Dfu);
        assert!(voyager2_among(&[generic]).is_none());
        assert!(voyager2_among(&[]).is_none());
    }
}
