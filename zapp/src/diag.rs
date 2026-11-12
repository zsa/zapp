//! `zapp diag`: the Wireless Voyager's diagnostic records, read over USB raw HID.

use anyhow::{Context, Result, bail};

use zapp_core::device::{self, Keyboard, UsbDevice};
use zapp_update::report::diag::{self as records, Read};

/// "zapp 1.1.14", as the diagnostics text names the program.
pub(crate) fn tool() -> String {
    format!("zapp {}", env!("CARGO_PKG_VERSION"))
}

/// Reads every record over a new connection, which is handed back open.
pub(crate) fn read(
    device: &UsbDevice,
    count: usize,
) -> Result<(zapp_core::hid::HidConnection, Result<Read>)> {
    records::read(device, count, &tool())
}

/// Reads every record off the attached Wireless Voyager and prints them.
pub fn run() -> Result<()> {
    let devices = device::list_devices().context("could not list USB devices")?;
    let (device, count) = pick(&devices)?;
    let (conn, read) = read(device, count)?;
    let notice = match &read {
        Ok(_) => crate::freeze::notice(&conn),
        Err(_) => None,
    };
    conn.close();

    let read = read?;
    print!("{}", read.text);
    if let Some(notice) = notice {
        println!("{notice}");
    }
    match read.stm32_problem {
        None => Ok(()),
        Some(problem) => bail!(problem),
    }
}

/// The first Wireless Voyager running its keyboard firmware, and how many there are.
pub(crate) fn pick(devices: &[UsbDevice]) -> Result<(&UsbDevice, usize)> {
    let voyagers: Vec<&UsbDevice> = devices
        .iter()
        .filter(|device| device.keyboard == Some(Keyboard::WirelessVoyager))
        .collect();
    let running: Vec<&UsbDevice> = voyagers
        .iter()
        .copied()
        .filter(|device| device.bootloader.is_none())
        .collect();
    match running.first() {
        Some(device) => Ok((device, running.len())),
        None if voyagers.is_empty() => bail!("No Wireless Voyager found on USB."),
        None => bail!(
            "The Wireless Voyager is in its bootloader. Restart it into the keyboard \
             firmware (hold any left-half key for 5 seconds, or run a zapp command that \
             restarts it), then run `zapp diag` again."
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::device::BootloaderKind;

    fn usb(keyboard: Option<Keyboard>, bootloader: Option<BootloaderKind>, pid: u16) -> UsbDevice {
        UsbDevice {
            keyboard,
            bootloader,
            serial: None,
            vid: 0x3297,
            pid,
        }
    }

    /// The error `pick` gives for `devices`.
    fn refused(devices: &[UsbDevice]) -> String {
        match pick(devices) {
            Ok(_) => panic!("a device was picked"),
            Err(err) => err.to_string(),
        }
    }

    #[test]
    fn only_a_wireless_voyager_running_its_firmware_is_picked() {
        let other = || usb(Some(Keyboard::Voyager), None, 0x1977);
        let booting = || {
            usb(
                Some(Keyboard::WirelessVoyager),
                Some(BootloaderKind::IgnitionV2),
                0x0792,
            )
        };
        let running = || usb(Some(Keyboard::WirelessVoyager), None, 0x1978);

        assert_eq!(refused(&[other()]), "No Wireless Voyager found on USB.");
        assert!(refused(&[other(), booting()]).contains("is in its bootloader"));

        let devices = [other(), booting(), running(), running()];
        let (device, count) = pick(&devices).unwrap();
        assert_eq!((device.pid, count), (0x1978, 2));
    }
}
