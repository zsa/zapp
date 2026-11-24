//! Looks at a bridge that has stopped answering ignition's vendor requests.
//!
//! A diagnostic for the wedge seen after a left confirm: every vendor request
//! stalls, STATUS included, while the device stays enumerated. What it asks
//! tells the possible causes apart:
//!
//! * standard requests to the device — answered by libopencm3 itself, so a
//!   reply means EP0 and the main loop are alive;
//! * the Microsoft OS 2.0 descriptor read — a vendor request that goes through
//!   the same registered callback as ignition's own, before ignition's gate;
//! * ignition's STATUS and CONFIRM_RESULT on interface 1.
//!
//! Nothing here writes anything. `--reset` then resets the port, opens the
//! device again and asks the same questions, to see whether re-enumeration
//! brings it back.
//!
//! ```text
//! cargo run -p zapp-wired --example probe_wedged [-- --reset]
//! ```

use std::time::Duration;

use nusb::transfer::{ControlIn, ControlType, Recipient};
use nusb::{Device, MaybeFuture};
use zapp_wired::ignition::{INTERFACE, PID, REQ_CONFIRM_RESULT, REQ_STATUS, VID};

const TIMEOUT: Duration = Duration::from_millis(1000);

fn open() -> Option<Device> {
    let info = match nusb::list_devices().wait() {
        Ok(mut devices) => devices.find(|d| d.vendor_id() == VID && d.product_id() == PID),
        Err(err) => {
            println!("could not list devices: {err}");
            return None;
        }
    };
    let Some(info) = info else {
        println!("no {VID:04x}:{PID:04x} on USB");
        return None;
    };
    println!("found {VID:04x}:{PID:04x} id {:?}", info.id());
    match info.open().wait() {
        Ok(device) => Some(device),
        Err(err) => {
            println!("could not open it: {err}");
            None
        }
    }
}

fn ask(device: &Device, label: &str, control: ControlIn) {
    for attempt in 1..=3 {
        let result = device.control_in(control, TIMEOUT).wait();
        match result {
            Ok(bytes) => {
                println!("{label}: ok {bytes:02x?}");
                return;
            }
            Err(err) => println!("{label}: attempt {attempt}: {err:?}"),
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn standard(request: u8, value: u16, length: u16) -> ControlIn {
    ControlIn {
        control_type: ControlType::Standard,
        recipient: Recipient::Device,
        request,
        value,
        index: 0,
        length,
    }
}

fn probe(device: &Device) {
    ask(device, "GET_STATUS", standard(0x00, 0, 2));
    ask(device, "GET_DESCRIPTOR device", standard(0x06, 0x0100, 18));
    ask(device, "GET_CONFIGURATION", standard(0x08, 0, 1));
    ask(
        device,
        "MS OS 2.0 descriptor (vendor 0x20, device)",
        ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: 0x20,
            value: 0,
            index: 7,
            length: 10,
        },
    );

    let interface = match device.detach_and_claim_interface(INTERFACE).wait() {
        Ok(interface) => interface,
        Err(err) => {
            println!("could not claim interface {INTERFACE}: {err}");
            return;
        }
    };
    for (label, request, length) in [
        ("STATUS", REQ_STATUS, 16u16),
        ("CONFIRM_RESULT", REQ_CONFIRM_RESULT, 36),
    ] {
        for attempt in 1..=3 {
            let result = interface
                .control_in(
                    ControlIn {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Interface,
                        request,
                        value: 0,
                        index: INTERFACE as u16,
                        length,
                    },
                    TIMEOUT,
                )
                .wait();
            match result {
                Ok(bytes) => {
                    println!("{label}: ok {bytes:02x?}");
                    break;
                }
                Err(err) => println!("{label}: attempt {attempt}: {err:?}"),
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

fn main() {
    let reset = std::env::args().any(|arg| arg == "--reset");

    let Some(device) = open() else { return };
    probe(&device);

    if !reset {
        return;
    }
    println!("--- resetting the port");
    if let Err(err) = device.reset().wait() {
        println!("reset: {err}");
    }
    drop(device);
    std::thread::sleep(Duration::from_secs(2));
    let Some(device) = open() else { return };
    probe(&device);
}
