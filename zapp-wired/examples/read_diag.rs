//! Reads the post-mortem record a `make DIAG=1` ignition keeps.
//!
//! A diagnostic ignition writes, into RAM that a reset does not clear, the PC
//! it was last interrupted at (sampled by SysTick every millisecond), a
//! HardFault's frame if one was taken, how often the EP0 guard fired, and a
//! trace of EP0 events and UART frames that freezes shortly after the guard
//! first fires. On the next boot it copies that record aside and serves it over
//! vendor request 0x91. So: reproduce, press the reset button (do not unplug),
//! run this. A successful `zapp flash` restarts into the keyboard firmware,
//! which overwrites the record; run it with `ZAPP_KEEP_IGNITION=1`.
//!
//! ```text
//! cargo run -p zapp-wired --example read_diag
//! arm-none-eabi-addr2line -f -e ignition-diag.elf 0x<pc>
//! ```
//!
//! A release ignition does not have the request and stalls it.

use std::time::Duration;

use nusb::transfer::{ControlIn, ControlType, Recipient};
use nusb::MaybeFuture;
use zapp_wired::ignition::{INTERFACE, PID, VID};

const REQ_DIAG: u8 = 0x91;
const MAGIC: u32 = 0x4449_4147;
const VERSION: u32 = 5;

/// Words before the trace, following `struct diag_record` (v5).
const FIELDS: [&str; 21] = [
    "magic",
    "version",
    "ticks",
    "loops",
    "pc",
    "lr",
    "xpsr",
    "fault_pc",
    "fault_lr",
    "fault_xpsr",
    "cfsr",
    "hfsr",
    "bfar",
    "last_frame",
    "loop_rate",
    "ep0_rescues",
    "ep0_rescue_ms",
    "ep0_rescue_epr",
    "trace_next",
    "trace_left",
    "trace_frozen",
];
const TRACE_LEN: usize = 32;
const EVENT_LEN: usize = 16;
/// The record, then the reset flags of the boot that served it.
const REPORT_LEN: usize = FIELDS.len() * 4 + TRACE_LEN * EVENT_LEN + 4;

const STATES: [&str; 8] = [
    "IDLE",
    "STALLED",
    "DATA_IN",
    "LAST_DATA_IN",
    "STATUS_IN",
    "DATA_OUT",
    "LAST_DATA_OUT",
    "STATUS_OUT",
];

fn exception(xpsr: u32) -> String {
    match xpsr & 0x1ff {
        0 => "thread mode (the main loop)".into(),
        2 => "NMI".into(),
        3 => "HardFault".into(),
        15 => "SysTick".into(),
        n if n >= 16 => format!("IRQ {} (37 = USART1/EXTI25)", n - 16),
        n => format!("exception {n}"),
    }
}

/// RCC_CSR on the F303 (RM0316 9.4.10).
fn reset_cause(csr: u32) -> String {
    let flags = [
        (23, "V18PWR"),
        (25, "OBL"),
        (26, "PIN (reset button or NRST)"),
        (27, "POR/PDR (power)"),
        (28, "SFT (software)"),
        (29, "IWDG"),
        (30, "WWDG"),
        (31, "LPWR"),
    ];
    let set: Vec<&str> = flags
        .iter()
        .filter(|(bit, _)| csr & (1 << bit) != 0)
        .map(|(_, name)| *name)
        .collect();
    if set.is_empty() {
        format!("none (0x{csr:08x})")
    } else {
        format!("{} (0x{csr:08x})", set.join(", "))
    }
}

fn stat(bits: u16) -> &'static str {
    ["DIS", "STALL", "NAK", "VALID"][(bits & 3) as usize]
}

fn request(code: u8) -> String {
    match code {
        0x00 => "GET_STATUS".into(),
        0x05 => "SET_ADDRESS".into(),
        0x06 => "GET_DESCRIPTOR".into(),
        0x09 => "SET_CONFIGURATION".into(),
        0x80 => "BEGIN".into(),
        0x81 => "SELECT".into(),
        0x82 => "END".into(),
        0x83 => "STATUS".into(),
        0x84 => "RELEASE_MD".into(),
        0x85 => "RESET_TARGET".into(),
        0x86 => "CONFIRM".into(),
        0x87 => "CONFIRM_RESULT".into(),
        0x92 => "VERSION".into(),
        other => format!("0x{other:02x}"),
    }
}

struct Event {
    us: u32,
    istr: u16,
    ep0r: u16,
    loops: u32,
    kind: u8,
    state: u8,
    request: u8,
}

impl Event {
    fn parse(bytes: &[u8]) -> Self {
        let u32_at = |i: usize| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap());
        let u16_at = |i: usize| u16::from_le_bytes(bytes[i..i + 2].try_into().unwrap());
        Self {
            us: u32_at(0),
            istr: u16_at(4),
            ep0r: u16_at(6),
            loops: u32_at(8),
            kind: bytes[12],
            state: bytes[13],
            request: bytes[14],
        }
    }

    fn describe(&self) -> String {
        let state = STATES.get(self.state as usize).copied().unwrap_or("?");
        let usb = format!(
            "ISTR ctr={} dir={} ep={}  EP0R ctr_rx={} setup={} rx={:<5} ctr_tx={} tx={:<5}",
            (self.istr >> 15) & 1,
            (self.istr >> 4) & 1,
            self.istr & 0xf,
            (self.ep0r >> 15) & 1,
            (self.ep0r >> 11) & 1,
            stat(self.ep0r >> 12),
            (self.ep0r >> 7) & 1,
            stat(self.ep0r >> 4),
        );
        match self.kind {
            1 => format!(
                "CTR     in {:<13} req {:<14} {usb}",
                state,
                request(self.request)
            ),
            2 => format!(
                "RESCUE  in {:<13} req {:<14} {usb}",
                state,
                request(self.request)
            ),
            3 => format!("UART frame 0x{:02x}{:>36}{usb}", self.request, ""),
            other => format!("kind {other}: {usb}"),
        }
    }
}

fn main() {
    let info = nusb::list_devices()
        .wait()
        .expect("could not list devices")
        .find(|d| d.vendor_id() == VID && d.product_id() == PID)
        .expect("no ignition on USB");
    let device = info.open().wait().expect("could not open ignition");
    let interface = device
        .detach_and_claim_interface(INTERFACE)
        .wait()
        .expect("could not claim the vendor interface");

    let bytes = match interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request: REQ_DIAG,
                value: 0,
                index: INTERFACE as u16,
                length: REPORT_LEN as u16,
            },
            Duration::from_secs(1),
        )
        .wait()
    {
        Ok(bytes) => bytes,
        Err(err) => {
            println!("DIAG request refused ({err:?}): this ignition was not built with DIAG=1");
            return;
        }
    };
    let word = |name: &str| {
        let i = FIELDS.iter().position(|field| *field == name).unwrap();
        u32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
    };
    if bytes.len() < FIELDS.len() * 4 || word("version") != VERSION {
        println!(
            "{} bytes of a record this reader does not know (it reads version {VERSION})",
            bytes.len()
        );
        return;
    }
    if bytes.len() < REPORT_LEN {
        println!("short report, {} of {REPORT_LEN} bytes", bytes.len());
        return;
    }
    let csr = u32::from_le_bytes(bytes[REPORT_LEN - 4..].try_into().unwrap());
    println!("this boot's reset cause: {}", reset_cause(csr));
    if word("magic") != MAGIC {
        println!("no record from the previous run (power-on, or it never booted a DIAG build)");
        return;
    }

    println!(
        "main loop: {} passes, {} in its last whole second; last sample pc 0x{:08x} in {}",
        word("loops"),
        word("loop_rate"),
        word("pc"),
        exception(word("xpsr"))
    );
    if word("fault_pc") != 0 {
        println!(
            "HARDFAULT: pc 0x{:08x} lr 0x{:08x} in {}, cfsr 0x{:08x} hfsr 0x{:08x} bfar 0x{:08x}",
            word("fault_pc"),
            word("fault_lr"),
            exception(word("fault_xpsr")),
            word("cfsr"),
            word("hfsr"),
            word("bfar")
        );
    }
    let rescues = word("ep0_rescues");
    if rescues == 0 {
        println!("EP0 guard never fired");
    } else {
        println!(
            "EP0 guard fired {rescues} time(s), last at {} ms, EP0R was 0x{:04x}",
            word("ep0_rescue_ms"),
            word("ep0_rescue_epr")
        );
    }

    // ---- The trace, oldest first ----
    let next = word("trace_next") as usize;
    let count = next.min(TRACE_LEN);
    let events: Vec<Event> = (next - count..next)
        .map(|n| {
            let at = FIELDS.len() * 4 + (n % TRACE_LEN) * EVENT_LEN;
            Event::parse(&bytes[at..at + EVENT_LEN])
        })
        .collect();
    let frozen = word("trace_frozen") != 0;
    println!();
    println!(
        "trace: {count} of {next} events, {}",
        if frozen {
            "frozen after the first rescue"
        } else {
            "still running when the board stopped (no rescue, or too few events after it)"
        }
    );
    // Times are from the first rescue when there is one, the first event if not.
    let zero = events
        .iter()
        .find(|event| event.kind == 2)
        .or(events.first())
        .map(|event| event.us)
        .unwrap_or(0);
    let mut previous: Option<&Event> = None;
    for event in &events {
        let passes = previous
            .map(|p| event.loops.wrapping_sub(p.loops))
            .unwrap_or(0);
        println!(
            "{:>+9} us  +{:<5} {}",
            event.us as i64 - zero as i64,
            passes,
            event.describe()
        );
        previous = Some(event);
    }
}
