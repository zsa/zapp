//! Reads the post-mortem record a `make DIAG=1` ignition keeps.
//!
//! A diagnostic ignition writes, into RAM that a reset does not clear, the PC
//! it was last interrupted at (sampled by SysTick every millisecond), a
//! HardFault's frame if one was taken, the type of the last UART frame it
//! handled, and a snapshot of the USB peripheral. On the next boot
//! it copies that record aside and serves it over vendor request 0x91. So:
//! reproduce the wedge, press the reset button (do not unplug), run this.
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

/// Word offsets in the report, following `struct diag_record` (v4) and then
/// the reset flags.
const FIELDS: [&str; 24] = [
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
    "usb_cntr",
    "usb_istr",
    "usb_fnr",
    "usb_daddr",
    "usb_ep0r",
    "ep0_rescues",
    "ep0_rescue_ms",
    "ep0_rescue_epr",
    "rcc_csr",
];
const REPORT_LEN: u16 = (FIELDS.len() * 4) as u16;

fn exception(xpsr: u32) -> String {
    match xpsr & 0x1ff {
        0 => "thread mode (the main loop)".into(),
        2 => "NMI".into(),
        3 => "HardFault".into(),
        11 => "SVCall".into(),
        14 => "PendSV".into(),
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

fn ep_stat(bits: u32) -> &'static str {
    ["DISABLED", "STALL", "NAK", "VALID"][(bits & 3) as usize]
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
                length: REPORT_LEN,
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
    if bytes.len() < REPORT_LEN as usize {
        println!(
            "short report, {} bytes (an older DIAG build?): {bytes:02x?}",
            bytes.len()
        );
        return;
    }
    let word = |name: &str| {
        let i = FIELDS.iter().position(|field| *field == name).unwrap();
        u32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
    };

    println!("this boot's reset cause: {}", reset_cause(word("rcc_csr")));
    if word("magic") != MAGIC {
        println!("no record from the previous run (power-on, or it never booted a DIAG build)");
        return;
    }
    for name in &FIELDS[..FIELDS.len() - 1] {
        println!("{name:>13}: 0x{:08x} ({})", word(name), word(name));
    }

    println!();
    println!(
        "main loop: {} iterations, {} in its last whole second",
        word("loops"),
        word("loop_rate")
    );
    println!(
        "last sample: pc 0x{:08x} lr 0x{:08x} in {}",
        word("pc"),
        word("lr"),
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
    if word("ep0_rescues") != 0 {
        println!(
            "EP0 GUARD FIRED {} time(s), last at {} ms, EP0R was 0x{:04x}",
            word("ep0_rescues"),
            word("ep0_rescue_ms"),
            word("ep0_rescue_epr")
        );
    } else {
        println!("EP0 guard never fired");
    }
    let frame = word("last_frame");
    if frame & 0x100 != 0 {
        println!("last UART frame handled: type 0x{:02x}", frame & 0xff);
    }

    println!();
    let cntr = word("usb_cntr");
    println!(
        "USB CNTR: FRES={} PDWN={} LP_MODE={} FSUSP={} (all 0 when running)",
        cntr & 1,
        (cntr >> 1) & 1,
        (cntr >> 2) & 1,
        (cntr >> 3) & 1
    );
    let istr = word("usb_istr");
    println!(
        "USB ISTR: CTR={} PMAOVR={} ERR={} WKUP={} SUSP={} RESET={} SOF={} EP_ID={}",
        (istr >> 15) & 1,
        (istr >> 14) & 1,
        (istr >> 13) & 1,
        (istr >> 12) & 1,
        (istr >> 11) & 1,
        (istr >> 10) & 1,
        (istr >> 9) & 1,
        istr & 0xf
    );
    let daddr = word("usb_daddr");
    println!(
        "USB DADDR: enabled={} address={}",
        (daddr >> 7) & 1,
        daddr & 0x7f
    );
    println!(
        "USB FNR: frame {} (RXDP={} RXDM={})",
        word("usb_fnr") & 0x7ff,
        (word("usb_fnr") >> 15) & 1,
        (word("usb_fnr") >> 14) & 1
    );
    let ep = word("usb_ep0r");
    println!(
        "EP0R 0x{ep:04x}: rx={} tx={} ctr_rx={} ctr_tx={} setup={}",
        ep_stat(ep >> 12),
        ep_stat(ep >> 4),
        (ep >> 15) & 1,
        (ep >> 7) & 1,
        (ep >> 11) & 1
    );
}
