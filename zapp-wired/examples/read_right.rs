//! Reads the **right** half's image state once, and prints the raw reply.
//!
//! A diagnostic, and deliberately the smallest one that can exist: it opens a
//! session, points the bridge at the right half, sends one image-state read,
//! and hex-dumps the bytes that come back before anything tries to decode
//! them. Nothing is uploaded, nothing is marked, and neither half is reset or
//! released — the board is left exactly as it was found, in its bootloader.
//!
//! It exists because a listing that will not decode is invisible from the
//! product: `zapp flash` reports "bad CBOR from the board" and throws the
//! bytes away, which is the one thing worth keeping.
//!
//! ```text
//! cargo run -p zapp-wired --example read_right
//! RUST_LOG=zapp_wired=trace cargo run -p zapp-wired --example read_right
//! ```

use std::process::ExitCode;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use zapp_smp::proto::Op;
use zapp_smp::{cbor, image, img, Half, SmpClient};
use zapp_wired::ignition::{usb::UsbLink, Halves, Ignition, IgnitionLink, SessionState};
use zapp_wired::{RightTransport, WiredError};

const BEGIN_TIMEOUT: Duration = Duration::from_secs(20);
const REPLY_TIMEOUT: Duration = Duration::from_secs(30);

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp(None)
        .init();

    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!();
            eprintln!("Failed: {err}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let link = match UsbLink::open() {
        Ok(link) => link,
        Err(WiredError::NoBootloader) => return Err("no ZSA bootloader is plugged in".to_string()),
        Err(err) => return Err(format!("could not open the bootloader: {err}")),
    };

    let shared = Shared::new(link);
    let mut ignition = Ignition::new(shared.clone());

    println!("Opening a session on both halves");
    ignition
        .begin(Halves::BOTH)
        .map_err(|err| format!("BEGIN was refused: {err}"))?;
    let status = wait_active(&mut ignition)?;
    println!(
        "  active · {} bytes per message · pipe v{} · TRRS {} kHz",
        status.max_payload, status.proto_version, status.i2c_khz
    );

    ignition
        .select(Half::Right)
        .map_err(|err| format!("SELECT was refused: {err}"))?;
    // The same resync the product sends: a run that died mid-line leaves
    // MCUboot holding a fragment, and the next request lands on the back of it.
    if let Err(err) = ignition.send(Half::Right.into(), b"\n") {
        println!("  (the resync newline was refused: {err})");
    }

    let mut client = SmpClient::new(Box::new(RightTransport::with_payload_budget(
        Ignition::new(shared.clone()),
        status.payload_budget(),
    )))
    .with_timeout(REPLY_TIMEOUT);

    println!();
    println!("Reading the right half's image state");
    let empty = cbor::encode(&cbor::map(vec![]));
    let body = client
        .transact(Op::Read, img::GROUP, img::CMD_STATE, &empty)
        .map_err(|err| format!("the read itself failed: {err}"))?;

    println!("  {} bytes of payload came back", body.len());
    println!();
    dump(&body);

    println!();
    match img::parse_state(&body) {
        Ok(slots) => {
            println!("It decodes. {} slot(s):", slots.len());
            for slot in &slots {
                println!(
                    "  slot {} · {} · {} · {}",
                    slot.slot,
                    slot.version,
                    image::hex(&slot.hash),
                    slot.flags().join(" ")
                );
            }
        }
        Err(err) => println!("It does not decode: {err}"),
    }

    // Nothing is put back on purpose: no RELEASE_MD, no reset of either half.
    // A reset of the right is the one thing that must not happen here — a
    // permanent mark waiting in its spare slot would be swapped in by it.
    println!();
    ignition
        .end()
        .map_err(|err| format!("END was refused: {err}"))?;
    println!("Session ended. Both halves are still in their bootloaders.");
    Ok(())
}

/// The bytes, 16 to a line, with the printable ones alongside.
fn dump(body: &[u8]) {
    for (index, chunk) in body.chunks(16).enumerate() {
        let hex: Vec<String> = chunk.iter().map(|byte| format!("{byte:02x}")).collect();
        let text: String = chunk
            .iter()
            .map(|byte| match byte.is_ascii_graphic() || *byte == b' ' {
                true => *byte as char,
                false => '.',
            })
            .collect();
        println!("  {:04x}  {:<47}  {text}", index * 16, hex.join(" "));
    }
}

fn wait_active<L: IgnitionLink>(ignition: &mut Ignition<L>) -> Result<zapp_wired::Status, String> {
    let started = Instant::now();
    loop {
        let status = ignition
            .status()
            .map_err(|err| format!("could not read STATUS: {err}"))?;
        match status.state {
            SessionState::Active => return Ok(status),
            SessionState::Pending => {}
            SessionState::Fault => {
                return Err(format!(
                    "the bootloader faulted opening the session (error {:#06x})",
                    status.error
                ))
            }
            SessionState::Idle => return Err("the bootloader says no session is open".to_string()),
            SessionState::Unrecognised(code) => {
                return Err(format!("the bootloader reports session state {code}"))
            }
        }
        if started.elapsed() >= BEGIN_TIMEOUT {
            return Err(format!(
                "the session was still opening after {} s",
                BEGIN_TIMEOUT.as_secs()
            ));
        }
    }
}

struct Shared<L: IgnitionLink>(Arc<Mutex<L>>);

impl<L: IgnitionLink> Shared<L> {
    fn new(link: L) -> Self {
        Self(Arc::new(Mutex::new(link)))
    }
}

impl<L: IgnitionLink> Clone for Shared<L> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<L: IgnitionLink> IgnitionLink for Shared<L> {
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
        self.0.lock().unwrap().control_out(request, value)
    }

    fn control_in(
        &mut self,
        request: u8,
        value: u16,
        length: usize,
    ) -> Result<Vec<u8>, WiredError> {
        self.0.lock().unwrap().control_in(request, value, length)
    }

    fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
        self.0.lock().unwrap().bulk_out(message)
    }

    fn bulk_in(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError> {
        self.0.lock().unwrap().bulk_in(timeout)
    }
}
