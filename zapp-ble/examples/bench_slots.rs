//! Bench tool: prints a half's MCUboot slot state over BLE, optionally resets it.
//!
//! ```text
//! cargo run -p zapp-ble --example bench_slots -- left
//! cargo run -p zapp-ble --example bench_slots -- right reset
//! ```

use std::time::Duration;

use zapp_ble::smp;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();

    let half: smp::Half = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(smp::Half::Left);
    let reset = std::env::args().nth(2).as_deref() == Some("reset");

    let mut client = smp::open(None, half)
        .expect("a bonded board")
        .with_timeout(Duration::from_secs(10));

    match client.image_state() {
        Ok(slots) => {
            for s in &slots {
                let hash: String = s.hash.iter().take(4).map(|b| format!("{b:02x}")).collect();
                println!(
                    "{half:?} image={:?} slot={} v{} {}… active={} pending={} confirmed={} bootable={}",
                    s.image, s.slot, s.version, hash, s.active, s.pending, s.confirmed, s.bootable
                );
            }
        }
        Err(e) => println!("{half:?}: image_state failed: {e}"),
    }

    if reset {
        match client.reset() {
            Ok(()) => println!("{half:?}: reset sent"),
            Err(e) => println!("{half:?}: reset: {e}"),
        }
    }
}
