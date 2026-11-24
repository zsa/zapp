//! Prints ZSA boards as the monitor sees them. `cargo run -p zapp-ble --example watch`

use std::time::{Duration, Instant};

use zapp_ble::{backend, BleMonitor};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();

    // `--all` dumps every device the platform sees, ZSA or not, with the
    // presence each one maps to. That is the view to reach for when a board is
    // not showing up and the question is what the platform actually reported.
    if std::env::args().any(|arg| arg == "--all") {
        let Some(mut backend) = backend::for_platform(&[]) else {
            println!("no BLE backend on this platform");
            return;
        };
        for sighting in backend.sightings().expect("the platform answered") {
            println!(
                "{:?} {:>9} zsa={} {:?}",
                sighting.presence(),
                sighting.name.as_deref().unwrap_or("-"),
                zapp_ble::identity::is_zsa_board(&sighting),
                sighting.modalias.as_deref().unwrap_or(""),
            );
        }
        return;
    }

    let started = Instant::now();
    let monitor = BleMonitor::start(&[]);
    let deadline = started
        + Duration::from_secs(
            std::env::var("WATCH_SECS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(12),
        );

    while Instant::now() < deadline {
        if let Some(event) = monitor.try_recv() {
            println!("[{:>6.2}s] {event:?}", started.elapsed().as_secs_f32());
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    println!("\nsnapshot:");
    for board in monitor.snapshot() {
        println!("  {board:?}");
    }
}
