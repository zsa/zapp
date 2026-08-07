//! Prints ZSA boards as the monitor sees them. `cargo run -p zapp-ble --example watch`

use std::time::{Duration, Instant};

use zapp_ble::{backend, BleMonitor};

fn main() {
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

    let monitor = BleMonitor::start(&[]);
    let deadline = Instant::now() + Duration::from_secs(12);

    while Instant::now() < deadline {
        if let Some(event) = monitor.try_recv() {
            println!("{event:?}");
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    println!("\nsnapshot:");
    for board in monitor.snapshot() {
        println!("  {board:?}");
    }
}
