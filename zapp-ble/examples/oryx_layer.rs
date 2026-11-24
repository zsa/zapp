//! Opens the Oryx live service on the connected board, prints what it says,
//! and asks it to switch to a layer and back: the direction smart layers use.
//!
//! `RUST_LOG=zapp_ble=trace cargo run -p zapp-ble --example oryx_layer -- 1`

use std::time::{Duration, Instant};

use zapp_core::hid::CMD_SET_LAYER;

fn main() {
    env_logger::init();
    let layer: u8 = std::env::args()
        .nth(1)
        .map(|arg| arg.parse().expect("a layer number"))
        .unwrap_or(1);

    let connection = zapp_ble::oryx::open(None).expect("the Oryx service");
    let started = Instant::now();
    let mut asked = false;
    let mut released = false;
    while started.elapsed() < Duration::from_secs(8) {
        if !asked && started.elapsed() > Duration::from_secs(2) {
            println!("-- set layer {layer}");
            connection.send(CMD_SET_LAYER, &[1, layer]);
            asked = true;
        }
        if !released && started.elapsed() > Duration::from_secs(4) {
            println!("-- unset layer {layer}");
            connection.send(CMD_SET_LAYER, &[0, layer]);
            released = true;
        }
        if let Some(event) = connection.recv_timeout(Duration::from_millis(50)) {
            println!("{:>5.2}s {event:?}", started.elapsed().as_secs_f32());
        }
    }
}
