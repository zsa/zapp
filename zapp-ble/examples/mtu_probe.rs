//! Temporary: finds the largest SMP request the board actually answers.
//!
//! `mtu_probe <lo> <hi> [step] [write_mtu] [half]`. With no write MTU each request is
//! one write of exactly its own size, which is what pins the ATT boundary; with
//! one, requests larger than it are fragmented, which is what pins the
//! reassembly boundary. Uses os_mgmt echo, so nothing on the board is written.

use std::time::Duration;

use ciborium::value::Value;
use zapp_ble::smp::{self, cbor, proto::Op};

/// An echo request whose whole frame — 8-byte header included — is `total`.
fn echo_payload(total: usize) -> Option<Vec<u8>> {
    for text in 0..total {
        let payload = cbor::encode(&cbor::map(vec![("d", Value::Text("A".repeat(text)))]));
        if 8 + payload.len() == total {
            return Some(payload);
        }
    }
    None
}

fn arg<T: std::str::FromStr>(n: usize, default: T) -> T {
    std::env::args()
        .nth(n)
        .and_then(|arg| arg.parse().ok())
        .unwrap_or(default)
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp(None)
        .init();

    let half: smp::Half = arg(5, smp::Half::Left);
    let mut client = smp::open(None, half).expect("a bonded board");
    println!("channel reports {} bytes per write", client.mtu());

    let lo: usize = arg(1, 240);
    let hi: usize = arg(2, 256);
    let step: usize = arg(3, 1);
    let write_mtu: usize = arg(4, 0);

    let mut total = lo;
    while total <= hi {
        let Some(payload) = echo_payload(total) else {
            println!("{total:5}: no exact encoding");
            total += step;
            continue;
        };
        let mtu = if write_mtu == 0 { total } else { write_mtu };
        client = client.with_mtu(mtu).with_timeout(Duration::from_secs(5));
        let started = std::time::Instant::now();
        let outcome = match client.transact(Op::Write, 0, 0, &payload) {
            Ok(reply) => format!("ok ({} bytes back)", reply.len()),
            Err(err) => format!("FAILED: {err}"),
        };
        println!(
            "{total:5} in {:2} writes: {outcome} [{:?}]",
            total.div_ceil(mtu),
            started.elapsed()
        );
        total += step;
    }
}
