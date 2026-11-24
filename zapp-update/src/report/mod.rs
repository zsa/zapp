//! The Wireless Voyager report file: the board's versions, its diagnostic
//! records, and the left half's freeze record and event log. `zapp report`
//! writes it, and so can any front end holding a session on the board.

use std::fmt::Write as _;

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD as BASE64;

use zapp_core::hid::{BLOB_EVENTS, BLOB_FREEZE, BlobRead, HidConnection};

pub mod devices;
pub mod diag;
pub mod freeze;

/// What goes into the file.
pub struct Parts {
    /// The program and its version, e.g. "zapp 1.1.14".
    pub tool: String,
    pub written: String,
    pub devices: Vec<String>,
    /// The `zapp diag` text, or why it could not be read.
    pub diag: Result<String, String>,
    pub freeze_summary: Option<String>,
    pub freeze: BlobRead,
    pub events: BlobRead,
}

/// The file's text. Blob lines are `freeze: <base64>` and `events: <base64>`,
/// present only when read.
pub fn render(parts: &Parts) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "{} report", parts.tool);
    let _ = writeln!(out, "written: {}", parts.written);

    let _ = writeln!(out, "\n== devices");
    for line in &parts.devices {
        let _ = writeln!(out, "{line}");
    }

    let _ = writeln!(out, "\n== diag");
    match &parts.diag {
        Ok(text) => out.push_str(text),
        Err(err) => {
            let _ = writeln!(out, "not read: {err}");
        }
    }

    let _ = writeln!(out, "\n== left half blobs");
    if let Some(summary) = &parts.freeze_summary {
        let _ = writeln!(out, "freeze summary: {summary}");
    }
    for (name, read) in [("freeze", &parts.freeze), ("events", &parts.events)] {
        if let BlobRead::Data(bytes) = read {
            let _ = writeln!(out, "{name}: {}", BASE64.encode(bytes));
        }
    }
    out
}

/// "zapp-report-2026-11-10-140322.txt" for `prefix` "zapp" and `stamp`
/// "2026-11-10-140322".
pub fn file_name(prefix: &str, stamp: &str) -> String {
    format!("{prefix}-report-{stamp}.txt")
}

/// What a session already knows about the board it is on.
#[derive(Debug, Clone, Default)]
pub struct Board {
    /// "Wireless Voyager".
    pub product: String,
    /// The USB serial, `None` over Bluetooth.
    pub serial: Option<String>,
    /// Layout and revision from the handshake.
    pub firmware: Option<(String, String)>,
    pub protocol: Option<u8>,
}

/// Everything [`render`] needs, read over a connection whose handshake is
/// done, on either transport. The same reads `zapp report` makes.
pub fn collect_on(conn: &HidConnection, tool: &str, written: &str, board: &Board) -> Parts {
    let wired = devices::wired_report_from(
        crate::probe::report_on(conn, board.protocol, true, true, true),
        true,
    );
    let name = match board.serial.as_deref().filter(|serial| !serial.is_empty()) {
        Some(serial) => format!("{} · {serial}", board.product),
        None => board.product.clone(),
    };
    let mut lines = vec![format!("{name}{}", wired.charge)];
    lines.extend(wired.lines.iter().map(|line| format!("  {line}")));
    if let Some(radio) = &wired.radio {
        lines.push(format!("  {}", devices::radio_line(radio, true)));
    }

    let diag = diag::read_on(
        conn,
        tool,
        &board.product,
        board.serial.clone(),
        board.firmware.clone(),
        board.protocol,
    )
    .map(|read| read.text)
    .map_err(|err| format!("{err:#}"));
    let freeze_read = freeze::read(conn, BLOB_FREEZE, usize::MAX);
    let events_read = freeze::read(conn, BLOB_EVENTS, usize::MAX);

    Parts {
        tool: tool.to_string(),
        written: written.to_string(),
        devices: lines,
        diag,
        freeze_summary: freeze::header_from(&freeze_read)
            .as_ref()
            .map(freeze::summary),
        freeze: freeze_read,
        events: events_read,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_file_name_carries_the_prefix_and_stamp() {
        assert_eq!(
            file_name("keymapp", "2026-11-10-140322"),
            "keymapp-report-2026-11-10-140322.txt"
        );
    }

    /// A board that answers nothing.
    struct Silent;

    impl zapp_core::hid::ReportLink for Silent {
        fn send(&mut self, _report: &[u8; 32]) -> std::io::Result<()> {
            Ok(())
        }

        fn receive(
            &mut self,
            timeout: std::time::Duration,
        ) -> std::io::Result<Option<Vec<u8>>> {
            std::thread::sleep(timeout);
            Ok(None)
        }
    }

    #[test]
    fn a_silent_board_still_gives_every_section() {
        let conn = HidConnection::over(Silent);
        let board = Board {
            product: "Wireless Voyager".into(),
            serial: None,
            firmware: Some(("abc".into(), "def".into())),
            protocol: Some(7),
        };
        let parts = collect_on(&conn, "keymapp 0.1.0", "2026-11-10 14:03:22 +07:00", &board);
        assert_eq!(parts.devices, ["Wireless Voyager"]);
        assert!(parts.diag.is_err());
        assert_eq!(parts.freeze, BlobRead::Unsupported);
        assert_eq!(parts.events, BlobRead::Unsupported);
        let text = render(&parts);
        assert!(text.starts_with("keymapp 0.1.0 report\nwritten: 2026-11-10 14:03:22 +07:00\n"));
        assert!(text.ends_with("== left half blobs\n"));
    }
}
