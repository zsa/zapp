//! `zapp report`: one plain-text file with the board's versions, its
//! diagnostic records, and the left half's freeze record and event log.

use std::path::PathBuf;

use anyhow::{Context, Result};

use zapp_core::device;
use zapp_core::hid::{BLOB_EVENTS, BLOB_FREEZE, BLOB_OK, BlobRead, parse_freeze_header};
use zapp_update::report::{Parts, file_name, render};

use crate::{describe_usb, diag, freeze, radio_line, wired_report};

/// Writes the report and marks the freeze record as reported.
pub fn run(output: Option<PathBuf>) -> Result<()> {
    let devices = device::list_devices().context("could not list USB devices")?;
    let (device, count) = diag::pick(&devices)?;
    let now = jiff::Zoned::now();

    let wired = wired_report(device, true);
    let mut devices_lines = vec![format!("{}{}", describe_usb(device), wired.charge)];
    devices_lines.extend(wired.lines.iter().map(|line| format!("  {line}")));
    if let Some(radio) = &wired.radio {
        devices_lines.push(format!("  {}", radio_line(radio, true)));
    }

    let (conn, read) = diag::read(device, count)?;
    let diag = read.map(|read| read.text).map_err(|err| format!("{err:#}"));
    let freeze_read = freeze::read(&conn, BLOB_FREEZE, usize::MAX);
    let events_read = freeze::read(&conn, BLOB_EVENTS, usize::MAX);
    let header = match &freeze_read {
        BlobRead::Data(bytes) => parse_freeze_header(bytes),
        _ => None,
    };

    let parts = Parts {
        tool: diag::tool(),
        written: now.strftime("%Y-%m-%d %H:%M:%S %:z").to_string(),
        devices: devices_lines,
        diag,
        freeze_summary: header.as_ref().map(freeze::summary),
        freeze: freeze_read,
        events: events_read,
    };
    let path = output.unwrap_or_else(|| {
        PathBuf::from(file_name(
            "zapp",
            &now.strftime("%Y-%m-%d-%H%M%S").to_string(),
        ))
    });
    let written = std::fs::write(&path, render(&parts))
        .with_context(|| format!("could not write {}", path.display()));
    if let Err(err) = written {
        conn.close();
        return Err(err);
    }

    for line in outcome_lines(&parts) {
        println!("{line}");
    }
    let shown = std::path::absolute(&path).unwrap_or_else(|_| path.clone());
    println!("Report written to {}.", shown.display());

    if header.is_some() {
        match freeze::mark_seen(&conn) {
            Some(BLOB_OK) => println!("Freeze record marked as reported."),
            Some(status) => println!(
                "Freeze record not marked as reported: {}.",
                freeze::status_text(status)
            ),
            None => println!("Freeze record not marked as reported: no answer."),
        }
    }
    conn.close();
    Ok(())
}

/// The lines printed besides the file's path: the freeze record, and what was
/// not read.
fn outcome_lines(parts: &Parts) -> Vec<String> {
    let mut lines = Vec::new();
    if matches!(parts.freeze, BlobRead::Unsupported)
        && matches!(parts.events, BlobRead::Unsupported)
    {
        lines.push("Freeze record and event log were not available.".to_string());
        return lines;
    }
    match (&parts.freeze, &parts.freeze_summary) {
        (BlobRead::Data(_), Some(summary)) => lines.push(format!("Left half: {summary}.")),
        (BlobRead::Data(bytes), None) => lines.push(format!(
            "Left half: freeze record of {} bytes without a known header.",
            bytes.len()
        )),
        (BlobRead::None, _) => lines.push("Left half: no freeze record.".to_string()),
        (read, _) => {
            if let Some(problem) = freeze::problem(read) {
                lines.push(format!("Freeze record not read: {problem}."));
            }
        }
    }
    if let Some(problem) = freeze::problem(&parts.events) {
        lines.push(format!("Event log not read: {problem}."));
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parts(freeze: BlobRead, events: BlobRead) -> Parts {
        Parts {
            tool: format!("zapp {}", env!("CARGO_PKG_VERSION")),
            written: "2026-11-10 14:03:22 +07:00".into(),
            devices: vec![
                "Wireless Voyager · ABC".into(),
                "  radio    · left 0.2.2".into(),
            ],
            diag: Ok("zapp 1.1.14 diagnostics\nkeyboard: Wireless Voyager\n".into()),
            freeze_summary: None,
            freeze,
            events,
        }
    }

    #[test]
    fn report_text_shape() {
        let text = render(&parts(
            BlobRead::Data(vec![0x5A, 0x46, 0x52, 0x5A]),
            BlobRead::Data(vec![1, 2, 3]),
        ));
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines[0],
            format!("zapp {} report", env!("CARGO_PKG_VERSION"))
        );
        assert_eq!(lines[1], "written: 2026-11-10 14:03:22 +07:00");
        assert_eq!(
            &lines[2..],
            [
                "",
                "== devices",
                "Wireless Voyager · ABC",
                "  radio    · left 0.2.2",
                "",
                "== diag",
                "zapp 1.1.14 diagnostics",
                "keyboard: Wireless Voyager",
                "",
                "== left half blobs",
                "freeze: WkZSWg==",
                "events: AQID",
            ]
        );
    }

    #[test]
    fn missing_blobs_leave_no_line() {
        let text = render(&parts(BlobRead::None, BlobRead::Unsupported));
        assert!(text.ends_with("== left half blobs\n"));
        assert!(!text.contains("freeze:") && !text.contains("events:"));
    }

    #[test]
    fn outcome_lines_are_facts() {
        let both = parts(BlobRead::Unsupported, BlobRead::Unsupported);
        assert_eq!(
            outcome_lines(&both),
            ["Freeze record and event log were not available."]
        );
        let none = parts(BlobRead::None, BlobRead::Data(vec![1]));
        assert_eq!(outcome_lines(&none), ["Left half: no freeze record."]);
        let failed = parts(
            BlobRead::Failed(zapp_core::hid::BLOB_NO_LINK),
            BlobRead::Failed(zapp_core::hid::BLOB_TIMEOUT),
        );
        assert_eq!(
            outcome_lines(&failed),
            [
                "Freeze record not read: no link to the left half.",
                "Event log not read: the left half did not answer.",
            ]
        );
    }
}
