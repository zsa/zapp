//! The left half's freeze record and event log, read over the raw HID
//! protocol on either transport.

use std::thread;
use std::time::{Duration, Instant};

use zapp_core::hid::{
    BLOB_BUSY, BLOB_FREEZE, BLOB_NO_LINK, BLOB_OK, BLOB_TIMEOUT, BlobChunk, BlobRead,
    CMD_FREEZE_SEEN, CMD_GET_BLOB, ERR_UNKNOWN_COMMAND, FREEZE_HEADER_SIZE, FreezeHeader,
    FreezeKind, HidConnection, HidEvent, blob_request, parse_freeze_header, read_blob,
};

/// Longest wait for one chunk; the board gives the left nRF 3 x 50 ms.
const CHUNK_WAIT: Duration = Duration::from_millis(1000);
/// Tries per chunk while the board answers `BLOB_BUSY`.
const BUSY_TRIES: usize = 3;
const BUSY_PAUSE: Duration = Duration::from_millis(200);

/// Reads blob `which`, at most `limit` bytes of it.
pub fn read(conn: &HidConnection, which: u8, limit: usize) -> BlobRead {
    read_with(conn, which, limit, &mut |_| {})
}

/// [`read`], handing every other event, `Disconnected` included, to `other`.
pub fn read_with(
    conn: &HidConnection,
    which: u8,
    limit: usize,
    other: &mut dyn FnMut(HidEvent),
) -> BlobRead {
    read_blob(which, limit, |offset| ask(conn, which, offset, other))
}

/// One chunk: `None` for no answer, `Some(Err(()))` for an unknown command.
fn ask(
    conn: &HidConnection,
    which: u8,
    offset: u16,
    other: &mut dyn FnMut(HidEvent),
) -> Option<Result<BlobChunk, ()>> {
    let mut answer = None;
    for _ in 0..BUSY_TRIES {
        conn.send(CMD_GET_BLOB, &blob_request(which, offset));
        answer = wait(conn, other, |event| match event {
            HidEvent::Blob(chunk) if chunk.which == which && chunk.offset == offset => {
                Some(Ok(chunk))
            }
            HidEvent::Error(ERR_UNKNOWN_COMMAND) => Some(Err(())),
            _ => None,
        });
        match &answer {
            Some(Ok(chunk)) if chunk.status == BLOB_BUSY => thread::sleep(BUSY_PAUSE),
            _ => break,
        }
    }
    answer
}

/// Waits up to `CHUNK_WAIT` for the event `take` is looking for; the rest go
/// to `other`.
fn wait<T>(
    conn: &HidConnection,
    other: &mut dyn FnMut(HidEvent),
    take: impl Fn(HidEvent) -> Option<T>,
) -> Option<T> {
    let deadline = Instant::now() + CHUNK_WAIT;
    loop {
        let left = deadline.checked_duration_since(Instant::now())?;
        match conn.recv_timeout(left)? {
            HidEvent::Disconnected => {
                other(HidEvent::Disconnected);
                return None;
            }
            event => {
                if let Some(answer) = take(event.clone()) {
                    return Some(answer);
                }
                other(event);
            }
        }
    }
}

/// The freeze record's header alone: two chunks.
pub fn header(conn: &HidConnection) -> Option<FreezeHeader> {
    header_from(&read(conn, BLOB_FREEZE, FREEZE_HEADER_SIZE))
}

/// The header out of a read of the freeze record.
pub fn header_from(read: &BlobRead) -> Option<FreezeHeader> {
    match read {
        BlobRead::Data(bytes) => parse_freeze_header(bytes),
        _ => None,
    }
}

/// The notice line for an unread freeze record, if there is one.
pub fn notice(conn: &HidConnection) -> Option<String> {
    header(conn)
        .filter(FreezeHeader::unread)
        .map(|header| notice_line(&header))
}

pub fn notice_line(header: &FreezeHeader) -> String {
    format!(
        "Left half: {} recorded at uptime {}, not yet reported.",
        kind(header),
        uptime(header.uptime_ms)
    )
}

/// Sends `CMD_FREEZE_SEEN`; the board's status, `None` without an answer.
pub fn mark_seen(conn: &HidConnection) -> Option<u8> {
    mark_seen_with(conn, &mut |_| {})
}

/// [`mark_seen`], handing every other event to `other`.
pub fn mark_seen_with(conn: &HidConnection, other: &mut dyn FnMut(HidEvent)) -> Option<u8> {
    conn.send(CMD_FREEZE_SEEN, &[]);
    wait(conn, other, |event| match event {
        HidEvent::FreezeSeen(status) => Some(Some(status)),
        HidEvent::Error(ERR_UNKNOWN_COMMAND) => Some(None),
        _ => None,
    })
    .flatten()
}

/// "freeze at uptime 20 min 13 s · stalled hid_wq, slow_wq · flags unread ·
/// 3 freezes since".
pub fn summary(header: &FreezeHeader) -> String {
    let mut line = format!("{} at uptime {}", kind(header), uptime(header.uptime_ms));
    let stalled = header.stalled_names();
    if !stalled.is_empty() {
        line.push_str(&format!(" · stalled {}", stalled.join(", ")));
    }
    let flags = header.flag_names();
    if !flags.is_empty() {
        line.push_str(&format!(" · flags {}", flags.join(", ")));
    }
    if header.freezes > 1 {
        line.push_str(&format!(" · {} freezes since", header.freezes));
    }
    line
}

fn kind(header: &FreezeHeader) -> String {
    match header.kind {
        FreezeKind::Freeze => "freeze".to_string(),
        FreezeKind::Fatal => format!("fatal error {}", header.fatal_reason),
        FreezeKind::Other(kind) => format!("record kind {kind}"),
    }
}

/// "1 h 2 min 3 s", "20 min 13 s", "13 s".
pub fn uptime(ms: u32) -> String {
    let secs = ms / 1000;
    let (h, min, s) = (secs / 3600, secs / 60 % 60, secs % 60);
    match (h, min) {
        (0, 0) => format!("{s} s"),
        (0, _) => format!("{min} min {s} s"),
        _ => format!("{h} h {min} min {s} s"),
    }
}

/// Why a read gave no data, as a fact; `None` for data or no blob.
pub fn problem(read: &BlobRead) -> Option<String> {
    match read {
        BlobRead::Data(_) | BlobRead::None => None,
        BlobRead::Unsupported => Some("the keyboard firmware did not answer".into()),
        BlobRead::Failed(status) => Some(status_text(*status)),
        BlobRead::Inconsistent => Some("its chunks did not match".into()),
    }
}

pub fn status_text(status: u8) -> String {
    match status {
        BLOB_OK => "ok".into(),
        BLOB_TIMEOUT => "the left half did not answer".into(),
        BLOB_NO_LINK => "no link to the left half".into(),
        BLOB_BUSY => "the keyboard was busy".into(),
        other => format!("status {other:#04x}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zapp_core::hid::FREEZE_HEADER_SIZE;

    fn header(kind: FreezeKind, flags: u8) -> FreezeHeader {
        FreezeHeader {
            layout: 1,
            total_len: FREEZE_HEADER_SIZE as u16,
            kind,
            stalled: 0x06,
            flags,
            freezes: 1,
            uptime_ms: 1_213_500,
            fatal_reason: 4,
            version: [0, 2, 3],
            build: 0,
            hash: [0; 8],
        }
    }

    #[test]
    fn uptime_reads_in_units() {
        assert_eq!(uptime(13_999), "13 s");
        assert_eq!(uptime(1_213_000), "20 min 13 s");
        assert_eq!(uptime(3_723_000), "1 h 2 min 3 s");
    }

    #[test]
    fn notice_and_summary_lines() {
        let freeze = header(FreezeKind::Freeze, 0x05);
        assert_eq!(
            notice_line(&freeze),
            "Left half: freeze recorded at uptime 20 min 13 s, not yet reported."
        );
        assert_eq!(
            summary(&freeze),
            "freeze at uptime 20 min 13 s · stalled hid_wq, slow_wq · flags unread, restored from flash"
        );
        let mut fatal = header(FreezeKind::Fatal, 0);
        fatal.stalled = 0;
        fatal.freezes = 3;
        assert_eq!(
            summary(&fatal),
            "fatal error 4 at uptime 20 min 13 s · 3 freezes since"
        );
    }
}
