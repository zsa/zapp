//! `zapp diag`: the Wireless Voyager's diagnostic records, read over USB raw HID.

use std::fmt::Write as _;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};

use zapp_core::device::ids::target_name_for_pid;
use zapp_core::device::{self, Keyboard, UsbDevice};
use zapp_core::hid::{
    CMD_GET_DIAG, DIAG_AGE_UNKNOWN, DIAG_FETCH, DIAG_NEVER, DIAG_PAGE_VERSION, DiagPage,
    DiagRecord, DiagStatus, HidConnection, HidEvent, LEFT_DIAG_BLE_HOST, LEFT_DIAG_STM32_LINK,
    LEFT_DIAG_STM32_SLEEP, LEFT_DIAG_STM32_TRRS, LEFT_DIAG_STM32_USB, LEFT_DIAG_THREADS,
    LEFT_DIAG_UART_RX_OFF, LEFT_DIAG_VBUS, LeftDiag, NRF_RESET_DOG0, NRF_RESET_DOG1,
    NRF_RESET_LOCKUP, NRF_RESET_SREQ, STM32_DIAG_BLE_HOST, STM32_DIAG_LINK_UP, STM32_DIAG_NRF_VBUS,
    STM32_DIAG_RIGHT_LEDS, STM32_DIAG_TRRS, STM32_DIAG_USB_HOST, STM32_DIAG_VBUS_SEEN,
    STM32_RESET_IWDG, STM32_RESET_WWDG, Stm32Diag, assemble_diag_record, diag_page_count,
    diag_request, nrf_reset_causes, parse_left_diag, parse_stm32_diag, stm32_reset_causes,
};

/// Longest wait for one reply; the board answers from its cache.
const REPLY_WAIT: Duration = Duration::from_millis(1500);
/// Time given to the left nRF to send its records after the first fetch.
const FETCH_SETTLE: Duration = Duration::from_millis(400);
/// Same, after the second fetch.
const REFETCH_SETTLE: Duration = Duration::from_secs(1);
/// Slack on a left page's age before it counts as older than the fetch.
const FRESH_SLACK_MS: u128 = 100;

const NO_ANSWER: &str = "The keyboard firmware did not answer the diagnostic request. \
    It may predate this feature; update it and retry.";

/// Reads every record off the attached Wireless Voyager and prints them.
pub fn run() -> Result<()> {
    let devices = device::list_devices().context("could not list USB devices")?;
    let (device, count) = pick(&devices)?;
    let (conn, read) = read(device, count)?;
    let notice = match &read {
        Ok(_) => crate::freeze::notice(&conn),
        Err(_) => None,
    };
    conn.close();

    let read = read?;
    print!("{}", read.text);
    if let Some(notice) = notice {
        println!("{notice}");
    }
    match read.stm32_problem {
        None => Ok(()),
        Some(problem) => bail!(problem),
    }
}

/// The rendered records, and what is wrong with the STM32 one.
pub(crate) struct Read {
    pub text: String,
    pub stm32_problem: Option<String>,
}

/// Reads every record over a new connection, which is handed back open.
pub(crate) fn read(device: &UsbDevice, count: usize) -> Result<(HidConnection, Result<Read>)> {
    let conn = HidConnection::open(device.vid, device.pid)
        .context("could not open the keyboard's raw HID interface")?;
    let mut session = Session::new(conn);
    let report = session.collect();
    let header = Header {
        product: target_name_for_pid(device.pid).to_string(),
        serial: device.serial.clone(),
        others: count - 1,
        firmware: session.firmware.take(),
        protocol: session.protocol,
    };
    let read = report.map(|report| Read {
        text: render(&header, &report),
        stm32_problem: match &report.stm32 {
            Found::Record { .. } => None,
            Found::NoRecord => Some("The keyboard MCU (STM32) has no record for this boot.".into()),
            Found::Problem(problem) => Some(format!(
                "Could not read the keyboard MCU (STM32) record: {problem}."
            )),
        },
    });
    Ok((session.conn, read))
}

/// The first Wireless Voyager running its keyboard firmware, and how many there are.
pub(crate) fn pick(devices: &[UsbDevice]) -> Result<(&UsbDevice, usize)> {
    let voyagers: Vec<&UsbDevice> = devices
        .iter()
        .filter(|device| device.keyboard == Some(Keyboard::Voyager2))
        .collect();
    let running: Vec<&UsbDevice> = voyagers
        .iter()
        .copied()
        .filter(|device| device.bootloader.is_none())
        .collect();
    match running.first() {
        Some(device) => Ok((device, running.len())),
        None if voyagers.is_empty() => bail!("No Wireless Voyager found on USB."),
        None => bail!(
            "The Wireless Voyager is in its bootloader. Restart it into the keyboard \
             firmware (hold any left-half key for 5 seconds, or run a zapp command that \
             restarts it), then run `zapp diag` again."
        ),
    }
}

/// What the connection says about the board besides the records.
struct Header {
    product: String,
    serial: Option<String>,
    /// Further Wireless Voyager keyboards on USB, not read.
    others: usize,
    firmware: Option<(String, String)>,
    protocol: Option<u8>,
}

/// One record as read.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Found<T> {
    /// `stale_ms`: age of the oldest page that predates the fetch.
    Record {
        diag: T,
        stale_ms: Option<u16>,
    },
    NoRecord,
    Problem(String),
}

impl<T> Found<T> {
    fn fresh(&self) -> bool {
        matches!(self, Found::Record { stale_ms: None, .. })
    }
}

/// The four records.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Report {
    stm32: Found<Stm32Diag>,
    stm32_previous: Found<Stm32Diag>,
    left: Found<LeftDiag>,
    left_previous: Found<LeftDiag>,
}

/// A raw HID connection and the handshake events seen on it.
struct Session {
    conn: HidConnection,
    firmware: Option<(String, String)>,
    protocol: Option<u8>,
}

impl Session {
    fn new(conn: HidConnection) -> Self {
        Self {
            conn,
            firmware: None,
            protocol: None,
        }
    }

    /// Sends one request and waits for its page. A version 0 page answers any
    /// request.
    fn ask(&mut self, record: DiagRecord, page: u8, flags: u8) -> Option<DiagPage> {
        self.conn
            .send(CMD_GET_DIAG, &diag_request(record, page, flags));

        let deadline = Instant::now() + REPLY_WAIT;
        loop {
            let left = deadline.checked_duration_since(Instant::now())?;
            match self.conn.recv_timeout(left)? {
                HidEvent::Disconnected => return None,
                HidEvent::FirmwareVersion {
                    layout_id,
                    revision_id,
                } => self.firmware = Some((layout_id, revision_id)),
                HidEvent::ProtocolVersion(version) => self.protocol = Some(version),
                HidEvent::Diag(reply)
                    if reply.version == 0
                        || (reply.record == record.id() && reply.page == page) =>
                {
                    return Some(reply);
                }
                _ => {}
            }
        }
    }

    /// Asks the board to re-fetch the left records; returns the reply and the
    /// time the request went out.
    fn fetch(&mut self) -> Option<(DiagPage, Instant)> {
        let sent = Instant::now();
        let reply = self.ask(DiagRecord::Left, 0, DIAG_FETCH)?;
        Some((reply, sent))
    }

    /// Fetch, read all four records, and fetch once more when the left nRF's
    /// current record is missing or older than the fetch.
    fn collect(&mut self) -> Result<Report> {
        let Some((reply, mut fetched)) = self.fetch() else {
            bail!(NO_ANSWER);
        };
        match reply.version {
            DIAG_PAGE_VERSION => {}
            0 => bail!("The keyboard firmware has no diagnostics. Update it and retry."),
            other => bail!(
                "The keyboard answers in diagnostic format {other}, which this zapp cannot \
                 read. Update zapp."
            ),
        }
        thread::sleep(FETCH_SETTLE);

        let stm32 = self.read(DiagRecord::Stm32, None, parse_stm32_diag);
        let stm32_previous = self.read(DiagRecord::Stm32Previous, None, parse_stm32_diag);
        let mut left = self.read(DiagRecord::Left, Some(fetched), parse_left_diag);
        let mut left_previous = self.read(DiagRecord::LeftPrevious, Some(fetched), parse_left_diag);

        if !left.fresh() {
            if let Some((_, sent)) = self.fetch() {
                fetched = sent;
                thread::sleep(REFETCH_SETTLE);
                left = self.read(DiagRecord::Left, Some(fetched), parse_left_diag);
                left_previous = self.read(DiagRecord::LeftPrevious, Some(fetched), parse_left_diag);
            } else {
                log::debug!("the second fetch went unanswered");
            }
        }

        Ok(Report {
            stm32,
            stm32_previous,
            left,
            left_previous,
        })
    }

    /// Reads one record page by page. `fetched` is when the left records were
    /// last asked for; pages older than that are stale.
    fn read<T>(
        &mut self,
        record: DiagRecord,
        fetched: Option<Instant>,
        decode: fn(&[u8]) -> Option<T>,
    ) -> Found<T> {
        let mut pages = Vec::new();
        let mut stale_ms: Option<u16> = None;
        let mut count = 1;
        let mut index = 0;
        while index < count {
            let Some(page) = self.ask(record, index, 0) else {
                return Found::Problem(format!("no answer for page {index}"));
            };
            let arrived = Instant::now();
            if page.version != DIAG_PAGE_VERSION {
                return Found::Problem(format!("page format {} not understood", page.version));
            }
            match page.status {
                DiagStatus::Ok => {}
                DiagStatus::NoRecord if index == 0 => return Found::NoRecord,
                DiagStatus::NoRecord => return Found::Problem("the record went away".into()),
                DiagStatus::OutOfRange => {
                    return Found::Problem(format!("page {index} out of range"));
                }
                DiagStatus::Other(status) => {
                    return Found::Problem(format!("unknown status {status}"));
                }
            }
            if index == 0 {
                count = diag_page_count(page.total_len).max(1);
            }
            if let Some(fetched) = fetched
                && !fresh(page.age_ms, arrived.duration_since(fetched))
            {
                stale_ms = stale_ms.max(Some(page.age_ms));
            }
            pages.push(page);
            index += 1;
        }

        let Some(bytes) = assemble_diag_record(&pages) else {
            return Found::Problem("the record changed while it was read".into());
        };
        match decode(&bytes) {
            Some(diag) => Found::Record { diag, stale_ms },
            None => Found::Problem(format!(
                "record format {} ({} bytes) not understood; update zapp",
                bytes.first().copied().unwrap_or(0),
                bytes.len()
            )),
        }
    }
}

/// Whether a left page arrived from the nRF after the fetch went out.
fn fresh(age_ms: u16, since_fetch: Duration) -> bool {
    age_ms != DIAG_AGE_UNKNOWN && u128::from(age_ms) <= since_fetch.as_millis() + FRESH_SLACK_MS
}

/// The whole report as plain text.
fn render(header: &Header, report: &Report) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "zapp {} diagnostics", env!("CARGO_PKG_VERSION"));
    let serial = header.serial.as_deref().filter(|serial| !serial.is_empty());
    let _ = match serial {
        Some(serial) => writeln!(out, "keyboard: {} (USB serial {serial})", header.product),
        None => writeln!(out, "keyboard: {}", header.product),
    };
    if header.others > 0 {
        let _ = writeln!(
            out,
            "note: {} more Wireless Voyager on USB, not read",
            header.others
        );
    }
    let _ = match &header.firmware {
        Some((layout, revision)) => {
            writeln!(out, "firmware: layout {layout}, revision {revision}")
        }
        None => writeln!(out, "firmware: did not say"),
    };
    let _ = match header.protocol {
        Some(version) => writeln!(out, "Oryx protocol: {version:#04x}"),
        None => writeln!(out, "Oryx protocol: did not say"),
    };

    section(
        &mut out,
        "Keyboard MCU (STM32), this boot",
        &report.stm32,
        stm32_lines,
    );
    section(
        &mut out,
        "Keyboard MCU (STM32), previous boot",
        &report.stm32_previous,
        stm32_lines,
    );
    section(
        &mut out,
        "Left half (nRF), this boot",
        &report.left,
        left_lines,
    );
    section(
        &mut out,
        "Left half (nRF), previous boot",
        &report.left_previous,
        left_lines,
    );

    let _ = writeln!(out, "\nUnusual");
    let flagged = unusual(report);
    if flagged.is_empty() {
        let _ = writeln!(out, "  nothing unusual");
    }
    for line in flagged {
        let _ = writeln!(out, "  {line}");
    }
    out
}

fn section<T>(out: &mut String, title: &str, found: &Found<T>, lines: fn(&T) -> Vec<String>) {
    let _ = writeln!(out, "\n{title}");
    match found {
        Found::Record { diag, stale_ms } => {
            if let Some(age) = stale_ms {
                let _ = writeln!(
                    out,
                    "  left nRF did not answer; this is data from {}",
                    page_age(*age)
                );
            }
            for line in lines(diag) {
                let _ = writeln!(out, "  {line}");
            }
        }
        Found::NoRecord => {
            let _ = writeln!(out, "  no record");
        }
        Found::Problem(problem) => {
            let _ = writeln!(out, "  could not read: {problem}");
        }
    }
}

fn page_age(age_ms: u16) -> String {
    match age_ms {
        DIAG_AGE_UNKNOWN => "65 s or more ago".into(),
        ms => format!("{} ago", duration(u64::from(ms))),
    }
}

fn stm32_lines(d: &Stm32Diag) -> Vec<String> {
    let flag = |bit: u8| d.flags & bit != 0;
    let mut lines = vec![
        format!("boot count: {}", d.boot_count),
        format!("uptime: {}", duration(u64::from(d.uptime_s) * 1000)),
        format!(
            "reset reason: {} (RCC_CSR flags {:#04x})",
            causes(&stm32_reset_causes(d.reset_flags)),
            d.reset_flags
        ),
        format!("link to left nRF: {}", up(flag(STM32_DIAG_LINK_UP))),
        format!(
            "USB power, as the left nRF reports: {}",
            yes(flag(STM32_DIAG_NRF_VBUS))
        ),
        format!(
            "USB power report received this boot: {}",
            yes(flag(STM32_DIAG_VBUS_SEEN))
        ),
        format!("USB host enumerated: {}", yes(flag(STM32_DIAG_USB_HOST))),
        format!(
            "Bluetooth host, as the left nRF reports: {}",
            yes(flag(STM32_DIAG_BLE_HOST))
        ),
        format!("TRRS cable: {}", plugged(flag(STM32_DIAG_TRRS))),
        format!("right LED driver: {}", up(flag(STM32_DIAG_RIGHT_LEDS))),
        format!("transport: {}", transport(d.transport)),
        format!("idle step: {}", idle(d.idle_step)),
        format!("right half: {}", right_link(d.right_link)),
        format!("power gate: {}", power_gate(d.power_gate)),
        format!("left nRF boots seen (LINK_UP frames): {}", d.link_ups),
        format!(
            "main-loop passes with UART errors: overrun {}, framing {}, noise {}, parity {}, \
             RX queue full {}",
            d.sd_overruns, d.sd_framing, d.sd_noise, d.sd_parity, d.sd_queue_full
        ),
        format!(
            "frames dropped: {} ({} bad CRC)",
            d.parser_resyncs, d.crc_errors
        ),
        format!("frames cut short (bytes lost): {}", d.frame_aborts),
        format!("frames received: {}, sent: {}", d.frames_rx, d.frames_tx),
        format!("last frame from left nRF: {}", age(d.rx_age_ms)),
        format!(
            "longest gap between left nRF frames: {}",
            duration(u64::from(d.max_rx_gap_ms))
        ),
    ];
    if let (Some(probes), Some(resets)) = (d.left_probes, d.left_resets) {
        lines.push(format!(
            "left radio rescue: {probes} {}, {resets} {}",
            if probes == 1 { "probe" } else { "probes" },
            if resets == 1 { "restart" } else { "restarts" }
        ));
    }
    lines
}

fn left_lines(d: &LeftDiag) -> Vec<String> {
    let flag = |bit: u8| d.flags & bit != 0;
    let threads = |value: fn(&zapp_core::hid::ThreadStall) -> u32| {
        LEFT_DIAG_THREADS
            .iter()
            .zip(&d.threads)
            .map(|(name, thread)| format!("{name} {}", duration(u64::from(value(thread)))))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let stalled_now: Vec<String> = LEFT_DIAG_THREADS
        .iter()
        .zip(&d.threads)
        .filter(|(_, thread)| thread.stall_ms != 0)
        .map(|(name, thread)| format!("{name} {}", duration(u64::from(thread.stall_ms))))
        .collect();
    let connection = match d.conn_interval {
        0 => "no host".to_string(),
        units => format!(
            "interval {}, latency {}, timeout {}",
            interval(units),
            d.conn_latency,
            duration(u64::from(d.conn_timeout) * 10)
        ),
    };
    vec![
        format!("boot count since power-on: {}", d.boot_count),
        format!("uptime: {}", duration(u64::from(d.uptime_s) * 1000)),
        format!(
            "reset reason: {} (RESETREAS {:#010x})",
            causes(&nrf_reset_causes(d.resetreas)),
            d.resetreas
        ),
        format!("link to STM32: {}", up(flag(LEFT_DIAG_STM32_LINK))),
        format!(
            "STM32 reports: {}",
            if flag(LEFT_DIAG_STM32_SLEEP) {
                "asleep"
            } else {
                "awake"
            }
        ),
        format!(
            "STM32 reports a USB host: {}",
            yes(flag(LEFT_DIAG_STM32_USB))
        ),
        format!(
            "STM32 reports TRRS cable: {}",
            plugged(flag(LEFT_DIAG_STM32_TRRS))
        ),
        format!("STM32 idle step: {}", idle(d.stm32_idle_step)),
        format!("USB power (VBUS): {}", yes(flag(LEFT_DIAG_VBUS))),
        format!(
            "UART receiver: {}",
            if flag(LEFT_DIAG_UART_RX_OFF) {
                "OFF (re-enable failed)"
            } else {
                "on"
            }
        ),
        format!(
            "Bluetooth host connected: {}",
            yes(flag(LEFT_DIAG_BLE_HOST))
        ),
        format!("Bluetooth slot: {}", slot(d.ble_slot)),
        format!("Bluetooth connection: {connection}"),
        format!("right half radio link (ESB): {}", esb_link(d.esb_link)),
        format!("UART frames: {}, bad: {}", d.uart_frames, d.uart_bad_frames),
        format!(
            "UART RX errors: {} (overruns {}, line errors {}), ring drops: {}, RX restarts: {}",
            d.uart_rx_errors,
            d.uart_rx_overruns,
            d.uart_rx_line_errors,
            d.uart_ring_drops,
            d.uart_rx_restarts
        ),
        format!(
            "keyboard reports: {}, dropped: {}",
            d.kbd_reports, d.kbd_dropped
        ),
        format!("failed Bluetooth key reports: {}", d.ble_send_errors),
        format!("failed battery notifications: {}", d.notify_errors),
        format!(
            "right half radio: {} hops, {} timeouts",
            d.esb_hops, d.esb_timeouts
        ),
        format!(
            "longest HID send: {}",
            duration(u64::from(d.hid_send_max_ms))
        ),
        format!("last byte from STM32: {}", age(d.uart_rx_age_ms)),
        format!("last packet from right half: {}", age(d.esb_rx_age_ms)),
        format!(
            "thread stalls (longest this boot): {}",
            threads(|t| t.max_stall_ms)
        ),
        format!(
            "stalled now: {}",
            if stalled_now.is_empty() {
                "none".to_string()
            } else {
                stalled_now.join(", ")
            }
        ),
        "(counts above are 16-bit and wrap at 65536)".to_string(),
    ]
}

/// One line per flagged item across all four records.
fn unusual(report: &Report) -> Vec<String> {
    let mut lines = Vec::new();
    let mut add = |who: &str, items: Vec<String>| {
        lines.extend(items.into_iter().map(|item| format!("{who}: {item}")));
    };
    for (who, found) in [
        ("STM32, this boot", &report.stm32),
        ("STM32, previous boot", &report.stm32_previous),
    ] {
        if let Found::Record { diag, .. } = found {
            add(who, stm32_unusual(diag));
        }
    }
    match &report.left {
        Found::Record {
            stale_ms: Some(age),
            ..
        } => add(
            "left, this boot",
            vec![format!(
                "left nRF did not answer; data is from {}",
                page_age(*age)
            )],
        ),
        Found::NoRecord => add(
            "left, this boot",
            vec!["no record from the left nRF".into()],
        ),
        _ => {}
    }
    for (who, found) in [
        ("left, this boot", &report.left),
        ("left, previous boot", &report.left_previous),
    ] {
        if let Found::Record { diag, .. } = found {
            add(who, left_unusual(diag));
        }
    }
    lines
}

/// Flags in an STM32 record.
fn stm32_unusual(d: &Stm32Diag) -> Vec<String> {
    let mut out = Vec::new();
    if d.reset_flags & STM32_RESET_IWDG != 0 {
        out.push("restarted by its watchdog: its main loop hung".into());
    }
    if d.reset_flags & STM32_RESET_WWDG != 0 {
        out.push("restarted by its window watchdog".into());
    }
    let uart = nonzero(&[
        ("overrun", d.sd_overruns),
        ("framing", d.sd_framing),
        ("noise", d.sd_noise),
        ("parity", d.sd_parity),
        ("RX queue full", d.sd_queue_full),
    ]);
    if !uart.is_empty() {
        out.push(format!("UART errors: {uart}"));
    }
    if d.parser_resyncs > 0 {
        out.push(format!(
            "{} frames dropped ({} bad CRC)",
            d.parser_resyncs, d.crc_errors
        ));
    } else if d.crc_errors > 0 {
        out.push(format!("{} bad CRC", d.crc_errors));
    }
    if d.frame_aborts > 0 {
        out.push(format!("{} frames cut short (bytes lost)", d.frame_aborts));
    }
    if d.flags & STM32_DIAG_LINK_UP == 0 {
        out.push("link to left nRF is down".into());
    }
    if d.rx_age_ms == DIAG_NEVER {
        out.push("never received a frame from the left nRF".into());
    } else if d.rx_age_ms > 2000 && d.idle_step == 0 {
        out.push(format!(
            "no frame from the left nRF for {}",
            duration(u64::from(d.rx_age_ms))
        ));
    }
    if d.max_rx_gap_ms > 2000 {
        let gap = duration(u64::from(d.max_rx_gap_ms));
        out.push(format!("longest gap between left nRF frames: {gap}"));
    }
    if d.transport == 0 {
        out.push("transport still pending (neither USB nor Bluetooth)".into());
    }
    if d.power_gate != 1 {
        out.push(format!("power gate: {}", power_gate(d.power_gate)));
    }
    if d.right_link == 2 {
        out.push("right half: version mismatch".into());
    }
    out
}

/// Flags in a left nRF record.
fn left_unusual(d: &LeftDiag) -> Vec<String> {
    let mut out = Vec::new();
    if d.resetreas & (NRF_RESET_DOG0 | NRF_RESET_DOG1) != 0 {
        out.push("restarted by its watchdog".into());
    }
    if d.resetreas & NRF_RESET_LOCKUP != 0 {
        out.push("restarted after a CPU lockup".into());
    }
    if d.resetreas & NRF_RESET_SREQ != 0 {
        out.push("software reset: a crash, an update, or a deliberate restart".into());
    }
    for (index, (name, thread)) in LEFT_DIAG_THREADS.iter().zip(&d.threads).enumerate() {
        let limit = if index == 0 { 2000 } else { 500 };
        if thread.max_stall_ms > limit {
            let mut line = format!(
                "{name} stalled {}",
                duration(u64::from(thread.max_stall_ms))
            );
            if thread.stall_ms != 0 {
                let now = duration(u64::from(thread.stall_ms));
                let _ = write!(line, ", and is stalled now ({now})");
            }
            out.push(line);
        }
    }
    if d.flags & LEFT_DIAG_UART_RX_OFF != 0 {
        out.push("UART receiver is off (re-enable failed)".into());
    }
    let stm32_awake = d.flags & LEFT_DIAG_STM32_LINK != 0 && d.flags & LEFT_DIAG_STM32_SLEEP == 0;
    if stm32_awake && d.uart_rx_age_ms != DIAG_NEVER && d.uart_rx_age_ms > 2000 {
        out.push(format!(
            "no byte from the STM32 for {}",
            duration(u64::from(d.uart_rx_age_ms))
        ));
    }
    let uart = nonzero(&[
        ("bad frames", d.uart_bad_frames),
        ("RX errors", d.uart_rx_errors),
        ("overruns", d.uart_rx_overruns),
        ("line errors", d.uart_rx_line_errors),
        ("ring drops", d.uart_ring_drops),
    ]);
    if !uart.is_empty() {
        out.push(format!("UART: {uart}"));
    }
    if d.kbd_dropped > 0 {
        out.push(format!("{} keyboard reports dropped", d.kbd_dropped));
    }
    if d.ble_send_errors > 0 {
        out.push(format!(
            "{} Bluetooth key reports failed to send",
            d.ble_send_errors
        ));
    }
    if d.notify_errors > 0 {
        out.push(format!("{} battery notifications failed", d.notify_errors));
    }
    if d.hid_send_max_ms > 100 {
        out.push(format!(
            "a HID send took {}",
            duration(u64::from(d.hid_send_max_ms))
        ));
    }
    if d.esb_link == 2 {
        out.push("right half radio link: mismatch".into());
    }
    out
}

/// "name n, name n" for the counters above zero.
fn nonzero(counters: &[(&str, u16)]) -> String {
    counters
        .iter()
        .filter(|(_, count)| *count > 0)
        .map(|(name, count)| format!("{name} {count}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// "850 ms", "12.3 s", "4 min 2 s", "2 h 14 min".
fn duration(ms: u64) -> String {
    match ms {
        0..1000 => format!("{ms} ms"),
        1000..60_000 => format!("{:.1} s", ms as f64 / 1000.0),
        60_000..3_600_000 => format!("{} min {} s", ms / 60_000, ms / 1000 % 60),
        _ => format!("{} h {} min", ms / 3_600_000, ms / 60_000 % 60),
    }
}

fn age(ms: u32) -> String {
    match ms {
        DIAG_NEVER => "never".into(),
        ms => format!("{} ago", duration(u64::from(ms))),
    }
}

/// A connection interval in 1.25 ms units.
fn interval(units: u16) -> String {
    let hundredths = u32::from(units) * 125;
    match hundredths % 100 {
        0 => format!("{} ms", hundredths / 100),
        frac if frac % 10 == 0 => format!("{}.{} ms", hundredths / 100, frac / 10),
        frac => format!("{}.{frac:02} ms", hundredths / 100),
    }
}

fn causes(names: &[&str]) -> String {
    if names.is_empty() {
        "none recorded".into()
    } else {
        names.join(", ")
    }
}

fn yes(on: bool) -> &'static str {
    if on { "yes" } else { "no" }
}

fn up(on: bool) -> &'static str {
    if on { "up" } else { "down" }
}

fn plugged(on: bool) -> &'static str {
    if on { "plugged" } else { "unplugged" }
}

fn idle(step: u8) -> String {
    match step {
        0 => "0 (active)".into(),
        step => format!("{step} (asleep or staged)"),
    }
}

fn transport(value: u8) -> String {
    match value {
        0 => "pending".into(),
        1 => "Bluetooth".into(),
        2 => "USB".into(),
        other => format!("unknown ({other})"),
    }
}

fn right_link(value: u8) -> String {
    match value {
        0 => "absent".into(),
        1 => "OK".into(),
        2 => "version mismatch".into(),
        0xFF => "unknown (no fresh report)".into(),
        other => format!("unknown ({other})"),
    }
}

fn power_gate(value: u8) -> String {
    match value {
        0 => "waiting for a battery verdict".into(),
        1 => "ok".into(),
        2 => "critical battery".into(),
        3 => "done (shipping)".into(),
        other => format!("unknown ({other})"),
    }
}

fn esb_link(value: u8) -> String {
    match value {
        0 => "absent".into(),
        1 => "OK".into(),
        2 => "mismatch".into(),
        other => format!("unknown ({other})"),
    }
}

fn slot(value: u8) -> String {
    match value {
        0xFF => "none".into(),
        slot => slot.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use zapp_core::device::BootloaderKind;
    use zapp_core::hid::{CMD_GET_DIAG, ReportLink, ThreadStall};

    fn stm32() -> Stm32Diag {
        Stm32Diag {
            boot_count: 7,
            uptime_s: 8040,
            reset_flags: 0x0C,
            flags: STM32_DIAG_LINK_UP | STM32_DIAG_USB_HOST | STM32_DIAG_TRRS,
            transport: 2,
            idle_step: 0,
            right_link: 1,
            power_gate: 1,
            link_ups: 1,
            frames_rx: 123_456,
            frames_tx: 23_456,
            rx_age_ms: 12,
            max_rx_gap_ms: 340,
            ..Stm32Diag::default()
        }
    }

    fn left() -> LeftDiag {
        LeftDiag {
            boot_count: 1,
            uptime_s: 8039,
            resetreas: 0,
            flags: LEFT_DIAG_STM32_LINK
                | LEFT_DIAG_STM32_USB
                | LEFT_DIAG_STM32_TRRS
                | LEFT_DIAG_VBUS,
            ble_slot: 0xFF,
            esb_link: 1,
            uart_frames: 40_000,
            kbd_reports: 512,
            esb_hops: 3,
            hid_send_max_ms: 4,
            uart_rx_age_ms: 9,
            esb_rx_age_ms: 2,
            threads: [
                ThreadStall {
                    stall_ms: 0,
                    max_stall_ms: 340,
                },
                ThreadStall {
                    stall_ms: 0,
                    max_stall_ms: 3,
                },
                ThreadStall {
                    stall_ms: 0,
                    max_stall_ms: 1,
                },
                ThreadStall {
                    stall_ms: 0,
                    max_stall_ms: 0,
                },
            ],
            ..LeftDiag::default()
        }
    }

    /// The wire bytes of an STM32 record.
    fn stm32_bytes(d: &Stm32Diag) -> Vec<u8> {
        let mut b = vec![1, 46];
        b.extend_from_slice(&d.boot_count.to_le_bytes());
        b.extend_from_slice(&d.uptime_s.to_le_bytes());
        b.extend_from_slice(&[
            d.reset_flags,
            d.flags,
            d.transport,
            d.idle_step,
            d.right_link,
            d.power_gate,
        ]);
        for value in [
            d.link_ups,
            d.sd_overruns,
            d.sd_framing,
            d.sd_noise,
            d.sd_parity,
            d.sd_queue_full,
            d.parser_resyncs,
            d.crc_errors,
            d.frame_aborts,
        ] {
            b.extend_from_slice(&value.to_le_bytes());
        }
        for value in [d.frames_rx, d.frames_tx, d.rx_age_ms, d.max_rx_gap_ms] {
            b.extend_from_slice(&value.to_le_bytes());
        }
        b
    }

    /// The wire bytes of a left nRF record.
    fn left_bytes(d: &LeftDiag) -> Vec<u8> {
        let mut b = vec![1, 88];
        b.extend_from_slice(&d.boot_count.to_le_bytes());
        b.extend_from_slice(&d.uptime_s.to_le_bytes());
        b.extend_from_slice(&d.resetreas.to_le_bytes());
        b.extend_from_slice(&[d.flags, d.ble_slot, d.esb_link, d.stm32_idle_step]);
        for value in [
            d.conn_interval,
            d.conn_latency,
            d.conn_timeout,
            d.uart_frames,
            d.uart_bad_frames,
            d.uart_rx_errors,
            d.uart_rx_overruns,
            d.uart_rx_line_errors,
            d.uart_ring_drops,
            d.uart_rx_restarts,
            d.kbd_reports,
            d.kbd_dropped,
            d.ble_send_errors,
            d.notify_errors,
            d.esb_hops,
            d.esb_timeouts,
            d.hid_send_max_ms,
        ] {
            b.extend_from_slice(&value.to_le_bytes());
        }
        b.extend_from_slice(&d.uart_rx_age_ms.to_le_bytes());
        b.extend_from_slice(&d.esb_rx_age_ms.to_le_bytes());
        for thread in d.threads {
            b.extend_from_slice(&thread.stall_ms.to_le_bytes());
            b.extend_from_slice(&thread.max_stall_ms.to_le_bytes());
        }
        b
    }

    /// A board answering `CMD_GET_DIAG` from fixed records.
    struct FakeBoard {
        records: [Option<Vec<u8>>; 4],
        version: u8,
        answers: bool,
        left_age_ms: u16,
        fetches: Arc<AtomicUsize>,
        pending: Vec<Vec<u8>>,
    }

    impl FakeBoard {
        fn new(records: [Option<Vec<u8>>; 4]) -> Self {
            Self {
                records,
                version: DIAG_PAGE_VERSION,
                answers: true,
                left_age_ms: 0,
                fetches: Arc::new(AtomicUsize::new(0)),
                pending: Vec::new(),
            }
        }

        fn page(&self, record: u8, page: u8) -> Vec<u8> {
            // EVT_DIAG shares the opcode with CMD_GET_DIAG.
            let mut r = vec![CMD_GET_DIAG, self.version, record, page];
            if self.version != DIAG_PAGE_VERSION {
                r.resize(32, 0);
                return r;
            }
            let age = if record >= 2 { self.left_age_ms } else { 0 };
            match self.records.get(usize::from(record)) {
                Some(Some(bytes)) => {
                    let start = usize::from(page) * 24;
                    if start >= bytes.len() {
                        r.extend_from_slice(&[2, 0, 0, 0]);
                    } else {
                        r.extend_from_slice(&[0, bytes.len() as u8]);
                        r.extend_from_slice(&age.to_le_bytes());
                        r.extend_from_slice(&bytes[start..(start + 24).min(bytes.len())]);
                    }
                }
                Some(None) => r.extend_from_slice(&[1, 0, 0xFF, 0xFF]),
                None => r.extend_from_slice(&[2, 0, 0, 0]),
            }
            r.resize(32, 0);
            r
        }
    }

    impl ReportLink for FakeBoard {
        fn send(&mut self, report: &[u8; 32]) -> io::Result<()> {
            match report[0] {
                0x00 => self.pending.push(b"\x00abc/def\xFE".to_vec()),
                0xFE => self.pending.push(vec![0xFD, 0x04, 0xFE]),
                CMD_GET_DIAG if self.answers => {
                    if report[3] & DIAG_FETCH != 0 {
                        self.fetches.fetch_add(1, Ordering::Relaxed);
                    }
                    let page = self.page(report[1], report[2]);
                    self.pending.push(page);
                }
                _ => {}
            }
            Ok(())
        }

        fn receive(&mut self, timeout: Duration) -> io::Result<Option<Vec<u8>>> {
            if self.pending.is_empty() {
                thread::sleep(timeout);
                return Ok(None);
            }
            Ok(Some(self.pending.remove(0)))
        }
    }

    fn all_records() -> [Option<Vec<u8>>; 4] {
        let mut previous = left();
        previous.resetreas = NRF_RESET_SREQ;
        previous.threads[0] = ThreadStall {
            stall_ms: 0,
            max_stall_ms: 47_200,
        };
        [
            Some(stm32_bytes(&stm32())),
            None,
            Some(left_bytes(&left())),
            Some(left_bytes(&previous)),
        ]
    }

    fn header() -> Header {
        Header {
            product: "Wireless Voyager".into(),
            serial: Some("abc/def".into()),
            others: 0,
            firmware: Some(("abc".into(), "def".into())),
            protocol: Some(4),
        }
    }

    #[test]
    fn the_left_radio_rescue_line_shows_only_when_the_board_sends_it() {
        let rescue = |d: &Stm32Diag| {
            stm32_lines(d)
                .into_iter()
                .filter(|line| line.starts_with("left radio rescue"))
                .collect::<Vec<_>>()
        };
        assert!(rescue(&stm32()).is_empty());
        let counted = Stm32Diag {
            left_probes: Some(2),
            left_resets: Some(1),
            ..stm32()
        };
        assert_eq!(rescue(&counted), ["left radio rescue: 2 probes, 1 restart"]);
    }

    #[test]
    fn the_builders_match_the_record_sizes() {
        assert_eq!(stm32_bytes(&stm32()).len(), 48);
        assert_eq!(left_bytes(&left()).len(), 90);
    }

    #[test]
    fn a_fresh_board_is_read_whole_with_one_fetch() {
        let board = FakeBoard::new(all_records());
        let fetches = board.fetches.clone();
        let mut session = Session::new(HidConnection::over(board));
        let report = session.collect().unwrap();

        assert_eq!(
            report.stm32,
            Found::Record {
                diag: stm32(),
                stale_ms: None
            }
        );
        assert_eq!(report.stm32_previous, Found::NoRecord);
        assert_eq!(
            report.left,
            Found::Record {
                diag: left(),
                stale_ms: None
            }
        );
        assert!(report.left_previous.fresh());
        assert_eq!(fetches.load(Ordering::Relaxed), 1);
        assert_eq!(session.firmware, Some(("abc".into(), "def".into())));
        assert_eq!(session.protocol, Some(4));
    }

    #[test]
    fn a_stale_left_record_is_fetched_again_and_marked() {
        let mut board = FakeBoard::new(all_records());
        board.left_age_ms = DIAG_AGE_UNKNOWN;
        let fetches = board.fetches.clone();
        let report = Session::new(HidConnection::over(board)).collect().unwrap();

        assert_eq!(fetches.load(Ordering::Relaxed), 2);
        assert_eq!(
            report.left,
            Found::Record {
                diag: left(),
                stale_ms: Some(DIAG_AGE_UNKNOWN)
            }
        );
        let text = render(&header(), &report);
        assert!(text.contains("left nRF did not answer; this is data from 65 s or more ago"));
    }

    #[test]
    fn a_board_that_never_answers_is_an_error() {
        let mut board = FakeBoard::new(all_records());
        board.answers = false;
        let err = Session::new(HidConnection::over(board))
            .collect()
            .unwrap_err();
        assert_eq!(err.to_string(), NO_ANSWER);
    }

    #[test]
    fn firmware_without_diagnostics_is_an_error() {
        let mut board = FakeBoard::new(all_records());
        board.version = 0;
        let err = Session::new(HidConnection::over(board))
            .collect()
            .unwrap_err();
        assert!(err.to_string().contains("has no diagnostics"));
    }

    #[test]
    fn freshness_allows_the_slack() {
        assert!(fresh(500, Duration::from_millis(400)));
        assert!(!fresh(501, Duration::from_millis(400)));
        assert!(!fresh(DIAG_AGE_UNKNOWN, Duration::from_secs(100)));
    }

    #[test]
    fn the_report_reads_as_plain_text() {
        let mut previous = left();
        previous.resetreas = NRF_RESET_SREQ;
        previous.threads[0].max_stall_ms = 47_200;
        let report = Report {
            stm32: Found::Record {
                diag: stm32(),
                stale_ms: None,
            },
            stm32_previous: Found::NoRecord,
            left: Found::Record {
                diag: left(),
                stale_ms: None,
            },
            left_previous: Found::Record {
                diag: previous,
                stale_ms: None,
            },
        };
        let text = render(&header(), &report);
        for line in [
            "keyboard: Wireless Voyager (USB serial abc/def)",
            "Oryx protocol: 0x04",
            "\nKeyboard MCU (STM32), previous boot\n  no record\n",
            "  uptime: 2 h 14 min\n",
            "  reset reason: reset pin, power-on (RCC_CSR flags 0x0c)\n",
            "  reset reason: software reset (RESETREAS 0x00000040)\n",
            "  thread stalls (longest this boot): main loop 47.2 s, HID queue 3 ms",
            "\nUnusual\n  left, previous boot: software reset: a crash, an update, or a \
             deliberate restart\n  left, previous boot: main loop stalled 47.2 s\n",
        ] {
            assert!(text.contains(line), "missing {line:?} in:\n{text}");
        }
        assert!(text.is_ascii());
    }

    #[test]
    fn a_clean_report_says_nothing_unusual() {
        let report = Report {
            stm32: Found::Record {
                diag: stm32(),
                stale_ms: None,
            },
            stm32_previous: Found::NoRecord,
            left: Found::Record {
                diag: left(),
                stale_ms: None,
            },
            left_previous: Found::NoRecord,
        };
        assert!(unusual(&report).is_empty());
        assert!(render(&header(), &report).ends_with("\nUnusual\n  nothing unusual\n"));
    }

    #[test]
    fn stm32_rules_flag_what_went_wrong() {
        assert!(stm32_unusual(&stm32()).is_empty());

        let mut d = stm32();
        d.reset_flags = STM32_RESET_IWDG | 0x04;
        d.flags = 0;
        d.sd_overruns = 2;
        d.sd_queue_full = 1;
        d.parser_resyncs = 3;
        d.crc_errors = 1;
        d.frame_aborts = 4;
        d.rx_age_ms = 5000;
        d.max_rx_gap_ms = 125_000;
        d.transport = 0;
        d.power_gate = 2;
        assert_eq!(
            stm32_unusual(&d),
            [
                "restarted by its watchdog: its main loop hung",
                "UART errors: overrun 2, RX queue full 1",
                "3 frames dropped (1 bad CRC)",
                "4 frames cut short (bytes lost)",
                "link to left nRF is down",
                "no frame from the left nRF for 5.0 s",
                "longest gap between left nRF frames: 2 min 5 s",
                "transport still pending (neither USB nor Bluetooth)",
                "power gate: critical battery",
            ]
        );

        // An old last frame is expected while asleep; never is not.
        d = stm32();
        d.idle_step = 2;
        d.rx_age_ms = 5000;
        assert!(stm32_unusual(&d).is_empty());
        d.rx_age_ms = DIAG_NEVER;
        assert_eq!(
            stm32_unusual(&d),
            ["never received a frame from the left nRF"]
        );
    }

    #[test]
    fn left_rules_flag_what_went_wrong() {
        assert!(left_unusual(&left()).is_empty());

        let mut d = left();
        d.resetreas = NRF_RESET_DOG0 | NRF_RESET_LOCKUP;
        d.threads[0] = ThreadStall {
            stall_ms: 12_000,
            max_stall_ms: 47_200,
        };
        d.threads[1].max_stall_ms = 501;
        d.threads[3].max_stall_ms = 500;
        d.flags |= LEFT_DIAG_UART_RX_OFF;
        d.uart_rx_age_ms = 3000;
        d.uart_rx_overruns = 2;
        d.uart_ring_drops = 1;
        d.kbd_dropped = 5;
        d.ble_send_errors = 6;
        d.notify_errors = 7;
        d.hid_send_max_ms = 101;
        d.esb_link = 2;
        assert_eq!(
            left_unusual(&d),
            [
                "restarted by its watchdog",
                "restarted after a CPU lockup",
                "main loop stalled 47.2 s, and is stalled now (12.0 s)",
                "HID queue stalled 501 ms",
                "UART receiver is off (re-enable failed)",
                "no byte from the STM32 for 3.0 s",
                "UART: overruns 2, ring drops 1",
                "5 keyboard reports dropped",
                "6 Bluetooth key reports failed to send",
                "7 battery notifications failed",
                "a HID send took 101 ms",
                "right half radio link: mismatch",
            ]
        );

        // A quiet UART is expected while the STM32 sleeps.
        d = left();
        d.flags |= LEFT_DIAG_STM32_SLEEP;
        d.uart_rx_age_ms = 60_000;
        assert!(left_unusual(&d).is_empty());
    }

    #[test]
    fn a_stale_or_missing_left_record_is_unusual() {
        let mut report = Report {
            stm32: Found::Record {
                diag: stm32(),
                stale_ms: None,
            },
            stm32_previous: Found::NoRecord,
            left: Found::Record {
                diag: left(),
                stale_ms: Some(12_000),
            },
            left_previous: Found::NoRecord,
        };
        assert_eq!(
            unusual(&report),
            ["left, this boot: left nRF did not answer; data is from 12.0 s ago"]
        );
        report.left = Found::NoRecord;
        assert_eq!(
            unusual(&report),
            ["left, this boot: no record from the left nRF"]
        );
    }

    #[test]
    fn durations_read_naturally() {
        assert_eq!(duration(0), "0 ms");
        assert_eq!(duration(999), "999 ms");
        assert_eq!(duration(1000), "1.0 s");
        assert_eq!(duration(12_345), "12.3 s");
        assert_eq!(duration(242_000), "4 min 2 s");
        assert_eq!(duration(8_040_000), "2 h 14 min");
        assert_eq!(age(DIAG_NEVER), "never");
        assert_eq!(age(12), "12 ms ago");
        assert_eq!(interval(6), "7.5 ms");
        assert_eq!(interval(9), "11.25 ms");
        assert_eq!(interval(12), "15 ms");
    }

    fn usb(keyboard: Option<Keyboard>, bootloader: Option<BootloaderKind>, pid: u16) -> UsbDevice {
        UsbDevice {
            keyboard,
            bootloader,
            serial: None,
            vid: 0x3297,
            pid,
        }
    }

    /// The error `pick` gives for `devices`.
    fn refused(devices: &[UsbDevice]) -> String {
        match pick(devices) {
            Ok(_) => panic!("a device was picked"),
            Err(err) => err.to_string(),
        }
    }

    #[test]
    fn only_a_voyager_2_running_its_firmware_is_picked() {
        let other = || usb(Some(Keyboard::Voyager), None, 0x1977);
        let booting = || {
            usb(
                Some(Keyboard::Voyager2),
                Some(BootloaderKind::IgnitionV2),
                0x0792,
            )
        };
        let running = || usb(Some(Keyboard::Voyager2), None, 0x1978);

        assert_eq!(refused(&[other()]), "No Wireless Voyager found on USB.");
        assert!(refused(&[other(), booting()]).contains("is in its bootloader"));

        let devices = [other(), booting(), running(), running()];
        let (device, count) = pick(&devices).unwrap();
        assert_eq!((device.pid, count), (0x1978, 2));
    }
}
