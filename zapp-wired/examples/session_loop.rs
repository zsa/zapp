//! Opens and closes an ignition session over and over, and counts what breaks.
//!
//! A bench diagnostic for the intermittent USB fault that only ever shows up
//! while a session is opening: after `BEGIN`, while the host polls `STATUS`
//! every few milliseconds waiting for the session to go active, a single
//! control transfer has twice completed EPROTO (`WiredError::Faulted`), and
//! once the STM32 rebooted underneath the host and the device disappeared.
//! Around 7% of flash runs are hit.
//!
//! Session open needs no bundle, no flashing and no reset button — ignition
//! stays in its bootloader between sessions — so the sequence can be run
//! thousands of times unattended, which is what turns a 7% event into
//! statistics. It also factors the cause, because `BEGIN`'s half bitmap
//! decides what the bootloader actually does:
//!
//! * `--halves both`  — pin-resets the left nRF **and** sends the right nRF a
//!   0x54 reboot over I2C.
//! * `--halves left`  — only the left reset blip.
//! * `--halves right` — only the 0x54 to the right.
//!
//! The link is opened with [`UsbLink::open_bare`], not `open()`: the retrying
//! wrapper exists precisely to hide stalls and read faults, and hiding them is
//! the opposite of what this is for. Every raw error is classified and counted
//! here instead.
//!
//! ```text
//! cargo run -p zapp-wired --example session_loop -- --halves both --iterations 200
//! cargo run -p zapp-wired --example session_loop -- --halves left --iterations 500 --csv left.csv
//! RUST_LOG=zapp_wired=debug cargo run -p zapp-wired --example session_loop
//! ```
//!
//! Timestamps are printed as unix milliseconds so a fault line can be lined up
//! against a usbmon capture. It exits non-zero on the first fault or
//! disconnect, so a shell loop stops with the capture still on screen; pass
//! `--keep-going` to run the whole count regardless.

use std::fs::OpenOptions;
use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use zapp_smp::Half;
use zapp_wired::ignition::{usb::UsbLink, Halves, Ignition, SessionState, TargetState};
use zapp_wired::WiredError;

/// How long to leave the device alone before trying to open it again after it
/// has vanished. A rebooting STM32 re-enumerates well inside this.
const REOPEN_DELAY: Duration = Duration::from_secs(3);

const USAGE: &str = "\
usage: session_loop [options]

  --halves both|left|right   which halves BEGIN covers (default both)
  --iterations N             how many open/close cycles to run (default 200)
  --pause MS                 wait between iterations, after END (default 200)
  --settle-timeout MS        how long to poll STATUS for Active (default 8000)
  --csv PATH                 append one line per iteration to PATH
  --keep-going               finish the count even after a fault or disconnect
  -h, --help                 this
";

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp(None)
        .init();

    let options = match Options::parse(std::env::args().skip(1)) {
        Ok(Some(options)) => options,
        Ok(None) => return ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("{err}");
            eprintln!();
            eprintln!("{USAGE}");
            return ExitCode::FAILURE;
        }
    };

    match run(&options) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(err) => {
            eprintln!();
            eprintln!("Failed: {err}");
            ExitCode::FAILURE
        }
    }
}

struct Options {
    halves: Halves,
    halves_name: String,
    iterations: u32,
    pause: Duration,
    settle_timeout: Duration,
    csv: Option<String>,
    keep_going: bool,
}

impl Options {
    /// `Ok(None)` for `--help`, which is a request rather than a mistake.
    fn parse(args: impl Iterator<Item = String>) -> Result<Option<Self>, String> {
        let mut halves = Halves::BOTH;
        let mut halves_name = "both".to_string();
        let mut iterations = 200u32;
        let mut pause = 200u64;
        let mut settle_timeout = 8000u64;
        let mut csv = None;
        let mut keep_going = false;

        let mut args = args;
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--halves" => {
                    let value = value_of(&arg, args.next())?;
                    halves = match value.as_str() {
                        "both" => Halves::BOTH,
                        "left" => Halves::of(Half::Left),
                        "right" => Halves::of(Half::Right),
                        other => {
                            return Err(format!(
                                "--halves takes both, left or right, not {other:?}"
                            ))
                        }
                    };
                    halves_name = value;
                }
                "--iterations" => iterations = number(&arg, args.next())? as u32,
                "--pause" => pause = number(&arg, args.next())?,
                "--settle-timeout" => settle_timeout = number(&arg, args.next())?,
                "--csv" => csv = Some(value_of(&arg, args.next())?),
                "--keep-going" => keep_going = true,
                "-h" | "--help" => {
                    println!("{USAGE}");
                    return Ok(None);
                }
                other => return Err(format!("{other:?} is not an option this example takes")),
            }
        }

        Ok(Some(Self {
            halves,
            halves_name,
            iterations,
            pause: Duration::from_millis(pause),
            settle_timeout: Duration::from_millis(settle_timeout),
            csv,
            keep_going,
        }))
    }
}

fn value_of(flag: &str, value: Option<String>) -> Result<String, String> {
    value.ok_or_else(|| format!("{flag} needs a value"))
}

fn number(flag: &str, value: Option<String>) -> Result<u64, String> {
    let value = value_of(flag, value)?;
    value
        .parse()
        .map_err(|_| format!("{flag} takes a number, not {value:?}"))
}

/// What a failed transfer was, as far as anything above USB can tell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// The device answered with a STALL: the known `BEGIN` race.
    Stalled,
    /// The transfer failed on the bus itself — EPROTO and friends. The event
    /// this tool exists to catch.
    Faulted,
    /// The device went away mid-run. The STM32 rebooted.
    Disconnected,
    Other,
}

/// Which [`WiredError`] means which.
///
/// A disconnect reaches this as a [`WiredError::Request`] carrying `nusb`'s
/// "device disconnected", or, once the device is off the bus, as the
/// [`WiredError::NoBootloader`] a reopen gets. Both are the same event.
fn classify(err: &WiredError) -> Kind {
    match err {
        WiredError::Stalled { .. } => Kind::Stalled,
        WiredError::Faulted { .. } => Kind::Faulted,
        WiredError::NoBootloader => Kind::Disconnected,
        WiredError::Request { reason, .. } | WiredError::Usb(reason) => {
            match reason.to_lowercase().contains("disconnect") || reason.contains("No such device")
            {
                true => Kind::Disconnected,
                false => Kind::Other,
            }
        }
        _ => Kind::Other,
    }
}

/// Unix milliseconds, the one clock a usbmon capture can be lined up against.
fn unix_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|since| since.as_millis())
        .unwrap_or(0)
}

/// How one iteration ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Result_ {
    Active,
    Timeout,
    Error,
}

impl Result_ {
    fn label(self) -> &'static str {
        match self {
            Self::Active => "ACTIVE",
            Self::Timeout => "TIMEOUT",
            Self::Error => "ERROR",
        }
    }
}

/// One pass through BEGIN, the STATUS poll, and END.
struct Iteration {
    index: u32,
    t0_unix_ms: u128,
    result: Result_,
    ms_to_active: Option<u128>,
    polls: u32,
    stalls: u32,
    faults: u32,
    disconnects: u32,
    others: u32,
    first_error: Option<String>,
    states: Option<(TargetState, TargetState)>,
}

impl Iteration {
    fn new(index: u32) -> Self {
        Self {
            index,
            t0_unix_ms: unix_ms(),
            result: Result_::Error,
            ms_to_active: None,
            polls: 0,
            stalls: 0,
            faults: 0,
            disconnects: 0,
            others: 0,
            first_error: None,
            states: None,
        }
    }

    /// Counts one failed transfer and keeps the first message seen.
    fn record(&mut self, err: &WiredError) -> Kind {
        let kind = classify(err);
        match kind {
            Kind::Stalled => self.stalls += 1,
            Kind::Faulted => self.faults += 1,
            Kind::Disconnected => self.disconnects += 1,
            Kind::Other => self.others += 1,
        }
        if self.first_error.is_none() {
            self.first_error = Some(format!("{kind:?}: {err}"));
        }
        kind
    }

    fn interesting(&self) -> bool {
        self.faults > 0 || self.disconnects > 0
    }
}

/// Everything the run has seen so far.
#[derive(Default)]
struct Summary {
    run: u32,
    active: u32,
    timeout: u32,
    error: u32,
    stalls: u32,
    faults: u32,
    disconnects: u32,
    others: u32,
    /// Iteration number and unix timestamp of the ones worth going back to.
    events: Vec<(u32, u128, String)>,
}

impl Summary {
    fn add(&mut self, iteration: &Iteration) {
        self.run += 1;
        match iteration.result {
            Result_::Active => self.active += 1,
            Result_::Timeout => self.timeout += 1,
            Result_::Error => self.error += 1,
        }
        self.stalls += iteration.stalls;
        self.faults += iteration.faults;
        self.disconnects += iteration.disconnects;
        self.others += iteration.others;
        if iteration.interesting() {
            self.events.push((
                iteration.index,
                iteration.t0_unix_ms,
                iteration
                    .first_error
                    .clone()
                    .unwrap_or_else(|| "?".to_string()),
            ));
        }
    }

    fn clean(&self) -> bool {
        self.faults == 0 && self.disconnects == 0
    }

    fn print(&self, options: &Options) {
        println!();
        println!("Summary · halves {}", options.halves_name);
        println!("  iterations   {}", self.run);
        println!("  active       {}", self.active);
        println!("  timeout      {}", self.timeout);
        println!("  error        {}", self.error);
        println!("  stalls       {}", self.stalls);
        println!("  faults       {}", self.faults);
        println!("  disconnects  {}", self.disconnects);
        println!("  other errors {}", self.others);
        if self.events.is_empty() {
            println!("  no faults and no disconnects");
            return;
        }
        println!();
        println!("Iterations that faulted or disconnected:");
        for (index, timestamp, first) in &self.events {
            println!("  #{index} · t0 {timestamp} · {first}");
        }
    }
}

/// The CSV, opened for append with a header written only into a new file.
struct Csv(std::fs::File);

impl Csv {
    fn open(path: &str) -> Result<Self, String> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .map_err(|err| format!("{path}: {err}"))?;
        let fresh = file.metadata().map(|meta| meta.len() == 0).unwrap_or(false);
        let mut csv = Self(file);
        if fresh {
            csv.line("iter,t0_unix_ms,result,ms_to_active,polls,stalls,faults,first_error,halves")?;
        }
        Ok(csv)
    }

    fn line(&mut self, text: &str) -> Result<(), String> {
        writeln!(self.0, "{text}").map_err(|err| format!("could not write the CSV: {err}"))?;
        // Flushed every line: a run that dies on the interesting iteration must
        // still have that iteration on disk.
        self.0
            .flush()
            .map_err(|err| format!("could not write the CSV: {err}"))
    }

    fn add(&mut self, iteration: &Iteration, options: &Options) -> Result<(), String> {
        let first = iteration.first_error.clone().unwrap_or_default();
        let first = first.replace([',', '\n'], " ");
        let ms = iteration
            .ms_to_active
            .map(|ms| ms.to_string())
            .unwrap_or_default();
        let line = format!(
            "{},{},{},{},{},{},{},{},{}",
            iteration.index,
            iteration.t0_unix_ms,
            iteration.result.label(),
            ms,
            iteration.polls,
            iteration.stalls,
            iteration.faults,
            first,
            options.halves_name,
        );
        self.line(&line)
    }
}

fn run(options: &Options) -> Result<bool, String> {
    let mut csv = match &options.csv {
        Some(path) => Some(Csv::open(path)?),
        None => None,
    };

    let mut ignition = Ignition::new(open()?);
    let mut summary = Summary::default();

    println!(
        "Looping BEGIN/STATUS/END {} times · halves {} · pause {} ms · settle timeout {} ms",
        options.iterations,
        options.halves_name,
        options.pause.as_millis(),
        options.settle_timeout.as_millis()
    );
    println!();

    for index in 1..=options.iterations {
        let mut iteration = Iteration::new(index);
        let started = Instant::now();
        let mut lost = false;

        // ---- BEGIN -------------------------------------------------------
        //
        // `Ignition::begin` already answers a stall by closing the session left
        // open and asking once more, so an error here is one the bootloader
        // meant, or a transfer that never landed.
        if let Err(err) = ignition.begin(options.halves) {
            match iteration.record(&err) {
                Kind::Disconnected => lost = true,
                kind => println!("  #{index} BEGIN failed ({kind:?}): {err}"),
            }
            if !lost {
                // No session to poll or close; on to the next one.
                finish(&mut iteration, &mut summary, csv.as_mut(), options)?;
                std::thread::sleep(options.pause);
                continue;
            }
        }

        // ---- The poll ----------------------------------------------------
        if !lost {
            lost = poll_active(&mut ignition, &mut iteration, started, options);
        }

        // ---- END ---------------------------------------------------------
        if !lost {
            if let Err(err) = ignition.end() {
                match iteration.record(&err) {
                    Kind::Disconnected => lost = true,
                    Kind::Faulted => println!(
                        "  #{index} !! FAULT on END · t {} ms · {} ms after BEGIN: {err}",
                        unix_ms(),
                        started.elapsed().as_millis()
                    ),
                    kind => println!("  #{index} END failed ({kind:?}): {err}"),
                }
            }
        }

        finish(&mut iteration, &mut summary, csv.as_mut(), options)?;

        // ---- The device went away ----------------------------------------
        if lost {
            println!();
            println!(
                "  *** #{index} THE DEVICE DISCONNECTED · t {} ms ***",
                unix_ms()
            );
            println!("  the STM32 has most likely rebooted; reopening in {REOPEN_DELAY:?}");
            std::thread::sleep(REOPEN_DELAY);
            match open() {
                Ok(link) => {
                    ignition = Ignition::new(link);
                    println!("  reopened, carrying on");
                    println!();
                }
                Err(err) => {
                    summary.print(options);
                    return Err(format!("the device did not come back: {err}"));
                }
            }
        }

        if !summary.clean() && !options.keep_going {
            println!();
            println!("Stopping on the first fault or disconnect (--keep-going overrides this)");
            break;
        }

        std::thread::sleep(options.pause);
    }

    summary.print(options);
    Ok(summary.clean() || options.keep_going)
}

/// Polls STATUS until the session is active, counting every failed transfer
/// rather than stopping on it. Returns true if the device disconnected.
fn poll_active(
    ignition: &mut Ignition<UsbLink>,
    iteration: &mut Iteration,
    started: Instant,
    options: &Options,
) -> bool {
    let index = iteration.index;
    loop {
        iteration.polls += 1;
        match ignition.status() {
            Ok(status) => {
                if iteration.states.is_none() {
                    iteration.states = Some((status.left_state, status.right_state));
                }
                match status.state {
                    SessionState::Active => {
                        iteration.result = Result_::Active;
                        iteration.ms_to_active = Some(started.elapsed().as_millis());
                        return false;
                    }
                    SessionState::Pending => {}
                    SessionState::Fault => {
                        iteration.result = Result_::Error;
                        iteration.first_error.get_or_insert(format!(
                            "SessionFault: the bootloader faulted opening the session \
                             (error {:#06x})",
                            status.error
                        ));
                        println!(
                            "  #{index} the session went to Fault, error {:#06x}",
                            status.error
                        );
                        return false;
                    }
                    SessionState::Idle | SessionState::Unrecognised(_) => {
                        iteration.result = Result_::Error;
                        iteration
                            .first_error
                            .get_or_insert(format!("SessionState: {:?}", status.state));
                        println!("  #{index} the session reports {:?}", status.state);
                        return false;
                    }
                }
            }
            Err(err) => {
                let elapsed = started.elapsed().as_millis();
                match iteration.record(&err) {
                    // The known race: the first request into the gap BEGIN
                    // opens. Counted and stepped over.
                    Kind::Stalled => {}
                    Kind::Faulted => println!(
                        "  #{index} !! FAULT · t {} ms · poll {} · {elapsed} ms after BEGIN: {err}",
                        unix_ms(),
                        iteration.polls
                    ),
                    Kind::Disconnected => {
                        iteration.result = Result_::Error;
                        println!(
                            "  #{index} !! DISCONNECT · t {} ms · poll {} · {elapsed} ms after \
                             BEGIN: {err}",
                            unix_ms(),
                            iteration.polls
                        );
                        return true;
                    }
                    Kind::Other => {
                        iteration.result = Result_::Error;
                        println!(
                            "  #{index} STATUS failed · poll {} · {elapsed} ms after BEGIN: {err}",
                            iteration.polls
                        );
                        return false;
                    }
                }
            }
        }

        if started.elapsed() >= options.settle_timeout {
            iteration.result = Result_::Timeout;
            println!(
                "  #{index} the session was still opening after {} ms",
                options.settle_timeout.as_millis()
            );
            return false;
        }
    }
}

/// Prints the iteration's line, adds it to the totals and to the CSV.
fn finish(
    iteration: &mut Iteration,
    summary: &mut Summary,
    csv: Option<&mut Csv>,
    options: &Options,
) -> Result<(), String> {
    let ms = iteration
        .ms_to_active
        .map(|ms| ms.to_string())
        .unwrap_or_else(|| "-".to_string());
    let states = match iteration.states {
        Some((left, right)) => format!("L={left:?} R={right:?}"),
        None => String::new(),
    };
    println!(
        "[{:>5}/{}] {:<7} ms={:>5} polls={:>5} stalls={} faults={} {states}",
        iteration.index,
        options.iterations,
        iteration.result.label(),
        ms,
        iteration.polls,
        iteration.stalls,
        iteration.faults,
    );
    summary.add(iteration);
    match csv {
        Some(csv) => csv.add(iteration, options),
        None => Ok(()),
    }
}

fn open() -> Result<UsbLink, String> {
    // Bare on purpose: `UsbLink::open()` wraps the link in `Retrying`, which
    // absorbs exactly the stalls and read faults this tool is here to count.
    UsbLink::open_bare().map_err(|err| match err {
        WiredError::NoBootloader => "no ZSA bootloader is plugged in. Press the reset button on \
                                     the left half — everything here goes through ignition."
            .to_string(),
        err => format!("could not open the bootloader: {err}"),
    })
}
