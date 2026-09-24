//! Logging setup: stderr as `RUST_LOG` says, plus a debug-level record of
//! every flash appended to `zapp-flash.log` in the temp directory.

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use log::{Level, Log, Metadata, Record};

/// The file's filter: debug for the workspace's crates, info for the rest.
const FILE_FILTER: &str = "info,zapp=debug,zapp_core=debug,zapp_ble=debug,zapp_hid_proto=debug,\
                           zapp_oryx=debug,zapp_smp=debug,zapp_wired=debug";

/// Size past which the log is moved to `.1` before a run appends to it.
const MAX_LOG_BYTES: u64 = 5 * 1024 * 1024;

/// The file logger, set only for a flashing run whose log could be opened.
static FILE_LOG: OnceLock<env_logger::Logger> = OnceLock::new();

/// Sends each record to stderr and to the flash log, each under its own filter.
struct Tee {
    stderr: env_logger::Logger,
}

impl Log for Tee {
    fn enabled(&self, metadata: &Metadata) -> bool {
        self.stderr.enabled(metadata) || FILE_LOG.get().is_some_and(|file| file.enabled(metadata))
    }

    fn log(&self, record: &Record) {
        self.stderr.log(record);
        if let Some(file) = FILE_LOG.get() {
            file.log(record);
        }
    }

    fn flush(&self) {
        self.stderr.flush();
    }
}

/// Installs the global logger. With `flashing`, also opens the flash log and
/// writes the run's header to it; a log that cannot be opened costs one
/// warning and nothing else.
pub fn init(flashing: bool) {
    let stderr =
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
            .format_timestamp(None)
            .build();
    let mut max_level = stderr.filter();

    if flashing {
        let path = log_path();
        match open_file_logger(&path) {
            Ok(file) => {
                max_level = max_level.max(file.filter());
                let args: Vec<String> = std::env::args_os()
                    .map(|arg| arg.to_string_lossy().into_owned())
                    .collect();
                write_file(
                    &file,
                    Level::Info,
                    format_args!(
                        "==== zapp {} run: {}",
                        env!("CARGO_PKG_VERSION"),
                        args.join(" ")
                    ),
                );
                let _ = FILE_LOG.set(file);
            }
            Err(err) => eprintln!(
                "Warning: cannot write the flash log {}: {err}",
                path.display()
            ),
        }
    }

    if log::set_boxed_logger(Box::new(Tee { stderr })).is_ok() {
        log::set_max_level(max_level);
    }
}

/// Closes the run's entry in the flash log with how it ended.
pub fn finish(result: &anyhow::Result<()>) {
    match result {
        Ok(()) => {
            if let Some(file) = FILE_LOG.get() {
                write_file(
                    file,
                    Level::Info,
                    format_args!("==== run finished: success"),
                );
            }
        }
        Err(err) => fail(err),
    }
}

/// Closes the run's entry in the flash log with the error main would print.
pub fn fail(err: &anyhow::Error) {
    if let Some(file) = FILE_LOG.get() {
        write_file(
            file,
            Level::Error,
            format_args!("==== run failed: Error: {err:#}"),
        );
    }
}

fn log_path() -> PathBuf {
    std::env::temp_dir().join("zapp-flash.log")
}

/// Rotates an oversized log, opens it for appending and builds its logger.
fn open_file_logger(path: &Path) -> std::io::Result<env_logger::Logger> {
    if fs::metadata(path).is_ok_and(|meta| meta.len() > MAX_LOG_BYTES) {
        fs::rename(path, path.with_extension("log.1"))?;
    }
    let file = OpenOptions::new().create(true).append(true).open(path)?;
    Ok(env_logger::Builder::new()
        .parse_filters(FILE_FILTER)
        .target(env_logger::Target::Pipe(Box::new(file)))
        .write_style(env_logger::WriteStyle::Never)
        .format(|buf, record| {
            writeln!(
                buf,
                "{} {:<5} {}: {}",
                buf.timestamp_millis(),
                record.level(),
                record.target(),
                record.args()
            )
        })
        .build())
}

/// Writes one line to the file alone, bypassing stderr.
fn write_file(file: &env_logger::Logger, level: Level, args: std::fmt::Arguments) {
    file.log(
        &Record::builder()
            .args(args)
            .level(level)
            .target("zapp")
            .build(),
    );
}
