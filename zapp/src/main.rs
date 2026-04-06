use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use serde::Deserialize;

use zapp_core::device::ids::{is_moonlander_revb, target_name_for_pid};
use zapp_core::device::{self, WatchStatus};
use zapp_core::firmware::{self, Firmware};
use zapp_core::flash::{self, FlashProgress};

#[derive(Parser)]
#[command(name = "zapp", version, about = format!("⚡ Flash ZSA keyboards — v{}", env!("CARGO_PKG_VERSION")))]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Flash firmware from a local file or Oryx URL
    #[command(
        long_about = "Flash firmware from a local file or Oryx URL.\n\n\
            Supports .bin (DFU) and .hex (Intel HEX) firmware files, as well as \
            Oryx layout URLs. When given a URL, the firmware is downloaded and \
            flashed directly.",
        after_long_help = "Examples:\n  \
            zapp flash firmware.bin\n  \
            zapp flash https://configure.zsa.io/voyager/layouts/default/latest\n  \
            zapp flash https://configure.zsa.io/moonlander/layouts/default/abc123"
    )]
    Flash {
        /// Path to firmware file (.bin or .hex), or an Oryx URL.
        firmware: String,
    },
    /// Check Oryx for updates on your layout and flash if available
    #[command(
        long_about = "Check Oryx for updates on your layout and flash if available.\n\n\
            Detects a connected ZSA keyboard, reads its layout and revision from the \
            USB serial number, and checks Oryx for a newer firmware revision. If an \
            update is available, it is downloaded and flashed automatically. \n\
            Note: This only works if your keyboard is currently flashed with a firmware \
            coming from Oryx and is not currently in bootloader mode."
    )]
    Update,
}

#[derive(Deserialize)]
struct LatestResponse {
    latest: String,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Flash { firmware } => {
            if firmware.starts_with("https://configure.zsa.io/") {
                cmd_flash_url(&firmware)
            } else {
                cmd_flash_file(&PathBuf::from(firmware))
            }
        }
        Commands::Update => cmd_update(),
    }
}

fn cmd_flash_file(path: &PathBuf) -> Result<()> {
    let fw = firmware::load_firmware(path).context("Failed to load firmware")?;
    print_firmware_info(&fw);
    wait_and_flash(&fw)
}

/// Parse an Oryx URL into (layoutId, Option<revisionId>).
///
/// Accepted forms:
///   /voyager/layouts/:layoutId
///   /voyager/layouts/:layoutId/latest
///   /voyager/layouts/:layoutId/latest/0
///   /voyager/layouts/:layoutId/:revisionId
fn parse_oryx_url(url: &str) -> Result<(&str, Option<&str>)> {
    let path = url
        .strip_prefix("https://configure.zsa.io/")
        .context("Not a valid Oryx URL")?;

    // segments: ["voyager", "layouts", ":layoutId", ...optional...]
    let segments: Vec<&str> = path.trim_end_matches('/').split('/').collect();

    if segments.len() < 3 || segments[1] != "layouts" {
        bail!("Not a valid Oryx layout URL");
    }

    let layout_id = segments[2];

    let revision_id = match segments.get(3) {
        None => None,
        Some(&"latest") => None,
        Some(rev) => Some(*rev),
    };

    Ok((layout_id, revision_id))
}

fn resolve_revision(layout_id: &str, revision_id: Option<&str>) -> Result<String> {
    if let Some(rev) = revision_id {
        return Ok(rev.to_string());
    }

    let url = format!("https://oryx.zsa.io/firmware/latest/{layout_id}");
    let resp: LatestResponse = reqwest::blocking::get(&url)
        .and_then(|r| r.error_for_status())
        .context("Failed to fetch latest revision")?
        .json()
        .context("Failed to parse latest revision response")?;

    Ok(resp.latest)
}

fn download_firmware(revision_id: &str) -> Result<Firmware> {
    download_firmware_with_alt(revision_id, false)
}

fn download_firmware_with_alt(revision_id: &str, alt: bool) -> Result<Firmware> {
    let mut download_url = format!("https://oryx.zsa.io/firmware/{revision_id}");
    if alt {
        download_url.push_str("?alt=true");
    }

    let spinner = new_spinner("Downloading firmware...");

    let fw_bytes = reqwest::blocking::get(&download_url)
        .and_then(|r| r.error_for_status())
        .context("Failed to download firmware")?
        .bytes()
        .context("Failed to read firmware bytes")?;

    spinner.finish_and_clear();

    firmware::load_firmware_from_bytes(&fw_bytes).context("Failed to parse downloaded firmware")
}

fn cmd_flash_url(url: &str) -> Result<()> {
    let (layout_id, revision_id) = parse_oryx_url(url)?;
    let revision = resolve_revision(layout_id, revision_id)?;

    println!("Layout: {layout_id}, revision: {revision}");

    let fw = download_firmware(&revision)?;
    print_firmware_info(&fw);
    wait_and_flash(&fw)
}

fn cmd_update() -> Result<()> {
    let connected = device::find_keyboard().context("Failed to scan USB devices")?;
    println!("Found {} connected", connected.keyboard);

    // Parse serial number as "layoutId/revisionId"
    let Some((layout_id, revision_id)) = connected.serial.split_once('/') else {
        bail!(
            "Updates only work if your keyboard is currently flashed with a firmware coming from Oryx."
        );
    };

    if layout_id.is_empty() || revision_id.is_empty() {
        bail!(
            "Updates only work if your keyboard is currently flashed with a firmware coming from Oryx."
        );
    }

    println!("Layout: {layout_id}, revision: {revision_id}");

    // Check for latest revision
    let url = format!("https://oryx.zsa.io/firmware/latest/{layout_id}");
    let resp: LatestResponse = reqwest::blocking::get(&url)
        .and_then(|r| r.error_for_status())
        .context("Failed to check for updates")?
        .json()
        .context("Failed to parse update response")?;

    if revision_id == resp.latest {
        println!("Firmware is already up to date.");
        return Ok(());
    }

    println!(
        "Update available: {} → {}",
        revision_id, resp.latest
    );

    let fw = download_firmware_with_alt(&resp.latest, is_moonlander_revb(connected.pid))?;
    print_firmware_info(&fw);
    wait_and_flash(&fw)
}

fn print_firmware_info(fw: &Firmware) {
    let desc = match fw {
        Firmware::DfuBinary { data, pid, .. } => {
            format!("{} ({} bytes)", target_name_for_pid(*pid), data.len())
        }
        Firmware::IgnitionDual { primary, alternate } => {
            format!(
                "{} + {} ({} + {} bytes)",
                target_name_for_pid(primary.pid),
                target_name_for_pid(alternate.pid),
                primary.data.len(),
                alternate.data.len()
            )
        }
        Firmware::IntelHex { data } => {
            format!("Ergodox EZ ({} bytes)", data.len())
        }
    };
    println!("Firmware loaded: {desc}");
}

fn new_spinner(msg: &str) -> ProgressBar {
    let spinner = ProgressBar::new_spinner();
    spinner.set_style(
        ProgressStyle::default_spinner()
            .tick_strings(&["_", "_", "_", "-", "`", "`", "'", "´", "-", "_", "_", "_"])
            .template("{spinner:.cyan} {msg}")
            .unwrap(),
    );
    spinner.set_message(msg.to_string());
    spinner.enable_steady_tick(std::time::Duration::from_millis(100));
    spinner
}

fn wait_and_flash(fw: &Firmware) -> Result<()> {
    let spinner = new_spinner("Waiting for keyboard in bootloader mode...");

    let device = device::wait_for_bootloader(None, |status| match status {
        WatchStatus::Waiting => {}
        WatchStatus::Found { .. } => {
            spinner.finish_and_clear();
        }
    })
    .context("Failed to detect bootloader")?;

    let pb = ProgressBar::new(100);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("⚡ {bar:40.cyan/blue} {pos}% {msg}")
            .unwrap()
            .progress_chars("█▓░"),
    );

    flash::flash_device(&device, fw, &|progress| match progress {
        FlashProgress::Erasing {
            bytes_erased,
            total_bytes,
        } => {
            let pct = (bytes_erased * 100) / total_bytes;
            pb.set_position(pct as u64);
            pb.set_message("Erasing...");
        }
        FlashProgress::Writing {
            bytes_written,
            total_bytes,
        } => {
            let pct = (bytes_written * 100) / total_bytes;
            pb.set_position(pct as u64);
            pb.set_message("Writing...");
        }
        FlashProgress::Resetting => {
            pb.set_position(100);
            pb.set_message("Resetting...");
        }
        FlashProgress::Complete => {
            pb.finish_with_message("Done!");
        }
    })
    .context("Flash failed")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_oryx_url_bare_layout() {
        let (layout, rev) = parse_oryx_url("https://configure.zsa.io/voyager/layouts/abcde").unwrap();
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_oryx_url_latest() {
        let (layout, rev) =
            parse_oryx_url("https://configure.zsa.io/voyager/layouts/abcde/latest").unwrap();
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_oryx_url_latest_with_zero() {
        let (layout, rev) =
            parse_oryx_url("https://configure.zsa.io/voyager/layouts/abcde/latest/0").unwrap();
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_oryx_url_specific_revision() {
        let (layout, rev) =
            parse_oryx_url("https://configure.zsa.io/moonlander/layouts/AbCdE/abc123").unwrap();
        assert_eq!(layout, "AbCdE");
        assert_eq!(rev, Some("abc123"));
    }

    #[test]
    fn test_parse_oryx_url_trailing_slash() {
        let (layout, rev) =
            parse_oryx_url("https://configure.zsa.io/voyager/layouts/abcde/").unwrap();
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_oryx_url_invalid_prefix() {
        assert!(parse_oryx_url("https://example.com/voyager/layouts/abcde").is_err());
    }

    #[test]
    fn test_parse_oryx_url_missing_layouts_segment() {
        assert!(parse_oryx_url("https://configure.zsa.io/voyager/abcde").is_err());
    }
}
