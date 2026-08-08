use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};

use zapp_ble::smp::{self, ImageSlot, SmpClient};
use zapp_ble::{Battery, BleMonitor, Presence};
use zapp_core::device::ids::{self as ids, is_moonlander_revb, target_name_for_pid};
use zapp_core::device::{self, UsbDevice, WatchStatus};
use zapp_core::firmware::{self, Firmware};
use zapp_core::flash::{self, FlashProgress};
use zapp_oryx::FirmwareVariant;

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
    /// List the ZSA keyboards this machine can see, wired and wireless
    #[command(long_about = "List the ZSA keyboards this machine can see.\n\n\
            Wired boards are read straight from USB. Bluetooth boards are listed \
            when they are connected to this machine, bonded to it, or advertising \
            in range. A board that has never been paired only advertises, so pass \
            --scan to hold a discovery window open long enough to catch one.")]
    Devices {
        /// Scan for boards that have never been paired to this machine
        #[arg(long)]
        scan: bool,
    },
    /// Drive an mcumgr (SMP) firmware update over Bluetooth
    #[command(
        long_about = "Drive an mcumgr (SMP) firmware update over Bluetooth.\n\n\
            Talks the standard Zephyr SMP service, the one nRF Connect Device \
            Manager speaks, to the wireless half of a Voyager 2. The board must \
            already be bonded to this machine: pairing is the desktop's job. \n\
            A firmware update is three steps — upload the image, mark it for \
            test, reset — and each is a subcommand here so a failure is visible \
            where it happens.",
        after_long_help = "Examples:\n  \
            zapp smp images\n  \
            zapp smp upload app.signed.bin\n  \
            zapp smp test 3a5f9c...\n  \
            zapp smp confirm 3a5f9c...\n  \
            zapp smp reset"
    )]
    Smp {
        #[command(subcommand)]
        command: SmpCommands,
    },
}

#[derive(Subcommand)]
enum SmpCommands {
    /// List the images the board is carrying, slot by slot
    Images(SmpArgs),
    /// Upload an MCUboot-signed image into the board's spare slot
    #[command(after_long_help = "The image is only written, not booted: mark it \
        with `zapp smp test <hash>` and reset to try it, or `zapp smp confirm \
        <hash>` to keep it. `zapp smp images` shows the hash afterwards.")]
    Upload {
        /// Path to an MCUboot-signed .bin
        firmware: PathBuf,
        /// Which image, on a board carrying more than one
        #[arg(long, default_value_t = 0)]
        image: u32,
        #[command(flatten)]
        args: SmpArgs,
    },
    /// Mark an image to be tried on the next boot, by hash
    Test {
        /// The image hash, as `zapp smp images` prints it
        hash: String,
        #[command(flatten)]
        args: SmpArgs,
    },
    /// Confirm an image permanently, by hash
    Confirm {
        /// The image hash, as `zapp smp images` prints it
        hash: String,
        #[command(flatten)]
        args: SmpArgs,
    },
    /// Reboot the board
    Reset(SmpArgs),
}

#[derive(clap::Args)]
struct SmpArgs {
    /// Which board, by advertised name or by the handle `zapp devices` shows
    #[arg(long, short = 'd')]
    device: Option<String>,
    /// Bytes of payload per Bluetooth write, the ATT MTU less its 3-byte
    /// header. Defaults to what the platform reports.
    #[arg(long)]
    mtu: Option<usize>,
    /// Largest request to send, fragmented across writes. Defaults to the
    /// buffer the board reports, or one write if it does not report one.
    #[arg(long)]
    chunk: Option<usize>,
    /// Seconds to wait for each reply
    #[arg(long, default_value_t = 15)]
    timeout: u64,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Flash { firmware } => {
            if zapp_oryx::is_oryx_url(&firmware) {
                cmd_flash_url(&firmware)
            } else {
                cmd_flash_file(&PathBuf::from(firmware))
            }
        }
        Commands::Update => cmd_update(),
        Commands::Devices { scan } => cmd_devices(scan),
        Commands::Smp { command } => match command {
            SmpCommands::Images(args) => cmd_smp_images(&args),
            SmpCommands::Upload {
                firmware,
                image,
                args,
            } => cmd_smp_upload(&firmware, image, &args),
            SmpCommands::Test { hash, args } => cmd_smp_mark(&hash, false, &args),
            SmpCommands::Confirm { hash, args } => cmd_smp_mark(&hash, true, &args),
            SmpCommands::Reset(args) => cmd_smp_reset(&args),
        },
    }
}

/// How long to give the radio. Without `--scan`, just long enough for the first
/// answer, which already carries every bonded board; with it, long enough for a
/// discovery window to open and close.
const BLE_SETTLE: Duration = Duration::from_millis(750);
const BLE_SCAN: Duration = Duration::from_secs(7);

fn cmd_devices(scan: bool) -> Result<()> {
    println!("USB");
    match device::list_devices() {
        Ok(devices) if devices.is_empty() => println!("  none"),
        Ok(devices) => {
            for device in &devices {
                println!("  {}", describe_usb(device));
            }
        }
        Err(err) => println!("  cannot scan: {err}"),
    }

    let monitor = BleMonitor::start(&[]);
    let wait = if scan { BLE_SCAN } else { BLE_SETTLE };
    let spinner = new_spinner("Looking for Bluetooth boards...");
    thread::sleep(wait);
    spinner.finish_and_clear();

    println!();
    println!("Bluetooth");
    let boards = monitor.snapshot();
    if boards.is_empty() {
        println!("  none");
        if !scan {
            println!("  (pass --scan to look for boards that have never been paired)");
        }
    }
    for board in boards {
        let name = board
            .keyboard
            .map(|keyboard| keyboard.to_string())
            .unwrap_or_else(|| "ZSA keyboard".to_string());
        let known_as = board.name.unwrap_or_else(|| "unnamed".to_string());
        println!(
            "  {name} · {} · {known_as:?}{}",
            state(board.presence),
            charge(board.battery)
        );
    }

    Ok(())
}

/// What the halves have left. A board only linked to one half says so by
/// leaving the other out.
fn charge(battery: Battery) -> String {
    match (battery.left, battery.right) {
        (Some(left), Some(right)) => format!(" · left {left}% · right {right}%"),
        (Some(left), None) => format!(" · left {left}% · right unknown"),
        (None, Some(right)) => format!(" · right {right}%"),
        (None, None) => battery
            .keyboard
            .map(|level| format!(" · {level}%"))
            .unwrap_or_default(),
    }
}

fn state(presence: Presence) -> &'static str {
    match presence {
        Presence::Connected => "connected",
        Presence::Nearby => "in range",
        Presence::Paired => "paired, not in range",
    }
}

fn describe_usb(device: &UsbDevice) -> String {
    let name = match device.bootloader {
        Some(_) => ids::friendly_name(device.vid, device.pid).to_string(),
        None => target_name_for_pid(device.pid).to_string(),
    };
    match &device.serial {
        Some(serial) if !serial.is_empty() => format!("{name} · {serial}"),
        _ => name,
    }
}

fn cmd_flash_file(path: &PathBuf) -> Result<()> {
    let fw = firmware::load_firmware(path).context("Failed to load firmware")?;
    print_firmware_info(&fw);
    wait_and_flash(&fw)
}

/// Download a revision's firmware, showing a spinner while it transfers.
fn download_firmware(revision_id: &str, variant: FirmwareVariant) -> Result<Firmware> {
    let spinner = new_spinner("Downloading firmware...");
    let fw = zapp_oryx::download_firmware(revision_id, variant);
    spinner.finish_and_clear();

    fw.context("Failed to download firmware")
}

fn cmd_flash_url(url: &str) -> Result<()> {
    let (geometry, layout_id, revision_id) = zapp_oryx::parse_url(url)?;
    let revision = zapp_oryx::resolve_revision(geometry, layout_id, revision_id)
        .context("Failed to fetch latest revision")?;

    println!("Layout: {layout_id}, revision: {revision}");

    // The Moonlander's two halves ship as one collated image.
    let variant = if geometry == "moonlander" {
        FirmwareVariant::Collated
    } else {
        FirmwareVariant::Standard
    };

    let fw = download_firmware(&revision, variant)?;
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

    let latest =
        zapp_oryx::fetch_latest_revision(layout_id).context("Failed to check for updates")?;

    if revision_id == latest {
        println!("Firmware is already up to date.");
        return Ok(());
    }

    println!("Update available: {revision_id} → {latest}");

    let geometry = zapp_oryx::geometry_for(connected.keyboard, connected.pid);
    print_revision_note("current", layout_id, revision_id, geometry);
    print_revision_note("update", layout_id, &latest, geometry);

    let variant = if is_moonlander_revb(connected.pid) {
        FirmwareVariant::Alternate
    } else {
        FirmwareVariant::Standard
    };

    let fw = download_firmware(&latest, variant)?;
    print_firmware_info(&fw);
    wait_and_flash(&fw)
}

/// Print the commit message attached to a revision. A revision may carry no
/// message, and a note is never worth failing an update over, so both cases
/// print the same placeholder.
fn print_revision_note(label: &str, layout_id: &str, revision_id: &str, geometry: &str) {
    let note = match zapp_oryx::fetch_revision_note(layout_id, revision_id, geometry) {
        Ok(note) => note,
        Err(e) => {
            log::debug!("Could not fetch note for revision {revision_id}: {e}");
            None
        }
    };

    let note = note.as_deref().unwrap_or("(no note)");
    println!("  {label:<7} {revision_id}: {note}");
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

/// Connects to the board these arguments name.
///
/// The spinner is the whole point of doing this in one place: a board that is
/// bonded but idle takes seconds to wake, and silence there looks like a hang.
fn smp_connect(args: &SmpArgs) -> Result<SmpClient> {
    let spinner = new_spinner("Connecting over Bluetooth...");
    let client = smp::open(args.device.as_deref());
    spinner.finish_and_clear();

    let mut client = client.context("Could not reach the board over Bluetooth")?;
    if let Some(mtu) = args.mtu {
        client = client.with_mtu(mtu);
    }
    let mut client = client.with_timeout(Duration::from_secs(args.timeout));

    // The timeout has to be set first: this is a request like any other, and
    // it is the first thing the board is asked.
    match args.chunk {
        Some(chunk) => client = client.with_request_limit(chunk),
        None => {
            client.negotiate_request_limit();
        }
    }
    log::debug!(
        "connected, {} bytes per write, {} per request",
        client.mtu(),
        client.request_limit()
    );
    Ok(client)
}

fn cmd_smp_images(args: &SmpArgs) -> Result<()> {
    let mut client = smp_connect(args)?;
    let slots = client.image_state().context("Could not read the image state")?;
    print_image_state(&slots);
    Ok(())
}

fn print_image_state(slots: &[ImageSlot]) {
    if slots.is_empty() {
        println!("The board reports no images.");
        return;
    }
    for slot in slots {
        let image = slot.image.map(|n| format!("image {n} ")).unwrap_or_default();
        let version = if slot.version.is_empty() {
            "(no version)"
        } else {
            &slot.version
        };
        println!("{image}slot {} · {version}", slot.slot);
        println!("  hash  {}", smp::image::hex(&slot.hash));
        println!("  flags {}", flags_of(slot));
    }
}

fn flags_of(slot: &ImageSlot) -> String {
    match slot.flags().join(" ") {
        flags if flags.is_empty() => "none".to_string(),
        flags => flags,
    }
}

fn cmd_smp_upload(path: &Path, image: u32, args: &SmpArgs) -> Result<()> {
    // Not `firmware::load_firmware`: that reads a DFU suffix or Intel HEX
    // records, and an MCUboot image is a flat binary with neither. Its own
    // header is what says whether this is the right kind of file.
    let data = std::fs::read(path)
        .with_context(|| format!("Failed to read {}", path.display()))?;
    if data.is_empty() {
        bail!("{} is empty", path.display());
    }

    match smp::image::inspect(&data) {
        Some(info) => println!(
            "Image loaded: v{} ({} bytes, {} of image)",
            info.version,
            data.len(),
            info.image_size
        ),
        // Not fatal: the board checks the header itself and will refuse it. A
        // warning here says which file to look at, which the board cannot.
        None => println!(
            "Warning: {} has no MCUboot header — uploading it anyway, but the board will likely refuse it",
            path.display()
        ),
    }
    println!("SHA-256: {}", smp::image::hex(&smp::image::sha256(&data)));

    let mut client = smp_connect(args)?;
    let pb = ProgressBar::new(data.len() as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("⚡ {bar:40.cyan/blue} {bytes}/{total_bytes} {bytes_per_sec} {msg}")
            .unwrap()
            .progress_chars("█▓░"),
    );

    client
        .upload(image, &data, &|progress| match progress {
            FlashProgress::Writing { bytes_written, .. } => {
                pb.set_position(bytes_written as u64);
                pb.set_message("Uploading...");
            }
            FlashProgress::Complete => pb.finish_with_message("Done!"),
            _ => {}
        })
        .context("Upload failed")?;

    println!();
    println!("Uploaded. Mark it and reset to boot it:");
    println!("  zapp smp test <hash>   (reverts unless confirmed after boot)");
    println!("  zapp smp confirm <hash>");
    print_image_state(&client.image_state().context("Could not read the image state")?);
    Ok(())
}

fn cmd_smp_mark(hash: &str, confirm: bool, args: &SmpArgs) -> Result<()> {
    let hash = smp::image::unhex(hash)
        .with_context(|| format!("{hash:?} is not a hash; expected hex, as `zapp smp images` prints it"))?;

    let mut client = smp_connect(args)?;
    let slots = client
        .mark(&hash, confirm)
        .with_context(|| match confirm {
            true => "Could not confirm that image".to_string(),
            false => "Could not mark that image for test".to_string(),
        })?;

    println!(
        "Marked {} as {}.",
        smp::image::hex(&hash),
        if confirm { "confirmed" } else { "pending test" }
    );
    print_image_state(&slots);
    Ok(())
}

fn cmd_smp_reset(args: &SmpArgs) -> Result<()> {
    let mut client = smp_connect(args)?;
    client.reset().context("Reset failed")?;
    println!("Resetting.");
    Ok(())
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
