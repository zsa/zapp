mod nrf_wired;
mod zsab;

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};

use zapp_ble::dfu::{self, DfuClient, DfuError, Wait};
use zapp_ble::smp::session::{Next, Status};
use zapp_ble::smp::{self, Half, ImageSlot, SmpClient};
use zapp_ble::{Battery, BleMonitor, Level, Presence};
use zapp_core::device::ids::{self as ids, is_moonlander_revb, target_name_for_pid};
use zapp_core::device::{self, UsbDevice, WatchStatus};
use zapp_core::firmware::{self, Firmware};
use zapp_core::flash::{self, FlashProgress};
use zapp_core::hid::{
    BatteryReading, CMD_GET_TELEMETRY, ChargeState, Charging, HidConnection, HidEvent, Nav,
    Source, Telemetry,
};
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
            Supports .bin (DFU) and .hex (Intel HEX) keyboard firmware, .zsab \
            bundles carrying both wireless halves, and Oryx layout URLs. When \
            given a URL, the firmware is downloaded and flashed directly.\n\n\
            Whichever it is, plug the keyboard in and press the reset button on \
            the left half when asked: this waits for the bootloader to appear \
            rather than expecting it to be there already.",
        after_long_help = "Examples:\n  \
            zapp flash firmware.bin\n  \
            zapp flash voyager2_0.1.9.zsab\n  \
            zapp flash https://configure.zsa.io/voyager/layouts/default/latest\n  \
            zapp flash https://configure.zsa.io/moonlander/layouts/default/abc123"
    )]
    Flash {
        /// Path to a firmware file (.bin, .hex or .zsab), or an Oryx URL.
        firmware: String,
        /// Write a half even if it already carries the image, or the bundle is
        /// older than what is installed. Only means anything for .zsab bundles:
        /// a keyboard image is always written.
        #[arg(long)]
        force: bool,
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
            where it happens.\n\n\
            --half right sends the same commands to the right half instead, \
            through a second service on the left that relays whole frames over \
            the halves' radio link. Update the right half first: it is only \
            reachable while both halves still speak the same protocol.\n\n\
            `zapp smp bundle` does all of that in one go, both halves, from a \
            .zsab release bundle. It is also the only thing here that declares \
            an update, and shipping firmware accepts nothing else over \
            Bluetooth: the single-step subcommands below are refused unless the \
            board was built with the guard compiled out. They are kept because \
            they are the debugging surface, and `zapp smp session` says what \
            the board has open.",
        after_long_help = "Examples:\n  \
            zapp smp bundle voyager2_0.1.8.zsab\n  \
            zapp smp images\n  \
            zapp smp upload app.signed.bin\n  \
            zapp smp images --half right\n  \
            zapp smp upload right.signed.bin --half right\n  \
            zapp smp test 3a5f9c...\n  \
            zapp smp confirm 3a5f9c...\n  \
            zapp smp reset\n  \
            zapp smp session"
    )]
    Smp {
        #[command(subcommand)]
        command: SmpCommands,
    },
    /// Flash the keyboard's own (QMK) firmware over Bluetooth
    #[command(
        long_about = "Flash the keyboard's own firmware over Bluetooth.\n\n\
            A Voyager 2 has two chips. `zapp smp` updates the wireless one; \
            this updates the other — the STM32 that runs QMK, scans the matrix \
            and drives the LEDs. The image goes over ZSA's own update service \
            on the left half, which relays it to the STM32 over the wire \
            between them and into the ignition bootloader.\n\n\
            The board must already be bonded to this machine: pairing is the \
            desktop's job. The firmware is a plain .bin — what `qmk compile` \
            leaves in .build, or what Oryx hands you.\n\n\
            The STM32 has to be in its bootloader before anything is written. \
            `zapp dfu flash` checks first and only reboots it if it is not, so \
            a keyboard already sitting there is written straight away — \
            including one you put there yourself with the reset button.",
        after_long_help = "Examples:\n  \
            zapp dfu flash voyager2_default.bin\n  \
            zapp dfu flash .build/zsa_voyager_tng_fdprod.bin --device \"Voyager\"\n  \
            zapp dfu status\n  \
            zapp dfu enter\n  \
            zapp dfu abort"
    )]
    Dfu {
        #[command(subcommand)]
        command: DfuCommands,
    },
    /// Update both nRF halves over the cable, through the bootloader
    #[command(
        long_about = "Update both nRF halves over the cable.\n\n\
            The same nRF firmware `zapp smp bundle` writes over Bluetooth, \
            written over USB instead. The STM32's ignition bootloader drives \
            both halves into MCUboot itself — the left over the UART between \
            them, the right over the TRRS I2C bus — and bridges MCUboot's \
            serial recovery to this machine over one vendor interface. There \
            is no radio in the path, so nothing depends on the halves being \
            paired, on the left's application being alive, or on how much \
            charge is in either pack.\n\n\
            The keyboard has to be in its bootloader: press the reset button \
            on the left half. Nothing here works against a keyboard running \
            its own firmware.",
        after_long_help = "Examples:\n  \
            zapp nrf flash voyager2_0.1.9.zsab\n  \
            zapp nrf status\n  \
            zapp nrf end"
    )]
    Nrf {
        #[command(subcommand)]
        command: NrfCommands,
    },
}

#[derive(Subcommand)]
enum NrfCommands {
    /// Write both halves from one .zsab bundle
    #[command(
        long_about = "Write both halves from one .zsab bundle.\n\n\
            A bundle names both halves and they are always written together. \
            There is deliberately no way to pick one: a keyboard whose halves \
            are on different builds is a keyboard whose halves cannot talk to \
            each other, and per-half flashing is how boards end up there.\n\n\
            The right half goes first, then the left. A half already carrying \
            the bundle's image is left alone rather than written again, and \
            every upload is read back and checked before the half is \
            restarted.",
        after_long_help = "Examples:\n  \
            zapp nrf flash voyager2_0.1.9.zsab"
    )]
    Flash {
        /// Path to a .zsab firmware bundle
        bundle: PathBuf,
        /// Write a half even if it already carries the image, or the bundle is
        /// older than what is installed
        #[arg(long)]
        force: bool,
        /// Seconds to wait for each reply from a half
        #[arg(long, default_value_t = 15)]
        timeout: u64,
    },
    /// Say what the bootloader thinks is going on, and what is on the I2C bus
    Status,
    /// End a session that was left open and put the hardware back
    End,
}

#[derive(Subcommand)]
enum DfuCommands {
    /// Write a QMK .bin into the keyboard's STM32
    #[command(after_long_help = "The image is written straight into the STM32's \
        application region and the chip is restarted onto it. Nothing is kept \
        in reserve and there is no going back to the old firmware, the same as \
        flashing over USB — but the image is checksummed on the way in, and an \
        STM32 that does not like what it got stays in its bootloader rather \
        than booting it.")]
    Flash {
        /// Path to a QMK firmware .bin
        firmware: PathBuf,
        #[command(flatten)]
        args: DfuArgs,
    },
    /// Say whether the STM32 is in its bootloader
    Status(DfuArgs),
    /// Reboot the STM32 into its bootloader and leave it there
    Enter(DfuArgs),
    /// Drop a transfer that was left open, and release the update lock
    Abort(DfuArgs),
}

#[derive(clap::Args)]
struct DfuArgs {
    /// Which board, by advertised name or by the handle `zapp devices` shows
    #[arg(long, short = 'd')]
    device: Option<String>,
    /// Bytes of image per Bluetooth write. Defaults to what the link
    /// negotiated: the ATT MTU less its 3-byte header.
    #[arg(long)]
    chunk: Option<usize>,
    /// Seconds to wait for the STM32 to reach its bootloader
    #[arg(long, default_value_t = 30)]
    wait: u64,
}

#[derive(Subcommand)]
enum SmpCommands {
    /// Update both halves from one .zsab bundle
    #[command(
        long_about = "Update both halves from one .zsab bundle.\n\n\
            A .zsab carries both nRF images and the build they came from, and \
            this drives the whole update from it: read what is installed, \
            write the right half, write the left, restart both and wait for \
            each to come back and confirm itself. A half already running its \
            image is skipped rather than written again.\n\n\
            The right half goes first, always. It is reachable only through \
            the left, and only while both halves still speak the same \
            protocol, so a left updated first would cut off the only route to \
            the half still waiting for its image.\n\n\
            The keyboard cannot be typed on while this runs and both halves go \
            dark. It takes a couple of minutes. Nothing is confirmed until it \
            has proved itself, so a keyboard left half-way through gets its old \
            firmware back on the next power cycle.",
        after_long_help = "Examples:\n  \
            zapp smp bundle voyager2_0.1.8.zsab\n  \
            zapp smp bundle voyager2_0.1.8.zsab --force\n  \
            zapp smp bundle voyager2_0.1.8.zsab --device \"Voyager\""
    )]
    Bundle {
        /// Path to a .zsab firmware bundle
        bundle: PathBuf,
        /// Write both halves even where the image is already installed
        #[arg(long)]
        force: bool,
        #[command(flatten)]
        args: LinkArgs,
    },
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
    /// Show the update session the keyboard has open, and end it
    #[command(
        long_about = "Show the update session the keyboard has open.\n\n\
            Over Bluetooth a Voyager 2 only accepts a declared update: a host \
            says which image each half is getting before it writes anything, \
            and every write is checked against that. This reads that \
            declaration back — what is declared, which half is next, how long \
            the session has left, and whether this board is enforcing any of \
            it at all.\n\n\
            Only the left half serves this; the right runs no session of its \
            own. --close ends the session, which is worth doing when something \
            stopped half way and left one open.",
        after_long_help = "Examples:\n  \
            zapp smp session\n  \
            zapp smp session --close"
    )]
    Session {
        /// End the session rather than only reporting it
        #[arg(long)]
        close: bool,
        #[command(flatten)]
        link: LinkArgs,
    },
}

#[derive(clap::Args)]
struct SmpArgs {
    /// Which half to talk to: the left's own SMP service, or the right's
    /// through the left's relay
    #[arg(long, default_value = "left")]
    half: Half,
    #[command(flatten)]
    link: LinkArgs,
}

/// Everything about reaching a board that does not say which half.
///
/// Split out so the bundle flow, which talks to both halves in turn and picks
/// the order itself, takes the same options as a single command.
#[derive(clap::Args)]
pub struct LinkArgs {
    /// Which board, by advertised name or by the handle `zapp devices` shows
    #[arg(long, short = 'd')]
    device: Option<String>,
    /// Bytes of payload per Bluetooth write, the ATT MTU less its 3-byte
    /// header. Defaults to what the platform reports.
    #[arg(long)]
    mtu: Option<usize>,
    /// Bluetooth writes to spend on one request. Defaults to 5.
    #[arg(long)]
    writes: Option<usize>,
    /// Largest request to send, fragmented across writes. Overrides --writes.
    /// Defaults to the buffer the board reports, capped by --writes.
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
        Commands::Flash { firmware, force } => {
            if zapp_oryx::is_oryx_url(&firmware) {
                cmd_flash_url(&firmware)
            } else {
                cmd_flash_file(&PathBuf::from(firmware), force)
            }
        }
        Commands::Update => cmd_update(),
        Commands::Devices { scan } => cmd_devices(scan),
        Commands::Smp { command } => {
            // The bundle flow prints its own fuller version, with the measured
            // levels in it, so it is left to do that.
            let bundle_flow = matches!(command, SmpCommands::Bundle { .. });
            let result = match command {
                SmpCommands::Bundle {
                    bundle,
                    force,
                    args,
                } => zsab::update(&bundle, force, &args),
                SmpCommands::Images(args) => cmd_smp_images(&args),
                SmpCommands::Upload {
                    firmware,
                    image,
                    args,
                } => cmd_smp_upload(&firmware, image, &args),
                SmpCommands::Test { hash, args } => cmd_smp_mark(&hash, false, &args),
                SmpCommands::Confirm { hash, args } => cmd_smp_mark(&hash, true, &args),
                SmpCommands::Reset(args) => cmd_smp_reset(&args),
                SmpCommands::Session { close, link } => cmd_smp_session(close, &link),
            };
            // A keyboard that said no said why, and the reason is worth more
            // than the one line the error chain gives it. The guard's own
            // refusals get a second paragraph: a bare subcommand that hits one
            // has to be told where the supported path is, because retrying it
            // will be refused in exactly the same way.
            if let (false, Err(err)) = (bundle_flow, &result)
                && let Some(advice) = zsab::refusal_advice(err)
            {
                println!();
                println!("{advice}");
                if let Some(note) = zsab::guard_note(err) {
                    println!();
                    println!("{note}");
                }
            }
            result
        }
        Commands::Dfu { command } => {
            let result = match command {
                DfuCommands::Flash { firmware, args } => cmd_dfu_flash(&firmware, &args),
                DfuCommands::Status(args) => cmd_dfu_status(&args),
                DfuCommands::Enter(args) => cmd_dfu_enter(&args),
                DfuCommands::Abort(args) => cmd_dfu_abort(&args),
            };
            // The board's own reasons come back as a code and a sentence; the
            // paragraph that says what to do about one is worth more than the
            // line the error chain gives it.
            if let Err(err) = &result
                && let Some(advice) = dfu_advice(err)
            {
                println!();
                println!("{advice}");
            }
            result
        }
        // Nothing is added to a failure here: the wired path reads the
        // bootloader's own status back and prints what it says, which is more
        // than an error chain has, and it does it while the board is still
        // plugged in and answering.
        Commands::Nrf { command } => match command {
            NrfCommands::Flash {
                bundle,
                force,
                timeout,
            } => nrf_wired::flash(&bundle, force, Duration::from_secs(timeout)),
            NrfCommands::Status => nrf_wired::status(),
            NrfCommands::End => nrf_wired::end(),
        },
    }
}

/// How long to give the radio. Without `--scan`, just long enough for the first
/// answer, which already carries every bonded board; with it, long enough for a
/// discovery window to open and close.
const BLE_SETTLE: Duration = Duration::from_millis(750);
const BLE_SCAN: Duration = Duration::from_secs(7);

/// How long to wait for a wired board to answer the telemetry query. Firmware
/// that predates the command never answers at all, so this elapses in the
/// ordinary case and must stay short enough not to be felt.
const TELEMETRY_WAIT: Duration = Duration::from_millis(500);

fn cmd_devices(scan: bool) -> Result<()> {
    let wired = device::list_devices();

    println!("USB");
    match &wired {
        Ok(devices) if devices.is_empty() => println!("  none"),
        Ok(devices) => {
            for device in devices {
                println!("  {}", describe_usb(device));
                for line in wired_report(device) {
                    println!("    {line}");
                }
            }
        }
        Err(err) => println!("  cannot scan: {err}"),
    }

    // Which models are plugged in, so a board listed twice can say which of
    // its two entries is the live one.
    let wired_models: Vec<_> = wired
        .iter()
        .flatten()
        .filter(|device| device.bootloader.is_none())
        .filter_map(|device| device.keyboard)
        .collect();

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
            "  {name} · {} · {known_as:?}{}{}",
            state(board.presence),
            charge(board.battery),
            also_wired(board.keyboard, &wired_models)
        );
    }

    Ok(())
}

/// Ask a wired board about itself, and say what it answered.
///
/// A board is preferred over the radio for its own state: plugged in, it can
/// read both halves directly, and the Bluetooth view of a wired board is
/// whatever the radio last carried. Firmware without the command simply never
/// answers, so an empty result here is the normal old-firmware case and says
/// nothing.
fn wired_report(device: &UsbDevice) -> Vec<String> {
    match usb_telemetry(device) {
        Some(telemetry) => describe_telemetry(&telemetry),
        None => Vec::new(),
    }
}

fn usb_telemetry(device: &UsbDevice) -> Option<Telemetry> {
    if device.bootloader.is_some() {
        return None; // a board in its bootloader runs no keyboard firmware
    }

    let conn = HidConnection::open(device.vid, device.pid).ok()?;
    conn.send(CMD_GET_TELEMETRY, &[]);

    let deadline = Instant::now() + TELEMETRY_WAIT;
    loop {
        let left = deadline.checked_duration_since(Instant::now())?;
        match conn.recv_timeout(left)? {
            HidEvent::Telemetry(telemetry) => return Some(telemetry),
            HidEvent::Disconnected => return None,
            _ => {} // the connection handshake, on its way past
        }
    }
}

/// The lines under a wired board: what each half has left, where each reading
/// came from, and what is plugged into it.
fn describe_telemetry(telemetry: &Telemetry) -> Vec<String> {
    let mut lines = Vec::new();

    let halves: Vec<String> = [("left", telemetry.left), ("right", telemetry.right)]
        .into_iter()
        .map(|(half, reading)| format!("{half} {}", read_as(reading)))
        .collect();
    lines.push(format!("battery · {}", halves.join(" · ")));

    let pointing = [
        ("build", telemetry.nav_build),
        ("left", telemetry.nav_left),
        ("right", telemetry.nav_right),
    ]
    .into_iter()
    .map(|(where_, nav)| format!("{where_} {}", named(nav)))
    .collect::<Vec<_>>();
    lines.push(format!("pointing · {}", pointing.join(" · ")));

    if let Some(line) = charging_as(telemetry.charging) {
        lines.push(line);
    }

    lines
}

/// How fast the board is charging and what decided that.
///
/// Two currents, because they are two different things and confusing them is
/// what the firmware used to do: what the board draws from the source, and
/// what reaches the cells. Nothing at all for firmware that predates the
/// fields — an empty line would read as a board that answered "none".
fn charging_as(charging: Charging) -> Option<String> {
    let state = charging.state?;
    let mut parts = vec![charging_state_as(state).to_string()];

    if let Some(ma) = charging.input_limit_ma {
        parts.push(format!("{ma} mA from the source"));
    }
    if let Some(ma) = charging.charge_current_ma {
        parts.push(format!("{ma} mA to each cell"));
    }
    // What the source said it could give, when that is not already obvious
    // from the limit we settled on.
    match charging.budget_ma {
        Some(0) | None => {}
        Some(ma) if Some(ma) == charging.input_limit_ma => {}
        Some(ma) => parts.push(format!("source offers {ma} mA")),
    }

    Some(format!("charging · {}", parts.join(" · ")))
}

fn charging_state_as(state: ChargeState) -> String {
    match state {
        ChargeState::Battery => "on battery".to_string(),
        ChargeState::Floor => "holding at the unconfigured limit".to_string(),
        ChargeState::Charging => "charging".to_string(),
        ChargeState::Full => "full".to_string(),
        ChargeState::Other(raw) => format!("a state this zapp does not know ({raw})"),
    }
}

/// One half's level, what it is doing, and which path the number took. The
/// path is worth the words: a relayed level keeps its last value when the link
/// carrying it stops, so it reads as current long after it stopped being so.
fn read_as(reading: BatteryReading) -> String {
    let level = match reading.level {
        Some(level) => format!("{level}%"),
        None => "unknown".to_string(),
    };
    let doing = match (reading.charging, reading.usb) {
        (Some(true), _) => " charging",
        (Some(false), Some(true)) => " on USB",
        _ => "",
    };
    match reading.source.filter(|_| reading.level.is_some()).and_then(came_from) {
        Some(from) => format!("{level}{doing} ({from})"),
        None => format!("{level}{doing}"),
    }
}

/// Where a reading came from, when that tells the reader something.
///
/// A half reporting its own cell is the ordinary case and saying so is noise.
/// The rest earn their words: over the cable is live, over the radio may be a
/// moment old, and a source this build does not know should not be dressed up
/// as one it does.
fn came_from(source: Source) -> Option<&'static str> {
    match source {
        Source::Nrf => None,
        Source::Wired => Some("read over the cable"),
        Source::Esb => Some("relayed over the radio"),
        Source::Unknown => Some("no source"),
        Source::Other(_) => Some("a source this zapp does not know"),
    }
}

/// What a nav mask names. An empty mask is a board that looked and found
/// nothing; no mask at all is one that could not tell, and the two must not
/// print the same.
fn named(nav: Option<Nav>) -> String {
    let Some(nav) = nav else {
        return "unknown".to_string();
    };
    let devices: Vec<&str> = [("trackball", nav.trackball), ("trackpad", nav.trackpad)]
        .into_iter()
        .filter(|&(_, present)| present)
        .map(|(name, _)| name)
        .collect();

    match devices.is_empty() {
        true => "none".to_string(),
        false => devices.join(" + "),
    }
}

/// What the halves have left.
///
/// A half the board does not publish is left out; one it publishes without a
/// level is named and called unknown, because a half that is there and quiet is
/// worth saying and a half that is not there is not.
fn charge(battery: Battery) -> String {
    let halves: Vec<String> = [("left", battery.left), ("right", battery.right)]
        .into_iter()
        .filter(|(_, level)| !level.is_absent())
        .map(|(half, level)| format!("{half} {}", said(level)))
        .collect();

    match halves.is_empty() {
        false => format!(" · {}", halves.join(" · ")),
        // A board with one battery, which reports only the whole.
        true => match battery.keyboard {
            Level::Percent(level) => format!(" · {level}%"),
            _ => String::new(),
        },
    }
}

/// Points at the wired entry for a board that is listed in both places. What
/// the radio published is what the board last had reason to publish; plugged
/// in, it answers for itself. Saying which is which is the whole difference
/// between a reading and a memory.
fn also_wired(keyboard: Option<device::Keyboard>, wired: &[device::Keyboard]) -> &'static str {
    match keyboard.is_some_and(|model| wired.contains(&model)) {
        true => " · also on USB above, which reads live",
        false => "",
    }
}

fn said(level: Level) -> String {
    match level {
        Level::Percent(level) => format!("{level}%"),
        _ => "unknown".to_string(),
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

fn cmd_flash_file(path: &PathBuf, force: bool) -> Result<()> {
    // A bundle carries both halves' wireless firmware and goes over the cable
    // through the bootloader; everything else is a keyboard image for the
    // STM32. Told apart by what the file says it is rather than by its name,
    // so a renamed bundle still does the right thing instead of failing deep
    // inside the DFU loader.
    if is_bundle(path) {
        return nrf_wired::flash(path, force, Duration::from_secs(15));
    }

    if force {
        // Said rather than ignored: a keyboard image is always written, so the
        // flag is not doing anything here and silently accepting it would
        // suggest otherwise.
        println!("--force only applies to .zsab bundles; a keyboard image is always written.");
    }

    let fw = firmware::load_firmware(path).context("Failed to load firmware")?;
    print_firmware_info(&fw);
    wait_and_flash(&fw)
}

/// Whether the file leads with the .zsab magic.
fn is_bundle(path: &Path) -> bool {
    let mut magic = [0u8; 4];
    File::open(path)
        .and_then(|mut file| file.read_exact(&mut magic))
        .is_ok_and(|()| &magic == b"ZSAB")
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
pub fn smp_connect(args: &LinkArgs, half: Half) -> Result<SmpClient> {
    let spinner = new_spinner(match half {
        Half::Left => "Connecting over Bluetooth...",
        Half::Right => "Connecting to the right half over Bluetooth...",
    });
    let client = smp::open(args.device.as_deref(), half);
    spinner.finish_and_clear();

    let mut client = client.with_context(|| match half {
        Half::Left => "Could not reach the board over Bluetooth".to_string(),
        Half::Right => "Could not reach the right half over Bluetooth. It is reached \
                        through the left, so the left must be running firmware with the \
                        relay in it, and the halves' radio link must be up."
            .to_string(),
    })?;
    if let Some(mtu) = args.mtu {
        client = client.with_mtu(mtu);
    }
    let mut client = client.with_timeout(Duration::from_secs(args.timeout));

    // The timeout has to be set first: this is a request like any other, and
    // it is the first thing the board is asked.
    match args.chunk {
        Some(chunk) => client = client.with_request_limit(chunk),
        None => {
            // What the board says it can reassemble is more than it will take.
            // Reassembly across writes carries no sequence numbers and no
            // checksum — neither Zephyr's nor the left relay's — so a fragment
            // lost anywhere discards the request silently and no reply ever
            // comes; the more writes a request spans, the more of that
            // exposure it buys. Measured on a Voyager 2's left half (reports
            // 2475, 249 per write): os_mgmt echo answers up to 8 writes and
            // stops dead at 9, and an upload of a 316 KB image takes 51 s at
            // one write, 25 s at five, 22 s at eight. Five keeps margin under
            // the ceiling for most of the gain.
            //
            // The right half is not measured: its ceiling is the 1024-byte
            // frame the relay and its own reassembly buffer share, which five
            // writes already exceeds, so the reported buffer is what binds and
            // the count only matters if that frame size changes. Same default
            // either way; `--writes` and `--chunk` override it.
            const WRITES_PER_REQUEST: usize = 5;
            let writes = args.writes.unwrap_or(WRITES_PER_REQUEST).max(1);
            let reported = client.negotiate_request_limit();
            let safe = (client.mtu() * writes).min(reported);
            if reported > safe {
                log::info!(
                    "board reports {reported} per request; using {safe} \
                     ({writes} writes) — pass --chunk or --writes to override"
                );
                client = client.with_request_limit(safe);
            }
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
    let mut client = smp_connect(&args.link, args.half)?;
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

    let mut client = smp_connect(&args.link, args.half)?;
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

    let mut client = smp_connect(&args.link, args.half)?;
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

/// Reads back the update the keyboard has been told about, and optionally ends
/// it.
///
/// Only the left half serves this — the right runs no session of its own, only
/// the claim the left repeats to it — so this takes no `--half`. It is the one
/// place to look after a refusal: a host that was turned down needs to see what
/// the board thinks is open before it can tell an expired session from a
/// half-finished one from another program holding the lock.
fn cmd_smp_session(close: bool, link: &LinkArgs) -> Result<()> {
    let mut client = smp_connect(link, Half::Left)?;
    let status = client
        .session_status()
        .context("Could not read the update session")?;
    print_session(&status);

    if !close {
        return Ok(());
    }
    if !status.open {
        println!();
        println!("Nothing to close.");
        return Ok(());
    }

    // By id, so this ends the session that was just read rather than whatever
    // happens to be open by the time the write lands.
    let closed = client
        .session_close(Some(status.id))
        .context("Could not close the update session")?;
    println!();
    if closed.open {
        println!("The board still reports session {} open.", closed.id);
    } else {
        println!("Session {} closed.", status.id);
    }
    Ok(())
}

fn print_session(status: &Status) {
    if status.open {
        println!(
            "Session {} · {} · {} s left",
            status.id,
            if status.version.is_empty() {
                "(no version)"
            } else {
                &status.version
            },
            status.ttl_ms / 1000
        );
    } else {
        println!("No update session is open.");
    }

    println!("  next   {}", next_of(status.next));
    println!("  left   {}", half_of(true, status.left_done));
    println!("  right  {}", half_of(status.right_seen, status.right_done));
    if !status.left_hash.is_empty() {
        println!(
            "  the left is running {}",
            smp::image::hex(&status.left_hash)
        );
    }
    if let Some(tag) = &status.right_tag {
        println!("  the right reports {}", smp::image::hex(tag));
    }
    println!(
        "  guard  {}",
        match status.guard {
            true => "on: this board accepts only a declared update over Bluetooth",
            false => "off: a development build, so the single-step subcommands work",
        }
    );
}

fn next_of(next: Next) -> String {
    match next {
        Next::None => "nothing is declared".to_string(),
        Next::Right => "the right half".to_string(),
        Next::Left => "the left half".to_string(),
        Next::Complete => "nothing: both halves carry what was declared".to_string(),
        Next::Unknown(code) => format!("a step this zapp does not know ({code})"),
    }
}

/// One half's line. A half nobody can read is not a half that is unfinished,
/// and the two must not print the same: the first is a keyboard to go and look
/// at, the second is one to wait for.
fn half_of(seen: bool, done: bool) -> &'static str {
    match (seen, done) {
        (false, _) => "not reporting: switched off, out of range, or older firmware",
        (true, false) => "not carrying its declared image, or has not confirmed it",
        (true, true) => "carrying its declared image, confirmed",
    }
}

fn cmd_smp_reset(args: &SmpArgs) -> Result<()> {
    let mut client = smp_connect(&args.link, args.half)?;
    client.reset().context("Reset failed")?;
    println!("Resetting.");
    Ok(())
}

/// Connects to the update service on the board these arguments name.
fn dfu_connect(args: &DfuArgs) -> Result<DfuClient> {
    let spinner = new_spinner("Connecting over Bluetooth...");
    let client = dfu::open(args.device.as_deref());
    spinner.finish_and_clear();

    let mut client = client.context("Could not reach the keyboard over Bluetooth")?;
    if let Some(chunk) = args.chunk {
        client = client.with_chunk(chunk);
    }
    log::debug!("connected, {} bytes of image per write", client.chunk());
    Ok(client)
}

/// The board's own advice about a refusal, wherever in the chain it is.
fn dfu_advice(err: &anyhow::Error) -> Option<&'static str> {
    err.chain()
        .find_map(|cause| cause.downcast_ref::<DfuError>())
        .and_then(DfuError::advice)
}

fn cmd_dfu_flash(path: &Path, args: &DfuArgs) -> Result<()> {
    let file =
        std::fs::read(path).with_context(|| format!("Failed to read {}", path.display()))?;
    let image = firmware::load_raw_image(&file)
        .with_context(|| format!("{} is not a firmware image this can flash", path.display()))?;

    if image.data.is_empty() {
        bail!("{} has no firmware in it", path.display());
    }
    // Refused here rather than by the keyboard: the board answers this with one
    // error byte, and the sentence that names the file and both figures is the
    // one that says what to do about it.
    if image.data.len() > dfu::MAX_IMAGE {
        bail!(
            "{} is {} bytes, and the STM32's application region holds {} ({} KiB). \
             This is a firmware built for another keyboard, or one built to run \
             without the ignition bootloader under it.",
            path.display(),
            image.data.len(),
            dfu::MAX_IMAGE,
            dfu::MAX_IMAGE / 1024
        );
    }

    println!(
        "Firmware: {} bytes{}",
        image.data.len(),
        match image.pid {
            Some(pid) => format!(" · {}", target_name_for_pid(pid)),
            None => String::new(),
        }
    );
    println!("CRC-32:   {:08x}", dfu::crc32(image.data));
    // Not fatal, because nothing here can prove what is on the other end of the
    // radio — but a Voyager image on a Voyager 2 is a keyboard that comes back
    // in its bootloader, and the file said which one it was for.
    if let Some(pid) = image.pid
        && !matches!(pid, 0x1978 | 0x0792)
    {
        println!(
            "Warning: this firmware says it is for a {}, and this command flashes a \
             Voyager 2.",
            target_name_for_pid(pid)
        );
    }

    let mut client = dfu_connect(args)?;
    println!();
    client
        .make_ready(Duration::from_secs(args.wait), &report_wait)
        .context("The STM32 is not in its bootloader, so nothing was written")?;

    let pb = ProgressBar::new(image.data.len() as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("⚡ {bar:40.cyan/blue} {bytes}/{total_bytes} {bytes_per_sec} {msg}")
            .unwrap()
            .progress_chars("█▓░"),
    );

    let started = Instant::now();
    let result = client.flash(image.data, &|progress| match progress {
        FlashProgress::Erasing { bytes_erased, .. } => {
            pb.set_message(match bytes_erased {
                0 => "Erasing...",
                _ => "Erased",
            });
        }
        FlashProgress::Writing { bytes_written, .. } => {
            pb.set_position(bytes_written as u64);
            pb.set_message("Writing...");
        }
        FlashProgress::Resetting => pb.set_message("Checking the image..."),
        FlashProgress::Complete => pb.finish_with_message("Done!"),
    });
    pb.finish_and_clear();
    result.context("The flash failed")?;

    let took = started.elapsed();
    println!(
        "Wrote {} bytes in {:.1} s ({:.1} KiB/s).",
        image.data.len(),
        took.as_secs_f64(),
        image.data.len() as f64 / took.as_secs_f64() / 1024.0
    );
    println!("The STM32 accepted the image and is restarting on it.");
    Ok(())
}

/// What the readiness step is doing, said out loud.
fn report_wait(wait: Wait) {
    match wait {
        Wait::Already => println!("The STM32 is already in its bootloader."),
        Wait::Rebooting => println!("Rebooting the STM32 into its bootloader..."),
        Wait::Waiting(waited) => println!(
            "  not there yet after {} s — pressing the keyboard's reset button also works",
            waited.as_secs()
        ),
        Wait::Arrived(waited) => println!("  in the bootloader after {} s", waited.as_secs()),
    }
}

fn cmd_dfu_status(args: &DfuArgs) -> Result<()> {
    let report = dfu_connect(args)?
        .query()
        .context("Could not ask the keyboard what the STM32 is doing")?;

    println!(
        "STM32: {}",
        match report.status.in_bootloader() {
            true => "in the ignition bootloader, ready to be flashed",
            false => "running its own firmware",
        }
    );
    println!("  the board reports {}", report.status);
    if report.fault != dfu::Fault::None {
        println!("  {}", report.fault);
    }
    if let Some(received) = report.received.filter(|received| *received > 0) {
        println!("  {received} bytes of a transfer have reached it");
    }
    Ok(())
}

fn cmd_dfu_enter(args: &DfuArgs) -> Result<()> {
    let mut client = dfu_connect(args)?;
    client
        .make_ready(Duration::from_secs(args.wait), &report_wait)
        .context("The STM32 never reached its bootloader")?;
    println!();
    println!("Ready. `zapp dfu flash <firmware.bin>` writes an image into it.");
    Ok(())
}

fn cmd_dfu_abort(args: &DfuArgs) -> Result<()> {
    let report = dfu_connect(args)?
        .abort()
        .context("Could not drop the transfer")?;
    println!("The board reports {}.", report.status);
    println!("Nothing holds the update lock now.");
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_halves_of_a_split_board_are_named() {
        let battery = Battery {
            left: Level::Percent(0),
            right: Level::Percent(27),
            keyboard: Level::Percent(0),
        };
        assert_eq!(charge(battery), " · left 0% · right 27%");
    }

    #[test]
    fn a_half_that_is_there_and_would_not_say_is_called_unknown() {
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Unknown,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery), " · left 36% · right unknown");
    }

    #[test]
    fn a_half_the_board_does_not_have_is_left_out() {
        // Not the same as one that would not say: naming it would invent a half.
        let battery = Battery {
            left: Level::Percent(36),
            right: Level::Absent,
            keyboard: Level::Percent(36),
        };
        assert_eq!(charge(battery), " · left 36%");
    }

    #[test]
    fn a_board_with_one_battery_reports_the_whole() {
        let battery = Battery {
            keyboard: Level::Percent(80),
            ..Battery::default()
        };
        assert_eq!(charge(battery), " · 80%");
    }

    fn reading(level: Option<u8>, source: Source) -> BatteryReading {
        BatteryReading {
            level,
            charging: Some(false),
            usb: Some(false),
            source: Some(source),
        }
    }

    #[test]
    fn a_wired_board_says_where_each_level_came_from() {
        let telemetry = Telemetry {
            version: 1,
            left: reading(Some(87), Source::Nrf),
            right: reading(Some(64), Source::Wired),
            nav_build: Some(Nav {
                trackball: true,
                trackpad: false,
            }),
            nav_left: Some(Nav::default()),
            nav_right: Some(Nav {
                trackball: true,
                trackpad: false,
            }),
            charging: Charging {
                budget_ma: Some(1500),
                input_limit_ma: Some(700),
                charge_current_ma: Some(550),
                state: Some(ChargeState::Charging),
            },
        };
        assert_eq!(
            describe_telemetry(&telemetry),
            vec![
                "battery · left 87% · right 64% (read over the cable)",
                "pointing · build trackball · left none · right trackball",
                "charging · charging · 700 mA from the source · 550 mA to each cell · source offers 1500 mA",
            ]
        );
    }

    #[test]
    fn the_two_currents_are_kept_apart() {
        // What the board draws and what reaches the cells are different
        // numbers, and reporting one as the other is the bug this line exists
        // to make visible.
        assert_eq!(
            charging_as(Charging {
                budget_ma: Some(500),
                input_limit_ma: Some(200),
                charge_current_ma: Some(50),
                state: Some(ChargeState::Charging),
            }),
            Some(
                "charging · charging · 200 mA from the source · 50 mA to each cell · source offers 500 mA"
                    .to_string()
            )
        );
    }

    #[test]
    fn a_source_that_has_not_said_what_it_is_reads_as_held_back() {
        assert_eq!(
            charging_as(Charging {
                budget_ma: Some(0),
                input_limit_ma: Some(100),
                charge_current_ma: Some(32),
                state: Some(ChargeState::Floor),
            }),
            Some(
                "charging · holding at the unconfigured limit · 100 mA from the source · 32 mA to each cell"
                    .to_string()
            )
        );
    }

    #[test]
    fn a_source_offering_more_than_we_take_says_so() {
        // A 3 A charger against a board whose input register stops at 1.5 A:
        // the difference is real and the line has to show it.
        assert_eq!(
            charging_as(Charging {
                budget_ma: Some(3000),
                input_limit_ma: Some(1500),
                charge_current_ma: Some(700),
                state: Some(ChargeState::Charging),
            }),
            Some(
                "charging · charging · 1500 mA from the source · 700 mA to each cell · source offers 3000 mA"
                    .to_string()
            )
        );
    }

    #[test]
    fn firmware_without_the_charging_fields_says_nothing_at_all() {
        // Not an empty line and not zeroes: an old board did not answer this
        // question, which is different from answering "none".
        assert_eq!(charging_as(Charging::default()), None);
    }

    #[test]
    fn a_relayed_level_is_not_dressed_up_as_a_live_one() {
        // The distinction this whole payload exists for: same number, and the
        // line has to say which of the two it is.
        assert_eq!(
            read_as(reading(Some(64), Source::Wired)),
            "64% (read over the cable)"
        );
        assert_eq!(
            read_as(reading(Some(64), Source::Esb)),
            "64% (relayed over the radio)"
        );
    }

    #[test]
    fn a_level_that_was_not_read_is_not_a_flat_one() {
        assert_eq!(read_as(reading(None, Source::Wired)), "unknown");
        assert_eq!(
            read_as(reading(Some(0), Source::Wired)),
            "0% (read over the cable)"
        );
    }

    #[test]
    fn unknown_flags_do_not_become_a_confident_no() {
        let unsure = BatteryReading {
            level: Some(50),
            charging: None,
            usb: None,
            source: Some(Source::Nrf),
        };
        assert_eq!(read_as(unsure), "50%");

        let charging = BatteryReading {
            charging: Some(true),
            ..unsure
        };
        assert_eq!(read_as(charging), "50% charging");
    }

    #[test]
    fn a_half_we_could_not_read_is_not_a_half_with_nothing_in_it() {
        assert_eq!(named(None), "unknown");
        assert_eq!(named(Some(Nav::default())), "none");
        assert_eq!(
            named(Some(Nav {
                trackball: true,
                trackpad: true
            })),
            "trackball + trackpad"
        );
    }

    #[test]
    fn a_board_listed_twice_points_at_its_live_entry() {
        let wired = [device::Keyboard::Voyager2];
        assert_eq!(
            also_wired(Some(device::Keyboard::Voyager2), &wired),
            " · also on USB above, which reads live"
        );
        assert_eq!(also_wired(Some(device::Keyboard::Voyager), &wired), "");
        assert_eq!(also_wired(None, &wired), "");
        assert_eq!(also_wired(Some(device::Keyboard::Voyager2), &[]), "");
    }

    #[test]
    fn a_board_that_said_nothing_says_nothing() {
        assert_eq!(charge(Battery::default()), "");
        // Including one connected but unreadable, which must not read as flat.
        let battery = Battery {
            keyboard: Level::Unknown,
            ..Battery::default()
        };
        assert_eq!(charge(battery), "");
    }
}
