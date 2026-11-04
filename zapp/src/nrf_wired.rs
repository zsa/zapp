//! Updating both nRFs over the cable, and pairing the halves, through
//! ignition. The orchestration is `zapp_update::wired`; this is what the
//! terminal shows of it.

use std::time::Duration;

use anyhow::{anyhow, Result};

use zapp_smp::image;
use zapp_smp::settings::{self, EsbConfig};
use zapp_update::text::{named, short, sizes};
use zapp_update::wired::{self, HalfKey, MatchState, ShowReport};
use zapp_update::{Bundle, Half, Options, Outcome};

use crate::screen::Screen;
use crate::zsab::UP_TO_DATE;

/// Whether a successful run should leave the STM32 in Ignition rather than
/// restart it into the keyboard firmware, which would overwrite the RAM a
/// `make DIAG=1` Ignition keeps its record in. Press reset afterwards and read
/// the record with zapp-wired's read_diag example.
fn keep_ignition() -> bool {
    std::env::var("ZAPP_KEEP_IGNITION").is_ok_and(|v| v != "0" && !v.is_empty())
}

/// Whether a failed run should leave the keyboard exactly as it is, for
/// reading a half's own counters over SWD before anything zeroes them.
fn keep_state_on_failure() -> bool {
    std::env::var("ZAPP_KEEP_STATE_ON_FAILURE").is_ok_and(|v| v != "0" && !v.is_empty())
}

/// Reply timeout override, for measuring a half that answers slowly rather
/// than not at all. Seconds; unset or unparsable means the library's default.
fn reply_timeout() -> Duration {
    std::env::var("ZAPP_REPLY_TIMEOUT_SECS")
        .ok()
        .and_then(|secs| secs.parse().ok())
        .map(Duration::from_secs)
        .unwrap_or(Options::default().reply_timeout)
}

/// Writes both halves from one bundle.
pub fn flash(bundle: &Bundle, force: bool) -> Result<()> {
    announce(bundle);

    let options = Options {
        force,
        reply_timeout: reply_timeout(),
        keep_state_on_failure: keep_state_on_failure(),
        keep_ignition: keep_ignition(),
        ..Options::default()
    };
    let mut screen = Screen::default();
    match wired::update_bundle(bundle, &options, &mut |event| screen.show(event))? {
        // Nothing was written and nothing was checked, so this one line is the
        // whole of the output.
        Outcome::UpToDate => println!("{UP_TO_DATE}"),
        // Left in Ignition, the keyboard was not checked.
        Outcome::Flashed if options.keep_ignition => {}
        Outcome::Flashed => println!("Done!"),
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Printing
// ---------------------------------------------------------------------------

fn announce(bundle: &Bundle) {
    println!(
        "Bundle loaded: {} ({}) flashing over USB",
        named(&bundle.header.build_version),
        sizes(bundle)
    );
    for half in [Half::Left, Half::Right] {
        if let Some(info) = image::inspect(bundle.image(half)) {
            let named_hash = bundle.hash(half).map(|hash| short(&hash)).unwrap_or_default();
            log::debug!("{half}: v{} {named_hash}", info.version);
        }
    }
    log::debug!("radio key fingerprint {:08x}", bundle.header.key_fingerprint);
}

// ---------------------------------------------------------------------------
// Bonding: the ESB pairing key onto both halves
// ---------------------------------------------------------------------------

/// Provisions a pairing key onto both halves over the cable.
///
/// Generates a fresh key unless one is given as 60 hex characters.
pub fn bond(key: Option<&str>) -> Result<()> {
    let config = match key {
        Some(text) => EsbConfig::parse_hex(text)
            .map_err(|err| anyhow!("that is not a usable pairing key: {err}"))?,
        None => EsbConfig::generate(),
    };

    match key {
        Some(_) => println!("Using the pairing key you supplied."),
        None => println!("Generated a fresh pairing key."),
    }
    describe_config(&config);

    let options = Options {
        reply_timeout: reply_timeout(),
        ..Options::default()
    };
    let mut screen = Screen::default();
    let bonded = wired::bond(&config, &options, &mut |event| screen.show(event))?;
    prove_bond(bonded.came_back);
    Ok(())
}

/// Reads both halves' pairing keys and reports whether they match, writing
/// nothing.
///
/// Prints the two fingerprints and whether they agree.
pub fn bond_show() -> Result<()> {
    let mut screen = Screen::default();
    let report = wired::bond_show(&Options::default(), &mut |event| screen.show(event))?;
    print_show(&report);
    Ok(())
}

/// Says what was written, and never the key: the AES key reaches neither a log
/// nor a screen, so a fingerprint stands in for it.
fn describe_config(config: &EsbConfig) {
    println!("  {:<15} {}", "channel", config.rf_channel());
    println!(
        "  {:<15} {}:{}",
        "address",
        settings::hex(config.base0()),
        settings::hex(&[config.prefix0()])
    );
    println!("  {:<15} {}", "device id", settings::hex(config.device_id()));
    println!(
        "  {:<15} {}",
        "key fingerprint",
        settings::hex(&config.key_fingerprint())
    );
}

/// What the restart check's answer means for a bond.
///
/// The bytes are on both halves and read back before this runs, so nothing here
/// undoes the provisioning. What it shows is that both halves rebooted into
/// their applications and answer, not that the radio link is up — that is only
/// visible on battery with no cables. A run that does not see both halves come
/// back exits non-zero, the way the flash's does.
fn prove_bond(came_back: bool) {
    if came_back {
        println!();
        println!(
            "The pairing key was written and read back on both halves; both halves answered \
             after the restart."
        );
    } else {
        println!();
        println!("{}", unproven_bond());
        std::process::exit(1);
    }
}

/// What a bond whose halves did not come back on USB says.
fn unproven_bond() -> String {
    "The pairing key was written and read back on both halves, but the keyboard did not come \
     back on USB with both halves answering."
        .to_string()
}

/// The line one half's key reads as, fingerprint only.
fn show_line(name: &str, key: &HalfKey) -> String {
    let label = format!("{name} half");
    match key {
        HalfKey::Provisioned(bytes) => {
            format!("  {label:<10}: {}", settings::fingerprint_hex(bytes))
        }
        HalfKey::NotProvisioned => format!("  {label:<10}: not provisioned"),
    }
}

/// The verdict line, from whether the two halves' keys agree.
fn show_verdict(state: MatchState) -> String {
    match state {
        MatchState::Paired => "The two halves carry the same pairing key.".to_string(),
        MatchState::Mismatched => "The two halves carry different pairing keys.".to_string(),
        MatchState::Incomplete => "At least one half is not provisioned.".to_string(),
    }
}

fn print_show(report: &ShowReport) {
    println!("{}", show_line("Left", &report.left));
    println!("{}", show_line("Right", &report.right));
    println!("{}", show_verdict(report.match_state()));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fixed, valid pairing bundle: channel 42, base0 1..4, prefix 5, and a
    /// distinct key so its fingerprint is stable across runs.
    fn a_config() -> EsbConfig {
        let mut bytes = [0u8; settings::ESB_CONFIG_LEN];
        for (i, byte) in bytes.iter_mut().enumerate() {
            *byte = i as u8;
        }
        bytes[0] = 42; // rf_channel
        EsbConfig::from_bytes(bytes).expect("a valid fixed bundle")
    }

    #[test]
    fn the_match_verdict_says_whether_the_two_keys_agree() {
        assert!(show_verdict(MatchState::Mismatched).contains("different pairing keys"));
        assert!(show_verdict(MatchState::Paired).contains("same pairing key"));
        assert!(show_verdict(MatchState::Incomplete).contains("not provisioned"));
    }

    #[test]
    fn a_show_line_carries_a_fingerprint_and_never_the_key() {
        let config = a_config();
        let line = show_line("Left", &HalfKey::Provisioned(config.as_bytes().to_vec()));
        // The fingerprint, not the 30 bytes.
        assert!(line.contains(&settings::fingerprint_hex(config.as_bytes())));
        // None of the AES-key bytes spelled out in full.
        let key_hex = settings::hex(&config.as_bytes()[14..30]);
        assert!(!line.contains(&key_hex), "the key leaked into the line: {line}");
    }

    #[test]
    fn the_unproven_bond_message_says_what_did_not_happen() {
        let said = unproven_bond();
        assert!(said.contains("read back on both halves"), "{said}");
        assert!(said.contains("did not come back"), "{said}");
        assert!(!said.contains("run bond again"), "{said}");
    }
}
