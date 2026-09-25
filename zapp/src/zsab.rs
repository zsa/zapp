//! Updating a whole keyboard from one `.zsab` bundle, over Bluetooth. The
//! orchestration is `zapp_update::ble`; this is what the terminal shows of it.

use std::path::Path;

use anyhow::Result;

use zapp_ble::smp::{self, Half};
use zapp_update::text::{named, short, sizes};
use zapp_update::{ble, Bundle, Options, Outcome};

use crate::screen::Screen;

/// A keyboard that already carries the bundle, said the same way whichever way
/// the bundle was going to travel.
pub(crate) const UP_TO_DATE: &str = "The keyboard is up to date, skipping the flash.";

/// Updates both halves from a bundle.
///
/// `device` picks between bonded boards, by advertised name or by the handle
/// the device list shows.
pub fn update(path: &Path, force: bool, device: Option<&str>) -> Result<()> {
    let bundle = zapp_update::load_bundle(path)?;
    announce(&bundle, path);

    let options = Options {
        force,
        device: device.map(str::to_string),
        ..Options::default()
    };
    let mut screen = Screen::default();
    match ble::update_bundle(&bundle, &options, &mut |event| screen.show(event))? {
        Outcome::UpToDate => println!("{UP_TO_DATE}"),
        Outcome::Flashed => println!("Done!"),
    }
    Ok(())
}

fn announce(bundle: &Bundle, path: &Path) {
    println!(
        "Bundle loaded: {} ({}) flashing over Bluetooth",
        named(&bundle.header.build_version),
        sizes(bundle)
    );
    let len = std::fs::metadata(path).map(|meta| meta.len()).unwrap_or_default();
    log::debug!("{} · {len} bytes", path.display());
    for half in [Half::Left, Half::Right] {
        let image = bundle.image(half);
        let version = smp::image::inspect(image)
            .map(|info| info.version)
            .unwrap_or_else(|| "no MCUboot header".to_string());
        log::debug!(
            "{:<5}  v{version} · {} bytes · {}",
            half.as_str(),
            image.len(),
            bundle
                .hash(half)
                .map(|hash| short(&hash))
                .unwrap_or_else(|| "no hash".to_string())
        );
    }
    // Nothing stamps the fingerprint yet: shared/cmake/zsa_esb_key.cmake is not
    // wired into either app, so every bundle built today carries zero and it is
    // read as "unknown" rather than as a mismatch.
    log::debug!("radio key fingerprint {:08x}", bundle.header.key_fingerprint);
}
