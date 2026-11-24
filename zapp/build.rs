//! Embeds an Info.plist in the macOS binary.
//!
//! macOS will not let a process near Bluetooth without a usage description to
//! show in the permission prompt, and a command-line tool has no bundle to keep
//! one in. Linked into `__TEXT,__info_plist`, the binary carries its own: it is
//! what TCC reads when zapp is the process responsible for itself — launched
//! by something other than a terminal that already holds the permission —
//! where without it the first CoreBluetooth call would end the process.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("macos") {
        return;
    }

    let version = env::var("CARGO_PKG_VERSION").expect("cargo sets the package version");
    let plist = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>CFBundleIdentifier</key>
	<string>com.zsa.zapp</string>
	<key>CFBundleName</key>
	<string>zapp</string>
	<key>CFBundleShortVersionString</key>
	<string>{version}</string>
	<key>CFBundleVersion</key>
	<string>{version}</string>
	<key>NSBluetoothAlwaysUsageDescription</key>
	<string>zapp talks to ZSA keyboards over Bluetooth to list them and to update their firmware.</string>
</dict>
</plist>
"#
    );

    let path = PathBuf::from(env::var("OUT_DIR").expect("cargo sets OUT_DIR")).join("Info.plist");
    fs::write(&path, plist).expect("write Info.plist");
    println!(
        "cargo:rustc-link-arg-bins=-Wl,-sectcreate,__TEXT,__info_plist,{}",
        path.display()
    );
}
