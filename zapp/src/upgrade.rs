//! `zapp upgrade`: replaces the running zapp with the newest release from CrabNebula Cloud.

use std::io::{self, BufRead, Cursor, IsTerminal, Read, Write};
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use base64::Engine as _;
use base64::engine::general_purpose::STANDARD as B64;
use serde::Deserialize;

/// Update endpoint; `/<platform>/<current version>` is appended.
const ENDPOINT: &str = "https://cdn.crabnebula.app/update/zsa-technology-labs-inc/io-zsa-zapp";
/// Minisign public key (Tauri format: base64 of the key file) the archives are signed with.
const PUBLIC_KEY: &str = "dW50cnVzdGVkIGNvbW1lbnQ6IG1pbmlzaWduIHB1YmxpYyBrZXk6IEU5QkQ1OUJGNzc1MEQzMApSV1F3RFhYM205V2JEdW5RU3pyZWptK3VwbXVpc2Y1TVJHMFZsb0N4enl2RlNQUGR6ajVYSGJRbgo=";
const CURRENT: &str = env!("CARGO_PKG_VERSION");

const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(300);

#[cfg(windows)]
const EXE_NAME: &str = "zapp.exe";
#[cfg(not(windows))]
const EXE_NAME: &str = "zapp";

/// What the update server offers.
#[derive(Debug, Deserialize, PartialEq)]
struct Release {
    version: String,
    #[serde(default)]
    notes: Option<String>,
    url: String,
    signature: String,
}

/// Checks for a newer release and, unless `check` is set, installs it.
pub fn run(yes: bool, check: bool) -> Result<()> {
    let platform = platform_for(std::env::consts::OS, std::env::consts::ARCH).ok_or_else(|| {
        anyhow!(
            "zapp cannot update itself on this platform ({} {}).",
            std::env::consts::OS,
            std::env::consts::ARCH
        )
    })?;
    let client = reqwest::blocking::Client::builder()
        .user_agent(concat!("zapp/", env!("CARGO_PKG_VERSION")))
        .connect_timeout(CONNECT_TIMEOUT)
        .timeout(REQUEST_TIMEOUT)
        .build()
        .context("could not set up the HTTP client")?;

    let url = format!("{ENDPOINT}/{platform}/{CURRENT}");
    let response = client.get(&url).send().map_err(network_error)?;
    let status = response.status().as_u16();
    let body = response.bytes().map_err(network_error)?;
    let release = match parse_response(status, &body, platform)? {
        Some(release) if is_newer(&release.version, CURRENT)? => release,
        _ => {
            println!("zapp {CURRENT} is up to date.");
            return Ok(());
        }
    };

    println!(
        "zapp {} is available (you have {CURRENT}).",
        release.version
    );
    if let Some(notes) = release
        .notes
        .as_deref()
        .map(str::trim)
        .filter(|n| !n.is_empty())
    {
        println!();
        println!("{notes}");
        println!();
    }
    if check {
        return Ok(());
    }

    let exe = install_path()?;
    let staging = stage_next_to(&exe)?;

    if !yes {
        if !io::stdin().is_terminal() {
            println!("Run `zapp upgrade --yes` to install it.");
            return Ok(());
        }
        if !confirm("Update now? [y/N] ")? {
            return Ok(());
        }
    }

    println!("Downloading zapp {}...", release.version);
    let response = client.get(&release.url).send().map_err(network_error)?;
    let status = response.status();
    if !status.is_success() {
        bail!("The download failed (HTTP {}).", status.as_u16());
    }
    let archive = response.bytes().map_err(network_error)?;

    verify(&archive, &release.signature, PUBLIC_KEY)
        .map_err(|_| anyhow!("The download's signature does not match. Nothing was changed."))?;
    let binary = extract(&archive, EXE_NAME)?;

    let mut staging = staging;
    staging
        .write_all(&binary)
        .and_then(|()| staging.flush())
        .with_context(|| {
            format!(
                "could not write to {}",
                exe.parent().unwrap_or(&exe).display()
            )
        })?;
    let staged = staging.into_temp_path();
    self_replace::self_replace(&staged)
        .with_context(|| format!("could not replace {}", exe.display()))?;
    drop(staged);

    println!("Updated to {}.", release.version);
    Ok(())
}

/// The CrabNebula update platform for an OS and architecture.
fn platform_for(os: &str, arch: &str) -> Option<&'static str> {
    Some(match (os, arch) {
        ("linux", "x86_64") => "linux-x86_64",
        ("linux", "aarch64") => "linux-aarch64",
        ("macos", "x86_64") => "darwin-x86_64",
        ("macos", "aarch64") => "darwin-aarch64",
        ("windows", "x86_64") => "windows-x86_64",
        ("windows", "aarch64") => "windows-aarch64",
        _ => return None,
    })
}

/// The release in a reply, or `None` for 204 (nothing newer).
fn parse_response(status: u16, body: &[u8], platform: &str) -> Result<Option<Release>> {
    match status {
        200 => serde_json::from_slice(body)
            .map(Some)
            .map_err(|_| anyhow!("The update server sent a reply zapp cannot read.")),
        204 => Ok(None),
        400 | 404 => bail!("The update server has no zapp release for {platform}."),
        other => bail!("The update server answered with HTTP {other}."),
    }
}

/// Whether `offered` is a later version than `current`.
fn is_newer(offered: &str, current: &str) -> Result<bool> {
    let parse = |v: &str| {
        semver::Version::parse(v.trim().trim_start_matches('v'))
            .map_err(|_| anyhow!("The update server sent an unreadable version: {v}."))
    };
    Ok(parse(offered)? > parse(current)?)
}

/// Checks `data` against a Tauri-format minisign signature and public key.
fn verify(data: &[u8], signature_b64: &str, public_key_b64: &str) -> Result<()> {
    let decode = |b64: &str| -> Result<String> { Ok(String::from_utf8(B64.decode(b64.trim())?)?) };
    let key = minisign_verify::PublicKey::decode(&decode(public_key_b64)?)?;
    let signature = minisign_verify::Signature::decode(&decode(signature_b64)?)?;
    key.verify(data, &signature, false)?;
    Ok(())
}

/// The file named `name` from a .tar.gz or .zip archive.
fn extract(archive: &[u8], name: &str) -> Result<Vec<u8>> {
    let found = if archive.starts_with(&[0x1f, 0x8b]) {
        extract_tar_gz(archive, name)?
    } else if archive.starts_with(b"PK\x03\x04") {
        extract_zip(archive, name)?
    } else {
        bail!("The download is not an archive zapp can read.");
    };
    found.ok_or_else(|| anyhow!("The download does not contain {name}."))
}

fn extract_tar_gz(archive: &[u8], name: &str) -> Result<Option<Vec<u8>>> {
    let unreadable = |_| anyhow!("The download is not an archive zapp can read.");
    let mut tar = tar::Archive::new(flate2::read::GzDecoder::new(archive));
    for entry in tar.entries().map_err(unreadable)? {
        let mut entry = entry.map_err(unreadable)?;
        let path = entry.path().map_err(unreadable)?;
        if entry.header().entry_type().is_file() && file_name_is(&path, name) {
            let mut data = Vec::new();
            entry.read_to_end(&mut data).map_err(unreadable)?;
            return Ok(Some(data));
        }
    }
    Ok(None)
}

fn extract_zip(archive: &[u8], name: &str) -> Result<Option<Vec<u8>>> {
    let unreadable = |_| anyhow!("The download is not an archive zapp can read.");
    let mut zip = zip::ZipArchive::new(Cursor::new(archive)).map_err(unreadable)?;
    for index in 0..zip.len() {
        let mut entry = zip.by_index(index).map_err(unreadable)?;
        let matches = entry.is_file()
            && entry
                .enclosed_name()
                .is_some_and(|path| file_name_is(&path, name));
        if matches {
            let mut data = Vec::new();
            entry
                .read_to_end(&mut data)
                .map_err(|_| anyhow!("The download is not an archive zapp can read."))?;
            return Ok(Some(data));
        }
    }
    Ok(None)
}

fn file_name_is(path: &Path, name: &str) -> bool {
    path.file_name().is_some_and(|file| file == name)
}

/// The running executable, with symlinks resolved.
fn install_path() -> Result<PathBuf> {
    let exe = std::env::current_exe().context("could not locate the running zapp")?;
    exe.canonicalize()
        .with_context(|| format!("could not locate the running zapp ({})", exe.display()))
}

/// The package manager that owns `exe`, if its path says so.
fn package_manager(exe: &Path) -> Option<&'static str> {
    let path = exe.to_string_lossy();
    if path.starts_with("/nix/store/") {
        Some("Nix")
    } else if path.contains("/Cellar/") {
        Some("Homebrew")
    } else if path.contains("/mise/installs/") {
        Some("mise")
    } else if cfg!(target_os = "linux")
        && path.starts_with("/usr/")
        && !path.starts_with("/usr/local/")
    {
        Some("your package manager")
    } else {
        None
    }
}

/// A temporary file beside `exe`, or why zapp cannot replace itself there.
fn stage_next_to(exe: &Path) -> Result<tempfile::NamedTempFile> {
    if let Some(manager) = package_manager(exe) {
        bail!(
            "zapp is installed at {}, which {manager} manages. Update it through {manager}.",
            exe.display()
        );
    }
    let dir = exe
        .parent()
        .ok_or_else(|| anyhow!("could not locate the folder zapp is installed in"))?;
    tempfile::Builder::new()
        .prefix(".zapp-upgrade")
        .tempfile_in(dir)
        .map_err(|err| match err.kind() {
            io::ErrorKind::PermissionDenied | io::ErrorKind::ReadOnlyFilesystem => anyhow!(
                "zapp is installed at {}, and that folder is not writable. \
                 Run the upgrade with permission to write there.",
                exe.display()
            ),
            _ => anyhow!(err).context(format!("could not write to {}", dir.display())),
        })
}

fn confirm(prompt: &str) -> Result<bool> {
    print!("{prompt}");
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().lock().read_line(&mut answer)?;
    Ok(matches!(
        answer.trim().to_ascii_lowercase().as_str(),
        "y" | "yes"
    ))
}

fn network_error(err: reqwest::Error) -> anyhow::Error {
    log::debug!("update request failed: {err:?}");
    if err.is_timeout() {
        anyhow!("The update server did not answer in time.")
    } else if err.is_connect() {
        anyhow!("Cannot reach the update server. Check the internet connection.")
    } else {
        anyhow!("The update request failed: {err}.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Throwaway key that signed the fixture; not the release key.
    const TEST_PUBLIC_KEY: &str = "dW50cnVzdGVkIGNvbW1lbnQ6IG1pbmlzaWduIHB1YmxpYyBrZXk6IEE4MjcyMDI5REIxQkJEM0YKUldRL3ZSdmJLU0FucURwMlg1Q0NHY0VsZTFSYW5kVThZdUQ4TUNvOEhXdFlreDNUa2dNaW1hSHUK";
    /// Holds `README` and `zapp` (contents "zapp test binary\n").
    const TAR_GZ: &[u8] = include_bytes!("../testdata/upgrade/zapp-test.tar.gz");
    const TAR_GZ_SIG: &str = include_str!("../testdata/upgrade/zapp-test.tar.gz.sig");
    /// Holds `README` and `zapp.exe` (contents "zapp test binary\n").
    const ZIP: &[u8] = include_bytes!("../testdata/upgrade/zapp-test.zip");

    #[test]
    fn platforms() {
        assert_eq!(platform_for("linux", "x86_64"), Some("linux-x86_64"));
        assert_eq!(platform_for("linux", "aarch64"), Some("linux-aarch64"));
        assert_eq!(platform_for("macos", "x86_64"), Some("darwin-x86_64"));
        assert_eq!(platform_for("macos", "aarch64"), Some("darwin-aarch64"));
        assert_eq!(platform_for("windows", "x86_64"), Some("windows-x86_64"));
        assert_eq!(platform_for("windows", "aarch64"), Some("windows-aarch64"));
        assert_eq!(platform_for("freebsd", "x86_64"), None);
        assert_eq!(platform_for("linux", "riscv64"), None);
    }

    #[test]
    fn newer_versions() {
        assert!(is_newer("1.1.11", "1.1.10").unwrap());
        assert!(is_newer("1.2.0", "1.1.10").unwrap());
        assert!(is_newer("v1.1.11", "1.1.10").unwrap());
        assert!(!is_newer("1.1.10", "1.1.10").unwrap());
        assert!(!is_newer("1.1.9", "1.1.10").unwrap());
        assert!(!is_newer("1.1.11-beta.1", "1.1.11").unwrap());
        assert!(is_newer("nonsense", "1.1.10").is_err());
    }

    #[test]
    fn no_content_means_up_to_date() {
        assert_eq!(parse_response(204, b"", "linux-x86_64").unwrap(), None);
    }

    #[test]
    fn release_reply() {
        let body = br#"{"version":"1.1.11","notes":"- Fixes","pub_date":"2026-10-28T00:00:00Z",
            "url":"https://cdn.example/zapp.tar.gz","signature":"c2ln","format":"tar.gz"}"#;
        let release = parse_response(200, body, "linux-x86_64").unwrap().unwrap();
        assert_eq!(
            release,
            Release {
                version: "1.1.11".into(),
                notes: Some("- Fixes".into()),
                url: "https://cdn.example/zapp.tar.gz".into(),
                signature: "c2ln".into(),
            }
        );
    }

    #[test]
    fn release_reply_without_notes() {
        let body = br#"{"version":"1.1.11","url":"u","signature":"s"}"#;
        let release = parse_response(200, body, "linux-x86_64").unwrap().unwrap();
        assert_eq!(release.notes, None);
    }

    #[test]
    fn bad_replies() {
        assert!(parse_response(200, b"<html>", "linux-x86_64").is_err());
        let err = parse_response(400, b"", "linux-x86_64").unwrap_err();
        assert_eq!(
            err.to_string(),
            "The update server has no zapp release for linux-x86_64."
        );
        let err = parse_response(500, b"", "linux-x86_64").unwrap_err();
        assert_eq!(err.to_string(), "The update server answered with HTTP 500.");
    }

    #[test]
    fn signature_accepts_signed_archive() {
        verify(TAR_GZ, TAR_GZ_SIG, TEST_PUBLIC_KEY).unwrap();
    }

    #[test]
    fn signature_rejects_tampered_archive() {
        let mut tampered = TAR_GZ.to_vec();
        let last = tampered.len() - 1;
        tampered[last] ^= 1;
        assert!(verify(&tampered, TAR_GZ_SIG, TEST_PUBLIC_KEY).is_err());
    }

    #[test]
    fn signature_rejects_other_key() {
        assert!(verify(TAR_GZ, TAR_GZ_SIG, PUBLIC_KEY).is_err());
    }

    #[test]
    fn signature_rejects_garbage() {
        assert!(verify(TAR_GZ, "not base64!", TEST_PUBLIC_KEY).is_err());
        assert!(verify(TAR_GZ, TAR_GZ_SIG, "").is_err());
    }

    #[test]
    fn release_key_decodes() {
        let text = String::from_utf8(B64.decode(PUBLIC_KEY).unwrap()).unwrap();
        minisign_verify::PublicKey::decode(&text).unwrap();
    }

    #[test]
    fn extracts_binary_from_tar_gz() {
        assert_eq!(extract(TAR_GZ, "zapp").unwrap(), b"zapp test binary\n");
        assert!(extract(TAR_GZ, "zapp.exe").is_err());
    }

    #[test]
    fn extracts_binary_from_zip() {
        assert_eq!(extract(ZIP, "zapp.exe").unwrap(), b"zapp test binary\n");
        assert!(extract(ZIP, "zapp").is_err());
    }

    #[test]
    fn rejects_unknown_archive() {
        let err = extract(b"plain bytes", "zapp").unwrap_err();
        assert_eq!(
            err.to_string(),
            "The download is not an archive zapp can read."
        );
    }

    #[test]
    fn refuses_nix_store() {
        let err = stage_next_to(Path::new("/nix/store/abc-zapp-1.1.10/bin/zapp")).unwrap_err();
        assert!(err.to_string().contains("which Nix manages"));
    }

    #[test]
    fn package_managers() {
        let owner = |path: &str| package_manager(Path::new(path));
        assert_eq!(owner("/nix/store/abc-zapp-1.1.10/bin/zapp"), Some("Nix"));
        assert_eq!(
            owner("/opt/homebrew/Cellar/zapp/1.1.10/bin/zapp"),
            Some("Homebrew")
        );
        assert_eq!(
            owner("/home/u/.local/share/mise/installs/github-zsa-zapp/1.1.10/zapp"),
            Some("mise")
        );
        assert_eq!(owner("/usr/local/bin/zapp"), None);
        assert_eq!(owner("/home/u/.cargo/bin/zapp"), None);
        if cfg!(target_os = "linux") {
            assert_eq!(owner("/usr/bin/zapp"), Some("your package manager"));
        }
    }

    #[cfg(unix)]
    #[test]
    fn refuses_read_only_folder() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        std::fs::set_permissions(dir.path(), std::fs::Permissions::from_mode(0o555)).unwrap();
        let result = stage_next_to(&dir.path().join("zapp"));
        std::fs::set_permissions(dir.path(), std::fs::Permissions::from_mode(0o755)).unwrap();
        assert!(result.unwrap_err().to_string().contains("not writable"));
    }

    #[test]
    fn stages_beside_executable() {
        let dir = tempfile::tempdir().unwrap();
        let staged = stage_next_to(&dir.path().join("zapp")).unwrap();
        assert_eq!(staged.path().parent(), Some(dir.path()));
    }
}
