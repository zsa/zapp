//! Releases on CrabNebula Cloud: zapp itself, and the Voyager 2 radio firmware.
//!
//! The server answers `GET <org>/<app>/<platform>/<current version>` with 204
//! when `current` string-equals its latest release, and with that release
//! otherwise, older or not. Version ordering is done here, with semver.
//! Every download is checked against a minisign signature (Tauri format).

use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use base64::Engine as _;
use base64::engine::general_purpose::STANDARD as B64;
use serde::Deserialize;

use crate::Bundle;

/// ZSA's update endpoints on CrabNebula Cloud.
pub const ORG_ENDPOINT: &str = "https://cdn.crabnebula.app/update/zsa-technology-labs-inc";

/// The radio firmware's app on CrabNebula Cloud.
pub const RADIO_APP: &str = "io-zsa-wireless-voyager";
/// The radio firmware's update platform.
pub const RADIO_PLATFORM: &str = "voyager2";
/// Minisign public key (Tauri format: base64 of the key file) the radio
/// firmware bundles are signed with.
pub const RADIO_PUBLIC_KEY: &str = "dW50cnVzdGVkIGNvbW1lbnQ6IG1pbmlzaWduIHB1YmxpYyBrZXk6IDdEQ0EzNjhDQjUwNjMwNTAKUldSUU1BYTFqRGJLZmJWaW14Z3V0elVyQmJHNkplU05RajgwcDNLN0RtcVNLUzZldmo2NmpMU3kK";
/// What the radio firmware is called in messages.
const RADIO_SUBJECT: &str = "radio firmware release";

/// The version to ask with for the latest release, whatever is installed.
pub const ANY_VERSION: &str = "0.0.0";

const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(300);

/// What the update server offers.
#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct Release {
    pub version: String,
    #[serde(default)]
    pub notes: Option<String>,
    pub url: String,
    pub signature: String,
}

impl Release {
    /// The notes, trimmed, or `None` when there are none.
    pub fn notes(&self) -> Option<&str> {
        self.notes.as_deref().map(str::trim).filter(|notes| !notes.is_empty())
    }
}

/// A radio firmware release, downloaded, verified and parsed.
#[derive(Debug)]
pub struct RadioRelease {
    pub release: Release,
    pub bundle: Bundle,
}

/// An HTTP client with the update server's timeouts.
pub fn client(user_agent: &str) -> Result<reqwest::blocking::Client> {
    reqwest::blocking::Client::builder()
        .user_agent(user_agent)
        .connect_timeout(CONNECT_TIMEOUT)
        .timeout(REQUEST_TIMEOUT)
        .build()
        .context("could not set up the HTTP client")
}

/// The URL that asks for `app`'s release on `platform`.
pub fn release_url(app: &str, platform: &str, current: &str) -> String {
    format!("{ORG_ENDPOINT}/{app}/{platform}/{current}")
}

/// The release the server offers against `current`, or `None` for 204.
/// `subject` names the release in messages ("zapp release").
pub fn check(
    client: &reqwest::blocking::Client,
    app: &str,
    platform: &str,
    current: &str,
    subject: &str,
) -> Result<Option<Release>> {
    let response = client
        .get(release_url(app, platform, current))
        .send()
        .map_err(network_error)?;
    let status = response.status().as_u16();
    let body = response.bytes().map_err(network_error)?;
    parse_response(status, &body, subject, platform)
}

/// Downloads a release and checks it against its signature and `public_key`.
pub fn download(
    client: &reqwest::blocking::Client,
    release: &Release,
    public_key: &str,
) -> Result<Vec<u8>> {
    let response = client.get(&release.url).send().map_err(network_error)?;
    let status = response.status();
    if !status.is_success() {
        bail!("The download failed (HTTP {}).", status.as_u16());
    }
    let data = response.bytes().map_err(network_error)?.to_vec();
    verify(&data, &release.signature, public_key)
        .map_err(|_| anyhow!("The download's signature does not match. Nothing was changed."))?;
    Ok(data)
}

/// The radio firmware release newer than `current`, or the latest one when
/// `current` is `None`. `None` when there is nothing newer.
pub fn check_radio(
    client: &reqwest::blocking::Client,
    current: Option<&str>,
) -> Result<Option<Release>> {
    let offered = check(
        client,
        RADIO_APP,
        RADIO_PLATFORM,
        current.unwrap_or(ANY_VERSION),
        RADIO_SUBJECT,
    )?;
    newer_than(offered, current)
}

/// Downloads, verifies and parses a radio firmware release.
pub fn download_radio(
    client: &reqwest::blocking::Client,
    release: Release,
) -> Result<RadioRelease> {
    let data = download(client, &release, RADIO_PUBLIC_KEY)?;
    let bundle = crate::parse_bundle(&data, &format!("Radio firmware {}", release.version))?;
    Ok(RadioRelease { release, bundle })
}

/// The latest radio firmware release, downloaded, verified and parsed.
pub fn fetch_latest_radio(user_agent: &str) -> Result<RadioRelease> {
    let client = client(user_agent)?;
    let release = check_radio(&client, None)?
        .ok_or_else(|| anyhow!("The update server has no {RADIO_SUBJECT}."))?;
    download_radio(&client, release)
}

/// `offered` when it is later than `current`, or whenever `current` is `None`.
fn newer_than(offered: Option<Release>, current: Option<&str>) -> Result<Option<Release>> {
    match (offered, current) {
        (Some(release), Some(current)) if !is_newer(&release.version, current)? => Ok(None),
        (offered, _) => Ok(offered),
    }
}

/// The release in a reply, or `None` for 204 (nothing newer).
pub fn parse_response(
    status: u16,
    body: &[u8],
    subject: &str,
    platform: &str,
) -> Result<Option<Release>> {
    match status {
        200 => serde_json::from_slice(body)
            .map(Some)
            .map_err(|_| anyhow!("The update server sent an unreadable reply.")),
        204 => Ok(None),
        400 | 404 => bail!("The update server has no {subject} for {platform}."),
        other => bail!("The update server answered with HTTP {other}."),
    }
}

/// Whether `offered` is a later version than `current`.
pub fn is_newer(offered: &str, current: &str) -> Result<bool> {
    let parse = |v: &str| {
        semver::Version::parse(v.trim().trim_start_matches('v'))
            .map_err(|_| anyhow!("The update server sent an unreadable version: {v}."))
    };
    Ok(parse(offered)? > parse(current)?)
}

/// Checks `data` against a Tauri-format minisign signature and public key.
pub fn verify(data: &[u8], signature_b64: &str, public_key_b64: &str) -> Result<()> {
    let key = public_key(public_key_b64)?;
    let signature = minisign_verify::Signature::decode(&decode(signature_b64)?)?;
    key.verify(data, &signature, false)?;
    Ok(())
}

/// A Tauri-format minisign public key.
pub fn public_key(b64: &str) -> Result<minisign_verify::PublicKey> {
    Ok(minisign_verify::PublicKey::decode(&decode(b64)?)?)
}

fn decode(b64: &str) -> Result<String> {
    Ok(String::from_utf8(B64.decode(b64.trim())?)?)
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

    /// Throwaway key that signed the fixture; not a release key.
    const TEST_PUBLIC_KEY: &str = "dW50cnVzdGVkIGNvbW1lbnQ6IG1pbmlzaWduIHB1YmxpYyBrZXk6IEE4MjcyMDI5REIxQkJEM0YKUldRL3ZSdmJLU0FucURwMlg1Q0NHY0VsZTFSYW5kVThZdUQ4TUNvOEhXdFlreDNUa2dNaW1hSHUK";
    const TAR_GZ: &[u8] = include_bytes!("../../zapp/testdata/upgrade/zapp-test.tar.gz");
    const TAR_GZ_SIG: &str = include_str!("../../zapp/testdata/upgrade/zapp-test.tar.gz.sig");

    fn release(version: &str) -> Release {
        Release {
            version: version.into(),
            notes: None,
            url: "u".into(),
            signature: "s".into(),
        }
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
        assert_eq!(
            parse_response(204, b"", "zapp release", "linux-x86_64").unwrap(),
            None
        );
    }

    #[test]
    fn release_reply() {
        let body = br#"{"version":"1.1.11","notes":"- Fixes","pub_date":"2026-10-28T00:00:00Z",
            "url":"https://cdn.example/zapp.tar.gz","signature":"c2ln","format":"tar.gz"}"#;
        let release = parse_response(200, body, "zapp release", "linux-x86_64")
            .unwrap()
            .unwrap();
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
        let release = parse_response(200, body, "zapp release", "linux-x86_64")
            .unwrap()
            .unwrap();
        assert_eq!(release.notes, None);
        assert_eq!(release.notes(), None);
    }

    #[test]
    fn blank_notes_read_as_none() {
        let mut release = release("0.1.20");
        release.notes = Some("  \n".into());
        assert_eq!(release.notes(), None);
        release.notes = Some("\n- Fixes\n".into());
        assert_eq!(release.notes(), Some("- Fixes"));
    }

    #[test]
    fn bad_replies() {
        assert!(parse_response(200, b"<html>", "zapp release", "linux-x86_64").is_err());
        let err = parse_response(400, b"", "zapp release", "linux-x86_64").unwrap_err();
        assert_eq!(
            err.to_string(),
            "The update server has no zapp release for linux-x86_64."
        );
        let err = parse_response(404, b"", RADIO_SUBJECT, RADIO_PLATFORM).unwrap_err();
        assert_eq!(
            err.to_string(),
            "The update server has no radio firmware release for voyager2."
        );
        let err = parse_response(500, b"", "zapp release", "linux-x86_64").unwrap_err();
        assert_eq!(err.to_string(), "The update server answered with HTTP 500.");
    }

    #[test]
    fn radio_feed_url() {
        assert_eq!(
            release_url(RADIO_APP, RADIO_PLATFORM, ANY_VERSION),
            "https://cdn.crabnebula.app/update/zsa-technology-labs-inc/io-zsa-wireless-voyager/voyager2/0.0.0"
        );
    }

    #[test]
    fn latest_takes_any_offer() {
        // The server offers its latest even when it is older than the version asked with.
        let offered = newer_than(Some(release("0.0.1")), None).unwrap();
        assert_eq!(offered, Some(release("0.0.1")));
        assert_eq!(newer_than(None, None).unwrap(), None);
    }

    #[test]
    fn current_version_filters_older_offers() {
        let newer = newer_than(Some(release("0.1.20")), Some("0.1.19")).unwrap();
        assert_eq!(newer, Some(release("0.1.20")));
        assert_eq!(newer_than(Some(release("0.1.19")), Some("0.1.20")).unwrap(), None);
        assert_eq!(newer_than(Some(release("0.1.20")), Some("0.1.20")).unwrap(), None);
        assert_eq!(newer_than(None, Some("0.1.20")).unwrap(), None);
        assert!(newer_than(Some(release("x")), Some("0.1.20")).is_err());
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
        assert!(verify(TAR_GZ, TAR_GZ_SIG, RADIO_PUBLIC_KEY).is_err());
    }

    #[test]
    fn signature_rejects_garbage() {
        assert!(verify(TAR_GZ, "not base64!", TEST_PUBLIC_KEY).is_err());
        assert!(verify(TAR_GZ, TAR_GZ_SIG, "").is_err());
    }

    /// Asks the live server. Run with `--ignored`.
    #[test]
    #[ignore]
    fn live_radio_feed() {
        match fetch_latest_radio("zapp-test") {
            Ok(radio) => println!("radio firmware {}", radio.release.version),
            Err(err) => println!("{err:#}"),
        }
    }

    #[test]
    fn radio_key_decodes() {
        public_key(RADIO_PUBLIC_KEY).unwrap();
    }
}
