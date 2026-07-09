use serde::Deserialize;
use zapp_core::device::ids::Keyboard;
use zapp_core::firmware::{self, Firmware};

#[derive(Debug, thiserror::Error)]
pub enum OryxError {
    #[error("Not a valid Oryx URL")]
    InvalidUrl,
    #[error("Network error: {0}")]
    Network(#[from] reqwest::Error),
    #[error("Firmware error: {0}")]
    Firmware(#[from] zapp_core::ZappError),
}

#[derive(Deserialize)]
struct LatestResponse {
    latest: String,
}

/// Oryx's API host, serving firmware builds and the GraphQL endpoint.
const ORYX_API_URL: &str = "https://oryx.zsa.io";

/// The layout editor front end, where the URLs users copy come from.
const CONFIGURE_URL: &str = "https://configure.zsa.io/";

const REVISION_NOTE_QUERY: &str = "\
    query ($hashId: String!, $revisionId: String!, $geometry: String) {\
      layout(hashId: $hashId, revisionId: $revisionId, geometry: $geometry) {\
        revision { title }\
      }\
    }";

#[derive(Deserialize)]
struct GqlResponse {
    data: Option<GqlData>,
}

#[derive(Deserialize)]
struct GqlData {
    layout: Option<GqlLayout>,
}

#[derive(Deserialize)]
struct GqlLayout {
    revision: Option<GqlRevision>,
}

#[derive(Deserialize)]
struct GqlRevision {
    title: Option<String>,
}

/// Oryx geometry slug for a keyboard. The Ergodox EZ shipped with two different
/// MCUs, and Oryx models them as separate geometries.
pub fn geometry_for(keyboard: Keyboard, pid: u16) -> &'static str {
    match keyboard {
        Keyboard::Voyager => "voyager",
        Keyboard::Moonlander => "moonlander",
        Keyboard::PlanckEz => "planck-ez",
        Keyboard::ErgodoxEz => match pid {
            0x2010 | 0x2020 | 0x2030 => "ergodox-ez-st",
            _ => "ergodox-ez",
        },
    }
}

/// Fetch the commit message a user attached to a layout revision.
///
/// Returns `Ok(None)` when the revision exists but carries no message, or when
/// Oryx cannot resolve the layout.
pub fn fetch_revision_note(
    layout_id: &str,
    revision_id: &str,
    geometry: &str,
) -> Result<Option<String>, OryxError> {
    let body = serde_json::json!({
        "query": REVISION_NOTE_QUERY,
        "variables": {
            "hashId": layout_id,
            "revisionId": revision_id,
            "geometry": geometry,
        },
    });

    let resp: GqlResponse = reqwest::blocking::Client::new()
        .post(format!("{ORYX_API_URL}/graphql"))
        .json(&body)
        .send()?
        .error_for_status()?
        .json()?;

    let Some(title) = resp
        .data
        .and_then(|d| d.layout)
        .and_then(|l| l.revision)
        .and_then(|r| r.title)
    else {
        return Ok(None);
    };

    let title = title.trim();

    Ok((!title.is_empty()).then(|| title.to_string()))
}

/// Whether a string looks like an Oryx layout URL rather than a local path.
pub fn is_oryx_url(s: &str) -> bool {
    s.starts_with(CONFIGURE_URL)
}

/// Parse an Oryx URL into (geometry, layout_id, Option<revision_id>).
///
/// Accepted forms:
///   /voyager/layouts/:layoutId
///   /voyager/layouts/:layoutId/latest
///   /voyager/layouts/:layoutId/latest/0
///   /voyager/layouts/:layoutId/:revisionId
pub fn parse_url(url: &str) -> Result<(&str, &str, Option<&str>), OryxError> {
    let path = url
        .strip_prefix(CONFIGURE_URL)
        .ok_or(OryxError::InvalidUrl)?;

    let segments: Vec<&str> = path.trim_end_matches('/').split('/').collect();

    if segments.len() < 3 || segments[1] != "layouts" {
        return Err(OryxError::InvalidUrl);
    }

    let geometry = segments[0];
    let layout_id = segments[2];

    let revision_id = match segments.get(3) {
        None => None,
        Some(&"latest") => None,
        Some(rev) => Some(*rev),
    };

    Ok((geometry, layout_id, revision_id))
}

/// Resolve a layout to a specific revision ID.
/// If `revision_id` is already provided, returns it directly.
/// Otherwise fetches the latest revision from Oryx.
pub fn resolve_revision(
    geometry: &str,
    layout_id: &str,
    revision_id: Option<&str>,
) -> Result<String, OryxError> {
    if let Some(rev) = revision_id {
        return Ok(rev.to_string());
    }

    // Every geometry has a stock layout, and they all share the id "default",
    // so the geometry is what disambiguates them.
    let url = if layout_id == "default" {
        format!("{ORYX_API_URL}/firmware/latest/{geometry}/default")
    } else {
        format!("{ORYX_API_URL}/firmware/latest/{layout_id}")
    };

    let resp: LatestResponse = reqwest::blocking::get(&url)?.error_for_status()?.json()?;

    Ok(resp.latest)
}

/// Fetch the latest revision ID for a user layout.
pub fn fetch_latest_revision(layout_id: &str) -> Result<String, OryxError> {
    resolve_revision("", layout_id, None)
}

/// Which build of a revision's firmware to fetch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareVariant {
    /// The build for the layout's own model.
    Standard,
    /// The build for the board's alternate MCU (e.g. Moonlander rev B).
    Alternate,
    /// Both halves collated into a single image (Moonlander).
    Collated,
}

impl FirmwareVariant {
    fn query(self) -> &'static str {
        match self {
            Self::Standard => "",
            Self::Alternate => "?alt=true",
            Self::Collated => "?collate=true",
        }
    }
}

/// Download firmware for a given revision.
pub fn download_firmware(
    revision_id: &str,
    variant: FirmwareVariant,
) -> Result<Firmware, OryxError> {
    let url = format!("{ORYX_API_URL}/firmware/{revision_id}{}", variant.query());

    let fw_bytes = reqwest::blocking::get(&url)?.error_for_status()?.bytes()?;

    Ok(firmware::load_firmware_from_bytes(&fw_bytes)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_url_bare_layout() {
        let (geo, layout, rev) =
            parse_url("https://configure.zsa.io/voyager/layouts/abcde").unwrap();
        assert_eq!(geo, "voyager");
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_url_latest() {
        let (geo, layout, rev) =
            parse_url("https://configure.zsa.io/voyager/layouts/abcde/latest").unwrap();
        assert_eq!(geo, "voyager");
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_url_latest_with_zero() {
        let (geo, layout, rev) =
            parse_url("https://configure.zsa.io/voyager/layouts/abcde/latest/0").unwrap();
        assert_eq!(geo, "voyager");
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_url_specific_revision() {
        let (geo, layout, rev) =
            parse_url("https://configure.zsa.io/moonlander/layouts/AbCdE/abc123").unwrap();
        assert_eq!(geo, "moonlander");
        assert_eq!(layout, "AbCdE");
        assert_eq!(rev, Some("abc123"));
    }

    #[test]
    fn test_parse_url_trailing_slash() {
        let (geo, layout, rev) =
            parse_url("https://configure.zsa.io/voyager/layouts/abcde/").unwrap();
        assert_eq!(geo, "voyager");
        assert_eq!(layout, "abcde");
        assert_eq!(rev, None);
    }

    #[test]
    fn test_parse_url_invalid_prefix() {
        assert!(parse_url("https://example.com/voyager/layouts/abcde").is_err());
    }

    #[test]
    fn test_parse_url_missing_layouts_segment() {
        assert!(parse_url("https://configure.zsa.io/voyager/abcde").is_err());
    }

    #[test]
    fn test_geometry_for() {
        assert_eq!(geometry_for(Keyboard::Voyager, 0x1977), "voyager");
        assert_eq!(geometry_for(Keyboard::Moonlander, 0x1972), "moonlander");
        assert_eq!(geometry_for(Keyboard::PlanckEz, 0x6060), "planck-ez");
    }

    #[test]
    fn test_geometry_for_ergodox_splits_on_mcu() {
        // STM32 Ergodoxes are their own geometry in Oryx.
        assert_eq!(geometry_for(Keyboard::ErgodoxEz, 0x2010), "ergodox-ez-st");
        assert_eq!(geometry_for(Keyboard::ErgodoxEz, 0x2020), "ergodox-ez-st");
        assert_eq!(geometry_for(Keyboard::ErgodoxEz, 0x2030), "ergodox-ez-st");
        // AVR originals.
        assert_eq!(geometry_for(Keyboard::ErgodoxEz, 0x1307), "ergodox-ez");
        assert_eq!(geometry_for(Keyboard::ErgodoxEz, 0x4974), "ergodox-ez");
    }
}
