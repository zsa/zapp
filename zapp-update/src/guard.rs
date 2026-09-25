//! The downgrade guard: refusing to put an older image on a half than the one
//! it carries.
//!
//! The cable path calls it before every upload unless `force` is set. The
//! Bluetooth path has never had it and still does not; it takes the same
//! image-state listing, so it could.

use zapp_smp::{image, Half, ImageSlot};

/// `major.minor.patch` out of a version, ignoring any `-suffix` or `+build`.
///
/// A board reports what MCUboot formatted out of the image header; a bundle
/// reports whatever it was packed with, which is where the `-suffix` comes
/// from. Anything unparseable returns None, which is read as "cannot tell" and
/// lets the write through: refusing on a version string nobody can read would
/// be a worse failure than the one this guards against.
pub fn ordering_of(version: &str) -> Option<(u32, u32, u32)> {
    let mut parts = version.split(['-', '+']).next()?.split('.');
    let major = parts.next()?.parse().ok()?;
    let minor = parts.next()?.parse().ok()?;
    let patch = parts.next().unwrap_or("0").parse().ok()?;
    Some((major, minor, patch))
}

/// Says so if `image` is older than what the half's slot 0 carries.
///
/// Two halves on different builds is a keyboard whose halves cannot talk over
/// the wire between them, and the way boards get there is a bundle built from
/// a directory that was rebuilt afterwards — the versions are right there in
/// the output, going backwards, and nothing was reading them.
pub fn going_backwards(slots: &[ImageSlot], image: &[u8], half: Half) -> Option<String> {
    let installed = ordering_of(&slots.iter().find(|slot| slot.slot == 0)?.version)?;
    let offered_text = image::inspect(image)?.version;
    let offered = ordering_of(&offered_text)?;
    if offered >= installed {
        return None;
    }
    let (a, b, c) = installed;
    Some(format!(
        "The bundle's {half} image is v{offered_text}, older than the v{a}.{b}.{c} that half \
         carries."
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bundle_named_after_its_release_is_still_read_as_a_version() {
        // `0.1.9-wired` is what a real bundle is packed as, and it names the
        // same build as the `0.1.9` a board reports.
        assert_eq!(ordering_of("0.1.9-wired"), Some((0, 1, 9)));
        assert_eq!(ordering_of("0.1.9"), Some((0, 1, 9)));
        assert_eq!(ordering_of("0.1.9+42"), Some((0, 1, 9)));
        assert_eq!(ordering_of(""), None);
    }
}
