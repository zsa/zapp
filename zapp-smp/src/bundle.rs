//! The `.zsab` pair bundle, and the decisions an update makes from it.
//!
//! One file carrying both halves' MCUboot-signed images, so a release is a
//! single artefact rather than two binaries that have to be kept together. The
//! layout is fixed by `tools/zsa_bundle.py` in the firmware tree, which is what
//! writes them; everything is little endian.
//!
//! ```text
//! 0x00  4   magic "ZSAB"
//! 0x04  2   format version
//! 0x06  2   flags (reserved, 0)
//! 0x08  4   left image offset
//! 0x0C  4   left image size
//! 0x10  4   right image offset
//! 0x14  4   right image size
//! 0x18  4   key fingerprint
//! 0x1C  32  build version, NUL-padded ASCII
//! 0x3C  ..  the left image, then the right
//! ```
//!
//! Nothing here touches a radio: parsing a bundle, deciding whether a half
//! needs writing and deciding whether the pack has the charge to see it through
//! are all settled from bytes, and all tested that way.

use crate::{image, Half, ImageSlot};

pub const MAGIC: [u8; 4] = *b"ZSAB";
pub const FORMAT_VERSION: u16 = 1;
pub const HEADER_LEN: usize = 0x3C;

/// The eight bytes the build stamps in front of the key fingerprint it embeds
/// in an image, from `shared/cmake/zsa_esb_key.cmake`.
///
/// Scanned for rather than read at a fixed offset: the section it lands in is
/// covered by the signature but its address is the linker's business.
const FINGERPRINT_MARKER: [u8; 8] = *b"ZSAKEYFP";

/// The charge below which an update is refused.
///
/// The same figure as `ZSA_DFU_MIN_SOC_PERCENT` in the firmware, which gates
/// the very same upload from the other end — the SMP service is standard, so
/// the board cannot rely on a host having checked.
pub const MIN_SOC_PERCENT: u8 = 30;

#[derive(Debug, thiserror::Error)]
pub enum BundleError {
    #[error("a bundle header is {HEADER_LEN} bytes and this file is {0}")]
    TooShort(usize),

    #[error("not a .zsab bundle: it starts {0}, not \"ZSAB\"")]
    BadMagic(String),

    #[error("this bundle is format version {0}, and this build reads version {FORMAT_VERSION}")]
    BadFormat(u16),

    #[error(
        "the bundle puts its {half} image at {offset} + {size} bytes, past the end of a \
         {len}-byte file — it is truncated"
    )]
    Overrun {
        half: Half,
        offset: u32,
        size: u32,
        len: usize,
    },

    #[error("the bundle's {0} image is empty")]
    EmptyImage(Half),

    #[error(
        "the bundle says it was built with key {header:08x} but its {half} image carries \
         {image:08x} — the two halves would not be able to talk"
    )]
    KeyMismatch { half: Half, header: u32, image: u32 },
}

/// Where one image sits in the file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Span {
    pub offset: u32,
    pub size: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BundleHeader {
    pub format_version: u16,
    pub flags: u16,
    pub left: Span,
    pub right: Span,
    /// The first four bytes of SHA-256 over the ESB key both images were built
    /// against. Zero is "not stamped" — see `key_fingerprint`.
    pub key_fingerprint: u32,
    pub build_version: String,
}

impl BundleHeader {
    pub fn span(&self, half: Half) -> Span {
        match half {
            Half::Left => self.left,
            Half::Right => self.right,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bundle {
    pub header: BundleHeader,
    left: Vec<u8>,
    right: Vec<u8>,
}

impl Bundle {
    /// One half's signed image, exactly the bytes `SmpClient::upload` takes.
    pub fn image(&self, half: Half) -> &[u8] {
        match half {
            Half::Left => &self.left,
            Half::Right => &self.right,
        }
    }

    /// The hash a board will name this half's image by, once it is on it.
    pub fn hash(&self, half: Half) -> Option<Vec<u8>> {
        image::signed_hash(self.image(half))
    }
}

/// Reads a `.zsab` file.
pub fn parse(blob: &[u8]) -> Result<Bundle, BundleError> {
    let header = parse_header(blob)?;

    let cut = |half: Half| -> Result<Vec<u8>, BundleError> {
        let Span { offset, size } = header.span(half);
        let end = (offset as usize).checked_add(size as usize);
        let image = end
            .filter(|end| *end <= blob.len())
            .map(|end| blob[offset as usize..end].to_vec())
            .ok_or(BundleError::Overrun {
                half,
                offset,
                size,
                len: blob.len(),
            })?;
        if image.is_empty() {
            return Err(BundleError::EmptyImage(half));
        }
        Ok(image)
    };

    let bundle = Bundle {
        left: cut(Half::Left)?,
        right: cut(Half::Right)?,
        header,
    };
    check_key(&bundle)?;
    Ok(bundle)
}

/// Just the header, for a caller that only wants to know what a file is.
pub fn parse_header(blob: &[u8]) -> Result<BundleHeader, BundleError> {
    let header: &[u8; HEADER_LEN] = blob
        .get(..HEADER_LEN)
        .and_then(|head| head.try_into().ok())
        .ok_or(BundleError::TooShort(blob.len()))?;

    let word = |at: usize| u32::from_le_bytes(header[at..at + 4].try_into().unwrap());
    let half = |at: usize| u16::from_le_bytes(header[at..at + 2].try_into().unwrap());

    if header[..4] != MAGIC {
        return Err(BundleError::BadMagic(describe(&header[..4])));
    }
    let format_version = half(4);
    if format_version != FORMAT_VERSION {
        return Err(BundleError::BadFormat(format_version));
    }

    Ok(BundleHeader {
        format_version,
        flags: half(6),
        left: Span {
            offset: word(8),
            size: word(12),
        },
        right: Span {
            offset: word(16),
            size: word(20),
        },
        key_fingerprint: word(24),
        // NUL-padded, and anything that is not printable ASCII is dropped
        // rather than failing the file: the version is for a human to read and
        // is never load-bearing.
        build_version: header[28..HEADER_LEN]
            .iter()
            .take_while(|byte| **byte != 0)
            .map(|byte| *byte as char)
            .filter(|c| c.is_ascii_graphic() || *c == ' ')
            .collect(),
    })
}

/// The fingerprint an image carries, if it carries one.
pub fn key_fingerprint(image: &[u8]) -> Option<u32> {
    let at = image
        .windows(FINGERPRINT_MARKER.len())
        .position(|window| window == FINGERPRINT_MARKER)?
        + FINGERPRINT_MARKER.len();
    let bytes: [u8; 4] = image.get(at..at + 4)?.try_into().ok()?;
    Some(u32::from_le_bytes(bytes))
}

/// Cross-checks the key both images were built against.
///
/// The halves talk over an AES-128 ESB link, so a pair built against different
/// keys is a keyboard whose halves cannot hear each other — worth refusing
/// before anything is written rather than after.
///
/// A header fingerprint of zero means "unknown, do not check", and is not a
/// reason to refuse anything: the current firmware builds do not stamp one,
/// because `shared/cmake/zsa_esb_key.cmake` is not wired into either app, so
/// every bundle built today carries zero. Production bundles are meant to carry
/// it, which is why a non-zero one is checked properly.
fn check_key(bundle: &Bundle) -> Result<(), BundleError> {
    let header = bundle.header.key_fingerprint;
    if header == 0 {
        return Ok(());
    }
    for half in [Half::Left, Half::Right] {
        // An image without the marker is one built before the stamp existed.
        // Nothing to disagree with, so nothing to refuse.
        if let Some(image) = key_fingerprint(bundle.image(half)) {
            if image != header {
                return Err(BundleError::KeyMismatch {
                    half,
                    header,
                    image,
                });
            }
        }
    }
    Ok(())
}

/// What four bytes that should have been "ZSAB" were.
fn describe(bytes: &[u8]) -> String {
    let printable: String = bytes
        .iter()
        .map(|byte| {
            if byte.is_ascii_graphic() {
                *byte as char
            } else {
                '.'
            }
        })
        .collect();
    format!("{printable:?} ({})", image::hex(bytes))
}

/// What an update has to do to one half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Already running this exact image, and keeping it. Nothing to do.
    Installed,
    /// Running it already, but on a test boot that has not been confirmed —
    /// it would revert on the next restart, so it only needs keeping.
    Confirm,
    /// Not running it. Upload, mark, restart.
    Upload,
}

/// What one half needs, given the image meant for it and what it reports.
///
/// The comparison is against the *active* slot only: an image sitting in the
/// spare slot is staged, not installed, and re-uploading it is both cheap and
/// the only way to be sure of what is there.
pub fn step_for(image: &[u8], slots: &[ImageSlot]) -> Step {
    let Some(hash) = image::signed_hash(image) else {
        // Not a signed image, or one whose trailer carries no hash: nothing to
        // compare, so let the board be the judge of it.
        return Step::Upload;
    };
    match slots.iter().find(|slot| slot.active && slot.hash == hash) {
        Some(slot) if slot.confirmed => Step::Installed,
        Some(_) => Step::Confirm,
        None => Step::Upload,
    }
}

/// Whether the pack has the charge to see an update through.
///
/// Fails open on purpose. An unknown level must never make a keyboard
/// unflashable, which is exactly the state a dead fuel gauge would otherwise
/// leave it in, and the firmware's own gate in `smp_dfu_gate.c` fails open the
/// same way. Inverting either direction would turn a broken gauge into a brick.
pub fn battery_allows(level: Option<u8>, charging: bool) -> bool {
    charging || level.is_none_or(|soc| soc >= MIN_SOC_PERCENT)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A signed image with a given hash TLV and, optionally, a key stamp.
    fn image(payload: u8, hash: &[u8], fingerprint: Option<u32>) -> Vec<u8> {
        const MCUBOOT_MAGIC: u32 = 0x96f3_b83d;
        let mut body = vec![payload; 96];
        if let Some(fingerprint) = fingerprint {
            body[16..24].copy_from_slice(&FINGERPRINT_MARKER);
            body[24..28].copy_from_slice(&fingerprint.to_le_bytes());
        }

        let mut out = Vec::new();
        out.extend_from_slice(&MCUBOOT_MAGIC.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // load address
        out.extend_from_slice(&32u16.to_le_bytes()); // header size
        out.extend_from_slice(&0u16.to_le_bytes()); // protected tlvs
        out.extend_from_slice(&(body.len() as u32).to_le_bytes());
        out.extend_from_slice(&[0u8; 16]);
        out.extend_from_slice(&body);

        out.extend_from_slice(&0x6907u16.to_le_bytes());
        out.extend_from_slice(&((4 + 4 + hash.len()) as u16).to_le_bytes());
        out.extend_from_slice(&0x0012u16.to_le_bytes()); // SHA-512
        out.extend_from_slice(&(hash.len() as u16).to_le_bytes());
        out.extend_from_slice(hash);
        out
    }

    fn left_image() -> Vec<u8> {
        image(0x11, &[0xa1; 64], None)
    }

    fn right_image() -> Vec<u8> {
        image(0x22, &[0xb2; 64], None)
    }

    fn bundle_of(left: &[u8], right: &[u8], fingerprint: u32, version: &str) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&MAGIC);
        out.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&(HEADER_LEN as u32).to_le_bytes());
        out.extend_from_slice(&(left.len() as u32).to_le_bytes());
        out.extend_from_slice(&((HEADER_LEN + left.len()) as u32).to_le_bytes());
        out.extend_from_slice(&(right.len() as u32).to_le_bytes());
        out.extend_from_slice(&fingerprint.to_le_bytes());
        let mut name = [0u8; 32];
        name[..version.len()].copy_from_slice(version.as_bytes());
        out.extend_from_slice(&name);
        assert_eq!(out.len(), HEADER_LEN);
        out.extend_from_slice(left);
        out.extend_from_slice(right);
        out
    }

    fn good() -> Vec<u8> {
        bundle_of(&left_image(), &right_image(), 0, "1.2.3-test")
    }

    #[test]
    fn a_bundle_reads_back_as_the_two_images_that_went_into_it() {
        let bundle = parse(&good()).expect("a bundle");
        assert_eq!(bundle.header.build_version, "1.2.3-test");
        assert_eq!(bundle.header.format_version, 1);
        assert_eq!(bundle.header.flags, 0);
        assert_eq!(bundle.header.left.offset, HEADER_LEN as u32);
        assert_eq!(bundle.image(Half::Left), left_image());
        assert_eq!(bundle.image(Half::Right), right_image());
        // And each half's hash is the one its trailer carries, which is what a
        // board will call it.
        assert_eq!(bundle.hash(Half::Left), Some(vec![0xa1; 64]));
        assert_eq!(bundle.hash(Half::Right), Some(vec![0xb2; 64]));
    }

    #[test]
    fn a_build_version_stops_at_its_padding() {
        let bundle = parse(&bundle_of(&left_image(), &right_image(), 0, "0.1.8")).unwrap();
        assert_eq!(bundle.header.build_version, "0.1.8");
        // An empty one is a bundle, not an error.
        let bundle = parse(&bundle_of(&left_image(), &right_image(), 0, "")).unwrap();
        assert_eq!(bundle.header.build_version, "");
    }

    #[test]
    fn anything_that_is_not_a_bundle_says_what_it_is_instead() {
        // A signed image handed over by mistake, which is the likeliest wrong
        // file: it has to be refused by name rather than read as a header.
        let err = parse(&left_image()).unwrap_err();
        assert!(matches!(err, BundleError::BadMagic(_)), "{err:?}");
        assert!(err.to_string().contains("ZSAB"), "{err}");

        let mut zeros = good();
        zeros[..4].fill(0);
        assert!(matches!(parse(&zeros), Err(BundleError::BadMagic(_))));
    }

    #[test]
    fn a_file_too_short_to_hold_a_header_is_not_read_as_one() {
        let whole = good();
        for len in [0usize, 1, 4, HEADER_LEN - 1] {
            let err = parse(&whole[..len]).unwrap_err();
            assert!(matches!(err, BundleError::TooShort(short) if short == len), "{err:?}");
        }
        // One byte more and the header itself is readable, so the failure moves
        // on to the images it describes.
        assert!(matches!(
            parse(&whole[..HEADER_LEN]),
            Err(BundleError::Overrun { .. })
        ));
    }

    #[test]
    fn a_bundle_from_a_newer_format_is_refused_rather_than_guessed_at() {
        let mut future = good();
        future[4..6].copy_from_slice(&2u16.to_le_bytes());
        let err = parse(&future).unwrap_err();
        assert!(matches!(err, BundleError::BadFormat(2)), "{err:?}");
    }

    #[test]
    fn a_truncated_bundle_is_caught_before_a_short_image_is_uploaded() {
        let whole = good();
        let err = parse(&whole[..whole.len() - 1]).unwrap_err();
        assert!(
            matches!(err, BundleError::Overrun { half: Half::Right, .. }),
            "{err:?}"
        );
        assert!(err.to_string().contains("truncated"), "{err}");

        // And a header whose offsets point past the file, whatever its length:
        // the arithmetic must not wrap into an in-range slice.
        let mut wild = good();
        wild[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(matches!(
            parse(&wild),
            Err(BundleError::Overrun { half: Half::Left, .. })
        ));
        let mut huge = good();
        huge[12..16].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(matches!(
            parse(&huge),
            Err(BundleError::Overrun { half: Half::Left, .. })
        ));
    }

    #[test]
    fn a_bundle_carrying_no_image_at_all_is_refused() {
        let mut empty = good();
        empty[12..16].copy_from_slice(&0u32.to_le_bytes());
        assert!(matches!(
            parse(&empty),
            Err(BundleError::EmptyImage(Half::Left))
        ));
    }

    #[test]
    fn a_stamped_bundle_is_checked_against_the_key_its_images_carry() {
        let left = image(0x11, &[0xa1; 64], Some(0xdead_beef));
        let right = image(0x22, &[0xb2; 64], Some(0xdead_beef));
        let bundle = parse(&bundle_of(&left, &right, 0xdead_beef, "1.0")).expect("a bundle");
        assert_eq!(bundle.header.key_fingerprint, 0xdead_beef);
        assert_eq!(key_fingerprint(bundle.image(Half::Left)), Some(0xdead_beef));
    }

    #[test]
    fn a_bundle_whose_images_were_built_against_another_key_is_refused() {
        let left = image(0x11, &[0xa1; 64], Some(0xdead_beef));
        let right = image(0x22, &[0xb2; 64], Some(0x0bad_0bad));
        let err = parse(&bundle_of(&left, &right, 0xdead_beef, "1.0")).unwrap_err();
        assert!(
            matches!(err, BundleError::KeyMismatch { half: Half::Right, .. }),
            "{err:?}"
        );
        // And a header that disagrees with both images is caught on the first.
        let err = parse(&bundle_of(&left, &right, 0x1111_2222, "1.0")).unwrap_err();
        assert!(
            matches!(err, BundleError::KeyMismatch { half: Half::Left, .. }),
            "{err:?}"
        );
    }

    #[test]
    fn a_fingerprint_of_zero_is_unknown_rather_than_wrong() {
        // Every bundle built today carries zero, because the build does not
        // stamp the images either. Refusing those would make the whole flow
        // unusable, so zero turns the check off — including when the images do
        // carry a stamp of their own.
        assert!(parse(&bundle_of(&left_image(), &right_image(), 0, "1.0")).is_ok());
        let left = image(0x11, &[0xa1; 64], Some(0xdead_beef));
        let right = image(0x22, &[0xb2; 64], Some(0xdead_beef));
        assert!(parse(&bundle_of(&left, &right, 0, "1.0")).is_ok());
    }

    #[test]
    fn an_unstamped_image_in_a_stamped_bundle_is_nothing_to_disagree_with() {
        let left = image(0x11, &[0xa1; 64], Some(0xdead_beef));
        assert!(parse(&bundle_of(&left, &right_image(), 0xdead_beef, "1.0")).is_ok());
    }

    fn slot(hash: &[u8], active: bool, confirmed: bool) -> ImageSlot {
        ImageSlot {
            slot: if active { 0 } else { 1 },
            hash: hash.to_vec(),
            active,
            confirmed,
            bootable: true,
            ..ImageSlot::default()
        }
    }

    #[test]
    fn a_half_already_running_the_image_is_left_alone() {
        let installed = [slot(&[0xa1; 64], true, true)];
        assert_eq!(step_for(&left_image(), &installed), Step::Installed);
    }

    #[test]
    fn a_half_running_the_image_on_an_unconfirmed_test_boot_only_needs_keeping() {
        // What a run that died between the reset and the confirm leaves: the
        // image is there and running, and re-uploading it would be pointless.
        let staged = [slot(&[0xa1; 64], true, false)];
        assert_eq!(step_for(&left_image(), &staged), Step::Confirm);
    }

    #[test]
    fn the_same_build_in_both_slots_still_reads_as_installed() {
        // What flashing one build over and over leaves behind: every swap puts
        // the outgoing copy in the other slot, so both hold the same image.
        // The active one is what decides, and the run that follows must not try
        // to mark it — the board refuses to test-mark the slot it is running
        // from (image manager rc 33).
        let both = [
            slot(&[0xa1; 64], true, true),
            slot(&[0xa1; 64], false, true),
        ];
        assert_eq!(step_for(&left_image(), &both), Step::Installed);
    }

    #[test]
    fn a_half_running_something_else_is_written() {
        let other = [slot(&[0x99; 64], true, true)];
        assert_eq!(step_for(&left_image(), &other), Step::Upload);
        assert_eq!(step_for(&left_image(), &[]), Step::Upload);
    }

    #[test]
    fn the_image_sitting_in_the_spare_slot_does_not_count_as_installed() {
        // Staged is not running: an update that stopped after its upload has to
        // be finished, not skipped.
        let staged = [slot(&[0x99; 64], true, true), slot(&[0xa1; 64], false, false)];
        assert_eq!(step_for(&left_image(), &staged), Step::Upload);
    }

    #[test]
    fn the_wrong_halfs_image_is_never_mistaken_for_installed() {
        let left_installed = [slot(&[0xa1; 64], true, true)];
        assert_eq!(step_for(&right_image(), &left_installed), Step::Upload);
    }

    #[test]
    fn an_image_with_no_hash_to_compare_is_always_written() {
        let anything = [slot(&[0xa1; 64], true, true)];
        assert_eq!(step_for(&[0u8; 64], &anything), Step::Upload);
    }

    #[test]
    fn a_flat_pack_is_refused_and_a_charging_one_is_not() {
        assert!(!battery_allows(Some(0), false));
        assert!(!battery_allows(Some(MIN_SOC_PERCENT - 1), false));
        assert!(battery_allows(Some(MIN_SOC_PERCENT), false));
        assert!(battery_allows(Some(100), false));
        // On the charger, the level does not matter.
        assert!(battery_allows(Some(1), true));
    }

    #[test]
    fn a_level_nobody_can_read_allows_the_update() {
        // The direction that matters: a dead gauge must not make a keyboard
        // unflashable, which is the one state it could never be got out of.
        assert!(battery_allows(None, false));
        assert!(battery_allows(None, true));
    }
}
