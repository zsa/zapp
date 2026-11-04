//! What an MCUboot-signed binary says about itself.
//!
//! Not firmware loading — `zapp_core::firmware` does that, for images with a
//! DFU suffix or Intel HEX records, and an MCUboot image has neither. It is
//! a flat binary whose first thirty-two bytes are a header, and reading them
//! is what catches the wrong file before it is written to a keyboard.

use sha2::{Digest, Sha256};

/// `IMAGE_MAGIC`, at the front of every MCUboot header.
const MAGIC: u32 = 0x96f3_b83d;
const HEADER_LEN: usize = 32;

/// The two TLV areas behind an image: the protected one, which the signature
/// covers, and the unprotected one, which carries the hash and the signature
/// itself.
const TLV_PROTECTED_MAGIC: u16 = 0x6908;
const TLV_MAGIC: u16 = 0x6907;

/// The hash TLVs MCUboot may name an image by, and the length each one is.
///
/// Which one a build uses is a build-time choice — `CONFIG_BOOT_IMG_HASH_ALG_*`
/// — and a Wireless Voyager signs with SHA-512, so assuming SHA-256 would find
/// nothing. The length is checked with the type because mcumgr checks both.
const HASH_TLVS: [(u16, usize); 3] = [(0x10, 32), (0x11, 48), (0x12, 64)];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImageInfo {
    pub version: String,
    pub load_address: u32,
    pub header_size: u16,
    /// The protected TLV area behind the image, its own four-byte header
    /// included. Zero on an image that has none.
    pub protected_tlv_size: u16,
    /// The image itself, header and trailing TLVs excluded.
    pub image_size: u32,
}

/// What the MCUboot header says, or `None` if there is not one.
///
/// Every field is little endian, whatever the target's own byte order.
pub fn inspect(data: &[u8]) -> Option<ImageInfo> {
    let header: &[u8; HEADER_LEN] = data.get(..HEADER_LEN)?.try_into().ok()?;
    let word = |at: usize| u32::from_le_bytes(header[at..at + 4].try_into().unwrap());
    let half = |at: usize| u16::from_le_bytes(header[at..at + 2].try_into().unwrap());

    if word(0) != MAGIC {
        return None;
    }
    Some(ImageInfo {
        load_address: word(4),
        header_size: half(8),
        protected_tlv_size: half(10),
        image_size: word(12),
        version: format!(
            "{}.{}.{}+{}",
            header[20],
            header[21],
            half(22),
            word(24)
        ),
    })
}

/// The hash a board names this image by, out of the image's own TLVs.
///
/// Not `sha256`, which is over the file as it lies: that digest is what an
/// upload announces so a device can resume one, and it is not what any slot is
/// called. The hash a slot is named by is a TLV in the image's trailer, put
/// there by the signing tool and copied out of flash by mcumgr, so reading it
/// here is the only way to ask "is this image already on the board?" without
/// reading the flash back.
///
/// `None` for anything that is not a signed image, or one whose trailer carries
/// no hash — either way, nothing to compare, so a caller should upload.
pub fn signed_hash(data: &[u8]) -> Option<Vec<u8>> {
    let info = inspect(data)?;
    let mut at = usize::from(info.header_size).checked_add(info.image_size as usize)?;

    // The protected area, when there is one, sits between the image and the
    // hash. Its own header says how long it is, but the image header says the
    // same thing and is the field MCUboot itself walks by.
    if area_magic(data, at)? == TLV_PROTECTED_MAGIC {
        at = at.checked_add(usize::from(info.protected_tlv_size))?;
    }
    if area_magic(data, at)? != TLV_MAGIC {
        return None;
    }
    let end = at
        .checked_add(usize::from(le16(data, at + 2)?))?
        .min(data.len());

    let mut at = at + 4;
    while at + 4 <= end {
        let kind = le16(data, at)?;
        let len = usize::from(le16(data, at + 2)?);
        let value = data.get(at + 4..at + 4 + len)?;
        if HASH_TLVS.contains(&(kind, len)) {
            return Some(value.to_vec());
        }
        at += 4 + len;
    }
    None
}

fn area_magic(data: &[u8], at: usize) -> Option<u16> {
    le16(data, at)
}

fn le16(data: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes(data.get(at..at + 2)?.try_into().ok()?))
}

/// The digest the first upload request announces, over the whole file.
pub fn sha256(data: &[u8]) -> [u8; 32] {
    Sha256::digest(data).into()
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// A hash as typed on a command line, in either case, with or without
/// separators between the bytes.
pub fn unhex(text: &str) -> Option<Vec<u8>> {
    let digits: Vec<char> = text
        .chars()
        .filter(|c| !matches!(c, ':' | '-' | ' ' | '_'))
        .collect();
    if digits.is_empty() || !digits.len().is_multiple_of(2) {
        return None;
    }
    digits
        .chunks(2)
        .map(|pair| u8::from_str_radix(&pair.iter().collect::<String>(), 16).ok())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(version: (u8, u8, u16, u32)) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&MAGIC.to_le_bytes()); // magic
        bytes.extend_from_slice(&0x0000_c000u32.to_le_bytes()); // load address
        bytes.extend_from_slice(&512u16.to_le_bytes()); // header size
        bytes.extend_from_slice(&0u16.to_le_bytes()); // protected tlv size
        bytes.extend_from_slice(&123_456u32.to_le_bytes()); // image size
        bytes.extend_from_slice(&0u32.to_le_bytes()); // flags
        bytes.push(version.0);
        bytes.push(version.1);
        bytes.extend_from_slice(&version.2.to_le_bytes());
        bytes.extend_from_slice(&version.3.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes()); // padding
        assert_eq!(bytes.len(), HEADER_LEN);
        bytes
    }

    #[test]
    fn a_signed_image_names_its_version_and_size() {
        let info = inspect(&header((1, 2, 3, 4))).expect("an MCUboot image");
        assert_eq!(info.version, "1.2.3+4");
        assert_eq!(info.image_size, 123_456);
        assert_eq!(info.header_size, 512);
        assert_eq!(info.protected_tlv_size, 0);
        assert_eq!(info.load_address, 0xc000);
    }

    /// A whole small image: a 32-byte header, some payload, then a TLV area.
    fn signed(hash: (u16, usize), protected: bool) -> Vec<u8> {
        let payload = 64usize;
        let mut image = Vec::new();
        image.extend_from_slice(&MAGIC.to_le_bytes());
        image.extend_from_slice(&0u32.to_le_bytes()); // load address
        image.extend_from_slice(&(HEADER_LEN as u16).to_le_bytes());
        let protected_len = if protected { 4 + 4 + 8 } else { 0usize };
        image.extend_from_slice(&(protected_len as u16).to_le_bytes());
        image.extend_from_slice(&(payload as u32).to_le_bytes());
        image.extend_from_slice(&[0u8; 16]); // flags, version, padding
        assert_eq!(image.len(), HEADER_LEN);
        image.extend(std::iter::repeat_n(0xAAu8, payload));

        if protected {
            image.extend_from_slice(&TLV_PROTECTED_MAGIC.to_le_bytes());
            image.extend_from_slice(&(protected_len as u16).to_le_bytes());
            image.extend_from_slice(&0x0050u16.to_le_bytes()); // dependency
            image.extend_from_slice(&8u16.to_le_bytes());
            image.extend_from_slice(&[0x77; 8]);
        }

        // A keyhash first, so the walk has to step over one entry to find the
        // hash rather than reading the first thing it sees.
        let (kind, len) = hash;
        let area = 4 + (4 + 32) + (4 + len);
        image.extend_from_slice(&TLV_MAGIC.to_le_bytes());
        image.extend_from_slice(&(area as u16).to_le_bytes());
        image.extend_from_slice(&0x0001u16.to_le_bytes()); // keyhash
        image.extend_from_slice(&32u16.to_le_bytes());
        image.extend_from_slice(&[0x33; 32]);
        image.extend_from_slice(&kind.to_le_bytes());
        image.extend_from_slice(&(len as u16).to_le_bytes());
        image.extend(std::iter::repeat_n(0xEEu8, len));
        image
    }

    #[test]
    fn the_hash_a_board_names_an_image_by_comes_out_of_its_trailer() {
        // SHA-256, SHA-384 and SHA-512, since which one is there is a build
        // choice and a Wireless Voyager signs with the last of the three.
        for (kind, len) in HASH_TLVS {
            let found = signed_hash(&signed((kind, len), false)).expect("a hash");
            assert_eq!(found, vec![0xEE; len], "tlv {kind:#x}");
        }
    }

    #[test]
    fn a_protected_tlv_area_is_stepped_over_on_the_way_to_the_hash() {
        assert_eq!(
            signed_hash(&signed((0x12, 64), true)),
            Some(vec![0xEE; 64])
        );
    }

    #[test]
    fn an_image_with_no_hash_in_it_has_nothing_to_compare() {
        // The wrong length for its type is not the hash either: mcumgr checks
        // both, so a 32-byte SHA-512 TLV would never match what a board says.
        assert_eq!(signed_hash(&signed((0x12, 32), false)), None);
        assert_eq!(signed_hash(&[0u8; 64]), None);
        assert_eq!(signed_hash(&[]), None);
        // Truncated part-way through the trailer, which is a half-written file.
        let whole = signed((0x10, 32), false);
        assert_eq!(signed_hash(&whole[..whole.len() - 8]), None);
    }

    #[test]
    fn anything_else_is_not_one() {
        // A QMK .bin, an Intel HEX file, a truncated image.
        assert_eq!(inspect(&[0u8; 64]), None);
        assert_eq!(inspect(b":100000000C9434000C9451000C9451000C945100D1"), None);
        assert_eq!(inspect(&header((1, 0, 0, 0))[..16]), None);
        assert_eq!(inspect(&[]), None);
    }

    #[test]
    fn the_digest_is_over_the_file_as_it_lies() {
        // The empty SHA-256, which is the one every implementation agrees on.
        assert_eq!(
            hex(&sha256(b"")),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(sha256(b"zapp").len(), 32);
    }

    #[test]
    fn a_hash_round_trips_through_the_form_it_is_typed_in() {
        let hash = [0x0a, 0xff, 0x10, 0x00];
        assert_eq!(hex(&hash), "0aff1000");
        assert_eq!(unhex("0aff1000"), Some(hash.to_vec()));
        assert_eq!(unhex("0AFF1000"), Some(hash.to_vec()));
        assert_eq!(unhex("0a:ff:10:00"), Some(hash.to_vec()));
    }

    #[test]
    fn a_hash_that_is_not_one_is_refused() {
        assert_eq!(unhex("abc"), None);
        assert_eq!(unhex("zz"), None);
        assert_eq!(unhex(""), None);
        assert_eq!(unhex("::"), None);
    }
}
