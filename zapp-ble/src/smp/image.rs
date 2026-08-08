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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImageInfo {
    pub version: String,
    pub load_address: u32,
    pub header_size: u16,
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
        assert_eq!(info.load_address, 0xc000);
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
