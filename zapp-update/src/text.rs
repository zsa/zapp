//! The few ways a hash, a version and a bundle are written out, shared by both
//! transports and by whatever shows them.

use zapp_smp::{image, Bundle, Half};

/// Enough of a hash to recognise, since a SHA-512 is 128 characters of hex.
pub fn short(hash: &[u8]) -> String {
    let hex = image::hex(hash);
    match hex.len() > 20 {
        true => format!("{}…{}", &hex[..8], &hex[hex.len() - 8..]),
        false => hex,
    }
}

/// A version, or a word for a half that reported none.
pub fn named(version: &str) -> String {
    match version.is_empty() {
        true => "(no version)".to_string(),
        false => version.to_string(),
    }
}

/// Both halves' images, right first — the order they are written in.
pub fn sizes(bundle: &Bundle) -> String {
    [Half::Right, Half::Left]
        .iter()
        .map(|half| format!("{} {} KiB", half.as_str(), bundle.image(*half).len() / 1024))
        .collect::<Vec<_>>()
        .join(" + ")
}

/// A half's name as a label, capitalised.
pub fn titled(half: Half) -> &'static str {
    match half {
        Half::Left => "Left",
        Half::Right => "Right",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_long_hash_is_shortened_to_something_a_person_can_match() {
        let hash = vec![0xab; 64];
        assert_eq!(short(&hash), "abababab…abababab");
        // A short one is left whole rather than elided into nothing.
        assert_eq!(short(&[0x01, 0x02]), "0102");
    }
}
