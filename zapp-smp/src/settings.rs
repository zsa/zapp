//! settings_mgmt: reading and writing a whitelisted setting on the board.
//!
//! Group 3, command 0. Both halves' MCUboot serial recovery carries a handler
//! that answers exactly one setting — `esb/config`, the 30-byte ESB
//! provisioning bundle both halves link on. Read is `{"name": tstr}` →
//! `{"val": bstr, "rc": uint}`; write is `{"name": tstr, "val": bstr}` →
//! `{"rc": uint}`. The whitelist is exact: a name it does not know is `ENOENT`
//! and a value that is not the length it expects is `EINVAL`.

use ciborium::value::Value;
use rand::{Rng, RngCore};
use sha2::{Digest, Sha256};

use crate::proto;
use crate::{cbor, SmpError};

pub const GROUP: u16 = 3;
pub const CMD_RW: u8 = 0;

/// The one setting the board's whitelist answers.
pub const ESB_CONFIG_NAME: &str = "esb/config";

/// The exact length the board takes for `esb/config`; any other is `EINVAL`.
pub const ESB_CONFIG_LEN: usize = 30;

// The layout inside those 30 bytes, from `tools/gen_esb_config.py`.
const RF_CHANNEL_MAX: u8 = 100;
const BASE0: std::ops::Range<usize> = 1..5;
const PREFIX0: usize = 5;
const DEVICE_ID: std::ops::Range<usize> = 6..14;
const AES128_KEY: std::ops::Range<usize> = 14..30;

/// A settings read request: the name to look up.
pub fn read_request(name: &str) -> Vec<u8> {
    cbor::encode(&cbor::map(vec![("name", Value::Text(name.to_string()))]))
}

/// A settings write request: the name and the raw value.
pub fn write_request(name: &str, value: &[u8]) -> Vec<u8> {
    cbor::encode(&cbor::map(vec![
        ("name", Value::Text(name.to_string())),
        ("val", cbor::bytes(value)),
    ]))
}

/// The value out of a read reply.
///
/// A miss on the whitelist, or a setting that was never written, comes back as
/// `ENOENT` — a [`SmpError::Remote`] the caller reads to tell "not provisioned"
/// apart from a real failure.
pub fn parse_read(payload: &[u8]) -> Result<Vec<u8>, SmpError> {
    let value = cbor::decode(payload)?;
    proto::check_rc(&value, GROUP)?;
    let val = cbor::field(&value, "val")
        .and_then(cbor::as_bytes)
        .ok_or_else(|| SmpError::Malformed("no \"val\" in the settings read".into()))?;
    Ok(val.to_vec())
}

/// Whether a write went through.
pub fn parse_write(payload: &[u8]) -> Result<(), SmpError> {
    let value = cbor::decode(payload)?;
    proto::check_rc(&value, GROUP)
}

/// The 30-byte ESB provisioning bundle, valid by construction.
///
/// Both halves take byte-identical bundles: there is no role field, and the
/// pair links only when both carry the same one. The address rules come from
/// `tools/gen_esb_config.py` — a lead byte ESB reserves, or an all-zero
/// address, is not a bundle a keyboard can talk over.
#[derive(Clone, PartialEq, Eq)]
pub struct EsbConfig([u8; ESB_CONFIG_LEN]);

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum EsbConfigError {
    #[error("an esb/config is {ESB_CONFIG_LEN} bytes and this is {0}")]
    BadLength(usize),

    #[error("a key is 60 hex characters and this is {0}")]
    BadHexLength(usize),

    #[error("{0:?} is not hexadecimal")]
    NotHex(String),

    #[error("the address leads with {0:#04x}, which ESB reserves (0x55 or 0xaa)")]
    ReservedLeadByte(u8),

    #[error("the address is all zero, which is not a usable ESB address")]
    ZeroAddress,
}

impl EsbConfig {
    /// A fresh bundle from the system CSPRNG, retrying until the address is one
    /// ESB can use.
    pub fn generate() -> Self {
        let mut rng = rand::rng();
        let mut bytes = [0u8; ESB_CONFIG_LEN];
        loop {
            bytes[0] = rng.random_range(0..=RF_CHANNEL_MAX);
            rng.fill_bytes(&mut bytes[BASE0.start..=PREFIX0]);
            if valid_address(&bytes[BASE0], bytes[PREFIX0]) {
                break;
            }
        }
        rng.fill_bytes(&mut bytes[DEVICE_ID.start..ESB_CONFIG_LEN]);
        Self(bytes)
    }

    /// A bundle from bytes an operator supplied, checked against the address
    /// rules the generator obeys.
    pub fn from_bytes(bytes: [u8; ESB_CONFIG_LEN]) -> Result<Self, EsbConfigError> {
        let lead = bytes[BASE0.start];
        if lead == 0x55 || lead == 0xAA {
            return Err(EsbConfigError::ReservedLeadByte(lead));
        }
        if !valid_address(&bytes[BASE0], bytes[PREFIX0]) {
            return Err(EsbConfigError::ZeroAddress);
        }
        Ok(Self(bytes))
    }

    /// A bundle from 60 hexadecimal characters.
    pub fn parse_hex(text: &str) -> Result<Self, EsbConfigError> {
        let trimmed: String = text.split_whitespace().collect();
        if trimmed.len() != ESB_CONFIG_LEN * 2 {
            return Err(EsbConfigError::BadHexLength(trimmed.len()));
        }
        let raw = hex::decode(&trimmed).map_err(|_| EsbConfigError::NotHex(trimmed))?;
        let bytes: [u8; ESB_CONFIG_LEN] = raw
            .try_into()
            .map_err(|raw: Vec<u8>| EsbConfigError::BadLength(raw.len()))?;
        Self::from_bytes(bytes)
    }

    /// A bundle from the raw value a board read back, checked the same way.
    pub fn from_readback(bytes: &[u8]) -> Result<Self, EsbConfigError> {
        let bytes: [u8; ESB_CONFIG_LEN] = bytes
            .try_into()
            .map_err(|_| EsbConfigError::BadLength(bytes.len()))?;
        Self::from_bytes(bytes)
    }

    pub fn as_bytes(&self) -> &[u8; ESB_CONFIG_LEN] {
        &self.0
    }

    pub fn rf_channel(&self) -> u8 {
        self.0[0]
    }

    pub fn base0(&self) -> &[u8] {
        &self.0[BASE0]
    }

    pub fn prefix0(&self) -> u8 {
        self.0[PREFIX0]
    }

    pub fn device_id(&self) -> &[u8] {
        &self.0[DEVICE_ID]
    }

    /// The first four bytes of SHA-256 over the AES key, so two bundles can be
    /// told apart without either key ever being printed.
    pub fn key_fingerprint(&self) -> [u8; 4] {
        let digest = Sha256::digest(&self.0[AES128_KEY]);
        [digest[0], digest[1], digest[2], digest[3]]
    }
}

impl std::fmt::Debug for EsbConfig {
    /// The AES key never reaches a log or a screen, so neither does the whole
    /// value: a fingerprint stands in for it.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EsbConfig")
            .field("rf_channel", &self.rf_channel())
            .field("fingerprint", &hex(&self.key_fingerprint()))
            .finish_non_exhaustive()
    }
}

/// Whether an ESB address is one the radio can use.
///
/// Ported from `gen_esb_config.py`: the most significant base byte must not be
/// one of the two ESB reserves, and the fixed five-byte address must not be
/// all zero.
fn valid_address(base0: &[u8], prefix0: u8) -> bool {
    let reserved = base0[0] == 0x55 || base0[0] == 0xAA;
    let all_zero = prefix0 == 0x00 && base0[0] == 0x00 && base0[1] == 0x00;
    !reserved && !all_zero
}

/// Lower-case hex, for a fingerprint and nothing secret.
pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// A fingerprint of a raw value, for naming one in a message without printing
/// it.
///
/// Over the AES-key region of a full bundle, so it matches
/// [`EsbConfig::key_fingerprint`]; over the whole slice for anything else, so
/// even a garbled read-back can still be named.
pub fn fingerprint_hex(bytes: &[u8]) -> String {
    let region = if bytes.len() == ESB_CONFIG_LEN {
        &bytes[AES128_KEY]
    } else {
        bytes
    };
    hex(&Sha256::digest(region)[..4])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_read_request_is_just_the_name() {
        let value = cbor::decode(&read_request(ESB_CONFIG_NAME)).unwrap();
        assert_eq!(cbor::field(&value, "name").unwrap().as_text(), Some(ESB_CONFIG_NAME));
    }

    #[test]
    fn a_write_request_is_the_name_and_the_value_as_a_byte_string() {
        let value = cbor::decode(&write_request(ESB_CONFIG_NAME, &[1, 2, 3])).unwrap();
        assert_eq!(cbor::field(&value, "name").unwrap().as_text(), Some(ESB_CONFIG_NAME));
        // A byte string, not an array of integers, or the board rejects it.
        assert_eq!(
            cbor::as_bytes(cbor::field(&value, "val").unwrap()),
            Some([1, 2, 3].as_slice())
        );
    }

    #[test]
    fn a_read_reply_hands_back_the_value() {
        let reply = cbor::encode(&cbor::map(vec![
            ("val", cbor::bytes(&[0x5a; ESB_CONFIG_LEN])),
            ("rc", cbor::uint(0)),
        ]));
        assert_eq!(parse_read(&reply).unwrap(), vec![0x5a; ESB_CONFIG_LEN]);
    }

    #[test]
    fn a_missing_setting_reads_back_as_a_remote_no_entry() {
        // ENOENT is 5, and this is how "not provisioned" is told from a real
        // failure on --show.
        let reply = cbor::encode(&cbor::map(vec![("rc", cbor::uint(5))]));
        assert!(matches!(
            parse_read(&reply),
            Err(SmpError::Remote { group: GROUP, rc: 5, .. })
        ));
    }

    #[test]
    fn a_write_that_was_refused_for_a_bad_length_is_an_error() {
        let reply = cbor::encode(&cbor::map(vec![("rc", cbor::uint(3))]));
        assert!(matches!(
            parse_write(&reply),
            Err(SmpError::Remote { group: GROUP, rc: 3, .. })
        ));
    }

    #[test]
    fn a_generated_bundle_is_thirty_valid_bytes() {
        let config = EsbConfig::generate();
        assert_eq!(config.as_bytes().len(), ESB_CONFIG_LEN);
        assert!(config.rf_channel() <= RF_CHANNEL_MAX);
        // Round-trips through the validator it was built to satisfy.
        EsbConfig::from_bytes(*config.as_bytes()).expect("a generated bundle is valid");
    }

    #[test]
    fn generation_never_produces_a_reserved_lead_byte_or_a_zero_address() {
        // Statistical: the exclusions are one-in-a-few odds each, so a few
        // thousand draws is a real test of the retry loop rather than a lucky
        // pass.
        for _ in 0..5000 {
            let config = EsbConfig::generate();
            let base0 = config.base0();
            assert_ne!(base0[0], 0x55);
            assert_ne!(base0[0], 0xAA);
            assert!(
                !(config.prefix0() == 0 && base0[0] == 0 && base0[1] == 0),
                "an all-zero address slipped through"
            );
            assert!(config.rf_channel() <= RF_CHANNEL_MAX);
        }
    }

    #[test]
    fn generation_spreads_across_the_channels_and_the_key() {
        // Not a stuck RNG: over many draws the channel and the fingerprint both
        // move. A constant generator would fail this even while passing the
        // validity rules.
        let mut channels = std::collections::HashSet::new();
        let mut fingerprints = std::collections::HashSet::new();
        for _ in 0..200 {
            let config = EsbConfig::generate();
            channels.insert(config.rf_channel());
            fingerprints.insert(config.key_fingerprint());
        }
        assert!(channels.len() > 10, "the channel barely moved");
        assert_eq!(fingerprints.len(), 200, "the key repeated");
    }

    #[test]
    fn sixty_hex_characters_parse_to_the_bytes_they_spell() {
        let mut bytes = [0u8; ESB_CONFIG_LEN];
        bytes[0] = 42; // rf_channel
        bytes[1] = 0x12; // base0 lead, not reserved
        bytes[2] = 0x34;
        let text = hex(&bytes);
        let config = EsbConfig::parse_hex(&text).unwrap();
        assert_eq!(config.as_bytes(), &bytes);
        assert_eq!(config.rf_channel(), 42);
    }

    #[test]
    fn a_key_of_the_wrong_length_is_refused() {
        assert_eq!(
            EsbConfig::parse_hex("abcd"),
            Err(EsbConfigError::BadHexLength(4))
        );
    }

    #[test]
    fn a_key_that_is_not_hex_is_refused() {
        let text = "zz".repeat(ESB_CONFIG_LEN);
        assert!(matches!(
            EsbConfig::parse_hex(&text),
            Err(EsbConfigError::NotHex(_))
        ));
    }

    #[test]
    fn a_key_with_a_reserved_lead_byte_is_refused() {
        let mut bytes = [0x11u8; ESB_CONFIG_LEN];
        bytes[1] = 0x55; // base0 lead
        assert_eq!(
            EsbConfig::from_bytes(bytes),
            Err(EsbConfigError::ReservedLeadByte(0x55))
        );
        bytes[1] = 0xAA;
        assert_eq!(
            EsbConfig::from_bytes(bytes),
            Err(EsbConfigError::ReservedLeadByte(0xAA))
        );
    }

    #[test]
    fn a_key_with_an_all_zero_address_is_refused() {
        let mut bytes = [0u8; ESB_CONFIG_LEN];
        bytes[0] = 10; // a channel, so only the address is at fault
        // base0[0], base0[1] and prefix0 all zero.
        assert_eq!(
            EsbConfig::from_bytes(bytes),
            Err(EsbConfigError::ZeroAddress)
        );
    }

    #[test]
    fn the_fingerprint_is_four_bytes_of_the_key_and_not_the_key() {
        let mut a = [0x11u8; ESB_CONFIG_LEN];
        a[1] = 0x12;
        let mut b = a;
        // Differ only in the AES key: the fingerprint has to move with it.
        b[AES128_KEY.start] ^= 0xff;
        let fa = EsbConfig::from_bytes(a).unwrap().key_fingerprint();
        let fb = EsbConfig::from_bytes(b).unwrap().key_fingerprint();
        assert_ne!(fa, fb);
        assert_eq!(fa.len(), 4);
    }

    #[test]
    fn the_debug_form_never_carries_the_key() {
        let config = EsbConfig::from_bytes({
            let mut bytes = [0x11u8; ESB_CONFIG_LEN];
            bytes[1] = 0x12;
            bytes
        })
        .unwrap();
        let shown = format!("{config:?}");
        // The AES key bytes are 0x11; the whole value must not be in the string.
        assert!(!shown.contains("11111111"), "{shown}");
        assert!(shown.contains("fingerprint"), "{shown}");
    }
}
