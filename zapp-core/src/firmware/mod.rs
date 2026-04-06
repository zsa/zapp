pub mod dfu_bin;
pub mod ihex;

use std::path::Path;

use crate::ZappError;

/// Parsed firmware ready for flashing.
#[derive(Debug, Clone)]
pub enum Firmware {
    /// Single DFU binary (suffix stripped).
    DfuBinary { data: Vec<u8>, vid: u16, pid: u16 },

    /// Dual-firmware file (e.g., Ignition Voyager STM32+GD32, or Moonlander revA+revB).
    /// Contains two concatenated DFU images, each with its own suffix.
    IgnitionDual {
        primary: DfuImage,
        alternate: DfuImage,
    },

    /// Intel HEX firmware for HALFKAY.
    IntelHex {
        /// Contiguous firmware data extracted from HEX records.
        data: Vec<u8>,
    },
}

/// A single DFU image extracted from a firmware file.
#[derive(Debug, Clone)]
pub struct DfuImage {
    pub data: Vec<u8>,
    pub vid: u16,
    pub pid: u16,
}

/// Load firmware from a file, auto-detecting format.
///
/// Detection logic:
/// - Files starting with `:` are Intel HEX
/// - Files containing valid DFU suffix(es) are DFU binary
pub fn load_firmware(path: &Path) -> Result<Firmware, ZappError> {
    let file_data = std::fs::read(path).map_err(|e| ZappError::Io {
        path: path.to_owned(),
        source: e,
    })?;

    if file_data.is_empty() {
        return Err(ZappError::InvalidFirmware("file is empty".into()));
    }

    // Intel HEX starts with ':'
    if file_data[0] == b':' {
        let data = ihex::parse_ihex(&file_data)?;
        return Ok(Firmware::IntelHex { data });
    }

    // Try DFU binary
    let suffixes = dfu_bin::find_dfu_suffixes(&file_data);

    if suffixes.is_empty() {
        return Err(ZappError::InvalidFirmware(
            "not a valid firmware file (no DFU suffix or Intel HEX header found)".into(),
        ));
    }

    if suffixes.len() == 1 {
        let suffix = &suffixes[0];
        let data = file_data[..suffix.data_end].to_vec();
        return Ok(Firmware::DfuBinary {
            data,
            vid: suffix.vid,
            pid: suffix.pid,
        });
    }

    // Dual-firmware (Ignition / Moonlander dual-revision)
    if suffixes.len() == 2 {
        let first = &suffixes[0];
        let second = &suffixes[1];

        let primary_data = file_data[..first.data_end].to_vec();
        let alternate_data = file_data[first.suffix_end..second.data_end].to_vec();

        return Ok(Firmware::IgnitionDual {
            primary: DfuImage {
                data: primary_data,
                vid: first.vid,
                pid: first.pid,
            },
            alternate: DfuImage {
                data: alternate_data,
                vid: second.vid,
                pid: second.pid,
            },
        });
    }

    Err(ZappError::InvalidFirmware(format!(
        "unexpected number of DFU suffixes: {}",
        suffixes.len()
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_empty_file() {
        let dir = std::env::temp_dir().join("zapp_test_empty");
        std::fs::write(&dir, b"").unwrap();
        let result = load_firmware(&dir);
        assert!(result.is_err());
        std::fs::remove_file(&dir).ok();
    }

    #[test]
    fn test_load_single_dfu() {
        // Build a minimal DFU binary: 4 bytes of firmware + 16-byte suffix
        let firmware_data = vec![0xAA, 0xBB, 0xCC, 0xDD];
        let suffix = dfu_bin::build_test_suffix(0x3297, 0x0791);
        let mut file = firmware_data.clone();
        file.extend_from_slice(&suffix);

        let dir = std::env::temp_dir().join("zapp_test_single_dfu");
        std::fs::write(&dir, &file).unwrap();

        let fw = load_firmware(&dir).unwrap();
        match fw {
            Firmware::DfuBinary { data, vid, pid } => {
                assert_eq!(data, firmware_data);
                assert_eq!(vid, 0x3297);
                assert_eq!(pid, 0x0791);
            }
            _ => panic!("expected DfuBinary"),
        }
        std::fs::remove_file(&dir).ok();
    }

    #[test]
    fn test_load_dual_dfu() {
        let fw1 = vec![0x01; 100];
        let fw2 = vec![0x02; 200];
        let suffix1 = dfu_bin::build_test_suffix(0x3297, 0x0791);
        let suffix2 = dfu_bin::build_test_suffix(0x3297, 0x1791);

        let mut file = fw1.clone();
        file.extend_from_slice(&suffix1);
        file.extend_from_slice(&fw2);
        file.extend_from_slice(&suffix2);

        let dir = std::env::temp_dir().join("zapp_test_dual_dfu");
        std::fs::write(&dir, &file).unwrap();

        let fw = load_firmware(&dir).unwrap();
        match fw {
            Firmware::IgnitionDual { primary, alternate } => {
                assert_eq!(primary.data, fw1);
                assert_eq!(primary.pid, 0x0791);
                assert_eq!(alternate.data, fw2);
                assert_eq!(alternate.pid, 0x1791);
            }
            _ => panic!("expected IgnitionDual"),
        }
        std::fs::remove_file(&dir).ok();
    }
}
