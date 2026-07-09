use crate::ZappError;

/// Parse Intel HEX data into a contiguous firmware blob.
///
/// The result is a byte vector covering addresses 0 through the highest
/// address found in the HEX records, with gaps filled with 0xFF.
pub fn parse_ihex(data: &[u8]) -> Result<Vec<u8>, ZappError> {
    let text = std::str::from_utf8(data)
        .map_err(|_| ZappError::InvalidFirmware("invalid UTF-8 in HEX file".into()))?;

    let mut result: Vec<u8> = Vec::new();
    let mut base_address: u32 = 0;

    for (line_num, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if !line.starts_with(':') {
            return Err(ZappError::InvalidFirmware(format!(
                "line {}: expected ':' prefix",
                line_num + 1
            )));
        }

        let hex = &line[1..];
        if hex.len() < 10 {
            return Err(ZappError::InvalidFirmware(format!(
                "line {}: too short",
                line_num + 1
            )));
        }

        let bytes = parse_hex_bytes(hex).map_err(|_| {
            ZappError::InvalidFirmware(format!("line {}: invalid hex", line_num + 1))
        })?;

        // Verify checksum
        let checksum: u8 = bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
        if checksum != 0 {
            return Err(ZappError::InvalidFirmware(format!(
                "line {}: bad checksum",
                line_num + 1
            )));
        }

        let byte_count = bytes[0] as usize;
        let address = u16::from_be_bytes([bytes[1], bytes[2]]) as u32;
        let record_type = bytes[3];

        match record_type {
            // Data record
            0x00 => {
                let abs_addr = (base_address + address) as usize;
                let end = abs_addr + byte_count;
                if result.len() < end {
                    result.resize(end, 0xFF);
                }
                result[abs_addr..end].copy_from_slice(&bytes[4..4 + byte_count]);
            }
            // End of file
            0x01 => break,
            // Extended linear address
            0x04 => {
                if byte_count == 2 {
                    base_address = (u16::from_be_bytes([bytes[4], bytes[5]]) as u32) << 16;
                }
            }
            // Extended segment address
            0x02 => {
                if byte_count == 2 {
                    base_address = (u16::from_be_bytes([bytes[4], bytes[5]]) as u32) << 4;
                }
            }
            // Start linear address / start segment address — ignored
            0x03 | 0x05 => {}
            _ => {
                return Err(ZappError::InvalidFirmware(format!(
                    "line {}: unknown record type {:#04x}",
                    line_num + 1,
                    record_type
                )));
            }
        }
    }

    if result.is_empty() {
        return Err(ZappError::InvalidFirmware(
            "HEX file contains no data records".into(),
        ));
    }

    Ok(result)
}

fn parse_hex_bytes(hex: &str) -> Result<Vec<u8>, ()> {
    if hex.len() % 2 != 0 {
        return Err(());
    }
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).map_err(|_| ()))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_ihex() {
        // A minimal Intel HEX with one data record and EOF
        let hex = ":04000000DEADBEEFC4\n:00000001FF\n";
        let data = parse_ihex(hex.as_bytes()).unwrap();
        assert_eq!(data, vec![0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn test_ihex_with_gap() {
        // Data at address 0x0000 and 0x0010, gap filled with 0xFF
        let hex = ":02000000AABB99\n:020010001122BB\n:00000001FF\n";
        let data = parse_ihex(hex.as_bytes()).unwrap();
        assert_eq!(data[0], 0xAA);
        assert_eq!(data[1], 0xBB);
        // Gap should be 0xFF
        for &b in &data[2..0x10] {
            assert_eq!(b, 0xFF);
        }
        assert_eq!(data[0x10], 0x11);
        assert_eq!(data[0x11], 0x22);
    }

    #[test]
    fn test_ihex_bad_checksum() {
        let hex = ":04000000DEADBEEF00\n:00000001FF\n";
        assert!(parse_ihex(hex.as_bytes()).is_err());
    }

    #[test]
    fn test_ihex_empty() {
        let hex = ":00000001FF\n";
        assert!(parse_ihex(hex.as_bytes()).is_err());
    }
}
