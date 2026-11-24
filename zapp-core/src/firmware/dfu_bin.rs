/// DFU suffix length in bytes.
pub const DFU_SUFFIX_LENGTH: usize = 16;

/// Information extracted from a DFU suffix found in a firmware file.
#[derive(Debug, Clone)]
pub struct DfuSuffix {
    pub vid: u16,
    pub pid: u16,
    /// End of the firmware data (start of the suffix).
    pub data_end: usize,
    /// End of the suffix (data_end + DFU_SUFFIX_LENGTH).
    pub suffix_end: usize,
}

/// Scan a firmware file for all valid DFU suffixes.
///
/// A DFU suffix is 16 bytes at the end of a firmware image.
/// Bytes 8-10 (from suffix start) contain "UFD" (reversed "DFU").
/// Bytes 2-3 contain the PID (little-endian).
/// Bytes 4-5 contain the VID (little-endian).
///
/// For dual-firmware files (Ignition), there are two concatenated images
/// each with their own suffix.
pub fn find_dfu_suffixes(data: &[u8]) -> Vec<DfuSuffix> {
    let mut suffixes = Vec::new();

    if data.len() < DFU_SUFFIX_LENGTH {
        return suffixes;
    }

    // Scan byte-by-byte looking for the "UFD" magic at offset 8 within a 16-byte window.
    // This matches the Go reference which scans the entire file.
    let mut i = 0;
    while i + DFU_SUFFIX_LENGTH <= data.len() {
        let window = &data[i..i + DFU_SUFFIX_LENGTH];

        if window[8] == b'U' && window[9] == b'F' && window[10] == b'D' {
            let pid = u16::from_le_bytes([window[2], window[3]]);
            let vid = u16::from_le_bytes([window[4], window[5]]);

            if pid != 0 {
                suffixes.push(DfuSuffix {
                    vid,
                    pid,
                    data_end: i,
                    suffix_end: i + DFU_SUFFIX_LENGTH,
                });
                // Skip past this suffix to avoid overlapping matches
                i += DFU_SUFFIX_LENGTH;
                continue;
            }
        }
        i += 1;
    }

    suffixes
}

/// The DFU suffix on the very end of a file, if there is one.
///
/// Not `find_dfu_suffixes`: that scans every offset, which is what a
/// dual-image container needs and what a raw application binary must not have
/// done to it. Three bytes of "UFD" turn up in a couple of hundred kilobytes of
/// compiled firmware often enough to matter, and a false match mid-file would
/// truncate the image. Only the last sixteen bytes can be a suffix, so only
/// they are looked at.
pub fn trailing_suffix(data: &[u8]) -> Option<DfuSuffix> {
    let start = data.len().checked_sub(DFU_SUFFIX_LENGTH)?;
    let window = &data[start..];

    if window[8] != b'U' || window[9] != b'F' || window[10] != b'D' {
        return None;
    }
    let pid = u16::from_le_bytes([window[2], window[3]]);
    if pid == 0 {
        return None;
    }
    Some(DfuSuffix {
        vid: u16::from_le_bytes([window[4], window[5]]),
        pid,
        data_end: start,
        suffix_end: data.len(),
    })
}

/// Build a DFU suffix for testing purposes.
#[cfg(test)]
pub fn build_test_suffix(vid: u16, pid: u16) -> [u8; DFU_SUFFIX_LENGTH] {
    let mut suffix = [0u8; DFU_SUFFIX_LENGTH];
    // bytes 2-3: PID (LE)
    suffix[2] = (pid & 0xFF) as u8;
    suffix[3] = (pid >> 8) as u8;
    // bytes 4-5: VID (LE)
    suffix[4] = (vid & 0xFF) as u8;
    suffix[5] = (vid >> 8) as u8;
    // bytes 8-10: "UFD"
    suffix[8] = b'U';
    suffix[9] = b'F';
    suffix[10] = b'D';
    suffix
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_suffix() {
        let data = vec![0u8; 32];
        assert!(find_dfu_suffixes(&data).is_empty());
    }

    #[test]
    fn test_too_short() {
        let data = vec![0u8; 8];
        assert!(find_dfu_suffixes(&data).is_empty());
    }

    #[test]
    fn test_single_suffix() {
        let mut data = vec![0xAA; 1024];
        let suffix = build_test_suffix(0x3297, 0x1969);
        // Place suffix at the end
        let start = data.len() - DFU_SUFFIX_LENGTH;
        data[start..].copy_from_slice(&suffix);

        let results = find_dfu_suffixes(&data);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].vid, 0x3297);
        assert_eq!(results[0].pid, 0x1969);
        assert_eq!(results[0].data_end, start);
        assert_eq!(results[0].suffix_end, data.len());
    }

    #[test]
    fn a_trailing_suffix_is_the_one_at_the_end_and_only_that() {
        let mut data = vec![0xAA; 1024];
        assert!(trailing_suffix(&data).is_none());

        let start = data.len() - DFU_SUFFIX_LENGTH;
        data[start..].copy_from_slice(&build_test_suffix(0x3297, 0x1978));
        let found = trailing_suffix(&data).unwrap();
        assert_eq!(found.pid, 0x1978);
        assert_eq!(found.data_end, start);
    }

    #[test]
    fn a_suffix_shaped_run_of_bytes_inside_an_image_is_not_a_suffix() {
        // The case that makes this different from find_dfu_suffixes: "UFD"
        // lands in the middle of a compiled binary now and then, and treating
        // it as the end of the image would flash two thirds of a firmware.
        let mut data = vec![0xAA; 1024];
        data[100..116].copy_from_slice(&build_test_suffix(0x3297, 0x1978));
        assert!(trailing_suffix(&data).is_none());
        assert_eq!(find_dfu_suffixes(&data).len(), 1, "which the scan does find");
    }

    #[test]
    fn a_file_shorter_than_a_suffix_carries_none() {
        assert!(trailing_suffix(&[]).is_none());
        assert!(trailing_suffix(&[0u8; 15]).is_none());
    }

    #[test]
    fn test_dual_suffix() {
        let mut data = Vec::new();
        // First firmware + suffix
        data.extend_from_slice(&[0x01; 512]);
        data.extend_from_slice(&build_test_suffix(0x3297, 0x0791));
        // Second firmware + suffix
        data.extend_from_slice(&[0x02; 256]);
        data.extend_from_slice(&build_test_suffix(0x3297, 0x1791));

        let results = find_dfu_suffixes(&data);
        assert_eq!(results.len(), 2);
        assert_eq!(results[0].pid, 0x0791);
        assert_eq!(results[0].data_end, 512);
        assert_eq!(results[1].pid, 0x1791);
        assert_eq!(results[1].data_end, 512 + DFU_SUFFIX_LENGTH + 256);
    }
}
