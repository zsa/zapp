//! MCUboot's serial-recovery line codec, and nothing else.
//!
//! An SMP frame goes out as `[len:u16 BE][frame][crc16:u16 BE]`, base64'd and
//! cut into lines: `\x06\x09` in front of the first, `\x04\x14` in front of
//! every one after it, `\n` at the end of each. The length counts the frame
//! and the CRC, not itself, and the CRC is XMODEM over the frame alone.
//!
//! Two limits are not advisory, and neither one reports itself when broken:
//!
//! * **A line's base64 body must be a multiple of four characters.** Zephyr's
//!   `base64_decode` accumulates every line into one buffer before decoding,
//!   so a line that ends mid-quantum shifts everything after it and the frame
//!   decodes to noise.
//! * **A line must be at most [`MAX_LINE`] bytes**, markers and newline
//!   included. Zephyr's receive ISR drops whatever arrives past
//!   `CONFIG_BOOT_MAX_LINE_INPUT_LEN` and says nothing, so an over-long line
//!   is a frame that is quietly truncated rather than one that is refused.
//!
//! Nothing here does I/O.

use base64::engine::general_purpose::STANDARD;
use base64::Engine;

/// The two bytes in front of a frame's first line.
pub const START: [u8; 2] = [0x06, 0x09];

/// The two bytes in front of every line after the first.
pub const CONT: [u8; 2] = [0x04, 0x14];

/// The most a line may be on the wire: two marker bytes, the base64, and the
/// newline. `CONFIG_BOOT_MAX_LINE_INPUT_LEN`.
pub const MAX_LINE: usize = 254;

/// The most base64 one line may carry.
///
/// [`MAX_LINE`] less the markers and the newline, rounded down to a whole
/// base64 quantum: 251 becomes 248.
pub const MAX_BODY: usize = (MAX_LINE - 3) & !3;

const _: () = assert!(MAX_BODY.is_multiple_of(4));
const _: () = assert!(MAX_BODY + START.len() < MAX_LINE);

/// The two-byte length and the two-byte CRC around a frame.
const OVERHEAD: usize = 4;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum NlipError {
    #[error("a frame of {0} bytes will not fit the 16-bit length field")]
    FrameTooLong(usize),

    #[error("the line is not base64: {0}")]
    NotBase64(String),

    #[error("a frame needs at least {OVERHEAD} bytes, got {0}")]
    Short(usize),

    #[error("the frame says it is {declared} bytes and it is {actual}")]
    LengthMismatch { declared: usize, actual: usize },

    #[error("the frame's checksum is {read:#06x}, the bytes come to {computed:#06x}")]
    BadCrc { read: u16, computed: u16 },
}

/// CRC-16/XMODEM: polynomial 0x1021, initial value zero, no reflection, no
/// final xor. What MCUboot checks, and what a wrong answer here looks like on
/// the device is silence.
pub fn crc16_xmodem(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for byte in data {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// The bytes that get base64'd: length, frame, checksum.
pub fn packet(frame: &[u8]) -> Result<Vec<u8>, NlipError> {
    let declared =
        u16::try_from(frame.len() + 2).map_err(|_| NlipError::FrameTooLong(frame.len()))?;
    let mut packet = Vec::with_capacity(frame.len() + OVERHEAD);
    packet.extend_from_slice(&declared.to_be_bytes());
    packet.extend_from_slice(frame);
    packet.extend_from_slice(&crc16_xmodem(frame).to_be_bytes());
    Ok(packet)
}

/// One SMP frame as the lines that carry it, newlines included.
pub fn encode(frame: &[u8]) -> Result<Vec<Vec<u8>>, NlipError> {
    let encoded = STANDARD.encode(packet(frame)?);
    Ok(encoded
        .as_bytes()
        .chunks(MAX_BODY)
        .enumerate()
        .map(|(index, body)| {
            let marker = if index == 0 { START } else { CONT };
            let mut line = Vec::with_capacity(marker.len() + body.len() + 1);
            line.extend_from_slice(&marker);
            line.extend_from_slice(body);
            line.push(b'\n');
            line
        })
        .collect())
}

/// The same lines, run together, for a transport that sends a whole frame at
/// once.
pub fn encode_concat(frame: &[u8]) -> Result<Vec<u8>, NlipError> {
    Ok(encode(frame)?.concat())
}

/// How many bytes on the wire a frame of this length turns into.
pub fn encoded_len(frame_len: usize) -> usize {
    let base64 = (frame_len + OVERHEAD).div_ceil(3) * 4;
    let lines = base64.div_ceil(MAX_BODY).max(1);
    base64 + lines * (START.len() + 1)
}

/// The longest frame whose encoding still fits in `budget` bytes.
///
/// Zero when the budget cannot carry even an empty frame. This is the inverse
/// of [`encoded_len`], which is monotonic, so a search finds it exactly.
pub fn max_frame_in(budget: usize) -> usize {
    if encoded_len(0) > budget {
        return 0;
    }
    let (mut low, mut high) = (0usize, budget);
    while low < high {
        let mid = (low + high).div_ceil(2);
        if encoded_len(mid) <= budget {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    low
}

/// One whole frame's base64, back to the frame it came from.
///
/// Checks both things the device checks and the device does not report: that
/// the length field describes what actually arrived, and that the checksum is
/// over those bytes.
pub fn decode(base64: &[u8]) -> Result<Vec<u8>, NlipError> {
    let decoded = STANDARD
        .decode(base64)
        .map_err(|err| NlipError::NotBase64(err.to_string()))?;
    if decoded.len() < OVERHEAD {
        return Err(NlipError::Short(decoded.len()));
    }

    let declared = u16::from_be_bytes([decoded[0], decoded[1]]) as usize;
    let actual = decoded.len() - 2;
    if declared != actual {
        return Err(NlipError::LengthMismatch { declared, actual });
    }

    let frame = &decoded[2..decoded.len() - 2];
    let read = u16::from_be_bytes([decoded[decoded.len() - 2], decoded[decoded.len() - 1]]);
    let computed = crc16_xmodem(frame);
    if read != computed {
        return Err(NlipError::BadCrc { read, computed });
    }
    Ok(frame.to_vec())
}

/// Where the next line starts, at or after `from`.
///
/// Scanning for the markers is sound rather than a heuristic: base64 is
/// `A-Za-z0-9+/=`, every character of it `0x2b` or above, so neither marker's
/// bytes can occur inside a line's body. `\r` and `\n` are not marker bytes
/// either. The only place `\x06\x09` or `\x04\x14` can appear is in front of a
/// line.
fn marker_at(bytes: &[u8], from: usize) -> Option<usize> {
    let at = bytes
        .get(from..)?
        .windows(START.len())
        .position(|pair| pair == START || pair == CONT)?;
    Some(at + from)
}

/// Frames out of a byte stream that arrives however it arrives, and wherever
/// it is joined.
///
/// Follows the markers and accumulates base64 the way the device does, so a
/// frame comes out only once the length field and the checksum both agree.
///
/// The discipline is resynchronisation, not splitting: a line begins at a
/// marker, never at wherever the last one happened to end. That is what a byte
/// stream you can join at an arbitrary point requires, and it is not
/// hypothetical here. Ignition's rings survive a session, so a fresh `BEGIN`
/// and `SELECT` can be followed by hundreds of bytes of the *previous*
/// exchange's tail before the host has sent anything at all. Assuming a line
/// starts where the buffer starts glues that tail onto the front of the real
/// reply's first line, which then has no marker, is discarded whole, and takes
/// the reply with it — a working link that times out.
///
/// So nothing here is fatal, and [`Self::take`] cannot fail. Everything that is
/// not a whole well-formed frame is something to skip and count:
///
/// * Bytes before the first marker — a stale tail, a boot banner, line noise.
/// * A continuation line with no frame open, which is what joining mid-frame
///   looks like from here.
/// * A line that never ended, when the next line's marker has already arrived.
/// * A frame whose base64, length or checksum does not come out.
///
/// Each of those is logged with its size and leaves the assembler ready for the
/// next marker, so one bad line never costs the frame behind it. [`Self::discarded`]
/// is the running total, because bytes quietly going missing is exactly the
/// failure this is meant to survive and it should still be visible afterwards.
#[derive(Debug, Default)]
pub struct LineAssembler {
    /// Bytes not yet cut into lines.
    stream: Vec<u8>,
    /// Base64 of the frame being assembled.
    base64: Vec<u8>,
    /// Bytes thrown away for want of a marker, over this assembler's life.
    discarded: usize,
}

impl LineAssembler {
    pub fn push(&mut self, bytes: &[u8]) {
        self.stream.extend_from_slice(bytes);
    }

    /// The next whole frame, if the lines so far make one.
    pub fn take(&mut self) -> Option<Vec<u8>> {
        loop {
            // ---- Line up on a marker ----
            match marker_at(&self.stream, 0) {
                Some(0) => {}
                Some(at) => self.throw_away(at, "before the first marker"),
                None => {
                    // Nothing buffered begins a line. All of it goes, except a
                    // marker that might be split across this push and the next.
                    let keep = self.stream.len().min(START.len() - 1);
                    self.throw_away(self.stream.len() - keep, "with no marker in them");
                    return None;
                }
            }

            // ---- Find where the line ends ----
            let newline = self.stream.iter().position(|byte| *byte == b'\n');
            let next = marker_at(&self.stream, START.len());
            let end = match (newline, next) {
                (Some(end), None) => end,
                (Some(end), Some(next)) if end < next => end,
                // The line never ended and the one after it has already begun,
                // so it is a line the device cut short. It cannot decode, and
                // the frame it belonged to cannot either.
                (_, Some(next)) => {
                    self.base64.clear();
                    self.throw_away(next, "of a line that never ended");
                    continue;
                }
                // Still arriving.
                (None, None) => return None,
            };

            // ---- Take it ----
            let line: Vec<u8> = self.stream.drain(..=end).collect();
            let wire = line.len();
            let mut body = &line[..end];
            if body.last() == Some(&b'\r') {
                body = &body[..body.len() - 1];
            }

            if body.starts_with(&START) {
                self.base64.clear();
                self.base64.extend_from_slice(&body[START.len()..]);
            } else {
                // A continuation, since the resync above landed on one of the
                // two markers. With no frame open it is the tail of an exchange
                // this assembler never saw the start of: normal after a
                // reconnect, and nothing to continue.
                if self.base64.is_empty() {
                    self.count(wire, "of a continuation with no frame open");
                    continue;
                }
                self.base64.extend_from_slice(&body[CONT.len()..]);
            }

            match self.complete() {
                Ok(Some(frame)) => {
                    self.base64.clear();
                    return Some(frame);
                }
                Ok(None) => continue,
                // A frame that will never decode is over. Keeping it would fail
                // every line after it too, and the reply that matters may well
                // be the next thing on the wire.
                Err(err) => {
                    self.count(self.base64.len(), "of a frame that will not decode");
                    log::debug!("the frame was dropped because {err}");
                    self.base64.clear();
                    continue;
                }
            }
        }
    }

    /// Drops `count` bytes off the front of the stream, and says so.
    fn throw_away(&mut self, count: usize, why: &str) {
        if count == 0 {
            return;
        }
        self.stream.drain(..count);
        self.count(count, why);
    }

    /// Notes bytes that went nowhere, wherever they were held.
    fn count(&mut self, bytes: usize, why: &str) {
        self.discarded += bytes;
        log::debug!(
            "discarding {bytes} bytes {why}; {} in all so far",
            self.discarded
        );
    }

    /// Whether what has accumulated is a whole frame yet.
    ///
    /// A partial frame is not an error: it is the next line's job. Only a
    /// frame that claims to be shorter than what arrived, or one whose
    /// checksum disagrees, is.
    fn complete(&self) -> Result<Option<Vec<u8>>, NlipError> {
        // Base64 only decodes a quantum at a time, and a well-formed sender
        // never ends a line mid-quantum.
        if !self.base64.len().is_multiple_of(4) || self.base64.len() < 4 {
            return Ok(None);
        }
        let decoded = STANDARD
            .decode(&self.base64)
            .map_err(|err| NlipError::NotBase64(err.to_string()))?;
        if decoded.len() < OVERHEAD {
            return Ok(None);
        }
        let declared = u16::from_be_bytes([decoded[0], decoded[1]]) as usize;
        if decoded.len() - 2 < declared {
            return Ok(None);
        }
        decode(&self.base64).map(Some)
    }

    /// Throws away a frame half-arrived, for a caller starting again.
    ///
    /// The resync makes this an optimisation rather than a correctness fix —
    /// stale bytes would be skipped anyway — but a new session should not start
    /// by rediscovering that the last one left something behind. The discard
    /// total is deliberately kept: it is a diagnostic for the whole run, not
    /// for one session of it.
    pub fn reset(&mut self) {
        let stale = self.stream.len() + self.base64.len();
        self.stream.clear();
        self.base64.clear();
        if stale > 0 {
            self.count(stale, "left over from an exchange that is over");
        }
    }

    /// Bytes held back waiting for the rest of a frame.
    pub fn pending(&self) -> usize {
        self.stream.len() + self.base64.len()
    }

    /// Bytes this assembler has thrown away for want of a marker.
    ///
    /// Zero on a clean stream. Anything else is the host having joined
    /// mid-exchange or having been sent something that is not a frame, which is
    /// worth being able to say out loud after a transfer that went wrong.
    pub fn discarded(&self) -> usize {
        self.discarded
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `python3 -c "..."`, a hand-rolled XMODEM CRC — poly 0x1021, init 0 —
    /// over the standard check string, plus two the codec actually meets.
    #[test]
    fn the_checksum_is_the_one_the_standard_names() {
        assert_eq!(crc16_xmodem(b"123456789"), 0x31c3);
        assert_eq!(crc16_xmodem(b""), 0x0000);
        assert_eq!(crc16_xmodem(b"A"), 0x58e5);
    }

    /// An image-state read — the first request every session sends — encoded
    /// by hand in Python and pasted here whole.
    ///
    /// ```text
    /// frame = 08 00 00 01 00 01 00 00 a0
    /// crc16 = 0xaf01
    /// packet = 000b 0800000100010000a0 af01
    /// base64 = AAsIAAABAAEAAKCvAQ==
    /// ```
    const STATE_READ: [u8; 9] = [0x08, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xa0];

    #[test]
    fn a_state_read_encodes_to_the_bytes_python_produces() {
        assert_eq!(crc16_xmodem(&STATE_READ), 0xaf01);
        assert_eq!(
            packet(&STATE_READ).unwrap(),
            [0x00, 0x0b, 0x08, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xa0, 0xaf, 0x01]
        );

        let lines = encode(&STATE_READ).unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0], b"\x06\x09AAsIAAABAAEAAKCvAQ==\n");
    }

    /// And the frame those bytes were hand-computed from is the one the client
    /// actually builds, rather than one invented for the test.
    #[test]
    fn the_golden_frame_is_the_frame_an_image_state_read_produces() {
        let frame = zapp_smp::proto::frame(zapp_smp::proto::Op::Read, 1, 0, 0, &[0xa0]).unwrap();
        assert_eq!(frame, STATE_READ);
    }

    #[test]
    fn that_same_line_decodes_back_to_the_frame() {
        let mut assembler = LineAssembler::default();
        assembler.push(b"\x06\x09AAsIAAABAAEAAKCvAQ==\n");
        assert_eq!(assembler.take(), Some(STATE_READ.to_vec()));
        assert_eq!(assembler.take(), None);
    }

    /// A 300-byte frame, `(i * 7 + 3) % 251`, run through Python's base64.
    fn long_frame() -> Vec<u8> {
        (0..300u32).map(|i| ((i * 7 + 3) % 251) as u8).collect()
    }

    const LONG_BASE64: &str = concat!(
        "AS4DChEYHyYtNDtCSVBXXmVsc3qBiI+WnaSrsrnAx87V3OPq8fgECxIZICcuNTxDSlFYX2ZtdHuC",
        "iZCXnqWss7rByM/W3eTr8vkFDBMaISgvNj1ES1JZYGdudXyDipGYn6attLvCydDX3uXs8/oGDRQb",
        "IikwNz5FTFNaYWhvdn2Ei5KZoKeutbzDytHY3+bt9AAHDhUcIyoxOD9GTVRbYmlwd36FjJOaoaiv",
        "tr3Ey9LZ4Ofu9QEIDxYdJCsyOUBHTlVcY2pxeH+GjZSboqmwt77FzNPa4ejv9gIJEBceJSwzOkFI",
        "T1ZdZGtyeYCHjpWco6qxuL/GzdTb4unw9wMKERgfJi00O0JJUFdeZWxzeoGIj5adpKuyucDHztXc",
        "4+rx+AQLEhkgJy41PENKUVgKGg==",
    );

    #[test]
    fn a_frame_over_one_line_matches_pythons_base64_end_to_end() {
        let frame = long_frame();
        assert_eq!(crc16_xmodem(&frame), 0x0a1a);

        let lines = encode(&frame).unwrap();
        let bodies: Vec<u8> = lines
            .iter()
            .flat_map(|line| line[2..line.len() - 1].to_vec())
            .collect();
        assert_eq!(String::from_utf8(bodies).unwrap(), LONG_BASE64);
        // 304 bytes of packet is 408 base64 characters: 248 and 160.
        assert_eq!(
            lines.iter().map(|line| line.len() - 3).collect::<Vec<_>>(),
            vec![248, 160]
        );
    }

    #[test]
    fn only_the_first_line_carries_the_start_marker() {
        let lines = encode(&long_frame()).unwrap();
        assert!(lines[0].starts_with(&START));
        for line in &lines[1..] {
            assert!(line.starts_with(&CONT), "{:?}", &line[..2]);
        }
        assert!(lines.iter().all(|line| line.ends_with(b"\n")));
    }

    /// The first silent killer: Zephyr decodes every line into one buffer, so
    /// a body that is not a whole number of quanta shifts the rest of the
    /// frame.
    #[test]
    fn every_lines_base64_body_is_a_whole_number_of_quanta() {
        for size in [0usize, 1, 2, 3, 100, 181, 182, 183, 400, 1000, 2475, 4096] {
            let frame: Vec<u8> = (0..size).map(|byte| byte as u8).collect();
            for (index, line) in encode(&frame).unwrap().iter().enumerate() {
                let body = line.len() - 3;
                assert_eq!(body % 4, 0, "line {index} of a {size}-byte frame: {body}");
            }
        }
    }

    /// The second: the receive ISR drops whatever is past the line limit and
    /// reports nothing, so a line one byte over is a frame that vanishes.
    #[test]
    fn no_line_ever_reaches_the_length_the_isr_truncates_at() {
        for size in [0usize, 1, 181, 182, 183, 500, 2475, 10_000] {
            let frame: Vec<u8> = (0..size).map(|byte| byte as u8).collect();
            for line in encode(&frame).unwrap() {
                assert!(
                    line.len() <= MAX_LINE,
                    "a {}-byte line from a {size}-byte frame",
                    line.len()
                );
            }
        }
    }

    #[test]
    fn the_body_limit_is_the_largest_quantum_that_fits_the_line() {
        assert_eq!(MAX_BODY, 248);
        assert_eq!(MAX_BODY + START.len() + 1, 251);
        // One quantum more would be 252 characters and a 255-byte line.
        assert!(MAX_BODY + 4 + START.len() + 1 > MAX_LINE);
    }

    #[test]
    fn a_frame_of_any_length_survives_the_round_trip() {
        for size in [0usize, 1, 7, 181, 182, 183, 247, 248, 249, 1000, 5000] {
            let frame: Vec<u8> = (0..size).map(|byte| (byte % 251) as u8).collect();
            let mut assembler = LineAssembler::default();
            assembler.push(&encode_concat(&frame).unwrap());
            assert_eq!(assembler.take(), Some(frame), "{size} bytes");
            assert_eq!(assembler.pending(), 0);
        }
    }

    #[test]
    fn frames_arriving_one_byte_at_a_time_still_come_out_whole() {
        let frame = long_frame();
        let wire = encode_concat(&frame).unwrap();
        let mut assembler = LineAssembler::default();
        let mut seen = None;
        for byte in &wire {
            assembler.push(&[*byte]);
            if let Some(done) = assembler.take() {
                seen = Some(done);
            }
        }
        assert_eq!(seen, Some(frame));
    }

    #[test]
    fn two_frames_back_to_back_come_out_as_two() {
        let first: Vec<u8> = (0..50u8).collect();
        let second: Vec<u8> = (0..300u32).map(|byte| byte as u8).collect();
        let mut wire = encode_concat(&first).unwrap();
        wire.extend(encode_concat(&second).unwrap());

        let mut assembler = LineAssembler::default();
        assembler.push(&wire);
        assert_eq!(assembler.take(), Some(first));
        assert_eq!(assembler.take(), Some(second));
        assert_eq!(assembler.take(), None);
    }

    #[test]
    fn a_line_the_board_printed_rather_than_framed_is_skipped() {
        let frame: Vec<u8> = (0..40u8).collect();
        let mut wire = b"*** Booting MCUboot ***\n".to_vec();
        wire.extend(encode_concat(&frame).unwrap());
        let mut assembler = LineAssembler::default();
        assembler.push(&wire);
        assert_eq!(assembler.take(), Some(frame));
    }

    #[test]
    fn a_frame_that_starts_again_abandons_the_one_before_it() {
        let frame: Vec<u8> = (0..300u32).map(|byte| byte as u8).collect();
        let lines = encode(&frame).unwrap();
        let mut assembler = LineAssembler::default();
        // The first line of one frame, then a whole other frame.
        assembler.push(&lines[0]);
        assert_eq!(assembler.take(), None);
        assembler.push(&encode_concat(b"short").unwrap());
        assert_eq!(assembler.take(), Some(b"short".to_vec()));
    }

    #[test]
    fn a_continuation_with_nothing_to_continue_is_dropped_rather_than_failed() {
        // Joining a stream mid-frame is the ordinary case this has to survive,
        // not an error: it is what a fresh session sees while the previous
        // exchange's tail is still coming out of the bootloader's rings.
        let mut assembler = LineAssembler::default();
        assembler.push(b"\x04\x14AAAA\n");
        assert_eq!(assembler.take(), None);
        // The whole line, markers and newline included: what came off the wire.
        assert_eq!(assembler.discarded(), 7);

        // And the frame behind it still arrives.
        assembler.push(&encode_concat(b"after").unwrap());
        assert_eq!(assembler.take(), Some(b"after".to_vec()));
    }

    #[test]
    fn a_flipped_bit_is_caught_by_the_checksum() {
        let frame: Vec<u8> = (0..60u8).collect();
        let mut packet = packet(&frame).unwrap();
        packet[10] ^= 0x01;
        let err = decode(STANDARD.encode(&packet).as_bytes()).unwrap_err();
        assert!(matches!(err, NlipError::BadCrc { .. }), "{err:?}");
    }

    #[test]
    fn a_length_field_that_lies_is_caught_before_the_checksum() {
        let frame: Vec<u8> = (0..60u8).collect();
        let mut packet = packet(&frame).unwrap();
        packet[1] = 0x09;
        let err = decode(STANDARD.encode(&packet).as_bytes()).unwrap_err();
        assert_eq!(
            err,
            NlipError::LengthMismatch {
                declared: 9,
                actual: 62
            }
        );
    }

    #[test]
    fn a_frame_too_long_for_the_length_field_is_refused_rather_than_wrapped() {
        assert_eq!(
            encode(&vec![0u8; 65_534]).unwrap_err(),
            NlipError::FrameTooLong(65_534)
        );
        assert!(encode(&vec![0u8; 65_533]).is_ok());
    }

    #[test]
    fn a_truncated_frame_yields_nothing_and_the_next_one_still_arrives() {
        // What the ISR leaves behind when a line runs past its buffer: fewer
        // bytes than the length field promised, and no way to know it yet.
        let frame: Vec<u8> = (0..300u32).map(|byte| byte as u8).collect();
        let wire = encode_concat(&frame).unwrap();
        let mut assembler = LineAssembler::default();
        assembler.push(&wire[..wire.len() - 41]);
        assembler.push(b"\n");
        assert_eq!(assembler.take(), None);

        assembler.push(&encode_concat(b"after").unwrap());
        assert_eq!(assembler.take(), Some(b"after".to_vec()));
    }

    #[test]
    fn a_frame_that_will_never_decode_is_not_retried_on_every_line() {
        let mut assembler = LineAssembler::default();
        // A length field of one over a packet of sixty-two.
        let mut packet = packet(&(0..60u8).collect::<Vec<_>>()).unwrap();
        packet[0..2].copy_from_slice(&1u16.to_be_bytes());
        let mut line = START.to_vec();
        line.extend_from_slice(STANDARD.encode(&packet).as_bytes());
        line.push(b'\n');
        assembler.push(&line);
        assert_eq!(assembler.take(), None);
        assert!(assembler.discarded() > 0);

        assembler.push(&encode_concat(b"after").unwrap());
        assert_eq!(assembler.take(), Some(b"after".to_vec()));
    }

    // ---- Joining the stream part-way through ---------------------------

    /// What the bootloader had waiting on the endpoint after a fresh BEGIN and
    /// SELECT, before the host had sent anything: the tail of the exchange
    /// before, cut off wherever the rings happened to hold it.
    ///
    /// Deliberately ends mid-line, with no newline. That is the case that broke
    /// it: splitting on newlines glues this onto the front of the next line, so
    /// the real reply's `START` ends up in the middle of a line that has no
    /// marker of its own and the whole thing is dropped.
    fn stale_tail() -> Vec<u8> {
        let previous = encode_concat(&long_frame()).unwrap();
        previous[120..391].to_vec()
    }

    #[test]
    fn a_stale_tail_in_front_of_a_reply_is_skipped_and_the_reply_still_arrives() {
        let frame: Vec<u8> = (0..80u8).collect();
        let mut wire = stale_tail();
        assert!(!wire.ends_with(b"\n"), "the tail has to end mid-line");
        wire.extend(encode_concat(&frame).unwrap());

        let mut assembler = LineAssembler::default();
        assembler.push(&wire);
        assert_eq!(assembler.take(), Some(frame));
        assert!(assembler.discarded() > 0);
        assert_eq!(assembler.pending(), 0);
    }

    #[test]
    fn a_stale_tail_arriving_in_usb_sized_pieces_is_skipped_just_the_same() {
        // The tail and the reply do not arrive as one buffer on a real link,
        // and a marker landing across a push boundary must not be missed.
        let frame: Vec<u8> = (0..200u8).collect();
        let mut wire = stale_tail();
        wire.extend(encode_concat(&frame).unwrap());

        let mut assembler = LineAssembler::default();
        let mut seen = None;
        for packet in wire.chunks(64) {
            assembler.push(packet);
            if let Some(done) = assembler.take() {
                seen = Some(done);
            }
        }
        assert_eq!(seen, Some(frame));
    }

    #[test]
    fn a_marker_split_across_two_pushes_is_still_found() {
        // One byte at a time is the worst case: the resync must hold back a
        // lone marker byte rather than discarding it as unmarked noise.
        let frame: Vec<u8> = (0..30u8).collect();
        let mut wire = b"noise with no marker in it".to_vec();
        wire.extend(encode_concat(&frame).unwrap());

        let mut assembler = LineAssembler::default();
        let mut seen = None;
        for byte in &wire {
            assembler.push(&[*byte]);
            if let Some(done) = assembler.take() {
                seen = Some(done);
            }
        }
        assert_eq!(seen, Some(frame));
    }

    #[test]
    fn a_corrupt_line_between_two_good_frames_costs_only_itself() {
        let first: Vec<u8> = (0..40u8).collect();
        let second: Vec<u8> = (100..180u8).collect();

        // A frame whose checksum will not come out, between two that will.
        let mut broken = packet(&(0..50u8).collect::<Vec<_>>()).unwrap();
        let last = broken.len() - 1;
        broken[last] ^= 0xff;
        let mut corrupt = START.to_vec();
        corrupt.extend_from_slice(STANDARD.encode(&broken).as_bytes());
        corrupt.push(b'\n');

        let mut wire = encode_concat(&first).unwrap();
        wire.extend(corrupt);
        wire.extend(encode_concat(&second).unwrap());

        let mut assembler = LineAssembler::default();
        assembler.push(&wire);
        assert_eq!(assembler.take(), Some(first));
        assert_eq!(assembler.take(), Some(second));
        assert_eq!(assembler.take(), None);
    }

    #[test]
    fn a_line_that_never_ended_does_not_swallow_the_frame_behind_it() {
        // A line the device cut short, with the next frame's marker arriving
        // before its newline ever did.
        let frame: Vec<u8> = (0..60u8).collect();
        let mut wire = b"\x06\x09AAsIAAABAAEAAKCv".to_vec();
        wire.extend(encode_concat(&frame).unwrap());

        let mut assembler = LineAssembler::default();
        assembler.push(&wire);
        assert_eq!(assembler.take(), Some(frame));
        assert_eq!(assembler.discarded(), 18);
    }

    #[test]
    fn a_clean_stream_discards_nothing_at_all() {
        // The property that keeps the resync honest: it must cost nothing on a
        // link behaving itself, or it is hiding something rather than surviving it.
        for size in [0usize, 1, 182, 183, 1000, 5000] {
            let frame: Vec<u8> = (0..size).map(|byte| (byte % 251) as u8).collect();
            let mut assembler = LineAssembler::default();
            assembler.push(&encode_concat(&frame).unwrap());
            assert_eq!(assembler.take(), Some(frame), "{size} bytes");
            assert_eq!(assembler.discarded(), 0, "{size} bytes");
            assert_eq!(assembler.pending(), 0, "{size} bytes");
        }
    }

    #[test]
    fn a_reset_forgets_a_frame_half_arrived_and_counts_what_it_dropped() {
        let lines = encode(&long_frame()).unwrap();
        let mut assembler = LineAssembler::default();
        assembler.push(&lines[0]);
        assert_eq!(assembler.take(), None);
        assert!(assembler.pending() > 0);

        assembler.reset();
        assert_eq!(assembler.pending(), 0);
        assert_eq!(assembler.discarded(), 248);

        // And what follows is read as a frame of its own rather than as the
        // continuation of the one that was abandoned.
        assembler.push(&encode_concat(b"next").unwrap());
        assert_eq!(assembler.take(), Some(b"next".to_vec()));
    }

    #[test]
    fn a_reset_on_a_clean_assembler_is_free() {
        let mut assembler = LineAssembler::default();
        assembler.reset();
        assembler.push(&encode_concat(b"clean").unwrap());
        assert_eq!(assembler.take(), Some(b"clean".to_vec()));
        assert_eq!(assembler.discarded(), 0);
    }

    #[test]
    fn nothing_but_noise_is_not_accumulated_for_ever() {
        // A stream carrying no markers at all must not grow the buffer without
        // limit: all but a possible split marker goes on every look.
        let mut assembler = LineAssembler::default();
        for _ in 0..100 {
            assembler.push(&[0x41; 512]);
            assert_eq!(assembler.take(), None);
            assert!(assembler.pending() <= START.len(), "{}", assembler.pending());
        }
        assert!(assembler.discarded() >= 100 * 512 - START.len());
    }

    #[test]
    fn the_wire_length_of_a_frame_is_what_the_encoder_writes() {
        for size in [0usize, 1, 100, 182, 183, 1000, 4096] {
            let frame: Vec<u8> = (0..size).map(|byte| byte as u8).collect();
            assert_eq!(
                encoded_len(size),
                encode_concat(&frame).unwrap().len(),
                "{size} bytes"
            );
        }
    }

    #[test]
    fn the_largest_frame_in_a_budget_is_the_largest_one_that_fits() {
        for budget in [0usize, 3, 8, 11, 24, 64, 251, 252, 1024, 2048, 4096] {
            let most = max_frame_in(budget);
            if most > 0 {
                assert!(encoded_len(most) <= budget, "{most} in {budget}");
            }
            assert!(encoded_len(most + 1) > budget, "{most} + 1 in {budget}");
        }
    }

    #[test]
    fn a_budget_that_cannot_hold_an_empty_frame_holds_nothing() {
        // Four bytes of overhead is two quanta, three bytes of markers.
        assert_eq!(encoded_len(0), 11);
        assert_eq!(max_frame_in(10), 0);
        // The same eleven bytes carry two of frame, because base64 rounds up.
        assert_eq!(max_frame_in(11), 2);
        assert_eq!(max_frame_in(15), 5);
    }
}
