//! SMP framing: an eight-byte header, then a CBOR map.
//!
//! Nothing here touches a radio, so the wire format is exercised entirely in
//! this file's tests.
//!
//! ```text
//! byte 0   : reserved (3 bits) | version (2 bits) | op (3 bits)
//! byte 1   : flags
//! bytes 2-3: payload length, big endian
//! bytes 4-5: group id, big endian
//! byte 6   : sequence number
//! byte 7   : command id
//! ```

use ciborium::value::Value;

use super::cbor;
use super::SmpError;

pub const HEADER_LEN: usize = 8;

/// The SMP protocol version this client speaks.
///
/// Version 1 exists and adds a richer error shape, but a device answers in the
/// version it was asked in, so speaking 0 is what keeps the replies simple, and
/// every mcumgr build understands it.
pub const VERSION: u8 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Read,
    ReadRsp,
    Write,
    WriteRsp,
}

impl Op {
    pub fn code(self) -> u8 {
        match self {
            Self::Read => 0,
            Self::ReadRsp => 1,
            Self::Write => 2,
            Self::WriteRsp => 3,
        }
    }

    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Read),
            1 => Some(Self::ReadRsp),
            2 => Some(Self::Write),
            3 => Some(Self::WriteRsp),
            _ => None,
        }
    }

    /// The op a device answers this one with.
    pub fn response(self) -> Self {
        match self {
            Self::Read => Self::ReadRsp,
            Self::Write => Self::WriteRsp,
            already => already,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub op: Op,
    pub version: u8,
    pub flags: u8,
    pub len: u16,
    pub group: u16,
    pub seq: u8,
    pub command: u8,
}

impl Header {
    pub fn encode(&self) -> [u8; HEADER_LEN] {
        let group = self.group.to_be_bytes();
        let len = self.len.to_be_bytes();
        [
            (self.version & 0x03) << 3 | self.op.code(),
            self.flags,
            len[0],
            len[1],
            group[0],
            group[1],
            self.seq,
            self.command,
        ]
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, SmpError> {
        let head: [u8; HEADER_LEN] = bytes
            .get(..HEADER_LEN)
            .and_then(|head| head.try_into().ok())
            .ok_or(SmpError::ShortHeader(bytes.len()))?;
        Ok(Self {
            op: Op::from_code(head[0] & 0x07).ok_or(SmpError::BadOp(head[0] & 0x07))?,
            version: (head[0] >> 3) & 0x03,
            flags: head[1],
            len: u16::from_be_bytes([head[2], head[3]]),
            group: u16::from_be_bytes([head[4], head[5]]),
            seq: head[6],
            command: head[7],
        })
    }
}

/// One request, header and payload, ready to be split across writes.
pub fn frame(op: Op, group: u16, command: u8, seq: u8, payload: &[u8]) -> Result<Vec<u8>, SmpError> {
    let len = u16::try_from(payload.len()).map_err(|_| SmpError::PayloadTooLong(payload.len()))?;
    let header = Header {
        op,
        version: VERSION,
        flags: 0,
        len,
        group,
        seq,
        command,
    };
    let mut packet = Vec::with_capacity(HEADER_LEN + payload.len());
    packet.extend_from_slice(&header.encode());
    packet.extend_from_slice(payload);
    Ok(packet)
}

/// A packet split into writes no larger than the MTU.
///
/// SMP over GATT has no framing of its own below the header: a request longer
/// than one ATT packet is simply the next bytes in the next write, and the
/// device puts them back together by counting the header's length field.
pub fn fragments(packet: &[u8], mtu: usize) -> Result<std::slice::Chunks<'_, u8>, SmpError> {
    if mtu == 0 {
        return Err(SmpError::MtuTooSmall(mtu));
    }
    Ok(packet.chunks(mtu))
}

/// The other end of `fragments`: notifications in, whole packets out.
#[derive(Debug, Default)]
pub struct Reassembler {
    buffer: Vec<u8>,
}

impl Reassembler {
    /// Adds one notification.
    pub fn push(&mut self, fragment: &[u8]) {
        self.buffer.extend_from_slice(fragment);
    }

    /// The next whole packet, if one has arrived.
    ///
    /// Call until it answers `None`: one notification can complete a packet and
    /// begin the next, and a caller that stops at the first would leave the
    /// second sitting in the buffer until a notification that may never come.
    pub fn take(&mut self) -> Result<Option<(Header, Vec<u8>)>, SmpError> {
        if self.buffer.len() < HEADER_LEN {
            return Ok(None);
        }
        let header = Header::decode(&self.buffer)?;
        let total = HEADER_LEN + usize::from(header.len);
        if self.buffer.len() < total {
            return Ok(None);
        }
        let rest = self.buffer.split_off(total);
        let packet = std::mem::replace(&mut self.buffer, rest);
        Ok(Some((header, packet[HEADER_LEN..].to_vec())))
    }

    /// Forgets a half-arrived packet, which is what the leftovers of a request
    /// that timed out are.
    pub fn reset(&mut self) {
        self.buffer.clear();
    }

    pub fn pending(&self) -> usize {
        self.buffer.len()
    }
}

/// Fails on a response that carries a management error.
///
/// Two shapes exist: `{"rc": n}`, which every version sends, and version 1's
/// `{"err": {"group": g, "rc": n}}`, whose `rc` is scoped to the group rather
/// than being one of the shared codes. Both are read, since a device is free to
/// answer in the newer shape.
pub fn check_rc(payload: &Value, group: u16) -> Result<(), SmpError> {
    if let Some(err) = cbor::field(payload, "err") {
        let rc = cbor::as_int(cbor::field(err, "rc").unwrap_or(&Value::Null)).unwrap_or(0);
        let group = cbor::as_int(cbor::field(err, "group").unwrap_or(&Value::Null))
            .and_then(|group| u16::try_from(group).ok())
            .unwrap_or(group);
        if rc != 0 {
            return Err(SmpError::Remote {
                group,
                rc,
                meaning: group_meaning(group, rc),
            });
        }
    }

    let Some(rc) = cbor::field(payload, "rc").and_then(cbor::as_int) else {
        return Ok(());
    };
    if rc == 0 {
        return Ok(());
    }
    Err(SmpError::Remote {
        group,
        rc,
        meaning: shared_meaning(rc),
    })
}

/// The shared `MGMT_ERR_*` codes, which are what a version 0 `rc` carries.
fn shared_meaning(rc: i64) -> &'static str {
    match rc {
        1 => "unknown error",
        2 => "out of memory",
        3 => "invalid argument",
        4 => "timeout",
        5 => "no entry",
        6 => "bad state",
        7 => "response too long",
        8 => "command not supported",
        9 => "corrupt",
        10 => "busy",
        11 => "access denied",
        12 => "protocol version too old",
        13 => "protocol version too new",
        _ => "unrecognised error",
    }
}

/// A group-scoped code, for the version 1 error shape.
fn group_meaning(group: u16, rc: i64) -> &'static str {
    match (group, rc) {
        (super::img::GROUP, 1) => "flash configuration query failed",
        (super::img::GROUP, 2) => "no free slot",
        (super::img::GROUP, 3) => "flash area open failed",
        (super::img::GROUP, 6) => "invalid slot",
        (super::img::GROUP, 8) => "image header is not MCUboot's",
        (super::img::GROUP, 9) => "image too large for the slot",
        (super::img::GROUP, 13) => "image already pending",
        (super::img::GROUP, 24) => "image is not confirmable",
        _ => "group-specific error",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image_state_read() -> Header {
        Header {
            op: Op::Read,
            version: VERSION,
            flags: 0,
            len: 1,
            group: 1,
            seq: 0x2a,
            command: 0,
        }
    }

    #[test]
    fn a_header_survives_a_round_trip() {
        for header in [
            image_state_read(),
            Header {
                op: Op::WriteRsp,
                version: 1,
                flags: 0,
                len: u16::MAX,
                group: 0xabcd,
                seq: 0xff,
                command: 0x7f,
            },
            Header {
                op: Op::Write,
                version: 0,
                flags: 0,
                len: 0,
                group: 0,
                seq: 0,
                command: 5,
            },
        ] {
            let bytes = header.encode();
            assert_eq!(Header::decode(&bytes).unwrap(), header, "{header:?}");
        }
    }

    #[test]
    fn the_header_is_laid_out_the_way_the_device_reads_it() {
        // An image-state read: op 0, one byte of payload, group 1, command 0.
        assert_eq!(
            image_state_read().encode(),
            [0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x2a, 0x00]
        );
        // Multi-byte fields are big endian, which is the one thing a native
        // encoding would get silently wrong on this machine.
        let wide = Header {
            len: 0x0102,
            group: 0x0304,
            ..image_state_read()
        };
        assert_eq!(wide.encode()[2..6], [0x01, 0x02, 0x03, 0x04]);
    }

    #[test]
    fn the_version_rides_above_the_op_in_the_first_byte() {
        let v1 = Header {
            op: Op::Write,
            version: 1,
            ..image_state_read()
        };
        assert_eq!(v1.encode()[0], 0b0000_1010);
        // And a device answering with reserved bits set is still understood.
        let decoded = Header::decode(&[0b1110_1011, 0, 0, 0, 0, 1, 0, 0]).unwrap();
        assert_eq!(decoded.op, Op::WriteRsp);
        assert_eq!(decoded.version, 1);
    }

    #[test]
    fn a_truncated_or_nonsense_header_is_refused() {
        assert!(matches!(
            Header::decode(&[0, 0, 0]),
            Err(SmpError::ShortHeader(3))
        ));
        // Ops only run 0..=3; 4 is not one.
        assert!(matches!(
            Header::decode(&[0x04, 0, 0, 0, 0, 0, 0, 0]),
            Err(SmpError::BadOp(4))
        ));
    }

    #[test]
    fn a_request_that_fits_is_one_write() {
        let packet = frame(Op::Write, 1, 1, 3, &[0xa0]).unwrap();
        assert_eq!(packet.len(), HEADER_LEN + 1);
        let written: Vec<&[u8]> = fragments(&packet, 244).unwrap().collect();
        assert_eq!(written, vec![packet.as_slice()]);
    }

    #[test]
    fn a_request_larger_than_the_mtu_is_split_across_writes() {
        let payload = vec![0x41; 500];
        let packet = frame(Op::Write, 1, 1, 0, &payload).unwrap();
        assert_eq!(packet.len(), 508);

        let written: Vec<&[u8]> = fragments(&packet, 100).unwrap().collect();
        assert_eq!(written.len(), 6);
        assert!(written[..5].iter().all(|fragment| fragment.len() == 100));
        assert_eq!(written[5].len(), 8);
        // Split and rejoined is the original, header included: there is no
        // per-fragment framing to account for.
        assert_eq!(written.concat(), packet);
    }

    #[test]
    fn an_mtu_of_zero_is_refused_rather_than_looping_forever() {
        assert!(matches!(
            fragments(&[0; 8], 0).map(|_| ()),
            Err(SmpError::MtuTooSmall(0))
        ));
    }

    #[test]
    fn a_payload_too_long_for_the_length_field_is_refused() {
        let huge = vec![0u8; usize::from(u16::MAX) + 1];
        assert!(matches!(
            frame(Op::Write, 1, 1, 0, &huge),
            Err(SmpError::PayloadTooLong(65536))
        ));
    }

    #[test]
    fn a_response_split_across_notifications_is_put_back_together() {
        let payload: Vec<u8> = (0..300u32).map(|byte| byte as u8).collect();
        let packet = frame(Op::WriteRsp, 1, 1, 7, &payload).unwrap();

        let mut reassembler = Reassembler::default();
        let mut recovered = None;
        for fragment in packet.chunks(23) {
            reassembler.push(fragment);
            if let Some(whole) = reassembler.take().unwrap() {
                recovered = Some(whole);
            }
        }

        let (header, body) = recovered.expect("the response completes");
        assert_eq!(header.len, 300);
        assert_eq!(header.seq, 7);
        assert_eq!(body, payload);
        assert_eq!(reassembler.pending(), 0);
    }

    #[test]
    fn a_header_split_across_notifications_is_not_decoded_early() {
        let packet = frame(Op::WriteRsp, 1, 1, 1, &[0xa0]).unwrap();
        let mut reassembler = Reassembler::default();

        // Four bytes in, the length field is only half there: guessing at it
        // would mean deciding the packet is over before it is.
        reassembler.push(&packet[..4]);
        assert_eq!(reassembler.take().unwrap(), None);
        reassembler.push(&packet[4..]);
        let (header, body) = reassembler.take().unwrap().expect("now it is whole");
        assert_eq!(header.len, 1);
        assert_eq!(body, vec![0xa0]);
    }

    #[test]
    fn two_packets_arriving_together_both_come_out() {
        let first = frame(Op::WriteRsp, 1, 1, 1, &[0xa0]).unwrap();
        let second = frame(Op::WriteRsp, 1, 1, 2, &[0xa0]).unwrap();
        let mut reassembler = Reassembler::default();
        reassembler.push(&[first.clone(), second].concat());

        let (one, _) = reassembler.take().unwrap().expect("the first");
        assert_eq!(one.seq, 1);
        let (two, _) = reassembler.take().unwrap().expect("the second");
        assert_eq!(two.seq, 2);
        assert_eq!(reassembler.take().unwrap(), None);
    }

    #[test]
    fn a_packet_ending_where_the_next_begins_leaves_the_remainder_alone() {
        let first = frame(Op::WriteRsp, 1, 1, 1, &[0xa0]).unwrap();
        let second = frame(Op::WriteRsp, 1, 1, 2, &[0x42; 40]).unwrap();
        let stream = [first, second].concat();

        let mut reassembler = Reassembler::default();
        // A notification boundary that falls in the middle of the second
        // packet, three bytes past the end of the first.
        reassembler.push(&stream[..12]);
        let (one, _) = reassembler.take().unwrap().expect("the first is whole");
        assert_eq!(one.seq, 1);
        assert_eq!(reassembler.pending(), 3);
        assert_eq!(reassembler.take().unwrap(), None);

        reassembler.push(&stream[12..]);
        let (two, body) = reassembler.take().unwrap().expect("and then the second");
        assert_eq!(two.seq, 2);
        assert_eq!(body.len(), 40);
    }

    #[test]
    fn resetting_drops_what_a_dead_request_left_behind() {
        let mut reassembler = Reassembler::default();
        reassembler.push(&[0x03, 0x00, 0x01, 0x00]);
        reassembler.reset();
        assert_eq!(reassembler.pending(), 0);
    }

    #[test]
    fn a_zero_rc_is_not_an_error() {
        let ok = cbor::map(vec![("rc", cbor::uint(0))]);
        assert!(check_rc(&ok, 1).is_ok());
        // And a response with no rc at all is the ordinary success case.
        assert!(check_rc(&cbor::map(vec![("off", cbor::uint(64))]), 1).is_ok());
    }

    #[test]
    fn a_non_zero_rc_is_surfaced_with_what_it_means() {
        let err = check_rc(&cbor::map(vec![("rc", cbor::uint(3))]), 1).unwrap_err();
        assert!(matches!(
            err,
            SmpError::Remote {
                group: 1,
                rc: 3,
                meaning: "invalid argument"
            }
        ));
        assert!(err.to_string().contains("invalid argument"));

        // An unrecognised code still fails, and still says which one it was.
        let odd = check_rc(&cbor::map(vec![("rc", cbor::uint(99))]), 1).unwrap_err();
        assert!(odd.to_string().contains("99"), "{odd}");
    }

    #[test]
    fn the_version_one_error_shape_is_understood_too() {
        let payload = cbor::map(vec![(
            "err",
            cbor::map(vec![("group", cbor::uint(1)), ("rc", cbor::uint(9))]),
        )]);
        let err = check_rc(&payload, 1).unwrap_err();
        assert!(matches!(err, SmpError::Remote { group: 1, rc: 9, .. }));
        // Group-scoped, so 9 is not "corrupt" the way the shared code is.
        assert!(err.to_string().contains("too large"), "{err}");
    }
}
