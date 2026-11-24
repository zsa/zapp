//! Framing for ignition's vendor bulk endpoint.
//!
//! Four bytes of header in both directions, then the payload:
//!
//! ```text
//! u8 type | u8 target | u16 len, little endian | u8 payload[len]
//! ```
//!
//! A message is not a USB packet. The endpoint moves 64 bytes at a time, a
//! message can be [`MAX_PAYLOAD`] bytes long, and the boundaries of the two
//! have nothing to do with each other — so the inbound side is a state
//! machine over a byte stream rather than a parser over a packet.
//!
//! Nothing here does I/O either.

/// Bytes of header, both directions.
pub const HEADER_LEN: usize = 4;

/// The most one message may carry. Ignition reports its own limit in the
/// STATUS response; this is the ceiling that limit is checked against.
pub const MAX_PAYLOAD: usize = 1024;

/// Host to device: bytes for the selected target.
pub const TYPE_DATA: u8 = 0x01;
/// Host to device: push what is buffered out to the target now.
pub const TYPE_FLUSH: u8 = 0x02;
/// Device to host: something happened that is not data.
pub const TYPE_EVENT: u8 = 0x02;

/// Which side of the keyboard a message is for.
///
/// `Selected` is whatever the last `SELECT` vendor request named, which is how
/// a session that only ever talks to one target avoids saying so every time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Target {
    #[default]
    Selected,
    Left,
    Right,
    Stm32,
}

impl Target {
    pub fn code(self) -> u8 {
        match self {
            Self::Selected => 0,
            Self::Left => 1,
            Self::Right => 2,
            Self::Stm32 => 3,
        }
    }

    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Selected),
            1 => Some(Self::Left),
            2 => Some(Self::Right),
            3 => Some(Self::Stm32),
            _ => None,
        }
    }
}

impl From<zapp_smp::Half> for Target {
    fn from(half: zapp_smp::Half) -> Self {
        match half {
            zapp_smp::Half::Left => Self::Left,
            zapp_smp::Half::Right => Self::Right,
        }
    }
}

/// What an `EVENT` message reports.
///
/// `Unknown` is deliberate: a firmware newer than this build will invent codes,
/// and an update that fails because the host did not recognise a diagnostic
/// would be the wrong outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventCode {
    I2cNack,
    I2cTimeout,
    I2cRecovered,
    UartOverrun,
    RxOverflow,
    MailboxUp,
    MailboxGone,
    TargetNotSelected,
    Unknown(u8),
}

impl EventCode {
    pub fn code(self) -> u8 {
        match self {
            Self::I2cNack => 0x01,
            Self::I2cTimeout => 0x02,
            Self::I2cRecovered => 0x03,
            Self::UartOverrun => 0x04,
            Self::RxOverflow => 0x05,
            Self::MailboxUp => 0x06,
            Self::MailboxGone => 0x07,
            Self::TargetNotSelected => 0x08,
            Self::Unknown(code) => code,
        }
    }

    pub fn from_code(code: u8) -> Self {
        match code {
            0x01 => Self::I2cNack,
            0x02 => Self::I2cTimeout,
            0x03 => Self::I2cRecovered,
            0x04 => Self::UartOverrun,
            0x05 => Self::RxOverflow,
            0x06 => Self::MailboxUp,
            0x07 => Self::MailboxGone,
            0x08 => Self::TargetNotSelected,
            other => Self::Unknown(other),
        }
    }

    /// Whether the link is still usable after this.
    ///
    /// A NACK or a recovered bus is the bridge coping; the mailbox going away
    /// or a target that was never selected is the session being over.
    pub fn is_fatal(self) -> bool {
        matches!(self, Self::MailboxGone | Self::TargetNotSelected)
    }
}

impl std::fmt::Display for EventCode {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::I2cNack => formatter.write_str("the target did not acknowledge on I2C"),
            Self::I2cTimeout => formatter.write_str("the I2C transfer timed out"),
            Self::I2cRecovered => formatter.write_str("the I2C bus was recovered"),
            Self::UartOverrun => formatter.write_str("the UART overran"),
            Self::RxOverflow => formatter.write_str("the receive buffer overflowed"),
            Self::MailboxUp => formatter.write_str("the mailbox came up"),
            Self::MailboxGone => formatter.write_str("the mailbox went away"),
            Self::TargetNotSelected => formatter.write_str("no target is selected"),
            Self::Unknown(code) => write!(formatter, "event {code:#04x} from a newer firmware"),
        }
    }
}

/// One `EVENT` payload: a code and whatever number goes with it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Event {
    pub code: EventCode,
    pub detail: u16,
}

impl Event {
    pub fn encode(self) -> Vec<u8> {
        let mut payload = vec![self.code.code()];
        payload.extend_from_slice(&self.detail.to_le_bytes());
        payload
    }

    pub fn decode(payload: &[u8]) -> Result<Self, PipeError> {
        if payload.len() < 3 {
            return Err(PipeError::ShortEvent(payload.len()));
        }
        Ok(Self {
            code: EventCode::from_code(payload[0]),
            detail: u16::from_le_bytes([payload[1], payload[2]]),
        })
    }
}

impl std::fmt::Display for Event {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{} ({})", self.code, self.detail)
    }
}

/// One message, before anyone has decided which way it was going.
///
/// Type `0x02` is `FLUSH` on the way to the device and `EVENT` on the way
/// back, so the framing can be undone without knowing the direction but the
/// meaning cannot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    pub kind: u8,
    pub target: Target,
    pub payload: Vec<u8>,
}

impl Message {
    /// What it means coming from the device.
    pub fn to_host(self) -> Result<Inbound, PipeError> {
        match self.kind {
            TYPE_DATA => Ok(Inbound::Data {
                target: self.target,
                payload: self.payload,
            }),
            TYPE_EVENT => Ok(Inbound::Event {
                target: self.target,
                event: Event::decode(&self.payload)?,
            }),
            other => Err(PipeError::UnknownType(other)),
        }
    }

    /// What it means going to the device.
    pub fn to_device(self) -> Result<Outbound, PipeError> {
        match self.kind {
            TYPE_DATA => Ok(Outbound::Data {
                target: self.target,
                payload: self.payload,
            }),
            TYPE_FLUSH => Ok(Outbound::Flush {
                target: self.target,
            }),
            other => Err(PipeError::UnknownType(other)),
        }
    }
}

/// One message from the device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Inbound {
    Data { target: Target, payload: Vec<u8> },
    Event { target: Target, event: Event },
}

/// One message to the device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outbound {
    Data { target: Target, payload: Vec<u8> },
    Flush { target: Target },
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum PipeError {
    #[error("a payload of {0} bytes is past the {MAX_PAYLOAD}-byte limit")]
    PayloadTooLong(usize),

    #[error("{0:#04x} is not a message type this host knows")]
    UnknownType(u8),

    #[error("{0:#04x} is not a target")]
    UnknownTarget(u8),

    #[error("an event needs three bytes of payload, got {0}")]
    ShortEvent(usize),
}

/// One message, header and payload, ready for the endpoint.
pub fn encode(kind: u8, target: Target, payload: &[u8]) -> Result<Vec<u8>, PipeError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(PipeError::PayloadTooLong(payload.len()));
    }
    let mut message = Vec::with_capacity(HEADER_LEN + payload.len());
    message.push(kind);
    message.push(target.code());
    message.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    message.extend_from_slice(payload);
    Ok(message)
}

/// Bytes for a target.
pub fn data(target: Target, payload: &[u8]) -> Result<Vec<u8>, PipeError> {
    encode(TYPE_DATA, target, payload)
}

/// Nothing more is coming; send what is buffered.
pub fn flush(target: Target) -> Vec<u8> {
    encode(TYPE_FLUSH, target, &[]).expect("an empty payload is within the limit")
}

/// Messages out of a byte stream that arrives in whatever pieces USB hands
/// over.
///
/// Feeding it a whole message, a message split down the middle, or four
/// messages at once all come to the same thing.
#[derive(Debug, Default)]
pub struct Reassembler {
    buffer: Vec<u8>,
}

impl Reassembler {
    pub fn push(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
    }

    /// The next whole message from the device, if one has arrived.
    pub fn take(&mut self) -> Result<Option<Inbound>, PipeError> {
        self.take_message()?.map(Message::to_host).transpose()
    }

    /// The next whole message, framing undone and nothing else.
    ///
    /// A message this host does not understand is consumed before it is
    /// reported, so the stream stays aligned and the caller can carry on.
    pub fn take_message(&mut self) -> Result<Option<Message>, PipeError> {
        if self.buffer.len() < HEADER_LEN {
            return Ok(None);
        }
        let len = u16::from_le_bytes([self.buffer[2], self.buffer[3]]) as usize;
        if len > MAX_PAYLOAD {
            // The length field cannot be trusted to find the end of a message
            // that claims more than the protocol allows, so there is nothing
            // to skip past: everything buffered goes.
            self.buffer.clear();
            return Err(PipeError::PayloadTooLong(len));
        }
        if self.buffer.len() < HEADER_LEN + len {
            return Ok(None);
        }

        let message: Vec<u8> = self.buffer.drain(..HEADER_LEN + len).collect();
        let (kind, target) = (message[0], message[1]);
        Ok(Some(Message {
            kind,
            target: Target::from_code(target).ok_or(PipeError::UnknownTarget(target))?,
            payload: message[HEADER_LEN..].to_vec(),
        }))
    }

    /// Bytes held back waiting for the rest of a message.
    pub fn pending(&self) -> usize {
        self.buffer.len()
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_header_is_a_type_a_target_and_a_little_endian_length() {
        let message = data(Target::Right, &[0xaa, 0xbb, 0xcc]).unwrap();
        assert_eq!(message, [0x01, 0x02, 0x03, 0x00, 0xaa, 0xbb, 0xcc]);
    }

    #[test]
    fn a_length_of_more_than_a_byte_is_little_endian() {
        let message = data(Target::Left, &vec![0u8; 300]).unwrap();
        assert_eq!(&message[..4], [0x01, 0x01, 0x2c, 0x01]);
        assert_eq!(message.len(), 304);
    }

    #[test]
    fn a_flush_carries_nothing() {
        assert_eq!(flush(Target::Selected), [0x02, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn a_payload_past_the_limit_is_refused_rather_than_truncated() {
        assert_eq!(
            data(Target::Left, &vec![0u8; MAX_PAYLOAD + 1]),
            Err(PipeError::PayloadTooLong(MAX_PAYLOAD + 1))
        );
        assert!(data(Target::Left, &vec![0u8; MAX_PAYLOAD]).is_ok());
    }

    #[test]
    fn every_target_survives_the_round_trip_and_nothing_else_is_one() {
        for target in [Target::Selected, Target::Left, Target::Right, Target::Stm32] {
            assert_eq!(Target::from_code(target.code()), Some(target));
        }
        assert_eq!(Target::from_code(4), None);
    }

    #[test]
    fn every_event_code_survives_the_round_trip() {
        for code in [
            EventCode::I2cNack,
            EventCode::I2cTimeout,
            EventCode::I2cRecovered,
            EventCode::UartOverrun,
            EventCode::RxOverflow,
            EventCode::MailboxUp,
            EventCode::MailboxGone,
            EventCode::TargetNotSelected,
        ] {
            assert_eq!(EventCode::from_code(code.code()), code);
        }
        // And the numbers are not reordered from one build to the next.
        assert_eq!(EventCode::I2cNack.code(), 0x01);
        assert_eq!(EventCode::TargetNotSelected.code(), 0x08);
    }

    #[test]
    fn a_code_this_build_has_never_heard_of_is_carried_rather_than_refused() {
        assert_eq!(EventCode::from_code(0x40), EventCode::Unknown(0x40));
        assert_eq!(EventCode::Unknown(0x40).code(), 0x40);
        assert!(!EventCode::Unknown(0x40).is_fatal());
    }

    #[test]
    fn an_event_is_a_code_and_a_little_endian_detail() {
        let event = Event {
            code: EventCode::I2cNack,
            detail: 0x0145,
        };
        assert_eq!(event.encode(), [0x01, 0x45, 0x01]);
        assert_eq!(Event::decode(&event.encode()).unwrap(), event);
    }

    #[test]
    fn an_event_with_no_detail_in_it_is_an_error_rather_than_a_zero() {
        assert_eq!(Event::decode(&[0x01, 0x02]), Err(PipeError::ShortEvent(2)));
    }

    #[test]
    fn a_message_read_back_is_the_message_that_went_in() {
        let mut reassembler = Reassembler::default();
        reassembler.push(&data(Target::Right, b"hello").unwrap());
        assert_eq!(
            reassembler.take().unwrap(),
            Some(Inbound::Data {
                target: Target::Right,
                payload: b"hello".to_vec()
            })
        );
        assert_eq!(reassembler.take().unwrap(), None);
    }

    #[test]
    fn an_event_message_comes_back_as_an_event() {
        let event = Event {
            code: EventCode::MailboxGone,
            detail: 0x45,
        };
        let mut reassembler = Reassembler::default();
        reassembler.push(&encode(TYPE_EVENT, Target::Right, &event.encode()).unwrap());
        assert_eq!(
            reassembler.take().unwrap(),
            Some(Inbound::Event {
                target: Target::Right,
                event
            })
        );
    }

    #[test]
    fn a_message_split_across_two_pushes_is_still_one_message() {
        let message = data(Target::Left, b"split me down the middle").unwrap();
        let mut reassembler = Reassembler::default();
        reassembler.push(&message[..7]);
        assert_eq!(reassembler.take().unwrap(), None);
        reassembler.push(&message[7..]);
        assert!(reassembler.take().unwrap().is_some());
    }

    #[test]
    fn a_header_split_across_pushes_is_not_read_early() {
        let message = data(Target::Left, &vec![0x5a; 500]).unwrap();
        let mut reassembler = Reassembler::default();
        for split in 0..HEADER_LEN {
            reassembler.push(&message[split..split + 1]);
            assert_eq!(reassembler.take().unwrap(), None, "at byte {split}");
        }
        reassembler.push(&message[HEADER_LEN..]);
        assert_eq!(
            reassembler.take().unwrap(),
            Some(Inbound::Data {
                target: Target::Left,
                payload: vec![0x5a; 500]
            })
        );
    }

    #[test]
    fn an_unknown_type_is_reported_without_losing_the_message_after_it() {
        let mut reassembler = Reassembler::default();
        reassembler.push(&encode(0x7f, Target::Left, b"what").unwrap());
        reassembler.push(&data(Target::Left, b"next").unwrap());
        assert_eq!(reassembler.take(), Err(PipeError::UnknownType(0x7f)));
        assert_eq!(
            reassembler.take().unwrap(),
            Some(Inbound::Data {
                target: Target::Left,
                payload: b"next".to_vec()
            })
        );
    }

    #[test]
    fn the_same_type_byte_is_a_flush_going_out_and_an_event_coming_back() {
        let mut reassembler = Reassembler::default();
        reassembler.push(&flush(Target::Left));
        let message = reassembler.take_message().unwrap().unwrap();
        assert_eq!(
            message.clone().to_device().unwrap(),
            Outbound::Flush {
                target: Target::Left
            }
        );
        // The same bytes read as a reply are an event with no payload, which
        // is not one — the direction is the caller's to know.
        assert_eq!(message.to_host(), Err(PipeError::ShortEvent(0)));
    }

    #[test]
    fn a_data_message_means_the_same_thing_in_both_directions() {
        let mut reassembler = Reassembler::default();
        reassembler.push(&data(Target::Stm32, b"either way").unwrap());
        let message = reassembler.take_message().unwrap().unwrap();
        assert_eq!(
            message.clone().to_device().unwrap(),
            Outbound::Data {
                target: Target::Stm32,
                payload: b"either way".to_vec()
            }
        );
        assert_eq!(
            message.to_host().unwrap(),
            Inbound::Data {
                target: Target::Stm32,
                payload: b"either way".to_vec()
            }
        );
    }

    #[test]
    fn a_length_past_the_limit_is_refused_rather_than_waited_on_forever() {
        let mut reassembler = Reassembler::default();
        reassembler.push(&[TYPE_DATA, 0x01, 0xff, 0xff]);
        assert_eq!(reassembler.take(), Err(PipeError::PayloadTooLong(0xffff)));
        assert_eq!(reassembler.pending(), 0);
    }

    /// A 64-bit xorshift, so the chunking is arbitrary but the failure is
    /// always the same failure.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn below(&mut self, bound: usize) -> usize {
            (self.next() % bound as u64) as usize
        }
    }

    /// The property the whole inbound path rests on: a message is whatever the
    /// header says it is, however the bytes were cut up on the way in.
    #[test]
    fn any_chunking_of_any_stream_of_messages_reassembles_to_the_same_messages() {
        for seed in 1..40u64 {
            let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15));

            let mut sent = Vec::new();
            let mut wire = Vec::new();
            for _ in 0..rng.below(20) + 1 {
                let target = Target::from_code(rng.below(4) as u8).unwrap();
                let message = if rng.below(4) == 0 {
                    let event = Event {
                        code: EventCode::from_code(rng.below(10) as u8),
                        detail: rng.below(0x1_0000) as u16,
                    };
                    wire.extend(encode(TYPE_EVENT, target, &event.encode()).unwrap());
                    Inbound::Event { target, event }
                } else {
                    let payload: Vec<u8> = (0..rng.below(MAX_PAYLOAD + 1))
                        .map(|byte| (byte % 253) as u8)
                        .collect();
                    wire.extend(data(target, &payload).unwrap());
                    Inbound::Data { target, payload }
                };
                sent.push(message);
            }

            let mut reassembler = Reassembler::default();
            let mut got = Vec::new();
            let mut at = 0;
            while at < wire.len() {
                let chunk = (rng.below(80) + 1).min(wire.len() - at);
                reassembler.push(&wire[at..at + chunk]);
                at += chunk;
                while let Some(message) = reassembler.take().unwrap() {
                    got.push(message);
                }
            }

            assert_eq!(got, sent, "seed {seed}");
            assert_eq!(reassembler.pending(), 0, "seed {seed}");
        }
    }

    /// The same stream, one byte at a time: the worst chunking there is.
    #[test]
    fn a_stream_delivered_a_byte_at_a_time_reassembles_the_same_way() {
        let messages = [
            data(Target::Left, b"one").unwrap(),
            encode(
                TYPE_EVENT,
                Target::Right,
                &Event {
                    code: EventCode::I2cRecovered,
                    detail: 3,
                }
                .encode(),
            )
            .unwrap(),
            data(Target::Stm32, &vec![0x11; MAX_PAYLOAD]).unwrap(),
        ];
        let wire: Vec<u8> = messages.concat();

        let mut reassembler = Reassembler::default();
        let mut got = 0;
        for byte in wire {
            reassembler.push(&[byte]);
            while reassembler.take().unwrap().is_some() {
                got += 1;
            }
        }
        assert_eq!(got, 3);
    }
}
