//! The two ways an SMP frame reaches a half.
//!
//! Both are [`zapp_smp::SmpTransport`], and both put the same NLIP lines on
//! the wire. What differs is how many pipe messages that takes:
//!
//! * The **left** half is a UART the bootloader writes straight through, so a
//!   whole frame's worth of lines goes out as one `DATA` message.
//! * The **right** half is reached through an I2C mailbox that takes one unit
//!   at a time, so each line is its own `DATA` message.
//!
//! Reading is the same either way: the inbound stream is bytes, not messages,
//! and a frame comes out when a length and a checksum agree.

use std::time::{Duration, Instant};

use zapp_smp::{SmpTransport, TransportError};

use crate::ignition::{Ignition, IgnitionLink};
use crate::nlip;
use crate::pipe::{self, Event, Inbound, Target};
use crate::WiredError;

/// How many events to keep for a caller trying to explain a failure.
const EVENT_HISTORY: usize = 16;

/// The half of a transport that does not depend on how frames are split.
struct Wired<L: IgnitionLink> {
    ignition: Ignition<L>,
    target: Target,
    /// The largest SMP frame that fits the board's message limit.
    frame_budget: usize,
    lines: nlip::LineAssembler,
    messages: pipe::Reassembler,
    events: Vec<Event>,
}

impl<L: IgnitionLink> Wired<L> {
    fn new(ignition: Ignition<L>, target: Target, payload_budget: usize) -> Self {
        let frame_budget = nlip::max_frame_in(payload_budget);
        log::debug!(
            "{target:?}: {payload_budget} bytes per message is {frame_budget} bytes of frame"
        );
        let mut wired = Self {
            ignition,
            target,
            frame_budget,
            lines: nlip::LineAssembler::default(),
            messages: pipe::Reassembler::default(),
            events: Vec::new(),
        };
        // Both assemblers are already empty here. Doing it anyway is what makes
        // "a transport starts from a known state" a property of the type rather
        // than of how it happens to be built today.
        wired.reset();
        wired
    }

    /// Forgets everything half-arrived, on both layers.
    fn reset(&mut self) {
        self.messages.reset();
        self.lines.reset();
        self.events.clear();
    }

    /// A frame as one message: every line run together.
    fn write_whole(&mut self, frame: &[u8]) -> Result<(), WiredError> {
        let encoded = nlip::encode_concat(frame)?;
        if encoded.len() > pipe::MAX_PAYLOAD {
            return Err(WiredError::FrameTooLong {
                len: encoded.len(),
                budget: pipe::MAX_PAYLOAD,
            });
        }
        self.ignition.send(self.target, &encoded)?;
        self.ignition.flush(self.target)
    }

    /// A frame as one message per line, for a target that takes one at a time.
    fn write_per_line(&mut self, frame: &[u8]) -> Result<(), WiredError> {
        for line in nlip::encode(frame)? {
            self.ignition.send(self.target, &line)?;
        }
        self.ignition.flush(self.target)
    }

    /// The next SMP frame, or nothing by `timeout`.
    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, WiredError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.drain()? {
                return Ok(Some(frame));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            let bytes = self.ignition.receive(left)?;
            if bytes.is_empty() {
                // The link waited out the time there was and nothing came.
                return Ok(None);
            }
            self.messages.push(&bytes);
        }
    }

    /// Everything already reassembled, up to the next whole frame.
    fn drain(&mut self) -> Result<Option<Vec<u8>>, WiredError> {
        loop {
            match self.messages.take() {
                Ok(Some(Inbound::Data { payload, .. })) => {
                    self.lines.push(&payload);
                    // Cannot fail: everything that is not a whole frame is
                    // skipped and counted rather than reported, because a
                    // stream you can join anywhere has no such thing as a
                    // byte the caller could act on.
                    if let Some(frame) = self.lines.take() {
                        return Ok(Some(frame));
                    }
                }
                Ok(Some(Inbound::Event { event, .. })) => self.record(event)?,
                Ok(None) => return Ok(None),
                // A message this build cannot read is a diagnostic lost, not a
                // transfer lost: the stream stays aligned either way.
                Err(err) => log::warn!("{err}"),
            }
        }
    }

    /// Notes an event, and fails the read if it means the link is gone.
    fn record(&mut self, event: Event) -> Result<(), WiredError> {
        if event.code.is_fatal() {
            log::error!("{event}");
        } else {
            log::debug!("{event}");
        }
        if self.events.len() == EVENT_HISTORY {
            self.events.remove(0);
        }
        self.events.push(event);
        if event.code.is_fatal() {
            return Err(WiredError::Event(event));
        }
        Ok(())
    }
}

macro_rules! transport {
    ($name:ident, $target:expr, $write:ident, $doc:literal) => {
        #[doc = $doc]
        pub struct $name<L: IgnitionLink> {
            inner: Wired<L>,
        }

        impl<L: IgnitionLink> $name<L> {
            /// Asks the board what one message may carry, and sizes requests
            /// from it.
            pub fn new(mut ignition: Ignition<L>) -> Result<Self, WiredError> {
                let budget = ignition.status()?.payload_budget();
                Ok(Self::with_payload_budget(ignition, budget))
            }

            /// The same, for a caller that has already read the status.
            pub fn with_payload_budget(ignition: Ignition<L>, payload_budget: usize) -> Self {
                Self {
                    inner: Wired::new(ignition, $target, payload_budget),
                }
            }

            /// The vendor interface underneath, for the requests that are not
            /// frames: selecting a half, releasing MD, resetting.
            pub fn ignition(&mut self) -> &mut Ignition<L> {
                &mut self.inner.ignition
            }

            /// What the bridge has reported, oldest first.
            pub fn events(&self) -> &[Event] {
                &self.inner.events
            }

            /// Forgets a frame half-arrived and everything reported so far.
            ///
            /// For a caller that has just pointed the bridge somewhere else:
            /// whatever is still buffered belongs to the half that was
            /// selected before, and the events belong to that exchange too.
            /// A transport built fresh per half — which is what both callers
            /// in this workspace do — gets this from its constructor.
            pub fn reset(&mut self) {
                self.inner.reset();
            }

            /// Bytes thrown away for want of a marker.
            ///
            /// Non-zero after joining a stream mid-exchange, which is worth
            /// being able to report when a transfer times out on a link that
            /// looks otherwise healthy.
            pub fn discarded(&self) -> usize {
                self.inner.lines.discarded()
            }

            pub fn into_ignition(self) -> Ignition<L> {
                self.inner.ignition
            }
        }

        impl<L: IgnitionLink + 'static> SmpTransport for $name<L> {
            /// The largest SMP frame one write can carry.
            ///
            /// MCUboot's serial recovery has no `os mcumgr_params` command, so
            /// `SmpClient::negotiate_request_limit` finds nothing and leaves
            /// the request limit at this number. It is the real per-request
            /// budget, not a placeholder, which is why it comes from the
            /// board's reported message limit rather than a constant.
            fn mtu(&self) -> usize {
                self.inner.frame_budget
            }

            fn write(&mut self, packet: &[u8]) -> Result<(), TransportError> {
                Ok(self.inner.$write(packet)?)
            }

            fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, TransportError> {
                Ok(self.inner.read(timeout)?)
            }
        }
    };
}

transport!(
    LeftTransport,
    Target::Left,
    write_whole,
    "The left half, over ignition's UART bridge: one message per frame."
);

transport!(
    RightTransport,
    Target::Right,
    write_per_line,
    "The right half, over the TRRS I2C mailbox: one message per line."
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ignition::{Ignition, IgnitionLink};
    use std::collections::VecDeque;

    /// A link that keeps whatever was written and answers from a queue.
    #[derive(Default)]
    struct Wire {
        written: Vec<Vec<u8>>,
        inbound: VecDeque<Vec<u8>>,
        status: Vec<u8>,
    }

    impl IgnitionLink for Wire {
        fn control_out(&mut self, _request: u8, _value: u16) -> Result<(), WiredError> {
            Ok(())
        }
        fn control_in(
            &mut self,
            _request: u8,
            _value: u16,
            _length: usize,
        ) -> Result<Vec<u8>, WiredError> {
            Ok(self.status.clone())
        }
        fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
            self.written.push(message.to_vec());
            Ok(())
        }
        fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
            Ok(self.inbound.pop_front().unwrap_or_default())
        }
    }

    fn status_bytes(max_payload: u16) -> Vec<u8> {
        let mut bytes = vec![0u8; crate::ignition::STATUS_LEN];
        bytes[0] = crate::ignition::STATUS_VERSION;
        bytes[1] = 1;
        bytes[10..12].copy_from_slice(&max_payload.to_le_bytes());
        bytes
    }

    /// Every `DATA` message a transport wrote, payloads only.
    fn payloads(messages: &[Vec<u8>]) -> Vec<Vec<u8>> {
        let mut reassembler = pipe::Reassembler::default();
        let mut out = Vec::new();
        for message in messages {
            reassembler.push(message);
            while let Some(raw) = reassembler.take_message().unwrap() {
                if let pipe::Outbound::Data { payload, .. } = raw.to_device().unwrap() {
                    out.push(payload);
                }
            }
        }
        out
    }

    #[test]
    fn the_left_half_gets_one_message_for_a_whole_frame() {
        let frame: Vec<u8> = (0..300u32).map(|byte| byte as u8).collect();
        let mut transport =
            LeftTransport::with_payload_budget(Ignition::new(Wire::default()), 1024);
        transport.write(&frame).unwrap();

        let written = transport.into_ignition().into_link().written;
        // One DATA and one FLUSH.
        assert_eq!(written.len(), 2);
        assert_eq!(written[1], pipe::flush(Target::Left));
        assert_eq!(
            payloads(&written),
            vec![nlip::encode_concat(&frame).unwrap()]
        );
    }

    #[test]
    fn the_right_half_gets_one_message_for_every_line() {
        let frame: Vec<u8> = (0..600u32).map(|byte| byte as u8).collect();
        let lines = nlip::encode(&frame).unwrap();
        assert!(lines.len() > 2, "a frame worth splitting");

        let mut transport =
            RightTransport::with_payload_budget(Ignition::new(Wire::default()), 1024);
        transport.write(&frame).unwrap();

        let written = transport.into_ignition().into_link().written;
        assert_eq!(payloads(&written), lines);
    }

    #[test]
    fn a_frame_too_long_for_one_message_is_refused_before_it_is_sent() {
        let frame = vec![0x5a; pipe::MAX_PAYLOAD];
        let mut transport =
            LeftTransport::with_payload_budget(Ignition::new(Wire::default()), 1024);
        let err = transport.write(&frame).unwrap_err();
        assert!(err.to_string().contains("message limit"), "{err}");
    }

    #[test]
    fn the_write_budget_is_the_frame_that_fits_the_boards_message_limit() {
        for budget in [64usize, 256, 512, 1024] {
            let transport =
                LeftTransport::with_payload_budget(Ignition::new(Wire::default()), budget);
            let most = transport.mtu();
            assert!(nlip::encoded_len(most) <= budget, "{most} in {budget}");
            assert!(nlip::encoded_len(most + 1) > budget);
        }
    }

    #[test]
    fn the_budget_comes_from_the_status_the_board_answers_with() {
        let ignition = Ignition::new(Wire {
            status: status_bytes(512),
            ..Wire::default()
        });
        let transport = LeftTransport::new(ignition).unwrap();
        assert_eq!(transport.mtu(), nlip::max_frame_in(512));
        // And nothing about it is a constant this crate chose.
        assert_ne!(nlip::max_frame_in(512), nlip::max_frame_in(1024));
    }

    #[test]
    fn a_reply_split_across_usb_packets_still_comes_back_as_one_frame() {
        let frame: Vec<u8> = (0..500u32).map(|byte| (byte % 251) as u8).collect();
        let message = pipe::data(Target::Left, &nlip::encode_concat(&frame).unwrap()).unwrap();

        let mut transport = LeftTransport::with_payload_budget(
            Ignition::new(Wire {
                inbound: message.chunks(64).map(<[u8]>::to_vec).collect(),
                ..Wire::default()
            }),
            1024,
        );
        assert_eq!(
            transport.read(Duration::from_millis(50)).unwrap(),
            Some(frame)
        );
    }

    /// The hardware failure, end to end through the transport.
    ///
    /// After a fresh BEGIN and SELECT, ignition's rings still hold the tail of
    /// the exchange before and push it out before the host has sent anything.
    /// Read as the start of a line, that tail swallows the real reply's first
    /// line and the transfer times out on a link that is working perfectly.
    #[test]
    fn a_reply_behind_the_previous_exchanges_tail_still_comes_back() {
        let previous: Vec<u8> = (0..300u32).map(|byte| byte as u8).collect();
        let previous = nlip::encode_concat(&previous).unwrap();
        // Cut mid-line, the way a ring buffer holds whatever it happens to.
        let stale = &previous[120..391];
        assert!(!stale.ends_with(b"\n"));
        let frame = b"the reply that matters".to_vec();

        let mut inbound = VecDeque::new();
        inbound.push_back(pipe::data(Target::Left, stale).unwrap());
        inbound.push_back(pipe::data(Target::Left, &nlip::encode_concat(&frame).unwrap()).unwrap());

        let mut transport = LeftTransport::with_payload_budget(
            Ignition::new(Wire {
                inbound,
                ..Wire::default()
            }),
            1024,
        );
        assert_eq!(
            transport.read(Duration::from_millis(50)).unwrap(),
            Some(frame)
        );
        assert!(transport.discarded() > 0, "the tail was not counted");
    }

    #[test]
    fn a_read_with_nothing_to_read_is_a_timeout_and_not_an_error() {
        let mut transport =
            LeftTransport::with_payload_budget(Ignition::new(Wire::default()), 1024);
        assert_eq!(transport.read(Duration::from_millis(5)).unwrap(), None);
    }

    #[test]
    fn a_bus_event_is_recorded_and_the_frame_after_it_still_arrives() {
        let frame = b"the frame".to_vec();
        let event = Event {
            code: pipe::EventCode::I2cRecovered,
            detail: 2,
        };
        let mut inbound = VecDeque::new();
        inbound.push_back(pipe::encode(pipe::TYPE_EVENT, Target::Right, &event.encode()).unwrap());
        inbound
            .push_back(pipe::data(Target::Right, &nlip::encode_concat(&frame).unwrap()).unwrap());

        let mut transport = RightTransport::with_payload_budget(
            Ignition::new(Wire {
                inbound,
                ..Wire::default()
            }),
            1024,
        );
        assert_eq!(
            transport.read(Duration::from_millis(50)).unwrap(),
            Some(frame)
        );
        assert_eq!(transport.events(), [event]);
    }

    #[test]
    fn the_mailbox_going_away_fails_the_read_rather_than_timing_out() {
        let event = Event {
            code: pipe::EventCode::MailboxGone,
            detail: 0x45,
        };
        let mut inbound = VecDeque::new();
        inbound.push_back(pipe::encode(pipe::TYPE_EVENT, Target::Right, &event.encode()).unwrap());

        let mut transport = RightTransport::with_payload_budget(
            Ignition::new(Wire {
                inbound,
                ..Wire::default()
            }),
            1024,
        );
        let err = transport.read(Duration::from_secs(30)).unwrap_err();
        assert!(err.to_string().contains("mailbox"), "{err}");
    }
}
