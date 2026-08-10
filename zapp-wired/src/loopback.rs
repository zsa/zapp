//! The whole wired stack, with no wire.
//!
//! An [`IgnitionLink`] that is not USB: it takes the bulk messages a transport
//! writes, pulls the NLIP lines out of them, decodes the frames, and hands
//! them to `zapp-smp`'s in-memory SMP device. Replies come back the same way —
//! encoded, framed, and cut into 64-byte packets, because a reply that arrives
//! whole would not exercise the thing most likely to be wrong.
//!
//! Everything between `SmpClient` and the endpoint is therefore live: the
//! codec, the length and checksum checks, the pipe framing, the reassembly on
//! both sides, and the per-line split the right half needs. What is not here
//! is `nusb`, which is the only part a keyboard would add.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use zapp_smp::fake::FakeDevice;
use zapp_smp::proto::Reassembler as FrameReassembler;
use zapp_smp::{Half, SmpTransport, TransportError};

use crate::ignition::{Ignition, IgnitionLink, STATUS_LEN, STATUS_VERSION};
use crate::pipe::{self, Event, EventCode, Outbound, Target};
use crate::transport::{LeftTransport, RightTransport};
use crate::{nlip, WiredError};

/// What a USB packet holds, so a reply is split the way the bus splits it.
const PACKET: usize = 64;

/// The largest frame the in-memory device will reassemble.
const DEVICE_CAPACITY: usize = 2475;

/// The device on the other end, for a test that wants to look at it or lean on
/// it.
#[derive(Clone)]
pub struct LoopbackDevice {
    fake: Arc<Mutex<FakeDevice>>,
    /// Every `DATA` payload the host wrote, in order.
    sent: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl LoopbackDevice {
    /// The image it has been written so far.
    pub fn image(&self) -> Vec<u8> {
        self.fake.lock().unwrap().image.clone()
    }

    /// Every command it has been sent, as `(group, command)`.
    pub fn requests(&self) -> Vec<(u16, u8)> {
        self.fake.lock().unwrap().requests.lock().unwrap().clone()
    }

    /// How many times it has been told to reboot.
    pub fn resets(&self) -> u32 {
        *self.fake.lock().unwrap().resets.lock().unwrap()
    }

    /// The largest request that reached it whole.
    pub fn largest_request(&self) -> usize {
        *self.fake.lock().unwrap().largest.lock().unwrap()
    }

    /// Every NLIP line the host put on the wire, in order.
    ///
    /// The left half writes a whole frame as one payload and the right half
    /// writes one line at a time, so this splits payloads back into lines.
    pub fn lines(&self) -> Vec<Vec<u8>> {
        self.sent
            .lock()
            .unwrap()
            .iter()
            .flat_map(|payload| {
                payload
                    .split_inclusive(|byte| *byte == b'\n')
                    .map(<[u8]>::to_vec)
                    .collect::<Vec<_>>()
            })
            .collect()
    }

    /// Every `DATA` payload the host wrote, before it is split into lines.
    pub fn sent_payloads(&self) -> Vec<Vec<u8>> {
        self.sent.lock().unwrap().clone()
    }

    /// Reaches inside, for a test that wants it to misbehave.
    pub fn tweak<R>(&self, change: impl FnOnce(&mut FakeDevice) -> R) -> R {
        change(&mut self.fake.lock().unwrap())
    }
}

/// An ignition that is a data structure.
pub struct LoopbackLink {
    device: Arc<Mutex<FakeDevice>>,
    /// The most one message may carry, as the fake STATUS reports it.
    max_payload: u16,
    /// What the last `SELECT` named.
    selected: Option<Target>,
    /// Vendor requests, in the order they came.
    pub control: Vec<(u8, u16)>,
    /// Host-to-device bytes, still to be cut into messages.
    messages: pipe::Reassembler,
    /// The lines of the request being assembled.
    lines: nlip::LineAssembler,
    /// The device's replies, still to be cut into frames.
    replies: FrameReassembler,
    /// Packets waiting to be read back.
    inbound: VecDeque<Vec<u8>>,
    /// Shared with the handle, so a test can see what was written.
    sent: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl LoopbackLink {
    pub fn new(max_payload: u16) -> (Self, LoopbackDevice) {
        let mut device = FakeDevice::new(DEVICE_CAPACITY);
        // MCUboot's serial recovery has no os mcumgr-parameters command, so
        // the client has nothing to negotiate with and keeps one request per
        // write. The fake has to say so, or the loopback would be easier than
        // the board.
        device.buffers = None;
        let device = Arc::new(Mutex::new(device));
        let sent = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                device: Arc::clone(&device),
                sent: Arc::clone(&sent),
                max_payload,
                selected: None,
                control: Vec::new(),
                messages: pipe::Reassembler::default(),
                lines: nlip::LineAssembler::default(),
                replies: FrameReassembler::default(),
                inbound: VecDeque::new(),
            },
            LoopbackDevice { fake: device, sent },
        )
    }

    fn status(&self) -> Vec<u8> {
        let mut bytes = vec![0u8; STATUS_LEN];
        bytes[0] = STATUS_VERSION;
        bytes[1] = 1; // active
        bytes[2] = self.selected.unwrap_or_default().code();
        bytes[3] = 2; // left: mcuboot
        bytes[4] = 2; // right: mcuboot
        bytes[7] = 50; // kHz
        bytes[10..12].copy_from_slice(&self.max_payload.to_le_bytes());
        bytes[12] = 1;
        bytes
    }

    /// One request frame into the device, and its reply back out.
    fn deliver(&mut self, target: Target, frame: &[u8]) {
        {
            let mut device = self.device.lock().unwrap();
            device
                .write(frame)
                .expect("the in-memory device never fails a write");
            while let Some(fragment) = device.read(Duration::ZERO).unwrap() {
                self.replies.push(&fragment);
            }
        }
        while let Some((header, body)) = self
            .replies
            .take()
            .expect("the device frames its own replies")
        {
            let mut reply = header.encode().to_vec();
            reply.extend_from_slice(&body);
            self.queue(
                target,
                &nlip::encode_concat(&reply).expect("a reply that fits"),
            );
        }
    }

    /// Bytes back to the host, framed and cut into packets.
    fn queue(&mut self, target: Target, bytes: &[u8]) {
        let per_message = (self.max_payload as usize).clamp(1, pipe::MAX_PAYLOAD);
        for chunk in bytes.chunks(per_message) {
            let message = pipe::data(target, chunk).expect("a chunk within the limit");
            for packet in message.chunks(PACKET) {
                self.inbound.push_back(packet.to_vec());
            }
        }
    }

    /// An event back to the host, framed the same way.
    fn report(&mut self, target: Target, code: EventCode, detail: u16) {
        let event = Event { code, detail };
        let message = pipe::encode(pipe::TYPE_EVENT, target, &event.encode())
            .expect("three bytes is within the limit");
        for packet in message.chunks(PACKET) {
            self.inbound.push_back(packet.to_vec());
        }
    }
}

impl IgnitionLink for LoopbackLink {
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
        self.control.push((request, value));
        if request == crate::ignition::REQ_SELECT {
            self.selected = match value {
                1 => Some(Target::Left),
                2 => Some(Target::Right),
                _ => None,
            };
        }
        Ok(())
    }

    fn control_in(
        &mut self,
        request: u8,
        value: u16,
        length: usize,
    ) -> Result<Vec<u8>, WiredError> {
        self.control.push((request, value));
        let mut answer = match request {
            crate::ignition::REQ_STATUS => self.status(),
            crate::ignition::REQ_I2C_SCAN => vec![0u8; crate::ignition::I2C_SCAN_LEN],
            other => {
                return Err(WiredError::Request {
                    request: other,
                    reason: "the loopback does not carry this request".into(),
                })
            }
        };
        answer.truncate(length);
        Ok(answer)
    }

    fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
        self.messages.push(message);
        while let Some(raw) = self.messages.take_message()? {
            let (target, payload) = match raw.to_device()? {
                Outbound::Data { target, payload } => (target, payload),
                // Nothing buffers on this side, so there is nothing to push.
                Outbound::Flush { .. } => continue,
            };
            self.sent.lock().unwrap().push(payload.clone());
            let target = match target {
                Target::Selected => match self.selected {
                    Some(selected) => selected,
                    None => {
                        self.report(Target::Selected, EventCode::TargetNotSelected, 0);
                        continue;
                    }
                },
                named => named,
            };
            self.lines.push(&payload);
            while let Some(frame) = self.lines.take()? {
                self.deliver(target, &frame);
            }
        }
        Ok(())
    }

    fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
        Ok(self.inbound.pop_front().unwrap_or_default())
    }
}

/// A transport onto the in-memory device, over the whole wired stack.
pub struct LoopbackTransport {
    inner: Box<dyn SmpTransport>,
}

impl LoopbackTransport {
    /// A transport for one half, and the device behind it.
    ///
    /// `max_payload` is what the fake ignition reports in its STATUS, so a
    /// caller can shrink the pipe and watch the request size follow. The
    /// transport reads it back the way it would from a board, which puts the
    /// status parse on the path too.
    pub fn new(half: Half, max_payload: u16) -> (Self, LoopbackDevice) {
        let (link, device) = LoopbackLink::new(max_payload);
        let ignition = Ignition::new(link);
        let inner: Box<dyn SmpTransport> = match half {
            Half::Left => Box::new(LeftTransport::new(ignition).expect("the loopback answers")),
            Half::Right => Box::new(RightTransport::new(ignition).expect("the loopback answers")),
        };
        (Self { inner }, device)
    }

    /// The left half, over a pipe the size ignition reports today.
    pub fn left() -> (Self, LoopbackDevice) {
        Self::new(Half::Left, pipe::MAX_PAYLOAD as u16)
    }

    /// The right half, one line per message.
    pub fn right() -> (Self, LoopbackDevice) {
        Self::new(Half::Right, pipe::MAX_PAYLOAD as u16)
    }
}

impl SmpTransport for LoopbackTransport {
    fn mtu(&self) -> usize {
        self.inner.mtu()
    }

    fn write(&mut self, packet: &[u8]) -> Result<(), TransportError> {
        self.inner.write(packet)
    }

    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, TransportError> {
        self.inner.read(timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipe::Inbound;
    use zapp_smp::fake::Accept;
    use zapp_smp::{img, SmpClient, SmpError};

    fn client(half: Half) -> (SmpClient, LoopbackDevice) {
        let (transport, device) = LoopbackTransport::new(half, pipe::MAX_PAYLOAD as u16);
        (SmpClient::new(Box::new(transport)), device)
    }

    #[test]
    fn a_state_read_goes_all_the_way_down_and_comes_all_the_way_back() {
        for half in [Half::Left, Half::Right] {
            let (mut client, device) = client(half);
            let slots = client.image_state().unwrap();
            assert_eq!(slots.len(), 1, "{half}");
            assert_eq!(slots[0].version, "9.9.9");
            assert_eq!(device.requests(), vec![(img::GROUP, img::CMD_STATE)]);
        }
    }

    #[test]
    fn an_image_uploaded_through_the_whole_stack_is_the_image_that_went_in() {
        for half in [Half::Left, Half::Right] {
            let image: Vec<u8> = (0..9000u32).map(|byte| (byte % 251) as u8).collect();
            let (mut client, device) = client(half);
            client.upload(0, &image, &|_| {}).unwrap();
            assert_eq!(device.image(), image, "{half}");
        }
    }

    /// The one thing the codec cannot survive getting wrong, checked against
    /// the device rather than against the encoder: every line that reaches the
    /// far end is a whole number of base64 quanta and short enough to survive
    /// the receive ISR.
    #[test]
    fn every_line_a_whole_upload_puts_on_the_wire_obeys_both_limits() {
        for half in [Half::Left, Half::Right] {
            let image: Vec<u8> = (0..4000u32).map(|byte| (byte % 251) as u8).collect();
            let (mut client, device) = client(half);
            client.upload(0, &image, &|_| {}).unwrap();

            let lines = device.lines();
            assert!(lines.len() > 20, "{half}: only {} lines", lines.len());
            for line in &lines {
                assert!(line.ends_with(b"\n"), "{half}: a line with no newline");
                assert!(
                    line.len() <= nlip::MAX_LINE,
                    "{half}: a {}-byte line",
                    line.len()
                );
                let body = line.len() - 3;
                assert_eq!(body % 4, 0, "{half}: a {body}-character body");
            }
            // And the markers alternate the way a stream of frames does.
            assert!(lines[0].starts_with(&nlip::START));
        }
    }

    /// The right half's mailbox takes one line per message, and a transport
    /// that ran them together would still work here — so it is checked at the
    /// message boundary rather than at the line.
    #[test]
    fn the_right_half_never_gets_two_lines_in_one_message() {
        let (transport, device) = LoopbackTransport::new(Half::Right, pipe::MAX_PAYLOAD as u16);
        SmpClient::new(Box::new(transport))
            .upload(0, &vec![0x42; 3000], &|_| {})
            .unwrap();
        for payload in device.sent_payloads() {
            assert_eq!(
                payload.iter().filter(|byte| **byte == b'\n').count(),
                1,
                "{} bytes in one message",
                payload.len()
            );
        }
    }

    #[test]
    fn the_left_half_gets_a_whole_frame_in_one_message() {
        let (transport, device) = LoopbackTransport::new(Half::Left, pipe::MAX_PAYLOAD as u16);
        SmpClient::new(Box::new(transport))
            .upload(0, &vec![0x42; 3000], &|_| {})
            .unwrap();
        let payloads = device.sent_payloads();
        assert!(
            payloads
                .iter()
                .any(|payload| payload.iter().filter(|byte| **byte == b'\n').count() > 1),
            "no message carried more than one line"
        );
    }

    #[test]
    fn the_request_size_is_what_the_pipe_can_carry_and_nothing_larger() {
        for max_payload in [128u16, 256, 512, 1024] {
            let (transport, device) = LoopbackTransport::new(Half::Left, max_payload);
            let mut client = SmpClient::new(Box::new(transport));
            // No mcumgr-parameters command here, so this changes nothing.
            let limit = client.negotiate_request_limit();
            assert_eq!(limit, nlip::max_frame_in(max_payload as usize));

            let image: Vec<u8> = (0..3000u32).map(|byte| byte as u8).collect();
            client.upload(0, &image, &|_| {}).unwrap();
            assert_eq!(device.image(), image, "{max_payload} per message");
            assert!(
                device.largest_request() <= limit,
                "{} in {limit}",
                device.largest_request()
            );
        }
    }

    #[test]
    fn a_smaller_pipe_means_more_round_trips_and_the_same_image() {
        let image: Vec<u8> = (0..6000u32).map(|byte| (byte % 251) as u8).collect();
        let mut counts = Vec::new();
        for max_payload in [128u16, 1024] {
            let (transport, device) = LoopbackTransport::new(Half::Left, max_payload);
            SmpClient::new(Box::new(transport))
                .upload(0, &image, &|_| {})
                .unwrap();
            assert_eq!(device.image(), image);
            counts.push(device.requests().len());
        }
        assert!(counts[0] > counts[1], "{counts:?}");
    }

    #[test]
    fn a_device_that_takes_part_of_a_chunk_is_followed_the_same_as_over_ble() {
        let image: Vec<u8> = (0..2000u32).map(|byte| (byte % 249) as u8).collect();
        let (transport, device) = LoopbackTransport::left();
        device.tweak(|fake| fake.accept = Accept::AtMost(64));
        SmpClient::new(Box::new(transport))
            .upload(0, &image, &|_| {})
            .unwrap();
        assert_eq!(device.image(), image);
    }

    #[test]
    fn a_refusal_from_the_device_arrives_as_a_refusal_and_not_as_a_timeout() {
        let (transport, device) = LoopbackTransport::left();
        device.tweak(|fake| fake.fail_with = Some(9));
        let err = SmpClient::new(Box::new(transport))
            .upload(0, &[0x42; 1000], &|_| {})
            .unwrap_err();
        assert!(
            matches!(
                err,
                SmpError::Remote {
                    group: 1,
                    rc: 9,
                    ..
                }
            ),
            "{err:?}"
        );
    }

    #[test]
    fn marking_and_resetting_reach_the_device_through_the_pipe() {
        let (transport, device) = LoopbackTransport::left();
        let mut client = SmpClient::new(Box::new(transport));
        client.mark(&[0x11; 32], true).unwrap();
        client.reset().unwrap();
        assert_eq!(device.resets(), 1);
    }

    #[test]
    fn nothing_is_left_in_the_pipe_once_a_transaction_is_over() {
        let (link, _device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        let mut transport = LeftTransport::with_payload_budget(Ignition::new(link), 1024);
        transport
            .write(&[0x08, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xa0])
            .unwrap();
        let frame = transport.read(Duration::from_millis(50)).unwrap();
        assert!(frame.is_some());
        assert!(transport.read(Duration::from_millis(5)).unwrap().is_none());
    }

    #[test]
    fn data_for_a_target_that_was_never_selected_comes_back_as_an_event() {
        let (link, _device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        let mut ignition = Ignition::new(link);
        ignition
            .send(Target::Selected, &nlip::encode_concat(b"anything").unwrap())
            .unwrap();

        let mut messages = pipe::Reassembler::default();
        loop {
            let packet = ignition.receive(Duration::ZERO).unwrap();
            if packet.is_empty() {
                break;
            }
            messages.push(&packet);
        }
        assert_eq!(
            messages.take().unwrap(),
            Some(Inbound::Event {
                target: Target::Selected,
                event: Event {
                    code: EventCode::TargetNotSelected,
                    detail: 0
                }
            })
        );
    }

    #[test]
    fn selecting_a_half_first_sends_the_frame_where_it_was_pointed() {
        let (link, device) = LoopbackLink::new(pipe::MAX_PAYLOAD as u16);
        let mut ignition = Ignition::new(link);
        ignition.select(Half::Right).unwrap();
        assert_eq!(ignition.status().unwrap().target, Target::Right);

        let state_read = [0x08, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xa0];
        ignition
            .send(Target::Selected, &nlip::encode_concat(&state_read).unwrap())
            .unwrap();
        assert_eq!(device.requests(), vec![(img::GROUP, img::CMD_STATE)]);
    }
}
