//! The request/response loop, over any transport.

use std::time::{Duration, Instant};

use zapp_core::flash::FlashProgress;

use crate::img::{self, ImageSlot};
use crate::proto::{self, Header, Op, Reassembler, HEADER_LEN};
use crate::session::{self, Declaration, Status};
use crate::settings;
use crate::transport::SmpTransport;
use crate::{cbor, image, SmpError};

/// os_mgmt, and the two commands this client uses from it.
pub const OS_GROUP: u16 = 0;
pub const OS_CMD_RESET: u8 = 5;
pub const OS_CMD_MCUMGR_PARAMS: u8 = 6;

pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(15);

/// The most a request may ever be, whatever a device claims.
///
/// The header's length field is sixteen bits, so a frame larger than this
/// cannot be described at all.
pub const MAX_REQUEST: usize = HEADER_LEN + u16::MAX as usize;

/// How many replies in a row may leave the offset where it was.
///
/// A device that answers but never advances would otherwise be an upload that
/// never ends.
const MAX_STALLS: u32 = 3;

/// How many times an upload may halve its request size before giving up.
///
/// Enough to walk a 2.4 KiB request back down to one write.
const MAX_SHRINKS: u32 = 4;

/// Bench knob: milliseconds to sleep after every acknowledged upload chunk.
/// Unset, zero or unparsable means no pacing.
fn upload_pace() -> Option<Duration> {
    std::env::var("ZAPP_UPLOAD_PACE_MS")
        .ok()
        .and_then(|ms| ms.parse::<u64>().ok())
        .filter(|ms| *ms > 0)
        .map(Duration::from_millis)
}

/// The smallest request the `ZAPP_REQUEST_LIMIT` knob may ask for. Below this
/// a request cannot carry a chunk worth sending, and `img::chunk_size` returns
/// 0 for a [`SmpError::ChunkTooSmall`].
const MIN_REQUEST_LIMIT: usize = 64;

/// Bench knob: an upper bound on the request size, in bytes. May go below the
/// MTU, so one request becomes a short write. Unset, zero or unparsable leaves
/// the negotiated limit alone.
fn upload_request_limit() -> Option<usize> {
    std::env::var("ZAPP_REQUEST_LIMIT")
        .ok()
        .and_then(|bytes| bytes.parse::<usize>().ok())
        .filter(|bytes| *bytes > 0)
}

/// What a device says about the buffers it reassembles requests into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McumgrParams {
    /// The largest whole SMP frame it can put back together from a run of
    /// writes. The eight-byte header counts against it: Zephyr's own protocol
    /// documentation calls this "single SMP buffer size, this includes SMP
    /// header and CBOR payload", and its reassembly adds the header to the
    /// length field before comparing.
    pub buf_size: usize,
    /// How many of those buffers it has.
    ///
    /// Not used here, because this client keeps one request in flight. It is
    /// the other half of the throughput story: Nordic's own clients treat it
    /// as a pipelining window — `windowCapacity` on Android, `pipelineDepth`
    /// on iOS, both set to `buf_count - 1` by convention — and send that many
    /// requests without waiting for each reply. Nothing in the protocol
    /// promises that, since the pool is shared with responses, which is why it
    /// is a convention and not a rule.
    pub buf_count: usize,
}

pub struct SmpClient {
    channel: Box<dyn SmpTransport>,
    reassembler: Reassembler,
    seq: u8,
    /// Bytes one write puts on the wire: the ATT payload, header already off.
    mtu: usize,
    /// The largest request to build before fragmenting, header included. One
    /// MTU by default, so a request is one write; `negotiate_request_limit`
    /// raises it to whatever the device says it can reassemble.
    request_limit: usize,
    timeout: Duration,
}

impl SmpClient {
    pub fn new(channel: Box<dyn SmpTransport>) -> Self {
        let mtu = channel.mtu();
        Self {
            channel,
            reassembler: Reassembler::default(),
            seq: 0,
            mtu,
            request_limit: mtu,
            timeout: DEFAULT_TIMEOUT,
        }
    }

    /// Overrides what the platform said one write can carry.
    ///
    /// This is bytes of ATT payload, not the ATT MTU: a write of exactly the
    /// MTU is three bytes too long and the board never sees it.
    ///
    /// Takes the request size with it, so the default stays "one request, one
    /// write". Call `with_request_limit` after this to part the two.
    pub fn with_mtu(mut self, mtu: usize) -> Self {
        self.mtu = mtu;
        self.request_limit = mtu;
        self
    }

    /// Sets how large a request may get before it is fragmented.
    pub fn with_request_limit(mut self, limit: usize) -> Self {
        self.request_limit = limit.min(MAX_REQUEST);
        self
    }

    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    pub fn mtu(&self) -> usize {
        self.mtu
    }

    pub fn request_limit(&self) -> usize {
        self.request_limit
    }

    /// Sends one request and waits for the reply that belongs to it.
    pub fn transact(
        &mut self,
        op: Op,
        group: u16,
        command: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, SmpError> {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);

        let packet = proto::frame(op, group, command, seq, payload)?;
        // Whatever is still in the buffer belongs to a request that is over.
        // Feeding its tail to this one would decode as nonsense.
        if self.reassembler.pending() > 0 {
            log::debug!(
                "dropping {} bytes left over from an earlier request",
                self.reassembler.pending()
            );
            self.reassembler.reset();
        }
        // Seeing the exact bytes is the reason this client exists rather than a
        // phone app, so they are one RUST_LOG level away.
        log::trace!("--> {}", image::hex(&packet));
        for fragment in proto::fragments(&packet, self.mtu)? {
            self.channel.write(fragment)?;
        }

        let deadline = Instant::now() + self.timeout;
        loop {
            while let Some((header, body)) = self.reassembler.take()? {
                log::trace!("<-- {} {}", image::hex(&header.encode()), image::hex(&body));
                match self.matching(&header, op, group, command, seq) {
                    Ok(true) => return Ok(body),
                    Ok(false) => continue,
                    Err(err) => return Err(err),
                }
            }

            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(SmpError::Timeout(self.timeout));
            }
            match self.channel.read(left)? {
                Some(notification) => self.reassembler.push(&notification),
                None => return Err(SmpError::Timeout(self.timeout)),
            }
        }
    }

    /// Whether a decoded packet is the answer being waited for.
    ///
    /// A reply to an earlier request can still be in flight — one that timed
    /// out, or a notification that arrived while the request was being written
    /// — so a sequence number that is not this one is skipped rather than
    /// failed on. Anything with the right sequence number and the wrong group
    /// is a real disagreement about what was asked.
    fn matching(
        &self,
        header: &Header,
        op: Op,
        group: u16,
        command: u8,
        seq: u8,
    ) -> Result<bool, SmpError> {
        if header.seq != seq {
            log::debug!("ignoring a reply for seq {} (waiting on {seq})", header.seq);
            return Ok(false);
        }
        if header.group != group || header.command != command {
            return Err(SmpError::Mismatch {
                expected: (group, command),
                got: (header.group, header.command),
            });
        }
        if header.op != op.response() {
            log::debug!("reply op {:?}, expected {:?}", header.op, op.response());
        }
        Ok(true)
    }

    /// What the device says its request buffers are, if it will say.
    ///
    /// `Ok(None)` is a device that does not carry the command — it is optional,
    /// and mcumgr answers an unknown one with an error rather than silence.
    pub fn mcumgr_params(&mut self) -> Result<Option<McumgrParams>, SmpError> {
        let empty = cbor::encode(&cbor::map(vec![]));
        let reply = self.transact(Op::Read, OS_GROUP, OS_CMD_MCUMGR_PARAMS, &empty)?;
        match parse_params(&reply) {
            // "Command not supported" is the device refusing the question, not
            // failing at it: mcumgr answers an unknown command that way.
            Err(SmpError::Remote { rc, meaning, .. }) => {
                log::debug!("no mcumgr parameters on this board: {meaning} (rc {rc})");
                Ok(None)
            }
            other => other,
        }
    }

    /// Sizes requests from what the device can reassemble, and says what it
    /// settled on.
    ///
    /// Without this a request is one write, so an upload costs a round trip per
    /// two hundred-odd bytes — at a 15 ms connection interval that round trip
    /// *is* the transfer time, and the radio sits idle through most of it. A
    /// request that fills the device's reassembly buffer carries ten times as
    /// much image for the same round trip, and the fragmenter already knows how
    /// to spread one across writes.
    ///
    /// Nothing here is fatal. A board that does not answer, or answers that it
    /// has never heard of the command, keeps the one-write default: a slow
    /// upload rather than a failed one.
    ///
    /// A board that reports a buffer it cannot actually take is a real hazard
    /// and not a hypothetical: Zephyr's reassembly carries no sequence numbers
    /// and no checksum, so one fragment lost anywhere below GATT silently
    /// discards the whole request and the reply never comes. The more writes a
    /// request is spread over, the more of that exposure it buys. `upload`
    /// halves its way back down when that happens.
    pub fn negotiate_request_limit(&mut self) -> usize {
        match self.mcumgr_params() {
            Ok(Some(params)) => {
                if params.buf_size > HEADER_LEN {
                    self.request_limit = params.buf_size.min(MAX_REQUEST);
                    log::info!(
                        "the board reassembles {} bytes ({} buffers); {} per request, {} per write",
                        params.buf_size,
                        params.buf_count,
                        self.request_limit,
                        self.mtu
                    );
                } else {
                    log::warn!(
                        "the board claims a {}-byte buffer; keeping {}",
                        params.buf_size,
                        self.request_limit
                    );
                }
            }
            Ok(None) => log::info!(
                "the board does not report its buffers; one request per write ({})",
                self.request_limit
            ),
            Err(err) => log::warn!(
                "could not ask the board about its buffers ({err}); \
                 keeping {} per request",
                self.request_limit
            ),
        }
        self.request_limit
    }

    /// Every image the board is carrying, slot by slot.
    pub fn image_state(&mut self) -> Result<Vec<ImageSlot>, SmpError> {
        let empty = cbor::encode(&cbor::map(vec![]));
        let reply = self.transact(Op::Read, img::GROUP, img::CMD_STATE, &empty)?;
        img::parse_state(&reply)
    }

    /// Marks an image by hash: for test, or confirmed.
    pub fn mark(&mut self, hash: &[u8], confirm: bool) -> Result<Vec<ImageSlot>, SmpError> {
        let payload = img::state_write(hash, confirm);
        let reply = self.transact(Op::Write, img::GROUP, img::CMD_STATE, &payload)?;
        img::parse_state(&reply)
    }

    /// Reads a whitelisted setting's raw bytes.
    ///
    /// A name the board does not carry, or one it has never been written, comes
    /// back as a [`SmpError::Remote`] with the board's `ENOENT` in it, which a
    /// caller reads to tell "not provisioned" apart from a link that failed.
    pub fn settings_read(&mut self, name: &str) -> Result<Vec<u8>, SmpError> {
        let payload = settings::read_request(name);
        let reply = self.transact(Op::Read, settings::GROUP, settings::CMD_RW, &payload)?;
        settings::parse_read(&reply)
    }

    /// Writes a whitelisted setting.
    pub fn settings_write(&mut self, name: &str, value: &[u8]) -> Result<(), SmpError> {
        let payload = settings::write_request(name, value);
        let reply = self.transact(Op::Write, settings::GROUP, settings::CMD_RW, &payload)?;
        settings::parse_write(&reply)
    }

    /// Declares an update: both halves' images, by both hashes, and the build
    /// they came from.
    ///
    /// Only the left half serves this. Declaring the *same* update again while
    /// it is open is a renewal rather than a second session, so a host that
    /// reconnected calls this again rather than working out whether it has to;
    /// a different declaration while one is live is refused.
    pub fn session_open(&mut self, declaration: &Declaration) -> Result<Status, SmpError> {
        let payload = session::open_request(declaration);
        let reply = self.transact(Op::Write, session::GROUP, session::CMD_OPEN, &payload)?;
        session::parse_status(&reply)
    }

    /// What is declared, what is done, and what is next.
    ///
    /// Never refused, whether or not a session is open: it is how a host works
    /// out why it is being turned down, so gating it would leave a refused host
    /// with nowhere to look.
    pub fn session_status(&mut self) -> Result<Status, SmpError> {
        let payload = session::status_request();
        let reply = self.transact(Op::Read, session::GROUP, session::CMD_STATUS, &payload)?;
        session::parse_status(&reply)
    }

    /// Ends the session, and reports what it left behind.
    ///
    /// An id ends only that session; `None` ends whatever is open. Closing when
    /// there is nothing to close is not an error — the caller wanted no session
    /// and there is none.
    pub fn session_close(&mut self, id: Option<u32>) -> Result<Status, SmpError> {
        let payload = session::close_request(id);
        let reply = self.transact(Op::Write, session::GROUP, session::CMD_CLOSE, &payload)?;
        session::parse_status(&reply)
    }

    /// Writes an image into the board's spare slot.
    ///
    /// The device decides where the next request starts, every time: this loop
    /// only ever sends what the last reply asked for.
    pub fn upload(
        &mut self,
        image_number: u32,
        data: &[u8],
        on_progress: &dyn Fn(FlashProgress),
    ) -> Result<(), SmpError> {
        let total = u32::try_from(data.len()).map_err(|_| SmpError::ImageTooLarge(data.len()))?;
        if total == 0 {
            return Err(SmpError::EmptyImage);
        }
        let sha = image::sha256(data);

        let pace = upload_pace();
        if let Some(pace) = pace {
            log::debug!("pacing uploads by {} ms", pace.as_millis());
        }
        if let Some(limit) = upload_request_limit() {
            let capped = self.request_limit.min(limit).max(MIN_REQUEST_LIMIT);
            if capped != self.request_limit {
                self.request_limit = capped;
                log::debug!("request limit capped to {capped} bytes");
            }
        }

        let mut off = 0u32;
        let mut stalls = 0;
        let mut shrinks = 0;
        on_progress(FlashProgress::Writing {
            bytes_written: 0,
            total_bytes: data.len(),
        });

        while off < total {
            let start = off as usize;
            let size = img::chunk_size(
                image_number,
                off,
                total,
                Some(&sha),
                data.len() - start,
                self.request_limit,
            );
            if size == 0 {
                return Err(SmpError::ChunkTooSmall(self.request_limit));
            }

            let payload = img::upload_request(
                image_number,
                off,
                total,
                Some(&sha),
                &data[start..start + size],
            );
            let reply = match self.transact(Op::Write, img::GROUP, img::CMD_UPLOAD, &payload) {
                Ok(reply) => reply,
                // A board that reported a buffer it cannot actually fill says
                // nothing at all: the fragments land, the request is dropped
                // somewhere below the handlers, and no error comes back to
                // read. Halving is the only way to tell that apart from a
                // board that has gone away, and it costs one timeout.
                Err(SmpError::Timeout(waited))
                    if self.request_limit > self.mtu && shrinks < MAX_SHRINKS =>
                {
                    shrinks += 1;
                    self.request_limit = (self.request_limit / 2).max(self.mtu);
                    log::warn!(
                        "no reply to a {size}-byte chunk in {waited:?}; \
                         trying {} bytes per request",
                        self.request_limit
                    );
                    continue;
                }
                Err(err) => return Err(err),
            };
            let next = img::advance(off, size as u32, total, img::parse_upload(&reply)?.off)?;
            // mcumgr resumes an upload whose hash it already holds — and after
            // any swap the spare slot holds the image that was swapped out, so
            // flipping between two builds sends next to nothing. Worth a line:
            // otherwise a finished "upload" looks impossibly fast.
            if next > off.saturating_add(size as u32) {
                log::info!(
                    "the board already holds this image up to byte {next} of {total}; \
                     skipping ahead from {off}"
                );
            }

            if next <= off {
                stalls += 1;
                if stalls > MAX_STALLS {
                    return Err(SmpError::Stalled(off));
                }
            } else {
                stalls = 0;
            }
            off = next;

            on_progress(FlashProgress::Writing {
                bytes_written: off as usize,
                total_bytes: data.len(),
            });

            if let Some(pace) = pace {
                std::thread::sleep(pace);
            }
        }

        on_progress(FlashProgress::Complete);
        Ok(())
    }

    /// Reboots the board.
    ///
    /// The reply comes back before the reboot, but a device that reboots first
    /// is not an error worth failing on: the command did what was asked.
    pub fn reset(&mut self) -> Result<(), SmpError> {
        let empty = cbor::encode(&cbor::map(vec![]));
        match self.transact(Op::Write, OS_GROUP, OS_CMD_RESET, &empty) {
            Ok(reply) => proto::check_rc(&cbor::decode(&reply)?, OS_GROUP),
            Err(SmpError::Timeout(_)) => {
                log::info!("no reply to the reset; the board rebooted first");
                Ok(())
            }
            Err(err) => Err(err),
        }
    }
}

/// The buffer sizes out of an mcumgr-parameters response.
///
/// A reply with an error in it is the device saying no; a reply with neither
/// error nor sizes is one that answered the wrong question, and is treated the
/// same as not having the command at all.
fn parse_params(payload: &[u8]) -> Result<Option<McumgrParams>, SmpError> {
    let value = cbor::decode(payload)?;
    proto::check_rc(&value, OS_GROUP)?;

    let field = |name: &str| cbor::field(&value, name).and_then(cbor::as_u32);
    match (field("buf_size"), field("buf_count")) {
        (Some(buf_size), Some(buf_count)) => Ok(Some(McumgrParams {
            buf_size: buf_size as usize,
            buf_count: buf_count as usize,
        })),
        _ => {
            log::debug!("an mcumgr-parameters reply with no sizes in it");
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake::{Accept, FakeDevice};
    use ciborium::value::Value;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    fn client(device: FakeDevice) -> SmpClient {
        SmpClient::new(Box::new(device))
    }

    #[test]
    fn an_image_state_read_comes_back_slot_by_slot() {
        let device = FakeDevice::new(64);
        let requests = Arc::clone(&device.requests);
        let slots = client(device).image_state().unwrap();

        assert_eq!(slots.len(), 1);
        assert_eq!(slots[0].version, "9.9.9");
        assert!(slots[0].active);
        assert_eq!(*requests.lock().unwrap(), vec![(img::GROUP, img::CMD_STATE)]);
    }

    #[test]
    fn a_reply_longer_than_the_mtu_is_reassembled_from_its_notifications() {
        // 23 bytes is the smallest ATT MTU there is, so the image-state reply
        // arrives in a handful of pieces.
        let slots = client(FakeDevice::new(23 - 3)).image_state().unwrap();
        assert_eq!(slots[0].hash, vec![0x11; 32]);
    }

    #[test]
    fn an_upload_writes_the_image_the_device_ends_up_with() {
        let image: Vec<u8> = (0..5000u32).map(|byte| (byte % 251) as u8).collect();
        let seen = Mutex::new(Vec::new());

        client(FakeDevice::new(244))
            .upload(0, &image, &|progress| {
                if let FlashProgress::Writing { bytes_written, .. } = progress {
                    seen.lock().unwrap().push(bytes_written);
                }
            })
            .unwrap();
        let seen = seen.into_inner().unwrap();

        // Progress runs from nothing to the whole image, and never backwards.
        assert_eq!(seen.first(), Some(&0));
        assert_eq!(seen.last(), Some(&image.len()));
        assert!(seen.windows(2).all(|pair| pair[0] <= pair[1]), "{seen:?}");
    }

    /// The `len` and `sha` the first request told the device to expect.
    type DeclaredImage = (Option<u32>, Option<Vec<u8>>);
    type Declared = Arc<Mutex<DeclaredImage>>;

    /// The image the fake device is left holding after an upload.
    ///
    /// The client owns the channel, so the device is watched through a wrapper
    /// that copies out what it has written after every packet.
    struct Watched {
        inner: FakeDevice,
        landed: Arc<Mutex<Vec<u8>>>,
        declared: Declared,
    }

    impl SmpTransport for Watched {
        fn mtu(&self) -> usize {
            self.inner.mtu()
        }
        fn write(&mut self, value: &[u8]) -> Result<(), crate::TransportError> {
            let out = self.inner.write(value);
            self.landed.lock().unwrap().clone_from(&self.inner.image);
            *self.declared.lock().unwrap() =
                (self.inner.declared_len, self.inner.declared_sha.clone());
            out
        }
        fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, crate::TransportError> {
            self.inner.read(timeout)
        }
    }

    /// Runs an upload, and reports what the device ended up with and what the
    /// first request told it to expect.
    fn upload_to(device: FakeDevice, image: &[u8], limit: usize) -> (Vec<u8>, DeclaredImage) {
        let landed = Arc::new(Mutex::new(Vec::new()));
        let declared = Arc::new(Mutex::new((None, None)));
        let mut client = SmpClient::new(Box::new(Watched {
            inner: device,
            landed: Arc::clone(&landed),
            declared: Arc::clone(&declared),
        }))
        .with_request_limit(limit);

        client.upload(0, image, &|_| {}).unwrap();
        let written = landed.lock().unwrap().clone();
        let announced = declared.lock().unwrap().clone();
        (written, announced)
    }

    #[test]
    fn the_bytes_the_device_ends_up_with_are_the_bytes_that_went_in() {
        let image: Vec<u8> = (0..3333u32).map(|byte| (byte % 253) as u8).collect();
        assert_eq!(upload_to(FakeDevice::new(244), &image, 244).0, image);
    }

    #[test]
    fn an_upload_survives_a_device_that_takes_only_part_of_each_chunk() {
        // Every reply says it took 64 bytes, however much was sent: the loop
        // has to resend the rest rather than skipping over it.
        let image: Vec<u8> = (0..2000u32).map(|byte| (byte % 249) as u8).collect();
        let mut device = FakeDevice::new(244);
        device.accept = Accept::AtMost(64);
        assert_eq!(upload_to(device, &image, 244).0, image);
    }

    #[test]
    fn an_upload_survives_a_device_that_reports_no_offset_at_all() {
        let image: Vec<u8> = (0..1500u32).map(|byte| byte as u8).collect();
        let mut device = FakeDevice::new(128);
        device.accept = Accept::Silent;
        assert_eq!(upload_to(device, &image, 128).0, image);
    }

    #[test]
    fn an_upload_gives_up_on_a_device_that_never_advances() {
        let mut device = FakeDevice::new(244);
        device.accept = Accept::Stuck;
        let err = client(device)
            .upload(0, &vec![0x42; 1000], &|_| {})
            .unwrap_err();
        assert!(matches!(err, SmpError::Stalled(0)), "{err:?}");
    }

    #[test]
    fn a_request_larger_than_the_mtu_is_fragmented_and_still_understood() {
        // A 1 KiB request over a 100-byte MTU: eleven writes for one command.
        let image: Vec<u8> = (0..4096u32).map(|byte| (byte % 251) as u8).collect();
        assert_eq!(upload_to(FakeDevice::new(100), &image, 1024).0, image);
    }

    #[test]
    fn a_device_that_rejects_a_chunk_fails_the_upload() {
        let mut device = FakeDevice::new(244);
        device.fail_with = Some(9);
        let err = client(device)
            .upload(0, &vec![0x42; 1000], &|_| {})
            .unwrap_err();
        assert!(
            matches!(err, SmpError::Remote { group: 1, rc: 9, .. }),
            "{err:?}"
        );
    }

    #[test]
    fn the_first_request_announces_a_length_and_a_digest_the_device_can_check() {
        let image: Vec<u8> = (0..800u32).map(|byte| byte as u8).collect();
        let (_, (len, sha)) = upload_to(FakeDevice::new(244), &image, 244);
        assert_eq!(len, Some(800));
        assert_eq!(sha, Some(image::sha256(&image).to_vec()));
    }

    #[test]
    fn an_empty_image_is_refused_before_anything_is_written() {
        let err = client(FakeDevice::new(244))
            .upload(0, &[], &|_| {})
            .unwrap_err();
        assert!(matches!(err, SmpError::EmptyImage), "{err:?}");
    }

    #[test]
    fn an_mtu_too_small_for_any_image_data_is_refused() {
        let mut client = client(FakeDevice::new(244)).with_request_limit(24);
        let err = client.upload(0, &[0; 100], &|_| {}).unwrap_err();
        assert!(matches!(err, SmpError::ChunkTooSmall(24)), "{err:?}");
    }

    #[test]
    fn marking_an_image_reaches_the_device_as_a_state_write() {
        let device = FakeDevice::new(244);
        let requests = Arc::clone(&device.requests);
        client(device).mark(&[0x11; 32], true).unwrap();
        assert_eq!(*requests.lock().unwrap(), vec![(img::GROUP, img::CMD_STATE)]);
    }

    #[test]
    fn a_reset_reaches_the_device() {
        let device = FakeDevice::new(244);
        let resets = Arc::clone(&device.resets);
        client(device).reset().unwrap();
        assert_eq!(*resets.lock().unwrap(), 1);
    }

    #[test]
    fn a_reset_the_board_reboots_through_is_not_a_failure() {
        // Nothing ever answers: the board went down with the request.
        let mut client = SmpClient::new(Box::new(Deaf)).with_timeout(Duration::from_millis(10));
        assert!(client.reset().is_ok());
        // But a command that needs an answer still fails.
        assert!(matches!(
            client.image_state(),
            Err(SmpError::Timeout(_))
        ));
    }

    #[test]
    fn a_stale_reply_is_skipped_rather_than_taken_for_this_ones() {
        // The answer to a request that timed out, arriving late, followed by
        // the real one.
        struct Late {
            queued: VecDeque<Vec<u8>>,
        }
        impl SmpTransport for Late {
            fn mtu(&self) -> usize {
                244
            }
            fn write(&mut self, _value: &[u8]) -> Result<(), crate::TransportError> {
                Ok(())
            }
            fn read(
                &mut self,
                _timeout: Duration,
            ) -> Result<Option<Vec<u8>>, crate::TransportError> {
                Ok(self.queued.pop_front())
            }
        }

        let stale = proto::frame(
            Op::ReadRsp,
            img::GROUP,
            img::CMD_STATE,
            99,
            &cbor::encode(&cbor::map(vec![("rc", cbor::uint(3))])),
        )
        .unwrap();
        let real = proto::frame(
            Op::ReadRsp,
            img::GROUP,
            img::CMD_STATE,
            0,
            &cbor::encode(&cbor::map(vec![("images", Value::Array(vec![]))])),
        )
        .unwrap();

        let mut client = SmpClient::new(Box::new(Late {
            queued: VecDeque::from(vec![stale, real]),
        }));
        // The stale reply's rc would have failed this; it is not this request's.
        assert_eq!(client.image_state().unwrap(), Vec::new());
    }

    #[test]
    fn a_reply_about_a_different_command_is_an_error() {
        struct Confused;
        impl SmpTransport for Confused {
            fn mtu(&self) -> usize {
                244
            }
            fn write(&mut self, _value: &[u8]) -> Result<(), crate::TransportError> {
                Ok(())
            }
            fn read(
                &mut self,
                _timeout: Duration,
            ) -> Result<Option<Vec<u8>>, crate::TransportError> {
                Ok(Some(
                    proto::frame(Op::ReadRsp, 63, 7, 0, &[0xa0]).unwrap(),
                ))
            }
        }
        let err = SmpClient::new(Box::new(Confused)).image_state().unwrap_err();
        assert!(
            matches!(
                err,
                SmpError::Mismatch {
                    expected: (1, 0),
                    got: (63, 7)
                }
            ),
            "{err:?}"
        );
    }

    /// A channel that answers nothing, for the cases where the board does not.
    struct Deaf;
    impl SmpTransport for Deaf {
        fn mtu(&self) -> usize {
            249
        }
        fn write(&mut self, _value: &[u8]) -> Result<(), crate::TransportError> {
            Ok(())
        }
        fn read(&mut self, _timeout: Duration) -> Result<Option<Vec<u8>>, crate::TransportError> {
            Ok(None)
        }
    }

    #[test]
    fn a_request_never_outgrows_the_write_budget() {
        // Bug 1, in arithmetic: BlueZ reports the ATT MTU, so the budget is
        // three bytes less, and a request built to fill it has to fit in one
        // write rather than being three bytes over and dropped in silence.
        let att_mtu = 252usize;
        let budget = att_mtu - 3;

        let mut device = FakeDevice::new(budget);
        device.buffers = None;
        device.capacity = budget;
        let widest = Arc::clone(&device.largest);
        let image: Vec<u8> = (0..2000u32).map(|byte| byte as u8).collect();

        // The FakeDevice asserts every write is within the MTU on its way in.
        client(device).upload(0, &image, &|_| {}).unwrap();
        let widest = *widest.lock().unwrap();
        assert!(widest <= budget, "{widest} bytes in a {budget}-byte write");
        // And it is spending the budget, not hiding under it.
        assert_eq!(widest, budget);
    }

    #[test]
    fn the_request_size_comes_from_the_buffer_the_board_reports() {
        let mut client = client(FakeDevice::new(249));
        assert_eq!(client.request_limit(), 249);
        assert_eq!(client.negotiate_request_limit(), 2475);
        assert_eq!(client.mcumgr_params().unwrap().unwrap().buf_count, 4);
        // The write size is a property of the link and does not move with it.
        assert_eq!(client.mtu(), 249);
    }

    #[test]
    fn a_board_without_the_parameters_command_keeps_one_request_per_write() {
        let mut device = FakeDevice::new(249);
        device.buffers = None;
        let mut client = client(device);
        assert_eq!(client.mcumgr_params().unwrap(), None);
        assert_eq!(client.negotiate_request_limit(), 249);
    }

    #[test]
    fn a_board_that_will_not_answer_at_all_keeps_one_request_per_write() {
        // A failure to negotiate is not a failure to update.
        let mut client = SmpClient::new(Box::new(Deaf)).with_timeout(Duration::from_millis(10));
        assert!(client.mcumgr_params().is_err());
        assert_eq!(client.negotiate_request_limit(), 249);
    }

    #[test]
    fn a_nonsense_buffer_size_is_not_taken_at_its_word() {
        for buffers in [(0usize, 4usize), (8, 4)] {
            let mut device = FakeDevice::new(249);
            device.buffers = Some(buffers);
            assert_eq!(client(device).negotiate_request_limit(), 249, "{buffers:?}");
        }
        // And one that would not fit the header's length field is clamped.
        let mut device = FakeDevice::new(249);
        device.buffers = Some((1 << 20, 4));
        assert_eq!(client(device).negotiate_request_limit(), MAX_REQUEST);
        assert_eq!(
            SmpClient::new(Box::new(Deaf)).with_request_limit(usize::MAX).request_limit(),
            MAX_REQUEST
        );
    }

    #[test]
    fn a_negotiated_upload_fills_the_boards_buffer_and_lands_the_same_image() {
        // Bug 2: the same image, in a tenth of the round trips, still byte for
        // byte what went in.
        let image: Vec<u8> = (0..20_000u32).map(|byte| (byte % 251) as u8).collect();
        let device = FakeDevice::new(249);
        let widest = Arc::clone(&device.largest);
        let requests = Arc::clone(&device.requests);
        let landed = Arc::new(Mutex::new(Vec::new()));

        let mut client = SmpClient::new(Box::new(Watched {
            inner: device,
            landed: Arc::clone(&landed),
            declared: Arc::new(Mutex::new((None, None))),
        }));
        assert_eq!(client.negotiate_request_limit(), 2475);
        client.upload(0, &image, &|_| {}).unwrap();

        assert_eq!(*landed.lock().unwrap(), image);
        // Filling the buffer, and never overrunning it — the FakeDevice would
        // have panicked on that.
        assert!(*widest.lock().unwrap() > 2400, "{}", widest.lock().unwrap());

        let uploads = requests
            .lock()
            .unwrap()
            .iter()
            .filter(|request| **request == (img::GROUP, img::CMD_UPLOAD))
            .count();
        // 20 KiB in 2.4 KiB requests, rather than the 84 a 249-byte one needs.
        assert!(uploads <= 10, "{uploads} round trips");
    }

    #[test]
    fn a_board_that_claimed_more_than_it_can_take_is_followed_back_down() {
        // Seen on hardware: the board reports a 2475-byte buffer and then
        // answers nothing at all to a request that fills it. A slower upload
        // is the right outcome; a failed one is not.
        let mut device = FakeDevice::new(249);
        device.silent_over = Some(600);
        let landed = Arc::new(Mutex::new(Vec::new()));
        let image: Vec<u8> = (0..4000u32).map(|byte| (byte % 251) as u8).collect();

        let mut client = SmpClient::new(Box::new(Watched {
            inner: device,
            landed: Arc::clone(&landed),
            declared: Arc::new(Mutex::new((None, None))),
        }))
        .with_timeout(Duration::from_millis(10));
        assert_eq!(client.negotiate_request_limit(), 2475);

        client.upload(0, &image, &|_| {}).unwrap();
        assert_eq!(*landed.lock().unwrap(), image);
        // 2475 -> 1237 -> 618 -> 309, the first size the board will answer.
        assert_eq!(client.request_limit(), 309);
    }

    #[test]
    fn a_board_that_answers_nothing_at_all_still_fails_the_upload() {
        // Shrinking is bounded: a dead link must not turn into an endless
        // sequence of ever-smaller requests.
        let mut client = SmpClient::new(Box::new(Deaf))
            .with_request_limit(2475)
            .with_timeout(Duration::from_millis(5));
        let err = client.upload(0, &[0x42; 5000], &|_| {}).unwrap_err();
        assert!(matches!(err, SmpError::Timeout(_)), "{err:?}");
        assert_eq!(client.request_limit(), 249);
    }

    fn declaration() -> session::Declaration {
        session::Declaration {
            left: session::Declared {
                slot_hash: vec![0x11; 32],
                upload_hash: vec![0x22; 32],
            },
            right: session::Declared {
                slot_hash: vec![0x33; 32],
                upload_hash: vec![0x44; 32],
            },
            version: "0.1.9".into(),
            redo: false,
        }
    }

    #[test]
    fn declaring_an_update_reaches_the_board_as_a_group_67_write() {
        let device = FakeDevice::new(244);
        let requests = Arc::clone(&device.requests);
        let status = client(device).session_open(&declaration()).unwrap();

        assert!(status.open);
        assert_eq!(status.id, 1);
        assert_eq!(status.version, "0.1.9");
        assert_eq!(status.next, session::Next::Right);
        assert!(status.guard);
        assert_eq!(
            *requests.lock().unwrap(),
            vec![(session::GROUP, session::CMD_OPEN)]
        );
    }

    #[test]
    fn declaring_the_same_update_again_renews_it_rather_than_starting_another() {
        // What a host that lost its connection does, which is every reconnect
        // in a bundle run: the id has to stand, or the close at the end names
        // a session that is not the one it opened.
        let mut client = client(FakeDevice::new(244));
        let first = client.session_open(&declaration()).unwrap();
        let again = client.session_open(&declaration()).unwrap();
        assert_eq!(first.id, again.id);
        assert!(again.open);
    }

    #[test]
    fn asking_for_a_rewrite_renews_the_session_rather_than_colliding_with_it() {
        // `redo` is not part of the declaration, so turning it on mid-run must
        // not read as a second host installing something else.
        let mut client = client(FakeDevice::new(244));
        let first = client.session_open(&declaration()).unwrap();
        let forced = client
            .session_open(&session::Declaration {
                redo: true,
                ..declaration()
            })
            .unwrap();
        assert_eq!(forced.id, first.id);
        assert!(forced.open);
    }

    #[test]
    fn a_second_host_declaring_something_else_is_refused_in_zsas_group() {
        let mut client = client(FakeDevice::new(244));
        client.session_open(&declaration()).unwrap();

        let other = session::Declaration {
            version: "0.2.0".into(),
            ..declaration()
        };
        let err = client.session_open(&other).unwrap_err();
        // Group 66, not 67, and a code this build has a sentence for.
        assert!(
            matches!(err, SmpError::Remote { group: 66, rc: 16, .. }),
            "{err:?}"
        );
        assert!(err.to_string().contains("session"), "{err}");
    }

    #[test]
    fn a_status_read_answers_whether_or_not_anything_is_declared() {
        let device = FakeDevice::new(244);
        let requests = Arc::clone(&device.requests);
        let mut client = client(device);

        let closed = client.session_status().unwrap();
        assert!(!closed.open);
        assert_eq!(closed.next, session::Next::None);
        assert_eq!(closed.ttl_ms, 0);

        client.session_open(&declaration()).unwrap();
        let open = client.session_status().unwrap();
        assert!(open.open);
        assert_eq!(open.ttl_ms, 299_000);
        assert_eq!(open.right_tag, Some(vec![0xcd; 8]));

        assert_eq!(
            requests.lock().unwrap()[0],
            (session::GROUP, session::CMD_STATUS)
        );
    }

    #[test]
    fn closing_names_the_session_it_opened_and_leaves_nothing_behind() {
        let mut client = client(FakeDevice::new(244));
        let opened = client.session_open(&declaration()).unwrap();
        let closed = client.session_close(Some(opened.id)).unwrap();
        assert!(!closed.open);
        assert_eq!(closed.id, 0);
        // And closing again is not a failure: the caller wanted no session and
        // there is none.
        assert!(!client.session_close(Some(opened.id)).unwrap().open);
    }

    #[test]
    fn closing_a_session_that_is_not_this_ones_leaves_it_alone() {
        let mut client = client(FakeDevice::new(244));
        let opened = client.session_open(&declaration()).unwrap();
        let after = client.session_close(Some(opened.id + 99)).unwrap();
        assert!(after.open, "someone else's id closed this session");
        assert_eq!(after.id, opened.id);
    }

    #[test]
    fn a_development_build_says_it_is_not_enforcing() {
        let mut device = FakeDevice::new(244);
        device.guard = false;
        assert!(!client(device).session_status().unwrap().guard);
    }

    #[test]
    fn sequence_numbers_move_on_and_wrap() {
        let mut client = client(FakeDevice::new(244));
        for _ in 0..3 {
            client.image_state().unwrap();
        }
        assert_eq!(client.seq, 3);
        client.seq = 255;
        client.image_state().unwrap();
        assert_eq!(client.seq, 0);
    }
}
