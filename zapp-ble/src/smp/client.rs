//! The request/response loop, over any GATT channel.

use std::time::{Duration, Instant};

use zapp_core::flash::FlashProgress;

use super::img::{self, ImageSlot};
use super::proto::{self, Header, Op, Reassembler};
use super::{cbor, image, SmpError};
use crate::backend::GattChannel;

/// os_mgmt, and the one command this client uses from it.
pub const OS_GROUP: u16 = 0;
pub const OS_CMD_RESET: u8 = 5;

pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(15);

/// How many replies in a row may leave the offset where it was.
///
/// A device that answers but never advances would otherwise be an upload that
/// never ends.
const MAX_STALLS: u32 = 3;

pub struct SmpClient {
    channel: Box<dyn GattChannel>,
    reassembler: Reassembler,
    seq: u8,
    mtu: usize,
    /// The largest request to build before fragmenting. One MTU by default, so
    /// a request is one write and a reply is one notification.
    request_limit: usize,
    timeout: Duration,
}

impl SmpClient {
    pub fn new(channel: Box<dyn GattChannel>) -> Self {
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
    /// Takes the request size with it, so the default stays "one request, one
    /// write". Call `with_request_limit` after this to part the two.
    pub fn with_mtu(mut self, mtu: usize) -> Self {
        self.mtu = mtu;
        self.request_limit = mtu;
        self
    }

    /// Sets how large a request may get before it is fragmented.
    pub fn with_request_limit(mut self, limit: usize) -> Self {
        self.request_limit = limit;
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
            match self.channel.notification(left)? {
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

        let mut off = 0u32;
        let mut stalls = 0;
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
            let reply = self.transact(Op::Write, img::GROUP, img::CMD_UPLOAD, &payload)?;
            let next = img::advance(off, size as u32, total, img::parse_upload(&reply)?.off)?;

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

#[cfg(test)]
mod tests {
    use super::*;
    use ciborium::value::Value;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// How the fake device answers an upload request.
    #[derive(Debug, Clone, Copy)]
    enum Accept {
        /// Everything it was sent.
        All,
        /// This many bytes of it, however much was sent.
        AtMost(u32),
        /// Nothing, and no `off` field either.
        Silent,
        /// Never moves, whatever it is sent.
        Stuck,
    }

    /// A Zephyr device that only exists in this test.
    ///
    /// Reassembles requests the way the firmware does, answers them, and splits
    /// the answers into notifications no larger than the MTU — which is what
    /// makes it worth having: the fragmenting, the reassembly and the sequence
    /// matching are all exercised on the way through.
    struct FakeDevice {
        mtu: usize,
        incoming: Reassembler,
        outgoing: VecDeque<Vec<u8>>,
        image: Vec<u8>,
        declared_len: Option<u32>,
        declared_sha: Option<Vec<u8>>,
        accept: Accept,
        /// Set once the next reply should carry this instead of a success.
        fail_with: Option<i64>,
        requests: Arc<Mutex<Vec<(u16, u8)>>>,
        resets: Arc<Mutex<u32>>,
    }

    impl FakeDevice {
        fn new(mtu: usize) -> Self {
            Self {
                mtu,
                incoming: Reassembler::default(),
                outgoing: VecDeque::new(),
                image: Vec::new(),
                declared_len: None,
                declared_sha: None,
                accept: Accept::All,
                fail_with: None,
                requests: Arc::new(Mutex::new(Vec::new())),
                resets: Arc::new(Mutex::new(0)),
            }
        }

        fn answer(&mut self, header: Header, payload: Value) {
            let body = cbor::encode(&payload);
            let packet =
                proto::frame(header.op.response(), header.group, header.command, header.seq, &body)
                    .unwrap();
            for fragment in packet.chunks(self.mtu) {
                self.outgoing.push_back(fragment.to_vec());
            }
        }

        fn handle(&mut self, header: Header, body: &[u8]) {
            self.requests.lock().unwrap().push((header.group, header.command));
            if let Some(rc) = self.fail_with.take() {
                self.answer(header, cbor::map(vec![("rc", Value::Integer(rc.into()))]));
                return;
            }

            match (header.group, header.command) {
                (OS_GROUP, OS_CMD_RESET) => {
                    *self.resets.lock().unwrap() += 1;
                    self.answer(header, cbor::map(vec![("rc", cbor::uint(0))]));
                }
                (img::GROUP, img::CMD_STATE) => {
                    let state = cbor::map(vec![(
                        "images",
                        Value::Array(vec![cbor::map(vec![
                            ("slot", cbor::uint(0)),
                            ("version", Value::Text("9.9.9".into())),
                            ("hash", cbor::bytes(&[0x11; 32])),
                            ("active", Value::Bool(true)),
                        ])]),
                    )]);
                    self.answer(header, state);
                }
                (img::GROUP, img::CMD_UPLOAD) => self.upload(header, body),
                _ => self.answer(header, cbor::map(vec![("rc", cbor::uint(8))])),
            }
        }

        fn upload(&mut self, header: Header, body: &[u8]) {
            let request = cbor::decode(body).unwrap();
            let off = cbor::as_u32(cbor::field(&request, "off").unwrap()).unwrap();
            let data = cbor::as_bytes(cbor::field(&request, "data").unwrap()).unwrap();

            if off == 0 {
                self.image.clear();
                self.declared_len = cbor::field(&request, "len").and_then(cbor::as_u32);
                self.declared_sha = cbor::field(&request, "sha")
                    .and_then(cbor::as_bytes)
                    .map(<[u8]>::to_vec);
            }
            assert_eq!(off as usize, self.image.len(), "the device was sent a gap");

            let taken = match self.accept {
                Accept::All => data.len() as u32,
                Accept::AtMost(most) => most.min(data.len() as u32),
                Accept::Silent => data.len() as u32,
                Accept::Stuck => 0,
            };
            self.image.extend_from_slice(&data[..taken as usize]);

            let mut reply = vec![("rc", cbor::uint(0))];
            if !matches!(self.accept, Accept::Silent) {
                reply.push(("off", cbor::uint(self.image.len() as u32)));
            }
            self.answer(header, cbor::map(reply));
        }
    }

    impl GattChannel for FakeDevice {
        fn mtu(&self) -> usize {
            self.mtu
        }

        fn read(&mut self) -> Result<Vec<u8>, crate::BleError> {
            Ok(Vec::new())
        }

        fn write(&mut self, value: &[u8]) -> Result<(), crate::BleError> {
            assert!(value.len() <= self.mtu, "a write longer than the MTU");
            self.incoming.push(value);
            while let Some((header, body)) = self.incoming.take().unwrap() {
                self.handle(header, &body);
            }
            Ok(())
        }

        fn notification(&mut self, _timeout: Duration) -> Result<Option<Vec<u8>>, crate::BleError> {
            Ok(self.outgoing.pop_front())
        }
    }

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

    impl GattChannel for Watched {
        fn mtu(&self) -> usize {
            self.inner.mtu()
        }
        fn read(&mut self) -> Result<Vec<u8>, crate::BleError> {
            self.inner.read()
        }
        fn write(&mut self, value: &[u8]) -> Result<(), crate::BleError> {
            let out = self.inner.write(value);
            self.landed.lock().unwrap().clone_from(&self.inner.image);
            *self.declared.lock().unwrap() =
                (self.inner.declared_len, self.inner.declared_sha.clone());
            out
        }
        fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, crate::BleError> {
            self.inner.notification(timeout)
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
        struct Deaf;
        impl GattChannel for Deaf {
            fn mtu(&self) -> usize {
                244
            }
            fn read(&mut self) -> Result<Vec<u8>, crate::BleError> {
                Ok(Vec::new())
            }
            fn write(&mut self, _value: &[u8]) -> Result<(), crate::BleError> {
                Ok(())
            }
            fn notification(
                &mut self,
                _timeout: Duration,
            ) -> Result<Option<Vec<u8>>, crate::BleError> {
                Ok(None)
            }
        }
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
        impl GattChannel for Late {
            fn mtu(&self) -> usize {
                244
            }
            fn read(&mut self) -> Result<Vec<u8>, crate::BleError> {
                Ok(Vec::new())
            }
            fn write(&mut self, _value: &[u8]) -> Result<(), crate::BleError> {
                Ok(())
            }
            fn notification(
                &mut self,
                _timeout: Duration,
            ) -> Result<Option<Vec<u8>>, crate::BleError> {
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
        impl GattChannel for Confused {
            fn mtu(&self) -> usize {
                244
            }
            fn read(&mut self) -> Result<Vec<u8>, crate::BleError> {
                Ok(Vec::new())
            }
            fn write(&mut self, _value: &[u8]) -> Result<(), crate::BleError> {
                Ok(())
            }
            fn notification(
                &mut self,
                _timeout: Duration,
            ) -> Result<Option<Vec<u8>>, crate::BleError> {
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
