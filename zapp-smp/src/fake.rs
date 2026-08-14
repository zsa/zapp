//! A Zephyr SMP device that only exists in memory.
//!
//! Compiled into the crate's own tests, and into any crate that turns the
//! `test-double` feature on — `zapp-wired` drives the whole wired stack
//! through it, codec and USB framing included, with no board attached.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use ciborium::value::Value;

use crate::client::{OS_CMD_MCUMGR_PARAMS, OS_CMD_RESET, OS_GROUP};
use crate::proto::{self, Header, Op, Reassembler, HEADER_LEN};
use crate::transport::SmpTransport;
use crate::{cbor, img, session, settings};

/// How the fake device answers an upload request.
#[derive(Debug, Clone, Copy)]
pub enum Accept {
    /// Everything it was sent.
    All,
    /// This many bytes of it, however much was sent.
    AtMost(u32),
    /// Nothing, and no `off` field either.
    Silent,
    /// Never moves, whatever it is sent.
    Stuck,
}

/// One state write: the hash it named, and whether it confirmed.
pub type Mark = (Vec<u8>, bool);

/// One slot of one image, as the fake lists it.
///
/// The flags are the ones MCUboot's serial recovery derives from the swap
/// status, so a listing here can stand for any of `NONE`, `TEST` or `REVERT`.
#[derive(Debug, Clone, Default)]
pub struct FakeSlot {
    pub slot: u32,
    pub hash: Vec<u8>,
    pub version: String,
    pub active: bool,
    pub confirmed: bool,
    pub pending: bool,
}

impl FakeSlot {
    /// A slot carrying `hash`, at the version the fake reports everywhere.
    pub fn new(slot: u32, hash: &[u8]) -> Self {
        Self {
            slot,
            hash: hash.to_vec(),
            version: "9.9.9".into(),
            ..Self::default()
        }
    }

    pub fn active(mut self) -> Self {
        self.active = true;
        self
    }

    pub fn confirmed(mut self) -> Self {
        self.confirmed = true;
        self
    }

    pub fn pending(mut self) -> Self {
        self.pending = true;
        self
    }

    fn encode(&self) -> Value {
        let mut entries = vec![
            ("slot", cbor::uint(self.slot)),
            ("version", Value::Text(self.version.clone())),
            ("hash", cbor::bytes(&self.hash)),
        ];
        for (set, name) in [
            (self.active, "active"),
            (self.confirmed, "confirmed"),
            (self.pending, "pending"),
        ] {
            if set {
                entries.push((name, Value::Bool(true)));
            }
        }
        cbor::map(entries)
    }
}

/// A Zephyr device that only exists in memory.
///
/// Reassembles requests the way the firmware does, answers them, and splits
/// the answers into notifications no larger than the MTU — which is what
/// makes it worth having: the fragmenting, the reassembly and the sequence
/// matching are all exercised on the way through.
pub struct FakeDevice {
    pub mtu: usize,
    pub incoming: Reassembler,
    pub outgoing: VecDeque<Vec<u8>>,
    pub image: Vec<u8>,
    pub declared_len: Option<u32>,
    pub declared_sha: Option<Vec<u8>>,
    pub accept: Accept,
    /// Set once the next reply should carry this instead of a success.
    pub fail_with: Option<i64>,
    /// What it really reassembles into. A request past this is dropped on
    /// the floor by the firmware, so here it is a failed test.
    pub capacity: usize,
    /// A request past this gets no answer, the way a board that reported
    /// more than it can take behaves.
    pub silent_over: Option<usize>,
    /// What it says when asked, as `(buf_size, buf_count)`. `None` is a
    /// build without the mcumgr-parameters command.
    pub buffers: Option<(usize, usize)>,
    /// The update session it is holding, as `(id, the OPEN request that
    /// declared it)`. The whole request is kept because that is what a
    /// re-declaration has to match to count as a renewal.
    pub session: Option<(u32, Vec<u8>)>,
    /// The id the next new session gets.
    pub next_session_id: u32,
    /// What it answers `next` with while a session is open.
    pub next_step: u32,
    /// False for a build with the guard compiled out.
    pub guard: bool,
    /// What slot 0 reports, read by read: each state read takes the next
    /// entry, and the last one is repeated once the list runs out. A board
    /// that reports something else after a restart is one entry more.
    ///
    /// Ignored once `listings` is set.
    pub slot_hashes: Vec<Vec<u8>>,
    /// Answered instead of any listing at all, byte for byte. `None` is the
    /// ordinary board; `Some` is one whose reply did not fit its buffer.
    pub listing_bytes: Option<Vec<u8>>,
    /// Whole multi-slot listings, read by read, the same way. Empty is the
    /// one-slot listing `slot_hashes` describes. A state *write* answers with
    /// the head of the list without taking it, the way `bs_set` replies with
    /// the listing it has just changed.
    pub listings: Vec<Vec<FakeSlot>>,
    /// The largest request that ever reached it whole.
    pub largest: Arc<Mutex<usize>>,
    pub requests: Arc<Mutex<Vec<(u16, u8)>>>,
    pub resets: Arc<Mutex<u32>>,
    /// The `image` number each upload announced, in order.
    pub uploads: Arc<Mutex<Vec<u32>>>,
    /// Every state write, as `(hash, confirm)`.
    pub marks: Arc<Mutex<Vec<Mark>>>,
    /// What `esb/config` currently holds. `None` is a board that was never
    /// provisioned, which a read answers with `ENOENT`.
    pub esb_config: Option<Vec<u8>>,
    /// A half that acks a settings write but does not take it — a dropped line
    /// on the bus — so the read-back does not match and the write is retried.
    pub esb_config_frozen: bool,
    /// Every value written to `esb/config`, in order.
    pub setting_writes: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl FakeDevice {
    pub fn new(mtu: usize) -> Self {
        Self {
            mtu,
            incoming: Reassembler::default(),
            outgoing: VecDeque::new(),
            image: Vec::new(),
            declared_len: None,
            declared_sha: None,
            accept: Accept::All,
            fail_with: None,
            capacity: 2475,
            silent_over: None,
            buffers: Some((2475, 4)),
            session: None,
            next_session_id: 1,
            next_step: 1,
            guard: true,
            slot_hashes: vec![vec![0x11; 32]],
            listing_bytes: None,
            listings: Vec::new(),
            largest: Arc::new(Mutex::new(0)),
            requests: Arc::new(Mutex::new(Vec::new())),
            resets: Arc::new(Mutex::new(0)),
            uploads: Arc::new(Mutex::new(Vec::new())),
            marks: Arc::new(Mutex::new(Vec::new())),
            esb_config: None,
            esb_config_frozen: false,
            setting_writes: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn answer(&mut self, header: Header, payload: Value) {
        let body = cbor::encode(&payload);
        self.answer_raw(header, &body);
    }

    /// The same, with the body handed over as it is.
    ///
    /// For the one thing a well-formed answer cannot stand in for: a board
    /// whose reply ran out of buffer half-way and is not valid CBOR at all.
    fn answer_raw(&mut self, header: Header, body: &[u8]) {
        let packet =
            proto::frame(header.op.response(), header.group, header.command, header.seq, body)
                .unwrap();
        for fragment in packet.chunks(self.mtu) {
            self.outgoing.push_back(fragment.to_vec());
        }
    }

    fn handle(&mut self, header: Header, body: &[u8]) {
        self.requests.lock().unwrap().push((header.group, header.command));

        // What the firmware's reassembly does with a frame too big for a
        // net_buf, minus the politeness of an error.
        let whole = HEADER_LEN + body.len();
        let mut largest = self.largest.lock().unwrap();
        *largest = (*largest).max(whole);
        drop(largest);
        assert!(
            whole <= self.capacity,
            "a {whole}-byte request into a {}-byte buffer",
            self.capacity
        );
        if self.silent_over.is_some_and(|most| whole > most) {
            return;
        }

        if let Some(rc) = self.fail_with.take() {
            self.answer(header, cbor::map(vec![("rc", Value::Integer(rc.into()))]));
            return;
        }

        match (header.group, header.command) {
            (OS_GROUP, OS_CMD_MCUMGR_PARAMS) => match self.buffers {
                Some((buf_size, buf_count)) => self.answer(
                    header,
                    cbor::map(vec![
                        ("buf_size", cbor::uint(buf_size as u32)),
                        ("buf_count", cbor::uint(buf_count as u32)),
                    ]),
                ),
                // MGMT_ERR_ENOTSUP, which is what a build without the
                // command answers.
                None => self.answer(header, cbor::map(vec![("rc", cbor::uint(8))])),
            },
            (OS_GROUP, OS_CMD_RESET) => {
                *self.resets.lock().unwrap() += 1;
                self.answer(header, cbor::map(vec![("rc", cbor::uint(0))]));
            }
            (img::GROUP, img::CMD_STATE) => {
                if header.op == Op::Write {
                    self.record_mark(body);
                }
                // A board that ran out of response buffer answers with what it
                // managed to write, which is not CBOR any decoder will take.
                if let Some(cut_short) = self.listing_bytes.clone() {
                    self.answer_raw(header, &cut_short);
                    return;
                }
                let state = self.list_images(header.op == Op::Read);
                self.answer(header, state);
            }
            (img::GROUP, img::CMD_UPLOAD) => self.upload(header, body),
            (session::GROUP, session::CMD_OPEN) => self.session_open(header, body),
            (session::GROUP, session::CMD_STATUS) => {
                let status = self.session_status();
                self.answer(header, status);
            }
            (session::GROUP, session::CMD_CLOSE) => self.session_close(header, body),
            (settings::GROUP, settings::CMD_RW) => self.settings_rw(header, body),
            _ => self.answer(header, cbor::map(vec![("rc", cbor::uint(8))])),
        }
    }

    /// The whitelisted settings handler both halves' recovery carries.
    ///
    /// One name, `esb/config`, one exact length. A name off the whitelist is
    /// `ENOENT`, a value of the wrong length is `EINVAL`, and a read of a
    /// setting that was never written is `ENOENT` — the same three answers the
    /// firmware's `boot_serial_settings_mgmt.c` gives.
    fn settings_rw(&mut self, header: Header, body: &[u8]) {
        let Ok(request) = cbor::decode(body) else {
            self.answer(header, cbor::map(vec![("rc", cbor::uint(3))])); // EINVAL
            return;
        };
        let name = cbor::field(&request, "name").and_then(cbor::as_text);
        if name != Some(settings::ESB_CONFIG_NAME) {
            self.answer(header, cbor::map(vec![("rc", cbor::uint(5))])); // ENOENT
            return;
        }

        if header.op == Op::Write {
            let value = cbor::field(&request, "val").and_then(cbor::as_bytes);
            match value {
                Some(value) if value.len() == settings::ESB_CONFIG_LEN => {
                    self.setting_writes.lock().unwrap().push(value.to_vec());
                    if !self.esb_config_frozen {
                        self.esb_config = Some(value.to_vec());
                    }
                    self.answer(header, cbor::map(vec![("rc", cbor::uint(0))]));
                }
                // A missing or wrong-length value is EINVAL, the whitelist's
                // one length check.
                _ => self.answer(header, cbor::map(vec![("rc", cbor::uint(3))])),
            }
            return;
        }

        match &self.esb_config {
            Some(value) => {
                let reply = cbor::map(vec![
                    ("val", cbor::bytes(value)),
                    ("rc", cbor::uint(0)),
                ]);
                self.answer(header, reply);
            }
            None => self.answer(header, cbor::map(vec![("rc", cbor::uint(5))])), // ENOENT
        }
    }

    /// One image listing. `advance` takes the next scripted entry.
    fn list_images(&mut self, advance: bool) -> Value {
        let slots: Vec<FakeSlot> = if self.listings.is_empty() {
            let hash = self.slot_hashes.first().cloned().unwrap_or_default();
            if advance && self.slot_hashes.len() > 1 {
                self.slot_hashes.remove(0);
            }
            // What MCUboot reports with no swap outstanding.
            vec![FakeSlot::new(0, &hash).active().confirmed()]
        } else {
            let listing = self.listings.first().cloned().unwrap_or_default();
            if advance && self.listings.len() > 1 {
                self.listings.remove(0);
            }
            listing
        };
        cbor::map(vec![(
            "images",
            Value::Array(slots.iter().map(FakeSlot::encode).collect()),
        )])
    }

    /// Records a state write, the hash and the confirm flag it carried.
    fn record_mark(&mut self, body: &[u8]) {
        let Ok(request) = cbor::decode(body) else {
            return;
        };
        let hash = cbor::field(&request, "hash")
            .and_then(cbor::as_bytes)
            .unwrap_or_default()
            .to_vec();
        let confirm = cbor::field(&request, "confirm")
            .and_then(cbor::as_bool)
            .unwrap_or(false);
        self.marks.lock().unwrap().push((hash, confirm));
    }

    /// What the firmware answers every group 67 command with.
    fn session_status(&self) -> Value {
        let (open, id) = match &self.session {
            Some((id, _)) => (true, *id),
            None => (false, 0),
        };
        let version = self
            .session
            .as_ref()
            .and_then(|(_, request)| cbor::decode(request).ok())
            .and_then(|request| {
                cbor::field(&request, "ver").and_then(cbor::as_text).map(str::to_string)
            })
            .unwrap_or_default();
        cbor::map(vec![
            ("open", Value::Bool(open)),
            ("id", cbor::uint(id)),
            ("ver", Value::Text(version)),
            ("next", cbor::uint(if open { self.next_step } else { 0 })),
            ("ldone", Value::Bool(false)),
            ("rdone", Value::Bool(self.next_step >= 2)),
            ("rseen", Value::Bool(true)),
            ("lhash", cbor::bytes(&[0x11; 32])),
            ("rtag", cbor::bytes(&[0xcd; 8])),
            ("ttl", cbor::uint(if open { 299_000 } else { 0 })),
            ("guard", Value::Bool(self.guard)),
        ])
    }

    fn session_open(&mut self, header: Header, body: &[u8]) {
        let declared = declaration_of(body);
        // A different declaration while one is live is the one refusal an
        // open has of its own: ZSA_MGMT_ERR_SESSION_BUSY.
        if self
            .session
            .as_ref()
            .is_some_and(|(_, held)| *held != declared)
        {
            self.answer(
                header,
                cbor::map(vec![(
                    "err",
                    cbor::map(vec![
                        ("group", cbor::uint(u32::from(crate::zsa_err::GROUP))),
                        ("rc", cbor::uint(16)),
                    ]),
                )]),
            );
            return;
        }
        if self.session.is_none() {
            self.session = Some((self.next_session_id, declared));
            self.next_session_id += 1;
        }
        let status = self.session_status();
        self.answer(header, status);
    }

    fn session_close(&mut self, header: Header, body: &[u8]) {
        let asked = cbor::decode(body)
            .ok()
            .and_then(|request| cbor::field(&request, "id").and_then(cbor::as_u32));
        let matches = self
            .session
            .as_ref()
            .is_some_and(|(id, _)| asked.is_none_or(|asked| asked == *id));
        if matches {
            self.session = None;
        }
        let status = self.session_status();
        self.answer(header, status);
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
            let image = cbor::field(&request, "image")
                .and_then(cbor::as_u32)
                .unwrap_or(0);
            self.uploads.lock().unwrap().push(image);
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

/// The declaration out of an OPEN request, with `redo` dropped.
///
/// `redo` asks for the declared images to be written again and is not part
/// of what was declared, so setting it on an otherwise identical statement
/// must read as the same session rather than as a second host.
pub fn declaration_of(body: &[u8]) -> Vec<u8> {
    let Ok(request) = cbor::decode(body) else {
        return body.to_vec();
    };
    let entries: Vec<(&str, Value)> = ["lslot", "lup", "rslot", "rup", "ver"]
        .into_iter()
        .filter_map(|key| cbor::field(&request, key).map(|value| (key, value.clone())))
        .collect();
    cbor::encode(&cbor::map(entries))
}

impl SmpTransport for FakeDevice {
    fn mtu(&self) -> usize {
        self.mtu
    }

    fn write(&mut self, value: &[u8]) -> Result<(), crate::TransportError> {
        assert!(value.len() <= self.mtu, "a write longer than the MTU");
        self.incoming.push(value);
        while let Some((header, body)) = self.incoming.take().unwrap() {
            self.handle(header, &body);
        }
        Ok(())
    }

    fn read(&mut self, _timeout: Duration) -> Result<Option<Vec<u8>>, crate::TransportError> {
        Ok(self.outgoing.pop_front())
    }
}

