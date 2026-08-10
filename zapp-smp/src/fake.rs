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
use crate::proto::{self, Header, Reassembler, HEADER_LEN};
use crate::transport::SmpTransport;
use crate::{cbor, img, session};

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
    /// The largest request that ever reached it whole.
    pub largest: Arc<Mutex<usize>>,
    pub requests: Arc<Mutex<Vec<(u16, u8)>>>,
    pub resets: Arc<Mutex<u32>>,
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
            largest: Arc::new(Mutex::new(0)),
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
            (session::GROUP, session::CMD_OPEN) => self.session_open(header, body),
            (session::GROUP, session::CMD_STATUS) => {
                let status = self.session_status();
                self.answer(header, status);
            }
            (session::GROUP, session::CMD_CLOSE) => self.session_close(header, body),
            _ => self.answer(header, cbor::map(vec![("rc", cbor::uint(8))])),
        }
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

