//! Ignition's vendor interface: the control requests, and the bulk pipe under
//! them.
//!
//! Everything the host asks of the bootloader itself — open a session, pick a
//! half, read what it thinks is going on — is a vendor control request on
//! interface 1. The frames themselves go over the bulk endpoints, framed by
//! [`crate::pipe`].
//!
//! [`IgnitionLink`] is the whole of the I/O, so [`Ignition`] and everything
//! built on it can be driven with no device present.

use std::time::Duration;

use zapp_smp::Half;

use crate::pipe::{self, Target};
use crate::WiredError;

/// ZSA, and the Voyager 2 running ignition.
pub const VID: u16 = 0x3297;
pub const PID: u16 = 0x0792;

/// The interface the vendor requests are addressed to and the bulk endpoints
/// live on. Interface 0 is the DFU function.
pub const INTERFACE: u8 = 1;

/// Open a session. `wValue` is a bitmap of the halves it covers.
pub const REQ_BEGIN: u8 = 0x80;
/// Point the pipe at one half. `wValue` is 1 for left, 2 for right.
pub const REQ_SELECT: u8 = 0x81;
/// Close the session and put the hardware back.
pub const REQ_END: u8 = 0x82;
/// Read [`Status`].
pub const REQ_STATUS: u8 = 0x83;
/// Let go of the left half's MD pin, so it boots its application rather than
/// re-entering recovery.
pub const REQ_RELEASE_MD: u8 = 0x84;
/// Reset the target named in `wValue`.
pub const REQ_RESET_TARGET: u8 = 0x85;
/// Ask the running half named in `wValue` to confirm the image it is executing.
/// The data stage carries [`CONFIRM_HASH_LEN`] bytes of the hash the host
/// expects it to be running. Queued rather than blocking, and refused unless
/// the session is idle.
pub const REQ_CONFIRM: u8 = 0x86;
/// Read [`ConfirmResult`]. Polled after [`REQ_CONFIRM`].
pub const REQ_CONFIRM_RESULT: u8 = 0x87;
/// Diagnostic: what answers on the TRRS I2C bus.
pub const REQ_I2C_SCAN: u8 = 0x90;
/// Read [`IgnitionVersion`]: the version and the commit the image was built
/// from. Ignition 2.0.3 and later; older ones stall it.
pub const REQ_VERSION: u8 = 0x92;

/// Bytes in a VERSION response.
pub const VERSION_LEN: usize = 16;

/// The only VERSION layout this build can read; byte 0 of the response.
pub const VERSION_LAYOUT: u8 = 1;

/// Bytes in a STATUS response.
pub const STATUS_LEN: usize = 16;

/// Bytes in a CONFIRM_RESULT response.
pub const CONFIRM_RESULT_LEN: usize = 36;

/// Bytes of hash a confirm query names an image by.
///
/// The images on this product are hashed with SHA-512 and MCUboot publishes all
/// 64 bytes, but the query and its reply both carry only the leading 32
/// (`ZSA_CONFIRM_HASH_LEN`, and the firmware compares exactly that prefix), so
/// the whole exchange fits one 64-byte control packet.
pub const CONFIRM_HASH_LEN: usize = 32;

/// The only STATUS layout this build can read.
///
/// Byte 0 is the version of the struct and not part of it, precisely so that a
/// firmware that rearranges the rest is refused instead of misread — every
/// field after it is a number that would otherwise decode as a plausible
/// wrong answer.
pub const STATUS_VERSION: u8 = 2;

/// Bytes asked for in an I2C_SCAN response.
pub const I2C_SCAN_LEN: usize = 16;

const CONTROL_TIMEOUT: Duration = Duration::from_secs(2);

/// How long one message may take to get onto the bulk endpoint.
///
/// Not a measure of USB, which is instant here, but of how long the bridge can
/// refuse to take it. Ignition NAKs the endpoint while its ring is full, and
/// the ring drains at whatever the target runs at — for the right half a
/// 400 kHz I2C bus with MCUboot erasing a sector behind it, which stops the
/// mailbox accepting anything for as long as it takes.
///
/// It has to be generous, because a timeout here cannot be retried: a
/// cancelled bulk transfer may have delivered some of its bytes, and sending
/// them again would corrupt the stream. So running it out ends the link rather
/// than failing one message — [`WiredError::WriteStalled`], which reaches a
/// caller as a closed transport and reads as the half that stopped answering,
/// the same as a bridge event does.
const BULK_WRITE_TIMEOUT: Duration = Duration::from_secs(30);

/// Which halves a session covers. Bit 0 is left, bit 1 is right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Halves(u16);

impl Halves {
    pub const LEFT: Self = Self(0b01);
    pub const RIGHT: Self = Self(0b10);
    pub const BOTH: Self = Self(0b11);

    pub fn of(half: Half) -> Self {
        match half {
            Half::Left => Self::LEFT,
            Half::Right => Self::RIGHT,
        }
    }

    pub fn bits(self) -> u16 {
        self.0
    }

    pub fn contains(self, half: Half) -> bool {
        self.0 & Self::of(half).0 != 0
    }
}

/// Where the bootloader thinks the session is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    Idle,
    Active,
    Fault,
    /// The right-entry sequence is running; STATUS is polled until this moves.
    Pending,
    Unrecognised(u8),
}

impl SessionState {
    fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Idle,
            1 => Self::Active,
            2 => Self::Fault,
            3 => Self::Pending,
            other => Self::Unrecognised(other),
        }
    }
}

/// What a half is running, as far as the bootloader can tell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetState {
    Unknown,
    App,
    McuBoot,
    /// Answered once and has not since.
    Stale,
    Unrecognised(u8),
}

impl TargetState {
    fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Unknown,
            1 => Self::App,
            2 => Self::McuBoot,
            3 => Self::Stale,
            other => Self::Unrecognised(other),
        }
    }
}

/// Which Ignition is running: its version, and the commit it was built from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IgnitionVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
    /// The abbreviated SHA of the commit, as `git rev-parse --short` gave it.
    pub sha: String,
    /// Built from a tree with uncommitted changes, so `sha` is only where it
    /// started from.
    pub dirty: bool,
    /// A `make DIAG=1` build, which keeps a post-mortem record. Never a release.
    pub diag: bool,
}

impl IgnitionVersion {
    /// A VERSION response, checked before it is believed.
    pub fn parse(bytes: &[u8]) -> Result<Self, WiredError> {
        if bytes.len() < VERSION_LEN {
            return Err(WiredError::ShortVersion { got: bytes.len() });
        }
        if bytes[0] != VERSION_LAYOUT {
            return Err(WiredError::VersionLayout { got: bytes[0] });
        }
        let sha = &bytes[5..VERSION_LEN];
        let end = sha.iter().position(|&b| b == 0).unwrap_or(sha.len());
        Ok(Self {
            major: bytes[1],
            minor: bytes[2],
            patch: bytes[3],
            sha: String::from_utf8_lossy(&sha[..end]).into_owned(),
            dirty: bytes[4] & 0x01 != 0,
            diag: bytes[4] & 0x02 != 0,
        })
    }
}

impl std::fmt::Display for IgnitionVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{} ({}", self.major, self.minor, self.patch, self.sha)?;
        if self.dirty {
            f.write_str("-dirty")?;
        }
        if self.diag {
            f.write_str(", diag")?;
        }
        f.write_str(")")
    }
}

/// The version an Ignition puts in its device descriptor, for one too old to
/// answer [`REQ_VERSION`]: BCD `0xJJMN`, so 0x0203 is 2.0.3. Every Voyager 2
/// Ignition before 2.0.3 reports 0x0200 whatever it is.
pub fn bcd_version(bcd: u16) -> String {
    let digit = |shift: u16| (bcd >> shift) & 0xf;
    format!("{}.{}.{}", digit(12) * 10 + digit(8), digit(4), digit(0))
}

/// A STATUS response, version 2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub state: SessionState,
    pub target: Target,
    pub left_state: TargetState,
    pub right_state: TargetState,
    /// `SESSION_ERR_*`, zero for none.
    pub error: u16,
    /// What the TRRS bus is actually clocking at.
    pub i2c_khz: u16,
    /// How many times the bus has needed recovering since the session opened.
    pub recover_count: u8,
    /// Bytes the bridge dropped on the way back from the target. Non-zero
    /// means a stalled transfer with a cause.
    pub rx_drops: u8,
    /// The most one pipe message may carry, this firmware's number rather than
    /// the protocol's ceiling.
    pub max_payload: u16,
    /// The pipe framing version, which moves independently of the struct.
    pub proto_version: u8,
}

impl Status {
    /// A STATUS response, checked before it is believed.
    pub fn parse(bytes: &[u8]) -> Result<Self, WiredError> {
        if bytes.len() < STATUS_LEN {
            return Err(WiredError::ShortStatus { got: bytes.len() });
        }
        if bytes[0] != STATUS_VERSION {
            return Err(WiredError::StatusVersion { got: bytes[0] });
        }
        Ok(Self {
            state: SessionState::from_code(bytes[1]),
            target: Target::from_code(bytes[2]).ok_or(pipe::PipeError::UnknownTarget(bytes[2]))?,
            left_state: TargetState::from_code(bytes[3]),
            right_state: TargetState::from_code(bytes[4]),
            error: u16::from_le_bytes([bytes[5], bytes[6]]),
            i2c_khz: i2c_khz(bytes),
            recover_count: bytes[8],
            rx_drops: bytes[9],
            max_payload: u16::from_le_bytes([bytes[10], bytes[11]]),
            proto_version: bytes[12],
        })
    }

    /// What one pipe message may carry, kept inside the protocol's ceiling.
    ///
    /// A firmware that reports nothing, or more than the framing allows, gets
    /// the ceiling rather than a failed session.
    pub fn payload_budget(&self) -> usize {
        let claimed = self.max_payload as usize;
        if claimed == 0 || claimed > pipe::MAX_PAYLOAD {
            log::warn!(
                "ignition reports a {claimed}-byte payload limit; using {}",
                pipe::MAX_PAYLOAD
            );
            return pipe::MAX_PAYLOAD;
        }
        claimed
    }

    pub fn state_of(&self, half: Half) -> TargetState {
        match half {
            Half::Left => self.left_state,
            Half::Right => self.right_state,
        }
    }
}

/// The bus speed, from the 16-bit field at `[13..14]`.
///
/// Byte 7 held it until 400 kHz stopped fitting in a byte, and a firmware old
/// enough to leave the wide field zeroed still has a speed that fits there. A
/// new firmware saturates byte 7 rather than wrapping it, so preferring the
/// wide field is what keeps the two readings from disagreeing.
fn i2c_khz(bytes: &[u8]) -> u16 {
    match u16::from_le_bytes([bytes[13], bytes[14]]) {
        0 => bytes[7] as u16,
        wide => wide,
    }
}

// ---------------------------------------------------------------------------
// The confirm tunnel
// ---------------------------------------------------------------------------

/// How far along ignition's queued confirm exchange is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfirmState {
    Idle,
    Busy,
    Done,
    Unrecognised(u8),
}

impl ConfirmState {
    fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Idle,
            1 => Self::Busy,
            2 => Self::Done,
            other => Self::Unrecognised(other),
        }
    }
}

/// Which job a [`REQ_CONFIRM`] asks of the half.
///
/// Carried in the high byte of `wValue`; the low byte stays the half (1 left,
/// 2 right) and `wIndex` is left as the interface recipient it already is.
/// READ reports the running hash and writes nothing; COMMIT writes the trailer
/// if the running hash still matches the expected one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfirmMode {
    Read = 0,
    Commit = 1,
}

/// How the exchange itself went, which is ignition's answer rather than the
/// target's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfirmRc {
    Ok,
    /// The query went out and nothing came back inside ignition's own 2 s.
    Timeout,
    /// The query never reached the target at all.
    Transport,
    /// It answered with the wrong shape.
    BadReply,
    Unrecognised(u8),
}

impl ConfirmRc {
    fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Ok,
            1 => Self::Timeout,
            2 => Self::Transport,
            3 => Self::BadReply,
            other => Self::Unrecognised(other),
        }
    }
}

/// What the target did with the query. Only meaningful once the exchange is
/// [`ConfirmState::Done`] with [`ConfirmRc::Ok`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfirmStatus {
    /// COMMIT: the hash matched and the trailer is now confirmed.
    Ok,
    /// The hash matched and it was already confirmed.
    Already,
    /// A different image is running; nothing was written.
    Mismatch,
    /// It could not read the hash of what it is running.
    Unreadable,
    /// COMMIT: the hash matched and the trailer write failed.
    WriteFail,
    /// READ: it is running the reported image, not confirmed, nothing written.
    Running,
    Unrecognised(u8),
}

impl ConfirmStatus {
    fn from_code(code: u8) -> Self {
        match code {
            0x00 => Self::Ok,
            0x01 => Self::Already,
            0x02 => Self::Mismatch,
            0x03 => Self::Unreadable,
            0x04 => Self::WriteFail,
            0x05 => Self::Running,
            other => Self::Unrecognised(other),
        }
    }

    /// Whether the target is running the image that was named and keeping it.
    pub fn kept(self) -> bool {
        matches!(self, Self::Ok | Self::Already)
    }
}

/// One read of ignition's confirm result block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfirmResult {
    pub state: ConfirmState,
    pub rc: ConfirmRc,
    pub status: ConfirmStatus,
    /// Which half was asked, echoed back. `None` when nothing has been.
    pub target: Option<Half>,
    /// The leading [`CONFIRM_HASH_LEN`] bytes of what the target reports it is
    /// running, all zero when it never answered.
    pub hash: Vec<u8>,
}

impl ConfirmResult {
    /// A CONFIRM_RESULT response, checked before it is believed.
    ///
    /// No version byte to check, unlike [`Status`]: every field is an
    /// enumeration this build carries an `Unrecognised` arm for, so a firmware
    /// that adds a code is read rather than refused.
    pub fn parse(bytes: &[u8]) -> Result<Self, WiredError> {
        if bytes.len() < CONFIRM_RESULT_LEN {
            return Err(WiredError::ShortConfirm { got: bytes.len() });
        }
        Ok(Self {
            state: ConfirmState::from_code(bytes[0]),
            rc: ConfirmRc::from_code(bytes[1]),
            status: ConfirmStatus::from_code(bytes[2]),
            target: half_of_value(bytes[3] as u16),
            hash: bytes[4..4 + CONFIRM_HASH_LEN].to_vec(),
        })
    }

    /// Whether ignition has finished with the exchange, however it went.
    pub fn done(&self) -> bool {
        self.state == ConfirmState::Done
    }
}

/// The whole of the I/O against ignition.
///
/// Everything above this is arithmetic, which is the point: the codec, the
/// framing and the session logic are all exercised against an implementation
/// that is not USB.
pub trait IgnitionLink: Send {
    /// A vendor request with no data stage.
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError>;

    /// A vendor request that writes `data` in its data stage.
    ///
    /// [`REQ_CONFIRM`] is the only one, so this is defaulted rather than
    /// required: the links that never carry it — the in-memory ones, and the
    /// recorders in these tests — say so instead of quietly sending the request
    /// with its payload dropped, which the bootloader would stall anyway.
    /// Anything wrapping a link has to forward it; [`Retrying`] does.
    fn control_out_data(
        &mut self,
        request: u8,
        value: u16,
        data: &[u8],
    ) -> Result<(), WiredError> {
        let _ = (value, data);
        Err(WiredError::Request {
            request,
            reason: "this link carries no data stage".into(),
        })
    }

    /// A vendor request that reads `length` bytes back.
    fn control_in(&mut self, request: u8, value: u16, length: usize)
        -> Result<Vec<u8>, WiredError>;

    /// One message onto the bulk OUT endpoint.
    fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError>;

    /// Whatever the bulk IN endpoint has, or an empty vector once `timeout`
    /// elapses. Message boundaries are not preserved and must not be relied
    /// on: that is [`pipe::Reassembler`]'s job.
    fn bulk_in(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError>;
}

/// `wValue` for the requests that name one half: 1 for left, 2 for right.
///
/// Not the bitmap BEGIN takes — the same two halves are numbered one way for a
/// set and another way for a choice.
fn half_value(half: Half) -> u16 {
    match half {
        Half::Left => 1,
        Half::Right => 2,
    }
}

/// The same numbering, read back: which half a firmware field names.
fn half_of_value(value: u16) -> Option<Half> {
    match value {
        1 => Some(Half::Left),
        2 => Some(Half::Right),
        _ => None,
    }
}

/// Ignition's vendor interface, over any link.
pub struct Ignition<L: IgnitionLink> {
    link: L,
}

impl<L: IgnitionLink> Ignition<L> {
    pub fn new(link: L) -> Self {
        Self { link }
    }

    pub fn link_mut(&mut self) -> &mut L {
        &mut self.link
    }

    pub fn into_link(self) -> L {
        self.link
    }

    /// Opens a session covering these halves.
    ///
    /// A bootloader that still has a session open refuses this, and the way
    /// one is left open is a host that went away without closing it — an
    /// interrupted transfer, or a process that died. Closing it and asking
    /// again is what a person would do, and there is only ever one host, so
    /// the session being open means nothing is using it. Newer firmware
    /// supersedes it on its own; this is what makes an older one recoverable
    /// without the operator having to unplug the keyboard.
    pub fn begin(&mut self, halves: Halves) -> Result<(), WiredError> {
        log::debug!("BEGIN {:#04b}", halves.bits());
        match self.link.control_out(REQ_BEGIN, halves.bits()) {
            Err(WiredError::Stalled { .. }) => {
                log::debug!("BEGIN refused; closing the session left open and asking again");
                self.end()?;
                self.link.control_out(REQ_BEGIN, halves.bits())
            }
            settled => settled,
        }
    }

    /// Points the pipe at one half.
    pub fn select(&mut self, half: Half) -> Result<(), WiredError> {
        log::debug!("SELECT {half}");
        self.link.control_out(REQ_SELECT, half_value(half))
    }

    /// Closes the session and puts the hardware back.
    pub fn end(&mut self) -> Result<(), WiredError> {
        log::debug!("END");
        self.link.control_out(REQ_END, 0)
    }

    /// Lets go of the left half's MD pin.
    ///
    /// Called after the left half's image is uploaded and before the reset, or
    /// the half comes back up in recovery again.
    pub fn release_md(&mut self) -> Result<(), WiredError> {
        log::debug!("RELEASE_MD");
        self.link.control_out(REQ_RELEASE_MD, 0)
    }

    /// Resets one half.
    pub fn reset_target(&mut self, half: Half) -> Result<(), WiredError> {
        log::debug!("RESET_TARGET {half}");
        self.link.control_out(REQ_RESET_TARGET, half_value(half))
    }

    /// Restarts the STM32 itself, so it leaves the bootloader for the keyboard
    /// firmware.
    ///
    /// The bootloader defers the reset until the request's status stage has
    /// gone out, so this returns before the device disappears — but only just,
    /// and the device is gone from the bus immediately afterwards. Treat a
    /// failure here as the reset having happened rather than not.
    pub fn reset_self(&mut self) -> Result<(), WiredError> {
        log::debug!("RESET_TARGET self");
        self.link.control_out(REQ_RESET_TARGET, 0)
    }

    /// What the bootloader thinks is going on.
    pub fn status(&mut self) -> Result<Status, WiredError> {
        let bytes = self.link.control_in(REQ_STATUS, 0, STATUS_LEN)?;
        let status = Status::parse(&bytes)?;
        log::debug!("STATUS {status:?}");
        Ok(status)
    }

    /// Asks a half that is running its application to confirm the image it is
    /// executing, if that image is the one `hash` names.
    ///
    /// Queues the exchange and returns; [`Self::confirm_result`] is how it
    /// finishes. Ignition refuses this while a recovery session is open — the
    /// replies come back on the UART and the TRRS mailbox a session owns — so
    /// the caller has to have ended one first.
    ///
    /// `hash` may be longer than the [`CONFIRM_HASH_LEN`] bytes that go on the
    /// wire, because the images are hashed with SHA-512 and both ends compare
    /// the same leading prefix. Anything shorter is refused rather than padded.
    ///
    /// `mode` rides in the high byte of `wValue` and the half in the low byte,
    /// so [`ConfirmMode::Read`] leaves the value at the plain half number.
    pub fn confirm_start(
        &mut self,
        half: Half,
        mode: ConfirmMode,
        hash: &[u8],
    ) -> Result<(), WiredError> {
        if hash.len() < CONFIRM_HASH_LEN {
            return Err(WiredError::ShortHash { got: hash.len() });
        }
        log::debug!("CONFIRM {half} {mode:?}");
        let value = ((mode as u16) << 8) | half_value(half);
        self.link
            .control_out_data(REQ_CONFIRM, value, &hash[..CONFIRM_HASH_LEN])
    }

    /// One read of the queued confirm's result block.
    pub fn confirm_result(&mut self) -> Result<ConfirmResult, WiredError> {
        let bytes = self
            .link
            .control_in(REQ_CONFIRM_RESULT, 0, CONFIRM_RESULT_LEN)?;
        let result = ConfirmResult::parse(&bytes)?;
        log::trace!("CONFIRM_RESULT {result:?}");
        Ok(result)
    }

    /// Which Ignition this is. Ignition before 2.0.3 stalls the request; its
    /// device descriptor's version is the most there is to know about it.
    pub fn version(&mut self) -> Result<IgnitionVersion, WiredError> {
        let bytes = self.link.control_in(REQ_VERSION, 0, VERSION_LEN)?;
        IgnitionVersion::parse(&bytes)
    }

    /// Whatever answered on the TRRS I2C bus, as the firmware reported it.
    ///
    /// The bytes are handed back raw: what they mean has changed between
    /// firmware revisions, and a diagnostic is the wrong place to be strict.
    pub fn i2c_scan(&mut self) -> Result<Vec<u8>, WiredError> {
        self.link.control_in(REQ_I2C_SCAN, 0, I2C_SCAN_LEN)
    }

    /// One pipe message onto the bulk endpoint.
    pub fn send(&mut self, target: Target, payload: &[u8]) -> Result<(), WiredError> {
        log::trace!("-> {target:?} {} bytes: {:?}", payload.len(), Preview(payload));
        self.link.bulk_out(&pipe::data(target, payload)?)
    }

    /// Tells the bridge to push what it is holding.
    pub fn flush(&mut self, target: Target) -> Result<(), WiredError> {
        self.link.bulk_out(&pipe::flush(target))
    }

    /// Whatever has come back, if anything has by `timeout`.
    pub fn receive(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError> {
        let got = self.link.bulk_in(timeout)?;
        if !got.is_empty() {
            log::trace!("<- {} bytes: {:?}", got.len(), Preview(&got));
        }
        Ok(got)
    }
}

/// Renders the front of a buffer as text where it is printable, so a trace
/// shows an NLIP line as a line rather than as hex.
struct Preview<'a>(&'a [u8]);

impl std::fmt::Debug for Preview<'_> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for byte in self.0.iter().take(48) {
            match byte {
                0x20..=0x7e => write!(formatter, "{}", *byte as char)?,
                _ => write!(formatter, "\\x{byte:02x}")?,
            }
        }
        if self.0.len() > 48 {
            write!(formatter, "…")?;
        }
        Ok(())
    }
}

/// How many times a control transfer is tried before its stall is believed.
pub const CONTROL_ATTEMPTS: u32 = 7;

/// How long to leave before trying again, doubling each time.
///
/// Measured rather than guessed: the window after `BEGIN` was around 7 ms on
/// the bench, and it moves with whatever the bootloader does before its main
/// loop comes round again. A fixed 2 ms four times over covered 6 ms and was
/// therefore already marginal — the failure that motivated this was the window
/// growing slightly and outrunning it. Doubling reaches ~126 ms in total,
/// which is far past anything the entry sequence does, while a request that is
/// genuinely refused still fails in well under a fifth of a second.
pub const CONTROL_RETRY: Duration = Duration::from_millis(2);

/// How many times a control *read* is tried before its fault is believed.
///
/// More than a stall gets, because the two mean different things. A stall is
/// the bootloader answering — saying no — so the stall budget only has to
/// outlast the one gap `BEGIN` opens, and past that a fast refusal is what a
/// caller wants. A fault is no answer at all: the host controller gave up on
/// a transaction the device never handshook, which is never how ignition says
/// anything. Seen on a Mac (IOKit's `kIOReturnNotResponding`) on the
/// CONFIRM_RESULT poll during a commit, seven times in a row inside the
/// 126 ms the stall budget covers — so the fault budget keeps doubling to a
/// little over a second. A read is free to repeat, and the caller's own
/// deadline is still the real bound.
pub const FAULT_ATTEMPTS: u32 = 10;

/// A link that submits a stalled or faulted control transfer again.
///
/// A host controller retries a NAK by itself and never a STALL, so a control
/// request that races a state change on the device is a transient nothing below
/// this layer will absorb. Ignition has exactly one such race: `BEGIN` returns
/// the moment the request is queued and its main loop then runs the entry
/// sequence — the MD assert and a 500 us reset blip — during which `usbd_poll`
/// is not being called. A host that pipelines the next SETUP into that gap gets
/// it stalled, and only that one: the request after it, microseconds later,
/// answers normally. Measured on hardware, where any control transfer *with a
/// data stage* stalls once immediately after `BEGIN` and never again.
///
/// Retrying is safe here rather than merely convenient: `nusb` documents that a
/// control stall clears when the next request is submitted, so there is no halt
/// to clear and nothing to reset, and every vendor request this crate sends is
/// idempotent — `STATUS` and `I2C_SCAN` are reads, and `SELECT`, `RELEASE_MD`
/// and `END` all set a state rather than advancing one.
///
/// [`WiredError::Faulted`] is retried on the same policy, but only for reads.
/// usbmon caught a single control transfer completing EPROTO with no data
/// between two STATUS replies 3 ms either side of it, both healthy, and an
/// install was abandoned on that one transfer. A read costs nothing to repeat;
/// a write that faults is ambiguous — the transfer failed on the bus, so
/// whether the device acted on it is unknown — and is left to the caller.
///
/// Deliberately narrow:
///
/// * Only [`WiredError::Stalled`], plus [`WiredError::Faulted`] on
///   `control_in`. A timeout or a disconnect is not retried, because neither
///   goes away by being asked again.
/// * Only control transfers. A stalled bulk endpoint stays halted until it is
///   cleared, so a blind retry there would spin rather than recover.
/// * Bounded. A request the bootloader genuinely refuses stalls every single
///   time — that is how it says no — and has to reach the caller as a refusal
///   once the attempts are spent, not as a hang.
pub struct Retrying<L> {
    inner: L,
    /// Tries a stalled transfer gets, the first included.
    attempts: u32,
    /// Tries a faulted read gets, the first included.
    fault_attempts: u32,
    backoff: Duration,
}

impl<L> Retrying<L> {
    pub fn new(inner: L) -> Self {
        Self {
            inner,
            attempts: CONTROL_ATTEMPTS,
            fault_attempts: FAULT_ATTEMPTS,
            backoff: CONTROL_RETRY,
        }
    }

    /// The same, with one policy spelled out for stalls and faults alike.
    /// `attempts` counts the first try.
    pub fn with_policy(inner: L, attempts: u32, backoff: Duration) -> Self {
        Self::with_policies(inner, attempts, attempts, backoff)
    }

    /// The same, with the stall and fault budgets set apart.
    pub fn with_policies(
        inner: L,
        attempts: u32,
        fault_attempts: u32,
        backoff: Duration,
    ) -> Self {
        Self {
            inner,
            attempts: attempts.max(1),
            fault_attempts: fault_attempts.max(1),
            backoff,
        }
    }

    pub fn into_inner(self) -> L {
        self.inner
    }
}

impl<L: IgnitionLink> Retrying<L> {
    /// One control transfer, submitted again for as long as it stalls — and,
    /// when the request is an idempotent read, for as long as it faults.
    fn control<T>(
        &mut self,
        request: u8,
        idempotent: bool,
        mut transfer: impl FnMut(&mut L) -> Result<T, WiredError>,
    ) -> Result<T, WiredError> {
        let mut backoff = self.backoff;
        let started = std::time::Instant::now();
        // Each kind of failure has its own budget; the loop runs to the larger
        // and the guards below hand back whichever runs out first.
        let limit = self.attempts.max(self.fault_attempts);
        for attempt in 1..limit {
            let settled = transfer(&mut self.inner);
            let (again, budget) = match &settled {
                Err(WiredError::Stalled { .. }) if attempt < self.attempts => {
                    ("stalled", self.attempts)
                }
                Err(WiredError::Faulted { .. }) if idempotent && attempt < self.fault_attempts => {
                    ("faulted", self.fault_attempts)
                }
                _ => return settled,
            };
            log::debug!(
                "vendor request {request:#04x} {again} on attempt {attempt} of {budget}, \
                 {:?} after the first; submitting it again in {backoff:?}",
                started.elapsed(),
            );
            // Not the kind of wait this crate got rid of: nothing is being
            // given time to happen, the bus is being let go of for long enough
            // that the device's control endpoint has drained the transfer it
            // is still finishing.
            std::thread::sleep(backoff);
            // Doubling, because the window is not a fixed length: it is
            // however long the bootloader takes to come round, and a policy
            // that only covers the window it was measured against fails the
            // moment the firmware does slightly more.
            backoff = backoff.saturating_mul(2);
        }
        // The last attempt's answer is the one the caller gets, stall or fault
        // included.
        transfer(&mut self.inner)
    }
}

impl<L: IgnitionLink> IgnitionLink for Retrying<L> {
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
        self.control(request, false, |link| link.control_out(request, value))
    }

    fn control_out_data(
        &mut self,
        request: u8,
        value: u16,
        data: &[u8],
    ) -> Result<(), WiredError> {
        // Idempotent all the same — CONFIRM queues a request rather than
        // advancing anything — but held to the write policy, because a fault on
        // a write leaves it unknown whether the device acted on it.
        self.control(request, false, |link| {
            link.control_out_data(request, value, data)
        })
    }

    fn control_in(
        &mut self,
        request: u8,
        value: u16,
        length: usize,
    ) -> Result<Vec<u8>, WiredError> {
        self.control(request, true, |link| link.control_in(request, value, length))
    }

    fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
        self.inner.bulk_out(message)
    }

    fn bulk_in(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError> {
        self.inner.bulk_in(timeout)
    }
}

/// The real thing: `nusb` against a plugged-in bootloader.
///
/// Deliberately the thinnest layer in the crate. Everything it can get wrong
/// is a wrong endpoint or a wrong timeout; everything else is tested above it.
pub mod usb {
    use std::time::Duration;

    use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient};
    use nusb::{Endpoint, Interface, MaybeFuture};
    use zapp_core::device::access::permission_denied;

    use super::{IgnitionLink, Retrying, BULK_WRITE_TIMEOUT, CONTROL_TIMEOUT, INTERFACE, PID, VID};
    use crate::WiredError;

    /// How many maximum-size packets one read asks for.
    ///
    /// An IN transfer's length has to be a multiple of the packet size, and a
    /// transfer ends early on any short packet, so asking for more than will
    /// arrive costs nothing.
    const READ_PACKETS: usize = 16;

    pub struct UsbLink {
        interface: Interface,
        endpoint_out: Endpoint<Bulk, Out>,
        endpoint_in: Endpoint<Bulk, In>,
    }

    impl UsbLink {
        /// Claims interface 1 on the one attached bootloader.
        ///
        /// Wrapped in [`Retrying`], which is how anything above this should
        /// have it: ignition stalls the first control transfer submitted into
        /// the gap `BEGIN` opens, and absorbing that is not a decision a caller
        /// should have to remember to make. [`Self::open_bare`] is the
        /// unwrapped link, for a caller doing its own.
        pub fn open() -> Result<Retrying<Self>, WiredError> {
            Ok(Retrying::new(Self::open_bare()?))
        }

        /// The attached bootloader's `bcdDevice`, without opening it. See
        /// [`super::bcd_version`].
        pub fn attached_bcd() -> Option<u16> {
            nusb::list_devices()
                .wait()
                .ok()?
                .find(|device| device.vendor_id() == VID && device.product_id() == PID)
                .map(|device| device.device_version())
        }

        /// The same link, with nothing between it and the bus.
        pub fn open_bare() -> Result<Self, WiredError> {
            let info = nusb::list_devices()
                .wait()
                .map_err(|err| WiredError::Usb(err.to_string()))?
                .find(|device| device.vendor_id() == VID && device.product_id() == PID)
                .ok_or(WiredError::NoBootloader)?;

            let device = info.open().wait().map_err(open_error)?;
            let interface = device
                .detach_and_claim_interface(INTERFACE)
                .wait()
                .map_err(open_error)?;

            Self::on(interface)
        }

        /// The same link over an interface someone else claimed.
        pub fn on(interface: Interface) -> Result<Self, WiredError> {
            let (out, into) = bulk_endpoints(&interface)?;
            Ok(Self {
                endpoint_out: interface
                    .endpoint::<Bulk, Out>(out)
                    .map_err(|err| WiredError::Usb(err.to_string()))?,
                endpoint_in: interface
                    .endpoint::<Bulk, In>(into)
                    .map_err(|err| WiredError::Usb(err.to_string()))?,
                interface,
            })
        }
    }

    /// An error opening the device, with a permission refusal kept apart.
    fn open_error(err: nusb::Error) -> WiredError {
        if permission_denied(&err) {
            WiredError::AccessDenied(err.to_string())
        } else {
            WiredError::Usb(err.to_string())
        }
    }

    /// The interface's bulk endpoints, taken from its descriptor.
    ///
    /// Read rather than hardcoded: the addresses are the device's to choose,
    /// and a wrong guess is a transfer that fails at run time on hardware
    /// nobody has yet.
    fn bulk_endpoints(interface: &Interface) -> Result<(u8, u8), WiredError> {
        let descriptor = interface
            .descriptor()
            .ok_or_else(|| WiredError::Usb(format!("interface {INTERFACE} has no descriptor")))?;
        let bulk: Vec<u8> = descriptor
            .endpoints()
            .filter(|endpoint| endpoint.transfer_type() == nusb::descriptors::TransferType::Bulk)
            .map(|endpoint| endpoint.address())
            .collect();

        let out = bulk.iter().find(|address| **address & 0x80 == 0);
        let into = bulk.iter().find(|address| **address & 0x80 != 0);
        match (out, into) {
            (Some(out), Some(into)) => Ok((*out, *into)),
            _ => Err(WiredError::Usb(format!(
                "interface {INTERFACE} has no bulk pair: {bulk:02x?}"
            ))),
        }
    }

    /// IOKit's `kIOReturnNotResponding`: the device did not answer a
    /// transaction the host controller retried, which is the same bus condition
    /// usbfs reports as EPROTO and this file already treats as a fault. `nusb`'s
    /// macOS backend leaves it as [`nusb::transfer::TransferError::Unknown`],
    /// so without the mapping below the one STATUS poll that lands in
    /// ignition's entry sequence ends a run with `unknown (error 0xe00002ed)`
    /// on a Mac while the same poll is quietly retried on Linux.
    #[cfg(target_os = "macos")]
    const IOKIT_NOT_RESPONDING: u32 = 0xe000_02ed;

    /// A failed control transfer, with a stall and a bus fault kept apart from
    /// the rest.
    ///
    /// The distinction is the whole point: [`Retrying`] submits a stall again,
    /// and a fault again on a read, so a timeout or a disconnect has to stay a
    /// [`WiredError::Request`] or it would be tried six more times for nothing.
    /// `Fault` is what the usbfs backend maps EPROTO, EILSEQ, EOVERFLOW, ECOMM
    /// and ETIME to; IOKit has no such variant, so its one equivalent code is
    /// picked out of `Unknown` here.
    fn control_error(request: u8, err: nusb::transfer::TransferError) -> WiredError {
        match err {
            nusb::transfer::TransferError::Stall => WiredError::Stalled { request },
            fault @ nusb::transfer::TransferError::Fault => WiredError::Faulted {
                request,
                reason: fault.to_string(),
            },
            #[cfg(target_os = "macos")]
            fault @ nusb::transfer::TransferError::Unknown(IOKIT_NOT_RESPONDING) => {
                WiredError::Faulted {
                    request,
                    reason: format!("the device did not respond ({fault})"),
                }
            }
            other => WiredError::Request {
                request,
                reason: other.to_string(),
            },
        }
    }

    impl IgnitionLink for UsbLink {
        fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
            self.control_out_data(request, value, &[])
        }

        fn control_out_data(
            &mut self,
            request: u8,
            value: u16,
            data: &[u8],
        ) -> Result<(), WiredError> {
            self.interface
                .control_out(
                    ControlOut {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Interface,
                        request,
                        value,
                        index: INTERFACE as u16,
                        data,
                    },
                    CONTROL_TIMEOUT,
                )
                .wait()
                .map_err(|err| control_error(request, err))
        }

        fn control_in(
            &mut self,
            request: u8,
            value: u16,
            length: usize,
        ) -> Result<Vec<u8>, WiredError> {
            self.interface
                .control_in(
                    ControlIn {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Interface,
                        request,
                        value,
                        index: INTERFACE as u16,
                        length: length as u16,
                    },
                    CONTROL_TIMEOUT,
                )
                .wait()
                .map_err(|err| control_error(request, err))
        }

        fn bulk_out(&mut self, message: &[u8]) -> Result<(), WiredError> {
            self.endpoint_out
                .transfer_blocking(message.to_vec().into(), BULK_WRITE_TIMEOUT)
                .into_result()
                .map(|_| ())
                .map_err(|err| match err {
                    // What a timeout looks like here too, and unlike a read
                    // there is nothing to salvage: the bridge never took the
                    // message.
                    nusb::transfer::TransferError::Cancelled => WiredError::WriteStalled {
                        after: BULK_WRITE_TIMEOUT,
                    },
                    err => WiredError::Usb(err.to_string()),
                })
        }

        fn bulk_in(&mut self, timeout: Duration) -> Result<Vec<u8>, WiredError> {
            let size = self.endpoint_in.max_packet_size() * READ_PACKETS;
            let completion = self
                .endpoint_in
                .transfer_blocking(Buffer::new(size), timeout);
            match completion.status {
                Ok(()) => Ok(completion.buffer.to_vec()),
                // What a timeout looks like: the transfer is cancelled, and
                // whatever had already arrived comes back with it.
                Err(nusb::transfer::TransferError::Cancelled) => Ok(completion.buffer.to_vec()),
                Err(err) => Err(WiredError::Usb(err.to_string())),
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use nusb::transfer::TransferError;

        #[cfg(target_os = "macos")]
        #[test]
        fn iokit_not_responding_is_a_fault() {
            // The code a Mac reports for a device that missed a transaction,
            // which the retry layer has to see as the fault it is.
            let err = control_error(super::super::REQ_STATUS, TransferError::Unknown(0xe00002ed));
            assert!(
                matches!(err, WiredError::Faulted { request: super::super::REQ_STATUS, .. }),
                "{err:?}"
            );
        }

        #[test]
        fn any_other_os_code_is_not_a_fault() {
            // kIOReturnTimeout, for one: not a bus fault, and not to be retried.
            let err = control_error(super::super::REQ_STATUS, TransferError::Unknown(0xe00002d6));
            assert!(matches!(err, WiredError::Request { .. }), "{err:?}");
            let err = control_error(super::super::REQ_STATUS, TransferError::Disconnected);
            assert!(matches!(err, WiredError::Request { .. }), "{err:?}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Sixteen bytes of a well-formed version 2 status.
    fn status_bytes() -> Vec<u8> {
        vec![
            0x02, // struct_version
            0x01, // state: active
            0x02, // target: right
            0x02, // left_state: mcuboot
            0x01, // right_state: app
            0x08, 0x00, // error: TARGET_NOT_SELECTED
            50,   // i2c_khz
            3,    // recover_count
            7,    // rx_drops
            0x00, 0x04, // max_payload: 1024
            0x01, // proto_version
            0, 0, 0, // reserved
        ]
    }

    #[test]
    fn a_version_two_status_parses_field_by_field() {
        let status = Status::parse(&status_bytes()).unwrap();
        assert_eq!(status.state, SessionState::Active);
        assert_eq!(status.target, Target::Right);
        assert_eq!(status.left_state, TargetState::McuBoot);
        assert_eq!(status.right_state, TargetState::App);
        assert_eq!(status.error, 0x0008);
        assert_eq!(status.i2c_khz, 50);
        assert_eq!(status.recover_count, 3);
        assert_eq!(status.rx_drops, 7);
        assert_eq!(status.max_payload, 1024);
        assert_eq!(status.proto_version, 1);
    }

    /// 400 kHz does not fit in byte 7, which is why the wide field exists.
    #[test]
    fn a_bus_speed_wider_than_a_byte_is_read_from_the_wide_field() {
        let mut bytes = status_bytes();
        bytes[7] = 255; // saturated, not wrapped
        bytes[13] = 0x90;
        bytes[14] = 0x01;
        assert_eq!(Status::parse(&bytes).unwrap().i2c_khz, 400);
    }

    /// A firmware from before the wide field zeroes it, and its speed fits.
    #[test]
    fn a_zeroed_wide_field_falls_back_to_byte_seven() {
        assert_eq!(Status::parse(&status_bytes()).unwrap().i2c_khz, 50);
    }

    /// The whole reason byte 0 exists. Version 1 was eight bytes with the
    /// session state in byte 0, so every field of a v1 response reads as a
    /// plausible v2 field one place to the left.
    #[test]
    fn a_status_from_any_other_version_is_refused_rather_than_misread() {
        for version in [0u8, 1, 3, 255] {
            let mut bytes = status_bytes();
            bytes[0] = version;
            assert_eq!(
                Status::parse(&bytes).unwrap_err().to_string(),
                WiredError::StatusVersion { got: version }.to_string(),
                "version {version}"
            );
        }
    }

    /// What ignition 2.0.3 built from a dirty tree at fa2b805b6f sends.
    fn version_bytes() -> Vec<u8> {
        let mut bytes = vec![1, 2, 0, 3, 0x01];
        bytes.extend_from_slice(b"fa2b805b6f\0");
        bytes
    }

    #[test]
    fn a_version_reads_its_numbers_flags_and_sha() {
        let version = IgnitionVersion::parse(&version_bytes()).unwrap();
        assert_eq!(
            version,
            IgnitionVersion {
                major: 2,
                minor: 0,
                patch: 3,
                sha: "fa2b805b6f".into(),
                dirty: true,
                diag: false,
            }
        );
        assert_eq!(version.to_string(), "2.0.3 (fa2b805b6f-dirty)");
    }

    #[test]
    fn a_diag_build_says_so() {
        let mut bytes = version_bytes();
        bytes[4] = 0x02;
        assert_eq!(
            IgnitionVersion::parse(&bytes).unwrap().to_string(),
            "2.0.3 (fa2b805b6f, diag)"
        );
    }

    /// A SHA as long as the field leaves no room for the NUL, and is still
    /// the whole SHA.
    #[test]
    fn a_sha_that_fills_the_field_is_read_whole() {
        let mut bytes = version_bytes();
        bytes.truncate(5);
        bytes.extend_from_slice(b"0123456789a");
        assert_eq!(IgnitionVersion::parse(&bytes).unwrap().sha, "0123456789a");
    }

    #[test]
    fn a_version_in_another_layout_or_cut_short_is_refused() {
        let mut bytes = version_bytes();
        bytes[0] = 2;
        assert!(matches!(
            IgnitionVersion::parse(&bytes),
            Err(WiredError::VersionLayout { got: 2 })
        ));
        assert!(matches!(
            IgnitionVersion::parse(&version_bytes()[..VERSION_LEN - 1]),
            Err(WiredError::ShortVersion { .. })
        ));
    }

    #[test]
    fn a_descriptor_version_reads_as_bcd() {
        assert_eq!(bcd_version(0x0203), "2.0.3");
        assert_eq!(bcd_version(0x0200), "2.0.0");
        assert_eq!(bcd_version(0x1234), "12.3.4");
    }

    #[test]
    fn a_status_that_is_too_short_is_refused_rather_than_padded() {
        let bytes = status_bytes();
        for length in 0..STATUS_LEN {
            assert!(matches!(
                Status::parse(&bytes[..length]),
                Err(WiredError::ShortStatus { .. })
            ));
        }
        assert!(Status::parse(&bytes).is_ok());
    }

    #[test]
    fn a_state_this_build_does_not_know_is_carried_rather_than_guessed_at() {
        let mut bytes = status_bytes();
        bytes[1] = 9;
        bytes[3] = 9;
        let status = Status::parse(&bytes).unwrap();
        assert_eq!(status.state, SessionState::Unrecognised(9));
        assert_eq!(status.left_state, TargetState::Unrecognised(9));
    }

    #[test]
    fn a_payload_limit_of_nothing_or_of_too_much_falls_back_to_the_ceiling() {
        let mut bytes = status_bytes();
        bytes[10..12].copy_from_slice(&0u16.to_le_bytes());
        assert_eq!(
            Status::parse(&bytes).unwrap().payload_budget(),
            pipe::MAX_PAYLOAD
        );

        bytes[10..12].copy_from_slice(&4096u16.to_le_bytes());
        assert_eq!(
            Status::parse(&bytes).unwrap().payload_budget(),
            pipe::MAX_PAYLOAD
        );

        bytes[10..12].copy_from_slice(&512u16.to_le_bytes());
        assert_eq!(Status::parse(&bytes).unwrap().payload_budget(), 512);
    }

    #[test]
    fn the_half_bitmap_is_bit_zero_for_left_and_bit_one_for_right() {
        assert_eq!(Halves::of(Half::Left).bits(), 0b01);
        assert_eq!(Halves::of(Half::Right).bits(), 0b10);
        assert_eq!(Halves::BOTH.bits(), 0b11);
        assert!(Halves::BOTH.contains(Half::Left));
        assert!(Halves::BOTH.contains(Half::Right));
        assert!(!Halves::LEFT.contains(Half::Right));
    }

    /// A link that records what it was asked and answers from a script.
    #[derive(Default)]
    struct Recorder {
        control: Vec<(u8, u16)>,
        /// The data stages, for the one request that has one.
        wrote: Vec<(u8, u16, Vec<u8>)>,
        status: Vec<u8>,
        /// What a CONFIRM_RESULT read answers with, when one is asked for.
        confirm: Vec<u8>,
    }

    impl IgnitionLink for Recorder {
        fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
            self.control.push((request, value));
            Ok(())
        }
        fn control_out_data(
            &mut self,
            request: u8,
            value: u16,
            data: &[u8],
        ) -> Result<(), WiredError> {
            self.control.push((request, value));
            self.wrote.push((request, value, data.to_vec()));
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
                REQ_CONFIRM_RESULT => self.confirm.clone(),
                _ => self.status.clone(),
            };
            answer.truncate(length);
            Ok(answer)
        }
        fn bulk_out(&mut self, _message: &[u8]) -> Result<(), WiredError> {
            Ok(())
        }
        fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
            Ok(Vec::new())
        }
    }

    #[test]
    fn every_request_goes_out_with_the_number_and_value_the_firmware_expects() {
        let mut ignition = Ignition::new(Recorder {
            status: status_bytes(),
            ..Recorder::default()
        });
        ignition.begin(Halves::BOTH).unwrap();
        ignition.select(Half::Left).unwrap();
        ignition.select(Half::Right).unwrap();
        ignition.reset_target(Half::Right).unwrap();
        ignition.release_md().unwrap();
        ignition.status().unwrap();
        ignition.i2c_scan().unwrap();
        ignition.end().unwrap();

        assert_eq!(
            ignition.into_link().control,
            vec![
                (REQ_BEGIN, 0b11),
                (REQ_SELECT, 1),
                (REQ_SELECT, 2),
                (REQ_RESET_TARGET, 2),
                (REQ_RELEASE_MD, 0),
                (REQ_STATUS, 0),
                (REQ_I2C_SCAN, 0),
                (REQ_END, 0),
            ]
        );
    }

    #[test]
    fn a_status_read_that_comes_back_short_is_an_error_and_not_a_guess() {
        let mut ignition = Ignition::new(Recorder {
            status: status_bytes()[..8].to_vec(),
            ..Recorder::default()
        });
        assert!(matches!(
            ignition.status(),
            Err(WiredError::ShortStatus { got: 8 })
        ));
    }

    // ---- The confirm tunnel ---------------------------------------------

    /// A finished result block: the target answered and kept the image.
    fn confirm_bytes(state: u8, rc: u8, status: u8, target: u8, hash: &[u8]) -> Vec<u8> {
        let mut bytes = vec![0u8; CONFIRM_RESULT_LEN];
        bytes[0] = state;
        bytes[1] = rc;
        bytes[2] = status;
        bytes[3] = target;
        bytes[4..4 + hash.len()].copy_from_slice(hash);
        bytes
    }

    #[test]
    fn a_read_carries_the_half_alone_and_a_commit_carries_the_mode_in_the_high_byte() {
        let hash: Vec<u8> = (0..64u8).collect();
        let mut ignition = Ignition::new(Recorder::default());
        ignition
            .confirm_start(Half::Right, ConfirmMode::Read, &hash)
            .unwrap();
        ignition
            .confirm_start(Half::Left, ConfirmMode::Read, &hash)
            .unwrap();
        ignition
            .confirm_start(Half::Right, ConfirmMode::Commit, &hash)
            .unwrap();
        ignition
            .confirm_start(Half::Left, ConfirmMode::Commit, &hash)
            .unwrap();

        let link = ignition.into_link();
        // Low byte is the half (1 left, 2 right), high byte the mode (0 READ,
        // 1 COMMIT), so a READ is the plain half number and a COMMIT is 0x01xx.
        assert_eq!(
            link.control,
            vec![
                (REQ_CONFIRM, 0x0002),
                (REQ_CONFIRM, 0x0001),
                (REQ_CONFIRM, 0x0102),
                (REQ_CONFIRM, 0x0101),
            ],
        );
        // Every one carries the leading 32 of a SHA-512, whatever the mode.
        assert!(link
            .wrote
            .iter()
            .all(|(_, _, data)| *data == hash[..32].to_vec()));
    }

    #[test]
    fn a_hash_too_short_to_name_an_image_is_refused_before_it_goes_out() {
        let mut ignition = Ignition::new(Recorder::default());
        let err = ignition
            .confirm_start(Half::Left, ConfirmMode::Read, &[0x11; 31])
            .unwrap_err();
        assert!(matches!(err, WiredError::ShortHash { got: 31 }), "{err:?}");
        assert!(ignition.into_link().control.is_empty());
    }

    #[test]
    fn a_result_block_parses_field_by_field() {
        let hash = [0xab; CONFIRM_HASH_LEN];
        let mut ignition = Ignition::new(Recorder {
            confirm: confirm_bytes(2, 0, 1, 2, &hash),
            ..Recorder::default()
        });
        let result = ignition.confirm_result().unwrap();
        assert!(result.done());
        assert_eq!(result.rc, ConfirmRc::Ok);
        assert_eq!(result.status, ConfirmStatus::Already);
        assert!(result.status.kept());
        assert_eq!(result.target, Some(Half::Right));
        assert_eq!(result.hash, hash);
    }

    #[test]
    fn a_result_still_being_worked_on_is_not_done_and_names_no_hash() {
        let mut ignition = Ignition::new(Recorder {
            // What `confirm_start` leaves behind: busy, and 0xFF for a status
            // nobody has reported yet.
            confirm: confirm_bytes(1, 0, 0xFF, 1, &[]),
            ..Recorder::default()
        });
        let result = ignition.confirm_result().unwrap();
        assert!(!result.done());
        assert_eq!(result.state, ConfirmState::Busy);
        assert_eq!(result.status, ConfirmStatus::Unrecognised(0xFF));
        assert_eq!(result.hash, vec![0u8; CONFIRM_HASH_LEN]);
    }

    #[test]
    fn every_way_the_exchange_itself_can_fail_reads_back_as_itself() {
        for (code, expected) in [
            (0u8, ConfirmRc::Ok),
            (1, ConfirmRc::Timeout),
            (2, ConfirmRc::Transport),
            (3, ConfirmRc::BadReply),
            (9, ConfirmRc::Unrecognised(9)),
        ] {
            let parsed = ConfirmResult::parse(&confirm_bytes(2, code, 0, 1, &[])).unwrap();
            assert_eq!(parsed.rc, expected, "rc {code}");
        }
    }

    #[test]
    fn only_a_match_the_target_is_keeping_counts_as_kept() {
        for (code, kept) in [
            (0x00u8, true),  // confirmed now
            (0x01, true),    // was already confirmed
            (0x02, false),   // a different image is running
            (0x03, false),   // it could not read its own hash
            (0x04, false),   // the trailer write failed
            (0x05, false),   // READ: running it, not confirmed
            (0x77, false),   // a code from a newer firmware
        ] {
            let parsed = ConfirmResult::parse(&confirm_bytes(2, 0, code, 1, &[])).unwrap();
            assert_eq!(parsed.status.kept(), kept, "status {code:#04x}");
        }
    }

    #[test]
    fn a_result_that_is_too_short_is_refused_rather_than_padded() {
        let bytes = confirm_bytes(2, 0, 0, 1, &[0x11; CONFIRM_HASH_LEN]);
        for length in 0..CONFIRM_RESULT_LEN {
            assert!(
                matches!(
                    ConfirmResult::parse(&bytes[..length]),
                    Err(WiredError::ShortConfirm { .. })
                ),
                "{length} bytes"
            );
        }
        assert!(ConfirmResult::parse(&bytes).is_ok());
    }

    #[test]
    fn a_link_that_carries_no_data_stage_says_so_rather_than_dropping_it() {
        // The trait's default. A wrapper that forgot to forward the request
        // fails loudly here rather than sending CONFIRM with no hash on it.
        struct NoData;
        impl IgnitionLink for NoData {
            fn control_out(&mut self, _request: u8, _value: u16) -> Result<(), WiredError> {
                Ok(())
            }
            fn control_in(&mut self, _r: u8, _v: u16, len: usize) -> Result<Vec<u8>, WiredError> {
                Ok(vec![0; len])
            }
            fn bulk_out(&mut self, _message: &[u8]) -> Result<(), WiredError> {
                Ok(())
            }
            fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
                Ok(Vec::new())
            }
        }
        let err = Ignition::new(NoData)
            .confirm_start(Half::Left, ConfirmMode::Read, &[0x11; 32])
            .unwrap_err();
        assert!(
            matches!(err, WiredError::Request { request: REQ_CONFIRM, .. }),
            "{err:?}"
        );
    }

    #[test]
    fn a_confirm_that_stalls_once_is_submitted_again_with_its_hash_intact() {
        // The retry has to forward the data stage too, or the second attempt
        // would reach the bootloader with nothing in it.
        #[derive(Default)]
        struct StallOnce {
            stalled: bool,
            wrote: Vec<Vec<u8>>,
        }
        impl IgnitionLink for StallOnce {
            fn control_out(&mut self, _request: u8, _value: u16) -> Result<(), WiredError> {
                Ok(())
            }
            fn control_out_data(
                &mut self,
                request: u8,
                _value: u16,
                data: &[u8],
            ) -> Result<(), WiredError> {
                self.wrote.push(data.to_vec());
                match std::mem::replace(&mut self.stalled, true) {
                    false => Err(WiredError::Stalled { request }),
                    true => Ok(()),
                }
            }
            fn control_in(&mut self, _r: u8, _v: u16, len: usize) -> Result<Vec<u8>, WiredError> {
                Ok(vec![0; len])
            }
            fn bulk_out(&mut self, _message: &[u8]) -> Result<(), WiredError> {
                Ok(())
            }
            fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
                Ok(Vec::new())
            }
        }

        let link = Retrying::with_policy(StallOnce::default(), CONTROL_ATTEMPTS, Duration::ZERO);
        let mut ignition = Ignition::new(link);
        ignition
            .confirm_start(Half::Right, ConfirmMode::Commit, &[0x5a; 32])
            .unwrap();
        assert_eq!(
            ignition.into_link().into_inner().wrote,
            vec![vec![0x5a; 32], vec![0x5a; 32]]
        );
    }

    // ---- Stalls ---------------------------------------------------------

    /// A link that stalls its first `stalls` control transfers, or every one.
    ///
    /// Counts bulk traffic separately, so a test can show that the retry never
    /// reaches an endpoint where re-submitting would be the wrong thing.
    #[derive(Default)]
    struct Stalls {
        /// Control transfers still to be stalled before one is answered.
        stalls: u32,
        /// Stall everything, the way a genuinely refused request behaves.
        always: bool,
        /// Control transfers still to be faulted before one is answered.
        faults: u32,
        /// Fault every one, the way a link that is really broken behaves.
        always_faults: bool,
        /// Fail with a disconnect, which must not be retried on any request.
        broken: bool,
        controls: u32,
        bulks: u32,
    }

    impl Stalls {
        fn answer(&mut self, request: u8) -> Result<(), WiredError> {
            self.controls += 1;
            if self.broken {
                return Err(WiredError::Request {
                    request,
                    reason: "the device went away".into(),
                });
            }
            if self.always_faults || self.faults > 0 {
                self.faults = self.faults.saturating_sub(1);
                return Err(WiredError::Faulted {
                    request,
                    reason: "hardware fault or protocol violation".into(),
                });
            }
            if self.always || self.stalls > 0 {
                self.stalls = self.stalls.saturating_sub(1);
                return Err(WiredError::Stalled { request });
            }
            Ok(())
        }
    }

    impl IgnitionLink for Stalls {
        fn control_out(&mut self, request: u8, _value: u16) -> Result<(), WiredError> {
            self.answer(request)
        }
        fn control_in(
            &mut self,
            request: u8,
            _value: u16,
            length: usize,
        ) -> Result<Vec<u8>, WiredError> {
            self.answer(request)?;
            let mut answer = status_bytes();
            answer.truncate(length);
            Ok(answer)
        }
        fn bulk_out(&mut self, _message: &[u8]) -> Result<(), WiredError> {
            self.bulks += 1;
            Err(WiredError::Stalled { request: 0 })
        }
        fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
            self.bulks += 1;
            Err(WiredError::Stalled { request: 0 })
        }
    }

    /// The policy under test, with the wait taken out of it.
    fn retrying(link: Stalls) -> Retrying<Stalls> {
        Retrying::with_policy(link, CONTROL_ATTEMPTS, Duration::ZERO)
    }

    /// The link the retry wrapped, once a run against it is over.
    fn spent(ignition: Ignition<Retrying<Stalls>>) -> Stalls {
        ignition.into_link().into_inner()
    }

    #[test]
    fn a_control_read_that_stalls_once_is_submitted_again_and_answers() {
        // The bug from the bench: BEGIN succeeds, the STATUS pipelined in
        // behind it stalls, and the very next one is fine.
        let mut ignition = Ignition::new(retrying(Stalls {
            stalls: 1,
            ..Stalls::default()
        }));
        let status = ignition.status().expect("the second attempt");
        assert_eq!(status.i2c_khz, 50);
        assert_eq!(spent(ignition).controls, 2);
    }

    #[test]
    fn a_control_write_that_stalls_is_submitted_again_too() {
        // I2C_SCAN showed it is not request-specific, so it cannot be only the
        // reads that are retried.
        let mut ignition = Ignition::new(retrying(Stalls {
            stalls: 1,
            ..Stalls::default()
        }));
        ignition.begin(Halves::BOTH).expect("the second attempt");
        assert_eq!(spent(ignition).controls, 2);
    }

    #[test]
    fn a_request_the_bootloader_really_refuses_still_fails_once_the_tries_are_spent() {
        // A refusal stalls every time — that is how the bootloader says no —
        // and has to reach the caller as one rather than as a hang.
        let mut ignition = Ignition::new(retrying(Stalls {
            always: true,
            ..Stalls::default()
        }));
        let err = ignition.status().unwrap_err();
        assert!(
            matches!(err, WiredError::Stalled { request: REQ_STATUS }),
            "{err:?}"
        );
        assert!(err.to_string().contains("refused"), "{err}");
        // Bounded, and it spent every attempt it had rather than stopping early.
        assert_eq!(spent(ignition).controls, CONTROL_ATTEMPTS);
    }

    #[test]
    fn a_control_read_that_faults_once_is_submitted_again_and_answers() {
        // The bug from the bench: one control transfer completes EPROTO with
        // no data between two healthy STATUS polls 3 ms apart.
        let mut ignition = Ignition::new(retrying(Stalls {
            faults: 1,
            ..Stalls::default()
        }));
        let status = ignition.status().expect("the second attempt");
        assert_eq!(status.i2c_khz, 50);
        assert_eq!(spent(ignition).controls, 2);
    }

    #[test]
    fn a_read_that_faults_every_time_still_fails_once_the_tries_are_spent() {
        let mut ignition = Ignition::new(retrying(Stalls {
            always_faults: true,
            ..Stalls::default()
        }));
        let err = ignition.status().unwrap_err();
        assert!(
            matches!(err, WiredError::Faulted { request: REQ_STATUS, .. }),
            "{err:?}"
        );
        // Reads as a plain failed request, which is what a spent fault is.
        assert!(
            err.to_string()
                .ends_with("failed: hardware fault or protocol violation"),
            "{err}"
        );
        assert_eq!(spent(ignition).controls, CONTROL_ATTEMPTS);
    }

    #[test]
    fn a_read_that_faults_past_the_stall_budget_is_still_tried_within_the_fault_budget() {
        // The Mac case: CONFIRM_RESULT faulted seven times running, which is
        // every try a stall would get, and the read has to outlast that.
        let mut ignition = Ignition::new(Retrying::with_policies(
            Stalls {
                faults: CONTROL_ATTEMPTS,
                ..Stalls::default()
            },
            CONTROL_ATTEMPTS,
            FAULT_ATTEMPTS,
            Duration::ZERO,
        ));
        ignition.status().expect("the attempt after the stall budget");
        assert_eq!(spent(ignition).controls, CONTROL_ATTEMPTS + 1);
    }

    #[test]
    fn a_stall_is_not_given_the_fault_budget() {
        // A refusal still has to come back fast, however long a fault is
        // waited out for.
        let mut ignition = Ignition::new(Retrying::with_policies(
            Stalls {
                always: true,
                ..Stalls::default()
            },
            CONTROL_ATTEMPTS,
            FAULT_ATTEMPTS,
            Duration::ZERO,
        ));
        let err = ignition.status().unwrap_err();
        assert!(matches!(err, WiredError::Stalled { .. }), "{err:?}");
        assert_eq!(spent(ignition).controls, CONTROL_ATTEMPTS);
    }

    #[test]
    fn a_control_write_that_faults_is_not_tried_again() {
        // A fault on a write leaves it ambiguous whether the request landed,
        // so only the reads are re-submitted.
        let mut ignition = Ignition::new(retrying(Stalls {
            always_faults: true,
            ..Stalls::default()
        }));
        let err = ignition.select(Half::Left).unwrap_err();
        assert!(matches!(err, WiredError::Faulted { .. }), "{err:?}");
        assert_eq!(spent(ignition).controls, 1);
    }

    #[test]
    fn a_failure_that_is_not_a_stall_is_not_tried_again() {
        // A disconnect or a timeout does not go away by being asked twice, and
        // retrying one would multiply a slow failure by four.
        let mut ignition = Ignition::new(retrying(Stalls {
            broken: true,
            ..Stalls::default()
        }));
        let err = ignition.status().unwrap_err();
        assert!(matches!(err, WiredError::Request { .. }), "{err:?}");
        assert_eq!(spent(ignition).controls, 1);
    }

    #[test]
    fn a_stall_on_a_bulk_endpoint_is_left_alone() {
        // A halted bulk endpoint stays halted until it is cleared, so a retry
        // there would spin instead of recovering. Only control is re-submitted.
        let mut link = retrying(Stalls::default());
        assert!(link.bulk_out(b"anything").is_err());
        assert!(link.bulk_in(Duration::ZERO).is_err());
        let inner = link.into_inner();
        assert_eq!(inner.bulks, 2);
        assert_eq!(inner.controls, 0);
    }

    #[test]
    fn a_link_that_never_stalls_costs_exactly_one_transfer_per_request() {
        let mut ignition = Ignition::new(retrying(Stalls::default()));
        ignition.status().unwrap();
        ignition.select(Half::Left).unwrap();
        ignition.end().unwrap();
        assert_eq!(spent(ignition).controls, 3);
    }

    /// A bootloader that refuses BEGIN until the open session is closed,
    /// which is what one looks like after a host was interrupted mid-transfer.
    #[derive(Default)]
    struct SessionLeftOpen {
        ended: bool,
        requests: Vec<u8>,
    }

    impl IgnitionLink for SessionLeftOpen {
        fn control_out(&mut self, request: u8, _value: u16) -> Result<(), WiredError> {
            self.requests.push(request);
            match request {
                REQ_END => {
                    self.ended = true;
                    Ok(())
                }
                REQ_BEGIN if !self.ended => Err(WiredError::Stalled { request }),
                _ => Ok(()),
            }
        }
        fn control_in(&mut self, _r: u8, _v: u16, len: usize) -> Result<Vec<u8>, WiredError> {
            Ok(vec![0; len])
        }
        fn bulk_out(&mut self, _message: &[u8]) -> Result<(), WiredError> {
            Ok(())
        }
        fn bulk_in(&mut self, _timeout: Duration) -> Result<Vec<u8>, WiredError> {
            Ok(Vec::new())
        }
    }

    #[test]
    fn a_session_someone_left_open_is_closed_and_begun_again() {
        let mut ignition = Ignition::new(SessionLeftOpen::default());
        ignition.begin(Halves::BOTH).unwrap();

        // Refused, closed, asked again — rather than surfacing a stall the
        // operator would have nothing to do about but unplug the keyboard.
        assert_eq!(
            ignition.into_link().requests,
            vec![REQ_BEGIN, REQ_END, REQ_BEGIN]
        );
    }

    #[test]
    fn the_policy_a_real_link_gets_recovers_and_still_gives_up() {
        // Every test above sets its own policy, so all of them would pass with
        // the shipped one set to a single attempt. This drives `Retrying::new`
        // — the constants a plugged-in board actually gets — and pays the real
        // backoff to do it.
        let mut ignition = Ignition::new(Retrying::new(Stalls {
            stalls: 2,
            ..Stalls::default()
        }));
        ignition.status().expect("the third attempt");
        assert_eq!(spent(ignition).controls, 3);

        // And it is bounded: a link that stalls forever fails rather than
        // hanging, in a time a person would not notice.
        let started = std::time::Instant::now();
        let mut ignition = Ignition::new(Retrying::new(Stalls {
            always: true,
            ..Stalls::default()
        }));
        assert!(ignition.status().is_err());
        assert!(started.elapsed() < Duration::from_millis(500));
    }
}
