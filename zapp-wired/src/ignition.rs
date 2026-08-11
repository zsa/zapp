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
/// Diagnostic: what answers on the TRRS I2C bus.
pub const REQ_I2C_SCAN: u8 = 0x90;

/// Bytes in a STATUS response.
pub const STATUS_LEN: usize = 16;

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
    pub i2c_khz: u8,
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
            i2c_khz: bytes[7],
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

/// The whole of the I/O against ignition.
///
/// Everything above this is arithmetic, which is the point: the codec, the
/// framing and the session logic are all exercised against an implementation
/// that is not USB.
pub trait IgnitionLink: Send {
    /// A vendor request with no data stage.
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError>;

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
    pub fn begin(&mut self, halves: Halves) -> Result<(), WiredError> {
        log::debug!("BEGIN {:#04b}", halves.bits());
        self.link.control_out(REQ_BEGIN, halves.bits())
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

/// A link that submits a stalled control transfer again.
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
/// Deliberately narrow:
///
/// * Only [`WiredError::Stalled`]. A timeout, a disconnect or a protocol error
///   is not retried, because none of them go away by being asked again.
/// * Only control transfers. A stalled bulk endpoint stays halted until it is
///   cleared, so a blind retry there would spin rather than recover.
/// * Bounded. A request the bootloader genuinely refuses stalls every single
///   time — that is how it says no — and has to reach the caller as a refusal
///   once the attempts are spent, not as a hang.
pub struct Retrying<L> {
    inner: L,
    attempts: u32,
    backoff: Duration,
}

impl<L> Retrying<L> {
    pub fn new(inner: L) -> Self {
        Self {
            inner,
            attempts: CONTROL_ATTEMPTS,
            backoff: CONTROL_RETRY,
        }
    }

    /// The same, with the policy spelled out. `attempts` counts the first try.
    pub fn with_policy(inner: L, attempts: u32, backoff: Duration) -> Self {
        Self {
            inner,
            attempts: attempts.max(1),
            backoff,
        }
    }

    pub fn into_inner(self) -> L {
        self.inner
    }
}

impl<L: IgnitionLink> Retrying<L> {
    /// One control transfer, submitted again for as long as it stalls.
    fn control<T>(
        &mut self,
        request: u8,
        mut transfer: impl FnMut(&mut L) -> Result<T, WiredError>,
    ) -> Result<T, WiredError> {
        let mut backoff = self.backoff;
        for attempt in 1..self.attempts {
            match transfer(&mut self.inner) {
                Err(WiredError::Stalled { .. }) => {
                    log::debug!(
                        "vendor request {request:#04x} stalled on attempt {attempt} of {}; \
                         submitting it again in {backoff:?}",
                        self.attempts,
                    );
                    // Not the kind of wait this crate got rid of: nothing is
                    // being given time to happen, the bus is being let go of
                    // for long enough that the device's control endpoint has
                    // drained the transfer it is still finishing.
                    std::thread::sleep(backoff);
                    // Doubling, because the window is not a fixed length: it
                    // is however long the bootloader takes to come round, and
                    // a policy that only covers the window it was measured
                    // against fails the moment the firmware does slightly more.
                    backoff = backoff.saturating_mul(2);
                }
                settled => return settled,
            }
        }
        // The last attempt's answer is the one the caller gets, stall included.
        transfer(&mut self.inner)
    }
}

impl<L: IgnitionLink> IgnitionLink for Retrying<L> {
    fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
        self.control(request, |link| link.control_out(request, value))
    }

    fn control_in(
        &mut self,
        request: u8,
        value: u16,
        length: usize,
    ) -> Result<Vec<u8>, WiredError> {
        self.control(request, |link| link.control_in(request, value, length))
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

    use super::{IgnitionLink, Retrying, CONTROL_TIMEOUT, INTERFACE, PID, VID};
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

        /// The same link, with nothing between it and the bus.
        pub fn open_bare() -> Result<Self, WiredError> {
            let info = nusb::list_devices()
                .wait()
                .map_err(|err| WiredError::Usb(err.to_string()))?
                .find(|device| device.vendor_id() == VID && device.product_id() == PID)
                .ok_or(WiredError::NoBootloader)?;

            let device = info
                .open()
                .wait()
                .map_err(|err| WiredError::Usb(err.to_string()))?;
            let interface = device
                .detach_and_claim_interface(INTERFACE)
                .wait()
                .map_err(|err| WiredError::Usb(err.to_string()))?;

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

    /// A failed control transfer, with a stall kept apart from the rest.
    ///
    /// The distinction is the whole point: [`Retrying`] submits a stall again
    /// and nothing else, so a timeout or a disconnect has to stay a
    /// [`WiredError::Request`] or it would be tried three more times for
    /// nothing.
    fn control_error(request: u8, err: nusb::transfer::TransferError) -> WiredError {
        match err {
            nusb::transfer::TransferError::Stall => WiredError::Stalled { request },
            other => WiredError::Request {
                request,
                reason: other.to_string(),
            },
        }
    }

    impl IgnitionLink for UsbLink {
        fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
            self.interface
                .control_out(
                    ControlOut {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Interface,
                        request,
                        value,
                        index: INTERFACE as u16,
                        data: &[],
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
                .transfer_blocking(message.to_vec().into(), Duration::from_secs(5))
                .into_result()
                .map(|_| ())
                .map_err(|err| WiredError::Usb(err.to_string()))
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
        status: Vec<u8>,
    }

    impl IgnitionLink for Recorder {
        fn control_out(&mut self, request: u8, value: u16) -> Result<(), WiredError> {
            self.control.push((request, value));
            Ok(())
        }
        fn control_in(
            &mut self,
            request: u8,
            value: u16,
            length: usize,
        ) -> Result<Vec<u8>, WiredError> {
            self.control.push((request, value));
            let mut answer = self.status.clone();
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
        /// Fail with something that is not a stall, which must not be retried.
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
