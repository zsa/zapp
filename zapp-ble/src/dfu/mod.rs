//! ZSA's own update service: a QMK image for the STM32, over Bluetooth.
//!
//! This is not `smp`. That is Zephyr's SMP server and it updates the nRF the
//! link ends at; this is `zapp_core::device::ids::ZSA_BLE_DFU_SERVICE`, served
//! by the same left half, and everything written to it is relayed on over UART
//! to the STM32 sitting in the ignition bootloader. Different service,
//! different protocol, different chip.
//!
//! Three characteristics, because the three jobs want three different kinds of
//! write (see `backend::Usage`):
//!
//! | | | |
//! |---|---|---|
//! | `32970002` | control | write with a response: one opcode at a time |
//! | `32970003` | data    | write without one: the image, at MTU a time |
//! | `32970004` | status  | read and notify: where every answer comes from |
//!
//! The board never notifies progress — it answers when a step ends, and holds
//! the byte count for whoever reads it — so the client below listens for the
//! state changes and reads for the count.

pub mod client;

use std::time::Duration;

pub use client::{DfuClient, Wait};

use crate::backend::{self, Usage};
use crate::pick::{self, Picked};
use crate::{identity, BleError, BleSighting};

pub use zapp_core::device::ids::ZSA_BLE_DFU_SERVICE as DFU_SERVICE;

/// The three characteristics, in ZSA's UUID space.
pub const DFU_CONTROL: u128 = 0x32970002_e5aa_4f42_a596_1ea5a0200001;
pub const DFU_DATA: u128 = 0x32970003_e5aa_4f42_a596_1ea5a0200001;
pub const DFU_STATUS: u128 = 0x32970004_e5aa_4f42_a596_1ea5a0200001;

/// The STM32's application region, `0x08004000`..=`0x0803ffff`: everything
/// above the 16 KiB the ignition bootloader keeps for itself.
///
/// The board checks this too, before it claims the lock or erases anything, and
/// answers `BadLength`. Checking here as well is what turns "the keyboard said
/// no" into a sentence naming the file that was too big.
pub const MAX_IMAGE: usize = 245_760;

/// Control opcodes. Payloads are little-endian.
pub const CMD_START: u8 = 0x01;
pub const CMD_ABORT: u8 = 0x02;
pub const CMD_STATUS: u8 = 0x03;
pub const CMD_ENTER: u8 = 0x04;

/// Where the board is in a transfer. Byte 0 of every answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Nothing in progress.
    Idle,
    /// The STM32 is in its bootloader; after a START, erased and taking data.
    Ready,
    /// The image landed, verified, and the STM32 was told to run it.
    Done,
    /// It stopped, and `error` says why.
    Error,
    /// The STM32 is not in its bootloader.
    NotReady,
    /// A state from a firmware newer than this build.
    Unknown(u8),
}

impl Status {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => Status::Idle,
            0x01 => Status::Ready,
            0x02 => Status::Done,
            0x03 => Status::Error,
            0x04 => Status::NotReady,
            other => Status::Unknown(other),
        }
    }

    /// Whether the STM32 is sitting in the ignition bootloader.
    ///
    /// Two states mean yes, which is the one thing about this protocol worth
    /// reading twice. A STATUS query answers `Idle` for a keyboard that is
    /// there with no transfer open and `Ready` for one that is mid-transfer;
    /// `NotReady` is the only answer that means the STM32 is running its
    /// application. `Ready` is also what a successful START or ENTER notifies,
    /// which is where the name comes from.
    pub fn in_bootloader(self) -> bool {
        matches!(self, Status::Idle | Status::Ready)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Status::Idle => "idle",
            Status::Ready => "ready",
            Status::Done => "done",
            Status::Error => "error",
            Status::NotReady => "the STM32 is not in its bootloader",
            Status::Unknown(_) => "a state this zapp does not know",
        }
    }
}

impl std::fmt::Display for Status {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Status::Unknown(code) => write!(f, "{} ({code})", self.as_str()),
            _ => f.write_str(self.as_str()),
        }
    }
}

/// Why it stopped. Byte 1, and only meaningful when there is one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    None,
    /// The mass erase was refused or timed out.
    Erase,
    /// A chunk, or the manifest that closes the transfer, did not reach the
    /// STM32.
    Write,
    /// Another update is already running: the halves share one DFU lock.
    Locked,
    /// Zero bytes, or more than the app region holds.
    BadLength,
    /// The STM32 checked the image it was given against the CRC and refused it.
    Verify,
    /// The STM32 never reached its bootloader after being told to.
    Enter,
    /// A code from a firmware newer than this build.
    Unknown(u8),
}

impl Fault {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => Fault::None,
            0x01 => Fault::Erase,
            0x02 => Fault::Write,
            0x03 => Fault::Locked,
            0x04 => Fault::BadLength,
            0x05 => Fault::Verify,
            0x06 => Fault::Enter,
            other => Fault::Unknown(other),
        }
    }

    /// What it means, in the words the person at the keyboard needs.
    pub fn meaning(self) -> &'static str {
        match self {
            Fault::None => "no fault",
            Fault::Erase => {
                "the keyboard could not erase the STM32's flash. Nothing was written, so \
                 it is still running the firmware it was"
            }
            Fault::Write => {
                "the image stopped reaching the STM32 part-way. The old firmware is \
                 already erased, so flash it again — over Bluetooth or over USB"
            }
            Fault::Locked => {
                "another update is already in progress. Wait for it to finish, or \
                 power-cycle the keyboard if nothing is updating"
            }
            Fault::BadLength => {
                "the keyboard refused the image's length: it is empty, or larger than the \
                 STM32's application region"
            }
            Fault::Verify => {
                "the image arrived corrupt — the STM32 checked it against the CRC and \
                 refused to boot it. Nothing was started, so try again"
            }
            Fault::Enter => {
                "the STM32 did not reach its bootloader. Reset the keyboard by hand and \
                 try again"
            }
            Fault::Unknown(_) => "a fault this zapp does not know",
        }
    }
}

impl std::fmt::Display for Fault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Fault::Unknown(code) => write!(f, "{} ({code})", self.meaning()),
            _ => f.write_str(self.meaning()),
        }
    }
}

/// One answer off the status characteristic.
///
/// A notification is `[status]`, or `[status, fault]` when there is a fault. A
/// read is always six bytes and carries the byte count with it, which is the
/// only place progress can be had from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    pub status: Status,
    pub fault: Fault,
    /// Bytes the board says it has taken, from a read. `None` from a
    /// notification, which does not carry it.
    pub received: Option<u32>,
}

impl Report {
    pub fn decode(bytes: &[u8]) -> Result<Report, DfuError> {
        let Some(&status) = bytes.first() else {
            return Err(DfuError::Malformed(
                "the keyboard answered with no bytes at all".into(),
            ));
        };
        Ok(Report {
            status: Status::from_byte(status),
            fault: bytes.get(1).copied().map_or(Fault::None, Fault::from_byte),
            received: bytes
                .get(2..6)
                .map(|count| u32::from_le_bytes([count[0], count[1], count[2], count[3]])),
        })
    }

    /// Whether this is the board saying no.
    ///
    /// Both halves of the answer are checked, not only the status: a failed
    /// ENTER comes back as `NotReady` with `Enter` beside it rather than as an
    /// error, and reading only byte 0 would take it for a keyboard that is
    /// merely not ready yet.
    pub fn refused(&self) -> bool {
        self.status == Status::Error || self.fault != Fault::None
    }

    fn check(self) -> Result<Report, DfuError> {
        match self.refused() {
            true => Err(DfuError::Refused {
                status: self.status,
                fault: self.fault,
            }),
            false => Ok(self),
        }
    }
}

/// What the START opcode looks like on the wire.
///
/// The CRC is always sent. The four-byte form is legal and means "do not
/// verify", which is a worse deal for one word less: without it the STM32 boots
/// whatever arrived, corrupt or not.
pub fn start_payload(total: u32, crc: u32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(9);
    payload.push(CMD_START);
    payload.extend_from_slice(&total.to_le_bytes());
    payload.extend_from_slice(&crc.to_le_bytes());
    payload
}

/// CRC-32/ISO-HDLC over the image, which is what the STM32 checks it against.
///
/// The same function as zlib's `crc32` and the `crc32fast` crate: reflected,
/// `0xedb88320`, initialised and finished with all ones. Written out rather
/// than pulled in because it is eight lines and the constant it has to agree
/// with lives in another repository.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for byte in data {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = match crc & 1 {
                0 => crc >> 1,
                _ => (crc >> 1) ^ 0xedb8_8320,
            };
        }
    }
    !crc
}

/// What can go wrong flashing the STM32 over Bluetooth.
#[derive(Debug, thiserror::Error)]
pub enum DfuError {
    #[error(transparent)]
    Ble(#[from] BleError),

    #[error("no bonded ZSA board to talk to{}", match .0 {
        Some(hint) => format!(" matching {hint:?}"),
        None => String::new(),
    })]
    NoBoard(Option<String>),

    #[error("more than one board matches: {}; name one with --device", .0.join(", "))]
    Ambiguous(Vec<String>),

    #[error("{0}")]
    Malformed(String),

    /// The board said no, and said why.
    #[error("the keyboard refused: {fault} (status {status})")]
    Refused { status: Status, fault: Fault },

    #[error("the keyboard said nothing while {doing} ({waited:?})")]
    Timeout {
        doing: &'static str,
        waited: Duration,
    },

    /// The STM32 never reached its bootloader, however it was asked.
    #[error("the STM32 did not reach its bootloader within {waited:?}")]
    NotReady { waited: Duration },

    #[error(
        "the image is {size} bytes; the STM32's application region holds {MAX_IMAGE} \
         ({} KiB)", MAX_IMAGE / 1024
    )]
    ImageTooLarge { size: usize },

    #[error("the image is empty")]
    EmptyImage,

    #[error("{0} bytes per write is not enough to carry an image")]
    ChunkTooSmall(usize),
}

/// Opens the update service on a bonded board.
///
/// Three channels rather than one: the characteristics want three different
/// kinds of write, and the backend has to be told which before it opens them.
pub fn open(hint: Option<&str>) -> Result<DfuClient, DfuError> {
    let mut backend = backend::for_platform(&[]).ok_or(BleError::NoAdapter)?;
    let sightings = backend.sightings()?;
    let key = select(&sightings, hint)?.key.clone();
    // The image is going over this link; the radio should not be spending its
    // airtime looking for other boards while it does.
    backend.stop_discovery();

    let control = backend.open_as(&key, DFU_SERVICE, DFU_CONTROL, Usage::COMMAND)?;
    let data = backend.open_as(&key, DFU_SERVICE, DFU_DATA, Usage::STREAM)?;
    // Subscribed, not polled: every answer the board gives arrives here, and a
    // client that only read would have to guess how often to ask.
    let status = backend.open_as(&key, DFU_SERVICE, DFU_STATUS, Usage::LISTEN)?;

    Ok(DfuClient::new(control, data, status))
}

/// The one board a hint names, out of everything the platform can see.
pub fn select<'a>(
    sightings: &'a [BleSighting],
    hint: Option<&str>,
) -> Result<&'a BleSighting, DfuError> {
    match pick::pick(sightings, hint, &speaks_dfu) {
        Picked::One(sighting) => Ok(sighting),
        Picked::None => Err(DfuError::NoBoard(hint.map(str::to_string))),
        Picked::Several(names) => Err(DfuError::Ambiguous(names)),
        Picked::Unbonded(name) => Err(BleError::NotBonded(name).into()),
    }
}

/// Whether a sighting could be carrying the update service.
///
/// Being a ZSA board is enough to try, the same as the SMP side: BlueZ lists
/// only the services it has resolved, and a bonded board it has not connected
/// to since boot lists none. A board that turns out not to carry it says so
/// when the channel is opened, which is a clearer error than "no board".
fn speaks_dfu(sighting: &BleSighting) -> bool {
    sighting.services.contains(&DFU_SERVICE) || identity::is_zsa_board(sighting)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn board(key: &str, name: &str) -> BleSighting {
        BleSighting {
            key: key.into(),
            name: Some(name.into()),
            services: vec![DFU_SERVICE],
            modalias: Some("usb:v3297p1978d0100".into()),
            paired: true,
            ..BleSighting::default()
        }
    }

    #[test]
    fn the_uuids_are_the_ones_the_firmware_registers() {
        // Written out the way a scanner shows them: a transposed digit here
        // fails as "no such characteristic" and reads like a firmware fault.
        let uuid = |text| identity::parse_uuid(text).unwrap();
        assert_eq!(uuid("32970001-e5aa-4f42-a596-1ea5a0200001"), DFU_SERVICE);
        assert_eq!(uuid("32970002-e5aa-4f42-a596-1ea5a0200001"), DFU_CONTROL);
        assert_eq!(uuid("32970003-e5aa-4f42-a596-1ea5a0200001"), DFU_DATA);
        assert_eq!(uuid("32970004-e5aa-4f42-a596-1ea5a0200001"), DFU_STATUS);
        // And they are not the SMP ones, which the same board also carries.
        assert_ne!(DFU_SERVICE, crate::smp::SMP_SERVICE);
        assert_ne!(DFU_SERVICE, crate::smp::RELAY_SERVICE);
    }

    #[test]
    fn the_app_region_is_the_one_ignition_reserves() {
        // 0x08004000..=0x0803ffff, which is what ble_dfu_svc.c checks against.
        assert_eq!(MAX_IMAGE, 0x0803_ffff - 0x0800_4000 + 1);
        assert_eq!(MAX_IMAGE, 240 * 1024);
    }

    #[test]
    fn a_start_says_the_length_and_the_checksum_little_endian_first() {
        let payload = start_payload(0x0004_0201, 0xdead_beef);
        assert_eq!(
            payload,
            vec![0x01, 0x01, 0x02, 0x04, 0x00, 0xef, 0xbe, 0xad, 0xde]
        );
        // Nine bytes is the form that asks the STM32 to verify what it got.
        assert_eq!(payload.len(), 9);
    }

    #[test]
    fn the_crc_is_the_one_zlib_computes() {
        // Cross-checked against python3 -c "import zlib; print(zlib.crc32(...))",
        // so these constants come from outside this file rather than from it.
        assert_eq!(crc32(b""), 0x0000_0000);
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32(b"The quick brown fox"), 0xb74574de);
        assert_eq!(crc32(&[0u8; 1024]), 0xefb5af2e);
        assert_eq!(crc32(&[0xffu8; 4096]), 0xf154_670a);
        let ramp: Vec<u8> = (0..1000u32).map(|byte| (byte % 251) as u8).collect();
        assert_eq!(crc32(&ramp), 0x7217_46a6);
    }

    #[test]
    fn being_in_the_bootloader_is_two_states_and_not_being_in_it_is_one() {
        // ble_dfu_svc.c answers a STATUS query with IDLE for an STM32 that is
        // in its bootloader with nothing open, and READY only once a transfer
        // is running. Reading READY as the only "yes" would reboot a keyboard
        // that was already exactly where it needed to be.
        assert!(Status::Idle.in_bootloader());
        assert!(Status::Ready.in_bootloader());
        assert!(!Status::NotReady.in_bootloader());
        assert!(!Status::Error.in_bootloader());
        assert!(!Status::Unknown(9).in_bootloader());
    }

    #[test]
    fn the_checksum_covers_the_image_and_not_the_suffix_that_came_with_it() {
        // The two halves of this have to agree or every Oryx download fails to
        // verify on the STM32: what is written is what is summed.
        let image: Vec<u8> = (0..2048u32).map(|byte| (byte % 251) as u8).collect();
        let mut download = image.clone();
        download.extend_from_slice(&[
            0x00, 0x01, 0x78, 0x19, 0x97, 0x32, 0x1a, 0x01, b'U', b'F', b'D', 0x10, 0x11, 0x22,
            0x33, 0x44,
        ]);

        let raw = zapp_core::firmware::load_raw_image(&download).unwrap();
        assert_eq!(raw.data, image);
        assert_eq!(raw.pid, Some(0x1978));
        assert_eq!(crc32(raw.data), crc32(&image));
        assert_ne!(crc32(raw.data), crc32(&download));
    }

    #[test]
    fn a_notification_of_one_byte_is_a_state_and_nothing_else() {
        let report = Report::decode(&[0x01]).unwrap();
        assert_eq!(report.status, Status::Ready);
        assert_eq!(report.fault, Fault::None);
        assert_eq!(report.received, None);
        assert!(!report.refused());
    }

    #[test]
    fn a_notification_of_two_bytes_carries_the_fault() {
        let report = Report::decode(&[0x03, 0x05]).unwrap();
        assert_eq!(report.status, Status::Error);
        assert_eq!(report.fault, Fault::Verify);
        assert!(report.refused());
        assert!(report.check().unwrap_err().to_string().contains("corrupt"));
    }

    #[test]
    fn a_failed_enter_is_a_refusal_even_though_its_status_is_not_an_error() {
        // What ble_dfu_svc.c sends when the STM32 never answers: NOT_RDY with
        // ERR_ENTER beside it. Reading byte 0 alone would call this "not ready
        // yet" and wait out the deadline for nothing.
        let report = Report::decode(&[0x04, 0x06]).unwrap();
        assert_eq!(report.status, Status::NotReady);
        assert_eq!(report.fault, Fault::Enter);
        assert!(report.refused());
    }

    #[test]
    fn a_read_carries_the_byte_count_the_board_has_taken() {
        let report = Report::decode(&[0x01, 0x00, 0x40, 0xe2, 0x01, 0x00]).unwrap();
        assert_eq!(report.status, Status::Ready);
        assert_eq!(report.received, Some(123_456));
    }

    #[test]
    fn nothing_at_all_is_not_a_status() {
        assert!(matches!(
            Report::decode(&[]),
            Err(DfuError::Malformed(_))
        ));
    }

    #[test]
    fn a_state_this_build_does_not_know_is_not_dressed_up_as_one_it_does() {
        let report = Report::decode(&[0x09, 0x77]).unwrap();
        assert_eq!(report.status, Status::Unknown(9));
        assert_eq!(report.fault, Fault::Unknown(0x77));
        assert!(report.refused());
        assert!(report.status.to_string().contains('9'));
    }

    #[test]
    fn the_faults_a_person_has_to_act_on_say_what_to_do() {
        let refused = |fault| DfuError::Refused {
            status: Status::Error,
            fault,
        };
        assert!(refused(Fault::Verify).to_string().contains("corrupt"));
        assert!(refused(Fault::Locked).to_string().contains("in progress"));
        assert!(refused(Fault::BadLength).to_string().contains("larger"));
    }

    #[test]
    fn an_image_too_large_names_the_region_it_would_not_fit() {
        let err = DfuError::ImageTooLarge { size: 300_000 };
        assert!(err.to_string().contains("300000"));
        assert!(err.to_string().contains("245760"));
    }

    #[test]
    fn one_bonded_board_needs_no_naming_and_an_unpaired_one_says_so() {
        assert_eq!(select(&[board("dev_A", "Voyager")], None).unwrap().key, "dev_A");

        let nearby = BleSighting {
            paired: false,
            ..board("dev_A", "Voyager")
        };
        let err = select(&[nearby], None).unwrap_err();
        assert!(matches!(err, DfuError::Ble(BleError::NotBonded(_))), "{err:?}");
    }

    #[test]
    fn two_boards_and_no_hint_is_a_question_rather_than_a_guess() {
        let boards = vec![board("dev_A", "Voyager"), board("dev_B", "Bench unit")];
        assert_eq!(select(&boards, Some("bench")).unwrap().key, "dev_B");
        let err = select(&boards, None).unwrap_err();
        assert!(matches!(err, DfuError::Ambiguous(ref names) if names.len() == 2), "{err:?}");
        assert!(err.to_string().contains("--device"));
    }
}
