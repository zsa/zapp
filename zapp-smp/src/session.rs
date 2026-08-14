//! ZSA's update-session commands: mcumgr group 67.
//!
//! The host half of `shared/include/zsa_update_mgmt.h`, and the reason it
//! exists is `shared/include/zsa_update_session.h`: over Bluetooth a Voyager 2
//! accepts one shape of update and nothing else. A host declares, up front, both
//! halves' images and the build they came from, and every write after that is
//! checked against the statement. Without it the board cannot tell a sanctioned
//! upload from a stray one, cannot tell a half that is already up to date from
//! one nobody has touched, and so cannot enforce the one rule that matters — the
//! right half first, because the right is reachable only through the left and
//! only while both halves speak the same protocol.
//!
//! Two hashes per half, because mcumgr uses two and they are different numbers
//! over different bytes: the SHA-256 in the image's own MCUboot trailer, which
//! is what a slot is called once the image is installed, and the SHA-256 of the
//! whole signed file, which is what an upload announces in `sha` and is the only
//! identity available while the first chunk is being admitted.
//!
//! Errors from these commands come back in group 66, ZSA's error namespace, not
//! in 67: a host decodes one namespace whatever it was asking, and these
//! refusals are the same refusals an upload gets. That is why every parser here
//! checks the reply against [`zsa_err::GROUP`] rather than against [`GROUP`].
//!
//! Only the left half serves this group. A group 67 request relayed to the right
//! is answered "not supported" by mcumgr itself, which is the right answer: the
//! right runs no session of its own, only the claim the left pushes to it.

use ciborium::value::Value;

use crate::proto;
use crate::{cbor, image, zsa_err, SmpError};

/// ZSA's update-session group, the next id after its error namespace.
pub const GROUP: u16 = 67;
pub const CMD_OPEN: u8 = 0;
pub const CMD_STATUS: u8 = 1;
pub const CMD_CLOSE: u8 = 2;

/// Either declared hash, in bytes. Both are SHA-256.
pub const HASH_LEN: usize = 32;

/// The prefix the right half's image is identified by across the halves' link,
/// which is what a status reports for the right rather than a whole hash.
pub const TAG_LEN: usize = 8;

/// One half's image, declared by both of the hashes mcumgr names it with.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Declared {
    /// The hash in the image's own MCUboot TLV trailer: what `image list`
    /// reports for a slot, and the only one of the two a half can still name
    /// after a reboot.
    pub slot_hash: Vec<u8>,
    /// The hash of the whole signed file as it goes over the wire: what an
    /// upload announces in mcumgr's `sha` field, and what the board matches a
    /// starting transfer against.
    pub upload_hash: Vec<u8>,
}

impl Declared {
    /// The two hashes of one signed image.
    ///
    /// `None` for anything that is not a signed image, or one whose trailer
    /// carries no hash: there is nothing to declare it by.
    ///
    /// The declared slot hash is the leading [`HASH_LEN`] bytes of what the
    /// trailer carries. A Voyager 2 signs with SHA-512, so the trailer's hash is
    /// 64 bytes while the wire field and the firmware's own view of a slot hash
    /// are both 32 — the leading half is what the two ends compare.
    pub fn of(image: &[u8]) -> Option<Self> {
        let trailer = image::signed_hash(image)?;
        Some(Self {
            slot_hash: trailer.get(..HASH_LEN)?.to_vec(),
            upload_hash: image::sha256(image).to_vec(),
        })
    }
}

/// What a host states it is installing.
///
/// Both halves are always declared, even when only one is changing: a half
/// already carrying its declared image reads as satisfied, which is the same
/// statement rather than a special case.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Declaration {
    pub left: Declared,
    pub right: Declared,
    /// The bundle's build version. Cosmetic: it is echoed back and logged so a
    /// board and a host agree on which bundle a session is for, never compared.
    pub version: String,
    /// Write the declared images even where a half already carries them: a
    /// deliberate rewrite rather than an update.
    ///
    /// Suppresses one check and no others. The hashes are still matched and the
    /// right half must still be finished before the left is touched; all it
    /// turns off is the refusal a session with nothing left to do would
    /// otherwise answer, which is the useful answer to "install this" and the
    /// wrong one to "install this again".
    ///
    /// Not part of the declaration, so setting it on an otherwise identical
    /// statement turns it on rather than colliding with the session already
    /// open.
    pub redo: bool,
}

/// What the board says the host should do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Next {
    /// No session is open.
    #[default]
    None,
    /// Write the right half. Always first when the right needs anything.
    Right,
    /// The right is carrying its declared image and has confirmed it; the left
    /// is what is left.
    Left,
    /// Both halves carry what was declared and both have confirmed.
    Complete,
    /// A step a firmware newer than this build named. Reported rather than
    /// guessed at, the same way an unknown refusal code is.
    Unknown(u32),
}

impl Next {
    pub fn from_code(code: u32) -> Self {
        match code {
            0 => Self::None,
            1 => Self::Right,
            2 => Self::Left,
            3 => Self::Complete,
            other => Self::Unknown(other),
        }
    }

    pub fn code(self) -> u32 {
        match self {
            Self::None => 0,
            Self::Right => 1,
            Self::Left => 2,
            Self::Complete => 3,
            Self::Unknown(code) => code,
        }
    }
}

impl std::fmt::Display for Next {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => f.write_str("none"),
            Self::Right => f.write_str("right"),
            Self::Left => f.write_str("left"),
            Self::Complete => f.write_str("complete"),
            Self::Unknown(code) => write!(f, "step {code}"),
        }
    }
}

/// What is declared, what is done, and what is next.
///
/// Every command in this group answers with one of these, so an open and a close
/// report the state they left behind rather than a bare acknowledgement.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Status {
    pub open: bool,
    /// Nonzero while a session is open, and what a close names to end that one
    /// rather than whatever is open now.
    pub id: u32,
    /// The build version as declared, empty when nothing is.
    pub version: String,
    pub next: Next,
    /// The left carries its declared image and has confirmed it.
    pub left_done: bool,
    /// The right does.
    pub right_done: bool,
    /// The right is reporting its image state at all. False means it is off, out
    /// of range, or running firmware that predates the field — all three read
    /// the same, because a half whose state cannot be read cannot count as done.
    pub right_seen: bool,
    /// What the left is actually running.
    pub left_hash: Vec<u8>,
    /// The [`TAG_LEN`]-byte prefix of what the right reports it is running.
    /// Absent when `right_seen` is false.
    pub right_tag: Option<Vec<u8>>,
    /// Milliseconds until the session expires, 0 when none is open.
    pub ttl_ms: u32,
    /// False on a build with the guard compiled out, where an undeclared write
    /// still lands and nothing is being enforced.
    pub guard: bool,
}

/// One OPEN request: both halves, by both hashes, and the build.
///
/// `redo` is sent only when it is set. It defaults to false on the board, and
/// leaving it out keeps the request the shape a firmware that predates the field
/// reads.
pub fn open_request(declaration: &Declaration) -> Vec<u8> {
    let mut entries = vec![
        ("lslot", cbor::bytes(&declaration.left.slot_hash)),
        ("lup", cbor::bytes(&declaration.left.upload_hash)),
        ("rslot", cbor::bytes(&declaration.right.slot_hash)),
        ("rup", cbor::bytes(&declaration.right.upload_hash)),
        ("ver", Value::Text(declaration.version.clone())),
    ];
    if declaration.redo {
        entries.push(("redo", Value::Bool(true)));
    }
    cbor::encode(&cbor::map(entries))
}

/// A STATUS request, which carries nothing.
pub fn status_request() -> Vec<u8> {
    cbor::encode(&cbor::map(vec![]))
}

/// One CLOSE request.
///
/// An id closes only the session with that id; no id closes whatever is open.
/// Neither is an error when there is nothing to close — the caller wanted no
/// session and there is none.
pub fn close_request(id: Option<u32>) -> Vec<u8> {
    let entries = match id {
        Some(id) => vec![("id", cbor::uint(id))],
        None => vec![],
    };
    cbor::encode(&cbor::map(entries))
}

/// The status map every command in this group answers with.
///
/// Fields are read leniently, the way `img::parse_state` reads a slot: a board
/// that leaves one out is better reported with that field unset than not
/// reported at all. `guard` is the exception and defaults to true, because
/// assuming a board is enforcing is the reading that cannot mislead anyone into
/// thinking an undeclared write will land.
pub fn parse_status(payload: &[u8]) -> Result<Status, SmpError> {
    let value = cbor::decode(payload)?;
    // Group 66, not 67: the firmware answers these commands out of ZSA's one
    // error namespace, so a refusal here is the same refusal an upload gets.
    proto::check_rc(&value, zsa_err::GROUP)?;

    let flag = |key: &str| {
        cbor::field(&value, key)
            .and_then(cbor::as_bool)
            .unwrap_or(false)
    };
    Ok(Status {
        open: flag("open"),
        id: cbor::field(&value, "id")
            .and_then(cbor::as_u32)
            .unwrap_or(0),
        version: cbor::field(&value, "ver")
            .and_then(cbor::as_text)
            .unwrap_or_default()
            .to_string(),
        next: Next::from_code(
            cbor::field(&value, "next")
                .and_then(cbor::as_u32)
                .unwrap_or(0),
        ),
        left_done: flag("ldone"),
        right_done: flag("rdone"),
        right_seen: flag("rseen"),
        left_hash: cbor::field(&value, "lhash")
            .and_then(cbor::as_bytes)
            .unwrap_or_default()
            .to_vec(),
        right_tag: cbor::field(&value, "rtag")
            .and_then(cbor::as_bytes)
            .map(<[u8]>::to_vec),
        ttl_ms: cbor::field(&value, "ttl")
            .and_then(cbor::as_u32)
            .unwrap_or(0),
        guard: cbor::field(&value, "guard")
            .and_then(cbor::as_bool)
            .unwrap_or(true),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const LSLOT: [u8; HASH_LEN] = [0x11; HASH_LEN];
    const LUP: [u8; HASH_LEN] = [0x22; HASH_LEN];
    const RSLOT: [u8; HASH_LEN] = [0x33; HASH_LEN];
    const RUP: [u8; HASH_LEN] = [0x44; HASH_LEN];

    fn declaration() -> Declaration {
        Declaration {
            left: Declared {
                slot_hash: LSLOT.to_vec(),
                upload_hash: LUP.to_vec(),
            },
            right: Declared {
                slot_hash: RSLOT.to_vec(),
                upload_hash: RUP.to_vec(),
            },
            version: "0.1.9".into(),
            redo: false,
        }
    }

    fn decoded(payload: &[u8]) -> Value {
        cbor::decode(payload).unwrap()
    }

    /// A signed image with one hash TLV of the given length, so the trailer can
    /// be a SHA-512 the way a Voyager 2's is.
    fn signed(hash_len: usize) -> Vec<u8> {
        const MCUBOOT_MAGIC: u32 = 0x96f3_b83d;
        let payload = 64usize;
        let mut out = Vec::new();
        out.extend_from_slice(&MCUBOOT_MAGIC.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // load address
        out.extend_from_slice(&32u16.to_le_bytes()); // header size
        out.extend_from_slice(&0u16.to_le_bytes()); // protected tlvs
        out.extend_from_slice(&(payload as u32).to_le_bytes());
        out.extend_from_slice(&[0u8; 16]);
        out.extend(std::iter::repeat_n(0xAAu8, payload));

        let kind = match hash_len {
            32 => 0x10u16,
            48 => 0x11,
            _ => 0x12,
        };
        out.extend_from_slice(&0x6907u16.to_le_bytes());
        out.extend_from_slice(&((4 + 4 + hash_len) as u16).to_le_bytes());
        out.extend_from_slice(&kind.to_le_bytes());
        out.extend_from_slice(&(hash_len as u16).to_le_bytes());
        out.extend((0..hash_len).map(|byte| byte as u8));
        out
    }

    #[test]
    fn the_group_is_the_one_after_zsas_error_namespace() {
        assert_eq!(GROUP, 67);
        assert_eq!(GROUP, zsa_err::GROUP + 1);
        assert_eq!((CMD_OPEN, CMD_STATUS, CMD_CLOSE), (0, 1, 2));
    }

    #[test]
    fn an_open_declares_both_halves_by_both_hashes() {
        let request = decoded(&open_request(&declaration()));
        for (key, expected) in [
            ("lslot", LSLOT.as_slice()),
            ("lup", LUP.as_slice()),
            ("rslot", RSLOT.as_slice()),
            ("rup", RUP.as_slice()),
        ] {
            assert_eq!(
                cbor::as_bytes(cbor::field(&request, key).unwrap_or_else(|| panic!("{key}"))),
                Some(expected),
                "{key}"
            );
        }
        assert_eq!(
            cbor::as_text(cbor::field(&request, "ver").unwrap()),
            Some("0.1.9")
        );
    }

    #[test]
    fn an_ordinary_update_asks_for_no_rewrite_at_all() {
        // Absent rather than false: a firmware that predates the field reads
        // the request unchanged, and the board's own default is false anyway.
        let request = decoded(&open_request(&declaration()));
        assert_eq!(cbor::field(&request, "redo"), None);
    }

    #[test]
    fn a_forced_update_asks_for_the_images_to_be_written_again() {
        // Without this a session with nothing left to do refuses every upload,
        // so --force could not work over the air at all.
        let request = decoded(&open_request(&Declaration {
            redo: true,
            ..declaration()
        }));
        assert_eq!(
            cbor::as_bool(cbor::field(&request, "redo").unwrap()),
            Some(true)
        );
        // And it is the only thing that changed: the declaration itself is the
        // same statement, which is what keeps a re-open a renewal.
        let plain = decoded(&open_request(&declaration()));
        for key in ["lslot", "lup", "rslot", "rup", "ver"] {
            assert_eq!(cbor::field(&request, key), cbor::field(&plain, key), "{key}");
        }
    }

    #[test]
    fn the_two_hashes_of_a_half_are_different_numbers() {
        // The whole reason both are declared: one is over the file, the other
        // is a TLV inside it, and sending one for the other would look right
        // and match nothing.
        let request = decoded(&open_request(&declaration()));
        let left = cbor::as_bytes(cbor::field(&request, "lslot").unwrap());
        let up = cbor::as_bytes(cbor::field(&request, "lup").unwrap());
        assert_ne!(left, up);
    }

    #[test]
    fn a_close_names_the_session_it_means_to_end_when_it_has_one() {
        let with = decoded(&close_request(Some(7)));
        assert_eq!(cbor::as_u32(cbor::field(&with, "id").unwrap()), Some(7));
        // And an id-less close is the empty map, which closes whatever is open.
        assert_eq!(close_request(None), vec![0xa0]);
        assert_eq!(status_request(), vec![0xa0]);
    }

    fn status_payload() -> Vec<u8> {
        cbor::encode(&cbor::map(vec![
            ("open", Value::Bool(true)),
            ("id", cbor::uint(3)),
            ("ver", Value::Text("0.1.9".into())),
            ("next", cbor::uint(1)),
            ("ldone", Value::Bool(false)),
            ("rdone", Value::Bool(false)),
            ("rseen", Value::Bool(true)),
            ("lhash", cbor::bytes(&[0xab; HASH_LEN])),
            ("rtag", cbor::bytes(&[0xcd; TAG_LEN])),
            ("ttl", cbor::uint(299_000)),
            ("guard", Value::Bool(true)),
        ]))
    }

    #[test]
    fn a_status_reads_back_field_for_field() {
        let status = parse_status(&status_payload()).unwrap();
        assert!(status.open);
        assert_eq!(status.id, 3);
        assert_eq!(status.version, "0.1.9");
        assert_eq!(status.next, Next::Right);
        assert!(!status.left_done && !status.right_done);
        assert!(status.right_seen);
        assert_eq!(status.left_hash, vec![0xab; HASH_LEN]);
        assert_eq!(status.right_tag, Some(vec![0xcd; TAG_LEN]));
        assert_eq!(status.ttl_ms, 299_000);
        assert!(status.guard);
    }

    #[test]
    fn a_closed_session_reads_as_closed_rather_than_as_missing_fields() {
        let payload = cbor::encode(&cbor::map(vec![
            ("open", Value::Bool(false)),
            ("id", cbor::uint(0)),
            ("ver", Value::Text(String::new())),
            ("next", cbor::uint(0)),
            ("ttl", cbor::uint(0)),
            ("guard", Value::Bool(true)),
        ]));
        let status = parse_status(&payload).unwrap();
        assert!(!status.open);
        assert_eq!(status.next, Next::None);
        assert_eq!(status.ttl_ms, 0);
        // The right said nothing, so there is no tag rather than an empty one.
        assert_eq!(status.right_tag, None);
        assert!(status.left_hash.is_empty());
    }

    #[test]
    fn a_right_half_nobody_can_read_carries_no_tag() {
        let payload = cbor::encode(&cbor::map(vec![
            ("open", Value::Bool(true)),
            ("rseen", Value::Bool(false)),
            ("next", cbor::uint(1)),
        ]));
        let status = parse_status(&payload).unwrap();
        assert!(!status.right_seen);
        assert_eq!(status.right_tag, None);
        assert!(!status.right_done);
    }

    #[test]
    fn every_step_the_firmware_names_decodes_to_itself() {
        for (code, next) in [
            (0, Next::None),
            (1, Next::Right),
            (2, Next::Left),
            (3, Next::Complete),
        ] {
            assert_eq!(Next::from_code(code), next, "{code}");
            assert_eq!(next.code(), code);
        }
        // And one this build has never heard of is carried rather than folded
        // into a step it is not.
        assert_eq!(Next::from_code(9), Next::Unknown(9));
        assert_eq!(Next::Unknown(9).code(), 9);
        assert_eq!(Next::Unknown(9).to_string(), "step 9");
        assert_eq!(Next::default(), Next::None);
    }

    #[test]
    fn a_board_that_is_not_enforcing_says_so_and_a_silent_one_is_assumed_to_be() {
        let off = cbor::encode(&cbor::map(vec![("guard", Value::Bool(false))]));
        assert!(!parse_status(&off).unwrap().guard);
        // Absent: assume the guard is on, since that is the reading that cannot
        // tell anyone an undeclared write will land when it will not.
        let quiet = cbor::encode(&cbor::map(vec![("open", Value::Bool(false))]));
        assert!(parse_status(&quiet).unwrap().guard);
    }

    #[test]
    fn a_refusal_is_read_out_of_zsas_namespace_and_not_out_of_group_67() {
        // The one that would otherwise print "unknown error": the firmware
        // answers group 67's own commands with a group 66 code.
        let payload = cbor::encode(&cbor::map(vec![(
            "err",
            cbor::map(vec![
                ("group", cbor::uint(u32::from(zsa_err::GROUP))),
                ("rc", cbor::uint(11)),
            ]),
        )]));
        let err = parse_status(&payload).unwrap_err();
        assert!(
            matches!(err, SmpError::Remote { group: 66, rc: 11, .. }),
            "{err:?}"
        );
        assert!(err.to_string().contains("no update session"), "{err}");
    }

    #[test]
    fn a_bare_rc_from_these_commands_is_still_zsas_group() {
        // A board answering the older error shape must not have its refusal
        // filed under 67, where no host would find a meaning for it.
        let payload = cbor::encode(&cbor::map(vec![("rc", cbor::uint(13))]));
        let err = parse_status(&payload).unwrap_err();
        assert!(matches!(err, SmpError::Remote { group: 66, rc: 13, .. }), "{err:?}");
        assert!(zsa_err::lookup(66, 13).is_some());
    }

    #[test]
    fn a_declaration_is_the_trailers_hash_and_the_files_hash() {
        let image = signed(64);
        let declared = Declared::of(&image).expect("a signed image");
        // The trailer's, cut to the 32 bytes the wire field and the firmware's
        // own view both hold.
        assert_eq!(declared.slot_hash.len(), HASH_LEN);
        assert_eq!(
            declared.slot_hash,
            image::signed_hash(&image).unwrap()[..HASH_LEN]
        );
        // And the file's, which is exactly what an upload announces in `sha`.
        assert_eq!(declared.upload_hash, image::sha256(&image).to_vec());
        assert_ne!(declared.slot_hash, declared.upload_hash);
    }

    #[test]
    fn a_sha_256_trailer_is_declared_whole() {
        let image = signed(32);
        let declared = Declared::of(&image).unwrap();
        assert_eq!(declared.slot_hash, image::signed_hash(&image).unwrap());
    }

    #[test]
    fn anything_with_no_hash_to_declare_it_by_declares_nothing() {
        assert_eq!(Declared::of(&[0u8; 64]), None);
        assert_eq!(Declared::of(&[]), None);
    }
}
