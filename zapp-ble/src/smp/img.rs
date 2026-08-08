//! img_mgmt: what the board is carrying, and how an image gets there.

use ciborium::value::Value;

use super::proto::{self, HEADER_LEN};
use super::{cbor, SmpError};

pub const GROUP: u16 = 1;
pub const CMD_STATE: u8 = 0;
pub const CMD_UPLOAD: u8 = 1;

/// One slot of one image, as the device describes it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ImageSlot {
    /// Which image this slot belongs to. Absent on a single-image device,
    /// which does not bother to say.
    pub image: Option<u32>,
    pub slot: u32,
    pub version: String,
    /// The SHA-256 MCUboot computed over the image, and the handle every other
    /// command names an image by.
    pub hash: Vec<u8>,
    pub bootable: bool,
    /// Marked to be tried on the next boot, and reverted if it is not confirmed.
    pub pending: bool,
    pub confirmed: bool,
    /// Running right now.
    pub active: bool,
    /// Marked to be kept, rather than tried.
    pub permanent: bool,
}

impl ImageSlot {
    /// The flags that are set, for a listing.
    pub fn flags(&self) -> Vec<&'static str> {
        [
            (self.active, "active"),
            (self.confirmed, "confirmed"),
            (self.pending, "pending"),
            (self.permanent, "permanent"),
            (self.bootable, "bootable"),
        ]
        .into_iter()
        .filter(|(set, _)| *set)
        .map(|(_, name)| name)
        .collect()
    }
}

/// The slots in an image-state response.
pub fn parse_state(payload: &[u8]) -> Result<Vec<ImageSlot>, SmpError> {
    let value = cbor::decode(payload)?;
    proto::check_rc(&value, GROUP)?;

    let images = cbor::field(&value, "images")
        .and_then(cbor::as_array)
        .ok_or_else(|| SmpError::Malformed("no \"images\" array in the image state".into()))?;

    Ok(images.iter().map(slot).collect())
}

/// One entry of the images array.
///
/// Every field is optional here on purpose: the device omits what does not
/// apply, and a listing that fails because one board leaves out `permanent` is
/// worse than one that shows the flag as unset.
fn slot(entry: &Value) -> ImageSlot {
    let flag = |key: &str| {
        cbor::field(entry, key)
            .and_then(cbor::as_bool)
            .unwrap_or(false)
    };
    ImageSlot {
        image: cbor::field(entry, "image").and_then(cbor::as_u32),
        slot: cbor::field(entry, "slot")
            .and_then(cbor::as_u32)
            .unwrap_or(0),
        version: cbor::field(entry, "version")
            .and_then(cbor::as_text)
            .unwrap_or_default()
            .to_string(),
        hash: cbor::field(entry, "hash")
            .and_then(cbor::as_bytes)
            .unwrap_or_default()
            .to_vec(),
        bootable: flag("bootable"),
        pending: flag("pending"),
        confirmed: flag("confirmed"),
        active: flag("active"),
        permanent: flag("permanent"),
    }
}

/// Marks an image, by hash: for test when `confirm` is false, permanently when
/// it is true.
pub fn state_write(hash: &[u8], confirm: bool) -> Vec<u8> {
    cbor::encode(&cbor::map(vec![
        ("hash", cbor::bytes(hash)),
        ("confirm", Value::Bool(confirm)),
    ]))
}

/// One upload request.
///
/// `len` and `sha` describe the whole image and go only in the request that
/// starts it, which is the one at offset zero — and that is the offset, not the
/// iteration, because a device is allowed to send the upload back to the start.
/// `image` rides along with them, since that is the only request in which the
/// device reads it.
pub fn upload_request(image: u32, off: u32, total: u32, sha: Option<&[u8]>, data: &[u8]) -> Vec<u8> {
    let mut entries = vec![("off", cbor::uint(off))];
    if off == 0 {
        entries.push(("image", cbor::uint(image)));
        entries.push(("len", cbor::uint(total)));
        if let Some(sha) = sha {
            entries.push(("sha", cbor::bytes(sha)));
        }
    }
    entries.push(("data", cbor::bytes(data)));
    cbor::encode(&cbor::map(entries))
}

/// What the device says after taking some of an image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UploadReply {
    /// The offset it wants next. Absent from a device that does not say, which
    /// is not supposed to happen but costs nothing to survive.
    pub off: Option<u32>,
}

pub fn parse_upload(payload: &[u8]) -> Result<UploadReply, SmpError> {
    let value = cbor::decode(payload)?;
    proto::check_rc(&value, GROUP)?;
    Ok(UploadReply {
        off: cbor::field(&value, "off").and_then(cbor::as_u32),
    })
}

/// How much image the next request can carry.
///
/// The answer depends on the request it is going into: the first one also
/// carries the length and the hash, and CBOR's length prefixes grow with the
/// numbers in them, so this is measured rather than assumed — the encoder is
/// asked how big the request would be, and the data shrinks until it fits.
pub fn chunk_size(
    image: u32,
    off: u32,
    total: u32,
    sha: Option<&[u8]>,
    remaining: usize,
    limit: usize,
) -> usize {
    let overhead = |data: usize| {
        HEADER_LEN + upload_request(image, off, total, sha, &vec![0u8; data]).len() - data
    };

    let mut size = remaining.min(limit.saturating_sub(overhead(0)));
    while size > 0 && overhead(size) + size > limit {
        // Shrinking by the overflow converges in one step unless the encoding
        // narrows as it goes, which costs one more.
        size = size.saturating_sub((overhead(size) + size - limit).max(1));
    }
    size
}

/// The offset to carry on from, given what the device answered.
///
/// A device is entitled to take less than it was sent — a full buffer, a write
/// that straddled a flash page — and to send the upload backwards, which is how
/// one that restarted asks for the image again. Both are followed. What is not
/// tolerated is an offset past the end of the image, which would mean the two
/// sides no longer agree on what is being written.
pub fn advance(off: u32, sent: u32, total: u32, reported: Option<u32>) -> Result<u32, SmpError> {
    let Some(next) = reported else {
        log::warn!("the device did not report an offset; assuming it took all {sent} bytes");
        return Ok(off.saturating_add(sent).min(total));
    };
    if next > total {
        return Err(SmpError::BadOffset { off: next, total });
    }
    if next < off {
        log::warn!("the device sent the upload back from {off} to {next}");
    } else if next < off.saturating_add(sent) {
        log::debug!(
            "the device took {} of {sent} bytes at {off}",
            next.saturating_sub(off)
        );
    }
    Ok(next)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHA: [u8; 32] = [0x5a; 32];

    fn decoded(payload: &[u8]) -> Value {
        cbor::decode(payload).unwrap()
    }

    #[test]
    fn the_first_upload_request_announces_the_whole_image() {
        let request = decoded(&upload_request(0, 0, 1024, Some(&SHA), &[1, 2, 3, 4]));
        assert_eq!(cbor::as_u32(cbor::field(&request, "image").unwrap()), Some(0));
        assert_eq!(cbor::as_u32(cbor::field(&request, "off").unwrap()), Some(0));
        assert_eq!(
            cbor::as_u32(cbor::field(&request, "len").unwrap()),
            Some(1024)
        );
        assert_eq!(
            cbor::as_bytes(cbor::field(&request, "sha").unwrap()),
            Some(SHA.as_slice())
        );
        assert_eq!(
            cbor::as_bytes(cbor::field(&request, "data").unwrap()),
            Some([1, 2, 3, 4].as_slice())
        );
    }

    #[test]
    fn every_later_request_carries_only_the_offset_and_the_data() {
        let request = decoded(&upload_request(0, 512, 1024, Some(&SHA), &[9]));
        assert_eq!(
            cbor::as_u32(cbor::field(&request, "off").unwrap()),
            Some(512)
        );
        for absent in ["len", "sha", "image"] {
            assert_eq!(cbor::field(&request, absent), None, "{absent}");
        }
    }

    #[test]
    fn a_rewind_to_zero_announces_the_image_again() {
        // The device asked for offset zero after taking some: that request has
        // to look like a fresh upload, or it starts appending to a stale one.
        let request = decoded(&upload_request(1, 0, 64, Some(&SHA), &[7]));
        assert_eq!(cbor::as_u32(cbor::field(&request, "image").unwrap()), Some(1));
        assert_eq!(cbor::as_u32(cbor::field(&request, "len").unwrap()), Some(64));
    }

    #[test]
    fn marking_an_image_is_a_hash_and_a_flag() {
        let test = decoded(&state_write(&SHA, false));
        assert_eq!(
            cbor::as_bytes(cbor::field(&test, "hash").unwrap()),
            Some(SHA.as_slice())
        );
        assert_eq!(cbor::as_bool(cbor::field(&test, "confirm").unwrap()), Some(false));
        let confirm = decoded(&state_write(&SHA, true));
        assert_eq!(
            cbor::as_bool(cbor::field(&confirm, "confirm").unwrap()),
            Some(true)
        );
    }

    fn state_payload() -> Vec<u8> {
        cbor::encode(&cbor::map(vec![(
            "images",
            Value::Array(vec![
                cbor::map(vec![
                    ("image", cbor::uint(0)),
                    ("slot", cbor::uint(0)),
                    ("version", Value::Text("1.2.3".into())),
                    ("hash", cbor::bytes(&SHA)),
                    ("bootable", Value::Bool(true)),
                    ("pending", Value::Bool(false)),
                    ("confirmed", Value::Bool(true)),
                    ("active", Value::Bool(true)),
                    ("permanent", Value::Bool(false)),
                ]),
                cbor::map(vec![
                    ("slot", cbor::uint(1)),
                    ("version", Value::Text("1.2.4".into())),
                    ("hash", cbor::bytes(&[0xab; 32])),
                    ("bootable", Value::Bool(true)),
                    ("pending", Value::Bool(true)),
                ]),
            ]),
        )]))
    }

    #[test]
    fn an_image_state_response_reads_slot_by_slot() {
        let slots = parse_state(&state_payload()).unwrap();
        assert_eq!(slots.len(), 2);

        assert_eq!(slots[0].image, Some(0));
        assert_eq!(slots[0].slot, 0);
        assert_eq!(slots[0].version, "1.2.3");
        assert_eq!(slots[0].hash, SHA);
        assert_eq!(slots[0].flags(), vec!["active", "confirmed", "bootable"]);

        // The second slot leaves out `image`, `confirmed`, `active` and
        // `permanent`, and the absent flags read as unset rather than failing.
        assert_eq!(slots[1].image, None);
        assert_eq!(slots[1].slot, 1);
        assert!(slots[1].pending && !slots[1].confirmed && !slots[1].active);
    }

    #[test]
    fn an_image_state_error_beats_the_images_it_does_not_carry() {
        let payload = cbor::encode(&cbor::map(vec![("rc", cbor::uint(6))]));
        assert!(matches!(
            parse_state(&payload),
            Err(SmpError::Remote { rc: 6, .. })
        ));
        // A success with no images at all is malformed, not empty.
        let empty = cbor::encode(&cbor::map(vec![]));
        assert!(matches!(parse_state(&empty), Err(SmpError::Malformed(_))));
    }

    #[test]
    fn an_upload_reply_is_the_offset_the_device_wants_next() {
        let payload = cbor::encode(&cbor::map(vec![("rc", cbor::uint(0)), ("off", cbor::uint(512))]));
        assert_eq!(parse_upload(&payload).unwrap().off, Some(512));
    }

    #[test]
    fn a_failing_upload_reply_is_an_error_even_with_an_offset() {
        // Zephyr answers a rejected chunk with rc set and off still present;
        // reading the offset and carrying on would silently corrupt the image.
        let payload = cbor::encode(&cbor::map(vec![
            ("rc", cbor::uint(9)),
            ("off", cbor::uint(0)),
        ]));
        let err = parse_upload(&payload).unwrap_err();
        assert!(matches!(err, SmpError::Remote { group: GROUP, rc: 9, .. }), "{err:?}");
        assert!(err.to_string().contains("corrupt"), "{err}");
    }

    #[test]
    fn a_chunk_fills_the_budget_without_overrunning_it() {
        let sized = |off: u32, data: usize| {
            HEADER_LEN + upload_request(0, off, 300_000, Some(&SHA), &vec![0u8; data]).len()
        };

        for limit in [64usize, 100, 244, 251, 512, 2048] {
            for off in [0u32, 1, 23, 24, 255, 256, 65535, 65536, 200_000] {
                let size = chunk_size(0, off, 300_000, Some(&SHA), 300_000, limit);
                if size == 0 {
                    // Only ever because the request's own fields already fill
                    // the budget — a 64-byte MTU cannot carry the first
                    // request, which announces a 32-byte hash.
                    assert!(sized(off, 0) > limit, "{off} in {limit}: gave up too soon");
                    continue;
                }
                let total = sized(off, size);
                assert!(total <= limit, "{off} in {limit}: {total} bytes");
                // And it is the largest that fits: one more byte overruns.
                assert!(
                    sized(off, size + 1) > limit,
                    "{off} in {limit}: could have sent {size}+1"
                );
            }
        }
    }

    #[test]
    fn the_first_chunk_is_smaller_than_the_ones_that_follow() {
        // It pays for the length and the hash, which is 40-odd bytes of the
        // budget the rest of the upload gets to spend on image.
        let first = chunk_size(0, 0, 300_000, Some(&SHA), 300_000, 244);
        let later = chunk_size(0, 4096, 300_000, Some(&SHA), 300_000, 244);
        assert!(first < later, "{first} vs {later}");
    }

    #[test]
    fn a_chunk_never_exceeds_what_is_left_of_the_image() {
        assert_eq!(chunk_size(0, 900, 1000, Some(&SHA), 100, 2048), 100);
    }

    #[test]
    fn a_budget_too_small_for_any_data_asks_for_nothing() {
        // Rather than looping forever, or sending an empty chunk the device
        // would answer to for ever.
        assert_eq!(chunk_size(0, 0, 1000, Some(&SHA), 1000, 20), 0);
    }

    #[test]
    fn a_device_that_took_everything_moves_the_offset_on() {
        assert_eq!(advance(0, 200, 1000, Some(200)).unwrap(), 200);
        assert_eq!(advance(800, 200, 1000, Some(1000)).unwrap(), 1000);
    }

    #[test]
    fn a_device_that_took_part_of_a_chunk_is_followed() {
        // 128 of the 200 bytes sent at 512: the next request starts at 640,
        // not at 712, and the 72 bytes it did not take are sent again.
        assert_eq!(advance(512, 200, 1000, Some(640)).unwrap(), 640);
    }

    #[test]
    fn a_device_that_rewound_is_followed_too() {
        // What a device that restarted mid-upload answers.
        assert_eq!(advance(4096, 200, 100_000, Some(0)).unwrap(), 0);
    }

    #[test]
    fn a_silent_device_is_assumed_to_have_taken_the_chunk() {
        assert_eq!(advance(512, 200, 1000, None).unwrap(), 712);
        // Clamped, so the loop still ends.
        assert_eq!(advance(900, 200, 1000, None).unwrap(), 1000);
    }

    #[test]
    fn an_offset_past_the_end_of_the_image_is_refused() {
        assert!(matches!(
            advance(512, 200, 1000, Some(1001)),
            Err(SmpError::BadOffset {
                off: 1001,
                total: 1000
            })
        ));
    }
}
