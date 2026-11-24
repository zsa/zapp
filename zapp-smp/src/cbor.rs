//! CBOR at the value level.
//!
//! SMP payloads are small maps of text keys, and two of the fields that matter
//! most — `hash` and `data` — are byte strings. A derived `Serialize` would
//! write a `Vec<u8>` as an array of integers instead, which is a different and
//! wrong encoding, so the maps are built by hand where the distinction is
//! visible.

use ciborium::value::{Integer, Value};

use crate::SmpError;

pub fn encode(value: &Value) -> Vec<u8> {
    let mut out = Vec::new();
    // Writing to a Vec cannot fail, and neither can any value this crate builds.
    ciborium::into_writer(value, &mut out).expect("CBOR into an in-memory buffer");
    out
}

pub fn decode(bytes: &[u8]) -> Result<Value, SmpError> {
    ciborium::from_reader(bytes).map_err(|err| SmpError::Cbor(err.to_string()))
}

/// A map with text keys, in the order given.
pub fn map(entries: Vec<(&str, Value)>) -> Value {
    Value::Map(
        entries
            .into_iter()
            .map(|(key, value)| (Value::Text(key.to_string()), value))
            .collect(),
    )
}

pub fn uint(number: u32) -> Value {
    Value::Integer(Integer::from(number))
}

pub fn bytes(value: &[u8]) -> Value {
    Value::Bytes(value.to_vec())
}

/// One entry of a map, by key.
pub fn field<'a>(value: &'a Value, key: &str) -> Option<&'a Value> {
    match value {
        Value::Map(entries) => entries
            .iter()
            .find(|(name, _)| name.as_text() == Some(key))
            .map(|(_, value)| value),
        _ => None,
    }
}

pub fn as_int(value: &Value) -> Option<i64> {
    match value {
        Value::Integer(number) => i128::from(*number).try_into().ok(),
        _ => None,
    }
}

pub fn as_u32(value: &Value) -> Option<u32> {
    as_int(value).and_then(|number| u32::try_from(number).ok())
}

pub fn as_text(value: &Value) -> Option<&str> {
    value.as_text()
}

pub fn as_bytes(value: &Value) -> Option<&[u8]> {
    match value {
        Value::Bytes(bytes) => Some(bytes),
        _ => None,
    }
}

pub fn as_bool(value: &Value) -> Option<bool> {
    match value {
        Value::Bool(flag) => Some(*flag),
        _ => None,
    }
}

pub fn as_array(value: &Value) -> Option<&[Value]> {
    match value {
        Value::Array(items) => Some(items),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_map_is_the_single_byte_a_reset_carries() {
        assert_eq!(encode(&map(vec![])), vec![0xa0]);
    }

    #[test]
    fn a_byte_string_stays_a_byte_string() {
        // Major type 2, length 3 — not major type 4, which is what a plain
        // Vec<u8> would become and what the device would reject.
        assert_eq!(encode(&bytes(&[1, 2, 3])), vec![0x43, 0x01, 0x02, 0x03]);
    }

    #[test]
    fn integers_are_written_at_their_smallest_width() {
        assert_eq!(encode(&uint(0)), vec![0x00]);
        assert_eq!(encode(&uint(23)), vec![0x17]);
        assert_eq!(encode(&uint(24)), vec![0x18, 0x18]);
        assert_eq!(encode(&uint(256)), vec![0x19, 0x01, 0x00]);
        assert_eq!(encode(&uint(70000)), vec![0x1a, 0x00, 0x01, 0x11, 0x70]);
    }

    #[test]
    fn a_map_round_trips_through_the_wire_form() {
        let value = map(vec![
            ("off", uint(4096)),
            ("data", bytes(&[0xde, 0xad])),
            ("ok", Value::Bool(true)),
        ]);
        let decoded = decode(&encode(&value)).unwrap();
        assert_eq!(as_u32(field(&decoded, "off").unwrap()), Some(4096));
        assert_eq!(
            as_bytes(field(&decoded, "data").unwrap()),
            Some([0xde, 0xad].as_slice())
        );
        assert_eq!(as_bool(field(&decoded, "ok").unwrap()), Some(true));
        assert_eq!(field(&decoded, "missing"), None);
    }

    #[test]
    fn a_negative_rc_reads_back_as_one() {
        assert_eq!(as_int(&Value::Integer(Integer::from(-5))), Some(-5));
    }

    #[test]
    fn fields_of_the_wrong_type_read_as_absent_rather_than_panicking() {
        let value = map(vec![("off", Value::Text("nonsense".into()))]);
        assert_eq!(as_u32(field(&value, "off").unwrap()), None);
        assert_eq!(as_bytes(field(&value, "off").unwrap()), None);
        // And a payload that is not a map at all has no fields.
        assert_eq!(field(&Value::Bool(true), "off"), None);
    }

    #[test]
    fn garbage_is_refused_rather_than_guessed_at() {
        // A byte string promising eight bytes and carrying none.
        assert!(matches!(decode(&[0x48]), Err(SmpError::Cbor(_))));
    }
}
