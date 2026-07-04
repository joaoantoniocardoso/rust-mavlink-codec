//! Router-compatibility tests for input we don't fully understand.
//!
//! A router must never drop or corrupt a frame just because the sender is ahead of us (new enum
//! values, new bitmask bits, or non-UTF-8 text from a field we don't model). These tests craft
//! such wire frames directly (the `rust-mavlink` random generator only produces conforming data)
//! and check that:
//!
//! * unknown *bitmask bits* survive forward and reverse, byte-identical to the serde baseline
//!   (rust-mavlink reads bitmasks with `from_bits_retain`, so serde keeps them too);
//! * unknown *enum values* are emitted as a bare number and round-trip JSON -> wire -> JSON;
//! * non-UTF-8 *char arrays* still yield valid JSON instead of being dropped.

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink_codec::mavlink_json::generated;
use mavlink_codec::mavlink_json::rt::{FieldKind, ScalarKind};
use mavlink_codec::v2::{V2Packet, V2_STX};
use mavlink_codec::Packet;
use rand::{prelude::StdRng, SeedableRng};

fn scalar_width(sk: ScalarKind) -> usize {
    match sk {
        ScalarKind::U8 | ScalarKind::I8 => 1,
        ScalarKind::U16 | ScalarKind::I16 => 2,
        ScalarKind::U32 | ScalarKind::I32 | ScalarKind::F32 => 4,
        ScalarKind::U64 | ScalarKind::I64 | ScalarKind::F64 => 8,
    }
}

/// Builds a wire v2 frame for `id` from a full (untrimmed) payload, matching rust-mavlink's
/// trailing-zero trimming and CRC.
fn build_frame(id: u32, sys: u8, comp: u8, seq: u8, full_payload: &[u8]) -> Packet {
    let desc = generated::descriptor(id).unwrap();
    let mut len = full_payload.len();
    while len > 1 && full_payload[len - 1] == 0 {
        len -= 1;
    }
    let msgid = id.to_le_bytes();
    let mut frame = Vec::with_capacity(12 + len);
    frame.push(V2_STX);
    frame.push(len as u8);
    frame.push(0);
    frame.push(0);
    frame.push(seq);
    frame.push(sys);
    frame.push(comp);
    frame.extend_from_slice(&msgid[0..3]);
    frame.extend_from_slice(&full_payload[..len]);
    let crc = mavlink::calculate_crc(&frame[1..], desc.crc_extra);
    frame.extend_from_slice(&crc.to_le_bytes());
    Packet::V2(V2Packet::new(bytes::Bytes::from(frame)))
}

/// A valid random message's full payload, zero-extended to the message's declared length so
/// fixed-offset overwrites are always in range (the zeros restore fields the wire trimmed).
fn full_payload(rng: &mut StdRng, id: u32) -> Vec<u8> {
    let desc = generated::descriptor(id).unwrap();
    let raw = dev_utils::create_random_v2_message_from_id(rng, id).unwrap();
    let packet = Packet::V2(V2Packet::from(raw));
    let mut payload = vec![0u8; desc.payload_len as usize];
    let src = packet.payload();
    payload[..src.len()].copy_from_slice(src);
    payload
}

/// Setting every bit of a bitmask field to 1 exercises the residual/unknown-bit path for any
/// bitmask that doesn't define all its bits; forward and reverse must still match serde exactly.
#[test]
fn unknown_bitmask_bits_match_serde_both_directions() {
    let mut rng: StdRng = SeedableRng::seed_from_u64(0xB17F_1A65);
    let mut out: Vec<u8> = Vec::new();
    let mut covered = 0u32;

    for &(name, id) in dev_utils::all_message_ids() {
        let desc = generated::descriptor(id).unwrap();
        let Some((off, sk)) = desc.fields.iter().find_map(|f| match f.kind {
            FieldKind::Bitmask(_, sk) => Some((f.offset as usize, sk)),
            _ => None,
        }) else {
            continue;
        };
        covered += 1;

        for _ in 0..80 {
            let mut payload = full_payload(&mut rng, id);
            for b in &mut payload[off..off + scalar_width(sk)] {
                *b = 0xFF;
            }
            let packet = build_frame(id, 7, 1, 42, &payload);

            let json =
                serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();

            // Forward: byte-identical to serde (which kept the unknown bits via from_bits_retain).
            out.clear();
            assert!(packet.write_json_transcoded(&mut out));
            assert_eq!(
                std::str::from_utf8(&out).unwrap(),
                json,
                "forward bitmask residual mismatch for {name} ({id})"
            );

            if json.contains("null") {
                continue; // non-finite floats don't round-trip; skip the reverse leg.
            }

            // Reverse: rust-mavlink's bitflags parser actually *rejects* the `0x..` residual it
            // serialized (serde can't round-trip its own output here), but the router must. Verify
            // our reverse reconstructs the exact original frame, unknown bits and all.
            let transcoded = Packet::from_json_transcoded(&out).unwrap();
            assert_eq!(
                transcoded.as_slice(),
                packet.as_slice(),
                "reverse bitmask residual round-trip mismatch for {name} ({id}): {json}"
            );
        }
    }
    assert!(
        covered > 10,
        "expected many bitmask messages, got {covered}"
    );
}

/// An enum value with no known name must be emitted as a bare number and survive a full
/// wire -> JSON -> wire round-trip. serde can't be the oracle here: it errors on unknown enum
/// values and drops the frame, which is exactly what a router must avoid.
#[test]
fn unknown_enum_value_propagates_round_trip() {
    // HEARTBEAT (id 0) is float-free, so wire -> JSON -> wire is an exact identity; it carries an
    // enum (`mavtype`) plus a bitmask, both driven off single wire bytes.
    let id = 0u32;
    let desc = generated::descriptor(id).unwrap();
    let (mavtype_off, table) = desc
        .fields
        .iter()
        .find_map(|f| match f.kind {
            FieldKind::Enum(t, _) if f.name == "mavtype" => Some((f.offset as usize, t)),
            _ => None,
        })
        .unwrap();

    // Pick a u8 value that is not a known MAV_TYPE variant.
    let unknown = (0u64..=255)
        .find(|v| !table.iter().any(|(known, _)| known == v))
        .expect("some u8 must be an unknown MAV_TYPE") as u8;

    let mut payload = vec![0u8; desc.payload_len as usize];
    payload[0..4].copy_from_slice(&0x1122_3344u32.to_le_bytes()); // custom_mode
    payload[mavtype_off] = unknown;
    // Fill the remaining single-byte fields with nonzero, valid-agnostic values (reverse doesn't
    // consult the enum tables for a bare number, so any value round-trips).
    for b in &mut payload[mavtype_off + 1..] {
        *b = 5;
    }
    let packet = build_frame(id, 7, 1, 42, &payload);

    let mut out = Vec::new();
    assert!(packet.write_json_transcoded(&mut out));

    // Output is valid JSON and carries the unknown value as a bare number, not `{"type":...}`.
    let value: serde_json::Value = serde_json::from_slice(&out).unwrap();
    assert_eq!(value["message"]["mavtype"], serde_json::json!(unknown));

    // Round-trip must reproduce the exact wire frame (the unknown enum value is preserved).
    let back = Packet::from_json_transcoded(&out).unwrap();
    assert_eq!(
        back.as_slice(),
        packet.as_slice(),
        "unknown enum value did not survive JSON round-trip"
    );
}

/// Non-UTF-8 bytes in a char array make rust-mavlink's `nulstr` error (dropping the frame). The
/// router must instead emit valid JSON (lossy U+FFFD), so downstream JSON sinks keep working.
#[test]
fn non_utf8_char_array_still_valid_json() {
    // STATUSTEXT (id 253) has a `text` char[50] field.
    let id = 253u32;
    let desc = generated::descriptor(id).unwrap();
    let (text_off, text_len) = desc
        .fields
        .iter()
        .find_map(|f| match f.kind {
            FieldKind::CharArray(len) if f.name == "text" => {
                Some((f.offset as usize, len as usize))
            }
            _ => None,
        })
        .unwrap();

    let mut payload = vec![0u8; desc.payload_len as usize];
    payload[0] = 3; // severity (MAV_SEVERITY_ERR), a valid enum value
                    // Invalid UTF-8: lone continuation bytes, a bare quote and a backslash to also
                    // exercise escaping, then a truncated multibyte lead byte.
    let bad = [0xFFu8, 0xFE, b'"', b'\\', 0xC3, b'A', 0x80];
    payload[text_off..text_off + bad.len()].copy_from_slice(&bad);
    let _ = text_len;
    let packet = build_frame(id, 7, 1, 42, &payload);

    // rust-mavlink parses the wire fine but *errors when serializing* the non-UTF-8 text (its
    // `nulstr` calls `from_utf8`), dropping the frame. This documents why serde can't be the oracle.
    let typed = packet.to_mavlink_json::<MavMessage>().unwrap();
    assert!(serde_json::to_string(&typed).is_err());

    let mut out = Vec::new();
    assert!(packet.write_json_transcoded(&mut out));

    // The transcoder output must be valid UTF-8 and parse as JSON.
    assert!(std::str::from_utf8(&out).is_ok());
    let value: serde_json::Value = serde_json::from_slice(&out)
        .unwrap_or_else(|e| panic!("router emitted invalid JSON for non-UTF-8 text: {e}"));
    assert!(value["message"]["text"].is_string());
}
