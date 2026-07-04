//! Property tests for the `build.rs`-generated descriptor transcoders.
//!
//! For every covered message id, `Packet::write_json_transcoded` (descriptor table + generic
//! interpreter) must be byte-identical to the `rust-mavlink` + `serde_json` baseline across many
//! random valid messages. This is the correctness oracle that lets the generated tables replace
//! the hand-written `experimental` spikes and scale to the whole dialect.

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink_codec::{v2::V2Packet, Packet};
use rand::{prelude::StdRng, SeedableRng};

/// Message ids currently emitted by the generator (kept in sync with `SUBSET` in `build.rs`).
const SUBSET_IDS: &[u32] = &[
    0,   // HEARTBEAT (enums + bitmask)
    25,  // GPS_STATUS (numeric arrays)
    30,  // ATTITUDE (floats, incl. non-finite -> null)
    33,  // GLOBAL_POSITION_INT (signed/unsigned ints)
    253, // STATUSTEXT (enum + char array)
];

#[test]
fn generated_transcoder_matches_serde_baseline() {
    let mut rng: StdRng = SeedableRng::seed_from_u64(0xC0DE_1234);
    let mut out: Vec<u8> = Vec::new();

    for &id in SUBSET_IDS {
        for _ in 0..5000 {
            let raw = dev_utils::create_random_v2_message_from_id(&mut rng, id)
                .unwrap_or_else(|| panic!("id {id} must exist in the dialect"));
            let packet = Packet::V2(V2Packet::from(raw));

            let baseline =
                serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();

            out.clear();
            assert!(
                packet.write_json_transcoded(&mut out),
                "id {id} is not covered by the generator"
            );

            assert_eq!(
                std::str::from_utf8(&out).unwrap(),
                baseline,
                "generated transcoder diverged from serde_json baseline for id {id}"
            );
        }
    }
}

/// The range-indexed interpreter must produce the same whole-message blob and each recorded
/// field range must slice out exactly that field's serde_json encoding.
#[test]
fn generated_indexed_ranges_match_serde_fields() {
    use mavlink_codec::mavlink_json::{generated, rt};

    let mut rng: StdRng = SeedableRng::seed_from_u64(0x00F1_E1D5);
    let mut out: Vec<u8> = Vec::new();
    let mut ranges = [(0u32, 0u32); 64];

    for &id in SUBSET_IDS {
        let desc = generated::descriptor(id).unwrap();
        for _ in 0..3000 {
            let raw = dev_utils::create_random_v2_message_from_id(&mut rng, id).unwrap();
            let packet = Packet::V2(V2Packet::from(raw));

            let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
            let expected_full = serde_json::to_string(&mavlink_json).unwrap();

            out.clear();
            rt::to_json_indexed(&packet, desc, &mut out, &mut ranges);
            assert_eq!(out, expected_full.as_bytes(), "blob mismatch for id {id}");

            // Extract each field value straight from the (correct) baseline text. Going through
            // `serde_json::to_value` would upcast f32 fields to f64 and add digits, so it can't
            // be used as the per-field oracle for float messages.
            for (i, field) in desc.fields.iter().enumerate() {
                let (start, end) = ranges[i];
                let slice = &out[start as usize..end as usize];
                let expected = field_value_in(&expected_full, field.name);
                assert_eq!(
                    slice,
                    expected.as_bytes(),
                    "field {} mismatch for id {id}",
                    field.name
                );
            }
        }
    }
}

/// Reverse direction: the generated `Packet::from_json_transcoded` must produce byte-identical
/// frames to the serde baseline (`from_str` + `to_packet`), tolerating extra whitespace.
#[test]
fn generated_from_json_matches_serde_baseline() {
    use mavlink::MavlinkVersion;
    use mavlink_codec::mavlink_json::MAVLinkJSON;

    use mavlink_codec::mavlink_json::{generated, rt::FieldKind};

    let mut rng: StdRng = SeedableRng::seed_from_u64(0x0FF_1234);

    for &id in SUBSET_IDS {
        // Naive `:`/`,` -> spaced replacement corrupts the interior of char-array string values,
        // so only exercise whitespace tolerance on messages without one. The scanner structure is
        // still covered by the other messages (incl. enum objects and bitmask strings).
        let has_char_array = generated::descriptor(id)
            .unwrap()
            .fields
            .iter()
            .any(|f| matches!(f.kind, FieldKind::CharArray(_)));
        let mut tested = 0;
        for _ in 0..8000 {
            let raw = dev_utils::create_random_v2_message_from_id(&mut rng, id).unwrap();
            let packet = Packet::V2(V2Packet::from(raw));

            let json =
                serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();
            // The serde baseline cannot deserialize the `null` emitted for non-finite floats, so
            // that value never round-trips through either path; keep both sides comparable.
            if json.contains("null") {
                continue;
            }

            // Baseline: serde deserialize + typed re-serialize to wire.
            let baseline: MAVLinkJSON<MavMessage> = serde_json::from_str(&json).unwrap();
            let baseline_packet = baseline.to_packet(MavlinkVersion::V2);

            let transcoded = Packet::from_json_transcoded(json.as_bytes())
                .unwrap_or_else(|| panic!("id {id} not covered by reverse generator"));
            assert_eq!(
                transcoded.as_slice(),
                baseline_packet.as_slice(),
                "reverse transcoder diverged for id {id}: {json}"
            );

            // Whitespace tolerance: the parser must reproduce the same frame.
            if !has_char_array {
                let spaced = json.replace(':', " : ").replace(',', " , ");
                let spaced_packet = Packet::from_json_transcoded(spaced.as_bytes()).unwrap();
                assert_eq!(
                    spaced_packet.as_slice(),
                    baseline_packet.as_slice(),
                    "reverse transcoder whitespace intolerance for id {id}"
                );
            }

            tested += 1;
        }
        assert!(tested > 100, "too few samples for id {id}: {tested}");
    }
}

/// Returns the raw JSON value token that follows `"key":` in `json` (a balanced number, string,
/// object or array). Keys are searched quoted+colon so shorter keys never match inside longer
/// ones. Used as an exact per-field oracle without the f32->f64 upcast of `to_value`.
fn field_value_in<'a>(json: &'a str, key: &str) -> &'a str {
    let needle = format!("\"{key}\":");
    let bytes = json.as_bytes();
    let start = json.find(&needle).expect("field key present") + needle.len();
    let rest = &bytes[start..];
    let len = match rest[0] {
        b'"' => {
            let mut i = 1;
            while rest[i] != b'"' {
                if rest[i] == b'\\' {
                    i += 1;
                }
                i += 1;
            }
            i + 1
        }
        open @ (b'{' | b'[') => {
            let close = if open == b'{' { b'}' } else { b']' };
            let mut depth = 0i32;
            let mut i = 0;
            let mut in_str = false;
            loop {
                let c = rest[i];
                if in_str {
                    if c == b'"' {
                        in_str = false;
                    }
                } else if c == b'"' {
                    in_str = true;
                } else if c == open {
                    depth += 1;
                } else if c == close {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                i += 1;
            }
            i + 1
        }
        _ => rest
            .iter()
            .position(|&c| c == b',' || c == b'}' || c == b']')
            .unwrap(),
    };
    &json[start..start + len]
}
