//! Pins the MAVLinkJSON text format and the `Packet <-> JSON` round-trip.
//!
//! The authoritative format is the `serde_json` output of rust-mavlink's own types, matching
//! `mavlink/tests/serde_test.rs`. These tests are the correctness oracle that any future
//! low-copy transcoder must keep satisfying byte-for-byte.

use mavlink::{dialects::ardupilotmega::MavMessage, MavlinkVersion};
use mavlink_codec::{mavlink_json::MAVLinkJSON, v1::V1Packet, v2::V2Packet, Packet};
use rand::{prelude::StdRng, SeedableRng};

/// Golden end-to-end string covering the wrapper (flattened header + `message_id`), the
/// message tag (`"type"`) and a char-array field serialized as a string (`param_id`).
///
/// This drives the full path JSON text -> typed -> wire -> typed -> JSON text and asserts the
/// output is identical to the hand-written expected string.
#[test]
fn golden_param_request_read_roundtrip() {
    let expected = r#"{"header":{"system_id":1,"component_id":1,"sequence":0,"message_id":20},"message":{"type":"PARAM_REQUEST_READ","param_index":0,"target_system":0,"target_component":0,"param_id":"TEST_PARAM"}}"#;

    let mavlink_json: MAVLinkJSON<MavMessage> = serde_json::from_str(expected).unwrap();
    let packet = mavlink_json.to_packet(MavlinkVersion::V2);

    assert_eq!(packet.message_id(), 20);

    let decoded = packet.to_mavlink_json::<MavMessage>().unwrap();
    let produced = serde_json::to_string(&decoded).unwrap();

    assert_eq!(produced, expected);
}

/// Every v2 frame must survive `wire -> JSON -> wire` unchanged.
#[test]
fn roundtrip_wire_json_wire_v2() {
    let mut rng: StdRng = SeedableRng::seed_from_u64(42);

    for _ in 0..2000 {
        let raw = dev_utils::create_random_v2_raw_message(&mut rng);
        let packet = Packet::V2(V2Packet::from(raw));

        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
        let json = serde_json::to_string(&mavlink_json).unwrap();

        let back: MAVLinkJSON<MavMessage> = serde_json::from_str(&json).unwrap();
        let packet_back = back.to_packet(MavlinkVersion::V2);

        assert_eq!(
            packet.as_slice(),
            packet_back.as_slice(),
            "wire mismatch for json: {json}"
        );
    }
}

/// Every v1 frame must survive `wire -> JSON -> wire` unchanged.
#[test]
fn roundtrip_wire_json_wire_v1() {
    let mut rng: StdRng = SeedableRng::seed_from_u64(42);

    for _ in 0..2000 {
        let raw = dev_utils::create_random_v1_raw_message(&mut rng);
        let packet = Packet::V1(V1Packet::from(raw));

        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
        let json = serde_json::to_string(&mavlink_json).unwrap();

        let back: MAVLinkJSON<MavMessage> = serde_json::from_str(&json).unwrap();
        let packet_back = back.to_packet(MavlinkVersion::V1);

        assert_eq!(
            packet.as_slice(),
            packet_back.as_slice(),
            "wire mismatch for json: {json}"
        );
    }
}

/// Option A must produce byte-identical output to the `serde_json::to_string` baseline.
#[test]
fn write_json_matches_to_string() {
    let mut rng: StdRng = SeedableRng::seed_from_u64(1234);

    let mut buf: Vec<u8> = Vec::new();

    for _ in 0..2000 {
        let raw = dev_utils::create_random_v2_raw_message(&mut rng);
        let packet = Packet::V2(V2Packet::from(raw));
        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();

        let baseline = serde_json::to_string(&mavlink_json).unwrap();

        buf.clear();
        mavlink_json.write_json(&mut buf).unwrap();
        assert_eq!(buf.as_slice(), baseline.as_bytes());

        let bytes = mavlink_json.to_json_bytes().unwrap();
        assert_eq!(bytes.as_ref(), baseline.as_bytes());
    }
}

/// Every v2 frame must survive `JSON -> wire -> JSON` unchanged (text stability).
#[test]
fn roundtrip_json_wire_json_v2() {
    let mut rng: StdRng = SeedableRng::seed_from_u64(7);

    for _ in 0..2000 {
        let raw = dev_utils::create_random_v2_raw_message(&mut rng);
        let packet = Packet::V2(V2Packet::from(raw));

        let json = serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();

        let mavlink_json: MAVLinkJSON<MavMessage> = serde_json::from_str(&json).unwrap();
        let packet_back = mavlink_json.to_packet(MavlinkVersion::V2);
        let json_back =
            serde_json::to_string(&packet_back.to_mavlink_json::<MavMessage>().unwrap()).unwrap();

        assert_eq!(json, json_back);
    }
}
