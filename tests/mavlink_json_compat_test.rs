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

/// Option C spike: the hand-written GLOBAL_POSITION_INT transcoder must be byte-identical to
/// the serde_json baseline across random field values.
#[test]
fn spike_global_position_int_matches_baseline() {
    use mavlink_codec::mavlink_json::experimental;

    let mut rng: StdRng = SeedableRng::seed_from_u64(99);
    let mut out: Vec<u8> = Vec::new();

    for _ in 0..5000 {
        let raw = dev_utils::create_random_v2_message_from_id(
            &mut rng,
            experimental::GLOBAL_POSITION_INT_ID,
        )
        .expect("GLOBAL_POSITION_INT must exist in the dialect");
        let packet = Packet::V2(V2Packet::from(raw));

        let baseline =
            serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();

        out.clear();
        experimental::global_position_int_to_json(&packet, &mut out);

        assert_eq!(
            std::str::from_utf8(&out).unwrap(),
            baseline,
            "spike transcoder output diverged from serde_json baseline"
        );
    }
}

/// Option C spike (floats): ATTITUDE transcoder must match serde_json, including non-finite
/// floats rendered as `null`. Uses arbitrary bit patterns to exercise NaN/Inf/subnormals.
#[test]
fn spike_attitude_matches_baseline() {
    use mavlink::dialects::ardupilotmega::{MavMessage as M, ATTITUDE_DATA};
    use mavlink_codec::mavlink_json::experimental;
    use rand::Rng;

    let mut rng: StdRng = SeedableRng::seed_from_u64(101);
    let mut out: Vec<u8> = Vec::new();

    for _ in 0..5000 {
        let data = ATTITUDE_DATA {
            time_boot_ms: rng.random(),
            roll: f32::from_bits(rng.random()),
            pitch: f32::from_bits(rng.random()),
            yaw: f32::from_bits(rng.random()),
            rollspeed: f32::from_bits(rng.random()),
            pitchspeed: f32::from_bits(rng.random()),
            yawspeed: f32::from_bits(rng.random()),
        };
        let packet = build_packet(&M::ATTITUDE(data));

        let baseline =
            serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();
        out.clear();
        experimental::attitude_to_json(&packet, &mut out);
        assert_eq!(std::str::from_utf8(&out).unwrap(), baseline);
    }
}

/// Option C spike (numeric arrays): GPS_STATUS transcoder must match serde_json.
#[test]
fn spike_gps_status_matches_baseline() {
    use mavlink::dialects::ardupilotmega::{MavMessage as M, GPS_STATUS_DATA};
    use mavlink_codec::mavlink_json::experimental;
    use rand::Rng;

    let mut rng: StdRng = SeedableRng::seed_from_u64(202);
    let mut out: Vec<u8> = Vec::new();

    for _ in 0..3000 {
        let mut rand_array = || {
            let mut a = [0u8; 20];
            a.iter_mut().for_each(|v| *v = rng.random());
            a
        };
        let data = GPS_STATUS_DATA {
            satellite_prn: rand_array(),
            satellite_used: rand_array(),
            satellite_elevation: rand_array(),
            satellite_azimuth: rand_array(),
            satellite_snr: rand_array(),
            satellites_visible: rng.random(),
        };
        let packet = build_packet(&M::GPS_STATUS(data));

        let baseline =
            serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();
        out.clear();
        experimental::gps_status_to_json(&packet, &mut out);
        assert_eq!(std::str::from_utf8(&out).unwrap(), baseline);
    }
}

/// Option C spike (enums + bitflags): HEARTBEAT transcoder must match serde_json across all
/// valid enum variants and every base_mode bit combination.
#[test]
fn spike_heartbeat_matches_baseline() {
    use mavlink::dialects::ardupilotmega::{
        MavAutopilot, MavMessage as M, MavModeFlag, MavState, MavType, HEARTBEAT_DATA,
    };
    use mavlink_codec::mavlink_json::experimental;
    use num_traits::FromPrimitive;
    use rand::Rng;

    let mut rng: StdRng = SeedableRng::seed_from_u64(303);
    let mut out: Vec<u8> = Vec::new();

    for _ in 0..5000 {
        let data = HEARTBEAT_DATA {
            custom_mode: rng.random(),
            // All 8 base_mode bits are defined, so any u8 is a valid known-flag combination.
            base_mode: MavModeFlag::from_bits_truncate(rng.random()),
            mavtype: MavType::from_u8(rng.random_range(0..=49)).unwrap(),
            autopilot: MavAutopilot::from_u8(rng.random_range(0..=20)).unwrap(),
            system_status: MavState::from_u8(rng.random_range(0..=8)).unwrap(),
            mavlink_version: rng.random(),
        };
        let packet = build_packet(&M::HEARTBEAT(data));

        let baseline =
            serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();
        out.clear();
        experimental::heartbeat_to_json(&packet, &mut out);
        assert_eq!(std::str::from_utf8(&out).unwrap(), baseline);
    }
}

fn build_packet(message: &MavMessage) -> Packet {
    let header = mavlink::MavHeader {
        system_id: 42,
        component_id: 17,
        sequence: 200,
    };
    let mut raw = mavlink::MAVLinkV2MessageRaw::new();
    raw.serialize_message(header, message);
    Packet::V2(V2Packet::from(raw))
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
