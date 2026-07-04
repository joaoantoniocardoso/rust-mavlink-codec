//! Tests for the dual-representation [`MAVLinkMessage`] envelope: lazy wire<->JSON materialization,
//! cheap cloning, and lossless passthrough of frames we cannot transcode (router semantics).

use bytes::Bytes;
use mavlink_codec::mavlink_json::{generated, MAVLinkMessage};
use mavlink_codec::v2::{V2Packet, V2_STX};
use mavlink_codec::Packet;
use rand::{prelude::StdRng, SeedableRng};

const GLOBAL_POSITION_INT: u32 = 33; // integer-only, so wire<->JSON round-trips exactly

fn sample_packet(seed: u64, id: u32) -> Packet {
    let mut rng: StdRng = SeedableRng::seed_from_u64(seed);
    let raw = dev_utils::create_random_v2_message_from_id(&mut rng, id).unwrap();
    Packet::V2(V2Packet::from(raw))
}

fn transcoded_json(packet: &Packet) -> Bytes {
    let mut buf = Vec::new();
    assert!(packet.write_json_transcoded(&mut buf));
    Bytes::from(buf)
}

#[test]
fn from_packet_materializes_json_lazily() {
    let packet = sample_packet(0xA11CE, GLOBAL_POSITION_INT);
    let msg = MAVLinkMessage::from_packet(packet.clone());

    // Only the wire side exists until JSON is requested.
    assert!(msg.has_wire());
    assert!(!msg.has_json());
    // The id is available without transcoding anything.
    assert_eq!(msg.message_id(), Some(GLOBAL_POSITION_INT));
    assert!(!msg.has_json());

    assert_eq!(msg.wire(), Some(&packet));
    assert_eq!(msg.json(), Some(&transcoded_json(&packet)));
    assert!(msg.has_json());
}

#[test]
fn from_json_materializes_wire_lazily() {
    let packet = sample_packet(0xB0B, GLOBAL_POSITION_INT);
    let json = transcoded_json(&packet);
    let msg = MAVLinkMessage::from_json(json.clone());

    assert!(msg.has_json());
    assert!(!msg.has_wire());
    // Resolving the id from the JSON `"type"` tag must not force a wire transcode.
    assert_eq!(msg.message_id(), Some(GLOBAL_POSITION_INT));
    assert!(!msg.has_wire());

    // Integer-only message: JSON -> wire reproduces the exact original frame.
    assert_eq!(msg.wire(), Some(&packet));
    assert!(msg.has_wire());
    assert_eq!(msg.json(), Some(&json));
}

#[test]
fn system_and_component_id_resolve_from_either_side() {
    let packet = sample_packet(0xC0FFEE, GLOBAL_POSITION_INT);
    let sys = *packet.system_id();
    let comp = *packet.component_id();

    // Wire-sourced: read straight from the frame header, no JSON needed.
    let from_wire = MAVLinkMessage::from_packet(packet.clone());
    assert_eq!(from_wire.system_id(), Some(sys));
    assert_eq!(from_wire.component_id(), Some(comp));
    assert!(!from_wire.has_json());

    // JSON-sourced: read from the JSON header without materializing the wire frame.
    let from_json = MAVLinkMessage::from_json(transcoded_json(&packet));
    assert_eq!(from_json.system_id(), Some(sys));
    assert_eq!(from_json.component_id(), Some(comp));
    assert!(!from_json.has_wire());
}

#[test]
fn unknown_wire_id_passes_binary_but_has_no_json() {
    // An id not present in the compiled dialect (a message a newer sender knows, we don't).
    let unknown_id = 0x00FF_FFFEu32;
    assert!(generated::descriptor(unknown_id).is_none());

    let msgid = unknown_id.to_le_bytes();
    let payload = [1u8, 2, 3, 4];
    let mut frame = vec![V2_STX, payload.len() as u8, 0, 0, 7, 1, 1];
    frame.extend_from_slice(&msgid[0..3]);
    frame.extend_from_slice(&payload);
    frame.extend_from_slice(&[0, 0]); // checksum (unchecked by the envelope)
    let packet = Packet::V2(V2Packet::new(Bytes::from(frame)));

    let msg = MAVLinkMessage::from_packet(packet.clone());
    // Binary flows through untouched; the id is still known from the wire header.
    assert_eq!(msg.wire(), Some(&packet));
    assert_eq!(msg.message_id(), Some(unknown_id));
    // JSON cannot be produced without a descriptor, but the frame is not lost.
    assert_eq!(msg.json(), None);
}

#[test]
fn unknown_json_type_passes_text_but_has_no_wire() {
    let json = Bytes::from_static(
        br#"{"header":{"system_id":1,"component_id":1,"sequence":0,"message_id":16777214},"message":{"type":"BRAND_NEW_MESSAGE","field":7}}"#,
    );
    let msg = MAVLinkMessage::from_json(json.clone());

    // Text flows through untouched.
    assert_eq!(msg.json(), Some(&json));
    // Unknown `"type"` -> no descriptor -> no wire frame and no resolvable id.
    assert_eq!(msg.wire(), None);
    assert_eq!(msg.message_id(), None);
}

#[test]
fn clone_carries_materialized_representations() {
    let packet = sample_packet(0xC0FFEE, GLOBAL_POSITION_INT);
    let msg = MAVLinkMessage::from_packet(packet);
    let _ = msg.json(); // materialize the JSON side

    let cloned = msg.clone();
    // The clone already has both sides (no re-transcoding needed).
    assert!(cloned.has_wire());
    assert!(cloned.has_json());
    assert_eq!(cloned.wire(), msg.wire());
    assert_eq!(cloned.json(), msg.json());
}
