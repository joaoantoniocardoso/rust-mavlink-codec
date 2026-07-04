//! Typed MAVLinkJSON layer bridging [`Packet`] wire frames and JSON text.
//!
//! This is the baseline (rust-mavlink + serde) implementation: it fully parses the wire
//! payload into the typed message `M` and relies on `serde` for JSON. It exists to pin the
//! exact JSON format (see `tests/mavlink_json_compat_test.rs`) and to provide the baseline
//! that future low-copy transcoders are measured against (see `benches/json_bench.rs`).

use bytes::Bytes;
use mavlink::{MavHeader, MavlinkVersion, Message};
use serde::{Deserialize, Serialize};

use crate::Packet;

/// Improved and back-compatible with our previous struct called `MAVLinkMessage`.
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
pub struct MAVLinkJSON<T: Message> {
    pub header: MAVLinkJSONHeader,
    pub message: T,
}

/// Improved and back-compatible with [`mavlink::MavHeader`].
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
pub struct MAVLinkJSONHeader {
    /// The original `MavHeader`.
    #[serde(flatten)]
    pub inner: MavHeader,
    /// Optional Message ID, missing in the original header.
    pub message_id: Option<u32>,
}

impl Packet {
    /// Parses the frame payload into a typed [`MAVLinkJSON`] wrapper.
    ///
    /// Serialize the returned value with `serde_json` to obtain the JSON text.
    pub fn to_mavlink_json<M: Message>(
        &self,
    ) -> Result<MAVLinkJSON<M>, mavlink::error::ParserError> {
        let version = match self {
            Packet::V1(_) => MavlinkVersion::V1,
            Packet::V2(_) => MavlinkVersion::V2,
        };
        let message_id = self.message_id();

        let message = M::parse(version, message_id, self.payload())?;

        let header = MAVLinkJSONHeader {
            inner: MavHeader {
                system_id: *self.system_id(),
                component_id: *self.component_id(),
                sequence: *self.sequence(),
            },
            message_id: Some(message_id),
        };

        Ok(MAVLinkJSON { header, message })
    }
}

impl<M: Message + Serialize> MAVLinkJSON<M> {
    /// Appends the JSON encoding to `buf`, reusing its allocation across calls.
    ///
    /// This is the Option A optimization: it produces the exact same bytes as
    /// `serde_json::to_string(self)` but writes into a caller-owned buffer, avoiding the
    /// per-call `String` allocation (and its trailing UTF-8 validation copy). The caller is
    /// responsible for clearing `buf` between messages if a single message per buffer is
    /// desired.
    pub fn write_json(&self, buf: &mut Vec<u8>) -> serde_json::Result<()> {
        serde_json::to_writer(buf, self)
    }

    /// Serializes the JSON into a freshly allocated [`Bytes`], ready for zero-copy fan-out.
    ///
    /// Intended for the dual-representation cache: this pays a single allocation, after which
    /// the returned `Bytes` can be cloned (refcount bump) and sent to any number of consumers
    /// for free. Uses serde_json's fast `Vec` writer and wraps the buffer into `Bytes` without
    /// copying (`Bytes::from(Vec<u8>)` takes ownership of the allocation).
    pub fn to_json_bytes(&self) -> serde_json::Result<Bytes> {
        Ok(Bytes::from(serde_json::to_vec(self)?))
    }
}

/// Experimental Option C spike: hand-written, table-free transcoders that read wire bytes and
/// emit JSON directly (no typed parse, no serde), byte-identical to the serde_json baseline.
///
/// This exists only to measure the achievable ceiling for one representative message before
/// investing in `build.rs`-generated descriptor tables for the whole dialect.
pub mod experimental {
    use crate::Packet;

    /// MAVLink message id for `GLOBAL_POSITION_INT`.
    pub const GLOBAL_POSITION_INT_ID: u32 = 33;
    const GLOBAL_POSITION_INT_PAYLOAD_LEN: usize = 28;

    /// Transcodes a `GLOBAL_POSITION_INT` frame straight to JSON, appended to `out`.
    ///
    /// Produces the exact same bytes as `serde_json::to_string(&packet.to_mavlink_json())`.
    /// Static field-name/punctuation runs are emitted as single `extend_from_slice` calls and
    /// only the dynamic integer values are formatted (via `itoa`), reading straight from the
    /// wire payload with no typed struct.
    pub fn global_position_int_to_json(packet: &Packet, out: &mut Vec<u8>) {
        // v2 truncates trailing zero bytes, so read into a zero-padded fixed buffer.
        let mut p = [0u8; GLOBAL_POSITION_INT_PAYLOAD_LEN];
        let payload = packet.payload();
        let n = payload.len().min(GLOBAL_POSITION_INT_PAYLOAD_LEN);
        p[..n].copy_from_slice(&payload[..n]);

        let time_boot_ms = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
        let lat = i32::from_le_bytes([p[4], p[5], p[6], p[7]]);
        let lon = i32::from_le_bytes([p[8], p[9], p[10], p[11]]);
        let alt = i32::from_le_bytes([p[12], p[13], p[14], p[15]]);
        let relative_alt = i32::from_le_bytes([p[16], p[17], p[18], p[19]]);
        let vx = i16::from_le_bytes([p[20], p[21]]);
        let vy = i16::from_le_bytes([p[22], p[23]]);
        let vz = i16::from_le_bytes([p[24], p[25]]);
        let hdg = u16::from_le_bytes([p[26], p[27]]);

        out.extend_from_slice(br#"{"header":{"system_id":"#);
        put_int(out, *packet.system_id());
        out.extend_from_slice(br#","component_id":"#);
        put_int(out, *packet.component_id());
        out.extend_from_slice(br#","sequence":"#);
        put_int(out, *packet.sequence());
        out.extend_from_slice(br#","message_id":"#);
        put_int(out, packet.message_id());
        out.extend_from_slice(br#"},"message":{"type":"GLOBAL_POSITION_INT","time_boot_ms":"#);
        put_int(out, time_boot_ms);
        out.extend_from_slice(br#","lat":"#);
        put_int(out, lat);
        out.extend_from_slice(br#","lon":"#);
        put_int(out, lon);
        out.extend_from_slice(br#","alt":"#);
        put_int(out, alt);
        out.extend_from_slice(br#","relative_alt":"#);
        put_int(out, relative_alt);
        out.extend_from_slice(br#","vx":"#);
        put_int(out, vx);
        out.extend_from_slice(br#","vy":"#);
        put_int(out, vy);
        out.extend_from_slice(br#","vz":"#);
        put_int(out, vz);
        out.extend_from_slice(br#","hdg":"#);
        put_int(out, hdg);
        out.extend_from_slice(br#"}}"#);
    }

    #[inline(always)]
    fn put_int<I: itoa::Integer>(out: &mut Vec<u8>, value: I) {
        let mut buffer = itoa::Buffer::new();
        out.extend_from_slice(buffer.format(value).as_bytes());
    }
}

impl<M: Message> MAVLinkJSON<M> {
    /// Serializes the typed message back into a wire [`Packet`] of the given version.
    pub fn to_packet(&self, version: MavlinkVersion) -> Packet {
        match version {
            MavlinkVersion::V1 => {
                let mut raw = mavlink::MAVLinkV1MessageRaw::new();
                raw.serialize_message(self.header.inner, &self.message);
                Packet::from(raw)
            }
            MavlinkVersion::V2 => {
                let mut raw = mavlink::MAVLinkV2MessageRaw::new();
                raw.serialize_message(self.header.inner, &self.message);
                Packet::from(raw)
            }
        }
    }
}
