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

pub mod generated;
pub mod message;
pub mod rt;

pub use message::MAVLinkMessage;

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

    /// Transcodes this frame straight to MAVLinkJSON text via the generated descriptor tables,
    /// appending to `out`. Returns `false` (leaving `out` untouched) if the message id is not
    /// covered by the generator. Produces the exact same bytes as serializing
    /// [`Packet::to_mavlink_json`] with `serde_json`, without a typed parse or serde.
    pub fn write_json_transcoded(&self, out: &mut Vec<u8>) -> bool {
        match generated::descriptor(self.message_id()) {
            Some(desc) => {
                rt::to_json(self, desc, out);
                true
            }
            None => false,
        }
    }

    /// Transcodes MAVLinkJSON text straight to a wire v2 [`Packet`] via the generated descriptor
    /// tables, resolving the message from its `"type"` tag. Returns `None` if the type is not
    /// covered by the generator or the JSON is malformed. Produces the exact same frame as
    /// deserializing into [`MAVLinkJSON`] and calling `to_packet(MavlinkVersion::V2)`.
    pub fn from_json_transcoded(json: &[u8]) -> Option<Packet> {
        rt::from_json_v2(json, generated::descriptor_by_name)
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
