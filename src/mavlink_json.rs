//! Typed MAVLinkJSON layer bridging [`Packet`] wire frames and JSON text.
//!
//! This is the baseline (rust-mavlink + serde) implementation: it fully parses the wire
//! payload into the typed message `M` and relies on `serde` for JSON. It exists to pin the
//! exact JSON format (see `tests/mavlink_json_compat_test.rs`) and to provide the baseline
//! that future low-copy transcoders are measured against (see `benches/json_bench.rs`).

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
