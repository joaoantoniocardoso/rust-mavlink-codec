//! Dual-representation message envelope for fan-out routing.
//!
//! [`MAVLinkMessage`] holds a message as its wire [`Packet`] and/or its MAVLinkJSON text
//! ([`Bytes`]), materializing the missing side lazily (and once) from the one it was built with.
//! Both representations are reference-counted, so cloning the envelope and handing each sink its
//! preferred form is cheap: a binary sink takes the `Packet` bytes, a JSON sink clones the
//! `Bytes`, and a frame is transcoded at most once regardless of the number of consumers.
//!
//! Router semantics: the source representation is always preserved, so a frame we cannot transcode
//! (an unknown message id on the wire side, or an unknown `"type"` on the JSON side) still flows to
//! sinks of its own kind — only the other representation is unavailable ([`None`]).

use std::sync::OnceLock;

use bytes::Bytes;

use crate::mavlink_json::generated;
use crate::Packet;

/// A message carried as its wire [`Packet`] and/or MAVLinkJSON text, each materialized on demand.
#[derive(Debug, Default)]
pub struct MAVLinkMessage {
    wire: OnceLock<Option<Packet>>,
    json: OnceLock<Option<Bytes>>,
}

impl MAVLinkMessage {
    /// Creates an envelope from a wire [`Packet`]; the JSON side is transcoded on first request.
    pub fn from_packet(packet: Packet) -> Self {
        let wire = OnceLock::new();
        let _ = wire.set(Some(packet));
        Self {
            wire,
            json: OnceLock::new(),
        }
    }

    /// Creates an envelope from MAVLinkJSON text; the wire side is transcoded on first request.
    pub fn from_json(json: impl Into<Bytes>) -> Self {
        let cell = OnceLock::new();
        let _ = cell.set(Some(json.into()));
        Self {
            wire: OnceLock::new(),
            json: cell,
        }
    }

    /// The wire representation, transcoding it from JSON on first access if needed.
    ///
    /// Returns [`None`] only for a JSON-sourced message whose `"type"` is not in the compiled
    /// dialect (or whose text is malformed); a wire-sourced message always yields its `Packet`.
    pub fn wire(&self) -> Option<&Packet> {
        self.wire
            .get_or_init(|| {
                let json = self.json.get()?.as_ref()?;
                Packet::from_json_transcoded(json)
            })
            .as_ref()
    }

    /// The MAVLinkJSON representation, transcoding it from the wire frame on first access if needed.
    ///
    /// Returns [`None`] only for a wire-sourced message whose id is not in the compiled dialect; a
    /// JSON-sourced message always yields its text.
    pub fn json(&self) -> Option<&Bytes> {
        self.json
            .get_or_init(|| {
                let packet = self.wire.get()?.as_ref()?;
                let mut buf = Vec::with_capacity(256);
                packet
                    .write_json_transcoded(&mut buf)
                    .then(|| Bytes::from(buf))
            })
            .as_ref()
    }

    /// The MAVLink message id, resolved as cheaply as possible.
    ///
    /// Uses the wire frame when available; otherwise resolves the JSON `"type"` tag against the
    /// dialect without transcoding the whole frame. Returns [`None`] for a JSON-sourced message
    /// with an unknown `"type"`.
    pub fn message_id(&self) -> Option<u32> {
        if let Some(Some(packet)) = self.wire.get() {
            return Some(packet.message_id());
        }
        let json = self.json.get()?.as_ref()?;
        let name = json_type_tag(json)?;
        generated::descriptor_by_name(name).map(|desc| desc.id)
    }

    /// The originating system id, resolved as cheaply as possible.
    ///
    /// Uses the wire header when available; otherwise reads the `"system_id"` member from the JSON
    /// header without transcoding the frame. Returns [`None`] only if neither side is present or the
    /// JSON is malformed.
    pub fn system_id(&self) -> Option<u8> {
        self.header_u8(b"system_id", |packet| *packet.system_id())
    }

    /// The originating component id, resolved as cheaply as possible (see [`Self::system_id`]).
    pub fn component_id(&self) -> Option<u8> {
        self.header_u8(b"component_id", |packet| *packet.component_id())
    }

    /// The MAVLink sequence number, resolved as cheaply as possible (see [`Self::system_id`]).
    pub fn sequence(&self) -> Option<u8> {
        self.header_u8(b"sequence", |packet| *packet.sequence())
    }

    /// Resolves a `u8` header field from the wire frame if present, else from the JSON header.
    fn header_u8(&self, key: &[u8], from_wire: fn(&Packet) -> u8) -> Option<u8> {
        if let Some(Some(packet)) = self.wire.get() {
            return Some(from_wire(packet));
        }
        let json = self.json.get()?.as_ref()?;
        json_header_number(json, key).map(|value| value as u8)
    }

    /// Whether the wire representation is already materialized (no transcoding on next `wire()`).
    pub fn has_wire(&self) -> bool {
        matches!(self.wire.get(), Some(Some(_)))
    }

    /// Whether the JSON representation is already materialized (no transcoding on next `json()`).
    pub fn has_json(&self) -> bool {
        matches!(self.json.get(), Some(Some(_)))
    }
}

impl Clone for MAVLinkMessage {
    /// Clones whichever representations are already materialized (each a cheap refcount bump),
    /// so a clone never re-transcodes what the original already computed.
    fn clone(&self) -> Self {
        let wire = OnceLock::new();
        if let Some(cached) = self.wire.get() {
            let _ = wire.set(cached.clone());
        }
        let json = OnceLock::new();
        if let Some(cached) = self.json.get() {
            let _ = json.set(cached.clone());
        }
        Self { wire, json }
    }
}

impl From<Packet> for MAVLinkMessage {
    fn from(packet: Packet) -> Self {
        Self::from_packet(packet)
    }
}

/// Returns the value of the first `"type":"NAME"` member (the MAVLinkJSON message tag, which serde
/// and the transcoder both emit before any field). Names carry no escapes, so the raw slice is
/// returned. Reads only up to the tag, avoiding a full parse.
fn json_type_tag(json: &[u8]) -> Option<&[u8]> {
    const NEEDLE: &[u8] = b"\"type\":";
    let mut i = 0;
    while i + NEEDLE.len() <= json.len() {
        if &json[i..i + NEEDLE.len()] == NEEDLE {
            let mut j = i + NEEDLE.len();
            while json.get(j)?.is_ascii_whitespace() {
                j += 1;
            }
            if *json.get(j)? != b'"' {
                return None;
            }
            j += 1;
            let start = j;
            while *json.get(j)? != b'"' {
                j += 1;
            }
            return Some(&json[start..j]);
        }
        i += 1;
    }
    None
}

/// Returns the unsigned integer value of the first `"<key>":<number>` member. The key is matched
/// quoted so it never collides with a message field of the same name (the header, which carries
/// `system_id`/`component_id`/`sequence`, is always serialized before the `"message"` object). Reads
/// only up to the value, avoiding a full parse.
fn json_header_number(json: &[u8], key: &[u8]) -> Option<u64> {
    let mut i = 0;
    while i < json.len() {
        // Match the exact token `"<key>":`, requiring the surrounding quotes and colon.
        if json[i] == b'"'
            && json[i + 1..].starts_with(key)
            && json.get(i + 1 + key.len()) == Some(&b'"')
        {
            let mut j = i + 1 + key.len() + 1;
            while json.get(j)?.is_ascii_whitespace() {
                j += 1;
            }
            if *json.get(j)? != b':' {
                return None;
            }
            j += 1;
            while json.get(j)?.is_ascii_whitespace() {
                j += 1;
            }
            let mut value: u64 = 0;
            let mut any = false;
            while let Some(d @ b'0'..=b'9') = json.get(j) {
                value = value * 10 + u64::from(d - b'0');
                any = true;
                j += 1;
            }
            return any.then_some(value);
        }
        i += 1;
    }
    None
}
