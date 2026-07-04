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
    use bytes::Bytes;

    use crate::{v2::V2Packet, Packet};

    /// MAVLink message id for `GLOBAL_POSITION_INT`.
    pub const GLOBAL_POSITION_INT_ID: u32 = 33;
    const GLOBAL_POSITION_INT_PAYLOAD_LEN: usize = 28;

    /// Reverse Option C spike: parses a `GLOBAL_POSITION_INT` MAVLinkJSON string straight into a
    /// wire [`Packet`], with no serde and no typed struct.
    ///
    /// Produces the exact same frame bytes as `serde_json::from_str::<MAVLinkJSON<_>>(json)`
    /// followed by `to_packet(MavlinkVersion::V2)`. The scanner is whitespace- and
    /// order-tolerant; integer values are written directly into the wire payload and the frame
    /// header, truncation and CRC are built by hand.
    pub fn global_position_int_from_json(json: &[u8]) -> Packet {
        let mut system_id = 0u8;
        let mut component_id = 0u8;
        let mut sequence = 0u8;
        // time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg
        let mut fields = [0i64; 9];

        let n = json.len();
        let mut i = 0;
        while i < n {
            if json[i] != b'"' {
                i += 1;
                continue;
            }

            // Read a quoted token.
            let key_start = i + 1;
            let mut j = key_start;
            while j < n && json[j] != b'"' {
                j += 1;
            }
            let key = &json[key_start..j];
            i = j + 1;

            // A key is a quoted token followed by ':'. Otherwise it was a string value.
            while i < n && json[i].is_ascii_whitespace() {
                i += 1;
            }
            if i >= n || json[i] != b':' {
                continue;
            }
            i += 1;
            while i < n && json[i].is_ascii_whitespace() {
                i += 1;
            }
            if i >= n {
                break;
            }

            match json[i] {
                b'"' => {
                    // String value (e.g. the "type" tag): skip it.
                    i += 1;
                    while i < n && json[i] != b'"' {
                        i += 1;
                    }
                    i += 1;
                }
                b'{' | b'[' => {
                    // Descend into the nested object/array to find inner keys.
                    i += 1;
                }
                _ => {
                    let num_start = i;
                    while i < n && matches!(json[i], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9')
                    {
                        i += 1;
                    }
                    let value = parse_i64(&json[num_start..i]);
                    match key {
                        b"system_id" => system_id = value as u8,
                        b"component_id" => component_id = value as u8,
                        b"sequence" => sequence = value as u8,
                        b"time_boot_ms" => fields[0] = value,
                        b"lat" => fields[1] = value,
                        b"lon" => fields[2] = value,
                        b"alt" => fields[3] = value,
                        b"relative_alt" => fields[4] = value,
                        b"vx" => fields[5] = value,
                        b"vy" => fields[6] = value,
                        b"vz" => fields[7] = value,
                        b"hdg" => fields[8] = value,
                        _ => {}
                    }
                }
            }
        }

        let mut payload = [0u8; GLOBAL_POSITION_INT_PAYLOAD_LEN];
        payload[0..4].copy_from_slice(&(fields[0] as u32).to_le_bytes());
        payload[4..8].copy_from_slice(&(fields[1] as i32).to_le_bytes());
        payload[8..12].copy_from_slice(&(fields[2] as i32).to_le_bytes());
        payload[12..16].copy_from_slice(&(fields[3] as i32).to_le_bytes());
        payload[16..20].copy_from_slice(&(fields[4] as i32).to_le_bytes());
        payload[20..22].copy_from_slice(&(fields[5] as i16).to_le_bytes());
        payload[22..24].copy_from_slice(&(fields[6] as i16).to_le_bytes());
        payload[24..26].copy_from_slice(&(fields[7] as i16).to_le_bytes());
        payload[26..28].copy_from_slice(&(fields[8] as u16).to_le_bytes());

        build_v2_frame(
            GLOBAL_POSITION_INT_ID,
            system_id,
            component_id,
            sequence,
            &payload,
        )
    }

    /// Builds a MAVLink v2 frame (STX, header, truncated payload, CRC) byte-identical to
    /// rust-mavlink's `serialize_message`.
    fn build_v2_frame(
        msgid: u32,
        system_id: u8,
        component_id: u8,
        sequence: u8,
        payload: &[u8],
    ) -> Packet {
        // v2 trims trailing zero bytes of the payload.
        let mut len = payload.len();
        while len > 0 && payload[len - 1] == 0 {
            len -= 1;
        }

        let msgid_bytes = msgid.to_le_bytes();
        let mut frame =
            Vec::with_capacity(1 + V2Packet::HEADER_SIZE + len + V2Packet::CHECKSUM_SIZE);
        frame.push(crate::v2::V2_STX);
        frame.push(len as u8);
        frame.push(0); // incompat flags
        frame.push(0); // compat flags
        frame.push(sequence);
        frame.push(system_id);
        frame.push(component_id);
        frame.extend_from_slice(&msgid_bytes[0..3]);
        frame.extend_from_slice(&payload[..len]);

        let extra_crc = crate::codec::get_extra_crc(msgid).unwrap_or(0);
        let crc = mavlink::calculate_crc(&frame[1..], extra_crc);
        frame.extend_from_slice(&crc.to_le_bytes());

        Packet::V2(V2Packet::new(Bytes::from(frame)))
    }

    #[inline(always)]
    fn parse_i64(s: &[u8]) -> i64 {
        let (neg, digits) = match s.first() {
            Some(b'-') => (true, &s[1..]),
            _ => (false, s),
        };
        let mut value = 0i64;
        for &b in digits {
            if b.is_ascii_digit() {
                value = value * 10 + (b - b'0') as i64;
            } else {
                break;
            }
        }
        if neg {
            -value
        } else {
            value
        }
    }

    /// Parses a JSON float slice into an `f32`. Rust's parser is correctly-rounding, so the
    /// shortest round-trippable text emitted by `zmij`/serde_json reproduces the exact bits.
    #[inline(always)]
    fn parse_f32(s: &[u8]) -> f32 {
        core::str::from_utf8(s)
            .ok()
            .and_then(|s| s.parse::<f32>().ok())
            .unwrap_or(0.0)
    }

    /// Reverse Option C spike for the float value type: parses an `ATTITUDE` MAVLinkJSON string
    /// straight into a wire [`Packet`], byte-identical to the serde baseline.
    ///
    /// Only handles finite floats: the serde baseline itself cannot deserialize the `null` that
    /// non-finite floats serialize to, so those never round-trip either way.
    pub fn attitude_from_json(json: &[u8]) -> Packet {
        let mut system_id = 0u8;
        let mut component_id = 0u8;
        let mut sequence = 0u8;
        let mut time_boot_ms = 0u32;
        // roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        let mut floats = [0f32; 6];

        let mut scanner = Scanner::new(json);
        while let Some((key, value)) = scanner.next_scalar() {
            match value {
                Value::Number(num) => match key {
                    b"system_id" => system_id = parse_i64(num) as u8,
                    b"component_id" => component_id = parse_i64(num) as u8,
                    b"sequence" => sequence = parse_i64(num) as u8,
                    b"time_boot_ms" => time_boot_ms = parse_i64(num) as u32,
                    b"roll" => floats[0] = parse_f32(num),
                    b"pitch" => floats[1] = parse_f32(num),
                    b"yaw" => floats[2] = parse_f32(num),
                    b"rollspeed" => floats[3] = parse_f32(num),
                    b"pitchspeed" => floats[4] = parse_f32(num),
                    b"yawspeed" => floats[5] = parse_f32(num),
                    _ => {}
                },
                Value::String(_) | Value::EnterObject | Value::EnterArray => {}
            }
        }

        let mut payload = [0u8; ATTITUDE_PAYLOAD_LEN];
        payload[0..4].copy_from_slice(&time_boot_ms.to_le_bytes());
        payload[4..8].copy_from_slice(&floats[0].to_le_bytes());
        payload[8..12].copy_from_slice(&floats[1].to_le_bytes());
        payload[12..16].copy_from_slice(&floats[2].to_le_bytes());
        payload[16..20].copy_from_slice(&floats[3].to_le_bytes());
        payload[20..24].copy_from_slice(&floats[4].to_le_bytes());
        payload[24..28].copy_from_slice(&floats[5].to_le_bytes());

        build_v2_frame(ATTITUDE_ID, system_id, component_id, sequence, &payload)
    }

    /// Reverse Option C spike for the enum and bitflag value types: parses a `HEARTBEAT`
    /// MAVLinkJSON string straight into a wire [`Packet`], byte-identical to the serde baseline.
    ///
    /// Enum names (`{"type":"NAME"}`) and the `base_mode` bitflag string are reverse-looked-up
    /// through the same static name tables used by the forward transcoder.
    pub fn heartbeat_from_json(json: &[u8]) -> Packet {
        let mut system_id = 0u8;
        let mut component_id = 0u8;
        let mut sequence = 0u8;
        let mut custom_mode = 0u32;
        let mut mavtype = 0u8;
        let mut autopilot = 0u8;
        let mut base_mode = 0u8;
        let mut system_status = 0u8;
        let mut mavlink_version = 0u8;

        // The `"type"` key collides between the message tag and each nested enum; track which
        // enum object we most recently descended into to route its inner `"type"` string.
        let mut enum_ctx = EnumCtx::None;

        let mut scanner = Scanner::new(json);
        while let Some((key, value)) = scanner.next_scalar() {
            match value {
                Value::Number(num) => match key {
                    b"system_id" => system_id = parse_i64(num) as u8,
                    b"component_id" => component_id = parse_i64(num) as u8,
                    b"sequence" => sequence = parse_i64(num) as u8,
                    b"custom_mode" => custom_mode = parse_i64(num) as u32,
                    b"mavlink_version" => mavlink_version = parse_i64(num) as u8,
                    _ => {}
                },
                Value::String(s) => match key {
                    b"type" => match enum_ctx {
                        EnumCtx::MavType => mavtype = lookup_name(&MAV_TYPE_NAMES, s),
                        EnumCtx::Autopilot => autopilot = lookup_name(&MAV_AUTOPILOT_NAMES, s),
                        EnumCtx::State => system_status = lookup_name(&MAV_STATE_NAMES, s),
                        EnumCtx::None => {}
                    },
                    b"base_mode" => base_mode = parse_base_mode(s),
                    _ => {}
                },
                Value::EnterObject => {
                    enum_ctx = match key {
                        b"mavtype" => EnumCtx::MavType,
                        b"autopilot" => EnumCtx::Autopilot,
                        b"system_status" => EnumCtx::State,
                        _ => enum_ctx,
                    };
                }
                Value::EnterArray => {}
            }
        }

        let mut payload = [0u8; HEARTBEAT_PAYLOAD_LEN];
        payload[0..4].copy_from_slice(&custom_mode.to_le_bytes());
        payload[4] = mavtype;
        payload[5] = autopilot;
        payload[6] = base_mode;
        payload[7] = system_status;
        payload[8] = mavlink_version;

        build_v2_frame(HEARTBEAT_ID, system_id, component_id, sequence, &payload)
    }

    enum EnumCtx {
        None,
        MavType,
        Autopilot,
        State,
    }

    /// Reverse of [`name_or_empty`]: finds the integer value whose name equals `s`.
    #[inline(always)]
    fn lookup_name(table: &[&'static [u8]], s: &[u8]) -> u8 {
        table.iter().position(|name| *name == s).unwrap_or(0) as u8
    }

    /// Reverse of [`put_base_mode`]: `" | "`-joined flag names back into a bitmask.
    fn parse_base_mode(s: &[u8]) -> u8 {
        if s.is_empty() {
            return 0;
        }
        let mut bits = 0u8;
        for name in s.split(|&b| b == b'|') {
            let name = trim_ascii(name);
            if let Some(index) = MAV_MODE_FLAG_NAMES.iter().position(|flag| *flag == name) {
                bits |= 1u8 << index;
            }
        }
        bits
    }

    #[inline(always)]
    fn trim_ascii(mut s: &[u8]) -> &[u8] {
        while let [first, rest @ ..] = s {
            if first.is_ascii_whitespace() {
                s = rest;
            } else {
                break;
            }
        }
        while let [rest @ .., last] = s {
            if last.is_ascii_whitespace() {
                s = rest;
            } else {
                break;
            }
        }
        s
    }

    /// A whitespace- and order-tolerant scalar-key/value scanner over MAVLinkJSON text.
    ///
    /// It walks the byte stream, descending into nested objects/arrays, and yields each
    /// `"key": <scalar>` pair (or the fact that a key opens an object/array). This is enough for
    /// the flat MAVLinkJSON layout without a full JSON parser.
    struct Scanner<'a> {
        bytes: &'a [u8],
        pos: usize,
    }

    enum Value<'a> {
        Number(&'a [u8]),
        String(&'a [u8]),
        EnterObject,
        EnterArray,
    }

    impl<'a> Scanner<'a> {
        #[inline(always)]
        fn new(bytes: &'a [u8]) -> Self {
            Self { bytes, pos: 0 }
        }

        fn next_scalar(&mut self) -> Option<(&'a [u8], Value<'a>)> {
            let n = self.bytes.len();
            while self.pos < n {
                if self.bytes[self.pos] != b'"' {
                    self.pos += 1;
                    continue;
                }

                let key = self.read_string();
                self.skip_ws();
                if self.pos >= n || self.bytes[self.pos] != b':' {
                    // The quoted token was a string value, not a key.
                    continue;
                }
                self.pos += 1;
                self.skip_ws();
                if self.pos >= n {
                    break;
                }

                let value = match self.bytes[self.pos] {
                    b'"' => Value::String(self.read_string()),
                    b'{' => {
                        self.pos += 1;
                        Value::EnterObject
                    }
                    b'[' => {
                        self.pos += 1;
                        Value::EnterArray
                    }
                    _ => Value::Number(self.read_number()),
                };
                return Some((key, value));
            }
            None
        }

        /// Reads a quoted token, leaving `pos` past the closing quote. Assumes the current byte
        /// is the opening quote.
        #[inline(always)]
        fn read_string(&mut self) -> &'a [u8] {
            let n = self.bytes.len();
            let start = self.pos + 1;
            let mut end = start;
            while end < n && self.bytes[end] != b'"' {
                end += 1;
            }
            self.pos = (end + 1).min(n);
            &self.bytes[start..end]
        }

        #[inline(always)]
        fn read_number(&mut self) -> &'a [u8] {
            let n = self.bytes.len();
            let start = self.pos;
            while self.pos < n
                && matches!(
                    self.bytes[self.pos],
                    b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9'
                )
            {
                self.pos += 1;
            }
            &self.bytes[start..self.pos]
        }

        #[inline(always)]
        fn skip_ws(&mut self) {
            while self.pos < self.bytes.len() && self.bytes[self.pos].is_ascii_whitespace() {
                self.pos += 1;
            }
        }
    }

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

        put_header(packet, out);
        out.extend_from_slice(br#","message":{"type":"GLOBAL_POSITION_INT","time_boot_ms":"#);
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

    /// MAVLink message id for `ATTITUDE`.
    pub const ATTITUDE_ID: u32 = 30;
    const ATTITUDE_PAYLOAD_LEN: usize = 28;

    /// Transcodes an `ATTITUDE` frame straight to JSON (covers the float value type).
    ///
    /// Floats are formatted with `ryu` (the same primitive serde_json uses); non-finite values
    /// are emitted as `null`, matching serde_json.
    pub fn attitude_to_json(packet: &Packet, out: &mut Vec<u8>) {
        let mut p = [0u8; ATTITUDE_PAYLOAD_LEN];
        let payload = packet.payload();
        let n = payload.len().min(ATTITUDE_PAYLOAD_LEN);
        p[..n].copy_from_slice(&payload[..n]);

        let time_boot_ms = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
        let read_f32 = |o: usize| f32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);

        put_header(packet, out);
        out.extend_from_slice(br#","message":{"type":"ATTITUDE","time_boot_ms":"#);
        put_int(out, time_boot_ms);
        out.extend_from_slice(br#","roll":"#);
        put_f32(out, read_f32(4));
        out.extend_from_slice(br#","pitch":"#);
        put_f32(out, read_f32(8));
        out.extend_from_slice(br#","yaw":"#);
        put_f32(out, read_f32(12));
        out.extend_from_slice(br#","rollspeed":"#);
        put_f32(out, read_f32(16));
        out.extend_from_slice(br#","pitchspeed":"#);
        put_f32(out, read_f32(20));
        out.extend_from_slice(br#","yawspeed":"#);
        put_f32(out, read_f32(24));
        out.extend_from_slice(br#"}}"#);
    }

    /// MAVLink message id for `GPS_STATUS`.
    pub const GPS_STATUS_ID: u32 = 25;
    const GPS_STATUS_PAYLOAD_LEN: usize = 101;

    /// Transcodes a `GPS_STATUS` frame straight to JSON (covers the numeric-array value type).
    pub fn gps_status_to_json(packet: &Packet, out: &mut Vec<u8>) {
        let mut p = [0u8; GPS_STATUS_PAYLOAD_LEN];
        let payload = packet.payload();
        let n = payload.len().min(GPS_STATUS_PAYLOAD_LEN);
        p[..n].copy_from_slice(&payload[..n]);

        put_header(packet, out);
        out.extend_from_slice(br#","message":{"type":"GPS_STATUS","satellites_visible":"#);
        put_int(out, p[0]);
        out.extend_from_slice(br#","satellite_prn":"#);
        put_u8_array(out, &p[1..21]);
        out.extend_from_slice(br#","satellite_used":"#);
        put_u8_array(out, &p[21..41]);
        out.extend_from_slice(br#","satellite_elevation":"#);
        put_u8_array(out, &p[41..61]);
        out.extend_from_slice(br#","satellite_azimuth":"#);
        put_u8_array(out, &p[61..81]);
        out.extend_from_slice(br#","satellite_snr":"#);
        put_u8_array(out, &p[81..101]);
        out.extend_from_slice(br#"}}"#);
    }

    /// MAVLink message id for `HEARTBEAT`.
    pub const HEARTBEAT_ID: u32 = 0;
    const HEARTBEAT_PAYLOAD_LEN: usize = 9;

    /// Transcodes a `HEARTBEAT` frame straight to JSON (covers the enum and bitflag value types).
    ///
    /// Enums render as `{"type":"NAME"}` via static name tables; the `base_mode` bitflags render
    /// as a `" | "`-joined string in descending bit order (empty when no bits are set), matching
    /// serde_json.
    pub fn heartbeat_to_json(packet: &Packet, out: &mut Vec<u8>) {
        let mut p = [0u8; HEARTBEAT_PAYLOAD_LEN];
        let payload = packet.payload();
        let n = payload.len().min(HEARTBEAT_PAYLOAD_LEN);
        p[..n].copy_from_slice(&payload[..n]);

        let custom_mode = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
        let mavtype = p[4];
        let autopilot = p[5];
        let base_mode = p[6];
        let system_status = p[7];
        let mavlink_version = p[8];

        put_header(packet, out);
        out.extend_from_slice(br#","message":{"type":"HEARTBEAT","custom_mode":"#);
        put_int(out, custom_mode);
        out.extend_from_slice(br#","mavtype":{"type":"#);
        put_str(out, name_or_empty(&MAV_TYPE_NAMES, mavtype));
        out.extend_from_slice(br#"},"autopilot":{"type":"#);
        put_str(out, name_or_empty(&MAV_AUTOPILOT_NAMES, autopilot));
        out.extend_from_slice(br#"},"base_mode":""#);
        put_base_mode(out, base_mode);
        out.extend_from_slice(br#"","system_status":{"type":"#);
        put_str(out, name_or_empty(&MAV_STATE_NAMES, system_status));
        out.extend_from_slice(br#"},"mavlink_version":"#);
        put_int(out, mavlink_version);
        out.extend_from_slice(br#"}}"#);
    }

    #[inline(always)]
    fn put_header(packet: &Packet, out: &mut Vec<u8>) {
        out.extend_from_slice(br#"{"header":{"system_id":"#);
        put_int(out, *packet.system_id());
        out.extend_from_slice(br#","component_id":"#);
        put_int(out, *packet.component_id());
        out.extend_from_slice(br#","sequence":"#);
        put_int(out, *packet.sequence());
        out.extend_from_slice(br#","message_id":"#);
        put_int(out, packet.message_id());
        out.extend_from_slice(br#"}"#);
    }

    #[inline(always)]
    fn put_int<I: itoa::Integer>(out: &mut Vec<u8>, value: I) {
        let mut buffer = itoa::Buffer::new();
        out.extend_from_slice(buffer.format(value).as_bytes());
    }

    #[inline(always)]
    fn put_f32(out: &mut Vec<u8>, value: f32) {
        if value.is_finite() {
            let mut buffer = zmij::Buffer::new();
            out.extend_from_slice(buffer.format_finite(value).as_bytes());
        } else {
            out.extend_from_slice(b"null");
        }
    }

    #[inline(always)]
    fn put_str(out: &mut Vec<u8>, value: &[u8]) {
        // Enum/flag names are ASCII identifiers with no JSON-escapable characters.
        out.push(b'"');
        out.extend_from_slice(value);
        out.push(b'"');
    }

    #[inline(always)]
    fn put_u8_array(out: &mut Vec<u8>, values: &[u8]) {
        out.push(b'[');
        for (i, value) in values.iter().enumerate() {
            if i != 0 {
                out.push(b',');
            }
            put_int(out, *value);
        }
        out.push(b']');
    }

    #[inline(always)]
    fn name_or_empty(table: &[&'static [u8]], value: u8) -> &'static [u8] {
        table.get(value as usize).copied().unwrap_or(b"")
    }

    fn put_base_mode(out: &mut Vec<u8>, bits: u8) {
        let mut first = true;
        for bit_index in (0..8u8).rev() {
            let mask = 1u8 << bit_index;
            if bits & mask != 0 {
                if !first {
                    out.extend_from_slice(b" | ");
                }
                first = false;
                out.extend_from_slice(MAV_MODE_FLAG_NAMES[bit_index as usize]);
            }
        }
    }

    static MAV_TYPE_NAMES: [&[u8]; 50] = [
        b"MAV_TYPE_GENERIC",
        b"MAV_TYPE_FIXED_WING",
        b"MAV_TYPE_QUADROTOR",
        b"MAV_TYPE_COAXIAL",
        b"MAV_TYPE_HELICOPTER",
        b"MAV_TYPE_ANTENNA_TRACKER",
        b"MAV_TYPE_GCS",
        b"MAV_TYPE_AIRSHIP",
        b"MAV_TYPE_FREE_BALLOON",
        b"MAV_TYPE_ROCKET",
        b"MAV_TYPE_GROUND_ROVER",
        b"MAV_TYPE_SURFACE_BOAT",
        b"MAV_TYPE_SUBMARINE",
        b"MAV_TYPE_HEXAROTOR",
        b"MAV_TYPE_OCTOROTOR",
        b"MAV_TYPE_TRICOPTER",
        b"MAV_TYPE_FLAPPING_WING",
        b"MAV_TYPE_KITE",
        b"MAV_TYPE_ONBOARD_CONTROLLER",
        b"MAV_TYPE_VTOL_TAILSITTER_DUOROTOR",
        b"MAV_TYPE_VTOL_TAILSITTER_QUADROTOR",
        b"MAV_TYPE_VTOL_TILTROTOR",
        b"MAV_TYPE_VTOL_FIXEDROTOR",
        b"MAV_TYPE_VTOL_TAILSITTER",
        b"MAV_TYPE_VTOL_TILTWING",
        b"MAV_TYPE_VTOL_RESERVED5",
        b"MAV_TYPE_GIMBAL",
        b"MAV_TYPE_ADSB",
        b"MAV_TYPE_PARAFOIL",
        b"MAV_TYPE_DODECAROTOR",
        b"MAV_TYPE_CAMERA",
        b"MAV_TYPE_CHARGING_STATION",
        b"MAV_TYPE_FLARM",
        b"MAV_TYPE_SERVO",
        b"MAV_TYPE_ODID",
        b"MAV_TYPE_DECAROTOR",
        b"MAV_TYPE_BATTERY",
        b"MAV_TYPE_PARACHUTE",
        b"MAV_TYPE_LOG",
        b"MAV_TYPE_OSD",
        b"MAV_TYPE_IMU",
        b"MAV_TYPE_GPS",
        b"MAV_TYPE_WINCH",
        b"MAV_TYPE_GENERIC_MULTIROTOR",
        b"MAV_TYPE_ILLUMINATOR",
        b"MAV_TYPE_SPACECRAFT_ORBITER",
        b"MAV_TYPE_GROUND_QUADRUPED",
        b"MAV_TYPE_VTOL_GYRODYNE",
        b"MAV_TYPE_GRIPPER",
        b"MAV_TYPE_RADIO",
    ];

    static MAV_AUTOPILOT_NAMES: [&[u8]; 21] = [
        b"MAV_AUTOPILOT_GENERIC",
        b"MAV_AUTOPILOT_RESERVED",
        b"MAV_AUTOPILOT_SLUGS",
        b"MAV_AUTOPILOT_ARDUPILOTMEGA",
        b"MAV_AUTOPILOT_OPENPILOT",
        b"MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY",
        b"MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY",
        b"MAV_AUTOPILOT_GENERIC_MISSION_FULL",
        b"MAV_AUTOPILOT_INVALID",
        b"MAV_AUTOPILOT_PPZ",
        b"MAV_AUTOPILOT_UDB",
        b"MAV_AUTOPILOT_FP",
        b"MAV_AUTOPILOT_PX4",
        b"MAV_AUTOPILOT_SMACCMPILOT",
        b"MAV_AUTOPILOT_AUTOQUAD",
        b"MAV_AUTOPILOT_ARMAZILA",
        b"MAV_AUTOPILOT_AEROB",
        b"MAV_AUTOPILOT_ASLUAV",
        b"MAV_AUTOPILOT_SMARTAP",
        b"MAV_AUTOPILOT_AIRRAILS",
        b"MAV_AUTOPILOT_REFLEX",
    ];

    static MAV_STATE_NAMES: [&[u8]; 9] = [
        b"MAV_STATE_UNINIT",
        b"MAV_STATE_BOOT",
        b"MAV_STATE_CALIBRATING",
        b"MAV_STATE_STANDBY",
        b"MAV_STATE_ACTIVE",
        b"MAV_STATE_CRITICAL",
        b"MAV_STATE_EMERGENCY",
        b"MAV_STATE_POWEROFF",
        b"MAV_STATE_FLIGHT_TERMINATION",
    ];

    /// Indexed by bit position (0..8), i.e. `MAV_MODE_FLAG_NAMES[i]` is the name of bit `1 << i`.
    static MAV_MODE_FLAG_NAMES: [&[u8]; 8] = [
        b"MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
        b"MAV_MODE_FLAG_TEST_ENABLED",
        b"MAV_MODE_FLAG_AUTO_ENABLED",
        b"MAV_MODE_FLAG_GUIDED_ENABLED",
        b"MAV_MODE_FLAG_STABILIZE_ENABLED",
        b"MAV_MODE_FLAG_HIL_ENABLED",
        b"MAV_MODE_FLAG_MANUAL_INPUT_ENABLED",
        b"MAV_MODE_FLAG_SAFETY_ARMED",
    ];
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
