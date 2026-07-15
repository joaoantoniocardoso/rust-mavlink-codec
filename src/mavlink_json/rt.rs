//! Runtime support for the generated MAVLinkJSON transcoders.
//!
//! `build.rs` emits, for every message in the dialect, a pure-data [`MsgDesc`] descriptor
//! (field names, wire offsets, and per-field [`FieldKind`]s plus enum/bitmask name tables). The
//! generic interpreter here walks a descriptor once and writes the MAVLinkJSON text, byte
//! identical to the `rust-mavlink` + `serde_json` baseline. Keeping the logic here (and only the
//! data generated) keeps the generated code tiny and lets the whole dialect share one hot loop.

use bytes::Bytes;
use mavlink::MavlinkVersion;

use crate::v1::{V1Packet, V1_STX};
use crate::v2::{V2Packet, V2_STX};
use crate::{Packet, PacketRef};

/// A whole message: its id, wire name (the serde `"type"` tag) and ordered fields.
///
/// `fields` are in MAVLink serialization order (base fields sorted by size, then extensions),
/// which is exactly the order `serde` emits them, so a single pass produces matching JSON.
pub struct MsgDesc {
    pub id: u32,
    pub name: &'static str,
    /// Full payload length including MAVLink v2 extension fields (used for v2 framing).
    pub payload_len: u16,
    /// Payload length of the base (non-extension) fields only, which is exactly what a MAVLink v1
    /// frame carries (v1 has no extensions and no trailing-zero truncation).
    pub base_len: u16,
    /// MAVLink `CRC_EXTRA` seed byte, baked at build time (rust-mavlink's `extra_crc`).
    pub crc_extra: u8,
    pub fields: &'static [FieldDesc],
}

/// One field: its JSON key, byte offset into the full payload, and how to render it.
pub struct FieldDesc {
    pub name: &'static str,
    pub offset: u16,
    pub kind: FieldKind,
}

/// How a field's value is read from the wire and written as JSON.
#[derive(Clone, Copy)]
pub enum FieldKind {
    /// A single numeric value.
    Scalar(ScalarKind),
    /// A `char[N]` rendered as a JSON string (NUL-trimmed, like `serde`).
    CharArray(u16),
    /// A fixed numeric array rendered as a JSON array.
    Array(ScalarKind, u16),
    /// An enum rendered as `{"type":"NAME"}`, read as the given primitive.
    Enum(&'static [(u64, &'static str)], ScalarKind),
    /// A bitmask rendered as a `" | "`-joined string, read as the given primitive.
    Bitmask(&'static [(u64, &'static str)], ScalarKind),
}

/// The wire primitive underlying a scalar / array element / enum discriminant.
#[derive(Clone, Copy)]
pub enum ScalarKind {
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    F32,
    F64,
}

/// Transcodes `packet` to MAVLinkJSON text using `desc`, appending to `out`.
pub fn to_json(packet: PacketRef<'_>, desc: &MsgDesc, out: &mut Vec<u8>) {
    write(packet, desc, out, &mut NoRec);
}

/// Like [`to_json`] but also records the byte range of each field's rendered value into `ranges`
/// (parallel to `desc.fields`), enabling zero-copy per-field `Bytes::slice` fan-out.
pub fn to_json_indexed(
    packet: PacketRef<'_>,
    desc: &MsgDesc,
    out: &mut Vec<u8>,
    ranges: &mut [(u32, u32)],
) {
    write(packet, desc, out, &mut SliceRec(ranges));
}

trait Recorder {
    fn record(&mut self, idx: usize, start: u32, end: u32);
}

struct NoRec;
impl Recorder for NoRec {
    #[inline(always)]
    fn record(&mut self, _idx: usize, _start: u32, _end: u32) {}
}

struct SliceRec<'a>(&'a mut [(u32, u32)]);
impl Recorder for SliceRec<'_> {
    #[inline(always)]
    fn record(&mut self, idx: usize, start: u32, end: u32) {
        self.0[idx] = (start, end);
    }
}

fn write<R: Recorder>(packet: PacketRef<'_>, desc: &MsgDesc, out: &mut Vec<u8>, rec: &mut R) {
    // Read straight from the wire payload: MAVLink v2 only trims *trailing zero* bytes, so any
    // byte past the truncated length reads back as zero (see `rd`). This avoids copying/zeroing a
    // scratch buffer per message.
    let p: &[u8] = packet.payload();

    put_header(packet, out);
    out.extend_from_slice(br#","message":{"type":""#);
    out.extend_from_slice(desc.name.as_bytes());
    out.push(b'"');

    for (i, field) in desc.fields.iter().enumerate() {
        out.push(b',');
        out.push(b'"');
        out.extend_from_slice(field.name.as_bytes());
        out.extend_from_slice(br#"":"#);
        let start = out.len() as u32;
        put_field(p, field, out);
        rec.record(i, start, out.len() as u32);
    }

    out.extend_from_slice(br#"}}"#);
}

#[inline]
fn put_field(p: &[u8], field: &FieldDesc, out: &mut Vec<u8>) {
    let off = field.offset as usize;
    match field.kind {
        FieldKind::Scalar(sk) => put_scalar(p, off, sk, out),
        FieldKind::CharArray(len) => put_char_array(p, off, len as usize, out),
        FieldKind::Array(sk, len) => put_array(p, off, sk, len as usize, out),
        FieldKind::Enum(table, sk) => {
            let value = read_u64(p, off, sk);
            // A router may be behind the sender: an enum value we don't know yet would make the
            // typed serde path error and drop the frame. Instead, emit the raw number so the value
            // propagates (and round-trips through the reverse parser).
            match enum_name(table, value) {
                Some(name) => {
                    out.extend_from_slice(br#"{"type":""#);
                    out.extend_from_slice(name);
                    out.extend_from_slice(br#""}"#);
                }
                None => put_int(out, value),
            }
        }
        FieldKind::Bitmask(table, sk) => {
            let bits = read_u64(p, off, sk);
            out.push(b'"');
            put_bitmask(table, bits, out);
            out.push(b'"');
        }
    }
}

#[inline]
fn put_scalar(p: &[u8], off: usize, sk: ScalarKind, out: &mut Vec<u8>) {
    match sk {
        ScalarKind::U8 => put_int(out, rd(p, off)),
        ScalarKind::U16 => put_int(out, u16::from_le_bytes(rd_n(p, off))),
        ScalarKind::U32 => put_int(out, u32::from_le_bytes(rd_n(p, off))),
        ScalarKind::U64 => put_int(out, u64::from_le_bytes(rd_n(p, off))),
        ScalarKind::I8 => put_int(out, rd(p, off) as i8),
        ScalarKind::I16 => put_int(out, i16::from_le_bytes(rd_n(p, off))),
        ScalarKind::I32 => put_int(out, i32::from_le_bytes(rd_n(p, off))),
        ScalarKind::I64 => put_int(out, i64::from_le_bytes(rd_n(p, off))),
        ScalarKind::F32 => put_f32(out, f32::from_le_bytes(rd_n(p, off))),
        ScalarKind::F64 => put_f64(out, f64::from_le_bytes(rd_n(p, off))),
    }
}

/// Renders a fixed numeric array as a JSON array. The element-kind `match` is hoisted out of the
/// loop so each element hits a monomorphic tight loop (crucial for the many-element MAVLink
/// arrays like `GPS_STATUS`).
#[inline]
fn put_array(p: &[u8], off: usize, sk: ScalarKind, len: usize, out: &mut Vec<u8>) {
    out.push(b'[');
    macro_rules! loop_int {
        ($ty:ty, $sz:expr) => {{
            for k in 0..len {
                if k != 0 {
                    out.push(b',');
                }
                put_int(out, <$ty>::from_le_bytes(rd_n(p, off + k * $sz)));
            }
        }};
    }
    match sk {
        ScalarKind::U8 => {
            for k in 0..len {
                if k != 0 {
                    out.push(b',');
                }
                put_int(out, rd(p, off + k));
            }
        }
        ScalarKind::I8 => {
            for k in 0..len {
                if k != 0 {
                    out.push(b',');
                }
                put_int(out, rd(p, off + k) as i8);
            }
        }
        ScalarKind::U16 => loop_int!(u16, 2),
        ScalarKind::I16 => loop_int!(i16, 2),
        ScalarKind::U32 => loop_int!(u32, 4),
        ScalarKind::I32 => loop_int!(i32, 4),
        ScalarKind::U64 => loop_int!(u64, 8),
        ScalarKind::I64 => loop_int!(i64, 8),
        ScalarKind::F32 => {
            for k in 0..len {
                if k != 0 {
                    out.push(b',');
                }
                put_f32(out, f32::from_le_bytes(rd_n(p, off + k * 4)));
            }
        }
        ScalarKind::F64 => {
            for k in 0..len {
                if k != 0 {
                    out.push(b',');
                }
                put_f64(out, f64::from_le_bytes(rd_n(p, off + k * 8)));
            }
        }
    }
    out.push(b']');
}

#[inline(always)]
fn read_u64(p: &[u8], off: usize, sk: ScalarKind) -> u64 {
    match sk {
        ScalarKind::U8 | ScalarKind::I8 => rd(p, off) as u64,
        ScalarKind::U16 | ScalarKind::I16 => u16::from_le_bytes(rd_n(p, off)) as u64,
        ScalarKind::U32 | ScalarKind::I32 => u32::from_le_bytes(rd_n(p, off)) as u64,
        ScalarKind::U64 | ScalarKind::I64 => u64::from_le_bytes(rd_n(p, off)),
        // Enums/bitmasks are never float-backed in MAVLink.
        ScalarKind::F32 | ScalarKind::F64 => 0,
    }
}

/// Reads one payload byte, returning 0 past the (possibly v2-truncated) end.
#[inline(always)]
fn rd(p: &[u8], i: usize) -> u8 {
    p.get(i).copied().unwrap_or(0)
}

/// Reads `N` little-endian payload bytes, zero-filling past the end.
#[inline(always)]
fn rd_n<const N: usize>(p: &[u8], off: usize) -> [u8; N] {
    let mut b = [0u8; N];
    let end = (off + N).min(p.len());
    if off < end {
        b[..end - off].copy_from_slice(&p[off..end]);
    }
    b
}

#[inline(always)]
fn enum_name(table: &[(u64, &'static str)], value: u64) -> Option<&'static [u8]> {
    for (v, name) in table {
        if *v == value {
            return Some(name.as_bytes());
        }
    }
    None
}

/// Emits contained flags in declaration order joined by `" | "`, matching `bitflags` v2's `serde`
/// string representation used by rust-mavlink. Bits not covered by any known flag are preserved as
/// a trailing lowercase `0x..` hex literal (rust-mavlink reads bitmasks with `from_bits_retain`,
/// so a newer sender's unknown bits survive both serde and this router). An empty set is `""`.
fn put_bitmask(table: &[(u64, &'static str)], bits: u64, out: &mut Vec<u8>) {
    let mut first = true;
    let mut known: u64 = 0;
    for (mask, name) in table {
        if *mask != 0 {
            known |= *mask;
            if bits & *mask == *mask {
                if !first {
                    out.extend_from_slice(b" | ");
                }
                first = false;
                out.extend_from_slice(name.as_bytes());
            }
        }
    }
    let residual = bits & !known;
    if residual != 0 {
        if !first {
            out.extend_from_slice(b" | ");
        }
        out.extend_from_slice(b"0x");
        put_hex_lower(out, residual);
    }
}

/// Writes `value` as lowercase hexadecimal with no leading zeros (matching `bitflags`/`{:x}`).
fn put_hex_lower(out: &mut Vec<u8>, value: u64) {
    let mut buf = [0u8; 16];
    let mut i = buf.len();
    let mut v = value;
    loop {
        i -= 1;
        buf[i] = hex_digit((v & 0xf) as u8);
        v >>= 4;
        if v == 0 {
            break;
        }
    }
    out.extend_from_slice(&buf[i..]);
}

#[inline(always)]
fn put_header(packet: PacketRef<'_>, out: &mut Vec<u8>) {
    out.extend_from_slice(br#"{"header":{"system_id":"#);
    put_int(out, *packet.system_id());
    out.extend_from_slice(br#","component_id":"#);
    put_int(out, *packet.component_id());
    out.extend_from_slice(br#","sequence":"#);
    put_int(out, *packet.sequence());
    out.extend_from_slice(br#","message_id":"#);
    put_int(out, packet.message_id());
    out.push(b'}');
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
fn put_f64(out: &mut Vec<u8>, value: f64) {
    if value.is_finite() {
        let mut buffer = zmij::Buffer::new();
        out.extend_from_slice(buffer.format_finite(value).as_bytes());
    } else {
        out.extend_from_slice(b"null");
    }
}

/// Renders a `char[len]` at `off` as a JSON string: bytes up to the first NUL, JSON-escaped,
/// matching `serde`'s `serialize_str` (via mavlink-core's `nulstr`). Non-ASCII bytes pass through
/// unescaped, exactly as `serde_json` does for valid UTF-8. Reads past a v2-truncated payload
/// yield 0 (an immediate NUL), i.e. an empty string.
fn put_char_array(p: &[u8], off: usize, len: usize, out: &mut Vec<u8>) {
    let field = &p[off.min(p.len())..(off + len).min(p.len())];
    let end = field.iter().position(|&b| b == 0).unwrap_or(field.len());
    // rust-mavlink's `nulstr` errors (dropping the frame) on non-UTF-8 content; a router instead
    // decodes lossily (U+FFFD for invalid sequences) so the frame still yields valid JSON. Valid
    // UTF-8 (incl. multibyte) borrows and is emitted byte-for-byte, matching serde_json.
    let text = String::from_utf8_lossy(&field[..end]);
    out.push(b'"');
    for &b in text.as_bytes() {
        match b {
            b'"' => out.extend_from_slice(b"\\\""),
            b'\\' => out.extend_from_slice(b"\\\\"),
            0x08 => out.extend_from_slice(b"\\b"),
            0x0c => out.extend_from_slice(b"\\f"),
            b'\n' => out.extend_from_slice(b"\\n"),
            b'\r' => out.extend_from_slice(b"\\r"),
            b'\t' => out.extend_from_slice(b"\\t"),
            0x00..=0x1f => {
                out.extend_from_slice(b"\\u00");
                let hi = b >> 4;
                let lo = b & 0xf;
                out.push(hex_digit(hi));
                out.push(hex_digit(lo));
            }
            _ => out.push(b),
        }
    }
    out.push(b'"');
}

#[inline(always)]
fn hex_digit(nibble: u8) -> u8 {
    if nibble < 10 {
        b'0' + nibble
    } else {
        b'a' + (nibble - 10)
    }
}

/// Transcodes MAVLinkJSON text straight to a wire [`Packet`] of the requested `version`, resolving
/// the message descriptor from the `"type"` tag via `resolve`. Returns `None` if the type is
/// unknown or the JSON is malformed.
///
/// Produces the exact same frame as `serde_json::from_str::<MAVLinkJSON<_>>` followed by
/// `to_packet(version)`, tolerating reordered members and extra whitespace.
pub fn from_json(
    json: &[u8],
    resolve: fn(&[u8]) -> Option<&'static MsgDesc>,
    version: MavlinkVersion,
) -> Option<Packet> {
    let mut p = Parser { b: json, i: 0 };
    let mut sys = 0u8;
    let mut comp = 0u8;
    let mut seq = 0u8;
    let mut payload = [0u8; 255];
    let mut desc: Option<&'static MsgDesc> = None;

    p.skip_ws();
    p.expect(b'{')?;
    loop {
        p.skip_ws();
        match p.peek()? {
            b'}' => break,
            b',' => p.i += 1,
            b'"' => {
                let key = p.string_raw()?;
                p.skip_ws();
                p.expect(b':')?;
                p.skip_ws();
                match key {
                    b"header" => parse_header(&mut p, &mut sys, &mut comp, &mut seq)?,
                    b"message" => desc = Some(parse_message(&mut p, resolve, &mut payload)?),
                    _ => p.skip_value()?,
                }
            }
            _ => return None,
        }
    }

    let desc = desc?;
    Some(build_frame(
        desc,
        sys,
        comp,
        seq,
        &payload[..desc.payload_len as usize],
        version,
    ))
}

fn parse_header(p: &mut Parser, sys: &mut u8, comp: &mut u8, seq: &mut u8) -> Option<()> {
    p.expect(b'{')?;
    loop {
        p.skip_ws();
        match p.peek()? {
            b'}' => {
                p.i += 1;
                return Some(());
            }
            b',' => p.i += 1,
            b'"' => {
                let key = p.string_raw()?;
                p.skip_ws();
                p.expect(b':')?;
                p.skip_ws();
                match key {
                    b"system_id" => *sys = p.number_i64()? as u8,
                    b"component_id" => *comp = p.number_i64()? as u8,
                    b"sequence" => *seq = p.number_i64()? as u8,
                    // `message_id` (and anything else) is redundant with the "type" tag.
                    _ => p.skip_value()?,
                }
            }
            _ => return None,
        }
    }
}

fn parse_message(
    p: &mut Parser,
    resolve: fn(&[u8]) -> Option<&'static MsgDesc>,
    payload: &mut [u8],
) -> Option<&'static MsgDesc> {
    p.expect(b'{')?;
    let mut desc: Option<&'static MsgDesc> = None;
    loop {
        p.skip_ws();
        match p.peek()? {
            b'}' => {
                p.i += 1;
                return desc;
            }
            b',' => p.i += 1,
            b'"' => {
                let key = p.string_raw()?;
                p.skip_ws();
                p.expect(b':')?;
                p.skip_ws();
                if key == b"type" {
                    // serde emits the tag first, so the descriptor is known before any field.
                    desc = resolve(p.string_raw()?);
                } else {
                    let d = desc?;
                    match d.fields.iter().find(|f| f.name.as_bytes() == key) {
                        Some(field) => write_field(p, field, payload)?,
                        None => p.skip_value()?,
                    }
                }
            }
            _ => return None,
        }
    }
}

fn write_field(p: &mut Parser, field: &FieldDesc, payload: &mut [u8]) -> Option<()> {
    let off = field.offset as usize;
    match field.kind {
        FieldKind::Scalar(sk) => write_scalar(p, off, sk, payload),
        FieldKind::CharArray(len) => {
            p.string_into(&mut payload[off..off + len as usize]);
            Some(())
        }
        FieldKind::Array(sk, len) => {
            p.expect(b'[')?;
            let size = scalar_size(sk);
            let mut k = 0usize;
            loop {
                p.skip_ws();
                match p.peek()? {
                    b']' => {
                        p.i += 1;
                        return Some(());
                    }
                    b',' => p.i += 1,
                    _ => {
                        if k < len as usize {
                            write_scalar(p, off + k * size, sk, payload)?;
                        } else {
                            p.skip_value()?;
                        }
                        k += 1;
                    }
                }
            }
        }
        FieldKind::Enum(table, sk) => {
            p.skip_ws();
            let value = match p.peek()? {
                b'{' => parse_enum_object(p, table)?,
                // A bare number is an enum value with no known name (emitted by the forward path
                // for values a router doesn't recognise, or sent by a newer producer).
                _ => parse_u64(p.number_token()?),
            };
            write_uint(payload, off, sk, value);
            Some(())
        }
        FieldKind::Bitmask(table, sk) => {
            let bits = parse_bitmask(table, p.string_raw()?);
            write_uint(payload, off, sk, bits);
            Some(())
        }
    }
}

/// Parses `{"type":"NAME"}` and resolves `NAME` to its value via `table`.
fn parse_enum_object(p: &mut Parser, table: &[(u64, &'static str)]) -> Option<u64> {
    p.expect(b'{')?;
    let mut value = 0u64;
    loop {
        p.skip_ws();
        match p.peek()? {
            b'}' => {
                p.i += 1;
                return Some(value);
            }
            b',' => p.i += 1,
            b'"' => {
                let key = p.string_raw()?;
                p.skip_ws();
                p.expect(b':')?;
                p.skip_ws();
                if key == b"type" {
                    value = enum_value(table, p.string_raw()?);
                } else {
                    p.skip_value()?;
                }
            }
            _ => return None,
        }
    }
}

fn write_scalar(p: &mut Parser, off: usize, sk: ScalarKind, payload: &mut [u8]) -> Option<()> {
    let tok = p.number_token()?;
    match sk {
        ScalarKind::U8 => payload[off] = parse_i64(tok) as u8,
        ScalarKind::I8 => payload[off] = parse_i64(tok) as i8 as u8,
        ScalarKind::U16 => {
            payload[off..off + 2].copy_from_slice(&(parse_i64(tok) as u16).to_le_bytes())
        }
        ScalarKind::I16 => {
            payload[off..off + 2].copy_from_slice(&(parse_i64(tok) as i16).to_le_bytes())
        }
        ScalarKind::U32 => {
            payload[off..off + 4].copy_from_slice(&(parse_i64(tok) as u32).to_le_bytes())
        }
        ScalarKind::I32 => {
            payload[off..off + 4].copy_from_slice(&(parse_i64(tok) as i32).to_le_bytes())
        }
        ScalarKind::U64 => payload[off..off + 8].copy_from_slice(&parse_u64(tok).to_le_bytes()),
        ScalarKind::I64 => payload[off..off + 8].copy_from_slice(&parse_i64(tok).to_le_bytes()),
        ScalarKind::F32 => payload[off..off + 4].copy_from_slice(&parse_f32(tok).to_le_bytes()),
        ScalarKind::F64 => payload[off..off + 8].copy_from_slice(&parse_f64(tok).to_le_bytes()),
    }
    Some(())
}

#[inline(always)]
fn write_uint(payload: &mut [u8], off: usize, sk: ScalarKind, value: u64) {
    match sk {
        ScalarKind::U8 | ScalarKind::I8 => payload[off] = value as u8,
        ScalarKind::U16 | ScalarKind::I16 => {
            payload[off..off + 2].copy_from_slice(&(value as u16).to_le_bytes())
        }
        ScalarKind::U32 | ScalarKind::I32 => {
            payload[off..off + 4].copy_from_slice(&(value as u32).to_le_bytes())
        }
        ScalarKind::U64 | ScalarKind::I64 => {
            payload[off..off + 8].copy_from_slice(&value.to_le_bytes())
        }
        ScalarKind::F32 | ScalarKind::F64 => {}
    }
}

#[inline(always)]
fn scalar_size(sk: ScalarKind) -> usize {
    match sk {
        ScalarKind::U8 | ScalarKind::I8 => 1,
        ScalarKind::U16 | ScalarKind::I16 => 2,
        ScalarKind::U32 | ScalarKind::I32 | ScalarKind::F32 => 4,
        ScalarKind::U64 | ScalarKind::I64 | ScalarKind::F64 => 8,
    }
}

#[inline(always)]
fn enum_value(table: &[(u64, &'static str)], name: &[u8]) -> u64 {
    for (value, n) in table {
        if n.as_bytes() == name {
            return *value;
        }
    }
    0
}

/// Reverse of [`put_bitmask`]: `" | "`-joined flag names back into a bitmask. A `0x..` residual
/// (unknown bits preserved by the forward path or a newer producer) is parsed back as hex, so
/// unknown bits survive a full JSON -> wire round-trip.
fn parse_bitmask(table: &[(u64, &'static str)], s: &[u8]) -> u64 {
    let mut bits = 0u64;
    for part in s.split(|&b| b == b'|') {
        let part = part.trim_ascii();
        if part.is_empty() {
            continue;
        }
        if let [b'0', b'x' | b'X', hex @ ..] = part {
            bits |= parse_hex_u64(hex);
        } else {
            bits |= enum_value(table, part);
        }
    }
    bits
}

#[inline(always)]
fn parse_hex_u64(s: &[u8]) -> u64 {
    let mut v = 0u64;
    for &b in s {
        match hex_val(b) {
            Some(d) => v = v * 16 + d as u64,
            None => break,
        }
    }
    v
}

/// Builds a wire frame of the requested `version`, byte-identical to rust-mavlink's
/// `serialize_message` for that version.
///
/// * v2 carries the full payload (base + extension fields) with trailing zeros trimmed to at least
///   one byte, and a 3-byte message id.
/// * v1 carries only the base fields (`base_len` bytes, never truncated), a 1-byte message id, and
///   cannot represent extension fields or ids above 255 (the id is truncated, matching mavlink).
fn build_frame(
    desc: &MsgDesc,
    sys: u8,
    comp: u8,
    seq: u8,
    payload: &[u8],
    version: MavlinkVersion,
) -> Packet {
    match version {
        MavlinkVersion::V2 => {
            // MAVLink v2 keeps at least one payload byte (see mavlink-core `remove_trailing_zeroes`).
            let mut len = payload.len();
            while len > 1 && payload[len - 1] == 0 {
                len -= 1;
            }

            let msgid = desc.id.to_le_bytes();
            let mut frame =
                Vec::with_capacity(1 + V2Packet::HEADER_SIZE + len + V2Packet::CHECKSUM_SIZE);
            frame.push(V2_STX);
            frame.push(len as u8);
            frame.push(0); // incompat flags
            frame.push(0); // compat flags
            frame.push(seq);
            frame.push(sys);
            frame.push(comp);
            frame.extend_from_slice(&msgid[0..3]);
            frame.extend_from_slice(&payload[..len]);

            let crc = mavlink::calculate_crc(&frame[1..], desc.crc_extra);
            frame.extend_from_slice(&crc.to_le_bytes());

            Packet::V2(V2Packet::new(Bytes::from(frame)))
        }
        MavlinkVersion::V1 => {
            let len = (desc.base_len as usize).min(payload.len());
            let mut frame =
                Vec::with_capacity(1 + V1Packet::HEADER_SIZE + len + V1Packet::CHECKSUM_SIZE);
            frame.push(V1_STX);
            frame.push(len as u8);
            frame.push(seq);
            frame.push(sys);
            frame.push(comp);
            frame.push(desc.id as u8);
            frame.extend_from_slice(&payload[..len]);

            let crc = mavlink::calculate_crc(&frame[1..], desc.crc_extra);
            frame.extend_from_slice(&crc.to_le_bytes());

            Packet::V1(V1Packet::new(Bytes::from(frame)))
        }
    }
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

#[inline(always)]
fn parse_u64(s: &[u8]) -> u64 {
    let mut value = 0u64;
    for &b in s {
        if b.is_ascii_digit() {
            value = value * 10 + (b - b'0') as u64;
        } else {
            break;
        }
    }
    value
}

/// Parses a JSON float token into an `f32`. Rust's parser is correctly-rounding, so the shortest
/// round-trippable text emitted by `zmij`/serde_json reproduces the exact bits. `null` (and any
/// junk) parses to 0.0; the serde baseline itself cannot deserialize `null` floats.
#[inline(always)]
fn parse_f32(s: &[u8]) -> f32 {
    core::str::from_utf8(s)
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(0.0)
}

#[inline(always)]
fn parse_f64(s: &[u8]) -> f64 {
    core::str::from_utf8(s)
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(0.0)
}

/// A minimal, whitespace-tolerant cursor over MAVLinkJSON text.
struct Parser<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> Parser<'a> {
    #[inline(always)]
    fn peek(&self) -> Option<u8> {
        self.b.get(self.i).copied()
    }

    #[inline(always)]
    fn expect(&mut self, c: u8) -> Option<()> {
        if self.peek()? == c {
            self.i += 1;
            Some(())
        } else {
            None
        }
    }

    #[inline(always)]
    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(b) if b.is_ascii_whitespace()) {
            self.i += 1;
        }
    }

    /// Reads a quoted string, returning the raw (still-escaped) inner slice. Fine for keys, enum
    /// names, the message tag and bitmask strings, none of which contain escapes.
    fn string_raw(&mut self) -> Option<&'a [u8]> {
        if self.peek()? != b'"' {
            return None;
        }
        self.i += 1;
        let start = self.i;
        while let Some(c) = self.peek() {
            match c {
                b'\\' => self.i += 2,
                b'"' => {
                    let s = &self.b[start..self.i];
                    self.i += 1;
                    return Some(s);
                }
                _ => self.i += 1,
            }
        }
        None
    }

    /// Reads a bare token (number, or `null`/`true`/`false`) up to the next structural byte.
    fn number_token(&mut self) -> Option<&'a [u8]> {
        let start = self.i;
        while let Some(c) = self.peek() {
            if matches!(c, b',' | b'}' | b']') || c.is_ascii_whitespace() {
                break;
            }
            self.i += 1;
        }
        (self.i != start).then(|| &self.b[start..self.i])
    }

    #[inline(always)]
    fn number_i64(&mut self) -> Option<i64> {
        Some(parse_i64(self.number_token()?))
    }

    /// Reads a JSON string, unescaping its contents into `dest` (truncating to `dest.len()`, like
    /// mavlink-core's char-array deserialize). Bytes not written stay as the caller left them
    /// (the payload buffer is zero-initialized).
    fn string_into(&mut self, dest: &mut [u8]) -> Option<()> {
        if self.peek()? != b'"' {
            return None;
        }
        self.i += 1;
        let mut w = 0usize;
        while let Some(c) = self.peek() {
            match c {
                b'"' => {
                    self.i += 1;
                    return Some(());
                }
                b'\\' => {
                    self.i += 1;
                    let e = self.peek()?;
                    self.i += 1;
                    match e {
                        b'u' => {
                            let cp = self.hex4()?;
                            w = utf8_encode(cp, dest, w);
                        }
                        _ => {
                            let byte = match e {
                                b'"' => b'"',
                                b'\\' => b'\\',
                                b'/' => b'/',
                                b'b' => 0x08,
                                b'f' => 0x0c,
                                b'n' => b'\n',
                                b'r' => b'\r',
                                b't' => b'\t',
                                _ => return None,
                            };
                            if w < dest.len() {
                                dest[w] = byte;
                                w += 1;
                            }
                        }
                    }
                }
                _ => {
                    if w < dest.len() {
                        dest[w] = c;
                        w += 1;
                    }
                    self.i += 1;
                }
            }
        }
        None
    }

    fn hex4(&mut self) -> Option<u32> {
        let mut v = 0u32;
        for _ in 0..4 {
            let d = hex_val(self.peek()?)?;
            self.i += 1;
            v = v * 16 + d as u32;
        }
        Some(v)
    }

    /// Skips one JSON value (string, number/literal, object or array).
    fn skip_value(&mut self) -> Option<()> {
        self.skip_ws();
        match self.peek()? {
            b'"' => self.string_raw().map(|_| ()),
            b'{' | b'[' => self.skip_container(),
            _ => self.number_token().map(|_| ()),
        }
    }

    fn skip_container(&mut self) -> Option<()> {
        let open = self.peek()?;
        let close = if open == b'{' { b'}' } else { b']' };
        self.i += 1;
        let mut depth = 1usize;
        while depth > 0 {
            match self.peek()? {
                b'"' => {
                    self.string_raw()?;
                }
                c if c == open => {
                    self.i += 1;
                    depth += 1;
                }
                c if c == close => {
                    self.i += 1;
                    depth -= 1;
                }
                _ => self.i += 1,
            }
        }
        Some(())
    }
}

#[inline(always)]
fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// UTF-8 encodes a code point into `dest` at `w` (truncating if it would overflow), returning the
/// new write index. Surrogate halves are passed through as the replacement-free raw code point;
/// MAVLink char arrays are plain UTF-8 text so this path is only hit for `\u00XX` control escapes.
fn utf8_encode(cp: u32, dest: &mut [u8], mut w: usize) -> usize {
    let mut push = |b: u8, w: &mut usize| {
        if *w < dest.len() {
            dest[*w] = b;
            *w += 1;
        }
    };
    if cp < 0x80 {
        push(cp as u8, &mut w);
    } else if cp < 0x800 {
        push(0xC0 | (cp >> 6) as u8, &mut w);
        push(0x80 | (cp & 0x3F) as u8, &mut w);
    } else {
        push(0xE0 | (cp >> 12) as u8, &mut w);
        push(0x80 | ((cp >> 6) & 0x3F) as u8, &mut w);
        push(0x80 | (cp & 0x3F) as u8, &mut w);
    }
    w
}
