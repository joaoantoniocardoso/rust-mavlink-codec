//! Runtime support for the generated MAVLinkJSON transcoders.
//!
//! `build.rs` emits, for every message in the dialect, a pure-data [`MsgDesc`] descriptor
//! (field names, wire offsets, and per-field [`FieldKind`]s plus enum/bitmask name tables). The
//! generic interpreter here walks a descriptor once and writes the MAVLinkJSON text, byte
//! identical to the `rust-mavlink` + `serde_json` baseline. Keeping the logic here (and only the
//! data generated) keeps the generated code tiny and lets the whole dialect share one hot loop.

use crate::Packet;

/// A whole message: its id, wire name (the serde `"type"` tag) and ordered fields.
///
/// `fields` are in MAVLink serialization order (base fields sorted by size, then extensions),
/// which is exactly the order `serde` emits them, so a single pass produces matching JSON.
pub struct MsgDesc {
    pub id: u32,
    pub name: &'static str,
    pub payload_len: u16,
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
pub fn to_json(packet: &Packet, desc: &MsgDesc, out: &mut Vec<u8>) {
    write(packet, desc, out, &mut NoRec);
}

/// Like [`to_json`] but also records the byte range of each field's rendered value into `ranges`
/// (parallel to `desc.fields`), enabling zero-copy per-field `Bytes::slice` fan-out.
pub fn to_json_indexed(
    packet: &Packet,
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

fn write<R: Recorder>(packet: &Packet, desc: &MsgDesc, out: &mut Vec<u8>, rec: &mut R) {
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
            out.extend_from_slice(br#"{"type":""#);
            out.extend_from_slice(enum_name(table, value));
            out.extend_from_slice(br#""}"#);
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
fn enum_name(table: &[(u64, &'static str)], value: u64) -> &'static [u8] {
    for (v, name) in table {
        if *v == value {
            return name.as_bytes();
        }
    }
    b""
}

/// Emits contained flags in declaration order joined by `" | "` (empty when no bits set),
/// matching the `bitflags` crate's `serde` string representation used by rust-mavlink.
fn put_bitmask(table: &[(u64, &'static str)], bits: u64, out: &mut Vec<u8>) {
    let mut first = true;
    for (mask, name) in table {
        if *mask != 0 && bits & *mask == *mask {
            if !first {
                out.extend_from_slice(b" | ");
            }
            first = false;
            out.extend_from_slice(name.as_bytes());
        }
    }
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
    out.push(b'"');
    for &b in &field[..end] {
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
