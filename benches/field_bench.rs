//! Per-field JSON egress benchmark, modeling the Zenoh JSON driver workload.
//!
//! For each message the Zenoh JSON driver publishes the whole-message JSON (aggregate +
//! per-message topics) *and* one topic per field carrying that field's value. This benchmark
//! compares three strategies for producing "whole-message JSON + all per-field values", all in
//! the serde_json-compatible format, on a stream of `GLOBAL_POSITION_INT` messages:
//!
//! * `to_value`    -- today's mavlink-server path: `serde_json::to_value` then per-field
//!   serialization (mirrors the `dev/parse_messages` "old" arm).
//! * `phf-getters` -- the `dev/parse_messages` technique: a static `phf` name->getter map, each
//!   getter reading wire bytes and appending the field's JSON value (refined to write bytes
//!   directly instead of `serde_json::Value`).
//! * `range-index` -- a single transcode pass that emits the whole-message blob and records each
//!   field's value byte-range, so per-field publishing is a zero-copy `Bytes::slice`.

use std::hint::black_box;

use bytes::Bytes;
use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use dev_utils::create_random_v2_message_from_id;
use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink_codec::{mavlink_json::experimental, v2::V2Packet, Packet};
use rand::{prelude::StdRng, SeedableRng};

const N: usize = 1000;

fn benchmark_per_field_egress(c: &mut Criterion) {
    let seed = 42;
    println!("Using seed {seed:?}");
    let mut rng: StdRng = SeedableRng::seed_from_u64(seed);

    let mut packets = Vec::with_capacity(N);
    for _ in 0..N {
        let raw = create_random_v2_message_from_id(&mut rng, experimental::GLOBAL_POSITION_INT_ID)
            .unwrap();
        packets.push(Packet::V2(V2Packet::from(raw)));
    }

    let mut group = c.benchmark_group("per_field_egress/global_position_int");
    group.confidence_level(0.95).sample_size(100);
    group.throughput(Throughput::Elements(N as u64));

    // Each arm yields *publishable* owned `Bytes` for the whole message and for every field, so
    // the per-field allocation cost that fan-out sinks pay is charged fairly across arms. Only
    // `range-index` can serve fields as zero-copy slices of the one whole-message blob.

    // Strategy 1: serde_json::to_value + per-field serialization (today's mavlink-server path).
    group.bench_function("to_value", |b| {
        b.iter(|| {
            for packet in &packets {
                let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                let full = Bytes::from(serde_json::to_vec(&mavlink_json).unwrap());
                black_box(&full);

                let value = serde_json::to_value(&mavlink_json.message).unwrap();
                for (name, field) in value.as_object().unwrap() {
                    if name == "type" {
                        continue;
                    }
                    let field_bytes = Bytes::from(serde_json::to_vec(field).unwrap());
                    black_box((name, field_bytes));
                }
            }
        })
    });

    // Strategy 2: phf name->getter map, each getter appends canonical JSON bytes.
    group.bench_function("phf-getters", |b| {
        let mut full = Vec::with_capacity(256);
        let mut field = Vec::with_capacity(16);
        b.iter(|| {
            for packet in &packets {
                full.clear();
                experimental::global_position_int_to_json(packet, &mut full);
                black_box(Bytes::copy_from_slice(&full));

                // v2 truncates trailing zero bytes; zero-pad so fixed-offset reads are valid.
                let mut payload = [0u8; 28];
                let src = packet.payload();
                let n = src.len().min(28);
                payload[..n].copy_from_slice(&src[..n]);

                for (name, getter) in GPI_FIELD_GETTERS.entries() {
                    field.clear();
                    getter(&payload, &mut field);
                    black_box((name, Bytes::copy_from_slice(&field)));
                }
            }
        })
    });

    // Strategy 3: single transcode pass -> whole-message blob + per-field zero-copy slices.
    group.bench_function("range-index", |b| {
        let mut out = Vec::with_capacity(256);
        b.iter(|| {
            for packet in &packets {
                out.clear();
                let mut ranges = [(0u32, 0u32); 9];
                experimental::global_position_int_to_json_indexed(packet, &mut out, &mut ranges);
                let blob = Bytes::copy_from_slice(&out);
                black_box(&blob);

                for (i, name) in experimental::GLOBAL_POSITION_INT_FIELD_NAMES
                    .iter()
                    .enumerate()
                {
                    let (start, end) = ranges[i];
                    let field = blob.slice(start as usize..end as usize);
                    black_box((name, field));
                }
            }
        })
    });

    group.finish();
}

type FieldGetter = fn(&[u8], &mut Vec<u8>);

static GPI_FIELD_GETTERS: phf::Map<&'static str, FieldGetter> = phf::phf_map! {
    "time_boot_ms" => (|p, o| put_int(o, rd_u32(p, 0))) as FieldGetter,
    "lat" => (|p, o| put_int(o, rd_i32(p, 4))) as FieldGetter,
    "lon" => (|p, o| put_int(o, rd_i32(p, 8))) as FieldGetter,
    "alt" => (|p, o| put_int(o, rd_i32(p, 12))) as FieldGetter,
    "relative_alt" => (|p, o| put_int(o, rd_i32(p, 16))) as FieldGetter,
    "vx" => (|p, o| put_int(o, rd_i16(p, 20))) as FieldGetter,
    "vy" => (|p, o| put_int(o, rd_i16(p, 22))) as FieldGetter,
    "vz" => (|p, o| put_int(o, rd_i16(p, 24))) as FieldGetter,
    "hdg" => (|p, o| put_int(o, rd_u16(p, 26))) as FieldGetter,
};

#[inline(always)]
fn put_int<I: itoa::Integer>(out: &mut Vec<u8>, value: I) {
    let mut buffer = itoa::Buffer::new();
    out.extend_from_slice(buffer.format(value).as_bytes());
}

#[inline(always)]
fn rd_u32(p: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]])
}

#[inline(always)]
fn rd_i32(p: &[u8], o: usize) -> i32 {
    i32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]])
}

#[inline(always)]
fn rd_i16(p: &[u8], o: usize) -> i16 {
    i16::from_le_bytes([p[o], p[o + 1]])
}

#[inline(always)]
fn rd_u16(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([p[o], p[o + 1]])
}

criterion_group!(benches, benchmark_per_field_egress);
criterion_main!(benches);
