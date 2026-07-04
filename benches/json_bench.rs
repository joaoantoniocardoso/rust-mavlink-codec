//! Baseline benchmark for the `Packet <-> JSON text` transcoding path.
//!
//! Measures the current rust-mavlink + serde_json implementation (`Packet::to_mavlink_json`
//! plus `serde_json`) so future low-copy transcoders can be compared against it.

use std::hint::black_box;

use criterion::{
    criterion_group, criterion_main, AxisScale, BenchmarkId, Criterion, PlotConfiguration,
    Throughput,
};
use dev_utils::{create_random_v2_message_from_id, create_random_v2_raw_message};
use mavlink::{dialects::ardupilotmega::MavMessage, MavlinkVersion};
use mavlink_codec::{
    mavlink_json::{experimental, MAVLinkJSON},
    v2::V2Packet,
    Packet,
};
use rand::{prelude::StdRng, SeedableRng};

fn benchmark_packet_to_json(c: &mut Criterion) {
    let seed = 42;
    println!("Using seed {seed:?}");
    let mut rng: StdRng = SeedableRng::seed_from_u64(seed);

    let mut group = c.benchmark_group("packet_to_json");
    group.confidence_level(0.95).sample_size(100);
    group.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    let messages_counts: Vec<usize> = vec![1, 5, 10, 50, 100, 500, 1000, 5000, 10000];

    for messages_count in &messages_counts {
        group.throughput(Throughput::Elements(*messages_count as u64));

        let mut packets = Vec::with_capacity(*messages_count);
        for _ in 0..*messages_count {
            packets.push(Packet::V2(V2Packet::from(create_random_v2_raw_message(
                &mut rng,
            ))));
        }

        // Baseline: parse + `serde_json::to_string` (fresh `String` per message).
        group.bench_with_input(
            BenchmarkId::new("to_string", messages_count),
            messages_count,
            |b, &_messages_count| {
                b.iter(|| {
                    for packet in &packets {
                        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                        let json = serde_json::to_string(&mavlink_json).unwrap();
                        black_box(json);
                    }
                })
            },
        );

        // Option A: parse + `write_json` into a single reused buffer.
        group.bench_with_input(
            BenchmarkId::new("write_json-reused-buf", messages_count),
            messages_count,
            |b, &_messages_count| {
                let mut buf: Vec<u8> = Vec::with_capacity(4096);
                b.iter(|| {
                    for packet in &packets {
                        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                        buf.clear();
                        mavlink_json.write_json(&mut buf).unwrap();
                        black_box(&buf);
                    }
                })
            },
        );

        // Option A (dual-repr flavor): parse + `to_json_bytes` producing owned `Bytes`.
        group.bench_with_input(
            BenchmarkId::new("to_json_bytes", messages_count),
            messages_count,
            |b, &_messages_count| {
                b.iter(|| {
                    for packet in &packets {
                        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                        let json = mavlink_json.to_json_bytes().unwrap();
                        black_box(json);
                    }
                })
            },
        );

        // Reference: parse + serialize to a null sink, to isolate pure format-compute
        // (itoa/ryu/escaping/serde dispatch) from buffer management.
        group.bench_with_input(
            BenchmarkId::new("to_sink", messages_count),
            messages_count,
            |b, &_messages_count| {
                b.iter(|| {
                    for packet in &packets {
                        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                        serde_json::to_writer(std::io::sink(), &mavlink_json).unwrap();
                    }
                })
            },
        );

        // Reference: parse only, to expose how much of the cost is the typed parse vs. the
        // JSON serialization (informs whether Option C is worth pursuing).
        group.bench_with_input(
            BenchmarkId::new("parse-only", messages_count),
            messages_count,
            |b, &_messages_count| {
                b.iter(|| {
                    for packet in &packets {
                        let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                        black_box(mavlink_json);
                    }
                })
            },
        );
    }

    group.finish();
}

fn benchmark_json_to_packet(c: &mut Criterion) {
    let seed = 42;
    println!("Using seed {seed:?}");
    let mut rng: StdRng = SeedableRng::seed_from_u64(seed);

    let mut group = c.benchmark_group("json_to_packet");
    group.confidence_level(0.95).sample_size(100);
    group.plot_config(PlotConfiguration::default().summary_scale(AxisScale::Logarithmic));

    let messages_counts: Vec<usize> = vec![1, 5, 10, 50, 100, 500, 1000, 5000, 10000];

    for messages_count in &messages_counts {
        group.throughput(Throughput::Elements(*messages_count as u64));

        let mut jsons = Vec::with_capacity(*messages_count);
        for _ in 0..*messages_count {
            let packet = Packet::V2(V2Packet::from(create_random_v2_raw_message(&mut rng)));
            jsons.push(
                serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap(),
            );
        }

        group.bench_with_input(
            BenchmarkId::new("serde_json+rust-mavlink", messages_count),
            messages_count,
            |b, &_messages_count| {
                b.iter(|| {
                    for json in &jsons {
                        let mavlink_json: MAVLinkJSON<MavMessage> =
                            serde_json::from_str(json).unwrap();
                        let packet = mavlink_json.to_packet(MavlinkVersion::V2);
                        black_box(packet);
                    }
                })
            },
        );
    }

    group.finish();
}

/// Reverse Option C spike: hand-written JSON -> wire transcoders vs. the serde baseline, across
/// integer, float and enum/bitflag messages.
fn benchmark_spike_from_json(c: &mut Criterion) {
    let seed = 42;
    println!("Using seed {seed:?}");

    let cases: &[(&str, u32, fn(&[u8]) -> Packet)] = &[
        (
            "global_position_int",
            experimental::GLOBAL_POSITION_INT_ID,
            experimental::global_position_int_from_json,
        ),
        (
            "attitude",
            experimental::ATTITUDE_ID,
            experimental::attitude_from_json,
        ),
        (
            "heartbeat",
            experimental::HEARTBEAT_ID,
            experimental::heartbeat_from_json,
        ),
    ];

    for (name, msg_id, transcode) in cases {
        let mut rng: StdRng = SeedableRng::seed_from_u64(seed);

        let mut jsons = Vec::with_capacity(1000);
        while jsons.len() < 1000 {
            let raw = create_random_v2_message_from_id(&mut rng, *msg_id).unwrap();
            let packet = Packet::V2(V2Packet::from(raw));
            let json =
                serde_json::to_string(&packet.to_mavlink_json::<MavMessage>().unwrap()).unwrap();
            // The serde baseline cannot deserialize `null` floats; keep both sides comparable.
            if json.contains("null") {
                continue;
            }
            jsons.push(json);
        }

        let mut group = c.benchmark_group(format!("json_to_packet_spike/{name}"));
        group.confidence_level(0.95).sample_size(100);
        group.throughput(Throughput::Elements(1000));

        group.bench_function("from_str+to_packet", |b| {
            b.iter(|| {
                for json in &jsons {
                    let mavlink_json: MAVLinkJSON<MavMessage> = serde_json::from_str(json).unwrap();
                    let packet = mavlink_json.to_packet(MavlinkVersion::V2);
                    black_box(packet);
                }
            })
        });

        group.bench_function("spike-transcode", |b| {
            b.iter(|| {
                for json in &jsons {
                    let packet = transcode(json.as_bytes());
                    black_box(packet);
                }
            })
        });

        // Generated: descriptor-driven JSON -> wire (the shippable reverse path).
        group.bench_function("generated-transcode", |b| {
            b.iter(|| {
                for json in &jsons {
                    let packet = Packet::from_json_transcoded(json.as_bytes());
                    black_box(packet);
                }
            })
        });

        group.finish();
    }
}

/// Option C spike: hand-written per-message transcoders vs. the Option A serde path, one
/// benchmark group per representative message type (covering ints, floats, arrays, enums,
/// bitflags), on a homogeneous stream of 1000 frames.
fn benchmark_spike(c: &mut Criterion) {
    let seed = 42;
    println!("Using seed {seed:?}");
    let mut rng: StdRng = SeedableRng::seed_from_u64(seed);

    type Transcoder = fn(&Packet, &mut Vec<u8>);
    let cases: [(&str, u32, Transcoder); 4] = [
        (
            "global_position_int",
            experimental::GLOBAL_POSITION_INT_ID,
            experimental::global_position_int_to_json,
        ),
        (
            "attitude",
            experimental::ATTITUDE_ID,
            experimental::attitude_to_json,
        ),
        (
            "gps_status",
            experimental::GPS_STATUS_ID,
            experimental::gps_status_to_json,
        ),
        (
            "heartbeat",
            experimental::HEARTBEAT_ID,
            experimental::heartbeat_to_json,
        ),
    ];

    let messages_count = 1000usize;

    for (label, msgid, transcode) in cases {
        let mut group = c.benchmark_group(format!("packet_to_json_spike/{label}"));
        group.confidence_level(0.95).sample_size(100);
        group.throughput(Throughput::Elements(messages_count as u64));

        let mut packets = Vec::with_capacity(messages_count);
        for _ in 0..messages_count {
            let raw = create_random_v2_message_from_id(&mut rng, msgid).unwrap();
            packets.push(Packet::V2(V2Packet::from(raw)));
        }

        // Option A: parse + serde into a reused buffer.
        group.bench_function("write_json-reused-buf", |b| {
            let mut buf: Vec<u8> = Vec::with_capacity(4096);
            b.iter(|| {
                for packet in &packets {
                    let mavlink_json = packet.to_mavlink_json::<MavMessage>().unwrap();
                    buf.clear();
                    mavlink_json.write_json(&mut buf).unwrap();
                    black_box(&buf);
                }
            })
        });

        // Option C spike: hand-written wire bytes -> JSON directly, into a reused buffer.
        group.bench_function("spike-transcode", |b| {
            let mut buf: Vec<u8> = Vec::with_capacity(4096);
            b.iter(|| {
                for packet in &packets {
                    buf.clear();
                    transcode(packet, &mut buf);
                    black_box(&buf);
                }
            })
        });

        // Generated: descriptor table + generic interpreter (the shippable path), reused buffer.
        group.bench_function("generated-transcode", |b| {
            let mut buf: Vec<u8> = Vec::with_capacity(4096);
            b.iter(|| {
                for packet in &packets {
                    buf.clear();
                    packet.write_json_transcoded(&mut buf);
                    black_box(&buf);
                }
            })
        });

        group.finish();
    }
}

criterion_group!(
    benches,
    benchmark_packet_to_json,
    benchmark_json_to_packet,
    benchmark_spike,
    benchmark_spike_from_json
);
criterion_main!(benches);
