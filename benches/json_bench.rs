//! Baseline benchmark for the `Packet <-> JSON text` transcoding path.
//!
//! Measures the current rust-mavlink + serde_json implementation (`Packet::to_mavlink_json`
//! plus `serde_json`) so future low-copy transcoders can be compared against it.

use std::hint::black_box;

use criterion::{
    criterion_group, criterion_main, AxisScale, BenchmarkId, Criterion, PlotConfiguration,
    Throughput,
};
use dev_utils::create_random_v2_raw_message;
use mavlink::{dialects::ardupilotmega::MavMessage, MavlinkVersion};
use mavlink_codec::{mavlink_json::MAVLinkJSON, v2::V2Packet, Packet};
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

        group.bench_with_input(
            BenchmarkId::new("rust-mavlink+serde_json", messages_count),
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

criterion_group!(benches, benchmark_packet_to_json, benchmark_json_to_packet);
criterion_main!(benches);
