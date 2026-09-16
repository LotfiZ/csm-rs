//! Prepared API steady-state benchmark.
use csm_rs::{Matcher, Params, PreparedPolarScan};
use std::hint::black_box;
use std::time::Instant;

fn main() {
    let n = 720;
    let angles: Vec<f64> = (0..n)
        .map(|i| -std::f64::consts::PI + 2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64)
        .collect();
    let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.5 * (3.0 * a).sin()).collect();
    let valid = vec![true; n];
    let mut reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let mut sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let estimated_workspace_bytes = (reference.capacity() + sensor.capacity())
        * (std::mem::size_of::<f64>() + std::mem::size_of::<bool>());
    for (label, matcher) in [
        ("full", Matcher::new(Params::default()).unwrap()),
        ("pose_only", Matcher::default_pose_only()),
    ] {
        for _ in 0..3 {
            black_box(matcher.match_prepared(&mut reference, &mut sensor).unwrap());
        }
        let start = Instant::now();
        let mut successful = 0;
        let mut samples = Vec::with_capacity(30);
        for _ in 0..30 {
            let sample_start = Instant::now();
            let outcome = matcher
                .match_prepared(black_box(&mut reference), black_box(&mut sensor))
                .unwrap();
            samples.push(sample_start.elapsed().as_secs_f64() * 1e3);
            successful += usize::from(outcome.valid);
            black_box(outcome);
        }
        samples.sort_by(f64::total_cmp);
        let percentile = |p: f64| -> f64 {
            let index = ((samples.len() - 1) as f64 * p).round() as usize;
            samples[index]
        };
        println!(
            "mode={label},rays=720,repetitions=30,workspace_bytes={estimated_workspace_bytes},total_ms={:.3},mean_ms={:.3},min_ms={:.3},p95_ms={:.3},p99_ms={:.3},max_ms={:.3},successful_matches={successful}",
            start.elapsed().as_secs_f64() * 1e3,
            samples.iter().sum::<f64>() / samples.len() as f64,
            samples.iter().copied().fold(f64::INFINITY, f64::min),
            percentile(0.95),
            percentile(0.99),
            samples.iter().copied().fold(0.0, f64::max),
        );
    }
}
