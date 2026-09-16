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
    let matcher = Matcher::new(Params::default());
    let start = Instant::now();
    let mut successful = 0;
    for _ in 0..30 {
        let outcome = matcher
            .match_prepared(black_box(&mut reference), black_box(&mut sensor))
            .unwrap();
        successful += usize::from(outcome.valid);
        black_box(outcome);
    }
    println!(
        "rays=720,repetitions=30,total_ms={:.3},successful_matches={successful}",
        start.elapsed().as_secs_f64() * 1e3
    );
}
