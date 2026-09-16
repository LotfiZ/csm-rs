//! Repeatable steady-state timing baseline for the current C-shaped API.
//!
//! Run with:
//!   cargo run --release -p csm-rs --example benchmark_baseline
//!
//! The reported time excludes scan construction and includes the matcher
//! execution. This is intentionally dependency-free; use the same command and
//! compiler profile when comparing later implementations.

use std::hint::black_box;
use std::time::{Duration, Instant};

use csm_rs::{sm_icp, LaserData, Params, SmResult};

// The faithful baseline rejects scans above 10,000 rays. Larger industrial
// sizes are added by the redesigned implementation once that inherited limit
// is removed.
const CASES: &[usize] = &[360, 720, 1_080, 2_160, 4_096, 8_192];
const REPETITIONS: usize = 30;

fn scan(nrays: usize, phase: f64) -> LaserData {
    let angles: Vec<_> = (0..nrays)
        .map(|i| -std::f64::consts::PI + 2.0 * std::f64::consts::PI * i as f64 / (nrays - 1) as f64)
        .collect();
    let readings = angles
        .iter()
        .map(|angle| 8.0 + 0.5 * (3.0 * angle + phase).sin())
        .collect();
    LaserData::from_polar(angles, readings, vec![true; nrays])
        .expect("synthetic baseline scan is valid")
}

fn main() {
    println!("rays,repetitions,total_ms,mean_us,successful_matches");
    for &nrays in CASES {
        let mut reference = scan(nrays, 0.0);
        let mut sensor = scan(nrays, 0.02);
        let params = Params::default();
        let start = Instant::now();
        let mut successful = 0;
        for _ in 0..REPETITIONS {
            let mut result = SmResult::default();
            sm_icp(
                black_box(&params),
                black_box(&mut reference),
                black_box(&mut sensor),
                black_box(&mut result),
            )
            .expect("baseline scans remain valid");
            successful += usize::from(result.valid);
            black_box(result);
        }
        let elapsed = start.elapsed();
        let mean = elapsed.as_secs_f64() * 1e6 / REPETITIONS as f64;
        println!(
            "{nrays},{REPETITIONS},{:.3},{mean:.3},{successful}",
            elapsed.as_secs_f64() * 1e3
        );
        // Keep the output stable even on platforms with coarse timers.
        assert!(elapsed >= Duration::ZERO);
    }
}
