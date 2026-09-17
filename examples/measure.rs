//! Reproducible latency, allocation, memory, and alignment measurements.
//!
//! Run in release mode from the repository root:
//!
//!     cargo run --release -p csm-rs --example measure
//!
//! Preparation (scan + workspace construction) is measured separately from
//! matching. Matching latency is reported as a distribution with percentiles,
//! and allocations are counted for the matching loop only.

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use csm_rs::{MatchOutcome, Matcher, Params, PreparedPolarScan, PreparedMatcher};

struct CountingAllocator;

static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        System.realloc(ptr, layout, new_size)
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const ROTATION: f64 = 0.05;

fn reading(angle: f64) -> f64 {
    8.0 + 0.5 * (2.0 * angle).sin() + 0.3 * (3.0 * angle).cos()
}

fn scan(nrays: usize, phase: f64, valid_frac: f64, noise: f64, seed: u64) -> PreparedPolarScan {
    let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).max(1);
    let mut next = move || {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        (state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    };
    let angles: Vec<f64> = (0..nrays)
        .map(|i| -1.0 + 2.0 * i as f64 / (nrays - 1) as f64)
        .collect();
    let keep = (nrays as f64 * valid_frac).round() as usize;
    let mut readings = Vec::with_capacity(nrays);
    let mut valid = Vec::with_capacity(nrays);
    for (i, &a) in angles.iter().enumerate() {
        if i < keep {
            let value = reading(a + phase) + noise * (next() - 0.5);
            readings.push(value);
            valid.push(true);
        } else {
            readings.push(f64::NAN);
            valid.push(false);
        }
    }
    PreparedPolarScan::from_polar(angles, readings, valid).expect("synthetic scan is valid")
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let index = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[index]
}

struct Workload {
    label: &'static str,
    nrays: usize,
    valid_frac: f64,
    noise: f64,
    reps: usize,
    covariance: bool,
    restart: bool,
}

fn run(workload: &Workload) {
    let mut params = Params {
        do_compute_covariance: workload.covariance,
        ..Params::default()
    };
    params.restart.enabled = workload.restart;
    let matcher = Matcher::new(params).expect("valid params");

    let prep_start = Instant::now();
    let reference = scan(workload.nrays, 0.0, workload.valid_frac, workload.noise, 1);
    let sensor = scan(workload.nrays, ROTATION, workload.valid_frac, workload.noise, 2);
    let mut workspace: PreparedMatcher = matcher
        .prepare(reference, sensor)
        .expect("workspace prepares");
    let prep_ms = prep_start.elapsed().as_secs_f64() * 1e3;
    let prepared_scan_bytes = 2
        * workload.nrays
        * (std::mem::size_of::<f64>() * 2 + std::mem::size_of::<bool>());
    let workspace_bytes = workspace.workspace_bytes();

    let mut estimate = || {
        let start = Instant::now();
        let outcome = workspace.match_once().expect("match runs");
        black_box(outcome);
        start.elapsed().as_secs_f64() * 1e6
    };

    // Warm up so buffers and jump tables are ready.
    for _ in 0..3 {
        black_box(estimate());
    }

    let mut samples = Vec::with_capacity(workload.reps);
    let mut translation_error = 0.0;
    let mut rotation_error = 0.0;
    let mut valid = 0;
    // Reserve reusable uncertainty storage before the allocation snapshot so
    // only the matching loop is counted.
    let mut outcome = MatchOutcome::default();
    if workload.covariance {
        outcome.reserve_uncertainty(workload.nrays, workload.nrays);
    }
    let allocations_before = ALLOCATIONS.load(Ordering::Relaxed);
    if workload.covariance {
        for _ in 0..workload.reps {
            let start = Instant::now();
            workspace
                .match_into(&mut outcome)
                .expect("instrumented match runs");
            samples.push(start.elapsed().as_secs_f64() * 1e6);
            valid += usize::from(outcome.valid);
            translation_error += outcome.pose.x.hypot(outcome.pose.y);
            rotation_error += (outcome.pose.theta - ROTATION).abs();
            black_box(&outcome);
        }
    } else {
        for _ in 0..workload.reps {
            let start = Instant::now();
            let outcome = workspace.match_once().expect("match runs");
            samples.push(start.elapsed().as_secs_f64() * 1e6);
            valid += usize::from(outcome.valid);
            translation_error += outcome.pose.x.hypot(outcome.pose.y);
            rotation_error += (outcome.pose.theta - ROTATION).abs();
            black_box(outcome);
        }
    }
    let allocations = ALLOCATIONS.load(Ordering::Relaxed) - allocations_before;
    samples.sort_by(f64::total_cmp);

    let mean_us = samples.iter().sum::<f64>() / samples.len() as f64;
    let mean_ms = mean_us / 1e3;
    let combined = prep_ms + mean_ms;
    let error = translation_error / workload.reps as f64;
    let rotation = rotation_error / workload.reps as f64;
    println!(
        "{},{},{},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{},{},{:.3e},{:.3e},{},{},{:.3e}",
        workload.label,
        workload.nrays,
        workload.reps,
        prep_ms,
        combined,
        mean_ms,
        percentile(&samples, 0.5) / 1e3,
        percentile(&samples, 0.95) / 1e3,
        percentile(&samples, 0.99) / 1e3,
        samples.iter().copied().fold(f64::INFINITY, f64::min) / 1e3,
        samples.iter().copied().fold(0.0, f64::max) / 1e3,
        valid,
        allocations,
        error,
        rotation,
        prepared_scan_bytes,
        workspace_bytes,
        mean_ms / 1e3,
    );
}

fn main() {
    println!("workload,rays,reps,prep_ms,combined_ms,mean_ms,p50_ms,p95_ms,p99_ms,min_ms,max_ms,valid,allocs,translation_error_m,rotation_error_rad,prepared_bytes,workspace_bytes,mean_s");
    // Baseline latency uses the documented no-restart pose-only path; the
    // restart shell is measured separately because it multiplies the cost.
    let workloads = [
        Workload { label: "easy_pose_only", nrays: 720, valid_frac: 1.0, noise: 0.0, reps: 200, covariance: false, restart: false },
        Workload { label: "typical_2048_pose_only", nrays: 2048, valid_frac: 1.0, noise: 0.0, reps: 200, covariance: false, restart: false },
        Workload { label: "typical_3000_pose_only", nrays: 3000, valid_frac: 1.0, noise: 0.0, reps: 150, covariance: false, restart: false },
        Workload { label: "large_10000_pose_only", nrays: 10_000, valid_frac: 1.0, noise: 0.0, reps: 50, covariance: false, restart: false },
        Workload { label: "typical_3000_uncertainty", nrays: 3000, valid_frac: 1.0, noise: 0.0, reps: 100, covariance: true, restart: false },
        Workload { label: "difficult_partial_pose_only", nrays: 3000, valid_frac: 0.6, noise: 0.02, reps: 100, covariance: false, restart: false },
        Workload { label: "restart_3000_pose_only", nrays: 3000, valid_frac: 1.0, noise: 0.005, reps: 30, covariance: false, restart: true },
    ];
    for workload in &workloads {
        run(workload);
    }
}
