//! Verify the prepared pose workspace does not allocate during repeated
//! matching, including retained options, restarts, unsuccessful outcomes, and
//! capacity growth performed during preparation. This binary has a single test
//! so no other test can allocate concurrently and perturb the counter.

use csm_rs::{
    CorrespondenceSearch, CovarianceStatus, MatchOutcome, Matcher, Params, PreparedPolarScan,
};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

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

fn allocations() -> usize {
    ALLOCATIONS.load(Ordering::Relaxed)
}

fn scan(n: usize, phase: f64) -> PreparedPolarScan {
    let angles: Vec<f64> = (0..n)
        .map(|i| -1.0 + 2.0 * i as f64 / (n - 1) as f64)
        .collect();
    let readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + 0.5 * (3.0 * a + phase).sin())
        .collect();
    PreparedPolarScan::from_polar(angles, readings, vec![true; n]).unwrap()
}

fn assert_zero_allocations(label: &str, params: Params, sensor_readings: Vec<f64>) {
    let n = sensor_readings.len();
    let mut workspace = Matcher::new(params)
        .unwrap()
        .prepare(scan(n, 0.0), scan(n, 0.0))
        .unwrap();

    // Warm up so any lazily initialized state is in place.
    for _ in 0..3 {
        workspace.match_once().unwrap();
    }

    let valid = vec![true; n];
    let before = allocations();
    for _ in 0..20 {
        workspace.update_sensor(&sensor_readings, &valid).unwrap();
        workspace.match_once().unwrap();
    }
    let after = allocations();
    assert_eq!(after - before, 0, "{label} allocated during matching");
}

#[test]
fn repeated_prepared_pose_matching_does_not_allocate() {
    let base: Vec<f64> = (0..720)
        .map(|i| {
            let a = -1.0 + 2.0 * i as f64 / 719.0;
            8.0 + 0.5 * (3.0 * a).sin()
        })
        .collect();
    let far = vec![80.0; 720];

    assert_zero_allocations("default", Params::default(), base.clone());

    let mut naive = Params::default();
    naive.correspondence.search = CorrespondenceSearch::Naive;
    assert_zero_allocations("naive search", naive, base.clone());

    let mut weighted = Params::default();
    weighted.weights.ml = true;
    weighted.weights.sigma = true;
    assert_zero_allocations("weighting", weighted, base.clone());

    let mut alpha = Params::default();
    alpha.correspondence.do_alpha_test = true;
    alpha.correspondence.do_visibility_test = true;
    assert_zero_allocations("alpha + visibility", alpha, base.clone());

    let mut restart = Params::default();
    restart.restart.enabled = true;
    restart.restart.threshold_mean_error = 0.0;
    assert_zero_allocations("restart", restart, base.clone());

    assert_zero_allocations("unsuccessful", Params::default(), far);

    // Covariance/Fisher uncertainty with caller-reused output buffers.
    let valid = vec![true; 720];
    let params = Params {
        do_compute_covariance: true,
        ..Params::default()
    };
    let mut workspace = Matcher::new(params)
        .unwrap()
        .prepare(scan(720, 0.0), scan(720, 0.0))
        .unwrap();
    let mut outcome = MatchOutcome::default();
    outcome.reserve_uncertainty(720, 720);
    for _ in 0..3 {
        workspace.match_into(&mut outcome).unwrap();
    }
    assert_eq!(outcome.covariance_status, CovarianceStatus::Computed);
    assert!(outcome.has_uncertainty());
    let before = allocations();
    for _ in 0..20 {
        workspace.update_sensor(&base, &valid).unwrap();
        workspace.match_into(&mut outcome).unwrap();
    }
    assert_eq!(
        allocations() - before,
        0,
        "covariance matching allocated during matching"
    );
}
