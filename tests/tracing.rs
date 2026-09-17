//! Optional iteration instrumentation: traced and uninstrumented matching must
//! agree, and traces must carry real correspondence and residual data.

use csm_rs::{Matcher, Params, PreparedPolarScan};

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

#[test]
fn traced_and_uninstrumented_matches_agree() {
    let matcher = Matcher::new(Params::default()).unwrap();
    let mut plain = matcher.prepare(scan(120, 0.0), scan(120, 0.0)).unwrap();
    let mut traced = matcher.prepare(scan(120, 0.0), scan(120, 0.0)).unwrap();

    let plain_outcome = plain.match_once().unwrap();
    let mut snapshots = Vec::new();
    let traced_outcome = traced
        .match_once_traced(|snapshot| snapshots.push(snapshot))
        .unwrap();

    assert_eq!(plain_outcome.valid, traced_outcome.valid);
    assert_eq!(plain_outcome.termination, traced_outcome.termination);
    for (a, b) in plain_outcome
        .pose
        .to_array()
        .iter()
        .zip(traced_outcome.pose.to_array())
    {
        assert!((a - b).abs() < 1e-12, "traced pose diverged: {a} != {b}");
    }

    assert!(
        !snapshots.is_empty(),
        "traces should contain real iterations"
    );
    let total: usize = snapshots
        .iter()
        .map(|snapshot| snapshot.correspondences.len())
        .sum();
    assert!(total > 0, "traces should contain correspondences");
    for snapshot in &snapshots {
        assert!(snapshot.pose.iter().all(|v| v.is_finite()));
        assert!(snapshot.error.is_finite());
        assert_eq!(
            snapshot.valid_correspondences,
            snapshot.correspondences.len(),
            "iteration correspondence counts should match the trace"
        );
        for correspondence in &snapshot.correspondences {
            assert!(correspondence.distance.is_finite());
        }
    }
}

#[test]
fn restart_iterations_are_marked() {
    let mut params = Params::default();
    params.restart.enabled = true;
    params.restart.threshold_mean_error = 0.0;
    let matcher = Matcher::new(params).unwrap();
    let mut workspace = matcher.prepare(scan(120, 0.0), scan(120, 0.02)).unwrap();
    let mut saw_restart = false;
    workspace
        .match_once_traced(|snapshot| {
            saw_restart |= snapshot.restart;
        })
        .unwrap();
    assert!(saw_restart, "restart iterations should be labelled");
}
