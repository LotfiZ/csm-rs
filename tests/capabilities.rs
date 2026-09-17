//! Public-API coverage of the retained CSM capabilities. Exact C agreement is
//! checked by the crate-internal golden fixture test; these tests confirm the
//! capabilities are reachable and have the documented observable effect
//! through the supported interface.
#![allow(clippy::field_reassign_with_default)]

use csm_rs::{
    CorrespondenceSearch, CovarianceStatus, DistanceMetric, Matcher, OutlierParams, Params,
    PolarScan, Pose, TerminationReason,
};

fn wall_scan(n: usize, phase: f64, offset: f64) -> (Vec<f64>, Vec<f64>, Vec<bool>) {
    let angles: Vec<f64> = (0..n).map(|i| -1.0 + 2.0 * i as f64 / (n - 1) as f64).collect();
    let readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + offset + 0.4 * (2.0 * a + phase).sin())
        .collect();
    (angles, readings, vec![true; n])
}

fn pose_of(params: Params, phase: f64, offset: f64) -> Pose {
    let (angles, reference_readings, valid) = wall_scan(120, 0.0, 0.0);
    let sensor_readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + offset + 0.4 * (2.0 * a + phase).sin())
        .collect();
    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &sensor_readings, &valid).unwrap();
    Matcher::new(params)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap()
        .pose
}

#[test]
fn correspondence_strategies_agree() {
    let mut tricks = Params::default();
    tricks.correspondence.search = CorrespondenceSearch::Tricks;
    let mut naive = Params::default();
    naive.correspondence.search = CorrespondenceSearch::Naive;
    let a = pose_of(tricks, 0.05, 0.10);
    let b = pose_of(naive, 0.05, 0.10);
    for (x, y) in a.to_array().iter().zip(b.to_array()) {
        assert!((x - y).abs() < 1e-9, "tricks {a:?} != naive {b:?}");
    }
}

#[test]
fn point_metrics_are_selectable_and_differ() {
    let mut line = Params::default();
    line.correspondence.metric = DistanceMetric::PointToLine;
    let mut point = Params::default();
    point.correspondence.metric = DistanceMetric::PointToPoint;
    let a = pose_of(line, 0.05, 0.10);
    let b = pose_of(point, 0.05, 0.10);
    assert!(
        a.to_array()
            .iter()
            .zip(b.to_array())
            .any(|(x, y)| (x - y).abs() > 1e-9),
        "point-to-line and point-to-point should not be identical"
    );
}

#[test]
fn outlier_rejection_changes_the_solution() {
    let base = Params::default();
    let mut rejecting = Params::default();
    rejecting.outliers = OutlierParams {
        max_perc: 0.5,
        remove_doubles: true,
        ..OutlierParams::default()
    };
    let a = pose_of(base, 0.05, 0.10);
    let b = pose_of(rejecting, 0.05, 0.10);
    assert!(
        a.to_array()
            .iter()
            .zip(b.to_array())
            .any(|(x, y)| (x - y).abs() > 1e-9),
        "outlier configuration should affect the pose"
    );
}

#[test]
fn orientation_and_visibility_filters_are_configurable() {
    let mut filtered = Params::default();
    filtered.correspondence.do_alpha_test = true;
    filtered.correspondence.do_visibility_test = true;
    let base = pose_of(Params::default(), 0.05, 0.10);
    let filtered = pose_of(filtered, 0.05, 0.10);
    assert!(
        base.to_array()
            .iter()
            .zip(filtered.to_array())
            .any(|(x, y)| (x - y).abs() > 1e-9),
        "alpha/visibility filtering should affect the pose"
    );
}

#[test]
fn sigma_weighting_is_reachable_and_changes_the_solution() {
    let (angles, reference_readings, valid) = wall_scan(120, 0.0, 0.0);
    let sensor_readings: Vec<f64> = angles
        .iter()
        .enumerate()
        .map(|(i, a)| 8.1 + 0.4 * (2.0 * a + 0.05).sin() + if i % 7 == 0 { 0.3 } else { 0.0 })
        .collect();
    let sigma: Vec<f64> = (0..angles.len()).map(|i| 0.01 + 0.05 * (i % 5) as f64).collect();

    let mut weighted = Params::default();
    weighted.weights.sigma = true;
    let mut plain = Params::default();
    plain.weights.sigma = false;

    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::with_inputs(&angles, &sensor_readings, &valid, Some(&sigma), None).unwrap();
    let a = Matcher::new(weighted).unwrap().match_polar(reference, sensor).unwrap();

    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::with_inputs(&angles, &sensor_readings, &valid, Some(&sigma), None).unwrap();
    let b = Matcher::new(plain).unwrap().match_polar(reference, sensor).unwrap();

    assert!(
        a.pose
            .to_array()
            .iter()
            .zip(b.pose.to_array())
            .any(|(x, y)| (x - y).abs() > 1e-9),
        "sigma weights should affect the solved pose"
    );
}

#[test]
fn ml_weighting_is_reachable_through_computed_alpha() {
    let mut weighted = Params::default();
    weighted.weights.ml = true;
    weighted.correspondence.do_alpha_test = true;
    let mut plain = Params::default();
    plain.correspondence.do_alpha_test = true;
    let a = pose_of(weighted, 0.05, 0.10);
    let b = pose_of(plain, 0.05, 0.10);
    assert!(
        a.to_array()
            .iter()
            .zip(b.to_array())
            .any(|(x, y)| (x - y).abs() > 1e-9),
        "ML weights over computed alpha should affect the pose"
    );
}

#[test]
fn restart_improves_a_difficult_initial_pose() {
    let (angles, reference_readings, valid) = wall_scan(120, 0.0, 0.0);
    let sensor_readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + 0.4 * (2.0 * (a - 0.06)).sin())
        .collect();
    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &sensor_readings, &valid).unwrap();

    let mut with = Params::default();
    with.restart.enabled = true;
    with.restart.threshold_mean_error = 0.0; // force the perturbation shell
    let mut without = Params::default();
    without.restart.enabled = false;

    let a = Matcher::new(with)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    let b = Matcher::new(without)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    assert!(a.valid, "restart result should be usable");
    assert!(
        a.iterations > b.iterations,
        "restart shell should add iterations: {} vs {}",
        a.iterations,
        b.iterations
    );
}

#[test]
fn covariance_derivatives_and_fisher_are_finite_and_consistent() {
    let (angles, reference_readings, valid) = wall_scan(120, 0.0, 0.0);
    let sensor_readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + 0.4 * (2.0 * a + 0.04).sin())
        .collect();
    let params = Params {
        do_compute_covariance: true,
        ..Params::default()
    };
    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &sensor_readings, &valid).unwrap();
    let outcome = Matcher::new(params)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();

    assert_eq!(outcome.covariance_status, CovarianceStatus::Computed);
    let covariance = outcome.covariance.expect("covariance");
    assert!(covariance.data.iter().flatten().all(|v| v.is_finite()));
    // Covariance is symmetric.
    for r in 0..3 {
        for c in 0..3 {
            assert!((covariance.data[r][c] - covariance.data[c][r]).abs() < 1e-12);
        }
    }
    let fisher = outcome.fisher_information.expect("fisher information");
    assert!(fisher.data.iter().flatten().all(|v| v.is_finite()));
    let dx1 = outcome.dx_dy_reference.expect("dx/dy reference");
    let dx2 = outcome.dx_dy_sensor.expect("dx/dy sensor");
    assert_eq!(dx1.rows(), 3);
    assert_eq!(dx2.rows(), 3);
}

#[test]
fn degenerate_geometry_is_an_outcome() {
    // Three collinear points over thirty rays: well formed, but too little
    // geometry for a confident match.
    let angles: Vec<f64> = (0..30).map(|i| -1.0 + 2.0 * i as f64 / 29.0).collect();
    let mut readings = vec![5.0; 30];
    let mut valid = vec![false; 30];
    for &i in &[8usize, 15, 22] {
        readings[i] = 4.4 + (i as f64) * 0.01;
        valid[i] = true;
    }
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    assert!(outcome.candidate().is_some());
    assert!(
        !outcome.valid || outcome.termination == TerminationReason::Converged,
        "degenerate geometry must be reported as an outcome, got {:?}",
        outcome.termination
    );
}

#[test]
fn partial_overlap_still_produces_a_usable_candidate() {
    let (angles, reference_readings, valid) = wall_scan(120, 0.0, 0.0);
    // Drop the last 40% of the sensor scan to simulate partial overlap.
    let mut sensor_valid = valid.clone();
    for flag in sensor_valid.iter_mut().skip(72) {
        *flag = false;
    }
    let sensor_readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + 0.4 * (2.0 * a + 0.04).sin())
        .collect();
    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &sensor_readings, &sensor_valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    let candidate = outcome.candidate().expect("candidate present");
    assert!(candidate.is_finite());
}
