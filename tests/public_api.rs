//! Public API behavior: the coherent scan/configuration/matching/result
//! surface.

use csm_rs::{
    CartesianScan, CovarianceStatus, MatchStatus, Matcher, Params, PolarScan, Pose, PreparedMatcher,
    PreparedPolarScan, TerminationReason,
};

fn polar(n: usize, phase: f64) -> (Vec<f64>, Vec<f64>, Vec<bool>) {
    let angles: Vec<f64> = (0..n).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings: Vec<f64> = angles
        .iter()
        .map(|a| 8.0 + 0.5 * (3.0 * a + phase).sin())
        .collect();
    let valid = vec![true; angles.len()];
    (angles, readings, valid)
}

#[test]
fn borrowed_polar_match_preserves_inputs() {
    let (angles, readings, valid) = polar(21, 0.0);
    let angles_before = angles.clone();
    let readings_before = readings.clone();
    let valid_before = valid.clone();

    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();

    assert!(outcome.valid);
    assert_eq!(outcome.status, MatchStatus::Converged);
    assert_eq!(outcome.termination, TerminationReason::Converged);
    assert!(outcome.pose.to_array().iter().all(|v| v.abs() < 1e-6));
    assert_eq!(angles, angles_before);
    assert_eq!(readings, readings_before);
    assert_eq!(valid, valid_before);
}

#[test]
fn identity_guess_is_the_default_convenience() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let matcher = Matcher::new(Params::default()).unwrap();
    let explicit = matcher
        .match_polar_from(reference, sensor, Pose::IDENTITY)
        .unwrap();
    let convenience = matcher.match_polar(reference, sensor).unwrap();
    assert_eq!(explicit.pose, convenience.pose);
}

#[test]
fn known_ground_truth_rotation_is_recovered() {
    // A sensor rotated by a known angle around its own origin sees the same
    // scene at shifted bearings, so the sensor-to-reference transform is a
    // pure rotation by that angle.
    let n = 41;
    let phi = 0.05;
    let angles: Vec<f64> = (0..n).map(|i| -1.0 + i as f64 * 0.05).collect();
    let reading = |a: f64| 8.0 + 0.5 * (3.0 * a).sin();
    let reference_readings: Vec<f64> = angles.iter().map(|&a| reading(a)).collect();
    let sensor_readings: Vec<f64> = angles.iter().map(|&a| reading(a + phi)).collect();
    let valid = vec![true; n];

    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &sensor_readings, &valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();

    assert!(outcome.valid);
    assert!(
        (outcome.pose.theta - phi).abs() < 1e-3,
        "expected rotation {phi}, got {:?}",
        outcome.pose
    );
    assert!(outcome.pose.x.abs() < 1e-3 && outcome.pose.y.abs() < 1e-3);
}

#[test]
fn explicit_initial_guess_is_propagated() {
    // With a single iteration and identical scans, the result still depends on
    // the supplied guess, proving the guess reaches the engine.
    let (angles, readings, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let params = Params {
        stopping: csm_rs::StoppingCriteria {
            max_iterations: 1,
            ..Params::default().stopping
        },
        restart: csm_rs::RestartParams {
            enabled: false,
            ..Params::default().restart
        },
        ..Params::default()
    };
    let matcher = Matcher::new(params).unwrap();
    let from_zero = matcher
        .match_polar_from(reference, sensor, Pose::IDENTITY)
        .unwrap();
    let from_offset = matcher
        .match_polar_from(reference, sensor, Pose::new(0.25, -0.15, 0.10))
        .unwrap();
    assert!(from_zero.valid && from_offset.valid);
    assert!(
        from_zero
            .pose
            .to_array()
            .iter()
            .zip(from_offset.pose.to_array())
            .any(|(a, b)| (a - b).abs() > 1e-9),
        "different guesses must produce different first-iteration results"
    );
}

#[test]
fn caller_selects_which_scan_is_the_reference() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let matcher = Matcher::new(Params::default()).unwrap();
    // Both orders are valid calls; identical scans give the identity in each.
    let forward = matcher.match_polar(reference, sensor).unwrap();
    let reverse = matcher.match_polar(sensor, reference).unwrap();
    assert!(forward.valid && reverse.valid);
    assert!(forward.pose.x.abs() < 1e-6 && reverse.pose.x.abs() < 1e-6);
}

#[test]
fn cartesian_points_are_preserved_and_match_polar_equivalently() {
    let (angles, readings, valid) = polar(21, 0.0);
    let points: Vec<[f64; 2]> = angles
        .iter()
        .zip(&readings)
        .map(|(a, r)| [r * a.cos(), r * a.sin()])
        .collect();
    let before = points.clone();

    let polar_outcome = Matcher::new(Params::default())
        .unwrap()
        .match_polar(
            PolarScan::new(&angles, &readings, &valid).unwrap(),
            PolarScan::new(&angles, &readings, &valid).unwrap(),
        )
        .unwrap();
    let cartesian_outcome = Matcher::new(Params::default())
        .unwrap()
        .match_cartesian(
            CartesianScan::new(&points, &valid).unwrap(),
            CartesianScan::new(&points, &valid).unwrap(),
        )
        .unwrap();

    assert_eq!(points, before);
    assert!(polar_outcome.valid && cartesian_outcome.valid);
    for (p, c) in polar_outcome
        .pose
        .to_array()
        .iter()
        .zip(cartesian_outcome.pose.to_array())
    {
        assert!((p - c).abs() < 1e-9);
    }
}

#[test]
fn pose_only_is_the_default_and_omits_covariance() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let outcome = Matcher::default_pose_only()
        .match_polar(reference, sensor)
        .unwrap();
    assert!(outcome.valid && outcome.covariance.is_none());
    assert_eq!(outcome.covariance_status, CovarianceStatus::Disabled);
}

#[test]
fn uncertainty_is_requested_explicitly() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let params = Params {
        do_compute_covariance: true,
        ..Params::default()
    };
    let outcome = Matcher::new(params)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    assert!(outcome.valid);
    assert_eq!(outcome.covariance_status, CovarianceStatus::Computed);
    assert!(outcome.has_uncertainty());
}

#[test]
fn prepared_scans_can_be_reused_without_reallocation_of_shape() {
    let (angles, readings, valid) = polar(21, 0.0);
    let mut reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let mut sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let matcher = Matcher::new(Params::default()).unwrap();
    assert!(matcher
        .match_prepared(&mut reference, &mut sensor)
        .unwrap()
        .valid);
    assert!(matcher
        .match_prepared(&mut reference, &mut sensor)
        .unwrap()
        .valid);
}

#[test]
fn prepared_scan_updates_preserve_capacity() {
    let (angles, _, valid) = polar(21, 0.0);
    let readings = vec![8.0; 21];
    let mut scan = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let capacity = scan.capacity();
    let updated = vec![7.5; scan.len()];
    scan.update(&updated, &vec![true; scan.len()]).unwrap();
    assert_eq!(scan.readings(), updated.as_slice());
    assert_eq!(scan.capacity(), capacity);
}

#[test]
fn prepared_scan_reports_capacity_overflow() {
    let (angles, _, _) = polar(21, 0.0);
    let mut scan = PreparedPolarScan::from_polar(angles, vec![8.0; 21], vec![true; 21]).unwrap();
    let error = scan.update(&[8.0; 22], &[true; 22]).unwrap_err();
    assert_eq!(
        error,
        csm_rs::ScanError::CapacityExceeded {
            capacity: 21,
            requested: 22,
        }
    );
}

#[test]
fn prepared_matcher_reuses_owned_scans() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut workspace =
        PreparedMatcher::new(Matcher::default_pose_only(), reference, sensor).unwrap();
    let outcome = workspace.match_once().unwrap();
    assert!(outcome.valid);
    assert_eq!(outcome.termination, TerminationReason::Converged);
    assert_eq!(workspace.capacities(), (21, 21));
    assert!(workspace.workspace_bytes() > 0);
    workspace.update_sensor(&[8.0; 21], &[true; 21]).unwrap();
    assert!(workspace.match_once().unwrap().valid);
}

#[test]
fn prepared_tracing_reports_real_iterations() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut workspace =
        PreparedMatcher::new(Matcher::new(Params::default()).unwrap(), reference, sensor).unwrap();
    let mut snapshots = 0;
    workspace
        .match_once_traced(|snapshot| {
            snapshots += 1;
            assert!(snapshot.pose.iter().all(|v| v.is_finite()));
        })
        .unwrap();
    assert!(snapshots > 0);
}

#[test]
fn cartesian_explicit_angles_are_validated_and_preserved() {
    let points = vec![[8.0, 0.0]; 21];
    let valid = vec![true; 21];
    let angles: Vec<f64> = (0..21).map(|i| i as f64 * 0.03).collect();
    let scan = CartesianScan::with_angles(&points, &angles, &valid).unwrap();
    assert_eq!(scan.angles(), Some(angles.as_slice()));

    let err = CartesianScan::with_angles(&points, &angles[..20], &valid).unwrap_err();
    assert!(matches!(
        err,
        csm_rs::ScanError::InconsistentLengths {
            field: "angles",
            ..
        }
    ));

    let mut duplicate = angles;
    duplicate[1] = duplicate[0];
    let err = CartesianScan::with_angles(&points, &duplicate, &valid).unwrap_err();
    assert!(matches!(err, csm_rs::ScanError::DuplicateBearing(1)));
}

#[test]
fn cartesian_validation_reports_offending_ray() {
    let mut points = vec![[1.0, 0.0]; 21];
    points[7][0] = f64::NAN;
    let err = CartesianScan::new(&points, &[true; 21]).unwrap_err();
    assert_eq!(err, csm_rs::ScanError::BadValidRay(7));
}
