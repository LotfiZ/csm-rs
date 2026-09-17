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

#[test]
fn cartesian_recovers_the_same_known_rotation_as_polar() {
    let n = 41;
    let phi = 0.05;
    let angles: Vec<f64> = (0..n).map(|i| -1.0 + i as f64 * 0.05).collect();
    let reading = |a: f64| 8.0 + 0.5 * (3.0 * a).sin();
    let reference_points: Vec<[f64; 2]> = angles
        .iter()
        .map(|&a| [reading(a) * a.cos(), reading(a) * a.sin()])
        .collect();
    let sensor_points: Vec<[f64; 2]> = angles
        .iter()
        .map(|&a| {
            let r = reading(a + phi);
            [r * a.cos(), r * a.sin()]
        })
        .collect();
    let valid = vec![true; n];

    let by_polar = Matcher::new(Params::default())
        .unwrap()
        .match_polar(
            PolarScan::new(
                &angles,
                &angles.iter().map(|&a| reading(a)).collect::<Vec<_>>(),
                &valid,
            )
            .unwrap(),
            PolarScan::new(
                &angles,
                &angles.iter().map(|&a| reading(a + phi)).collect::<Vec<_>>(),
                &valid,
            )
            .unwrap(),
        )
        .unwrap();
    let by_cartesian = Matcher::new(Params::default())
        .unwrap()
        .match_cartesian(
            CartesianScan::new(&reference_points, &valid).unwrap(),
            CartesianScan::new(&sensor_points, &valid).unwrap(),
        )
        .unwrap();

    assert!(by_polar.valid && by_cartesian.valid);
    assert!((by_cartesian.pose.theta - phi).abs() < 1e-3);
    for (p, c) in by_polar.pose.to_array().iter().zip(by_cartesian.pose.to_array()) {
        assert!((p - c).abs() < 1e-6, "polar vs cartesian: {p} != {c}");
    }
}

#[test]
fn cartesian_honors_explicit_guess_and_reference_selection() {
    let points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    let valid = vec![true; 21];
    let reference = CartesianScan::new(&points, &valid).unwrap();
    let sensor = CartesianScan::new(&points, &valid).unwrap();
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
    let identity = matcher.match_cartesian(reference, sensor).unwrap();
    let guessed = matcher
        .match_cartesian_from(reference, sensor, Pose::new(0.2, -0.1, 0.05))
        .unwrap();
    assert!(identity.valid && guessed.valid);
    assert!(identity
        .pose
        .to_array()
        .iter()
        .zip(guessed.pose.to_array())
        .any(|(a, b)| (a - b).abs() > 1e-9));
}

#[test]
fn cartesian_missing_returns_keep_positions() {
    let mut points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    let mut valid = vec![true; 21];
    // A missing return may carry an unusable point; it stays in its slot.
    points[5] = [f64::NAN, f64::NAN];
    valid[5] = false;
    let scan = CartesianScan::new(&points, &valid).expect("missing returns are allowed");
    assert_eq!(scan.len(), 21);
    assert!(!scan.valid()[5]);
    assert!(scan.points()[5][0].is_nan());

    // The scan still matches; the missing return is simply not used.
    let reference = CartesianScan::new(&points, &valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .unwrap()
        .match_cartesian(reference, scan)
        .unwrap();
    assert!(outcome.valid);
}

#[test]
fn cartesian_rejects_malformed_dimensions_and_valid_nonfinite_points() {
    let points = vec![[1.0, 0.0]; 21];
    let err = CartesianScan::new(&points, &[true; 20]).unwrap_err();
    assert!(matches!(
        err,
        csm_rs::ScanError::InconsistentLengths { field: "valid", .. }
    ));

    let mut bad = points.clone();
    bad[3] = [f64::INFINITY, 0.0];
    let err = CartesianScan::new(&bad, &[true; 21]).unwrap_err();
    assert_eq!(err, csm_rs::ScanError::BadValidRay(3));
}

#[test]
fn cartesian_rejects_nonfinite_bearing_on_a_valid_point() {
    let points = vec![[8.0, 0.0]; 21];
    let valid = vec![true; 21];
    let mut angles: Vec<f64> = (0..21).map(|i| i as f64 * 0.03).collect();
    angles[4] = f64::NAN;
    let err = CartesianScan::with_angles(&points, &angles, &valid).unwrap_err();
    assert_eq!(err, csm_rs::ScanError::NonFiniteBearing(4));

    // A non-finite bearing on a missing return is tolerated.
    let mut valid = valid;
    valid[4] = false;
    assert!(CartesianScan::with_angles(&points, &angles, &valid).is_ok());
}

#[test]
fn cartesian_derived_bearings_follow_input_order() {
    // Points are ordered by increasing bearing; the derived bearings must
    // follow that order rather than being sorted into something else.
    let points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    let scan = CartesianScan::new(&points, &[true; 21]).unwrap();
    assert!(scan.angles().is_none());
    let angles = scan.bearings();
    assert!(angles.windows(2).all(|w| w[1] > w[0]));
}

#[test]
fn converged_termination_is_reported_as_accepted() {
    let (angles, readings, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    assert_eq!(outcome.termination, TerminationReason::Converged);
    assert!(outcome.accepted());
    assert_eq!(outcome.candidate(), Some(outcome.pose));
}

#[test]
fn iteration_exhaustion_reports_a_candidate_not_an_acceptance() {
    let (angles, reference_readings, valid) = polar(41, 0.0);
    let sensor_readings: Vec<f64> = angles
        .iter()
        .map(|&a| 8.0 + 0.5 * (3.0 * (a + 0.10)).sin())
        .collect();
    let reference = PolarScan::new(&angles, &reference_readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &sensor_readings, &valid).unwrap();
    let params = Params {
        stopping: csm_rs::StoppingCriteria {
            max_iterations: 2,
            ..Params::default().stopping
        },
        restart: csm_rs::RestartParams {
            enabled: false,
            ..Params::default().restart
        },
        ..Params::default()
    };
    let outcome = Matcher::new(params)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    assert_eq!(outcome.termination, TerminationReason::IterationLimit);
    assert!(!outcome.accepted());
    let candidate = outcome.candidate().expect("a candidate pose is available");
    assert!(candidate.is_finite());
    assert!(outcome.iterations > 0);
    assert!(outcome.nvalid > 0);
}

#[test]
fn no_correspondence_reports_candidate_diagnostics() {
    let (angles, _, valid) = polar(21, 0.0);
    let reference = PolarScan::new(&angles, &[5.0; 21], &valid).unwrap();
    let sensor = PolarScan::new(&angles, &[6.0; 21], &valid).unwrap();
    let mut params = Params::default();
    params.correspondence.max_dist = 0.1;
    let outcome = Matcher::new(params)
        .unwrap()
        .match_polar(reference, sensor)
        .unwrap();
    assert!(!outcome.valid);
    assert_eq!(outcome.termination, TerminationReason::NoCorrespondences);
    assert!(!outcome.accepted());
    assert!(outcome.candidate().is_some());
}

#[test]
fn capacity_growth_is_explicit_and_reusable() {
    // Start from prepared-but-empty storage, then fill and grow explicitly.
    let small_angles: Vec<f64> = (0..32).map(|i| -1.0 + 2.0 * i as f64 / 31.0).collect();
    let small_readings = vec![8.0; 32];
    let small_valid = vec![true; 32];
    let mut workspace = Matcher::default_pose_only()
        .prepare(
            PreparedPolarScan::with_capacity(32),
            PreparedPolarScan::with_capacity(32),
        )
        .unwrap();
    workspace
        .set_reference_polar(&small_angles, &small_readings, &small_valid)
        .unwrap();
    workspace
        .set_sensor_polar(&small_angles, &small_readings, &small_valid)
        .unwrap();
    assert!(workspace.match_once().unwrap().valid);

    workspace.reserve(1024, 1024);
    assert!(workspace.capacities().0 >= 1024);

    let big_angles: Vec<f64> = (0..900)
        .map(|i| -1.0 + 2.0 * i as f64 / 899.0)
        .collect();
    let big_readings = vec![8.0; 900];
    let big_valid = vec![true; 900];
    workspace
        .set_reference_polar(&big_angles, &big_readings, &big_valid)
        .unwrap();
    workspace
        .set_sensor_polar(&big_angles, &big_readings, &big_valid)
        .unwrap();
    assert!(workspace.match_once().unwrap().valid);

    // An out-of-capacity update is a clear error, not a silent growth.
    let err = workspace
        .set_sensor_polar(&vec![0.0; 2048], &vec![8.0; 2048], &vec![true; 2048])
        .unwrap_err();
    assert!(matches!(err, csm_rs::ScanError::CapacityExceeded { .. }));
}

#[test]
fn prepared_matching_supports_large_and_independently_sized_scans() {
    let big_angles: Vec<f64> = (0..3_500)
        .map(|i| -1.0 + 2.0 * i as f64 / 3_499.0)
        .collect();
    let small_angles: Vec<f64> = (0..200).map(|i| -1.0 + 2.0 * i as f64 / 199.0).collect();
    let big_readings = vec![8.0; big_angles.len()];
    let small_readings = vec![8.0; small_angles.len()];
    let reference = PreparedPolarScan::from_polar(
        big_angles,
        big_readings,
        vec![true; 3_500],
    )
    .unwrap();
    let sensor = PreparedPolarScan::from_polar(
        small_angles,
        small_readings,
        vec![true; 200],
    )
    .unwrap();
    let mut workspace = Matcher::default_pose_only()
        .prepare(reference, sensor)
        .unwrap();
    assert_eq!(workspace.capacities(), (3_500, 200));
    assert!(workspace.match_once().unwrap().valid);
}

#[test]
fn uncertainty_failure_is_independent_of_pose_status() {
    // The engine reports uncertainty failure separately from pose status; a
    // usable, accepted pose must remain usable when covariance cannot be
    // produced.
    let outcome = csm_rs::MatchOutcome {
        valid: true,
        termination: TerminationReason::Converged,
        covariance_status: CovarianceStatus::Failed,
        ..csm_rs::MatchOutcome::default()
    };
    assert!(outcome.accepted());
    assert!(outcome.candidate().is_some());
    assert!(!outcome.has_uncertainty());
    assert_eq!(outcome.covariance_status, CovarianceStatus::Failed);
}

#[test]
fn uncertainty_includes_fisher_information() {
    let (angles, readings, valid) = polar(41, 0.0);
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
    assert_eq!(outcome.covariance_status, CovarianceStatus::Computed);
    assert!(outcome.covariance.is_some());
    assert!(outcome.fisher_information.is_some());
    assert!(outcome.dx_dy_reference.is_some() && outcome.dx_dy_sensor.is_some());
    assert!(outcome.has_uncertainty());
}
