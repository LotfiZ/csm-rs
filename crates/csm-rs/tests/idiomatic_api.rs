use csm_rs::{
    CartesianScan, CovarianceStatus, MatchOutcome, MatchStatus, Matcher, Params, PolarScan,
    PreparedMatcher, PreparedPolarScan, SmResult,
};

#[test]
fn borrowed_polar_match_preserves_inputs() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.2 * a.cos()).collect();
    let valid = vec![true; angles.len()];
    let angles_before = angles.clone();
    let readings_before = readings.clone();
    let valid_before = valid.clone();

    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let outcome: MatchOutcome = Matcher::new(Params::default())
        .match_polar(reference, sensor)
        .unwrap();

    assert!(outcome.valid);
    assert_eq!(outcome.status, MatchStatus::Converged);
    assert!(outcome.converged());
    assert!(outcome.pose.iter().all(|v| v.abs() < 1e-6));
    assert_eq!(angles, angles_before);
    assert_eq!(readings, readings_before);
    assert_eq!(valid, valid_before);
}

#[test]
fn prepared_scans_can_be_reused() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.2 * a.cos()).collect();
    let valid = vec![true; angles.len()];
    let mut reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let mut sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let matcher = Matcher::new(Params::default());
    assert!(
        matcher
            .match_prepared(&mut reference, &mut sensor)
            .unwrap()
            .valid
    );
    assert!(
        matcher
            .match_prepared(&mut reference, &mut sensor)
            .unwrap()
            .valid
    );
}

#[test]
fn prepared_match_can_reuse_result_storage() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.2 * a.cos()).collect();
    let valid = vec![true; angles.len()];
    let mut reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let mut sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut result = SmResult::default();
    Matcher::default()
        .match_prepared_into(&mut reference, &mut sensor, &mut result)
        .unwrap();
    assert!(result.valid);
}

#[test]
fn prepared_scan_updates_without_changing_shape() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let mut scan = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let capacity = scan.capacity();
    let updated = vec![7.5; scan.len()];
    scan.update(&updated, &vec![true; scan.len()]).unwrap();
    assert_eq!(scan.readings(), updated.as_slice());
    assert_eq!(scan.capacity(), capacity);
}

#[test]
fn prepared_cartesian_constructor_matches() {
    let points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    let valid = vec![true; points.len()];
    let mut reference = PreparedPolarScan::from_cartesian(&points, &valid).unwrap();
    let mut sensor = PreparedPolarScan::from_cartesian(&points, &valid).unwrap();
    assert!(
        Matcher::default()
            .match_prepared(&mut reference, &mut sensor)
            .unwrap()
            .valid
    );
}

#[test]
fn prepared_cartesian_scan_updates() {
    let points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    let valid = vec![true; points.len()];
    let mut scan = PreparedPolarScan::from_cartesian(&points, &valid).unwrap();
    let capacity = scan.capacity();
    let updated: Vec<[f64; 2]> = points.iter().map(|p| [p[0] * 0.9, p[1] * 0.9]).collect();
    scan.update_cartesian(&updated, &valid).unwrap();
    assert!((scan.readings()[10] - 7.2).abs() < 1e-9);
    assert_eq!(scan.capacity(), capacity);
}

#[test]
fn cartesian_validation_reports_offending_ray() {
    let mut points = vec![[1.0, 0.0]; 21];
    points[7][0] = f64::NAN;
    let err = CartesianScan::new(&points, &[true; 21]).unwrap_err();
    assert_eq!(err, csm_rs::LaserDataError::BadValidRay(7));
}

#[test]
fn cartesian_validation_rejects_undersized_scans() {
    let err = CartesianScan::new(&[[1.0, 0.0]; 9], &[true; 9]).unwrap_err();
    assert_eq!(err, csm_rs::LaserDataError::NraysOutOfRange);
}

#[test]
fn prepared_match_observer_receives_outcome() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let mut reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let mut sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut seen = false;
    let outcome = Matcher::default()
        .match_prepared_observed(&mut reference, &mut sensor, |o| {
            seen = o.converged();
        })
        .unwrap();
    assert!(seen && outcome.converged());
}

#[test]
fn pose_only_matcher_omits_optional_covariance() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference = PolarScan::new(&angles, &readings, &valid).unwrap();
    let sensor = PolarScan::new(&angles, &readings, &valid).unwrap();
    let outcome = Matcher::pose_only(Params::default())
        .match_polar(reference, sensor)
        .unwrap();
    assert!(outcome.valid && outcome.covariance.is_none());
    assert_eq!(outcome.covariance_status, CovarianceStatus::Disabled);
}

#[test]
fn borrowed_polar_rejects_mismatched_lengths() {
    let err = PolarScan::new(&[0.0; 10], &[1.0; 9], &[true; 10]).unwrap_err();
    assert!(matches!(
        err,
        csm_rs::LaserDataError::InconsistentLengths {
            field: "readings",
            ..
        }
    ));
}

#[test]
fn checked_matcher_constructor_rejects_invalid_params() {
    let mut params = Params::default();
    params.stopping.max_iterations = -1;
    assert!(Matcher::try_new(params).is_err());
}

#[test]
fn uncertainty_helper_requires_all_diagnostics() {
    let mut outcome = csm_rs::MatchOutcome::default();
    assert!(!outcome.has_uncertainty());
    outcome.covariance = Some(csm_rs::math::Mat3::new([[1.0, 0.0, 0.0]; 3]));
    assert!(!outcome.has_uncertainty());
}

#[test]
fn prepared_match_supports_orientation_and_visibility_filters() {
    let angles: Vec<f64> = (0..41).map(|i| -1.0 + i as f64 * 0.05).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut params = Params::default();
    params.correspondence.do_alpha_test = true;
    params.correspondence.do_visibility_test = true;
    let mut workspace = Matcher::new(params).prepare(reference, sensor).unwrap();
    let outcome = workspace.match_once().unwrap();
    assert!(outcome.iterations >= 0);
}

#[test]
fn prepared_match_supports_weighting_options() {
    let angles: Vec<f64> = (0..41).map(|i| -1.0 + i as f64 * 0.05).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut params = Params::default();
    params.weights.ml = true;
    params.weights.sigma = true;
    let mut workspace = Matcher::new(params).prepare(reference, sensor).unwrap();
    let outcome = workspace.match_once().unwrap();
    assert!(outcome.iterations >= 0);
}

#[test]
fn prepared_match_supports_search_and_outlier_options() {
    let angles: Vec<f64> = (0..41).map(|i| -1.0 + i as f64 * 0.05).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut params = Params::default();
    params.correspondence.search = csm_rs::params::CorrespondenceSearch::Naive;
    params.outliers.max_perc = 0.9;
    params.outliers.adaptive_order = 0.8;
    params.outliers.adaptive_mult = 2.0;
    params.outliers.remove_doubles = true;
    let mut workspace = Matcher::new(params).prepare(reference, sensor).unwrap();
    assert!(workspace.match_once().unwrap().iterations >= 0);
}

#[test]
fn prepared_match_supports_restart_and_bounded_termination() {
    let angles: Vec<f64> = (0..41).map(|i| -1.0 + i as f64 * 0.05).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut params = Params::default();
    params.first_guess = [0.05, -0.03, 0.02];
    params.stopping.max_iterations = 20;
    params.restart.enabled = true;
    let mut workspace = Matcher::new(params).prepare(reference, sensor).unwrap();
    let outcome = workspace.match_once().unwrap();
    assert!(outcome.iterations <= 20 * 7);
}

#[test]
fn prepared_match_terminates_with_cycle_detection_enabled() {
    let angles: Vec<f64> = (0..41).map(|i| -1.0 + i as f64 * 0.05).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut params = Params::default();
    params.stopping.max_iterations = 12;
    let mut workspace = Matcher::new(params).prepare(reference, sensor).unwrap();
    let outcome = workspace.match_once().unwrap();
    assert!(outcome.iterations <= 12);
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
        csm_rs::LaserDataError::InconsistentLengths {
            field: "angles",
            ..
        }
    ));
    let mut duplicate = angles;
    duplicate[1] = duplicate[0];
    let err = CartesianScan::with_angles(&points, &duplicate, &valid).unwrap_err();
    assert!(matches!(err, csm_rs::LaserDataError::DuplicateBearing(1)));
}

#[test]
fn cartesian_match_preserves_points() {
    let points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    let valid = vec![true; points.len()];
    let before = points.clone();
    let scan = CartesianScan::new(&points, &valid).unwrap();
    let outcome = Matcher::new(Params::default())
        .match_cartesian(scan, scan)
        .unwrap();
    assert!(outcome.valid);
    assert_eq!(points, before);
}

#[test]
fn prepared_matcher_reuses_owned_scans() {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings = vec![8.0; angles.len()];
    let valid = vec![true; angles.len()];
    let reference =
        PreparedPolarScan::from_polar(angles.clone(), readings.clone(), valid.clone()).unwrap();
    let sensor = PreparedPolarScan::from_polar(angles, readings, valid).unwrap();
    let mut workspace =
        PreparedMatcher::new(Matcher::pose_only(Params::default()), reference, sensor).unwrap();
    assert!(workspace.match_once().unwrap().valid);
    assert_eq!(workspace.reference().len(), 21);
    assert_eq!(workspace.capacities(), (21, 21));
    workspace.update_sensor(&[8.0; 21], &[true; 21]).unwrap();
    let points: Vec<[f64; 2]> = (0..21)
        .map(|i| {
            let a = -1.0 + i as f64 * 0.1;
            [8.0 * a.cos(), 8.0 * a.sin()]
        })
        .collect();
    workspace
        .update_sensor_cartesian(&points, &[true; 21])
        .unwrap();
    let mut result = csm_rs::SmResult::default();
    workspace.match_once_into(&mut result).unwrap();
    assert!(result.valid);
    let mut snapshots = 0;
    workspace
        .match_once_traced(|snapshot| {
            snapshots += 1;
            assert!(snapshot.pose.iter().all(|v| v.is_finite()));
        })
        .unwrap();
    assert!(snapshots > 0);
}
