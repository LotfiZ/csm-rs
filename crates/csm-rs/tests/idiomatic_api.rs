use csm_rs::{
    CartesianScan, MatchOutcome, MatchStatus, Matcher, Params, PolarScan, PreparedPolarScan,
    SmResult,
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
    let updated = vec![7.5; scan.len()];
    scan.update(&updated, &vec![true; scan.len()]).unwrap();
    assert_eq!(scan.readings(), updated.as_slice());
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
    let updated: Vec<[f64; 2]> = points.iter().map(|p| [p[0] * 0.9, p[1] * 0.9]).collect();
    scan.update_cartesian(&updated, &valid).unwrap();
    assert!((scan.readings()[10] - 7.2).abs() < 1e-9);
}

#[test]
fn cartesian_validation_reports_offending_ray() {
    let mut points = vec![[1.0, 0.0]; 21];
    points[7][0] = f64::NAN;
    let err = CartesianScan::new(&points, &vec![true; 21]).unwrap_err();
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
