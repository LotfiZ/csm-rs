use csm_rs::{MatchOutcome, Matcher, Params, PolarScan};

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
    assert!(outcome.pose.iter().all(|v| v.abs() < 1e-6));
    assert_eq!(angles, angles_before);
    assert_eq!(readings, readings_before);
    assert_eq!(valid, valid_before);
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
