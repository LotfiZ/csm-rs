//! Public-API validation: malformed input is an error, a failed match is an
//! outcome.

use csm_rs::{Matcher, Params, PolarScan, ScanError};

fn angles(n: usize) -> Vec<f64> {
    (0..n).map(|i| -1.0 + 2.0 * i as f64 / (n - 1) as f64).collect()
}

#[test]
fn mismatched_readings_are_rejected_by_field_name() {
    let theta = angles(10);
    let error = PolarScan::new(&theta, &[5.0; 9], &[true; 10])
        .expect_err("mismatched arrays must be rejected");
    assert_eq!(
        error,
        ScanError::InconsistentLengths {
            field: "readings",
            expected: 10,
            actual: 9,
        }
    );
}

#[test]
fn mismatched_validity_is_rejected_by_field_name() {
    let theta = angles(10);
    let error = PolarScan::new(&theta, &[5.0; 10], &[true; 9])
        .expect_err("mismatched validity must be rejected");
    assert_eq!(
        error,
        ScanError::InconsistentLengths {
            field: "valid",
            expected: 10,
            actual: 9,
        }
    );
}

#[test]
fn explicit_missing_returns_keep_their_position() {
    let theta = angles(10);
    let mut readings = vec![f64::NAN; 10];
    readings[0] = 5.0;
    let mut valid = vec![false; 10];
    valid[0] = true;
    let scan = PolarScan::new(&theta, &readings, &valid).expect("one valid ray is ten percent");
    assert_eq!(scan.len(), 10);
    assert!(scan.valid()[0]);
    assert!(!scan.valid()[1]);
    assert!(scan.readings()[1].is_nan());
}

#[test]
fn all_invalid_scans_are_rejected() {
    let theta = angles(10);
    let error = PolarScan::new(&theta, &[f64::NAN; 10], &[false; 10])
        .expect_err("an all-invalid scan cannot be matched");
    assert_eq!(error, ScanError::TooFewValidRays);
}

#[test]
fn nonfinite_readings_marked_valid_are_rejected() {
    let theta = angles(10);
    let mut readings = vec![5.0; 10];
    readings[4] = f64::NAN;
    let error = PolarScan::new(&theta, &readings, &[true; 10])
        .expect_err("a valid, non-finite reading is malformed input");
    assert_eq!(error, ScanError::BadValidRay(4));
}

#[test]
fn nonfinite_initial_pose_is_rejected() {
    let theta = angles(10);
    let reference = PolarScan::new(&theta, &[5.0; 10], &[true; 10]).unwrap();
    let sensor = PolarScan::new(&theta, &[5.0; 10], &[true; 10]).unwrap();
    let error = Matcher::new(Params::default())
        .unwrap()
        .match_polar_from(reference, sensor, csm_rs::Pose::new(f64::NAN, 0.0, 0.0))
        .expect_err("a non-finite initial pose is malformed input");
    assert_eq!(error, ScanError::NonFiniteGuess);
}

#[test]
fn no_correspondence_is_an_outcome_not_an_error() {
    let theta = angles(10);
    let reference = PolarScan::new(&theta, &[5.0; 10], &[true; 10]).unwrap();
    let sensor = PolarScan::new(&theta, &[6.0; 10], &[true; 10]).unwrap();
    let mut params = Params::default();
    params.correspondence.max_dist = 0.1;
    let outcome = Matcher::new(params)
        .unwrap()
        .match_polar(reference, sensor)
        .expect("well-formed scans return a match outcome");
    assert!(!outcome.valid);
}

#[test]
fn invalid_configuration_is_rejected() {
    let mut params = Params::default();
    params.stopping.max_iterations = -1;
    assert!(Matcher::new(params).is_err());
}
