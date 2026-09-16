use csm_rs::{sm_icp, LaserData, LaserDataError, Params, SmResult};

fn raw_scan(reading: f64) -> LaserData {
    let theta: Vec<_> = (0..10).map(|i| -1.0 + 2.0 * i as f64 / 9.0).collect();
    LaserData::from_polar(theta, vec![reading; 10], vec![true; 10])
        .expect("test scan must satisfy the CSM input contract")
}

#[test]
fn raw_polar_constructor_preserves_explicit_invalid_rays() {
    let theta: Vec<_> = (0..10).map(|i| -1.0 + 2.0 * i as f64 / 9.0).collect();
    let mut readings = vec![f64::NAN; 10];
    readings[0] = 5.0;
    let valid = [
        true, false, false, false, false, false, false, false, false, false,
    ]
    .to_vec();

    let scan = LaserData::from_polar(theta, readings, valid).expect("one valid ray is 10%");

    assert_eq!(scan.nrays, 10);
    assert_eq!(scan.min_theta, -1.0);
    assert_eq!(scan.max_theta, 1.0);
    assert!(scan.valid[0]);
    assert!(scan.readings[1].is_nan());
}

#[test]
fn raw_polar_constructor_rejects_inconsistent_lengths() {
    let error = LaserData::from_polar(vec![0.0; 10], vec![5.0; 9], vec![true; 10])
        .expect_err("mismatched raw arrays must be rejected");

    assert_eq!(
        error,
        LaserDataError::InconsistentLengths {
            field: "readings",
            expected: 10,
            actual: 9,
        }
    );
}

#[test]
fn raw_polar_constructor_rejects_all_invalid_rays() {
    let theta: Vec<_> = (0..10).map(|i| -1.0 + 2.0 * i as f64 / 9.0).collect();
    let error = LaserData::from_polar(theta, vec![f64::NAN; 10], vec![false; 10])
        .expect_err("an all-invalid scan cannot be matched");

    assert_eq!(error, LaserDataError::TooFewValidRays);
}

#[test]
fn public_matcher_returns_malformed_input_as_an_error() {
    let mut reference = raw_scan(5.0);
    let mut sensor = raw_scan(5.0);
    sensor.theta.pop();
    let mut result = SmResult::default();

    let error = sm_icp(&Params::default(), &mut reference, &mut sensor, &mut result)
        .expect_err("inconsistent scan storage must be an error");

    assert_eq!(
        error,
        LaserDataError::InconsistentLengths {
            field: "theta",
            expected: 10,
            actual: 9,
        }
    );
    assert!(!result.valid);
}

#[test]
fn public_matcher_keeps_no_correspondence_as_a_valid_call() {
    let mut reference = raw_scan(5.0);
    let mut sensor = raw_scan(6.0);
    let mut params = Params::default();
    params.correspondence.max_dist = 0.1;
    let mut result = SmResult::default();

    sm_icp(&params, &mut reference, &mut sensor, &mut result)
        .expect("well-formed scans should return a match result");

    assert!(!result.valid);
}
