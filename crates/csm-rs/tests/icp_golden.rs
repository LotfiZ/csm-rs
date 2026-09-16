use csm_rs::laser_data::LaserData;
use csm_rs::params::{CorrespondenceSearch, DistanceMetric, Params};
use csm_rs::{sm_icp, SmResult};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct Fixture {
    schema: String,
    cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
struct Case {
    name: String,
    params: FixtureParams,
    laser_ref: FixtureScan,
    laser_sens: FixtureScan,
    expected: ExpectedResult,
}

#[derive(Debug, Deserialize)]
struct FixtureScan {
    min_theta: f64,
    max_theta: f64,
    readings: Vec<f64>,
}

#[derive(Debug, Deserialize)]
struct FixtureParams {
    first_guess: [f64; 3],
    max_angular_correction_deg: f64,
    max_linear_correction: f64,
    max_iterations: i32,
    epsilon_xy: f64,
    epsilon_theta: f64,
    max_correspondence_dist: f64,
    sigma: f64,
    use_corr_tricks: i32,
    restart: i32,
    restart_threshold_mean_error: f64,
    restart_dt: f64,
    restart_dtheta: f64,
    clustering_threshold: f64,
    orientation_neighbourhood: i32,
    use_point_to_line_distance: i32,
    do_alpha_test: i32,
    #[serde(rename = "do_alpha_test_thresholdDeg")]
    do_alpha_test_threshold_deg: f64,
    #[serde(rename = "outliers_maxPerc")]
    outliers_max_perc: f64,
    outliers_adaptive_order: f64,
    outliers_adaptive_mult: f64,
    do_visibility_test: i32,
    outliers_remove_doubles: i32,
    do_compute_covariance: i32,
    min_reading: f64,
    max_reading: f64,
    use_ml_weights: i32,
    use_sigma_weights: i32,
}

#[derive(Debug, Deserialize)]
struct ExpectedResult {
    valid: bool,
    x: [f64; 3],
    error: f64,
    iterations: i32,
    nvalid: i32,
}

fn read_fixture() -> Fixture {
    let json = include_str!("fixtures/identity.json");
    serde_json::from_str(json).expect("identity fixture must match its schema")
}

fn build_scan(scan: &FixtureScan) -> LaserData {
    let mut result = LaserData::new(scan.readings.len(), scan.min_theta, scan.max_theta);
    for (i, &reading) in scan.readings.iter().enumerate() {
        result.valid[i] = true;
        result.readings[i] = reading;
    }
    result
}

fn build_params(params: &FixtureParams) -> Params {
    let mut result = Params {
        first_guess: params.first_guess,
        ..Params::default()
    };
    result.correction_limits.max_angular_deg = params.max_angular_correction_deg;
    result.correction_limits.max_linear = params.max_linear_correction;
    result.stopping.max_iterations = params.max_iterations;
    result.stopping.epsilon_xy = params.epsilon_xy;
    result.stopping.epsilon_theta = params.epsilon_theta;
    result.correspondence.search = if params.use_corr_tricks != 0 {
        CorrespondenceSearch::Tricks
    } else {
        CorrespondenceSearch::Naive
    };
    result.correspondence.metric = if params.use_point_to_line_distance != 0 {
        DistanceMetric::PointToLine
    } else {
        DistanceMetric::PointToPoint
    };
    result.correspondence.max_dist = params.max_correspondence_dist;
    result.correspondence.sigma = params.sigma;
    result.correspondence.do_alpha_test = params.do_alpha_test != 0;
    result.correspondence.alpha_test_threshold_deg = params.do_alpha_test_threshold_deg;
    result.correspondence.do_visibility_test = params.do_visibility_test != 0;
    result.correspondence.clustering_threshold = params.clustering_threshold;
    result.correspondence.orientation_neighbourhood = params.orientation_neighbourhood;
    result.outliers.max_perc = params.outliers_max_perc;
    result.outliers.adaptive_order = params.outliers_adaptive_order;
    result.outliers.adaptive_mult = params.outliers_adaptive_mult;
    result.outliers.remove_doubles = params.outliers_remove_doubles != 0;
    result.restart.enabled = params.restart != 0;
    result.restart.threshold_mean_error = params.restart_threshold_mean_error;
    result.restart.dt = params.restart_dt;
    result.restart.dtheta = params.restart_dtheta;
    result.do_compute_covariance = params.do_compute_covariance != 0;
    result.reading_bounds.min = params.min_reading;
    result.reading_bounds.max = params.max_reading;
    result.weights.ml = params.use_ml_weights != 0;
    result.weights.sigma = params.use_sigma_weights != 0;
    result
}

#[test]
fn identity_fixture_matches_c_reference() {
    let fixture = read_fixture();
    assert_eq!(fixture.schema, "csm-rs-fixture/v1");
    assert_eq!(fixture.cases.len(), 1);

    let case = &fixture.cases[0];
    assert_eq!(case.name, "identity");
    let params = build_params(&case.params);
    let mut laser_ref = build_scan(&case.laser_ref);
    let mut laser_sens = build_scan(&case.laser_sens);
    let mut result = SmResult::default();

    sm_icp(&params, &mut laser_ref, &mut laser_sens, &mut result);

    assert_eq!(result.valid, case.expected.valid);
    assert_eq!(result.iterations, case.expected.iterations);
    assert_eq!(result.nvalid, case.expected.nvalid);
    assert!((result.error - case.expected.error).abs() <= 1e-9);
    for (actual, expected) in result.x.iter().zip(case.expected.x) {
        assert!((actual - expected).abs() <= 1e-9, "{actual} != {expected}");
    }
}
