use csm_rs::laser_data::LaserData;
use csm_rs::math::corr_hash;
use csm_rs::params::{CorrespondenceSearch, DistanceMetric, Params};
use csm_rs::{sm_icp, SmResult};
use serde::Deserialize;

const POSE_TOLERANCE: f64 = 1e-9;
const ERROR_TOLERANCE: f64 = 1e-9;
const UPSTREAM_ERROR_TOLERANCE: f64 = 2e-9;
const COVARIANCE_RELATIVE_TOLERANCE: f64 = 1e-6;

#[derive(Debug, Deserialize)]
struct Fixture {
    schema: String,
    cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
struct Case {
    name: String,
    #[serde(default)]
    category: String,
    params: FixtureParams,
    laser_ref: FixtureScan,
    laser_sens: FixtureScan,
    expected: ExpectedResult,
}

#[derive(Debug, Deserialize)]
struct FixtureScan {
    min_theta: f64,
    max_theta: f64,
    #[serde(default)]
    theta: Vec<f64>,
    readings: Vec<Option<f64>>,
    #[serde(default)]
    valid: Vec<i32>,
    #[serde(default)]
    readings_sigma: Vec<Option<f64>>,
    #[serde(default)]
    true_alpha: Vec<Option<f64>>,
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
    correspondence_hash: u32,
    first_correspondence_hash: u32,
    nvalid: i32,
    #[serde(default)]
    cov_x: Option<[[f64; 3]; 3]>,
    #[serde(default)]
    dx_dy1: Option<Vec<Vec<Option<f64>>>>,
    #[serde(default)]
    dx_dy2: Option<Vec<Vec<Option<f64>>>>,
}

fn read_fixture() -> Fixture {
    let json = include_str!("fixtures/identity.json");
    serde_json::from_str(json).expect("identity fixture must match its schema")
}

fn build_scan(scan: &FixtureScan) -> LaserData {
    let mut result = LaserData::new(scan.readings.len(), scan.min_theta, scan.max_theta);
    if !scan.theta.is_empty() {
        assert_eq!(scan.theta.len(), result.nrays);
        result.theta.clone_from(&scan.theta);
    }
    if scan.valid.is_empty() {
        result.valid.fill(true);
    } else {
        assert_eq!(scan.valid.len(), result.nrays);
        for (valid, &value) in result.valid.iter_mut().zip(&scan.valid) {
            *valid = value != 0;
        }
    }
    for (reading, value) in result.readings.iter_mut().zip(&scan.readings) {
        *reading = value.unwrap_or(f64::NAN);
    }
    for (i, sigma) in scan.readings_sigma.iter().enumerate() {
        if let Some(sigma) = sigma {
            result.readings_sigma[i] = *sigma;
        }
    }
    for (i, alpha) in scan.true_alpha.iter().enumerate() {
        if let Some(alpha) = alpha {
            result.true_alpha[i] = *alpha;
        }
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

fn run_case(params: &Params, case: &Case) -> (SmResult, LaserData) {
    let mut laser_ref = build_scan(&case.laser_ref);
    let mut laser_sens = build_scan(&case.laser_sens);
    let mut result = SmResult::default();
    sm_icp(params, &mut laser_ref, &mut laser_sens, &mut result)
        .expect("golden fixture scans must pass input validation");
    (result, laser_sens)
}

fn correspondence_keys(scan: &LaserData) -> Vec<Option<(i32, i32)>> {
    scan.corr
        .iter()
        .map(|corr| corr.valid.then_some((corr.j1, corr.j2)))
        .collect()
}

fn assert_relative(actual: f64, expected: f64, label: &str) {
    let scale = expected.abs().max(1e-12);
    assert!(
        (actual - expected).abs() <= COVARIANCE_RELATIVE_TOLERANCE * scale,
        "{label}: {actual} != {expected}"
    );
}

fn assert_dynamic_matrix(
    actual: &csm_rs::math::Matrix,
    expected: &[Vec<Option<f64>>],
    label: &str,
) {
    assert_eq!(actual.rows(), expected.len(), "{label}: row count");
    for (row, expected_values) in expected.iter().enumerate() {
        assert_eq!(
            actual.data[row].len(),
            expected_values.len(),
            "{label}: column count at row {row}"
        );
        for (col, expected) in expected_values.iter().enumerate() {
            let actual = actual.data[row][col];
            match expected {
                Some(expected) => {
                    assert_relative(actual, *expected, &format!("{label}[{row}][{col}]"))
                }
                None => assert!(actual.is_nan(), "{label}[{row}][{col}] should be NaN"),
            }
        }
    }
}

#[test]
fn fixture_cases_match_c_reference_and_each_strategy() {
    let fixture = read_fixture();
    assert_eq!(fixture.schema, "csm-rs-fixture/v1");

    for case in &fixture.cases {
        let params = build_params(&case.params);
        let (result, laser_sens) = run_case(&params, case);

        assert_eq!(result.valid, case.expected.valid, "case {}", case.name);
        assert_eq!(
            result.iterations, case.expected.iterations,
            "case {}",
            case.name
        );
        assert_eq!(result.nvalid, case.expected.nvalid, "case {}", case.name);
        // C and Rust accumulate imported log errors through different native
        // math paths; keep the wider tolerance local to those upstream cases.
        let error_tolerance = if case.category == "upstream_misc_tests" {
            UPSTREAM_ERROR_TOLERANCE
        } else {
            ERROR_TOLERANCE
        };
        assert!(
            (result.error - case.expected.error).abs() <= error_tolerance,
            "case {}: {} != {}",
            case.name,
            result.error,
            case.expected.error
        );
        for (actual, expected) in result.x.iter().zip(case.expected.x) {
            assert!(
                (actual - expected).abs() <= POSE_TOLERANCE,
                "case {}: {actual} != {expected}",
                case.name
            );
        }

        let final_keys = correspondence_keys(&laser_sens);
        assert_eq!(
            corr_hash(&final_keys),
            case.expected.correspondence_hash,
            "case {}: final correspondence hash",
            case.name
        );

        // Compare the configured search at the first iteration, before
        // restart or later correspondence changes can hide a divergence.
        let mut first_params = params.clone();
        first_params.stopping.max_iterations = 1;
        first_params.restart.enabled = false;
        let (_, first_sens) = run_case(&first_params, case);
        assert_eq!(
            corr_hash(&correspondence_keys(&first_sens)),
            case.expected.first_correspondence_hash,
            "case {}: first correspondence hash",
            case.name
        );

        match (
            case.expected.cov_x,
            case.expected.dx_dy1.as_ref(),
            case.expected.dx_dy2.as_ref(),
        ) {
            (Some(expected), Some(expected_dx_dy1), Some(expected_dx_dy2)) => {
                assert!(params.do_compute_covariance, "case {}", case.name);
                let actual = result.cov_x.as_ref().expect("covariance result");
                for (row, values) in expected.iter().enumerate() {
                    for (col, expected) in values.iter().enumerate() {
                        assert_relative(
                            actual.data[row][col],
                            *expected,
                            &format!("case {}: cov_x[{row}][{col}]", case.name),
                        );
                    }
                }
                assert_dynamic_matrix(
                    result.dx_dy1.as_ref().expect("dx_dy1 result"),
                    expected_dx_dy1,
                    &format!("case {}: dx_dy1", case.name),
                );
                assert_dynamic_matrix(
                    result.dx_dy2.as_ref().expect("dx_dy2 result"),
                    expected_dx_dy2,
                    &format!("case {}: dx_dy2", case.name),
                );
            }
            (None, None, None) => {
                assert!(result.cov_x.is_none(), "case {}", case.name);
                assert!(result.dx_dy1.is_none(), "case {}", case.name);
                assert!(result.dx_dy2.is_none(), "case {}", case.name);
            }
            _ => panic!("case {} has incomplete covariance expectation", case.name),
        }

        // C's tricks search intentionally omits the optional alpha test. Keep
        // that reference behavior and validate alpha-enabled cases through
        // their configured C path; strategy equivalence is asserted below
        // when both C strategies share the same semantics.
        if params.correspondence.do_alpha_test {
            continue;
        }

        // The upstream stallo2 regression log is intentionally retained as a
        // smart-path corpus case even though C's smart and naive searches
        // diverge on its invalid sectors. Its configured path and first hash
        // are checked against C above; synthetic cases retain the strategy
        // equivalence property below.
        if case.name == "upstream_failure1_stallo2" {
            let mut tricks_params = params.clone();
            tricks_params.correspondence.search = CorrespondenceSearch::Tricks;
            let (_, tricks_sens) = run_case(&tricks_params, case);
            let mut naive_params = params.clone();
            naive_params.correspondence.search = CorrespondenceSearch::Naive;
            let (_, naive_sens) = run_case(&naive_params, case);
            assert_ne!(
                correspondence_keys(&tricks_sens),
                correspondence_keys(&naive_sens),
                "case {}: upstream C corpus is expected to expose its known smart/naive divergence",
                case.name
            );
            continue;
        }

        // Run both strategies through the public seam and compare the final
        // correspondence key/hash as well as the match result, regardless of
        // which strategy the fixture happens to configure.
        let mut tricks_params = params.clone();
        tricks_params.correspondence.search = CorrespondenceSearch::Tricks;
        let (tricks_result, tricks_sens) = run_case(&tricks_params, case);
        let mut naive_params = params.clone();
        naive_params.correspondence.search = CorrespondenceSearch::Naive;
        let (naive_result, naive_sens) = run_case(&naive_params, case);

        assert_eq!(
            tricks_result.valid, naive_result.valid,
            "case {}",
            case.name
        );
        assert_eq!(
            tricks_result.iterations, naive_result.iterations,
            "case {}",
            case.name
        );
        assert_eq!(
            tricks_result.nvalid, naive_result.nvalid,
            "case {}",
            case.name
        );
        assert!(
            (tricks_result.error - naive_result.error).abs() <= ERROR_TOLERANCE,
            "case {}: {} != {}",
            case.name,
            tricks_result.error,
            naive_result.error
        );
        for (actual, expected) in tricks_result.x.iter().zip(naive_result.x) {
            assert!(
                (actual - expected).abs() <= POSE_TOLERANCE,
                "case {}: {actual} != {expected}",
                case.name
            );
        }
        let tricks_keys = correspondence_keys(&tricks_sens);
        let naive_keys = correspondence_keys(&naive_sens);
        assert_eq!(tricks_keys, naive_keys, "case {}", case.name);
        assert_eq!(
            corr_hash(&tricks_keys),
            corr_hash(&naive_keys),
            "case {}",
            case.name
        );

        // Limit each public call to one pass so the correspondence fields
        // still contain the first ICP iteration when their hashes are read.
        // Force both strategies independently so a fixture's configured
        // search mode cannot hide a broken alternate path.
        let mut first_tricks_params = params.clone();
        first_tricks_params.correspondence.search = CorrespondenceSearch::Tricks;
        first_tricks_params.stopping.max_iterations = 1;
        let mut first_naive_params = params.clone();
        first_naive_params.correspondence.search = CorrespondenceSearch::Naive;
        first_naive_params.stopping.max_iterations = 1;
        let mut first_tricks_ref = build_scan(&case.laser_ref);
        let mut first_tricks_sens = build_scan(&case.laser_sens);
        let mut first_naive_ref = build_scan(&case.laser_ref);
        let mut first_naive_sens = build_scan(&case.laser_sens);
        let mut first_tricks_result = SmResult::default();
        let mut first_naive_result = SmResult::default();
        sm_icp(
            &first_tricks_params,
            &mut first_tricks_ref,
            &mut first_tricks_sens,
            &mut first_tricks_result,
        )
        .expect("golden fixture scans must pass input validation");
        sm_icp(
            &first_naive_params,
            &mut first_naive_ref,
            &mut first_naive_sens,
            &mut first_naive_result,
        )
        .expect("golden fixture scans must pass input validation");
        let first_tricks_keys: Vec<_> = first_tricks_sens
            .corr
            .iter()
            .map(|corr| corr.valid.then_some((corr.j1, corr.j2)))
            .collect();
        let first_naive_keys: Vec<_> = first_naive_sens
            .corr
            .iter()
            .map(|corr| corr.valid.then_some((corr.j1, corr.j2)))
            .collect();
        assert_eq!(first_tricks_keys, first_naive_keys, "case {}", case.name);
        assert_eq!(
            corr_hash(&first_tricks_keys),
            corr_hash(&first_naive_keys),
            "case {}",
            case.name
        );
    }
}

#[test]
fn restart_fixture_requires_the_restart_shell() {
    let fixture = read_fixture();
    let case = fixture
        .cases
        .iter()
        .find(|case| case.name == "restart_probe")
        .expect("restart fixture case");
    let params = build_params(&case.params);
    let (with_restart, _) = run_case(&params, case);

    let mut without_restart_params = params;
    without_restart_params.restart.enabled = false;
    let (without_restart, _) = run_case(&without_restart_params, case);

    assert_eq!(with_restart.iterations, 14);
    assert_eq!(without_restart.iterations, 2);
    assert!(with_restart.error < without_restart.error);
    assert!(with_restart.x[0].abs() < 0.02);
    assert!(without_restart.x[0].abs() > 0.01);
}

#[test]
fn alpha_fixtures_cover_default_visibility_and_rejection_paths() {
    let fixture = read_fixture();
    let default_case = fixture
        .cases
        .iter()
        .find(|case| case.name == "alpha_visibility_tricks")
        .expect("default alpha/visibility fixture case");
    assert_eq!(default_case.params.use_corr_tricks, 1);
    assert_eq!(default_case.params.do_alpha_test, 1);
    assert_eq!(default_case.params.do_visibility_test, 1);

    let rejection_case = fixture
        .cases
        .iter()
        .find(|case| case.name == "alpha_rejection")
        .expect("alpha rejection fixture case");
    assert_eq!(rejection_case.params.use_corr_tricks, 0);
    assert_eq!(rejection_case.params.do_alpha_test, 1);
    assert_eq!(rejection_case.params.max_angular_correction_deg, 0.0);
    let alpha_params = build_params(&rejection_case.params);
    let (with_alpha, _) = run_case(&alpha_params, rejection_case);
    let mut without_alpha_params = alpha_params;
    without_alpha_params.correspondence.do_alpha_test = false;
    let (without_alpha, _) = run_case(&without_alpha_params, rejection_case);
    assert!(
        without_alpha.nvalid > with_alpha.nvalid,
        "alpha rejection should remove correspondences"
    );
}

#[test]
fn weighting_fixtures_cover_each_branch_and_computed_alpha_fallback() {
    let fixture = read_fixture();
    for (name, ml, sigma) in [
        ("ml_weights", true, false),
        ("sigma_weights", false, true),
        ("computed_alpha_weights", true, false),
    ] {
        let case = fixture
            .cases
            .iter()
            .find(|case| case.name == name)
            .unwrap_or_else(|| panic!("{name} fixture case"));
        assert_eq!(case.params.use_ml_weights != 0, ml, "case {name}");
        assert_eq!(case.params.use_sigma_weights != 0, sigma, "case {name}");
        if name == "ml_weights" {
            assert!(case.laser_ref.true_alpha.iter().all(Option::is_some));
        }
        if name == "sigma_weights" {
            assert!(case.laser_sens.readings_sigma.iter().any(Option::is_none));
        }
        if name == "computed_alpha_weights" {
            assert!(case.laser_ref.true_alpha.is_empty());
            assert_eq!(case.params.do_alpha_test, 1);
        }

        let weighted_params = build_params(&case.params);
        let (weighted, _) = run_case(&weighted_params, case);
        let mut unweighted_params = weighted_params;
        unweighted_params.weights.ml = false;
        unweighted_params.weights.sigma = false;
        let (unweighted, _) = run_case(&unweighted_params, case);
        assert!(
            weighted
                .x
                .iter()
                .zip(unweighted.x)
                .any(|(weighted, unweighted)| (weighted - unweighted).abs() > 1e-6),
            "weight fields should affect the solved pose for {name}"
        );
    }
}

#[test]
fn fixture_corpus_covers_reference_logs_and_edge_cases() {
    let fixture = read_fixture();
    let categories: std::collections::BTreeSet<_> = fixture
        .cases
        .iter()
        .map(|case| case.category.as_str())
        .collect();

    for category in [
        "upstream_misc_tests",
        "degenerate_geometry",
        "non_convergence",
    ] {
        assert!(categories.contains(category), "missing {category} corpus");
    }

    let upstream: Vec<_> = fixture
        .cases
        .iter()
        .filter(|case| case.category == "upstream_misc_tests")
        .collect();
    assert_eq!(upstream.len(), 2);
    assert!(upstream
        .iter()
        .all(|case| !case.laser_ref.theta.is_empty() && !case.laser_ref.valid.is_empty()));
    assert!(upstream.iter().any(|case| {
        case.laser_ref.readings.iter().any(Option::is_none)
            || case.laser_sens.readings.iter().any(Option::is_none)
    }));

    let degenerate = fixture
        .cases
        .iter()
        .find(|case| case.category == "degenerate_geometry")
        .expect("degenerate geometry fixture case");
    assert_eq!(
        degenerate.laser_ref.theta.len(),
        degenerate.laser_ref.readings.len()
    );
    assert_eq!(
        degenerate.laser_ref.valid.len(),
        degenerate.laser_ref.readings.len()
    );
    assert_eq!(
        degenerate
            .laser_ref
            .valid
            .iter()
            .filter(|&&value| value != 0)
            .count(),
        3
    );

    let non_convergence = fixture
        .cases
        .iter()
        .find(|case| case.category == "non_convergence")
        .expect("non-convergence fixture case");
    assert_eq!(non_convergence.params.max_iterations, 1);
    assert!(non_convergence.expected.iterations > non_convergence.params.max_iterations);
}
