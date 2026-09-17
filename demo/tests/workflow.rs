//! Browser-to-Rust workflow tests: the HTTP handler runs the real matcher and
//! the controls change real inputs.

use axum::body::Body;
use csm_rs_demo::{app, FrameResponse};
use serde_json::json;
use tower::ServiceExt;

async fn post_frame(config: serde_json::Value) -> FrameResponse {
    let request = axum::http::Request::builder()
        .method("POST")
        .uri("/api/frame")
        .header("content-type", "application/json")
        .body(Body::from(config.to_string()))
        .expect("request");
    let response = app().oneshot(request).await.expect("router call");
    assert_eq!(response.status(), 200, "frame endpoint should succeed");
    let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .expect("body");
    serde_json::from_slice(&bytes).expect("frame response json")
}

fn base(step: u64) -> serde_json::Value {
    json!({
        "scenario": "asymmetric_room",
        "reference_mode": "fixed",
        "seed": 7,
        "motion": 1.0,
        "noise": 0.01,
        "dropout": 0.0,
        "initial_error": 0.05,
        "step": step,
        "request_id": step + 1
    })
}

#[tokio::test]
async fn index_is_served() {
    let response = app()
        .oneshot(
            axum::http::Request::builder()
                .uri("/")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), 200);
    let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .unwrap();
    assert!(String::from_utf8_lossy(&bytes).contains("csm-rs local demo"));
}

#[tokio::test]
async fn controls_change_real_inputs() {
    let baseline = post_frame(base(4)).await;

    let mut moved = base(4);
    moved["motion"] = json!(2.0);
    assert_ne!(post_frame(moved).await.truth_pose, baseline.truth_pose);

    let mut noisy = base(4);
    noisy["noise"] = json!(0.2);
    assert_ne!(post_frame(noisy).await.reference, baseline.reference);

    let mut guessed = base(4);
    guessed["initial_error"] = json!(1.2);
    assert_ne!(post_frame(guessed).await.initial_pose, baseline.initial_pose);

    let mut dropped = base(4);
    dropped["dropout"] = json!(0.6);
    assert!(
        post_frame(dropped).await.sensor_unaligned.len() < baseline.sensor_unaligned.len(),
        "dropout should remove rays from the actual sensor input"
    );
}

#[tokio::test]
async fn stepping_is_reproducible_for_a_seed() {
    let mut first = Vec::new();
    let mut second = Vec::new();
    for step in 0..6 {
        first.push(post_frame(base(step)).await);
    }
    for step in 0..6 {
        second.push(post_frame(base(step)).await);
    }
    for (a, b) in first.iter().zip(&second) {
        assert_eq!(a.estimated_pose, b.estimated_pose);
        assert_eq!(a.reference, b.reference);
        assert_eq!(a.sensor_unaligned, b.sensor_unaligned);
        assert_eq!(a.termination, b.termination);
    }
}

#[tokio::test]
async fn different_seeds_produce_different_inputs() {
    let mut other = base(4);
    other["seed"] = json!(1234);
    let a = post_frame(base(4)).await;
    let b = post_frame(other).await;
    assert_ne!(a.reference, b.reference);
}

#[tokio::test]
async fn response_echoes_request_id_for_stale_guarding() {
    let mut first = base(1);
    first["request_id"] = json!(11);
    let mut second = base(2);
    second["request_id"] = json!(12);
    assert_eq!(post_frame(first).await.request_id, 11);
    assert_eq!(post_frame(second).await.request_id, 12);
}

#[tokio::test]
async fn scenario_changes_geometry_and_reference() {
    let mut corridor = base(3);
    corridor["scenario"] = json!("ambiguous_corridor");
    let mut partial = base(3);
    partial["scenario"] = json!("partial_overlap");
    assert_ne!(
        post_frame(base(3)).await.reference,
        post_frame(corridor).await.reference
    );
    assert_ne!(
        post_frame(base(3)).await.reference,
        post_frame(partial).await.reference
    );
}

#[tokio::test]
async fn reference_policy_switching_replaces_the_reference() {
    let mut previous = base(6);
    previous["reference_mode"] = json!("previous_frame");
    let fixed = post_frame(base(6)).await;
    let previous = post_frame(previous).await;
    assert_ne!(fixed.reference, previous.reference);
    assert_eq!(previous.reference_mode, "previous_frame");
}

#[tokio::test]
async fn previous_frame_mode_accumulates_drift() {
    let mut early = base(3);
    early["reference_mode"] = json!("previous_frame");
    early["initial_error"] = json!(0.3);
    let mut late = base(12);
    late["reference_mode"] = json!("previous_frame");
    late["initial_error"] = json!(0.3);

    let early = post_frame(early).await;
    let late = post_frame(late).await;
    let drift = |r: &FrameResponse| r.drift_pose[0].hypot(r.drift_pose[1]);
    assert!(
        drift(&late) > drift(&early),
        "accumulated drift should grow with the number of composed frames: {} vs {}",
        drift(&late),
        drift(&early)
    );
}

#[tokio::test]
async fn advanced_configuration_changes_the_result() {
    let mut strict = base(4);
    strict["max_correspondence_dist"] = json!(0.0001);
    let strict = post_frame(strict).await;
    assert!(!strict.valid, "an impossible correspondence distance must fail");
    assert!(!strict.accepted);
    assert_eq!(strict.termination, "NoCorrespondences");
}

#[tokio::test]
async fn iteration_tracing_is_opt_in_and_agrees_with_plain_matching() {
    let plain = post_frame(base(4)).await;
    assert!(plain.trace.is_none(), "tracing must be opt-in");
    assert_eq!(plain.normal_ms, 0.0);
    assert_eq!(plain.instrumented_ms, 0.0);

    let mut traced_config = base(4);
    traced_config["trace"] = json!(true);
    let traced = post_frame(traced_config).await;

    let trace = traced.trace.as_ref().expect("trace requested");
    assert!(!trace.is_empty(), "trace should contain real iterations");
    assert!(traced.normal_ms.is_finite() && traced.instrumented_ms.is_finite());
    assert!(
        traced.normal_ms > 0.0 && traced.instrumented_ms > 0.0,
        "both timings must be reported separately"
    );
    assert!(
        trace
            .iter()
            .any(|iteration| !iteration.correspondences.is_empty()),
        "traced iterations should include correspondences"
    );
    for iteration in trace {
        assert!(iteration.pose.iter().all(|value| value.is_finite()));
        assert!(iteration.error.is_finite());
        assert_eq!(
            iteration.valid_correspondences,
            iteration.correspondences.len()
        );
    }
    // Instrumentation must not change the match result.
    for (a, b) in traced.estimated_pose.iter().zip(plain.estimated_pose) {
        assert!((a - b).abs() < 1e-9, "traced pose diverged: {a} != {b}");
    }
    assert_eq!(traced.termination, plain.termination);
}

async fn post_raw(path: &str, body: serde_json::Value) -> (u16, String) {
    let request = axum::http::Request::builder()
        .method("POST")
        .uri(path)
        .header("content-type", "application/json")
        .body(Body::from(body.to_string()))
        .expect("request");
    let response = app().oneshot(request).await.expect("router call");
    let status = response.status().as_u16();
    let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .expect("body");
    (status, String::from_utf8_lossy(&bytes).into_owned())
}

fn sample_pair() -> serde_json::Value {
    let angles: Vec<f64> = (0..61).map(|i| -1.5 + i as f64 * 0.05).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 6.0 + 0.5 * (3.0 * a).sin()).collect();
    json!({
        "format": "csm-rs-scan-pair",
        "version": 1,
        "initial_guess": [0.0, 0.0, 0.0],
        "reference": { "kind": "polar", "angles": angles, "readings": readings, "valid": vec![true; 61] },
        "sensor": { "kind": "polar", "angles": angles, "readings": readings, "valid": vec![true; 61] },
    })
}

#[tokio::test]
async fn imports_a_valid_polar_pair_without_ground_truth() {
    let (status, body) = post_raw("/api/import", sample_pair()).await;
    assert_eq!(status, 200, "{body}");
    let response: csm_rs_demo::ImportResponse = serde_json::from_str(&body).unwrap();
    assert!(response.valid);
    assert!(!response.reference.is_empty());
    // No ground-truth field exists on the response at all.
    let value: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert!(value.get("truth_pose").is_none());
}

#[tokio::test]
async fn imports_a_cartesian_pair() {
    let angles: Vec<f64> = (0..61).map(|i| -1.5 + i as f64 * 0.05).collect();
    let points: Vec<[f64; 2]> = angles
        .iter()
        .map(|a| {
            let r = 6.0 + 0.5 * (3.0 * a).sin();
            [r * a.cos(), r * a.sin()]
        })
        .collect();
    let pair = json!({
        "format": "csm-rs-scan-pair",
        "version": 1,
        "reference": { "kind": "cartesian", "points": points, "valid": vec![true; 61] },
        "sensor": { "kind": "cartesian", "points": points, "valid": vec![true; 61] },
    });
    let (status, body) = post_raw("/api/import", pair).await;
    assert_eq!(status, 200, "{body}");
}

#[tokio::test]
async fn malformed_imports_are_rejected_clearly() {
    let mut mismatched = sample_pair();
    mismatched["sensor"]["readings"] = json!(vec![1.0; 60]);
    let (status, body) = post_raw("/api/import", mismatched).await;
    assert_eq!(status, 400);
    assert!(body.to_lowercase().contains("length"), "{body}");

    // A null reading on a valid ray is malformed input.
    let mut invalid_missing = sample_pair();
    invalid_missing["sensor"]["readings"][3] = serde_json::Value::Null;
    let (status, body) = post_raw("/api/import", invalid_missing).await;
    assert_eq!(status, 400);
    assert!(body.to_lowercase().contains("missing"), "{body}");

    // Missing returns (null + valid=false) are accepted.
    let mut missing = sample_pair();
    missing["sensor"]["readings"][3] = serde_json::Value::Null;
    missing["sensor"]["valid"][3] = json!(false);
    let (status, _) = post_raw("/api/import", missing).await;
    assert_eq!(status, 200);

    let mut unsupported = sample_pair();
    unsupported["version"] = json!(2);
    let (status, body) = post_raw("/api/import", unsupported).await;
    assert_eq!(status, 400);
    assert!(body.contains("version"), "{body}");
}

#[tokio::test]
async fn export_replay_round_trips() {
    let (status, body) = post_raw("/api/export", json!({ "step": 5, "seed": 42 })).await;
    assert_eq!(status, 200, "{body}");
    let record: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert_eq!(record["version"], 1);
    assert!(record["result"]["estimated_pose"].is_array());

    let (status, replay_body) = post_raw("/api/replay", record.clone()).await;
    assert_eq!(status, 200, "{replay_body}");
    let replay: serde_json::Value = serde_json::from_str(&replay_body).unwrap();
    let stored = &record["result"]["estimated_pose"];
    for i in 0..3 {
        let delta = (replay["estimated_pose"][i].as_f64().unwrap()
            - stored[i].as_f64().unwrap())
        .abs();
        assert!(delta < 1e-9, "replay diverged at {i}: {delta}");
    }

    // Unsupported versions are rejected clearly.
    let mut future = record;
    future["version"] = json!(2);
    let (status, body) = post_raw("/api/replay", future).await;
    assert_eq!(status, 400);
    assert!(body.contains("version"), "{body}");
}
