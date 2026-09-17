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
