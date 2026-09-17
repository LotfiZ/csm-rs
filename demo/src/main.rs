//! Entry point for the local csm-rs demo server.

use csm_rs_demo::app;

#[tokio::main]
async fn main() {
    let address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:7878".to_owned());
    let listener = tokio::net::TcpListener::bind(&address)
        .await
        .expect("bind demo address");
    println!("csm-rs demo listening on http://{address}");
    axum::serve(listener, app()).await.expect("serve demo");
}
