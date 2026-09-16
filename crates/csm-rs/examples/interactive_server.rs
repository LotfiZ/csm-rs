//! Serve a minimal browser UI backed by a real csm-rs match.
use csm_rs::{Matcher, Params, PolarScan};
use std::io::{Read, Write};
use std::net::TcpListener;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let angles: Vec<f64> = (0..61).map(|i| -1.5 + i as f64 * 0.05).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 6.0 + 0.5 * (3.0 * a).sin()).collect();
    let valid = vec![true; angles.len()];
    let outcome = Matcher::new(Params::default()).match_polar(
        PolarScan::new(&angles, &readings, &valid)?,
        PolarScan::new(&angles, &readings, &valid)?,
    )?;
    let body = format!("<h1>csm-rs interactive match</h1><p>pose={:?}, valid={}</p><p>This page is served by the Rust matcher example.</p>", outcome.pose, outcome.valid);
    let listener = TcpListener::bind("127.0.0.1:7878")?;
    println!("open http://127.0.0.1:7878");
    for mut stream in listener.incoming().flatten() {
        let mut request = [0; 512];
        let _ = stream.read(&mut request);
        let response = format!("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
        stream.write_all(response.as_bytes())?;
    }
    Ok(())
}
