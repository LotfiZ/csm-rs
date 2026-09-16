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
    let body = format!("<!doctype html><meta charset='utf-8'><title>csm-rs</title><style>body{{font:16px sans-serif;background:#111827;color:#eee;max-width:720px;margin:2rem auto}}svg{{width:100%;background:#0b1220}}button{{margin:.5rem;padding:.5rem}}</style><h1>csm-rs interactive match</h1><svg id='view' viewBox='0 0 600 300'><circle id='robot' cx='300' cy='150' r='8' fill='#f59e0b'/></svg><p>pose={:?}, valid={}</p><button id='play'>play</button><button id='reset'>reset</button><script>let x=300,timer;const r=document.querySelector('#robot');function draw(){{r.setAttribute('cx',x)}}document.querySelector('#play').onclick=()=>{{clearInterval(timer);timer=setInterval(()=>{{x=x>560?40:x+4;draw()}},30)}};document.querySelector('#reset').onclick=()=>{{clearInterval(timer);x=300;draw()}};draw();</script>", outcome.pose, outcome.valid);
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
