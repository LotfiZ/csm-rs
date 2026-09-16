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
    let reference_points: String = angles
        .iter()
        .zip(&readings)
        .map(|(a, r)| {
            format!(
                "<circle cx='{:.1}' cy='{:.1}' r='2' fill='#60a5fa'/>",
                300.0 + r * a.cos() * 25.0,
                150.0 - r * a.sin() * 25.0
            )
        })
        .collect();
    let (tx, ty, theta) = (outcome.pose[0], outcome.pose[1], outcome.pose[2]);
    let sensor_points: String = angles
        .iter()
        .zip(&readings)
        .map(|(a, r)| {
            let (x, y) = (r * a.cos(), r * a.sin());
            let (c, s) = (theta.cos(), theta.sin());
            format!(
                "<circle cx='{:.1}' cy='{:.1}' r='2' fill='#f472b6'/>",
                300.0 + (c * x - s * y + tx) * 25.0,
                150.0 - (s * x + c * y + ty) * 25.0
            )
        })
        .collect();
    let body = format!("<!doctype html><meta charset='utf-8'><title>csm-rs</title><style>body{{font:16px sans-serif;background:#111827;color:#eee;max-width:720px;margin:2rem auto}}svg{{width:100%;background:#0b1220}}button,input{{margin:.5rem;padding:.5rem}}</style><h1>csm-rs interactive match</h1><p><span style='color:#60a5fa'>● reference</span> <span style='color:#f472b6'>● transformed sensor</span></p><label>motion <input id='motion' type='range' min='0' max='40' value='0'></label><label>noise <input id='noise' type='range' min='0' max='80' value='0'></label><label>missing rays <input id='missing' type='range' min='0' max='80' value='0'></label><svg id='view' viewBox='0 0 600 300'>{reference_points}<g id='sensor'>{sensor_points}</g><circle id='robot' cx='300' cy='150' r='8' fill='#f59e0b'/></svg><p>pose={:?}, valid={}</p><button id='play'>play</button><button id='reset'>reset</button><script>let x=300,timer;const r=document.querySelector('#robot'),s=document.querySelector('#sensor'),motion=document.querySelector('#motion'),noise=document.querySelector('#noise'),missing=document.querySelector('#missing');function draw(){{r.setAttribute('cx',x);s.style.transform=`translate(${{motion.value}}px,0)`;[...s.children].forEach((p,i)=>{{p.style.opacity=i%100<missing.value?'0':(1-noise.value/200)}})}}[motion,noise,missing].forEach(e=>e.oninput=draw);document.querySelector('#play').onclick=()=>{{clearInterval(timer);timer=setInterval(()=>{{x=x>560?40:x+4;draw()}},30)}};document.querySelector('#reset').onclick=()=>{{clearInterval(timer);x=300;motion.value=noise.value=missing.value=0;draw()}};document.querySelector('#reset').click();</script>", outcome.pose, outcome.valid);
    let address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:7878".to_owned());
    let listener = TcpListener::bind(&address)?;
    println!("open http://{address}");
    for mut stream in listener.incoming().flatten() {
        let mut request = [0; 512];
        let _ = stream.read(&mut request);
        let response = format!("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", body.len(), body);
        stream.write_all(response.as_bytes())?;
    }
    Ok(())
}
