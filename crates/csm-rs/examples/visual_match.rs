//! Generate a self-contained interactive HTML visual for a simple scan match.
use csm_rs::{Matcher, Params, PolarScan};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let angles: Vec<f64> = (0..61).map(|i| -1.5 + i as f64 * 0.05).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 6.0 + 0.5 * (3.0 * a).sin()).collect();
    let valid = vec![true; angles.len()];
    let reference = PolarScan::new(&angles, &readings, &valid)?;
    let sensor = PolarScan::new(&angles, &readings, &valid)?;
    let outcome = Matcher::new(Params::default()).match_polar(reference, sensor)?;

    let points: Vec<String> = angles
        .iter()
        .zip(&readings)
        .map(|(a, r)| format!("[{:.4},{:.4}]", r * a.cos() * 35.0, -r * a.sin() * 35.0))
        .collect();
    let html = r##"<!doctype html><meta charset="utf-8"><title>csm-rs visual match</title>
<style>body{margin:0;background:#111827;color:#e5e7eb;font:16px sans-serif}main{max-width:760px;margin:2rem auto}svg{width:100%;background:#0b1220;border-radius:8px}label{display:block;margin:1rem 0}input{width:100%}</style>
<main><h1>csm-rs scan match</h1><svg id="plot" viewBox="-300 -300 600 600"></svg>
<label>Overlay pose: <input id="theta" type="range" min="-1" max="1" value="0" step="0.01"></label><output id="readout"></output></main>
<script>const pts=__POINTS__,svg=document.querySelector('svg'),slider=document.querySelector('#theta'),out=document.querySelector('output');function draw(){let t=+slider.value;svg.innerHTML='<rect x="-300" y="-300" width="600" height="600" fill="#111827"/><g fill="#60a5fa">'+pts.map(p=>`<circle cx="${p[0]}" cy="${p[1]}" r="2"/>`).join('')+'</g><g fill="#f59e0b">'+pts.map(p=>{let x=p[0]*Math.cos(t)-p[1]*Math.sin(t),y=p[0]*Math.sin(t)+p[1]*Math.cos(t);return `<circle cx="${x}" cy="${y}" r="2"/>`}).join('')+'</g>';out.textContent=`estimated pose: (__X__, __Y__, __THETA__) | overlay rotation: ${t.toFixed(2)} rad`}slider.oninput=draw;draw();</script>"##
        .replace("__POINTS__", &format!("[{}]", points.join(",")))
        .replace("__X__", &format!("{:.3}", outcome.pose[0]))
        .replace("__Y__", &format!("{:.3}", outcome.pose[1]))
        .replace("__THETA__", &format!("{:.3}", outcome.pose[2]));
    println!("{html}");
    Ok(())
}
