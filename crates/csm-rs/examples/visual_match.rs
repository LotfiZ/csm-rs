//! Generate a browser-viewable SVG for a simple scan match.
use csm_rs::{Matcher, Params, PolarScan};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let angles: Vec<f64> = (0..61).map(|i| -1.5 + i as f64 * 0.05).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 6.0 + 0.5 * (3.0 * a).sin()).collect();
    let valid = vec![true; angles.len()];
    let reference = PolarScan::new(&angles, &readings, &valid)?;
    let sensor = PolarScan::new(&angles, &readings, &valid)?;
    let outcome = Matcher::new(Params::default()).match_polar(reference, sensor)?;

    println!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="-300 -300 600 600"><rect x="-300" y="-300" width="600" height="600" fill="#111827"/><g fill="#60a5fa">"##
    );
    for (a, r) in angles.iter().zip(&readings) {
        let x = r * a.cos() * 35.0;
        let y = -r * a.sin() * 35.0;
        println!(r#"<circle cx="{x:.2}" cy="{y:.2}" r="2"/>"#);
    }
    println!(
        r#"</g><text x="-285" y="-270" fill="white" font-family="sans-serif">CSM match: valid={} pose=({:.3}, {:.3}, {:.3})</text></svg>"#,
        outcome.valid, outcome.pose[0], outcome.pose[1], outcome.pose[2]
    );
    Ok(())
}
