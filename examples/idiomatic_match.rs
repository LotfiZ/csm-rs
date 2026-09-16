//! Minimal immutable API example.
use csm_rs::{Matcher, Params, PolarScan};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let angles: Vec<f64> = (0..21).map(|i| -1.0 + i as f64 * 0.1).collect();
    let readings: Vec<f64> = angles.iter().map(|a| 8.0 + 0.2 * a.cos()).collect();
    let valid = vec![true; angles.len()];
    let reference = PolarScan::new(&angles, &readings, &valid)?;
    let sensor = PolarScan::new(&angles, &readings, &valid)?;
    let outcome = Matcher::new(Params::default()).match_polar(reference, sensor)?;
    println!("valid={} pose={:?}", outcome.valid, outcome.pose);
    Ok(())
}
