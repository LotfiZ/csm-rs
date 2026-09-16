//! Render one or two whitespace-delimited Cartesian scans as standalone HTML.
//! Each input line is `x y`; pass `reference.txt sensor.txt` to compare scans.
use csm_rs::{CartesianScan, Matcher};
use std::fs;

fn read(path: &str) -> Result<Vec<[f64; 2]>, Box<dyn std::error::Error>> {
    fs::read_to_string(path)?
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|line| {
            let mut fields = line.split_whitespace();
            Ok([
                fields.next().ok_or("missing x")?.parse()?,
                fields.next().ok_or("missing y")?.parse()?,
            ])
        })
        .collect::<Result<_, Box<dyn std::error::Error>>>()
}

fn dots(points: &[[f64; 2]], color: &str) -> String {
    points
        .iter()
        .map(|p| {
            format!(
                "<circle cx='{:.2}' cy='{:.2}' r='2' fill='{color}'/>",
                300.0 + p[0] * 25.0,
                150.0 - p[1] * 25.0
            )
        })
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let reference = read(
        std::env::args()
            .nth(1)
            .ok_or("usage: import_scan reference.txt [sensor.txt]")?
            .as_str(),
    )?;
    let sensor = std::env::args().nth(2).map(|p| read(&p)).transpose()?;
    let valid = vec![true; reference.len()];
    let mut text = dots(&reference, "#60a5fa");
    let mut result = String::from("no sensor scan supplied");
    if let Some(sensor) = sensor {
        let sv = vec![true; sensor.len()];
        let outcome = Matcher::default().match_cartesian(
            CartesianScan::new(&reference, &valid)?,
            CartesianScan::new(&sensor, &sv)?,
        )?;
        text.push_str(&dots(&sensor, "#f472b6"));
        result = format!(
            "pose={:?}, valid={}, termination={:?}",
            outcome.pose, outcome.valid, outcome.termination
        );
    }
    println!("<!doctype html><meta charset='utf-8'><title>csm-rs imported scan</title><style>body{{font:16px sans-serif;background:#111827;color:#eee;max-width:720px;margin:2rem auto}}svg{{width:100%;background:#0b1220}}</style><h1>Imported scan</h1><svg viewBox='0 0 600 300'>{text}</svg><p>{result}</p>");
    Ok(())
}
