//! A small, readable CSM example for people new to scan matching and Rust.
//!
//! Run it from the repository root with:
//!
//!     cargo run -p csm-rs --example scan_matching

use csm_rs::{MatchOutcome, Matcher, Params, PolarScan, Pose};

const ROOM_HALF_SIZE: f64 = 5.0;
const FIRST_SENSOR_POSE: Pose = Pose::new(0.30, -0.20, 0.04);

/// Return the distance from a robot pose to the nearest wall of a square room.
///
/// The room is centered at the origin and has walls at x/y = +/- 5. The
/// returned distance is a synthetic lidar reading; no physical sensor is
/// required to run this example.
fn square_room_reading(angle: f64, pose: Pose) -> f64 {
    let world_angle = angle + pose.theta;
    let direction = [world_angle.cos(), world_angle.sin()];
    let mut distances = Vec::with_capacity(2);

    if direction[0] > 0.0 {
        distances.push((ROOM_HALF_SIZE - pose.x) / direction[0]);
    } else if direction[0] < 0.0 {
        distances.push((-ROOM_HALF_SIZE - pose.x) / direction[0]);
    }

    if direction[1] > 0.0 {
        distances.push((ROOM_HALF_SIZE - pose.y) / direction[1]);
    } else if direction[1] < 0.0 {
        distances.push((-ROOM_HALF_SIZE - pose.y) / direction[1]);
    }

    distances
        .into_iter()
        .filter(|distance| *distance > 0.0)
        .fold(f64::INFINITY, f64::min)
}

fn build_scan(angles: &[f64], pose: Pose) -> (Vec<f64>, Vec<bool>) {
    let readings = angles
        .iter()
        .map(|&angle| square_room_reading(angle, pose))
        .collect();
    (readings, vec![true; angles.len()])
}

fn print_scan_table(reference: &PolarScan<'_>, sensor: &PolarScan<'_>) {
    println!("\nLASER DATA");
    println!("Each row is one laser beam. Distances are in metres, angles in radians.");
    println!(
        "{:<5} {:>10} {:>14} {:>14} {:>8}",
        "ray", "angle", "reference", "sensor", "valid"
    );
    println!("{}", "-".repeat(58));

    for index in 0..reference.len() {
        println!(
            "{:<5} {:>9.1}° {:>14.3} {:>14.3} {:>8}",
            index,
            reference.angles()[index].to_degrees(),
            reference.readings()[index],
            sensor.readings()[index],
            if reference.valid()[index] && sensor.valid()[index] {
                "yes"
            } else {
                "no"
            }
        );
    }
}

fn print_result(result: &MatchOutcome) {
    println!("\nMATCHING RESULT");
    println!("{}", "-".repeat(58));
    println!(
        "status:          {}",
        if result.valid {
            "SUCCESS: scans matched"
        } else {
            "NO MATCH: inputs were valid, but ICP did not converge"
        }
    );
    println!("termination:     {:?}", result.termination);
    println!("estimated x:     {:>10.5}", result.pose.x);
    println!("estimated y:     {:>10.5}", result.pose.y);
    println!(
        "estimated angle: {:>10.5} rad ({:>8.3}°)",
        result.pose.theta,
        result.pose.theta.to_degrees()
    );
    println!("iterations:      {:>10}", result.iterations);
    println!("valid matches:   {:>10}", result.nvalid);
    println!("matching error:  {:>10.6}", result.error);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let angles: Vec<f64> = (0..21)
        .map(|index| -60.0_f64.to_radians() + index as f64 * 120.0_f64.to_radians() / 20.0)
        .collect();

    let (reference_readings, reference_valid) = build_scan(&angles, Pose::IDENTITY);
    let (sensor_readings, sensor_valid) = build_scan(&angles, FIRST_SENSOR_POSE);
    let reference = PolarScan::new(&angles, &reference_readings, &reference_valid)?;
    let sensor = PolarScan::new(&angles, &sensor_readings, &sensor_valid)?;
    print_scan_table(&reference, &sensor);

    println!("\nSCENARIO");
    println!("The reference scan is taken at the center of a square room.");
    println!(
        "The second scan is simulated from x = {:.2}, y = {:.2}, angle = {:.2} rad.",
        FIRST_SENSOR_POSE.x, FIRST_SENSOR_POSE.y, FIRST_SENSOR_POSE.theta
    );
    println!("CSM starts with an identity initial pose and tries to align the scans.");

    let matcher = Matcher::new(Params::default())?;
    let result = matcher.match_polar(reference, sensor)?;
    print_result(&result);

    println!("\nTo try different data, change FIRST_SENSOR_POSE or the room/readings above.");
    println!("For the complete automated suite, run: cargo test --all-targets --all-features");

    Ok(())
}
