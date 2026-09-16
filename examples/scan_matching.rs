//! A small, readable CSM example for people new to scan matching and Rust.
//!
//! Run it from the repository root with:
//!
//!     cargo run -p csm-rs --example scan_matching

use csm_rs::{sm_icp, LaserData, Params, SmResult};

const ROOM_HALF_SIZE: f64 = 5.0;
const FIRST_SENSOR_POSE: [f64; 3] = [0.30, -0.20, 0.04];

/// Return the distance from a robot pose to the nearest wall of a square room.
///
/// The room is centered at the origin and has walls at x/y = +/- 5. The
/// returned distance is a synthetic lidar reading; no physical sensor is
/// required to run this example.
fn square_room_reading(angle: f64, pose: [f64; 3]) -> f64 {
    let world_angle = angle + pose[2];
    let direction = [world_angle.cos(), world_angle.sin()];
    let mut distances = Vec::with_capacity(2);

    if direction[0] > 0.0 {
        distances.push((ROOM_HALF_SIZE - pose[0]) / direction[0]);
    } else if direction[0] < 0.0 {
        distances.push((-ROOM_HALF_SIZE - pose[0]) / direction[0]);
    }

    if direction[1] > 0.0 {
        distances.push((ROOM_HALF_SIZE - pose[1]) / direction[1]);
    } else if direction[1] < 0.0 {
        distances.push((-ROOM_HALF_SIZE - pose[1]) / direction[1]);
    }

    distances
        .into_iter()
        .filter(|distance| *distance > 0.0)
        .fold(f64::INFINITY, f64::min)
}

fn build_scan(angles: &[f64], pose: [f64; 3]) -> Result<LaserData, Box<dyn std::error::Error>> {
    let readings = angles
        .iter()
        .map(|&angle| square_room_reading(angle, pose))
        .collect();
    let valid = vec![true; angles.len()];
    Ok(LaserData::from_polar(angles.to_vec(), readings, valid)?)
}

fn print_scan_table(reference: &LaserData, sensor: &LaserData) {
    println!("\nLASER DATA");
    println!("Each row is one laser beam. Distances use arbitrary length units.");
    println!(
        "{:<5} {:>10} {:>14} {:>14} {:>8}",
        "ray", "angle", "reference", "sensor", "valid"
    );
    println!("{}", "-".repeat(58));

    for index in 0..reference.nrays {
        println!(
            "{:<5} {:>9.1}° {:>14.3} {:>14.3} {:>8}",
            index,
            reference.theta[index].to_degrees(),
            reference.readings[index],
            sensor.readings[index],
            if reference.valid[index] && sensor.valid[index] {
                "yes"
            } else {
                "no"
            }
        );
    }
}

fn print_result(result: &SmResult) {
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
    println!("estimated x:     {:>10.5}", result.x[0]);
    println!("estimated y:     {:>10.5}", result.x[1]);
    println!(
        "estimated angle: {:>10.5} rad ({:>8.3}°)",
        result.x[2],
        result.x[2].to_degrees()
    );
    println!("iterations:      {:>10}", result.iterations);
    println!("valid matches:   {:>10}", result.nvalid);
    println!("matching error:  {:>10.6}", result.error);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let angles: Vec<f64> = (0..21)
        .map(|index| -60.0_f64.to_radians() + index as f64 * 120.0_f64.to_radians() / 20.0)
        .collect();

    let reference_pose = [0.0, 0.0, 0.0];
    let reference = build_scan(&angles, reference_pose)?;
    let sensor = build_scan(&angles, FIRST_SENSOR_POSE)?;
    print_scan_table(&reference, &sensor);

    println!("\nSCENARIO");
    println!("The reference scan is taken at the center of a square room.");
    println!(
        "The second scan is simulated from x = {:.2}, y = {:.2}, angle = {:.2} rad.",
        FIRST_SENSOR_POSE[0], FIRST_SENSOR_POSE[1], FIRST_SENSOR_POSE[2]
    );
    println!("CSM starts with no movement guess and tries to align the scans.");

    let mut reference = reference;
    let mut sensor = sensor;
    let mut result = SmResult::default();
    sm_icp(&Params::default(), &mut reference, &mut sensor, &mut result)?;
    print_result(&result);

    println!("\nTo try different data, change FIRST_SENSOR_POSE or the room/readings above.");
    println!("For the complete automated suite, run: cargo test --all-targets --all-features");

    Ok(())
}
