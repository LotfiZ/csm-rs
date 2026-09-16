//! Correspondence finding and outlier rejection.
//!
//! C: `sm/csm/icp/icp_corr_tricks.c` (jump-table search),
//!     `sm/csm/icp/icp_corr_dumb.c` (naive search),
//!     `sm/csm/icp/icp_outliers.c` (trim + doubles rejection),
//!     `sm/csm/laser_data.c` (`possible_interval`, visibility test)
//!
//! Property-test invariant (grilling Q4): `Tricks` and `Naive` strategies
//! must produce identical correspondence sets — the same invariant C checks
//! with `debug_verify_tricks`.

use crate::laser_data::{CorrespondenceType, LaserData};
use crate::math::{angle_diff, distance_squared, norm};
use crate::params::{CorrespondenceSearch, DistanceMetric, Params};

/// Find correspondences using the currently available search strategy.
///
/// Ticket #5 ports the naive path first. The `Tricks` variant deliberately
/// uses the same implementation until the jump-table path is added in the
/// next ticket; both variants therefore retain the same public behavior.
pub(crate) fn find_correspondences(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &mut LaserData,
) {
    match params.correspondence.search {
        CorrespondenceSearch::Naive | CorrespondenceSearch::Tricks => {
            find_correspondences_naive(params, laser_ref, laser_sens)
        }
    }
}

/// Find the nearest reference ray for every valid sensor ray.
///
/// C: `sm/csm/icp/icp_corr_dumb.c:find_correspondences()`
pub(crate) fn find_correspondences_naive(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &mut LaserData,
) {
    let max_correspondence_dist2 = params.correspondence.max_dist * params.correspondence.max_dist;

    for i in 0..laser_sens.nrays {
        if !laser_sens.valid[i] {
            set_null_correspondence(laser_sens, i);
            continue;
        }

        let p_i_w = laser_sens.points_w[i].p;
        let (from, to, _) = possible_interval(
            p_i_w,
            laser_ref,
            params.correction_limits.max_angular_deg,
            params.correction_limits.max_linear,
        );

        let mut j1 = -1;
        let mut best_dist = 10_000.0;
        for j in from..=to {
            let j_usize = j as usize;
            if !laser_ref.valid[j_usize] {
                continue;
            }
            let dist = distance_squared(p_i_w, laser_ref.points[j_usize].p);
            if dist > max_correspondence_dist2 {
                continue;
            }
            if (j1 == -1 || dist < best_dist)
                && compatible(params, i, j_usize, laser_ref, laser_sens)
            {
                j1 = j;
                best_dist = dist;
            }
        }

        if j1 == -1 || j1 == 0 || j1 == laser_ref.nrays as i32 - 1 {
            set_null_correspondence(laser_sens, i);
            continue;
        }

        let j1_usize = j1 as usize;
        let j2_up = next_valid(laser_ref, j1_usize, 1);
        let j2_down = next_valid(laser_ref, j1_usize, -1);
        let j2 = match (j2_up, j2_down) {
            (None, None) => {
                set_null_correspondence(laser_sens, i);
                continue;
            }
            (Some(j2), None) | (None, Some(j2)) => j2,
            (Some(j2_up), Some(j2_down)) => {
                let dist_up = distance_squared(p_i_w, laser_ref.points[j2_up].p);
                let dist_down = distance_squared(p_i_w, laser_ref.points[j2_down].p);
                if dist_up < dist_down {
                    j2_up
                } else {
                    j2_down
                }
            }
        };

        let corr = &mut laser_sens.corr[i];
        corr.valid = true;
        corr.j1 = j1;
        corr.j2 = j2 as i32;
        corr.dist2_j1 = best_dist;
        corr.corr_type = match params.correspondence.metric {
            DistanceMetric::PointToPoint => CorrespondenceType::PointToPoint,
            DistanceMetric::PointToLine => CorrespondenceType::PointToLine,
        };
    }
}

/// C: `sm/csm/icp/icp_corr_dumb.c:compatible()`
fn compatible(
    params: &Params,
    i: usize,
    j: usize,
    laser_ref: &LaserData,
    laser_sens: &LaserData,
) -> bool {
    if !params.correspondence.do_alpha_test {
        return true;
    }
    if !laser_sens.alpha_valid[i] || !laser_ref.alpha_valid[j] {
        return true;
    }

    let theta = angle_diff(laser_ref.alpha[j], laser_sens.alpha[i]);
    let tolerance = params.correspondence.alpha_test_threshold_deg.to_radians();
    angle_diff(theta, 0.0)
        .abs()
        .partial_cmp(&(tolerance + params.correction_limits.max_angular_deg.to_radians()))
        != Some(std::cmp::Ordering::Greater)
}

/// Restrict a correspondence search to the angular cells reachable under the
/// configured correction limits.
///
/// C: `sm/csm/math_utils.c:possible_interval()`
fn possible_interval(
    point_world: [f64; 2],
    laser_ref: &LaserData,
    max_angular_correction_deg: f64,
    max_linear_correction: f64,
) -> (i32, i32, i32) {
    let angle_res = (laser_ref.max_theta - laser_ref.min_theta) / laser_ref.nrays as f64;
    let delta = max_angular_correction_deg.to_radians().abs()
        + (max_linear_correction / norm(point_world)).atan().abs();
    let range = (delta / angle_res).ceil() as i32;

    let mut start_theta = point_world[1].atan2(point_world[0]);
    if start_theta < laser_ref.min_theta {
        start_theta += 2.0 * std::f64::consts::PI;
    }
    if start_theta > laser_ref.max_theta {
        start_theta -= 2.0 * std::f64::consts::PI;
    }
    let start_cell = ((start_theta - laser_ref.min_theta)
        / (laser_ref.max_theta - laser_ref.min_theta)
        * laser_ref.nrays as f64) as i32;

    let from = (start_cell - range).clamp(0, laser_ref.nrays as i32 - 1);
    let to = (start_cell + range).clamp(0, laser_ref.nrays as i32 - 1);
    (from, to, start_cell)
}

/// C: `ld_next_valid_up()` / `ld_next_valid_down()` in `laser_data_inline.h`
fn next_valid(laser_data: &LaserData, i: usize, direction: i32) -> Option<usize> {
    let mut j = i as i32 + direction;
    while j >= 0 && j < laser_data.nrays as i32 && !laser_data.valid[j as usize] {
        j += direction;
    }
    (j >= 0 && j < laser_data.nrays as i32).then_some(j as usize)
}

/// C: `ld_set_null_correspondence()` in `laser_data_inline.h`
fn set_null_correspondence(laser_sens: &mut LaserData, i: usize) {
    laser_sens.corr[i].valid = false;
    laser_sens.corr[i].j1 = -1;
    laser_sens.corr[i].j2 = -1;
    laser_sens.corr[i].dist2_j1 = f64::NAN;
}
