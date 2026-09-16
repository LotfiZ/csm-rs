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
//!
//! The upstream tricks routine does not call `compatible()` for its optional
//! alpha filter. That C asymmetry is preserved here; the equivalence seam
//! therefore applies to the default alpha-disabled search and will be
//! revisited with the later alpha-feature ticket.

use crate::laser_data::{CorrespondenceType, LaserData};
use crate::math::{angle_diff, corr_hash, distance_squared, norm};
use crate::params::{CorrespondenceSearch, DistanceMetric, Params};

/// Find correspondences using the currently available search strategy.
///
/// C: `icp_loop.c` dispatches to `find_correspondences_tricks()` or
/// `find_correspondences()` based on `use_corr_tricks`.
pub(crate) fn find_correspondences(
    params: &Params,
    laser_ref: &LaserData,
    laser_sens: &mut LaserData,
) {
    match params.correspondence.search {
        CorrespondenceSearch::Naive => find_correspondences_naive(params, laser_ref, laser_sens),
        CorrespondenceSearch::Tricks => find_correspondences_tricks(params, laser_ref, laser_sens),
    }

    // C: `debug_correspondences()` in `icp_debug.c`. Run both strategies on
    // identical inputs and compare the observable correspondence key and its
    // hash when the caller requests the CSM debug check.
    if params.debug_verify_tricks {
        let mut tricks = laser_sens.clone();
        let mut naive = laser_sens.clone();
        find_correspondences_tricks(params, laser_ref, &mut tricks);
        find_correspondences_naive(params, laser_ref, &mut naive);
        let tricks_keys = correspondence_keys(&tricks);
        let naive_keys = correspondence_keys(&naive);
        assert_eq!(
            corr_hash(&tricks_keys),
            corr_hash(&naive_keys),
            "tricks and naive correspondence hashes differ"
        );
        assert_eq!(
            tricks_keys, naive_keys,
            "tricks and naive correspondence sets differ"
        );
    }
}

/// Find the nearest reference ray with the jump-table search.
///
/// C: `sm/csm/icp/icp_corr_tricks.c:find_correspondences_tricks()`
/// The C routine intentionally does not apply `compatible()`; preserve that
/// behavior until the alpha-feature ticket extends both strategies together.
fn find_correspondences_tricks(params: &Params, laser_ref: &LaserData, laser_sens: &mut LaserData) {
    let c1 = laser_ref.nrays as f64 / (laser_ref.max_theta - laser_ref.min_theta);
    let max_correspondence_dist2 = params.correspondence.max_dist * params.correspondence.max_dist;
    let mut last_best = -1;

    for i in 0..laser_sens.nrays {
        if !laser_sens.valid[i] {
            set_null_correspondence(laser_sens, i);
            continue;
        }

        let p_i_w = laser_sens.points_w[i].p;
        let p_i_w_norm = laser_sens.points_w[i].rho;
        let p_i_w_phi = laser_sens.points_w[i].phi;

        let from = 0;
        let to = laser_ref.nrays as i32 - 1;
        let start_cell = ((p_i_w_phi - laser_ref.min_theta) * c1) as i32;

        let mut j1 = -1;
        let mut best_dist = 42.0;

        let mut start = if last_best == -1 {
            start_cell
        } else {
            last_best + 1
        };
        start = start.clamp(from, to);

        let mut up = start + 1;
        let mut down = start;
        let mut last_dist_up = 0.0;
        let mut last_dist_down = -1.0;
        let mut up_stopped = false;
        let mut down_stopped = false;

        while !up_stopped || !down_stopped {
            // C chooses the side whose last distance is smaller. The
            // initial values make the first visit the `down` side.
            let now_up = if up_stopped {
                false
            } else if down_stopped {
                true
            } else {
                last_dist_up < last_dist_down
            };

            if now_up {
                if up > to {
                    up_stopped = true;
                    continue;
                }
                if !laser_ref.valid[up as usize] {
                    up += 1;
                    continue;
                }

                last_dist_up = distance_squared(p_i_w, laser_ref.points[up as usize].p);
                if last_dist_up < best_dist || j1 == -1 {
                    j1 = up;
                    best_dist = last_dist_up;
                }

                if up > start_cell {
                    let delta_theta = laser_ref.theta[up as usize] - p_i_w_phi;
                    let min_dist_up = p_i_w_norm
                        * if delta_theta > std::f64::consts::FRAC_PI_2 {
                            1.0
                        } else {
                            mysin(delta_theta)
                        };
                    if min_dist_up * min_dist_up > best_dist {
                        up_stopped = true;
                        continue;
                    }

                    up += if laser_ref.readings[up as usize] < p_i_w_norm {
                        laser_ref.up_bigger[up as usize]
                    } else {
                        laser_ref.up_smaller[up as usize]
                    };
                } else {
                    up += 1;
                }
            }

            if !now_up {
                if down < from {
                    down_stopped = true;
                    continue;
                }
                if !laser_ref.valid[down as usize] {
                    down -= 1;
                    continue;
                }

                last_dist_down = distance_squared(p_i_w, laser_ref.points[down as usize].p);
                if last_dist_down < best_dist || j1 == -1 {
                    j1 = down;
                    best_dist = last_dist_down;
                }

                if down < start_cell {
                    let delta_theta = p_i_w_phi - laser_ref.theta[down as usize];
                    let min_dist_down = p_i_w_norm
                        * if delta_theta > std::f64::consts::FRAC_PI_2 {
                            1.0
                        } else {
                            mysin(delta_theta)
                        };
                    if min_dist_down * min_dist_down > best_dist {
                        down_stopped = true;
                        continue;
                    }

                    down += if laser_ref.readings[down as usize] < p_i_w_norm {
                        laser_ref.down_bigger[down as usize]
                    } else {
                        laser_ref.down_smaller[down as usize]
                    };
                } else {
                    down -= 1;
                }
            }
        }

        if j1 == -1 || best_dist > max_correspondence_dist2 {
            set_null_correspondence(laser_sens, i);
            continue;
        }
        if j1 == 0 || j1 == laser_ref.nrays as i32 - 1 {
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
            (Some(j2), None) => j2,
            (None, Some(j2)) => j2,
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

        // C advances this state only after a complete correspondence has
        // been accepted, so failed and endpoint matches do not affect the
        // next sensor ray's starting cell.
        last_best = j1;

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

/// Approximation used by the C smart search's angular lower bound.
///
/// C: `mysin()` in `sm/csm/icp/icp_corr_tricks.c`; the approximation is
/// deliberately retained instead of replacing it with `f64::sin()`.
fn mysin(x: f64) -> f64 {
    let a = -1.0 / 6.0;
    let b = 1.0 / 120.0;
    let x2 = x * x;
    x * (0.99 + x2 * (a + b * x2))
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

fn correspondence_keys(laser_sens: &LaserData) -> Vec<Option<(i32, i32)>> {
    laser_sens
        .corr
        .iter()
        .map(|corr| corr.valid.then_some((corr.j1, corr.j2)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::corr_hash;

    fn smooth_scan(valid: impl Fn(usize) -> bool) -> LaserData {
        let mut scan = LaserData::new(360, -1.5, 1.5);
        for i in 0..scan.nrays {
            if valid(i) {
                let theta = scan.theta[i];
                scan.valid[i] = true;
                scan.readings[i] = 8.0
                    + 0.8 * (2.0 * theta).sin()
                    + 0.35 * (5.0 * theta).cos()
                    + 0.15 * (11.0 * theta).sin();
            }
        }
        scan
    }

    fn assert_tricks_matches_naive(
        mut laser_ref: LaserData,
        mut laser_sens: LaserData,
        pose: [f64; 3],
    ) {
        let mut naive_params = Params::default();
        naive_params.correspondence.search = CorrespondenceSearch::Naive;
        let mut tricks_params = Params::default();
        tricks_params.correspondence.search = CorrespondenceSearch::Tricks;
        tricks_params.debug_verify_tricks = true;
        laser_ref.compute_cartesian();
        laser_ref.create_jump_tables();
        laser_sens.compute_cartesian();

        let mut naive = laser_sens.clone();
        naive.compute_world_coords(&pose);
        // Both calls below are first correspondence passes at the same fixed
        // pose; no ICP iteration can overwrite either result.
        find_correspondences(&naive_params, &laser_ref, &mut naive);

        let mut tricks = laser_sens;
        tricks.compute_world_coords(&pose);
        find_correspondences(&tricks_params, &laser_ref, &mut tricks);

        let tricks_entries = correspondence_keys(&tricks);
        let naive_entries = correspondence_keys(&naive);
        if tricks_entries != naive_entries {
            for (i, (tricks_entry, naive_entry)) in
                tricks_entries.iter().zip(&naive_entries).enumerate()
            {
                if tricks_entry != naive_entry {
                    panic!("ray {i}: tricks {tricks_entry:?}, naive {naive_entry:?}");
                }
            }
        }
        assert_eq!(corr_hash(&tricks_entries), corr_hash(&naive_entries));
        for (tricks_corr, naive_corr) in tricks.corr.iter().zip(&naive.corr) {
            if tricks_corr.valid {
                assert_eq!(tricks_corr.j1, naive_corr.j1);
                assert_eq!(tricks_corr.j2, naive_corr.j2);
                assert_eq!(tricks_corr.dist2_j1, naive_corr.dist2_j1);
            } else {
                assert!(!naive_corr.valid);
            }
        }
    }

    #[test]
    fn tricks_and_naive_match_on_smooth_scan() {
        assert_tricks_matches_naive(
            smooth_scan(|_| true),
            smooth_scan(|_| true),
            [0.12, -0.07, 0.015],
        );
    }

    #[test]
    fn tricks_and_naive_match_across_invalid_rays() {
        assert_tricks_matches_naive(
            smooth_scan(|i| i % 29 != 0),
            smooth_scan(|i| i % 37 != 0),
            [0.04, -0.03, 0.008],
        );
    }
}
