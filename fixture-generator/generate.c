/*
 * csm-rs fixture generator — THROWAWAY tool, never built by cargo.
 *
 * Links the original CSM C library (AndreaCensi/csm, master/GSL flavor),
 * runs scan pairs through sm_icp, and prints JSON fixtures in csm-rs's own
 * schema (scans + params + expected result). The JSON output is the
 * permanent artifact; this program is not.
 *
 * Build: see build.sh (no root required; GSL/json-c come from apt .debs
 * extracted locally).
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <csm/csm_all.h>
#include <options/options.h>

/* No public header declares this; the C apps declare it extern too. */
extern void sm_options(struct sm_params *p, struct option *ops);

/* sm_options.c references HSM's option hook; HSM is out of scope for the
 * port (grilling Q1), so stub it — it only registers unused options. */
void hsm_add_options(struct option *ops, struct hsm_params *p) {
    (void)ops; (void)p;
}

#define NRAYS 360
#define MIN_THETA (-M_PI)
#define MAX_THETA (M_PI)

/* Deterministic synthetic world: square room, half-size 5 m, robot at
 * origin. Reading along (cos t, sin t) is the distance to the nearest wall. */
static double square_room_reading(double t) {
    double c = fabs(cos(t));
    double s = fabs(sin(t));
    double dx = (c > 1e-12) ? 5.0 / c : 1e12;
    double dy = (s > 1e-12) ? 5.0 / s : 1e12;
    return (dx < dy) ? dx : dy;
}

static void build_scan(LDP ld) {
    int i;
    ld->min_theta = MIN_THETA;
    ld->max_theta = MAX_THETA;
    for (i = 0; i < ld->nrays; i++) {
        double t = MIN_THETA + (MAX_THETA - MIN_THETA) * i / (ld->nrays - 1);
        ld->theta[i] = t;
        ld->readings[i] = square_room_reading(t);
        ld->valid[i] = 1;
    }
}

static void build_trim_case(LDP *ref, LDP *sens) {
    *ref = ld_alloc_new(NRAYS);
    *sens = ld_alloc_new(NRAYS);
    build_scan(*ref);
    build_scan(*sens);
    for (int i = 20; i < 40; i++) (*sens)->readings[i] += 1.0;
}

static void print_scan_json(const char *name, LDP ld) {
    int i;
    printf("    \"%s\": {\n", name);
    printf("      \"min_theta\": %.17g,\n", ld->min_theta);
    printf("      \"max_theta\": %.17g,\n", ld->max_theta);
    printf("      \"readings\": [");
    for (i = 0; i < ld->nrays; i++)
        printf("%s%.17g", i ? ", " : "", ld->readings[i]);
    printf("]\n");
    printf("    }");
}

/* Print the params sm_icp actually consumes, in C field names. The Rust
 * side maps these onto its idiomatic Params (the mapping is part of the
 * test). */
static void print_params_json(struct sm_params *p) {
    printf("    \"params\": {\n");
    printf("      \"first_guess\": [%.17g, %.17g, %.17g],\n",
        p->first_guess[0], p->first_guess[1], p->first_guess[2]);
    printf("      \"max_angular_correction_deg\": %.17g,\n", p->max_angular_correction_deg);
    printf("      \"max_linear_correction\": %.17g,\n", p->max_linear_correction);
    printf("      \"max_iterations\": %d,\n", p->max_iterations);
    printf("      \"epsilon_xy\": %.17g,\n", p->epsilon_xy);
    printf("      \"epsilon_theta\": %.17g,\n", p->epsilon_theta);
    printf("      \"max_correspondence_dist\": %.17g,\n", p->max_correspondence_dist);
    printf("      \"sigma\": %.17g,\n", p->sigma);
    printf("      \"use_corr_tricks\": %d,\n", p->use_corr_tricks);
    printf("      \"restart\": %d,\n", p->restart);
    printf("      \"restart_threshold_mean_error\": %.17g,\n", p->restart_threshold_mean_error);
    printf("      \"restart_dt\": %.17g,\n", p->restart_dt);
    printf("      \"restart_dtheta\": %.17g,\n", p->restart_dtheta);
    printf("      \"clustering_threshold\": %.17g,\n", p->clustering_threshold);
    printf("      \"orientation_neighbourhood\": %d,\n", p->orientation_neighbourhood);
    printf("      \"use_point_to_line_distance\": %d,\n", p->use_point_to_line_distance);
    printf("      \"do_alpha_test\": %d,\n", p->do_alpha_test);
    printf("      \"do_alpha_test_thresholdDeg\": %.17g,\n", p->do_alpha_test_thresholdDeg);
    printf("      \"outliers_maxPerc\": %.17g,\n", p->outliers_maxPerc);
    printf("      \"outliers_adaptive_order\": %.17g,\n", p->outliers_adaptive_order);
    printf("      \"outliers_adaptive_mult\": %.17g,\n", p->outliers_adaptive_mult);
    printf("      \"do_visibility_test\": %d,\n", p->do_visibility_test);
    printf("      \"outliers_remove_doubles\": %d,\n", p->outliers_remove_doubles);
    printf("      \"do_compute_covariance\": %d,\n", p->do_compute_covariance);
    printf("      \"min_reading\": %.17g,\n", p->min_reading);
    printf("      \"max_reading\": %.17g,\n", p->max_reading);
    printf("      \"use_ml_weights\": %d,\n", p->use_ml_weights);
    printf("      \"use_sigma_weights\": %d\n", p->use_sigma_weights);
    printf("    },\n");
}

static void run_case(const char *name, const double first_guess[3],
                     LDP ref, LDP sens, int max_iterations, int restart,
                     double outliers_max_perc, double outliers_adaptive_order,
                     double outliers_adaptive_mult, int outliers_remove_doubles) {
    struct sm_params params;
    struct sm_result result;
    struct option *ops = options_allocate(32);

    sm_options(&params, ops); /* defaults straight from the C source */

    params.laser_ref = ref;
    params.laser_sens = sens;
    params.first_guess[0] = first_guess[0];
    params.first_guess[1] = first_guess[1];
    params.first_guess[2] = first_guess[2];
    params.max_iterations = max_iterations;
    params.restart = restart;
    params.outliers_maxPerc = outliers_max_perc;
    params.outliers_adaptive_order = outliers_adaptive_order;
    params.outliers_adaptive_mult = outliers_adaptive_mult;
    params.outliers_remove_doubles = outliers_remove_doubles;

    sm_icp(&params, &result);

    printf("  {\n");
    printf("    \"name\": \"%s\",\n", name);
    print_params_json(&params);
    print_scan_json("laser_ref", ref);
    printf(",\n");
    print_scan_json("laser_sens", sens);
    printf(",\n");
    printf("    \"expected\": {\n");
    printf("      \"valid\": %s,\n", result.valid ? "true" : "false");
    printf("      \"x\": [%.17g, %.17g, %.17g],\n",
        result.x[0], result.x[1], result.x[2]);
    printf("      \"error\": %.17g,\n", result.error);
    printf("      \"iterations\": %d,\n", result.iterations);
    printf("      \"nvalid\": %d\n", result.nvalid);
    printf("    }\n");
    printf("  }\n");
}

int main(void) {
    static const double zero_guess[3] = {0.0, 0.0, 0.0};
    static const double translated_guess[3] = {1.0, 0.0, 0.0};

    LDP ref  = ld_alloc_new(NRAYS);
    LDP sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens); /* identity case: identical scans */

    printf("{\n");
    printf("  \"schema\": \"csm-rs-fixture/v1\",\n");
    printf("  \"cases\": [\n");
    run_case("identity", zero_guess, ref, sens, 1000, 1, 0.95, 0.7, 2.0, 1);

    /* Percentile-only trim: the twenty perturbed rays have a positive
     * point-to-line error, while the adaptive limit is disabled at order 1. */
    build_trim_case(&ref, &sens);
    printf(",\n");
    run_case("percentile_trim", zero_guess, ref, sens, 1, 0, 0.5, 1.0, 2.0, 0);

    /* Adaptive-only trim: maxPerc is disabled at order 1, leaving the
     * order-0.7, multiplier-2 threshold to reject the same outliers. */
    build_trim_case(&ref, &sens);
    printf(",\n");
    run_case("adaptive_trim", zero_guess, ref, sens, 1, 0, 1.0, 0.7, 2.0, 0);

    /* A translated first guess makes several sensor rays choose the same
     * reference ray. The fixed three-times-distance duplicate rule removes
     * the farther copies before the trim pass. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    printf(",\n");
    run_case("duplicate_correspondence", translated_guess, ref, sens,
        1, 0, 0.95, 0.7, 2.0, 1);
    printf("  ]\n");
    printf("}\n");
    return 0;
}
