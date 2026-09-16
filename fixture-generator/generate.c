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

static void build_oscillation_scan(LDP ld) {
    unsigned seed = 104;
    ld->min_theta = MIN_THETA;
    ld->max_theta = MAX_THETA;
    for (int i = 0; i < NRAYS; i++) {
        double t = MIN_THETA + (MAX_THETA - MIN_THETA) * i / (NRAYS - 1);
        seed = seed * 1664525u + 1013904223u;
        ld->theta[i] = t;
        ld->readings[i] = 3.0 + 8.0 * ((seed >> 8) / (double)0x00ffffff);
        ld->valid[i] = 1;
    }
}

static void build_oscillation_case(LDP *ref, LDP *sens) {
    *ref = ld_alloc_new(NRAYS);
    *sens = ld_alloc_new(NRAYS);
    build_oscillation_scan(*ref);
    build_oscillation_scan(*sens);
}

static void build_alpha_rejection_case(LDP *ref, LDP *sens) {
    *ref = ld_alloc_new(NRAYS);
    *sens = ld_alloc_new(NRAYS);
    build_scan(*ref);
    build_scan(*sens);
    for (int i = 0; i < NRAYS; i++) {
        (*sens)->readings[i] += 0.2;
    }
}

static int any_values_present(const double *values, int count) {
    for (int i = 0; i < count; i++) {
        if (!isnan(values[i])) return 1;
    }
    return 0;
}

static void print_double_array(const double *values, int count, int allow_null) {
    for (int i = 0; i < count; i++) {
        if (i) printf(", ");
        if (allow_null && isnan(values[i])) printf("null");
        else printf("%.17g", values[i]);
    }
}

static void print_matrix_json(const gsl_matrix *matrix) {
    printf("[");
    for (size_t row = 0; row < matrix->size1; row++) {
        if (row) printf(", ");
        printf("[");
        for (size_t col = 0; col < matrix->size2; col++) {
            if (col) printf(", ");
            double value = gsl_matrix_get(matrix, row, col);
            if (isnan(value)) printf("null");
            else printf("%.17g", value);
        }
        printf("]");
    }
    printf("]");
}

static void print_scan_json(const char *name, LDP ld) {
    printf("    \"%s\": {\n", name);
    printf("      \"min_theta\": %.17g,\n", ld->min_theta);
    printf("      \"max_theta\": %.17g,\n", ld->max_theta);
    printf("      \"readings\": [");
    print_double_array(ld->readings, ld->nrays, 0);
    printf("]");
    if (any_values_present(ld->readings_sigma, ld->nrays)) {
        printf(",\n      \"readings_sigma\": [");
        print_double_array(ld->readings_sigma, ld->nrays, 1);
        printf("]");
    }
    if (any_values_present(ld->true_alpha, ld->nrays)) {
        printf(",\n      \"true_alpha\": [");
        print_double_array(ld->true_alpha, ld->nrays, 1);
        printf("]");
    }
    printf("\n");
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

struct FeatureFlags {
    int use_corr_tricks;
    int do_alpha_test;
    int do_visibility_test;
    int use_ml_weights;
    int use_sigma_weights;
    int use_point_to_line_distance;
    double max_angular_correction_deg;
    double alpha_test_threshold_deg;
};

struct CaseConfig {
    double first_guess[3];
    int max_iterations;
    int restart;
    double outliers_max_perc;
    double outliers_adaptive_order;
    double outliers_adaptive_mult;
    int outliers_remove_doubles;
    double epsilon_xy;
    double epsilon_theta;
    int do_compute_covariance;
    struct FeatureFlags features;
};

static struct CaseConfig default_case_config(void) {
    return (struct CaseConfig) {
        .first_guess = {0.0, 0.0, 0.0},
        .max_iterations = 1000,
        .restart = 1,
        .outliers_max_perc = 0.95,
        .outliers_adaptive_order = 0.7,
        .outliers_adaptive_mult = 2.0,
        .outliers_remove_doubles = 1,
        .epsilon_xy = 0.0001,
        .epsilon_theta = 0.0001,
        .do_compute_covariance = 0,
        .features = {
            .use_corr_tricks = 1,
            .do_alpha_test = 0,
            .do_visibility_test = 0,
            .use_ml_weights = 0,
            .use_sigma_weights = 0,
            .use_point_to_line_distance = 1,
            .max_angular_correction_deg = 90.0,
            .alpha_test_threshold_deg = 20.0,
        },
    };
}

static void run_case(const char *name, LDP ref, LDP sens,
                     const struct CaseConfig *config) {
    struct sm_params params;
    struct sm_result result;
    struct option *ops = options_allocate(32);

    sm_options(&params, ops); /* defaults straight from the C source */

    params.laser_ref = ref;
    params.laser_sens = sens;
    params.first_guess[0] = config->first_guess[0];
    params.first_guess[1] = config->first_guess[1];
    params.first_guess[2] = config->first_guess[2];
    params.max_iterations = config->max_iterations;
    params.epsilon_xy = config->epsilon_xy;
    params.epsilon_theta = config->epsilon_theta;
    params.restart = config->restart;
    params.outliers_maxPerc = config->outliers_max_perc;
    params.outliers_adaptive_order = config->outliers_adaptive_order;
    params.outliers_adaptive_mult = config->outliers_adaptive_mult;
    params.outliers_remove_doubles = config->outliers_remove_doubles;
    params.use_corr_tricks = config->features.use_corr_tricks;
    params.do_alpha_test = config->features.do_alpha_test;
    params.do_visibility_test = config->features.do_visibility_test;
    params.use_ml_weights = config->features.use_ml_weights;
    params.use_sigma_weights = config->features.use_sigma_weights;
    params.use_point_to_line_distance = config->features.use_point_to_line_distance;
    params.max_angular_correction_deg = config->features.max_angular_correction_deg;
    params.do_alpha_test_thresholdDeg = config->features.alpha_test_threshold_deg;
    params.do_compute_covariance = config->do_compute_covariance;

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
    printf("      \"correspondence_hash\": %u,\n", ld_corr_hash(sens));
    printf("      \"nvalid\": %d", result.nvalid);
    if (result.valid && config->do_compute_covariance) {
        printf(",\n      \"cov_x\": ");
        print_matrix_json(result.cov_x_m);
        printf(",\n      \"dx_dy1\": ");
        print_matrix_json(result.dx_dy1_m);
        printf(",\n      \"dx_dy2\": ");
        print_matrix_json(result.dx_dy2_m);
    }
    printf("\n");
    printf("    }\n");
    printf("  }\n");
}

static void set_ml_fields(LDP ref) {
    for (int i = 0; i < NRAYS; i++) {
        /* A deterministic, slightly varying surface normal for the ML
         * incidence factor. */
        ref->true_alpha[i] = ref->theta[i] + 0.2 + 0.01 * sin(ref->theta[i]);
    }
}

static void set_sigma_fields(LDP sens, int leave_gaps) {
    for (int i = 0; i < NRAYS; i++) {
        /* C skips a ray whose sigma is NaN; retain a few such entries to keep
         * the fixture schema honest about per-ray optional data. */
        sens->readings_sigma[i] =
            leave_gaps && i % 11 == 0 ? NAN : 0.01 + 0.0001 * (i % 7);
    }
}

int main(void) {
    struct CaseConfig config = default_case_config();

    LDP ref  = ld_alloc_new(NRAYS);
    LDP sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens); /* identity case: identical scans */

    printf("{\n");
    printf("  \"schema\": \"csm-rs-fixture/v1\",\n");
    printf("  \"cases\": [\n");
    run_case("identity", ref, sens, &config);

    /* A deliberately offset, single-iteration match exercises Censi's exact
     * closed-form covariance and both range-derivative matrices. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    config = default_case_config();
    config.first_guess[0] = 0.25;
    config.first_guess[1] = -0.15;
    config.first_guess[2] = 0.04;
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.do_compute_covariance = 1;
    printf(",\n");
    run_case("covariance", ref, sens, &config);

    /* Percentile-only trim: the twenty perturbed rays have a positive
     * point-to-line error, while the adaptive limit is disabled at order 1. */
    build_trim_case(&ref, &sens);
    config = default_case_config();
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 0.5;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    printf(",\n");
    run_case("percentile_trim", ref, sens, &config);

    /* Adaptive-only trim: maxPerc is disabled at order 1, leaving the
     * order-0.7, multiplier-2 threshold to reject the same outliers. */
    build_trim_case(&ref, &sens);
    config = default_case_config();
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_remove_doubles = 0;
    printf(",\n");
    run_case("adaptive_trim", ref, sens, &config);

    /* A translated first guess makes several sensor rays choose the same
     * reference ray. The fixed three-times-distance duplicate rule removes
     * the farther copies before the trim pass. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    config = default_case_config();
    config.first_guess[0] = 1.0;
    config.max_iterations = 1;
    config.restart = 0;
    printf(",\n");
    run_case("duplicate_correspondence", ref, sens, &config);
    config.restart = 1;
    printf(",\n");
    run_case("restart_probe", ref, sens, &config);

    build_oscillation_case(&ref, &sens);
    config = default_case_config();
    config.first_guess[1] = 1.0;
    config.max_iterations = 100;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.epsilon_xy = 0.0;
    config.epsilon_theta = 0.0;
    printf(",\n");
    /* Zero thresholds make the repeated hash, rather than convergence,
     * responsible for the early exit. */
    run_case("oscillation", ref, sens, &config);

    /* C's default smart path with the optional orientation and visibility
     * gates enabled. The C tricks routine intentionally omits alpha filtering,
     * so this case verifies the default dispatch and the visibility shell. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    static const struct FeatureFlags alpha_tricks_features = {
        .use_corr_tricks = 1,
        .do_alpha_test = 1,
        .do_visibility_test = 1,
        .use_ml_weights = 0,
        .use_sigma_weights = 0,
        .use_point_to_line_distance = 1,
        .max_angular_correction_deg = 90.0,
        .alpha_test_threshold_deg = 20.0,
    };
    config = default_case_config();
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.features = alpha_tricks_features;
    printf(",\n");
    run_case("alpha_visibility_tricks", ref, sens, &config);

    /* The naive alpha path with a tight angular gate and a slightly changed
     * scan exercises actual orientation rejection. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_alpha_rejection_case(&ref, &sens);
    static const struct FeatureFlags alpha_rejection_features = {
        .use_corr_tricks = 0,
        .do_alpha_test = 1,
        .do_visibility_test = 0,
        .use_ml_weights = 0,
        .use_sigma_weights = 0,
        .use_point_to_line_distance = 1,
        .max_angular_correction_deg = 0.0,
        .alpha_test_threshold_deg = 0.0,
    };
    config = default_case_config();
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.features = alpha_rejection_features;
    printf(",\n");
    run_case("alpha_rejection", ref, sens, &config);

    /* ML weights consume true_alpha on the reference scan. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    set_ml_fields(ref);
    static const struct FeatureFlags ml_features = {
        .use_corr_tricks = 0,
        .do_alpha_test = 0,
        .do_visibility_test = 0,
        .use_ml_weights = 1,
        .use_sigma_weights = 0,
        .use_point_to_line_distance = 1,
        .max_angular_correction_deg = 90.0,
        .alpha_test_threshold_deg = 20.0,
    };
    config = default_case_config();
    config.first_guess[0] = 0.25;
    config.first_guess[1] = -0.15;
    config.first_guess[2] = 0.04;
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.features = ml_features;
    printf(",\n");
    run_case("ml_weights", ref, sens, &config);

    /* Sigma weights consume readings_sigma on the sensor scan. Include NaN
     * gaps to exercise C's per-ray fallback. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    set_sigma_fields(sens, 1);
    static const struct FeatureFlags sigma_features = {
        .use_corr_tricks = 0,
        .do_alpha_test = 0,
        .do_visibility_test = 0,
        .use_ml_weights = 0,
        .use_sigma_weights = 1,
        .use_point_to_line_distance = 1,
        .max_angular_correction_deg = 90.0,
        .alpha_test_threshold_deg = 20.0,
    };
    config = default_case_config();
    config.first_guess[0] = 0.25;
    config.first_guess[1] = -0.15;
    config.first_guess[2] = 0.04;
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.features = sigma_features;
    printf(",\n");
    run_case("sigma_weights", ref, sens, &config);

    /* With no true_alpha values, ML falls back to the orientation estimated
     * by the alpha pass. */
    ref = ld_alloc_new(NRAYS);
    sens = ld_alloc_new(NRAYS);
    build_scan(ref);
    build_scan(sens);
    static const struct FeatureFlags computed_alpha_features = {
        .use_corr_tricks = 0,
        .do_alpha_test = 1,
        .do_visibility_test = 0,
        .use_ml_weights = 1,
        .use_sigma_weights = 0,
        .use_point_to_line_distance = 1,
        .max_angular_correction_deg = 90.0,
        .alpha_test_threshold_deg = 20.0,
    };
    config = default_case_config();
    config.first_guess[0] = 0.25;
    config.first_guess[1] = -0.15;
    config.first_guess[2] = 0.04;
    config.max_iterations = 1;
    config.restart = 0;
    config.outliers_max_perc = 1.0;
    config.outliers_adaptive_order = 1.0;
    config.outliers_remove_doubles = 0;
    config.features = computed_alpha_features;
    printf(",\n");
    run_case("computed_alpha_weights", ref, sens, &config);
    printf("  ]\n");
    printf("}\n");
    return 0;
}
