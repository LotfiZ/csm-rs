#!/usr/bin/env bash
# Build and run the THROWAWAY fixture generator against the original CSM
# C library. No root required: GSL comes from apt .debs extracted locally;
# CSM's vendored json-c is compiled with a minimal generated config.h.
#
# Usage: ./build.sh <path-to-csm-source> [output.json]
# Example: ./build.sh ~/workspace/csm-src ../tests/fixtures/identity.json
set -euo pipefail

CSM_SRC="${1:?usage: build.sh <csm-source-dir> [output.json]}"
OUT="${2:-/dev/stdout}"
WORK="$(cd "$(dirname "$0")" && pwd)/build"
DEPS="$WORK/deps"
OBJS="$WORK/obj"
PREFIX="$DEPS/local"

mkdir -p "$DEPS" "$OBJS"

# --- 1. GSL from apt .debs, extracted locally (no root) -------------------
if [ ! -f "$PREFIX/usr/include/gsl/gsl_matrix.h" ]; then
    (cd "$DEPS" && apt-get download libgsl-dev libgsl23 libgslcblas0)
    for d in "$DEPS"/*.deb; do dpkg-deb -x "$d" "$PREFIX"; done
fi
GSL="$PREFIX/usr"
LIBDIR="$GSL/lib/aarch64-linux-gnu"

# --- 2. Minimal config.h for CSM's vendored json-c 0.9 --------------------
cat > "$OBJS/config.h" <<'EOF'
#define HAVE_STDLIB_H 1
#define HAVE_STRING_H 1
#define HAVE_STRINGS_H 1
#define HAVE_UNISTD_H 1
#define HAVE_STDINT_H 1
#define HAVE_INTTYPES_H 1
#define HAVE_STDDEF_H 1
#define HAVE_STRCASECMP 1
#define HAVE_STRNCASECMP 1
#define HAVE_SNPRINTF 1
#define HAVE_VSNPRINTF 1
#define HAVE_DECL_SNPRINTF 1
#define HAVE_DECL_VSNPRINTF 1
#define HAVE_DECL_INFINITY 1
#define HAVE_DECL_NAN 1
#define HAVE_DECL_ISNAN 1
#define HAVE_DECL_ISINF 1
#define HAVE_OPEN 1
#define HAVE_REALLOC 1
#define STDC_HEADERS 1
#define HAVE_FCNTL_H 1
#define HAVE_STDARG_H 1
#define HAVE_VASPRINTF 1
EOF

# --- 3. Compile the sm_icp-path subset of CSM ------------------------------
CFLAGS="-O2 -w -I$OBJS -I$CSM_SRC/sm -I$CSM_SRC/sm/lib -I$GSL/include"
SOURCES="
csm/clustering.c csm/orientation.c csm/utils.c csm/logging.c
csm/laser_data.c csm/laser_data_bbox.c csm/laser_data_fisher.c
csm/laser_data_json.c csm/json_journal.c csm/math_utils.c csm/math_utils_gsl.c
csm/icp/icp.c csm/icp/icp_loop.c csm/icp/icp_outliers.c
csm/icp/icp_corr_dumb.c csm/icp/icp_corr_tricks.c
csm/icp/icp_covariance.c csm/icp/icp_debug.c
csm/sm_options.c
lib/egsl/egsl.c lib/egsl/egsl_misc.c lib/egsl/egsl_ops.c lib/egsl/egsl_conversions.c
lib/gpc/gpc.c lib/gpc/gpc_utils.c
lib/options/options.c lib/options/options_interface.c
lib/json-c/arraylist.c lib/json-c/debug.c lib/json-c/json_object.c
lib/json-c/json_tokener.c lib/json-c/json_util.c lib/json-c/linkhash.c
lib/json-c/printbuf.c lib/json-c/JSON_checker.c lib/json-c/json_more_utils.c
"
for f in $SOURCES; do
    obj="$OBJS/$(echo "$f" | tr '/' '_').o"
    [ -f "$obj" ] || gcc $CFLAGS -c "$CSM_SRC/sm/$f" -o "$obj"
done

# --- 4. Build and run the generator ----------------------------------------
gcc $CFLAGS "$(dirname "$0")/generate.c" "$OBJS"/*.o \
    -L"$LIBDIR" -lgsl -lgslcblas -lm -o "$WORK/generate"

LD_LIBRARY_PATH="$LIBDIR" "$WORK/generate" "$CSM_SRC" > "$OUT"
echo "fixture written to $OUT" >&2
