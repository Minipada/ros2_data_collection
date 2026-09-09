# shellcheck shell=bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Shared skeleton of the twelve e2e scenario scripts (#496): obtaining the DC image, the
# cleanup trap, destination bring-up and readiness waits, volume extraction, teardown.
# Sourced, never executed — set -euo pipefail comes from the caller.
#
# The scenario describes itself once, to harness_init, and everything else is a function
# call — the harness reads nothing else out of the sourcing script, so a new scenario is
# a manifest plus a verify invocation, with no pasted teardown:
#
#   source "$SCRIPT_DIR/lib/harness.sh"
#   harness_init --tag e2e --run-dir "$RUN_DIR" --network host \
#     --containers "$DC_C" "$PG_C" "$RUSTFS_C" \
#     --volumes "${VOLUMES[*]}" \
#     --log-captures "$DC_C:dc.log" \
#     --pg-container "$PG_C" --rustfs-container "$RUSTFS_C"
#
#   --containers/--volumes/--teardown-networks/--log-captures  what remove_stack tears
#       down and what harness_capture_logs saves on the way out ("container:file" pairs)
#   --compose    a compose file to `down --volumes --remove-orphans` instead of removing
#       containers one by one (its services and volumes *are* the manifest)
#   --network    where destinations, probes and the bucket CLI run (default host);
#       --teardown-networks lists the ones remove_stack deletes, which is the same network
#       for every scenario that creates one and nothing for --network host
#   --pg-container/--rust-container  opt into pg_exec/wait_postgres_ready/wait_rustfs_ready
#
# Outputs, not inputs: DC_IMAGE (resolve_image) and FIRST_RECORD_LATENCY
# (wait_first_record). run_limits_drain_rate.sh overrides harness_rustfs_probe — the one
# hook, because it has no DC image to probe with.

HARNESS_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HARNESS_E2E_DIR="$(dirname "$HARNESS_SCRIPT_DIR")"

# vector_version() lives in its own lib file (#484) so the limits/load and release
# scripts can source it without pulling in this e2e-only skeleton.
# shellcheck disable=SC1091
source "$HARNESS_SCRIPT_DIR/lib/version.sh"

# rustfs/rustfs 1.0.0-beta.11 — the one bash copy of the digest. Bump deliberately: check
# https://hub.docker.com/r/rustfs/rustfs/tags for the new digest. compose.split.yaml and
# compose.test.yaml carry their own literal (plain YAML can't source this, the same
# reason compose.split.yaml's Vector tag keeps a literal default — see the vector-pin job).
RUSTFS_IMAGE="docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c"

log() { echo "[${HARNESS_TAG:-e2e} $(date -u +%H:%M:%S)] $*"; }

# --- the manifest ---------------------------------------------------------------------

HARNESS_STATS_PID=""

# harness_init <manifest>: record the scenario's shape, create its run dir, install the
# EXIT trap. List flags consume every following argument that isn't another flag.
harness_init() {
  HARNESS_TAG="e2e"
  HARNESS_NETWORK="host"
  HARNESS_RUN_DIR=""
  HARNESS_COMPOSE_FILE=""
  HARNESS_PG_C=""
  HARNESS_RUSTFS_C=""
  HARNESS_CONTAINERS=()
  HARNESS_VOLUMES=()
  HARNESS_TEARDOWN_NETWORKS=()
  HARNESS_LOG_CAPTURES=()

  while [ "$#" -gt 0 ]; do
    case "$1" in
      --tag)
        [ "$#" -ge 2 ] || { log "harness_init: --tag needs a value"; return 1; }
        HARNESS_TAG="$2"
        shift 2
        ;;
      --run-dir|--network|--compose|--pg-container|--rustfs-container)
        [ "$#" -ge 2 ] || { log "harness_init: $1 needs a value"; return 1; }
        case "$1" in
          --run-dir) HARNESS_RUN_DIR="$2" ;;
          --network) HARNESS_NETWORK="$2" ;;
          --compose) HARNESS_COMPOSE_FILE="$2" ;;
          --pg-container) HARNESS_PG_C="$2" ;;
          --rustfs-container) HARNESS_RUSTFS_C="$2" ;;
        esac
        shift 2
        ;;
      --containers|--volumes|--teardown-networks|--log-captures)
        local flag="$1" list=()
        shift
        while [ "$#" -gt 0 ] && [ "${1#--}" = "$1" ]; do
          list+=("$1")
          shift
        done
        case "$flag" in
          --containers) HARNESS_CONTAINERS=("${list[@]}") ;;
          --volumes) HARNESS_VOLUMES=("${list[@]}") ;;
          --teardown-networks) HARNESS_TEARDOWN_NETWORKS=("${list[@]}") ;;
          --log-captures) HARNESS_LOG_CAPTURES=("${list[@]}") ;;
        esac
        ;;
      *)
        log "harness_init: unknown flag $1"
        return 1
        ;;
    esac
  done

  [ -n "$HARNESS_RUN_DIR" ] || { log "harness_init: --run-dir is required"; return 1; }
  mkdir -p "$HARNESS_RUN_DIR"
  trap harness_cleanup EXIT
}

# The resource sampler's PID (measure_resources.sh) — unknown until the sampler starts,
# so set after harness_init. harness_cleanup kills it only if it is still running.
harness_stats_pid() { HARNESS_STATS_PID="$1"; }

# --- teardown -------------------------------------------------------------------------

# Building blocks remove_stack is made of, exposed for a scenario that tears a phase down
# mid-run (run_limits_shipper_fanin.sh) with the same teardown-safe semantics: a name that
# doesn't exist is not an error, a real failure still is.
harness_rm_containers() { [ "$#" -eq 0 ] || podman rm -f --ignore "$@" >/dev/null; }

harness_rm_volumes() {
  local v
  for v in "$@"; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
}

harness_rm_networks() {
  local n
  for n in "$@"; do
    podman network rm "$n" >/dev/null 2>&1 || true
  done
}

# On a failed run, the reload/error tail of each captured container goes to stdout too:
# CI's artifact upload can't see the run dir (a hidden directory), so the job log is the
# only copy of the failure's evidence that survives.
harness_failure_excerpts() {
  local pair container
  for pair in "${HARNESS_LOG_CAPTURES[@]}"; do
    container="${pair%%:*}"
    if podman container exists "$container"; then
      log "--- $container: reload/error tail"
      podman logs "$container" 2>&1 | grep -iE "reload|error|fatal" | tail -30 || true
    fi
  done
}

# Podman logs for every --log-captures pair, into the manifest's run dir.
harness_capture_logs() {
  local pair container
  for pair in "${HARNESS_LOG_CAPTURES[@]}"; do
    container="${pair%%:*}"
    if podman container exists "$container"; then
      podman logs "$container" > "$HARNESS_RUN_DIR/${pair##*:}" 2>&1 || true
    fi
  done
}

remove_stack() {
  if [ -n "$HARNESS_COMPOSE_FILE" ]; then
    podman compose -f "$HARNESS_COMPOSE_FILE" down --volumes --remove-orphans >/dev/null 2>&1 || true
  fi
  harness_rm_containers "${HARNESS_CONTAINERS[@]}"
  harness_rm_volumes "${HARNESS_VOLUMES[@]}"
  harness_rm_networks "${HARNESS_TEARDOWN_NETWORKS[@]}"
}

# The EXIT trap harness_init installs: kill a running resource sampler, honour DC_E2E_KEEP,
# capture the --log-captures logs, then remove the manifest's stack.
harness_cleanup() {
  local exit_code=$?
  if [ -n "$HARNESS_STATS_PID" ] && kill -0 "$HARNESS_STATS_PID" 2>/dev/null; then
    kill "$HARNESS_STATS_PID"
  fi
  if [ "$exit_code" -ne 0 ]; then
    harness_failure_excerpts
  fi
  if [ "$exit_code" -ne 0 ] && [ "${DC_E2E_KEEP:-false}" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  harness_capture_logs
  remove_stack
  exit "$exit_code"
}

# --- the DC image ---------------------------------------------------------------------

# Sets DC_IMAGE: DC_E2E_IMAGE (prebuilt, no build) > DC_WORKSPACE_IMAGE (prebuilt base)
# > a local build.sh + Containerfile.e2e. CI's e2e jobs set DC_E2E_IMAGE to the :<sha>
# image their build job pushed, so the harness runs the exact built artifact.
resolve_image() {
  if [ -n "${DC_E2E_IMAGE:-}" ]; then
    log "using prebuilt E2E image: $DC_E2E_IMAGE (no build)"
    podman image exists "$DC_E2E_IMAGE" || podman pull "$DC_E2E_IMAGE"
    DC_IMAGE="$DC_E2E_IMAGE"
  else
    local workspace_image
    if [ -n "${DC_WORKSPACE_IMAGE:-}" ]; then
      log "using prebuilt DC workspace image: $DC_WORKSPACE_IMAGE (skipping build.sh)"
      podman image exists "$DC_WORKSPACE_IMAGE" || podman pull "$DC_WORKSPACE_IMAGE"
      workspace_image="$DC_WORKSPACE_IMAGE"
    else
      log "building the DC workspace image (tools/e2e/scripts/build.sh — the same build CI uses)"
      "$HARNESS_SCRIPT_DIR/build.sh"
      workspace_image="dc-workspace:latest"
    fi
    log "building the E2E image (Containerfile.e2e, FROM the workspace image)"
    podman build --build-arg "BASE_IMAGE=$workspace_image" -t dc-e2e:latest \
      -f "$HARNESS_E2E_DIR/Containerfile.e2e" "$HARNESS_E2E_DIR"
    DC_IMAGE="dc-e2e:latest"
  fi
}

# --- destinations ---------------------------------------------------------------------

# One way for bash to run SQL: verify_zero_loss.py owns query execution (and every
# scenario's Record assertions), and this shells out to it — one psql seam for the whole
# e2e tree, not an implementation here and another in the module. Needs --pg-container.
pg_exec() {
  python3 "$HARNESS_SCRIPT_DIR/verify_zero_loss.py" \
    --postgres-container "$HARNESS_PG_C" --exec-sql "$1"
}

# start_postgres <pgdata-volume>: the destination Postgres, with sql/init.sql applied
# on the volume's first boot.
start_postgres() {
  podman run -d --network "$HARNESS_NETWORK" --name "$HARNESS_PG_C" \
    -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
    -v "$1:/var/lib/postgresql/data" \
    -v "$HARNESS_E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
    docker.io/library/postgres:13 >/dev/null
}

start_rustfs() {
  podman run -d --network "$HARNESS_NETWORK" --name "$HARNESS_RUSTFS_C" \
    -v "$1:/data" \
    "$RUSTFS_IMAGE" >/dev/null
}

# Deliberately not pg_isready: the postgres image applies /docker-entrypoint-initdb.d
# against a *temporary* server that pg_isready happily answers, racing init.sql. Waiting
# for the table to resolve gates on the only state that matters — the real server is up
# and init.sql applied.
wait_postgres_ready() {
  local timeout="${1:-120}"
  local deadline=$(( $(date +%s) + timeout ))
  until [ -n "$(pg_exec "SELECT to_regclass('public.dc_records')" 2>/dev/null || true)" ]; do
    [ "$(date +%s)" -lt "$deadline" ] \
      || { log "FAIL: Postgres never came up with sql/init.sql applied"; exit 1; }
    sleep 2
  done
}

# How the harness proves RustFS accepts connections on HARNESS_NETWORK. One hook: the
# default probes from a one-off container on the DC image, and run_limits_drain_rate.sh
# — which has no DC image at all — overrides it to probe from the Shipper image instead.
harness_rustfs_probe() {
  podman run --rm --network "$HARNESS_NETWORK" --entrypoint python3 "$DC_IMAGE" \
    /opt/e2e/measure_rtt.py "$HARNESS_RUSTFS_C" 9000 --count 1 --timeout 2 >/dev/null 2>&1
}

# On the host network RustFS is probed directly; otherwise from a one-off container on
# HARNESS_NETWORK. That probe needs --entrypoint python3 (or bash): the DC image's own
# ENTRYPOINT is entrypoint.sh (the full DC stack), so without the override the probe args
# land as extra ros2-launch arguments and the container always exits non-zero regardless
# of RustFS's actual reachability.
wait_rustfs_ready() {
  local timeout="${1:-60}"
  if [ "$HARNESS_NETWORK" = "host" ]; then
    timeout "$timeout" bash -c 'until curl -sf http://127.0.0.1:9000 >/dev/null 2>&1 || curl -s http://127.0.0.1:9000 >/dev/null 2>&1; do sleep 1; done' \
      || { log "FAIL: RustFS never became ready"; exit 1; }
  else
    log "waiting for RustFS to accept TCP connections on $HARNESS_NETWORK"
    local deadline=$(( $(date +%s) + timeout ))
    until harness_rustfs_probe; do
      [ "$(date +%s)" -lt "$deadline" ] \
        || { log "FAIL: RustFS never became reachable on $HARNESS_NETWORK"; exit 1; }
      sleep 1
    done
  fi
}

# The AWS CLI talks plain S3 to RustFS via --endpoint-url — same protocol dc_bridge's
# Uploader uses (aws-sdk-cpp), so no extra vendor in the loop. Neither the S3 sink nor
# the Uploader creates the bucket; someone has to.
create_rustfs_bucket() {
  podman run --rm --network "$HARNESS_NETWORK" \
    -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
    docker.io/amazon/aws-cli:latest \
    --endpoint-url "$1" s3 mb s3://dc-e2e
}

# extract_from_volume <volume> <entrypoint> [args...]: a one-off read-only container on
# <volume> (mounted at /vol) from the DC image; stdout passes through to the caller's
# redirect or command substitution.
extract_from_volume() {
  local volume="$1" entrypoint="$2"
  shift 2
  podman run --rm --entrypoint "$entrypoint" -v "$volume:/vol:ro" "$DC_IMAGE" "$@"
}

# --- the launch-to-first-Record gate --------------------------------------------------

# wait_first_record <timeout> <start-ts> <context>: poll dc_records every 0.2s until the
# first Record lands, and print the seconds it took since <start-ts> — the caller's, so the
# measurement covers whatever the caller is timing (the launch of the whole stack, or the
# late start of the Shipper). No Record within <timeout> logs FAIL with <context> and
# exits: the three scenario scripts each used to carry this loop verbatim.
wait_first_record() {
  local timeout="$1" start="$2" context="$3"
  local deadline count
  deadline="$(echo "$start + $timeout" | bc)"
  while (( $(echo "$(date +%s.%N) < $deadline" | bc) )); do
    count="$(pg_exec 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)"
    if [ "${count:-0}" -gt 0 ] 2>/dev/null; then
      echo "$(date +%s.%N) - $start" | bc
      return 0
    fi
    sleep 0.2
  done
  log "FAIL: no Record landed in Postgres within ${timeout}s $context" >&2
  exit 1
}
