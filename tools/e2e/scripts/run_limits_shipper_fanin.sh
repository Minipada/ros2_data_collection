#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Limits harness: Shipper fan-in axis + instrument proof (#382, the flagship axis of
# #323's epic). From the repo root:
#
#   ./tools/e2e/scripts/run_limits_shipper_fanin.sh
#
# A sibling of run_limits_two_tier.sh (#381) — its own gates (a ramp to saturation plus
# the induced-limit instrument proof, not a fixed-load pass/fail) — but reuses that
# ticket's topology verbatim: the same params/e2e_limits_params.yaml,
# params/e2e_limits_forward_sink.toml, and the same aggregating-Shipper container names
# those files hard-code (dc-e2e-limits-postgres/rustfs/agg), since the two-tier chain
# they compose is exactly what this ramp drives. The two scripts are never meant to run
# concurrently (same as every other pair of sibling scenario scripts in this harness);
# each does its own full teardown on start and on exit.
#
# What it does, in order:
#   1. Brings up the shared Postgres + RustFS + aggregating-Shipper topology once.
#   2. Runs #379's ramp controller over #378's synthetic load driver and #377's
#      saturation probe (both wired up in scripts/run_limits_shipper_fanin.py) against
#      an UNCONSTRAINED aggregating Shipper — the flagship number.
#   3. Truncates dc_records/dc_files, tears down that phase's Shipper + real stacks
#      (Postgres/RustFS/network stay up), and re-runs the identical ramp against a
#      deliberately CPU/memory-constrained aggregating Shipper.
#   4. Asserts the constrained run's ceiling is substantially lower than the
#      unconstrained one (DC_E2E_FANIN_CEILING_DROP_RATIO, default 0.5) — "a limits
#      harness that cannot demonstrate detecting an induced limit has not been shown to
#      work" (#323's PRD). Neither phase is allowed to end BOUND_NOT_FOUND: an
#      unconstrained run that never saturates has no ceiling to report the PRD's closing
#      sentence about, and a constrained run that never saturates has failed to
#      demonstrate the induced limit — both are hard failures, not skips.
#
# Zero loss is asserted by the Python orchestrator itself, via #379's own driver/probe
# injection seam — see run_limits_shipper_fanin.py's module docstring for exactly how
# (verify_zero_loss.py, reused unmodified, at every level whose verdict stays clear) and
# the one documented trade-off (a live-volume snapshot mid-run) that entails.
#
# Env vars (all optional):
#   DC_E2E_FANIN_REAL_STACKS            real DC stacks kept running for fidelity through
#                                        the whole ramp (default 1 — see
#                                        run_limits_two_tier.sh's own discovery about
#                                        the aggregating Shipper's shared-sink bottleneck
#                                        before raising this)
#   DC_E2E_FANIN_LEVELS                 ascending synthetic-connection levels to ramp
#                                        through (default "25 50 100 200 400 800")
#   DC_E2E_FANIN_SYNTH_RATE_HZ          per-connection Record rate (default 5)
#   DC_E2E_FANIN_SUB_WINDOW_SECONDS     each ramp level's sub-window duration (default 5)
#   DC_E2E_FANIN_SUB_WINDOWS_PER_LEVEL  sub-windows sampled per level (default 5)
#   DC_E2E_FANIN_CONSTRAINED_CPUS       constrained-phase --cpus (default 0.5)
#   DC_E2E_FANIN_CONSTRAINED_MEMORY     constrained-phase --memory (default 256m)
#   DC_E2E_FANIN_CEILING_DROP_RATIO     constrained ceiling / unconstrained ceiling must
#                                        be <= this (default 0.5 — "substantially lower")
#   DC_E2E_KEEP                  "true" to leave the stack (and its network) up after a
#                                 failure for debugging
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE   same meaning as run.sh / run_limits_two_tier.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run/limits_shipper_fanin"

REAL_STACKS="${DC_E2E_FANIN_REAL_STACKS:-1}"
LEVELS="${DC_E2E_FANIN_LEVELS:-25 50 100 200 400 800}"
SYNTH_RATE_HZ="${DC_E2E_FANIN_SYNTH_RATE_HZ:-5}"
SUB_WINDOW_SECONDS="${DC_E2E_FANIN_SUB_WINDOW_SECONDS:-5}"
SUB_WINDOWS_PER_LEVEL="${DC_E2E_FANIN_SUB_WINDOWS_PER_LEVEL:-5}"
CONSTRAINED_CPUS="${DC_E2E_FANIN_CONSTRAINED_CPUS:-0.5}"
CONSTRAINED_MEMORY="${DC_E2E_FANIN_CONSTRAINED_MEMORY:-256m}"
CEILING_DROP_RATIO="${DC_E2E_FANIN_CEILING_DROP_RATIO:-0.5}"

NET=dc-e2e-limits-net
# Same names params/e2e_limits_params.yaml and params/e2e_limits_forward_sink.toml
# hard-code — see this file's header for why reusing them unmodified requires it.
PG_C=dc-e2e-limits-postgres
RUSTFS_C=dc-e2e-limits-rustfs
AGG_C=dc-e2e-limits-agg
AGG_VECTOR_PORT=6000
AGG_FLUENT_PORT=24224
STACK_PREFIX=dc-e2e-limits-stack-
LEDGER_STACK_INDEX=0

cd "$E2E_DIR"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/harness.sh"

stack_name() { echo "${STACK_PREFIX}$1"; }

STACKS=()
STACK_VOLUMES=()
STACK_LOG_CAPTURES=()
i=0
while [ "$i" -lt "$REAL_STACKS" ]; do
  s="$(stack_name "$i")"
  STACKS+=("$s")
  STACK_VOLUMES+=("dc_e2e_limits_buffer_$i" "dc_e2e_limits_data_$i")
  STACK_LOG_CAPTURES+=("$s:$s.log")
  i=$((i + 1))
done

harness_init \
  --tag e2e-fanin \
  --run-dir "$RUN_DIR" \
  --network "$NET" \
  --containers "${STACKS[@]}" "$AGG_C" "$PG_C" "$RUSTFS_C" \
  --volumes "${STACK_VOLUMES[*]}" dc_e2e_limits_pgdata dc_e2e_limits_rustfs_data \
  --teardown-networks "$NET" \
  --log-captures "${STACK_LOG_CAPTURES[@]}" "$AGG_C:agg.log" \
  --pg-container "$PG_C" \
  --rustfs-container "$RUSTFS_C"

# remove_stack is the whole-stack teardown; the per-phase teardown between the two ramp
# phases is the same blocks minus the shared stores and network.
remove_phase_containers() {
  harness_rm_containers "${STACKS[@]}" "$AGG_C"
  harness_rm_volumes "${STACK_VOLUMES[@]}"
}

# --- obtain the DC stack image (shared with run.sh / run_limits_two_tier.sh) ------------
DC_IMAGE="" # set by resolve_image
resolve_image

# Resolved from the .repos pin that fetches vector_vendor (its own repo as of #424's
# follow-up split) by lib/version.sh (#484, already sourced by lib/harness.sh), not
# duplicated here as a second source of truth.
VECTOR_VERSION="$(vector_version)"
VECTOR_IMAGE="docker.io/timberio/vector:${VECTOR_VERSION}-debian"

remove_stack

# --- bridge network + shared destinations (kept up across both phases) ------------------
log "creating the bridge network ($NET)"
podman network create "$NET" >/dev/null

log "starting the shared Postgres + RustFS on $NET"
start_postgres dc_e2e_limits_pgdata
start_rustfs dc_e2e_limits_rustfs_data
wait_postgres_ready
wait_rustfs_ready

log "creating the RustFS bucket"
create_rustfs_bucket "http://$RUSTFS_C:9000"

# --- the aggregating Shipper's Vector config, rendered once and reused across both phases
cat > "$RUN_DIR/aggregator_vector.toml" <<EOF
data_dir = "/var/lib/vector"

[acknowledgements]
enabled = true

[sources.from_tier1]
type = "vector"
address = "0.0.0.0:${AGG_VECTOR_PORT}"

[sources.from_synth]
type = "fluent"
address = "0.0.0.0:${AGG_FLUENT_PORT}"

[sinks.to_postgres]
type = "postgres"
inputs = ["from_tier1"]
endpoint = "postgres://dc:password@${PG_C}:5432/dc"
table = "dc_records"

[sinks.to_postgres.buffer]
type = "disk"
max_size = 268435488

[sinks.discard_synth]
type = "blackhole"
inputs = ["from_synth"]
EOF

truncate_records() {
  log "truncating dc_records/dc_files so the next phase starts from a clean baseline"
  pg_exec "TRUNCATE dc_records, dc_files" >/dev/null
}

# --- run one phase (unconstrained or constrained) end to end -----------------------------
run_phase() {
  local label="$1"
  shift
  local agg_extra_args=("$@")

  log "=== phase: $label ==="
  log "starting the aggregating Shipper ($AGG_C)${agg_extra_args[*]:+ with ${agg_extra_args[*]}}"
  podman run -d --network "$NET" -p "${AGG_FLUENT_PORT}:${AGG_FLUENT_PORT}" --name "$AGG_C" \
    "${agg_extra_args[@]}" \
    -v "$RUN_DIR/aggregator_vector.toml:/etc/vector/vector.toml:Z" \
    "$VECTOR_IMAGE" -c /etc/vector/vector.toml >/dev/null

  log "waiting for the aggregating Shipper's synthetic-sender ingress to accept connections"
  if ! timeout 30 bash -c "until (exec 3<>/dev/tcp/127.0.0.1/${AGG_FLUENT_PORT}) 2>/dev/null; do sleep 0.5; done"; then
    log "FAIL: the aggregating Shipper's fluent source never started listening on port $AGG_FLUENT_PORT"
    exit 1
  fi
  log "waiting for the aggregating Shipper's tier-1 ingress to accept connections"
  timeout 60 bash -c "
    until podman run --rm --network $NET --entrypoint bash '$DC_IMAGE' -c 'exec 3<>/dev/tcp/$AGG_C/$AGG_VECTOR_PORT' >/dev/null 2>&1; do
      sleep 1
    done
  " || { log "FAIL: the aggregating Shipper's vector-protocol source never became reachable on $NET"; exit 1; }

  log "starting $REAL_STACKS real DC stack(s) against params/e2e_limits_params.yaml"
  local i s
  for ((i = 0; i < REAL_STACKS; i++)); do
    s="$(stack_name "$i")"
    podman volume create "dc_e2e_limits_buffer_$i" >/dev/null
    podman volume create "dc_e2e_limits_data_$i" >/dev/null
    podman run -d --network "$NET" --name "$s" \
      -v "dc_e2e_limits_buffer_$i:/root/.dc/e2e/buffer" \
      -v "dc_e2e_limits_data_$i:/root/.dc/e2e/data" \
      -v "$E2E_DIR/params/e2e_limits_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
      -v "$E2E_DIR/params/e2e_limits_forward_sink.toml:/opt/e2e/e2e_limits_forward_sink.toml:ro" \
      -v "$E2E_DIR/params/e2e_limits_pgsql_files_sink.toml:/opt/e2e/e2e_limits_pgsql_files_sink.toml:ro" \
      "$DC_IMAGE" >/dev/null
    sleep 5
  done

  log "waiting for the first Record to reach Postgres through the two-tier chain"
  wait_first_record 90 "$(date +%s.%N)" "through the two-tier chain" >/dev/null
  log "PASS: real-stack traffic is flowing through the aggregating Shipper — starting the ramp"

  local data_vol="dc_e2e_limits_data_${LEDGER_STACK_INDEX}"
  local report_json="$RUN_DIR/${label}_report.json"
  local report_txt="$RUN_DIR/${label}_summary.txt"

  # shellcheck disable=SC2086
  uv run --frozen python3 "$SCRIPT_DIR/run_limits_shipper_fanin.py" \
    --label "$label" \
    --agg-host 127.0.0.1 --agg-port "$AGG_FLUENT_PORT" \
    --agg-container "$AGG_C" \
    --postgres-container "$PG_C" \
    --data-volume "$data_vol" \
    --dc-image "$DC_IMAGE" \
    --real-stacks "$REAL_STACKS" \
    --levels $LEVELS \
    --synth-rate-hz "$SYNTH_RATE_HZ" \
    --sub-window-seconds "$SUB_WINDOW_SECONDS" \
    --sub-windows-per-level "$SUB_WINDOWS_PER_LEVEL" \
    --run-dir "$RUN_DIR" \
    --report-json "$report_json" \
    --report-txt "$report_txt" \
    | tee "$RUN_DIR/${label}_orchestrator.log"

  log "stopping $label's real DC stack(s)"
  for s in "${STACKS[@]}"; do
    podman stop "$s" >/dev/null
  done

  # The orchestrator's own final line is a single-line JSON summary — see
  # run_limits_shipper_fanin.py's `run()`.
  tail -1 "$RUN_DIR/${label}_orchestrator.log" > "$RUN_DIR/${label}_result.json"
}

stop_phase() {
  local s
  for s in "${STACKS[@]}"; do
    if podman container exists "$s"; then
      podman logs "$s" > "$RUN_DIR/${s}_$1.log" 2>&1
    fi
  done
  if podman container exists "$AGG_C"; then
    podman logs "$AGG_C" > "$RUN_DIR/agg_$1.log" 2>&1
  fi
  remove_phase_containers
}

ceiling_of() {
  # KNEE_FOUND's ceiling is the tripped level (the actual measured saturation point,
  # never highest_clear_level — that would understate what was actually driven to).
  python3 -c "
import json
r = json.load(open('$1'))
if r['outcome'] != 'knee_found':
    print('NONE')
else:
    print(r['tripped_level'])
"
}

outcome_of() {
  python3 -c "import json; print(json.load(open('$1'))['outcome'])"
}

# --- phase 1: unconstrained --------------------------------------------------------------
run_phase "unconstrained"
stop_phase "unconstrained"

UNCONSTRAINED_OUTCOME="$(outcome_of "$RUN_DIR/unconstrained_result.json")"
if [ "$UNCONSTRAINED_OUTCOME" != "knee_found" ]; then
  log "FAIL: the unconstrained run never saturated within the tested levels ($LEVELS) — no measured ceiling to report the PRD's closing sentence about. Raise DC_E2E_FANIN_LEVELS."
  exit 1
fi
UNCONSTRAINED_CEILING="$(ceiling_of "$RUN_DIR/unconstrained_result.json")"
log "unconstrained ceiling: $UNCONSTRAINED_CEILING synthetic connections"

truncate_records

# --- phase 2: deliberately constrained ----------------------------------------------------
run_phase "constrained" --cpus "$CONSTRAINED_CPUS" --memory "$CONSTRAINED_MEMORY"
stop_phase "constrained"

CONSTRAINED_OUTCOME="$(outcome_of "$RUN_DIR/constrained_result.json")"
if [ "$CONSTRAINED_OUTCOME" != "knee_found" ]; then
  log "FAIL: the constrained run (--cpus $CONSTRAINED_CPUS --memory $CONSTRAINED_MEMORY) never saturated within the tested levels ($LEVELS) — the instrument has not demonstrated detecting the induced limit."
  exit 1
fi
CONSTRAINED_CEILING="$(ceiling_of "$RUN_DIR/constrained_result.json")"
log "constrained ceiling: $CONSTRAINED_CEILING synthetic connections"

# --- the instrument proof: constrained ceiling must be substantially lower ---------------
RATIO="$(python3 -c "print($CONSTRAINED_CEILING / $UNCONSTRAINED_CEILING)")"
log "constrained/unconstrained ceiling ratio: $RATIO (must be <= $CEILING_DROP_RATIO)"
if ! python3 -c "import sys; sys.exit(0 if $RATIO <= $CEILING_DROP_RATIO else 1)"; then
  log "FAIL: the constrained run's ceiling ($CONSTRAINED_CEILING) is not substantially lower than the unconstrained run's ($UNCONSTRAINED_CEILING) — ratio $RATIO exceeds $CEILING_DROP_RATIO. The instrument has not demonstrated detecting the induced limit."
  exit 1
fi
log "PASS: the constrained aggregating Shipper's ceiling is substantially lower than the unconstrained one — the instrument detects an induced limit"

echo
echo "=== #323's required closing sentence (unconstrained run) ==="
tail -2 "$RUN_DIR/unconstrained_summary.txt"

log "PASS: limits harness Shipper fan-in axis + instrument proof (#382)"
