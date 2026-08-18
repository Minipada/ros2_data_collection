#!/bin/bash
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# Degraded-network E2E scenario (#366). From the repo root:
#
#   ./tools/e2e/scripts/run_degraded.sh                       # loopback profile (baseline)
#   DC_E2E_PROFILE=poor-wifi ./tools/e2e/scripts/run_degraded.sh
#
# run.sh's zero-loss guarantee has only ever been proven over intra-container loopback:
# ~microsecond RTT, zero loss, unbounded bandwidth, and a fault model where a Destination
# either runs or is a stopped process. Real robots reach their Destinations over WiFi and a
# site uplink — tens of milliseconds of RTT, packet loss, bufferbloat, and a link that can
# disappear for seconds without anything dying. This scenario asserts the same zero-loss
# guarantee (verify_zero_loss.py, reused unmodified) under stated, reproducible network
# conditions instead, plus a fault type run.sh has no equivalent of: the link going away
# while the process behind it keeps running.
#
# Sibling of run.sh, not a mode inside it (own gates, own topology decision below) — same
# `dc-e2e` image, same DC_E2E_IMAGE/DC_WORKSPACE_IMAGE env vars, same reference workload
# (params/e2e_degraded_params.yaml, byte-for-byte params/e2e_params.yaml except for the
# two Destination hosts — see that file's header for why). Not wired into ci.yaml, same as
# run_retention.sh/run_incident.sh: whether it gates merges is left for once its runtime
# and stability are measured (#366's own "Out of Scope").
#
# --- Why this needs its own network, and how shaping actually works here ---------------
#
# Every existing scenario runs `--network host`: every container shares the host's network
# namespace, so nothing can be shaped without shaping the host itself. This scenario instead
# puts Postgres and RustFS on their own bridge network ($NET below) with the `dc` stack
# alongside them (reachable by container-name DNS, not 127.0.0.1 — podman's aardvark-dns on
# a user-defined network resolves container names, verified empirically here). The
# Bridge-to-Shipper socket stays out of scope (same-container loopback — see #366's "Out of
# Scope" and e2e_degraded_params.yaml's header): what's actually shaped is the Shipper's
# path to Postgres/RustFS, the only path that ever leaves a container.
#
# `tc qdisc ... netem` needs CAP_NET_ADMIN in whatever network namespace it targets. Rather
# than grant that to the Postgres/RustFS images themselves, a short-lived helper container
# joins the target's netns (`podman run --network container:<name> --cap-add=NET_ADMIN`) and
# applies/changes/removes the qdisc from there — the target container runs completely
# unmodified. Verified empirically in this environment: rootless podman +
# `--cap-add=NET_ADMIN` on a netns-joining helper lets an otherwise-unprivileged container
# add a `netem` qdisc to another container's `eth0`, and a peer on the same bridge network
# measurably sees the added delay/jitter/loss. That this works at all was the single
# largest technical risk in #366's PRD and was established before anything else here was
# built; if it stops working in some other environment, the pre-check below (not a
# ping/`iputils` dependency, which the Postgres/RustFS images don't ship — a stdlib TCP
# connect-time sample, scripts/measure_rtt.py) fails the run loudly rather than silently
# passing a green "degraded" result that never actually shaped anything.
#
# --- Fault model -------------------------------------------------------------------------
#
# In addition to the profile's steady-state shaping, one LINK fault is induced partway
# through the run: `tc qdisc ... netem loss 100%` on Postgres's and RustFS's own interfaces
# for DC_E2E_LINK_OUTAGE_SECONDS, then restored to the profile's own parameters (or removed
# entirely, for the loopback profile). Both processes keep running throughout — unlike
# run.sh's `podman stop`/`restart`, nothing here resets Forwarder state, so this is a
# distinct fault type on purpose (see #366's PRD): "the network went away" tested apart
# from "the process went down", which run.sh already covers.
#
# Env vars (all optional):
#   DC_E2E_PROFILE                  named profile from scripts/network_profiles.py:
#                                    loopback (default) / good-wifi / poor-wifi / site-uplink
#   DC_E2E_STEADY_STATE_SECONDS     warmup under the profile before the link fault (default 30)
#   DC_E2E_LINK_OUTAGE_SECONDS      link-fault duration (default 60)
#   DC_E2E_DRAIN_SECONDS            settle time after the link is restored (default 30)
#   DC_E2E_STARTUP_TIMEOUT_SECONDS  startup-latency gate (default 10, same as run.sh)
#   DC_E2E_KEEP                     "true" to leave the stack (and its network) up after a
#                                    failure for debugging
#   DC_E2E_IMAGE / DC_WORKSPACE_IMAGE   same meaning as run.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$E2E_DIR/.run"

PROFILE_NAME="${DC_E2E_PROFILE:-loopback}"
STEADY_STATE_SECONDS="${DC_E2E_STEADY_STATE_SECONDS:-30}"
LINK_OUTAGE_SECONDS="${DC_E2E_LINK_OUTAGE_SECONDS:-60}"
DRAIN_SECONDS="${DC_E2E_DRAIN_SECONDS:-30}"
STARTUP_TIMEOUT_SECONDS="${DC_E2E_STARTUP_TIMEOUT_SECONDS:-10}"
KEEP="${DC_E2E_KEEP:-false}"

NET=dc_e2e_deg_net
SHAPER_IMAGE=dc-e2e-shaper:latest
PG_C=dc_e2e_deg_postgres
RUSTFS_C=dc_e2e_deg_rustfs
DC_C=dc_e2e_deg_dc
VOLUMES=(dc_e2e_deg_pgdata dc_e2e_deg_rustfs_data dc_e2e_deg_buffer dc_e2e_deg_data)

mkdir -p "$RUN_DIR"
cd "$E2E_DIR"

log() { echo "[e2e-degraded $(date -u +%H:%M:%S)] $*"; }

remove_stack() {
  podman rm -f --ignore "$DC_C" "$PG_C" "$RUSTFS_C" >/dev/null
  for v in "${VOLUMES[@]}"; do
    if podman volume exists "$v"; then
      podman volume rm "$v" >/dev/null
    fi
  done
  podman network rm "$NET" >/dev/null 2>&1 || true
}

pg_exec() {
  podman exec "$PG_C" psql -U dc -d dc -tAc "$1"
}

cleanup() {
  local exit_code=$?
  if [ "$exit_code" -ne 0 ] && [ "$KEEP" = "true" ]; then
    log "FAILED (exit $exit_code) — leaving the stack up (DC_E2E_KEEP=true) for debugging"
    exit "$exit_code"
  fi
  log "tearing down"
  if podman container exists "$DC_C"; then
    podman logs "$DC_C" > "$RUN_DIR/dc_degraded.log" 2>&1
  fi
  remove_stack
  exit "$exit_code"
}
trap cleanup EXIT

# --- resolve the network profile (scripts/network_profiles.py — one declarative place) --
log "resolving network profile '$PROFILE_NAME'"
if ! PROFILE_SHELL_VARS="$(python3 "$SCRIPT_DIR/network_profiles.py" "$PROFILE_NAME" --shell)"; then
  python3 "$SCRIPT_DIR/network_profiles.py" "$PROFILE_NAME" --shell || true
  log "FAIL: unknown network profile '$PROFILE_NAME'"
  exit 1
fi
eval "$PROFILE_SHELL_VARS"
NETEM_ARGS="$(python3 "$SCRIPT_DIR/network_profiles.py" "$PROFILE_NAME" --tc-args)"
log "profile '$PROFILE_NAME': delay=${PROFILE_DELAY_MS}ms jitter=${PROFILE_JITTER_MS}ms loss=${PROFILE_LOSS_PCT}% rate=${PROFILE_RATE_KBIT}kbit shaped=${PROFILE_IS_SHAPED}"

# --- obtain the DC stack image (shared with run.sh — see its header) ------------------
if [ -n "${DC_E2E_IMAGE:-}" ]; then
  log "using prebuilt E2E image: $DC_E2E_IMAGE (no build)"
  podman image exists "$DC_E2E_IMAGE" || podman pull "$DC_E2E_IMAGE"
  DC_IMAGE="$DC_E2E_IMAGE"
else
  if [ -n "${DC_WORKSPACE_IMAGE:-}" ]; then
    log "using prebuilt DC workspace image: $DC_WORKSPACE_IMAGE (skipping build.sh)"
    podman image exists "$DC_WORKSPACE_IMAGE" || podman pull "$DC_WORKSPACE_IMAGE"
    WORKSPACE_IMAGE="$DC_WORKSPACE_IMAGE"
  else
    log "building the DC workspace image (tools/e2e/scripts/build.sh — the same build CI uses)"
    "$SCRIPT_DIR/build.sh"
    WORKSPACE_IMAGE="dc-workspace:latest"
  fi
  log "building the E2E image (Containerfile.e2e, FROM the workspace image)"
  podman build --build-arg "BASE_IMAGE=$WORKSPACE_IMAGE" -t dc-e2e:latest -f Containerfile.e2e .
  DC_IMAGE="dc-e2e:latest"
fi

# --- the netem shaper helper image: alpine + iproute2, built once and reused -----------
if ! podman image exists "$SHAPER_IMAGE"; then
  log "building the netem shaper helper image ($SHAPER_IMAGE — alpine + iproute2, cached after this)"
  podman rm -f --ignore dc_e2e_shaper_build >/dev/null
  podman run --name dc_e2e_shaper_build docker.io/library/alpine:3.20 \
    sh -c 'apk add --no-cache iproute2' >/dev/null
  podman commit dc_e2e_shaper_build "$SHAPER_IMAGE" >/dev/null
  podman rm -f dc_e2e_shaper_build >/dev/null
fi

# tc_run/set_qdisc/clear_qdisc: apply netem to $1's OWN interface by joining its network
# namespace, without $1 itself needing any capability (see this script's header).
# QDISC_UP tracks, per target container, whether a netem qdisc is currently installed —
# `tc qdisc change` requires one to already exist; `add` requires one not to.
declare -A QDISC_UP=()

tc_run() {
  local target="$1"
  shift
  podman run --rm --network "container:$target" --cap-add=NET_ADMIN "$SHAPER_IMAGE" tc "$@"
}

set_qdisc() {
  local target="$1"
  shift
  if [ "${QDISC_UP[$target]:-false}" = "true" ]; then
    tc_run "$target" qdisc change dev eth0 root "$@"
  else
    tc_run "$target" qdisc add dev eth0 root "$@"
    QDISC_UP[$target]=true
  fi
}

clear_qdisc() {
  local target="$1"
  tc_run "$target" qdisc del dev eth0 root
  QDISC_UP[$target]=false
}

# Clean any leftovers from a previous (possibly DC_E2E_KEEP=true) run.
remove_stack

# --- shaped bridge network + destinations ------------------------------------------------
log "creating the shaped bridge network ($NET)"
podman network create "$NET" >/dev/null

log "starting Postgres + RustFS on $NET (unmodified — no special capabilities on either)"
podman run -d --network "$NET" --name "$PG_C" \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_e2e_deg_pgdata:/var/lib/postgresql/data \
  -v "$E2E_DIR/sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro" \
  docker.io/library/postgres:13 >/dev/null
podman run -d --network "$NET" --name "$RUSTFS_C" \
  -v dc_e2e_deg_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c >/dev/null

timeout 120 bash -c "until podman exec $PG_C psql -U dc -d dc -tAc \"SELECT to_regclass('public.dc_records')\" 2>/dev/null | grep -q dc_records; do sleep 2; done" \
  || { log "FAIL: Postgres never came up with sql/init.sql applied"; exit 1; }

log "waiting for RustFS to accept TCP connections on $NET"
timeout 60 bash -c "
  until podman run --rm --network $NET '$DC_IMAGE' python3 /opt/e2e/measure_rtt.py $RUSTFS_C 9000 --count 1 --timeout 2 >/dev/null 2>&1; do
    sleep 1
  done
" || { log "FAIL: RustFS never became reachable on $NET"; exit 1; }

log "creating the RustFS bucket"
podman run --rm --network "$NET" \
  -e AWS_ACCESS_KEY_ID=rustfsadmin -e AWS_SECRET_ACCESS_KEY=rustfsadmin -e AWS_DEFAULT_REGION=us-east-1 \
  docker.io/amazon/aws-cli:latest \
  --endpoint-url "http://$RUSTFS_C:9000" s3 mb s3://dc-e2e

# --- apply the profile's steady-state shaping, then verify it actually took effect -----
if [ "$PROFILE_IS_SHAPED" = "true" ]; then
  log "applying profile '$PROFILE_NAME' to Postgres/RustFS's own interfaces: tc qdisc ... $NETEM_ARGS"
  # shellcheck disable=SC2086  # NETEM_ARGS is a deliberate, simple-token word split
  set_qdisc "$PG_C" $NETEM_ARGS
  # shellcheck disable=SC2086
  set_qdisc "$RUSTFS_C" $NETEM_ARGS
else
  log "profile '$PROFILE_NAME' is unshaped — no qdisc applied (today's loopback behaviour)"
fi

log "shaping pre-check: measuring the actual TCP connect-time round trip to $PG_C"
RTT_JSON="$(podman run --rm --network "$NET" "$DC_IMAGE" python3 /opt/e2e/measure_rtt.py "$PG_C" 5432 --count 10 --timeout 3)"
echo "$RTT_JSON" > "$RUN_DIR/rtt_precheck.json"
AVG_MS="$(python3 -c "import json; print(json.load(open('$RUN_DIR/rtt_precheck.json'))['avg_ms'])")"
log "measured avg connect time: ${AVG_MS}ms"

if [ "$PROFILE_IS_SHAPED" = "true" ]; then
  # A conservative floor, not the full delay: netem on Postgres's own egress only delays
  # its outbound leg of the TCP handshake, not the client's SYN — asserting some
  # meaningful fraction of the requested delay is enough to prove shaping took effect
  # without the check being brittle to exactly which leg dominates the measurement.
  MIN_EXPECTED_MS="$(python3 -c "print(max(${PROFILE_DELAY_MS} * 0.4, 5))")"
  if ! python3 -c "import sys; sys.exit(0 if ${AVG_MS} >= ${MIN_EXPECTED_MS} else 1)"; then
    log "FAIL: shaping pre-check failed — measured ${AVG_MS}ms is below the ${MIN_EXPECTED_MS}ms floor"
    log "      expected for profile '$PROFILE_NAME' (delay=${PROFILE_DELAY_MS}ms). A degraded run"
    log "      must never pass silently as loopback — this container runtime may not support"
    log "      unprivileged tc/NET_ADMIN shaping the way this script assumes (see its header)."
    exit 1
  fi
  log "PASS: shaping pre-check — ${AVG_MS}ms is consistent with profile '$PROFILE_NAME' actually applying"
else
  LOOPBACK_CEILING_MS=20
  if ! python3 -c "import sys; sys.exit(0 if ${AVG_MS} < ${LOOPBACK_CEILING_MS} else 1)"; then
    log "FAIL: shaping pre-check failed — loopback should be fast and unshaped, but measured ${AVG_MS}ms"
    exit 1
  fi
  log "PASS: shaping pre-check — ${AVG_MS}ms confirms no shaping is applied for '$PROFILE_NAME'"
fi

python3 -c "
import json
rtt = json.load(open('$RUN_DIR/rtt_precheck.json'))
conditions = {
    'profile': '$PROFILE_NAME',
    'delay_ms': $PROFILE_DELAY_MS,
    'jitter_ms': $PROFILE_JITTER_MS,
    'loss_pct': $PROFILE_LOSS_PCT,
    'rate_kbit': $PROFILE_RATE_KBIT,
    'link_outage_seconds': $LINK_OUTAGE_SECONDS,
    'measured_rtt_precheck': rtt,
}
json.dump(conditions, open('$RUN_DIR/conditions.json', 'w'), indent=2)
"
log "conditions recorded: $RUN_DIR/conditions.json — a result is never separable from these"

# --- start the DC stack, measure launch-to-first-Record -----------------------------
log "starting the DC stack against params/e2e_degraded_params.yaml — measuring launch-to-first-Record latency"
START_TS=$(date +%s.%N)
podman run -d --network "$NET" --name "$DC_C" \
  -v dc_e2e_deg_buffer:/root/.dc/e2e/buffer \
  -v dc_e2e_deg_data:/root/.dc/e2e/data \
  -v "$E2E_DIR/params/e2e_degraded_params.yaml:/opt/e2e/e2e_params.yaml:ro" \
  "$DC_IMAGE" >/dev/null

FIRST_RECORD_LATENCY=""
DEADLINE=$(echo "$START_TS + $STARTUP_TIMEOUT_SECONDS + 5" | bc)
while (( $(echo "$(date +%s.%N) < $DEADLINE" | bc) )); do
  COUNT="$(pg_exec 'SELECT count(*) FROM dc_records' 2>/dev/null || echo 0)"
  if [ "${COUNT:-0}" -gt 0 ] 2>/dev/null; then
    FIRST_RECORD_LATENCY=$(echo "$(date +%s.%N) - $START_TS" | bc)
    break
  fi
  sleep 0.2
done

if [ -z "$FIRST_RECORD_LATENCY" ]; then
  log "FAIL: no Record landed in Postgres within $((STARTUP_TIMEOUT_SECONDS + 5))s of starting the stack"
  exit 1
fi
log "first Record landed after ${FIRST_RECORD_LATENCY}s"
if (( $(echo "$FIRST_RECORD_LATENCY > $STARTUP_TIMEOUT_SECONDS" | bc) )); then
  log "FAIL: startup latency ${FIRST_RECORD_LATENCY}s exceeds the ${STARTUP_TIMEOUT_SECONDS}s gate"
  exit 1
fi
log "PASS: startup latency gate (<${STARTUP_TIMEOUT_SECONDS}s) under profile '$PROFILE_NAME'"

log "steady state for ${STEADY_STATE_SECONDS}s under profile '$PROFILE_NAME'"
sleep "$STEADY_STATE_SECONDS"

# --- the LINK fault: the network goes away, the processes do not ----------------------
log "inducing a LINK fault for ${LINK_OUTAGE_SECONDS}s (tc netem loss 100%% — Postgres and RustFS keep running throughout, unlike run.sh's container stop)"
set_qdisc "$PG_C" netem loss 100%
set_qdisc "$RUSTFS_C" netem loss 100%
sleep "$LINK_OUTAGE_SECONDS"

log "restoring the link to profile '$PROFILE_NAME' (the processes were never stopped)"
if [ "$PROFILE_IS_SHAPED" = "true" ]; then
  # shellcheck disable=SC2086
  set_qdisc "$PG_C" $NETEM_ARGS
  # shellcheck disable=SC2086
  set_qdisc "$RUSTFS_C" $NETEM_ARGS
else
  clear_qdisc "$PG_C"
  clear_qdisc "$RUSTFS_C"
fi

log "draining for ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"

log "stopping the workload so counts settle before verification"
podman stop "$DC_C" >/dev/null
sleep 5

# --- extract the passthrough sink's output (ADR-0003) --------------------------------
log "extracting the passthrough sink's output"
podman run --rm --entrypoint bash \
  -v dc_e2e_deg_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/passthrough/records.ndjson 2>/dev/null || true' > "$RUN_DIR/passthrough_degraded.ndjson"

# --- extract raw / generic-subscription mode's output (#227) -------------------------
log "extracting raw mode's Destination output"
podman run --rm --entrypoint bash \
  -v dc_e2e_deg_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/raw/records.ndjson 2>/dev/null || true' > "$RUN_DIR/raw_degraded.ndjson"

# --- summarize the MCAP passthrough writer's output (ADR-0009, #210) -----------------
log "summarizing the MCAP passthrough writer's output"
podman run --rm --entrypoint python3 \
  -v dc_e2e_deg_data:/vol:ro "$DC_IMAGE" \
  /opt/e2e/mcap_summary.py /vol/mcap > "$RUN_DIR/mcap_summary_degraded.json"

# --- verify (verify_zero_loss.py's own logic, unmodified — only conditions are added) --
log "extracting the workload ledger (what the generator published)"
podman run --rm --entrypoint bash \
  -v dc_e2e_deg_data:/vol:ro "$DC_IMAGE" \
  -c 'cat /vol/workload_ledger.txt 2>/dev/null || true' > "$RUN_DIR/workload_ledger_degraded.txt"

log "verifying zero-loss against the published ledger, under profile '$PROFILE_NAME'"
python3 "$SCRIPT_DIR/verify_zero_loss.py" \
  --postgres-container "$PG_C" \
  --num-synth-topics 14 \
  --ledger-file "$RUN_DIR/workload_ledger_degraded.txt" \
  --passthrough-file "$RUN_DIR/passthrough_degraded.ndjson" \
  --mcap-summary-file "$RUN_DIR/mcap_summary_degraded.json" \
  --raw-file "$RUN_DIR/raw_degraded.ndjson" \
  --conditions-file "$RUN_DIR/conditions.json" \
  --report "$RUN_DIR/verification_report_degraded.json"

log "PASS: zero-loss E2E harness under network profile '$PROFILE_NAME' (#366)"
