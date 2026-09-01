# kind: NetworkPolicy validation

The [reference fleet architecture](https://blog.bensoussan.de/post/2026-08-25-a-reference-architecture-for-robot-fleets/)
(epic [#440](https://github.com/minipada/ros2_data_collection/issues/440)) claims a robot
tier has no internet route, sites cannot reach each other, and connections only ever flow
outward (robot -> edge -> hub). Routing alone does not prove that — `NetworkPolicy` is a
Kubernetes object, and enforcing it requires a CNI that implements it. This page proves it
with a real [kind](https://kind.sigs.k8s.io/) cluster (kubeadm, the same tool production
clusters use) running [Calico](https://docs.tigera.io/calico/latest/), a real
policy-enforcing CNI — unlike the [k3d dev loop](https://github.com/minipada/ros2_data_collection/tree/jazzy/deploy/robot)'s
default Flannel, which silently accepts `NetworkPolicy` objects without enforcing them.

```admonish info
Every command below is exactly what CI's `verify-kind-networkpolicy` job
(`.github/workflows/ci.yaml`) runs, one step each — no wrapper script standing between
this page and CI to fall out of sync with either. Run them in order to reproduce
locally, or read on for what each does and why.
```

## Prerequisites

- Podman (pulling or building `dc-ros`, loading it into the cluster)
- Docker (kind's node runtime — see [Docker dependency](#docker-dependency) below)
- `kind` (`v0.33.0`) and `kubectl` (`v1.31.4`) on `PATH` — CI installs these via
  [`helm/kind-action`](https://github.com/helm/kind-action)'s `install_only: true` mode;
  see [`tools/kind/README.md`](https://github.com/minipada/ros2_data_collection/tree/jazzy/tools/kind)
  for why (`kind create cluster --wait`, which that action otherwise always runs, can't
  succeed before Calico is installed)

## 1. Get the `dc-ros` image

`kubernetes/robot-a.yaml` commits a real default: `ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy`
— the same floating ref `build-dc-ros-image` pushes on every merge to `jazzy`, and the
same one `deploy/robot/kubernetes/robot-pod.yaml` runs. (Not `:latest` — this repo
doesn't push that tag; `:jazzy` is the one a real deployment actually pins to.) The
simplest reproduction just pulls it:

```sh
podman pull ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy
export DC_ROS_IMAGE=ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy
```

CI instead pulls the PR's just-built `:<sha>` image (`build-dc-ros-image`'s own output)
— testing what this run actually built, same as `verify-robot-manifests`/
`verify-published-images`. To reproduce *that* case locally instead — testing a change
before it's pushed — build it yourself and point `DC_ROS_IMAGE` at the local tag:

```sh
podman build -t dc-ros:local -f containers/dc-ros/Containerfile --build-arg BASE_IMAGE=dc-workspace:latest containers/dc-ros
export DC_ROS_IMAGE=dc-ros:local
```

Either way, step 4 substitutes `DC_ROS_IMAGE` into the manifest's default only when it
differs from `:jazzy` — one workflow, whichever image you're pointing at.

## 2. Bring up the cluster and its CNI

```sh
kind create cluster --name dc-kind --config tools/kind/kind-config.yaml
kubectl --context kind-dc-kind apply -f https://raw.githubusercontent.com/projectcalico/calico/v3.32.2/manifests/calico.yaml
kubectl --context kind-dc-kind -n kube-system rollout status daemonset/calico-node --timeout=180s
kubectl --context kind-dc-kind -n kube-system rollout status deployment/calico-kube-controllers --timeout=180s
kubectl --context kind-dc-kind wait --for=condition=Ready nodes --all --timeout=180s
```

`tools/kind/kind-config.yaml` sets `disableDefaultCNI: true` — kind's own default
(kindnet) doesn't enforce `NetworkPolicy` either, so a node reports `NotReady` until
Calico is applied.

## 3. Load `dc-ros` into the cluster — no registry

```sh
podman save -o /tmp/dc-ros.tar "$DC_ROS_IMAGE"
kind load image-archive /tmp/dc-ros.tar --name dc-kind
```

Not `kind load docker-image`: that reads from the *Docker* image store, which a
Podman-built image never populates (`CLAUDE.md` "Containers: Podman, not Docker"). The
tar round trip is also what keeps this registry-free — the robot Pod below needs no
`ghcr.io` pull secret.

## 4. Apply the topology

```sh
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/namespaces.yaml

kubectl --context kind-dc-kind create configmap hub-init-sql -n dc-hub \
  --from-file=tools/e2e/sql/init.sql --dry-run=client -o yaml \
  | kubectl --context kind-dc-kind apply -f -

# robot-a.yaml commits ghcr.io/.../dc-ros:jazzy as dc-ros's real default — patched in
# place only if step 1 pointed DC_ROS_IMAGE somewhere else, before the kustomize build
# below reads the file.
sed -i "s|ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy|$DC_ROS_IMAGE|" \
  tools/kind/kubernetes/robot-a.yaml

# Everything else — networkpolicies, the hub, edge and robot tiers, the probe Pods, and
# the two remaining ConfigMaps (generated from tools/kind/params/*) — in one apply.
# kustomize ships in kubectl; see tools/kind/kustomization.yaml for what's deliberately
# left out (the outage-inducing NetworkPolicy variant) and why hub-init-sql above isn't
# generated the same way.
kubectl --context kind-dc-kind apply -k tools/kind/

kubectl --context kind-dc-kind rollout status -n dc-hub deployment/hub-postgres --timeout=180s
kubectl --context kind-dc-kind rollout status -n dc-edge-a deployment/edge-vector --timeout=180s
kubectl --context kind-dc-kind wait -n dc-robot-a --for=condition=Ready pod/robot-a-probe --timeout=60s
kubectl --context kind-dc-kind wait -n dc-edge-a --for=condition=Ready pod/edge-a-probe --timeout=60s
kubectl --context kind-dc-kind wait -n dc-edge-b --for=condition=Ready pod/edge-b-probe --timeout=60s

timeout 90 bash -c \
  "until kubectl --context kind-dc-kind logs -n dc-robot-a dc-robot -c dc-ros 2>&1 | grep -q 'dc_bridge reports ready'; do sleep 2; done"
```

Four namespaces: `dc-robot-a` and `dc-edge-a` (site A, the real topology under test),
`dc-edge-b` (a stand-in second site — nothing runs there but a probe Pod, just enough to
have a second site to deny), and `dc-hub`. See `tools/kind/README.md`'s topology diagram
and each manifest's own header for what runs where and why.

## 5. Prove the claims

```sh
tools/kind/scripts/verify_network_policy.sh
```

Six `kubectl exec ... nc -z` connection attempts from dedicated probe Pods (same
`podSelector: {}` policy every real workload in that namespace is bound by):

| Attempt | Expected |
|---|---|
| robot-a -> public internet | denied |
| robot-a -> `dc-edge-b` | denied |
| `dc-edge-b` -> robot-a's edge (`dc-edge-a`) | denied |
| `dc-edge-b` -> hub | denied |
| robot-a -> `dc-edge-a` | **permitted** |
| `dc-edge-a` -> hub | **permitted** |

Then the real thing, not a probe: wait for `dc-ros`'s own Records to reach the hub's
Postgres through the edge aggregator, and check the row count.

## 6. Induced outage: buffering, not loss

```sh
sleep 15  # steady state
COUNT_BEFORE="$(kubectl --context kind-dc-kind exec -n dc-hub deploy/hub-postgres -- psql -U dc -d dc -tAc 'SELECT count(*) FROM dc_records' | tr -d '[:space:]')"
WINDOW_START_TS="$(date +%s)"

kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/networkpolicy-robot-outage.yaml  # cuts robot -> edge
sleep 30  # outage
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/networkpolicies.yaml              # restores it
sleep 15  # drain

COUNT_AFTER="$(kubectl --context kind-dc-kind exec -n dc-hub deploy/hub-postgres -- psql -U dc -d dc -tAc 'SELECT count(*) FROM dc_records' | tr -d '[:space:]')"
WINDOW_ELAPSED=$(( $(date +%s) - WINDOW_START_TS ))
DELTA=$(( COUNT_AFTER - COUNT_BEFORE ))
echo "records: $COUNT_BEFORE before, $COUNT_AFTER after (+$DELTA over ${WINDOW_ELAPSED}s)"
```

`networkpolicy-robot-outage.yaml` replaces `dc-robot-a`'s `NetworkPolicy` object (same
name, same namespace) with a version that drops the egress-to-edge rule and keeps only
DNS — a real policy-enforced site-link outage, not a stopped container. `dc-ros` and its
local Vector Shipper keep running and buffering to disk the whole time (ADR-0002); once
the policy is restored, the buffered backlog flushes and the hub's row count catches back
up. CI additionally checks `DELTA` against `WINDOW_ELAPSED` seconds at the `uptime`
Measurement's 1Hz rate (`tools/kind/params/robot-a-params.yaml`), with a 70% lower bound
— a real loss would show up as a permanent shortfall there, not a transient dip.

## 7. Tear down

```sh
kind delete cluster --name dc-kind
```

## Docker dependency

kind runs each cluster node as a Docker container, and Calico's kind guide — which this
harness follows — is documented and tested against Docker. Podman has experimental kind
support (`KIND_EXPERIMENTAL_PROVIDER=podman`), but this harness doesn't depend on it: it
is throwaway CI test infrastructure, not something DC ships, so it uses real Docker
rather than an experimental path nothing else in this repo relies on. Building and
shipping DC itself stays on Podman, unchanged (`CLAUDE.md` "Containers: Podman, not
Docker") — Podman is the only tool that ever touches a DC image here; Docker's only job
is running the kind nodes. GitHub-hosted `ubuntu-latest` runners ship Docker
preinstalled, so CI needs no extra setup step for it.
