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
Every command below is exactly what `tools/kind/scripts/run.sh` runs — CI's
`verify-kind-networkpolicy` job, a local reproduction, and this page are the same
sequence, not three different ones. Running the script end to end is the normal way to
do this; read on for what it does and why, or to run a step by hand.
```

## Prerequisites

- Podman (building `dc-ros`, loading it into the cluster)
- Docker (kind's node runtime — see [Docker dependency](#docker-dependency) below)
- `kind` and `kubectl`, pinned versions baked into `containers/kind-tools/Containerfile`
  ([`tools/kind/README.md`](https://github.com/minipada/ros2_data_collection/tree/jazzy/tools/kind))

## 1. Build (or obtain) the `dc-ros` image

```sh
podman build -t dc-ros:kind -f containers/dc-ros/Containerfile --build-arg BASE_IMAGE=dc-workspace:latest containers/dc-ros
```

CI instead pulls the image `build-dc-ros-image` already built and pushed for this commit
— no rebuild, same image `verify-robot-manifests`/`verify-published-images` exercise.

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
podman save -o /tmp/dc-ros.tar dc-ros:kind
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
kubectl --context kind-dc-kind create configmap edge-a-vector-config -n dc-edge-a \
  --from-file=vector.toml=tools/kind/params/edge-a-vector.toml --dry-run=client -o yaml \
  | kubectl --context kind-dc-kind apply -f -
kubectl --context kind-dc-kind create configmap robot-a-params -n dc-robot-a \
  --from-file=robot_params.yaml=tools/kind/params/robot-a-params.yaml --dry-run=client -o yaml \
  | kubectl --context kind-dc-kind apply -f -

kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/networkpolicies.yaml
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/hub-postgres.yaml
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/edge-a.yaml    # ${VECTOR_IMAGE} substituted first — see run.sh
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/probes.yaml
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/robot-a.yaml  # ${DC_ROS_IMAGE}/${VECTOR_IMAGE} substituted first
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
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/networkpolicy-robot-outage.yaml  # cuts robot -> edge
sleep 30
kubectl --context kind-dc-kind apply -f tools/kind/kubernetes/networkpolicies.yaml              # restores it
```

`networkpolicy-robot-outage.yaml` replaces `dc-robot-a`'s `NetworkPolicy` object (same
name, same namespace) with a version that drops the egress-to-edge rule and keeps only
DNS — a real policy-enforced site-link outage, not a stopped container. `dc-ros` and its
local Vector Shipper keep running and buffering to disk the whole time (ADR-0002); once
the policy is restored, the buffered backlog flushes and the hub's row count catches back
up. `run.sh` checks the count grew by roughly what the collection rate over the whole
window predicts — a real loss would show up as a permanent shortfall, not a transient dip.

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
