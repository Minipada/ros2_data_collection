# kind validation of the three-tier topology with NetworkPolicy (#452)

The fleet's security properties are the claim that most needs testing: the robot cannot
reach the internet, sites cannot reach each other, and connections only ever go outward
(epic #440's reference architecture). Routing alone does not prove this — `NetworkPolicy`
is a Kubernetes object, and enforcing it requires a CNI that implements it.

This harness brings up a [kind](https://kind.sigs.k8s.io/) cluster — kubeadm, the same
tool used for production clusters — with [Calico](https://docs.tigera.io/calico/latest/)
as a real, policy-enforcing CNI, deploys a robot tier, an edge aggregator, a stand-in
second site, and a hub, and proves the claim by attempted connection:

| Check | How |
|---|---|
| Robot tier has no internet route | `NetworkPolicy` denies egress to a real external address; the attempt is made and must fail |
| Sites cannot reach each other | robot-a/edge-a cannot reach `dc-edge-b` (a stand-in second site), and vice versa |
| Robot-to-edge and edge-to-hub are permitted | positive-control connection attempts succeed |
| Records collected on the robot tier arrive at the hub through the edge aggregator | a real `dc-ros` produces Records; the hub's Postgres row count is checked |
| The robot tier keeps collecting and buffering while the edge tier is unreachable, losing nothing when it returns | `NetworkPolicy` is swapped to cut robot -> edge for a window, then restored; the row count must have grown by roughly what the collection rate over that window predicts |

There is no wrapper script here — `.github/workflows/ci.yaml`'s `verify-kind-networkpolicy`
job and [the doc page](../../doc/src/dc/deploy_kind_networkpolicy.md) both run the same
`podman`/`kind`/`kubectl` commands directly, one place each, so there's nothing for the
two to drift out of sync from beyond each other (CLAUDE.md's "one script for CI and dev"
rule doesn't apply to a fixed sequence of one-shot commands with no control flow — it's
`verify_network_policy.sh` below, which *does* have real logic, that stays a script
rather than being duplicated). Follow the doc page to run this by hand.

## What this is not

This is the production-parity proof, run as a pre-merge CI check
(`.github/workflows/ci.yaml`'s `verify-kind-networkpolicy` job), separate from the fast
`colcon test`/`prek` development loop — it takes minutes, not seconds, because bringing
up a real cluster and inducing a real outage takes real time. #451's k3d cluster is the
opposite: a fast local inner loop for iterating on the Kubernetes rendering itself.
Neither replaces `deploy/robot/scripts/verify_kube_play.sh`/`verify_network_isolation.sh`
(#450), which prove the same manifests are runnable and prove the routing-level version
of the same claim without needing a cluster at all — see that directory's README for why
k3d's default Flannel CNI can't do what this harness does: it accepts `NetworkPolicy`
objects without enforcing them.

## Topology

```
dc-robot-a (robot, site A)  --24224-->  dc-edge-a (edge, site A)  --5432-->  dc-hub
                                              x
dc-edge-b (stand-in, site B) -- denied both directions to/from site A's tiers
```

- **dc-robot-a**: a Pod with `dc-ros` (unmanaged-shipper mode) and `vector` (the robot's
  local Shipper) — two of `deploy/robot/kubernetes/robot-pod.yaml`'s three containers.
  No `dc-uploader`: `params/robot-a-params.yaml`'s only Measurement (`uptime`) produces
  no Files, so there is nothing for one to do — this harness proves Records, not Files.
- **dc-edge-a**: plain Vector, no DC/ROS (ADR-0015: "No DC and no ROS run here"),
  aggregating robot-tier relays and writing straight into the hub's Postgres.
- **dc-edge-b**: a lone probe Pod standing in for a second site, so "sites cannot reach
  each other" has something concrete to fail against.
- **dc-hub**: Postgres, `tools/e2e/sql/init.sql`'s schema — the same `dc_records` shape
  every other E2E scenario in this repo uses.

`kubernetes/` holds the Kubernetes manifests (kubeconform-validated, same pre-commit hook
as `deploy/robot/kubernetes/`); `params/` holds the two files that differ from their
`deploy/robot/` counterparts only in pointing at real in-cluster DNS names instead of
placeholders — see each file's own header for what and why. `../kustomization.yaml`
(kubectl's built-in `apply -k`, no separate binary) applies the whole steady-state
topology — namespaces, `kubernetes/*.yaml`, and all three ConfigMaps generated from
`params/*` and `tools/e2e/sql/init.sql` — in one command instead of one `kubectl apply
-f`/`create configmap` per file. It lives at the `tools/` root, not here: kustomize
refuses by design to read a file outside its own root, and `tools/e2e/sql/init.sql`
shares no closer common ancestor with this directory than `tools/` itself. See its own
header for what it deliberately leaves out and why.

### Why not Helm

Helm charts earn their keep when the same application gets deployed with different
values across environments — that's not this. This harness has exactly one topology,
identical on every run: one robot, one edge, one hub, no per-site or per-robot
variation to parameterize. A Helm chart here would template a variance that doesn't
exist, and add a new tool this repo doesn't otherwise depend on (its own pinned
installer action, `Chart.yaml`/`values.yaml` conventions to maintain). Kustomize
already ships inside `kubectl`. The real fleet deployment tooling
(`deploy/robot/`, and whatever comes for edge/hub) is a different problem — many robots
and sites genuinely needing different config is exactly what
Helm-plus-per-site-Kustomize-overlays is for — but that's a separate design question,
tracked as #466, not this harness.

## Docker dependency

kind is Docker-based — it runs each cluster node as a Docker container, and its Calico
guide, which this harness follows, is documented and tested against Docker. Podman does
have experimental kind support (`KIND_EXPERIMENTAL_PROVIDER=podman`), but #452 chose not
to depend on that: this harness is throwaway CI test infrastructure, not something DC
ships, so it uses real Docker rather than an experimental path this repo doesn't
otherwise rely on. Building and shipping DC itself stays on Podman, unchanged
(CLAUDE.md "Containers: Podman, not Docker") — the load step deliberately routes the
`dc-ros` image through `podman save` / `kind load image-archive` rather than
`kind load docker-image`, so Podman stays the tool that ever touches a DC image; Docker's
only job is running the kind nodes themselves. GitHub-hosted `ubuntu-latest` runners
ship Docker preinstalled, so CI needs no extra setup step for it.

`kind` and `kubectl` themselves come from
[`helm/kind-action`](https://github.com/helm/kind-action) with `install_only: true` —
pinned versions, checksum-verified against their own published sha256sums, no custom
Containerfile to maintain for two binaries a well-maintained action already installs
correctly. `install_only` is required, not optional: the action otherwise always runs
`kind create cluster --wait=<duration>`, which blocks for node readiness — impossible
here before Calico is installed, since `disableDefaultCNI` leaves every node NotReady
until then. Cluster creation stays a plain `kind create cluster` (no `--wait`), same as
a local run. A local run needs `kind`/`kubectl` on `PATH` some other way (a package
manager, or `kind`'s own install docs) — nothing here pins that for you the way CI's
step does, so match `kind`/`kubectl`'s versions above by hand if reproducing a failure
exactly matters.
