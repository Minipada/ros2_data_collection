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

Run it end to end:

```sh
./tools/kind/scripts/run.sh
```

`scripts/run.sh`'s own header documents every env var (image refs, outage timing,
`DC_KIND_KEEP` to leave a failed cluster up for debugging).

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
placeholders — see each file's own header for what and why.

## Docker dependency

kind is Docker-based — it runs each cluster node as a Docker container, and its Calico
guide, which this harness follows, is documented and tested against Docker. Podman does
have experimental kind support (`KIND_EXPERIMENTAL_PROVIDER=podman`), but #452 chose not
to depend on that: this harness is throwaway CI test infrastructure, not something DC
ships, so it uses real Docker rather than an experimental path this repo doesn't
otherwise rely on. Building and shipping DC itself stays on Podman, unchanged
(CLAUDE.md "Containers: Podman, not Docker") — `scripts/load_images.sh` deliberately
routes the `dc-ros` image through `podman save` / `kind load image-archive` rather than
`kind load docker-image`, so Podman stays the tool that ever touches a DC image; Docker's
only job is running the kind nodes themselves. GitHub-hosted `ubuntu-latest` runners
ship Docker preinstalled, so CI needs no extra setup step for it.
