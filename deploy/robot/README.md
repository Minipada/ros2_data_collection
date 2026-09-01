# Robot topology renderings (#450)

The robot tier of the fleet architecture (epic #440, `docs/adr/0015-split-deployment-topology.md`)
is three containers — `dc-ros` (every ROS node plus the Bridge, unmanaged-shipper mode),
`vector` (the unmodified upstream Shipper image) and `dc-uploader` (the standalone Uploader
process, `docs/adr/0014-uploader-runs-as-its-own-process.md`) — sharing a config volume, a
Shipper buffer volume, and an upload-intent-queue-plus-Files volume, on one network. This
directory describes that same topology three ways, so a site deploys it with whatever it
already runs, without the topology itself becoming three different decisions:

| Rendering | File | What it is for |
|---|---|---|
| Compose | `compose.yaml` | `podman compose up` — the fastest way to run the robot tier on a bare Podman host with no systemd unit management and no cluster. |
| Podman Quadlet | `quadlet/*.container`, `quadlet/*.network` | Drop into `~/.config/containers/systemd/` (rootless) for a systemd-supervised robot: `systemctl --user` restart policy, boot ordering and `journalctl` logging, matching this repo's Podman-not-Docker convention (`CLAUDE.md`) and the `~/dev/monorepo` Quadlet reference (`server/docs/adr/002-rootless-podman-quadlet.md`). |
| Kubernetes | `kubernetes/robot-pod.yaml` | A single `Pod` with three containers, for a site that already runs a Kubernetes edge (epic #440 scenario 3) or wants the same shape locally for iteration (#451's k3d loop, #452's kind NetworkPolicy proof). |

All three point at the same published images
(`ghcr.io/minipada/ros2_data_collection/dc-ros`, `.../dc-uploader`,
`docker.io/timberio/vector`) built by `.github/workflows/ci.yaml`'s
`build-dc-ros-image`/`build-dc-uploader-image` jobs — this ticket adds no new build or
publish path (building and shipping still goes through Podman, unchanged).

## What differs from `tools/e2e/compose.split.yaml`

`compose.split.yaml` is the *test* rendering of the three-container topology (#447): it adds
Postgres and RustFS containers standing in for reachable Destinations, so the zero-loss E2E
harness can assert against them directly. The renderings here are the *deployable* robot
tier for the fleet shape (epic #440 scenario 3): no stand-in Destinations ship with it,
because in that scenario the robot has no internet access and no Destinations to reach
directly — Records go out through the blessed `vector` Destination type to an edge
aggregator, and Files go to edge-local object storage. Both are placeholder hostnames in
`params/robot_params.yaml`
(`edge.site.example`) that a real deployment overrides — the edge aggregator and its object
storage are deployment infrastructure outside this repository (epic #440's "Out of Scope"),
not something these manifests can stand up.

## Diagrams

`docs/` holds standalone, interactive diagrams of the three deployment shapes discussed
above — open any of them directly in a browser (dark/light, pan/zoom, guided views):

| Diagram | Shape |
|---|---|
| `docs/single-robot-quick-setup.html` | Scenario 1 (native/all-in-one): everything in one container, managed-shipper mode — fast local tests, no fleet needs. |
| `docs/containers-single-machine.html` | This directory's Compose/Quadlet rendering: the three containers above, direct to Postgres/S3 — testing the deployment architecture before fleet rollout. |
| `docs/fleet-topology.html` | Scenario 3: the fleet shape these renderings target — robots behind an edge tier, no direct internet access. |

Each includes a "Weaknesses" callout grounded in
[A Reference Architecture for Robot Fleets](https://blog.bensoussan.de/post/2026-08-25-a-reference-architecture-for-robot-fleets/):
why direct-to-destination shapes don't survive contact with a real fleet deployment
(security exposure, no edge buffer, sites that won't allow it on their network).

## Validating the renderings

```sh
./tools/ci/pre-commit/kubeconform.sh deploy/robot/kubernetes/robot-pod.yaml  # schema validation (also a pre-commit hook)
./deploy/robot/scripts/verify_kube_play.sh          # podman kube play reaches Ready
./deploy/robot/scripts/verify_network_isolation.sh  # podman networks prove the routing claim
```

Both scripts are runtime-free in the sense the issue means it: they need only Podman, no
Kubernetes cluster. `podman kube play` runs the Kubernetes Pod manifest directly, which is
possible only because the robot tier is a single Pod (see `kubernetes/robot-pod.yaml`'s
header). Cluster-backed validation of Kubernetes semantics themselves (Services, DNS, a real
scheduler, `NetworkPolicy` enforced by a CNI) is out of scope here — that is #451 (a fast k3d
inner loop) and #452 (a kind cluster with a policy-enforcing CNI, proving the same isolation
claim `verify_network_isolation.sh` proves at the Podman-network level, this time enforced by
`NetworkPolicy`).

## Network isolation

`verify_network_isolation.sh` creates an `--internal` Podman network for the robot
containers (no route out of the host) plus a second network joined only by a stand-in edge
container, and asserts:

- a robot-tier container cannot reach the public internet;
- a robot-tier container **can** reach the stand-in edge container (the one permitted
  outbound hop);
- a container outside the robot network cannot reach a robot-tier container's address.

This is a routing-level proof, not a stateful policy — Podman's `--internal` flag is a
missing default route, not a firewall rule per connection. The corresponding
policy-enforced proof, where a CNI actively denies rather than just not routing, is #452's
`NetworkPolicy` validation.
