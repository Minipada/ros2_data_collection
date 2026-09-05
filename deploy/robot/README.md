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

## Helm chart + Kustomize overlays per site/robot (#466)

`kubernetes/robot-pod.yaml` above is one fixed topology — correct for the runtime-free
check, k3d (#451) and kind (#452) loops, but a real fleet's robots genuinely differ:
edge aggregator address, robot identity, resource limits, image tags, credentials.
`helm/dc-robot/` is a chart that parameterizes exactly that (`docs/adr/0016`); its
defaults reproduce `kubernetes/robot-pod.yaml` + `params/robot_params.yaml` field for
field, so `helm template` with no overrides passes the same `kubeconform` check that
file does — this is a parameterized rendering path alongside the other three, not a
replacement of any of them. `helm/overlays/site-a/` is a reference Kustomize overlay
showing per-site variation composed on top: a site's own `values-<site>.yaml` (robot
identity, edge address — the chart's values surface) plus a patch for whatever a chart
value shouldn't cover (here, a `nodeSelector` pinning the Pod to that site's labeled
edge-adjacent node).

```sh
# Render the chart alone, with defaults (same shape as kubernetes/robot-pod.yaml):
helm template dc-robot deploy/robot/helm/dc-robot

# Render a site's overlay — chart values + Kustomize patches, in one command
# (--enable-helm is required: kustomize's Helm inflator is opt-in; --load-restrictor
# LoadRestrictionsNone is required because the overlay's base, the chart, lives outside
# its own directory tree, same reason k3d/kustomization.yaml needs it):
kubectl kustomize --enable-helm --load-restrictor LoadRestrictionsNone \
  --helm-command="$(which helm)" deploy/robot/helm/overlays/site-a

# Schema-validate either rendering the same way as kubernetes/robot-pod.yaml
# (kubeconform.sh resolves its argument against the repo root, so render to a path
# inside it, not /tmp):
kubectl kustomize --enable-helm --load-restrictor LoadRestrictionsNone \
  --helm-command="$(which helm)" deploy/robot/helm/overlays/site-a > deploy/robot/helm/.rendered-site-a.yaml
./tools/ci/pre-commit/kubeconform.sh deploy/robot/helm/.rendered-site-a.yaml
rm deploy/robot/helm/.rendered-site-a.yaml
```

A site with N robots installs N releases (one per robot, each its own
`values-<robot>.yaml`/overlay), never one release templating N robots internally — see
`docs/adr/0016` for the full design and rejected alternatives, including why this is a
different call from `tools/kind/README.md`'s "Why not Helm" (a fixed one-topology CI
harness, not a real fleet).

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

## Trying it against a real store (`compose.local-destinations.yaml`)

`compose.yaml`'s `dc-ros` ships `params/robot_params.yaml`, which points its only
Destination at `edge.site.example` — a placeholder, since the robot tier has no
internet access in the fleet shape it renders (see "What differs from
`tools/e2e/compose.split.yaml`" above). To try the same three-container topology end to
end before fleet rollout — the scenario `docs/containers-single-machine.html` describes
— stack `compose.local-destinations.yaml` (a real Postgres + RustFS pair) on top and
point `dc-ros` at `params/robot_params_local.yaml` instead, which swaps that one
Destination block for a direct Postgres connection to it:

```sh
DC_ROBOT_PARAMS=./params/robot_params_local.yaml \
  podman compose -f compose.yaml -f compose.local-destinations.yaml up
```

Same topology, same images, same volumes — only the rendered Vector config differs
(ADR-0015). `psql -h 127.0.0.1 -U dc -d dc -c 'select * from dc order by date desc
limit 5;'` (password `password`) checks Records are landing.

### Running it by hand, one terminal per container

`podman compose` backgrounds every container under one command; to watch each
container's own log stream directly instead, run this same stack as five plain
`podman run` commands, one per terminal — no compose file involved, same images,
volumes and network names `podman compose config` resolves them to above. Set up the
shared network and volumes once:

```sh
podman network create dc_robot_net
podman volume create dc_robot_pgdata
podman volume create dc_robot_rustfs_data
podman volume create dc_robot_uploader
podman volume create dc_robot_config
podman volume create dc_robot_buffer
```

Then, one command per terminal (any order — nothing here uses `depends_on`; see this
file's own compose.yaml header for why that's safe):

```sh
# Terminal 1 — Postgres
podman run --rm -it --name dc_robot_postgres --network dc_robot_net \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -p 5432:5432 -v dc_robot_pgdata:/var/lib/postgresql/data \
  docker.io/library/postgres:13

# Terminal 2 — RustFS
podman run --rm -it --name dc_robot_rustfs --network dc_robot_net \
  -p 9000:9000 -v dc_robot_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c

# Terminal 3 — vector (the Shipper compose.yaml's dc-ros renders config for)
podman run --rm -it --name dc_robot_vector --network dc_robot_net \
  -v dc_robot_buffer:/var/lib/vector -v dc_robot_config:/etc/dc/shipper:ro \
  --entrypoint /bin/sh docker.io/timberio/vector:0.57.0-debian -c \
  'trap exit TERM; until [ -f /etc/dc/shipper/vector.toml ]; do sleep 0.2; done; exec vector --config /etc/dc/shipper/vector.toml --watch-config'

# Terminal 4 — dc-ros (unmanaged shipper, run_uploader:=false — dc-uploader is its own container below)
podman run --rm -it --name dc_robot_dc_ros --network dc_robot_net \
  -v dc_robot_uploader:/root/.dc/robot/uploader -v dc_robot_config:/etc/dc/shipper \
  -v "$(pwd)/params/robot_params_local.yaml:/opt/dc/robot_params.yaml:ro" \
  ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy \
  dc_params_file:=/opt/dc/robot_params.yaml run_uploader:=false

# Terminal 5 — dc-uploader (still points at the edge.site.example S3 placeholder —
# robot_params_local.yaml only swaps the Records/Postgres path, see
# compose.local-destinations.yaml's rustfs comment for wiring Files to rustfs instead)
podman run --rm -it --name dc_robot_dc_uploader --network dc_robot_net \
  -e DC_UPLOADER_STORAGE_NAME=edge \
  -e DC_UPLOADER_QUEUE_DIR=/root/.dc/robot/uploader/queue/upload \
  -e DC_UPLOADER_STATE_DIR=/root/.dc/robot/uploader/uploader \
  -e DC_UPLOADER_SHIPPER_HOST=vector -e DC_UPLOADER_SHIPPER_PORT=24224 \
  -e DC_UPLOADER_S3_BUCKET=dc-robot -e DC_UPLOADER_S3_ENDPOINT=http://edge.site.example:9000 \
  -e DC_UPLOADER_S3_ACCESS_KEY_ID=changeme -e DC_UPLOADER_S3_SECRET_ACCESS_KEY=changeme \
  -e DC_UPLOADER_DELETE_WHEN_SENT=true \
  -v dc_robot_uploader:/root/.dc/robot/uploader \
  ghcr.io/minipada/ros2_data_collection/dc-uploader:jazzy
```

Tear down: `Ctrl-C` each terminal, then `podman network rm dc_robot_net` and `podman
volume rm dc_robot_pgdata dc_robot_rustfs_data dc_robot_uploader dc_robot_config
dc_robot_buffer` if you're done with the data too.

## All-in-one, single container (`compose.aio.yaml`)

For the other first-class mode ADR-0001/ADR-0015 describe — one container, managed
shipper, no `vector`/`dc-uploader` split — `compose.aio.yaml` is an alternative base to
`compose.yaml`, not something stacked with it. It reuses
`compose.local-destinations.yaml` for its stores (Postgres + RustFS, same as above) and
picks its network with one of two small overlays instead of a params-file choice alone,
since the Bridge doesn't env-expand the `host` field a network change would otherwise need:

```sh
# Host network: every container reaches the others on 127.0.0.1, so this runs the
# dc-ros image with no params override at all — its own baked-in default
# (dc_bringup/params/dc_params.yaml) already points at 127.0.0.1:5432.
podman compose -f compose.aio.yaml -f compose.local-destinations.yaml -f compose.host-network.yaml up

# Isolated network: dc-ros joins the same named bridge network
# (dc_robot_net) as compose.yaml's split topology, reaching Postgres as the hostname
# "postgres" — needs params/aio_params_local.yaml, which compose.isolated-network.yaml
# mounts in automatically.
podman compose -f compose.aio.yaml -f compose.isolated-network.yaml -f compose.local-destinations.yaml up
```

`psql -h 127.0.0.1 -U dc -d dc -c 'select * from dc order by date desc limit 5;'`
(password `password`) checks Records either way.

### Running it by hand, one terminal per container

Three plain `podman run` commands instead of the two-file `compose.aio.yaml` +
`compose.local-destinations.yaml` stack — one per terminal, same images/volumes those
resolve to. Host network (matches `compose.host-network.yaml`):

```sh
podman volume create dc_robot_pgdata
podman volume create dc_robot_rustfs_data
podman volume create dc_robot_aio_buffer

# Terminal 1 — Postgres
podman run --rm -it --network host --name dc_robot_postgres \
  -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
  -v dc_robot_pgdata:/var/lib/postgresql/data \
  docker.io/library/postgres:13

# Terminal 2 — RustFS
podman run --rm -it --network host --name dc_robot_rustfs \
  -v dc_robot_rustfs_data:/data \
  docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c

# Terminal 3 — dc-ros: no params override needed, its baked-in default
# (dc_bringup/params/dc_params.yaml) already points at 127.0.0.1:5432.
podman run --rm -it --network host --name dc_robot_aio \
  -v dc_robot_aio_buffer:/root/.dc/buffer \
  ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy
```

Isolated network (matches `compose.isolated-network.yaml`) — same three terminals, but
create `dc_robot_net` first, drop every `--network host` for `--network dc_robot_net`,
publish the stores' ports explicitly since there's no host networking to expose them
through (`-p 5432:5432` / `-p 9000:9000`), and point dc-ros at
`params/aio_params_local.yaml` (`host: "postgres"`) instead of running it bare:

```sh
podman network create dc_robot_net
# Terminal 3, isolated variant:
podman run --rm -it --network dc_robot_net --name dc_robot_aio \
  -v dc_robot_aio_buffer:/root/.dc/buffer \
  -v "$(pwd)/params/aio_params_local.yaml:/opt/dc/dc_params.yaml:ro" \
  ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy \
  dc_params_file:=/opt/dc/dc_params.yaml
```

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
inner loop) and `tools/kind/` (#452: a kind cluster with Calico, a policy-enforcing CNI,
proving the same isolation claim `verify_network_isolation.sh` proves at the Podman-network
level, this time enforced by `NetworkPolicy` against a real robot/edge/hub topology — see
`tools/kind/README.md`).

## k3d development loop (#451)

`verify_kube_play.sh` above proves the manifest runs; it proves nothing about the manifest
*as Kubernetes*, because `podman kube play` has no scheduler, no Services, and no DNS. This
is the fast loop for that: a disposable single-node [k3d](https://k3d.io) cluster (k3d wraps
k3s) running `kubernetes/robot-pod.yaml` for real, so changes to the manifest can be
iterated against an actual scheduler in seconds rather than the minutes a full cluster
normally costs. There is no wrapper script — every step below is a plain `podman`/`k3d`/
`kubectl` command, so what the loop actually does is never hidden behind a script:

```sh
# 1. Get the images into local Podman storage. This example uses the images this
#    branch already publishes, tagged :jazzy — ghcr.io tags follow the branch name
#    here, never :latest, the same way ROS's own images are tagged by distro codename
#    (ros:jazzy-ros-base), never ros:latest. --entrypoint true skips actually launching
#    DC; this step exists only to fetch the image. To test your own local changes
#    instead, build containers/dc-ros/Containerfile the same way CI does (BASE_IMAGE
#    from tools/e2e/scripts/build.sh) and tag the result identically — nothing below
#    changes.
podman run --rm --entrypoint true ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy
podman run --rm --entrypoint true ghcr.io/minipada/ros2_data_collection/dc-uploader:jazzy

# 2. Create the cluster. --volume stages params/robot_params.yaml where
#    kubernetes/robot-pod.yaml's hostPath volume expects it, on the node itself
#    (mirrors verify_kube_play.sh's own host-staging step, just done by k3d instead of
#    `sudo install`). --k3s-arg works around k3s's kubelet hard-failing on
#    `open /dev/kmsg: operation not permitted` in restricted/rootless container
#    environments (WSL2, devcontainers, some CI runners) that don't expose /dev/kmsg
#    to the node container — harmless where /dev/kmsg is already available.
k3d cluster create dc-robot-dev \
  --volume "$(pwd)/deploy/robot/params:/etc/dc/robot@server:0" \
  --k3s-arg '--kubelet-arg=feature-gates=KubeletInUserNamespace=true@server:*'

# 3. Load the images straight into the cluster's containerd — no push, no further
#    pull, no registry reachable from inside the cluster. `podman save` refuses to
#    overwrite a tar it already wrote, hence `rm -f` on each iteration. These images
#    are large (~17 GB as published today, since the runtime stage isn't stripped down
#    from the build workspace) — this step needs that much free disk twice over
#    (Podman's own storage, plus the tar) and is the slow part of the loop; everything
#    after it is fast.
rm -f /tmp/dc-ros.tar && podman save ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy -o /tmp/dc-ros.tar && k3d image import /tmp/dc-ros.tar -c dc-robot-dev
rm -f /tmp/dc-uploader.tar && podman save ghcr.io/minipada/ros2_data_collection/dc-uploader:jazzy -o /tmp/dc-uploader.tar && k3d image import /tmp/dc-uploader.tar -c dc-robot-dev

# 4. Run the Pod. `k3d/kustomization.yaml` overlays kubernetes/robot-pod.yaml: pins
#    both images to :jazzy instead of the base manifest's :latest, and sets
#    imagePullPolicy: Never so kubelet uses what was just imported instead of reaching
#    out to ghcr.io itself. `apply -k` itself refuses a base outside its own directory,
#    which is why this pipes through `kustomize` (which accepts the override) instead.
kubectl kustomize deploy/robot/k3d --load-restrictor LoadRestrictionsNone | kubectl --context k3d-dc-robot-dev apply -f -

# 5. Watch it come up — same readiness signal verify_kube_play.sh checks.
kubectl --context k3d-dc-robot-dev get pod dc-robot --watch
kubectl --context k3d-dc-robot-dev logs dc-robot -c dc-ros --follow

# 6. Iterate: rebuild/re-pull an image, redo step 3, then recreate the Pod (Pod spec
#    fields — including the image — are immutable in place, so this needs a delete
#    first; cheap on a single-node cluster).
kubectl --context k3d-dc-robot-dev delete pod dc-robot
kubectl kustomize deploy/robot/k3d --load-restrictor LoadRestrictionsNone | kubectl --context k3d-dc-robot-dev apply -f -

# 7. Tear down.
k3d cluster delete dc-robot-dev
```

Measured against this loop: cluster create ~15-20s, an image-reload-and-recreate
iteration ~10s excluding the save/import in step 3 (dominated by the image's own size,
several minutes for a ~17 GB image), teardown ~2s. Once an image is loaded, iterating on
the manifest itself (steps 4-6 without redoing step 3) is the genuinely fast part.

This is **explicitly not** the production-parity check. k3d's default CNI (Flannel) does not
enforce `NetworkPolicy`, so it cannot stand in for `verify_network_isolation.sh`'s isolation
claim — that enforcement proof is #452's kind cluster with Calico/Cilium. Nothing here runs
in CI; it is a local inner-loop tool only, same spirit as `podman kube play` but with a real
scheduler underneath.

**Docker dependency, scoped**: k3d's nodes are Docker containers, so this is the one place
in this repository's container tooling that touches Docker instead of Podman (`CLAUDE.md`
"Containers: Podman, not Docker"). It is confined to `k3d/` (the overlay) and the commands
above — building and shipping `dc-ros`/`dc-uploader` still goes through Podman unchanged,
and nothing else in the repo assumes Docker is present. #452's kind cluster carries the
identical, equally-scoped exception, for the same reason (kind is Docker-based too).

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
