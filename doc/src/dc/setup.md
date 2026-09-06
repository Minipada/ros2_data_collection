# Setup

DC 2.0 ships as published container images by default — `podman compose` or `podman
run`, no ROS 2 toolchain to install locally. It is also an ordinary ROS 2 workspace if
you'd rather build natively: `rosdep install`, `colcon build`, done — no forked shipper
to compile, no Go toolchain, nothing needs root.

## Containerized

The default, recommended way to run DC — nothing to build, nothing but Podman required.

### Quick run

No build needed — the published `:jazzy` images run the all-in-one shape (every ROS
node, the Bridge and the Shipper in one `dc-ros` container) directly against a local
Postgres, with RustFS available alongside it for a files/S3 Destination. Isolated
network by default — `dc-ros` reaches the stores by container name, not the host's
network — so the commands below run from `deploy/robot/`.

**`podman compose`**, all three containers, one command — the fastest path if you don't
need to watch each container's own output separately:

```sh
podman compose -f compose.aio.yaml -f compose.isolated-network.yaml -f compose.local-destinations.yaml up
```

**`podman run`, one step at a time** — same three containers, split so each command
can be run and checked before the next, in its own terminal:

1. Network and volumes:

   ```sh
   podman network create dc_robot_net
   podman volume create dc_robot_pgdata
   podman volume create dc_robot_rustfs_data
   podman volume create dc_robot_aio_buffer
   ```

2. Postgres, pinned by digest — bump deliberately, check
   <https://hub.docker.com/_/postgres/tags?name=13> for a newer one:

   ```sh
   # PostgreSQL 13.23
   podman run --rm -it --network dc_robot_net --name dc_robot_postgres \
     -e POSTGRES_USER=dc -e POSTGRES_PASSWORD=password -e POSTGRES_DB=dc \
     -p 5432:5432 -v dc_robot_pgdata:/var/lib/postgresql/data \
     docker.io/library/postgres@sha256:4689940c683801b4ab839ab3b0a0a3555a5fe425371422310944e89eca7d8068
   ```

   Wait for `database system is ready to accept connections` before moving on.

3. RustFS, pinned by digest (matches `compose.local-destinations.yaml` — bump
   deliberately, check <https://hub.docker.com/r/rustfs/rustfs/tags> for the new one):

   ```sh
   # RustFS v1.0.0-beta.11
   podman run --rm -it --network dc_robot_net --name dc_robot_rustfs \
     -p 9000:9000 -v dc_robot_rustfs_data:/data \
     docker.io/rustfs/rustfs@sha256:84ce557a0245a06a9aae5516f55ee0f007fca78d41df356f419306fdc0cb168c
   ```

4. `dc-ros` — on the isolated network, `127.0.0.1` no longer reaches Postgres, so this
   needs a params override pointed at the hostname `postgres` instead of the image's
   own baked-in `127.0.0.1` default:

   ```sh
   podman run --rm -it --network dc_robot_net --name dc_robot_aio \
     -v dc_robot_aio_buffer:/root/.dc/buffer \
     -v "$(pwd)/params/aio_params_local.yaml:/opt/dc/dc_params.yaml:ro" \
     ghcr.io/minipada/ros2_data_collection/dc-ros:jazzy \
     dc_params_file:=/opt/dc/dc_params.yaml
   ```

5. Check Records are landing, from a fourth terminal:

   ```sh
   psql -h 127.0.0.1 -U dc -d dc -c 'select * from dc order by date desc limit 5;'
   ```

   Password `password`.

See
[`deploy/robot/README.md`](https://github.com/minipada/ros2_data_collection/blob/jazzy/deploy/robot/README.md)
for the three-container split topology, the host-network variant, and trying it against
a real store before fleet rollout.

### Building the workspace image

The repository builds a full workspace image with Podman — the same one CI uses:

```bash
IMAGE_TAG=dc-workspace:local ./tools/e2e/scripts/build.sh
```

`tools/e2e/scripts/test.sh` runs `colcon test` against that image, and
`tools/e2e/scripts/run.sh` drives the zero-loss end-to-end harness. See
[`tools/e2e/README.md`](https://github.com/minipada/ros2_data_collection/blob/jazzy/tools/e2e/README.md).

### Deployment renderings and a local Kubernetes loop

`deploy/robot/` describes the three-container robot tier (`dc-ros`, `vector`,
`dc-uploader` — see [Deployment modes](./destinations.md#deployment-modes-shippermanaged))
as Compose, Podman Quadlet and Kubernetes manifests, for whichever a site already runs. For
iterating on the Kubernetes rendering itself, a loop of plain `podman build`,
[k3d](https://k3d.io) and `kubectl` commands brings up a disposable local cluster in
seconds — no wrapper script, no registry, just the commands themselves. See
[`deploy/robot/README.md`](https://github.com/minipada/ros2_data_collection/blob/jazzy/deploy/robot/README.md)
for the full command sequence and what each step is for.

```admonish warning title="Development loop, not production parity"
k3d's default CNI does not enforce `NetworkPolicy`, so it cannot validate the fleet's
network-isolation claims. It is the fast inner loop only, deliberately not the
production-parity check.
```

## Native

For developing DC itself, or wherever containers aren't an option.

### Requirements

- ROS 2 Jazzy (`ros-jazzy-ros-base` or larger), on Ubuntu 24.04 or a Debian equivalent
- `colcon`, `rosdep`, `git`, `vcstool` (`python3-vcstool`), a C++17 compiler
- x86-64 or aarch64 — the architectures `vector_vendor` has a pinned Vector binary for

### Build

1. Clone into a workspace:

   ```bash
   mkdir -p ~/ws/src && cd ~/ws/src
   git clone https://github.com/minipada/ros2_data_collection.git
   ```

2. Pull in `vector_vendor` and `aws_sdk_vendor` (both their own repos — see
   [ADR-0002](./adr/0002-vector-as-default-shipper.md)'s amendment and
   [ADR-0012](./adr/0012-aws-sdk-vendor-flattened-source.md)), register DC's local rosdep
   rules (two header-only C++
   libraries upstream rosdistro has no key for), then resolve dependencies:

   ```bash
   cd ~/ws
   vcs import src < src/ros2_data_collection/ros2_data_collection.repos
   echo "yaml file://$PWD/src/ros2_data_collection/rosdep/dc.yaml" \
     | sudo tee /etc/ros/rosdep/sources.list.d/10-dc.list
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build:

   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build
   ```

That is the whole install. `colcon build` also runs `vector_vendor`, which fetches a
pinned, checksummed [Vector](https://vector.dev/) release tarball live — the external
**Shipper** the Bridge supervises at runtime
([ADR-0002](./adr/0002-vector-as-default-shipper.md)) — and `aws_sdk_vendor`, which
fetches and builds the AWS SDK for C++ (`core` + `s3`) `dc_uploader` uses
([ADR-0014](./adr/0014-uploader-runs-as-its-own-process.md)) live from `github.com/aws/aws-sdk-cpp`
at a pinned tag; both steps need network access
([ADR-0002](./adr/0002-vector-as-default-shipper.md),
[ADR-0012](./adr/0012-aws-sdk-vendor-flattened-source.md)), and `aws_sdk_vendor`'s takes
several minutes the first time.

### Python dependencies

Only some Measurement plugins (camera inspection, QR code detection) need Python
packages beyond what ROS 2 installs. `rosdep` covers the ones with rosdistro keys; for
the rest, [uv](https://docs.astral.sh/uv/) installs `pyproject.toml`'s pins into a
project virtualenv:

```bash
uv sync --no-dev   # drop --no-dev to add the tooling and the demo dashboard's packages
```

### Run

```bash
source install/setup.bash
ros2 launch dc_bringup dc_bringup.launch.py
```

The default parameters file (`dc_bringup/params/dc_params.yaml`) collects uptime and
writes it to a local PostgreSQL Destination. To run your own:

```bash
ros2 launch dc_bringup dc_bringup.launch.py dc_params_file:=/path/to/my_params.yaml
```

See [Configuration examples](./configuration_examples.md) for configurations you can
copy, and [Destinations](./destinations.md) for the full Bridge configuration contract.

#### Useful launch arguments

| Argument      | Default          | Description                                                |
| ------------- | ---------------- | ---------------------------------------------------------- |
| `dc_params_file` | `dc_params.yaml` | Parameters file for every DC node                       |
| `group_node`  | `False`          | Start the Group node (needed by any `group_server` config) |
| `namespace`   | `""`             | Top-level namespace                                        |
| `log_level`   | `info`           | Log level for the DC nodes                                 |
| `autostart`   | `True`           | Let the lifecycle manager configure and activate the nodes |
| `use_sim_time` | `False`         | Use simulation (Gazebo) clock — set `True` against a simulator, or TF lookups run on the wall clock while the sim publishes on its own clock and drift into "extrapolation" errors |
| `run_uploader` | `True`          | Launch `dc_uploader` ([ADR-0014](./adr/0014-uploader-runs-as-its-own-process.md)) from this process. Set `False` when it runs in its own container instead (the three-container split) |

### What starts, in what order

`dc_bringup.launch.py` brings the pipeline up deterministically
([ADR-0006](./adr/0006-bridge-outside-lifecycle-manager.md)): the Bridge
and its Shipper first, then a readiness gate, and only then the collection nodes. If the
Shipper never becomes ready, the launch shuts down loudly instead of collecting data
nowhere. [Data Pipeline](./data_pipeline.md) describes this in full.

### Advanced build options

```admonish tip title="Air-gapped or distro-packaged Vector"
Point the build at a Vector binary you already have instead of downloading one:

    colcon build --cmake-args -Dvector_path=/usr/bin/vector

The `VECTOR_PATH` environment variable does the same thing.
```

## Infrastructure

DC delivers to systems you run yourself. `tools/infrastructure/docker/` has compose
files that bring up PostgreSQL and RustFS (S3-compatible object storage) preconfigured
for the demos; see [Infrastructure setup](./infrastructure_setup.md).

## Issues

If you run into problems building DC, search the issue tracker on
[GitHub](https://github.com/minipada/ros2_data_collection/issues) and feel free to
[open a ticket](https://github.com/minipada/ros2_data_collection/issues/new).
