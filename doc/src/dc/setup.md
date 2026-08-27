# Setup

DC 2.0 is an ordinary ROS 2 workspace: `rosdep install`, `colcon build`, done. There is
no forked shipper to compile, no Go toolchain, and nothing needs root.

```admonish info
This is the DC 2.0 (`jazzy`) install. The `humble` line still embeds a patched Fluent Bit
and has a considerably longer setup; if you are coming from it, read the
[migration guide](./migration.md).
```

## Requirements

- ROS 2 Jazzy (`ros-jazzy-ros-base` or larger), on Ubuntu 24.04 or a Debian equivalent
- `colcon`, `rosdep`, `git`, a C++17 compiler
- x86-64 or aarch64 — the architectures `vector_vendor` has a pinned Vector binary for

## Build

```bash
# 1. Clone into a workspace
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/minipada/ros2_data_collection.git

# 2. Register DC's local rosdep rules (two header-only C++ libraries upstream
#    rosdistro has no key for), then resolve dependencies
cd ~/ws
echo "yaml file://$PWD/src/ros2_data_collection/rosdep/dc.yaml" \
  | sudo tee /etc/ros/rosdep/sources.list.d/10-dc.list
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
source /opt/ros/jazzy/setup.bash
colcon build
```

That is the whole install. `colcon build` also runs `vector_vendor`, which installs a
pinned, checksummed [Vector](https://vector.dev/) binary checked into the package — the
external **Shipper** the Bridge supervises at runtime (ADR-0002).

```admonish tip title="Air-gapped or distro-packaged Vector"
Point the build at a Vector binary you already have instead of the checked-in one:

    colcon build --cmake-args -Dvector_path=/usr/bin/vector

The `VECTOR_PATH` environment variable does the same thing.
```

```admonish warning title="Simulation demo packages"
`dc_demos` and `dc_simulation` still target Gazebo Classic, which Jazzy dropped. Until
they are ported (tracked in [#268](https://github.com/minipada/ros2_data_collection/issues/268)),
exclude them if their dependencies are not resolvable on your machine:

    touch src/ros2_data_collection/dc_demos/COLCON_IGNORE
    touch src/ros2_data_collection/dc_simulation/COLCON_IGNORE
```

### Python dependencies

Only some Measurement plugins (camera inspection, QR code detection) need Python
packages beyond what ROS 2 installs. `rosdep` covers the ones with rosdistro keys; for
the rest, [uv](https://docs.astral.sh/uv/) installs `pyproject.toml`'s pins into a
project virtualenv:

```bash
uv sync --no-dev   # drop --no-dev to add the tooling and the demo dashboard's packages
```

## Run

```bash
source install/setup.bash
ros2 launch dc_bringup dc_bringup.launch.py
```

The default parameters file (`dc_bringup/params/dc_params.yaml`) collects uptime and
writes it to a local PostgreSQL Destination. To run your own:

```bash
ros2 launch dc_bringup dc_bringup.launch.py params_file:=/path/to/my_params.yaml
```

See [Configuration examples](./configuration_examples.md) for configurations you can
copy, and [Destinations](./destinations.md) for the full Bridge configuration contract.

### Useful launch arguments

| Argument      | Default          | Description                                                |
| ------------- | ---------------- | ---------------------------------------------------------- |
| `params_file` | `dc_params.yaml` | Parameters file for every DC node                          |
| `group_node`  | `False`          | Start the Group node (needed by any `group_server` config) |
| `namespace`   | `""`             | Top-level namespace                                        |
| `log_level`   | `info`           | Log level for the DC nodes                                 |
| `autostart`   | `True`           | Let the lifecycle manager configure and activate the nodes |

### What starts, in what order

`dc_bringup.launch.py` brings the pipeline up deterministically (ADR-0006): the Bridge
and its Shipper first, then a readiness gate, and only then the collection nodes. If the
Shipper never becomes ready, the launch shuts down loudly instead of collecting data
nowhere. [Data Pipeline](./data_pipeline.md) describes this in full.

## Containers

The repository builds a full workspace image with Podman — the same one CI uses:

```bash
IMAGE_TAG=dc-workspace:local ./tools/e2e/scripts/build.sh
```

`tools/e2e/scripts/test.sh` runs `colcon test` against that image, and
`tools/e2e/scripts/run.sh` drives the zero-loss end-to-end harness. See
[`tools/e2e/README.md`](https://github.com/minipada/ros2_data_collection/blob/jazzy/tools/e2e/README.md).

## Infrastructure

DC delivers to systems you run yourself. `tools/infrastructure/docker/` has compose
files that bring up PostgreSQL and RustFS (S3-compatible object storage) preconfigured
for the demos; see [Infrastructure setup](./infrastructure_setup.md).

## Issues

If you run into problems building DC, search the issue tracker on
[GitHub](https://github.com/minipada/ros2_data_collection/issues) and feel free to
[open a ticket](https://github.com/minipada/ros2_data_collection/issues/new).
