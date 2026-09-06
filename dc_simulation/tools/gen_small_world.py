#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Generate qrcodes_small.world and its matching qrcodes_small.{pgm,yaml}.

A CI-only fixture (#473's sim-job flakiness fix): qrcodes.world's real 55x55 m, 239-model
warehouse runs at a 0.11-0.20 real-time factor (dc_simulation/README.md's own measurements),
which occasionally leaves Nav2's lifecycle bringup right at the edge of the sim job's 1800s
activation timeout. This generates a much smaller, prop-light room -- one open room, two
2-station pallet rows, no decorative clutter -- that proves the same things (spawn,
localization, one drive-to-goal, one QR read) without the warehouse's model count.

Deliberately not wired in as any package's default: qrcodes.world stays what
tb3_qrcodes.launch.py, the docs and the daily full-pass job (sim.yaml) use. Only the PR-gating
`sim` job's nav/waypoints/detect stages point at this fixture (tools/sim/scripts/run.sh).

World frame == map frame here (no WORLD_TO_MAP offset to keep in sync across files, unlike
qrcodes.world/qrcodes.pgm, which inherited an arbitrary offset from the licensed mesh they
replaced -- see dc_simulation/models/warehouse/model.sdf).

Run from the repo root: `python3 dc_simulation/tools/gen_small_world.py`. Checked-in output,
not regenerated at build time -- the same convention as gen_qr_code_model.py.
"""

import pathlib

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
SIM_DIR = REPO_ROOT / "dc_simulation"

RESOLUTION = 0.05

# Interior room, centred on the origin.
HALF_X = 4.0
HALF_Y = 6.0
WALL_T = 0.15
WALL_H = 3.0

# The pallet+bag stack's own model origin sits this far behind (x) and below (z) the QR
# code stuck to its face -- measured off qrcodes.world's europallet_bag_1_1 (-9.5, -0.2, 0)
# and europallet_bag_q_1_1 (-8.998293, -0.2, 0.328453) pairs, reused verbatim so the QR
# sits at the same height and forward offset as the real demo's.
QR_DX = 0.501707
QR_DZ = 0.328453

# Standoff proven correct against this camera's 60-degree FOV in qrcodes_nav.yaml's
# general_goal_checker comment: at 0.73 m, 0.1 + 0.73*tan(0.1) + 0.181 = 0.354 <=
# 0.73*tan(30deg) = 0.422.
STANDOFF = 0.73

YAW = 1.570796  # Both rows are driven the same direction (open floor, no aisle walls).

# (qr_face_x, y, qr_model) -- one entry per station, in visiting order.
STATIONS = [
    (-1.8, -3.0, "qrcode_0001"),
    (-1.8, -1.7, "qrcode_0002"),
    (1.8, 1.7, "qrcode_0003"),
    (1.8, 3.0, "qrcode_0004"),
]

# Spawn matches station 1's approach point, so the first leg is a straight line north.
SPAWN = (STATIONS[0][0] + STANDOFF, -4.3, 0.01, 0.0, 0.0, YAW)

# Map covers a 1 m margin beyond the walls on every side.
MAP_HALF_X = HALF_X + WALL_T + 1.0
MAP_HALF_Y = HALF_Y + WALL_T + 1.0


def robot_path_x(qr_face_x):
    """The robot always approaches from the +x side, same convention as qrcodes.world."""
    return qr_face_x + STANDOFF


def station_positions():
    """(x, y, yaw) tuples in visiting order -- the small world's camera_stations()."""
    return [(robot_path_x(qr_face_x), y, YAW) for qr_face_x, y, _ in STATIONS]


WALL_MATERIAL = """        <material>
          <ambient>0.75 0.75 0.72 1</ambient>
          <diffuse>0.75 0.75 0.72 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>"""


def _wall(name, x, y, sx, sy):
    z = WALL_H / 2
    return f"""      <collision name="{name}_collision">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} {WALL_H}</size>
          </box>
        </geometry>
      </collision>
      <visual name="{name}_visual">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} {WALL_H}</size>
          </box>
        </geometry>
{WALL_MATERIAL}
      </visual>"""


def _floor(sx, sy):
    # Top surface at z=0, matching the walls' bottom edge (z=0, see _wall()) and the
    # z=0/0.01 ground-level convention every station, robot spawn and map is built on.
    return f"""      <collision name="floor_collision">
        <pose>0 0 -0.05 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="floor_visual">
        <pose>0 0 -0.05 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} 0.1</size>
          </box>
        </geometry>
{WALL_MATERIAL}
      </visual>"""


def build_world():
    outer_x = HALF_X + WALL_T
    outer_y = HALF_Y + WALL_T
    walls = "\n\n".join(
        [
            _floor(2 * outer_x, 2 * outer_y),
            _wall("wall_west", -outer_x, 0, WALL_T, 2 * outer_y),
            _wall("wall_east", outer_x, 0, WALL_T, 2 * outer_y),
            _wall("wall_south", 0, -outer_y, 2 * outer_x, WALL_T),
            _wall("wall_north", 0, outer_y, 2 * outer_x, WALL_T),
        ]
    )

    stations_xml = []
    for index, (qr_face_x, y, qr_model) in enumerate(STATIONS, start=1):
        pallet_x = qr_face_x - QR_DX
        stations_xml.append(f"""    <model name="pallet_{index}">
      <static>1</static>
      <pose>{pallet_x} {y} 0 0 0 0</pose>
      <include>
        <uri>model://europallet</uri>
      </include>
      <include>
        <uri>model://bag</uri>
      </include>
    </model>

    <model name="pallet_qr_{index}">
      <static>1</static>
      <pose>{qr_face_x} {y} {QR_DZ} 0 0 0</pose>
      <include>
        <uri>model://{qr_model}</uri>
      </include>
    </model>""")

    return f"""<?xml version="1.0" encoding='utf-8' ?>

<!--
SPDX-FileCopyrightText: 2022-2026 David Bensoussan

SPDX-License-Identifier: MPL-2.0
-->

<sdf version="1.6">
  <world name="qrcodes_small">

    <!--
      Generated by dc_simulation/tools/gen_small_world.py. Do not hand-edit station
      placement here without regenerating maps/qrcodes_small.{{pgm,yaml}} to match, and
      dc_demos/dc_demos/qrcodes_small_waypoint_follower.py's STATIONS constant.
    -->

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="room">
      <static>true</static>
      <link name="room_base">
{walls}
      </link>
    </model>

{chr(10).join(stations_xml)}

  </world>
</sdf>
"""


def build_map():
    width = round(2 * MAP_HALF_X / RESOLUTION)
    height = round(2 * MAP_HALF_Y / RESOLUTION)
    origin_x = -MAP_HALF_X
    origin_y = -MAP_HALF_Y

    unknown, free, occupied = 205, 254, 0
    grid = [[unknown] * width for _ in range(height)]

    def to_px(x, y):
        return (round((x - origin_x) / RESOLUTION), round((y - origin_y) / RESOLUTION))

    def fill_rect(x0, y0, x1, y1, value):
        px0, py0 = to_px(x0, y0)
        px1, py1 = to_px(x1, y1)
        for row in range(max(0, min(py0, py1)), min(height, max(py0, py1) + 1)):
            for col in range(max(0, min(px0, px1)), min(width, max(px0, px1) + 1)):
                grid[row][col] = value

    outer_x = HALF_X + WALL_T
    outer_y = HALF_Y + WALL_T
    fill_rect(-outer_x, -outer_y, outer_x, outer_y, free)
    fill_rect(-outer_x, -outer_y, -HALF_X, outer_y, occupied)  # west wall
    fill_rect(HALF_X, -outer_y, outer_x, outer_y, occupied)  # east wall
    fill_rect(-outer_x, -outer_y, outer_x, -HALF_Y, occupied)  # south wall
    fill_rect(-outer_x, HALF_Y, outer_x, outer_y, occupied)  # north wall

    # Each pallet+bag stack, approximated as a 1.0 x 1.3 m footprint around its own
    # origin (europallet_bag's model origin, QR_DX behind the QR face).
    for qr_face_x, y, _ in STATIONS:
        pallet_x = qr_face_x - QR_DX
        fill_rect(pallet_x - 0.6, y - 0.65, pallet_x + 0.4, y + 0.65, occupied)

    # PGM rows run top-to-bottom, i.e. decreasing y (map_server convention, matching
    # qrcodes.pgm) -- row 0 of the image is the map's northmost row.
    rows = list(reversed(grid))
    header = f"P5\n{width} {height}\n255\n".encode()
    body = bytes(value for row in rows for value in row)
    return header + body, width, height, origin_x, origin_y


def main():
    world_path = SIM_DIR / "worlds" / "qrcodes_small.world"
    world_path.write_text(build_world())
    print(f"wrote {world_path}")

    pgm_bytes, width, height, origin_x, origin_y = build_map()
    pgm_path = SIM_DIR / "maps" / "qrcodes_small.pgm"
    pgm_path.write_bytes(pgm_bytes)
    print(f"wrote {pgm_path} ({width}x{height})")

    yaml_path = SIM_DIR / "maps" / "qrcodes_small.yaml"
    yaml_path.write_text(f"""image: qrcodes_small.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{origin_x}, {origin_y}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    print(f"wrote {yaml_path}")

    print("\nstation_positions() ==", station_positions())
    print("SPAWN ==", SPAWN)


if __name__ == "__main__":
    main()
