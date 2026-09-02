.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* feat(dc_bridge): bless vector as a Destination type (`#443 <https://github.com/Minipada/ros2_data_collection/issues/443>`_)
* chore: remove progress.txt session log, rely on git history
* feat(dc_measurements): add the Fast DDS statistics Measurement (`#392 <https://github.com/Minipada/ros2_data_collection/issues/392>`_)
* feat(dc_simulation): publish battery state in the simulation (`#364 <https://github.com/Minipada/ros2_data_collection/issues/364>`_)
* feat(tools/infrastructure): utilisation, intervention rate and MTBF/MTTR KPI views (`#369 <https://github.com/Minipada/ros2_data_collection/issues/369>`_)
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* fix(tools/sim): read the QR ground truth the way the world file allows
* feat(dc_measurements): estimate the pose of detected codes (`#48 <https://github.com/Minipada/ros2_data_collection/issues/48>`_)
* build(tooling): run the hooks with prek and manage Python deps with uv
* refactor(demos): build the demo launch files from one shared description
* fix(simulation): declare the cameras' field of view instead of inheriting it
* fix(demos): aim the QR-code demo's cameras at the codes it stops in front of (`#51 <https://github.com/Minipada/ros2_data_collection/issues/51>`_)
* fix(demos): repoint the AWS warehouse demos at dc_simulation's world
* fix(demos): drop dead Jazzy paths from the tb3_simulation demos and docs
* fix(demos): port the Nav2 QR-codes waypoint demo onto `#268 <https://github.com/Minipada/ros2_data_collection/issues/268>`_'s gz-sim simulation
* fix(demos): rewrite Streamlit dashboard onto the flat postgres schema (`#272 <https://github.com/Minipada/ros2_data_collection/issues/272>`_)
* feat(uploader): optional thumbnail/preview generation for image and video Files
* feat(dc_bringup): auto-launch dc_mcap_writer so it configures like a Destination
* feat(dc_mcap_writer,docs): add MCAP as a passthrough Destination (`#210 <https://github.com/Minipada/ros2_data_collection/issues/210>`_)
* refactor(dc_measurements,dc_core): rename custom_params to custom_keys (`#186 <https://github.com/Minipada/ros2_data_collection/issues/186>`_)
* fix(demos): repair the InfluxDB passthrough demo, and cover passthrough in E2E
* docs(demos): add the Elasticsearch passthrough tutorial
* fix(bridge): send Record timestamps at nanosecond resolution (`#308 <https://github.com/Minipada/ros2_data_collection/issues/308>`_)
* feat(dc_measurements): move barcode/QR detection in-process via ZXing-C++ (`#123 <https://github.com/Minipada/ros2_data_collection/issues/123>`_)
* fix(dc_demos): resolve build failures surfaced by the full-workspace build
* feat(dc_simulation): migrate warehouse sim off Gazebo Classic to Gazebo Harmonic (`#268 <https://github.com/Minipada/ros2_data_collection/issues/268>`_)
* feat(dc_demos): rework demos onto the DC 2.0 Bridge/Vector pipeline (`#251 <https://github.com/Minipada/ros2_data_collection/issues/251>`_)
* conf: no email for streamlit (headless) (`#233 <https://github.com/Minipada/ros2_data_collection/issues/233>`_)
* fix: out plugins work (`#230 <https://github.com/Minipada/ros2_data_collection/issues/230>`_)
* [ImgBot] Optimize images (`#226 <https://github.com/Minipada/ros2_data_collection/issues/226>`_)
* feat: add grafana dashboard (`#209 <https://github.com/Minipada/ros2_data_collection/issues/209>`_)
* conf: update to reflect custom_params changes (`#191 <https://github.com/Minipada/ros2_data_collection/issues/191>`_)
* fix: run id set in measurement and not in destination (`#181 <https://github.com/Minipada/ros2_data_collection/issues/181>`_)
* conf: group_key is optional, empty by default (`#136 <https://github.com/Minipada/ros2_data_collection/issues/136>`_)
* doc: add turtlebot aws minio pgsql demos (`#45 <https://github.com/Minipada/ros2_data_collection/issues/45>`_)
* feat: set minio key and secret from environment in Streamlit (`#43 <https://github.com/Minipada/ros2_data_collection/issues/43>`_)
* feat: set step for streamlit slider dynamically (`#44 <https://github.com/Minipada/ros2_data_collection/issues/44>`_)
* refactor: rename qrcodes_dashboard to streamlit_dashboard (`#42 <https://github.com/Minipada/ros2_data_collection/issues/42>`_)
* feat: can select by time too (`#41 <https://github.com/Minipada/ros2_data_collection/issues/41>`_)
* feat: streamlit dashboard handles when there is no data or backend/storage not available (`#40 <https://github.com/Minipada/ros2_data_collection/issues/40>`_)
* feat: add simulation for tb3 pgsql and minio in aws warehouse (`#39 <https://github.com/Minipada/ros2_data_collection/issues/39>`_)
* feat: add streamlit dashboard (`#37 <https://github.com/Minipada/ros2_data_collection/issues/37>`_)
* fix: add missing includes (`#7 <https://github.com/Minipada/ros2_data_collection/issues/7>`_)
* doc+conf: qrcodes demo configuration works and doc updated
* fix: sim_time used in launch
* fix+conf: update hooks and fix package.xml
* fix: add aws warehouse package as dependency to dc_demos
* doc: add demo for custom uptime
* fix: add changes to tb3 stdout simulation launch and conf
* feat: measurement callback with failed validation has access to json data
* doc: add group memory_uptime demo
* doc: add uptime demo
* fix+doc: list string equal url
* feat: can set to start group node in launchfile
* fix: set composition to false tb3_qrcodes since it calls a service from a callback
* feat: can set to start dc services from bringup
* cleanup: fix syntax update_custom.json
* feat: add group demo
* feat: add demo on external measurement plugin and validation failed callback
* feat: add pre-commit and fix all errors
* feat+cleanup: add param to start dc and remove unused code
* refactor: move some simulation packages to a description one
* conf+feat: configure navigation for qrcodes demo
* feat: add Node that goes to all points in qr codes demo
* conf: collect from 2 cameras in demo, if not moving and only 1 measurement
* feat: add Camera measurement plugin
* feat: add dc_demo with qrcode environment
* conf: add map in tb3 demo
* conf: add cmd_vel in tb3 demo
* feat: add tb3_simulation in dc_demos
* feat: add Map measurement plugin
* doc: machine-id requires systemd package
* fix: need to cast to int to print bool
* feat: add newly created parameters in the uptime_stdout.yaml demo
* feat: add base structure for Fluent Bit destinations and flb stdout plugin
* feat: add Destination node and include in uptime configuration and bringup
* feat: add tags in the JSON for each measurement plugin. It will be used to match to destinations
* feat: add dc_demos repos with examples
* Contributors: David Bensoussan, imgbot[bot]
