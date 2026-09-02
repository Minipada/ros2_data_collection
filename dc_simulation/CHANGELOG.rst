.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* fix(dc_simulation): correct license metadata for third-party sim models (`#426 <https://github.com/Minipada/ros2_data_collection/issues/426>`_)
* chore: remove progress.txt session log, rely on git history
* feat(dc_simulation): publish battery state in the simulation (`#364 <https://github.com/Minipada/ros2_data_collection/issues/364>`_)
* fix(dc_simulation): replace the MOV.AI warehouse mesh with primitive geometry (`#357 <https://github.com/Minipada/ros2_data_collection/issues/357>`_)
* fix(dc_simulation): restore the MOV.AI warehouse model to its verbatim upstream
* feat(tools/sim): measure raw-mode collection volume against the simulated robot (`#326 <https://github.com/Minipada/ros2_data_collection/issues/326>`_)
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* fix(simulation): declare qrcodes.world's ambient light, verify `#61 <https://github.com/Minipada/ros2_data_collection/issues/61>`_ doesn't reproduce
* fix(simulation): measure, attribute and partially trim `#52 <https://github.com/Minipada/ros2_data_collection/issues/52>`_'s slow QR-codes demo startup
* fix(simulation): declare the cameras' field of view instead of inheriting it
* fix(demos): aim the QR-code demo's cameras at the codes it stops in front of (`#51 <https://github.com/Minipada/ros2_data_collection/issues/51>`_)
* fix(demos): drop dead Jazzy paths from the tb3_simulation demos and docs
* fix(simulation): spawn the robot, bridge joint_states, and retract a bad diagnosis (`#324 <https://github.com/Minipada/ros2_data_collection/issues/324>`_)
* fix(demos): port the Nav2 QR-codes waypoint demo onto `#268 <https://github.com/Minipada/ros2_data_collection/issues/268>`_'s gz-sim simulation
* fix(dc_simulation): resolve qrcode texture-loading errors from a real run
* fix(dc_demos): resolve build failures surfaced by the full-workspace build
* feat(dc_simulation): migrate warehouse sim off Gazebo Classic to Gazebo Harmonic (`#268 <https://github.com/Minipada/ros2_data_collection/issues/268>`_)
* [ImgBot] Optimize images (`#226 <https://github.com/Minipada/ros2_data_collection/issues/226>`_)
* fix: remove fireextanguisher
* cleanup: remove fireextanguisher item for simulation
* feat: add MonitorAndKeyboard item for simulation
* fix+doc: list string equal url
* feat: add pre-commit and fix all errors
* feat: add tool to generate qrcodes
* cleanup: remove unused MonitorAndKeyboard model
* refactor: move some simulation packages to a description one
* conf+feat: configure navigation for qrcodes demo
* fix: camera left points left and right right
* feat: add dc_demo with qrcode environment
* Contributors: David Bensoussan, imgbot[bot]
