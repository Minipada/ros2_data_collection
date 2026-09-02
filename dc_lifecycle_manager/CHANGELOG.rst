.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_lifecycle_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* test(dc_lifecycle_manager): add gtest coverage for startup ordering, bonds and respawn
* feat: deterministic launch ordering — Vector/Bridge first, ready gate, then activation (`#247 <https://github.com/Minipada/ros2_data_collection/issues/247>`_)
* feat: port C++ core to ROS 2 Jazzy, ignore Fluent Bit packages (`#242 <https://github.com/Minipada/ros2_data_collection/issues/242>`_)
* feat: add dc_lifecycle package, to start nodes in requested order (`#220 <https://github.com/Minipada/ros2_data_collection/issues/220>`_)
* Contributors: David Bensoussan
