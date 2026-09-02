.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_util
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* refactor(dc_measurements,dc_util): unify parameter declaration on dc_util helpers (`#178 <https://github.com/Minipada/ros2_data_collection/issues/178>`_)
* feat(dc_measurements): move barcode/QR detection in-process via ZXing-C++ (`#123 <https://github.com/Minipada/ros2_data_collection/issues/123>`_)
* feat: deterministic launch ordering — Vector/Bridge first, ready gate, then activation (`#247 <https://github.com/Minipada/ros2_data_collection/issues/247>`_)
* feat: port C++ core to ROS 2 Jazzy, ignore Fluent Bit packages (`#242 <https://github.com/Minipada/ros2_data_collection/issues/242>`_)
* feat: add grafana dashboard (`#209 <https://github.com/Minipada/ros2_data_collection/issues/209>`_)
* feat: save camera image as base64 (`#205 <https://github.com/Minipada/ros2_data_collection/issues/205>`_)
* test: add test for os value (`#2 <https://github.com/Minipada/ros2_data_collection/issues/2>`_)
* ci: add ci job (`#1 <https://github.com/Minipada/ros2_data_collection/issues/1>`_)
* fix: camera measurement only send measurement if field non empty
* fix+conf: update hooks and fix package.xml
* feat: change permission format from integer to string(rwx/int)
* fix+doc: list string equal url
* feat: add same_as_previous condition
* feat: add pre-commit and fix all errors
* feat: add Camera measurement plugin
* feat: add Fluent Bit PostgreSQL destination plugin
* conf: add cmd_vel in tb3 demo
* feat: add Command Velocity measurement plugin
* feat: add Map measurement plugin
* feat: add run id and filter in destination
* feat: add base structure for Fluent Bit destinations and flb stdout plugin
* feat: add Conditions, plugins to enable collect on condition. Add moving condition
* feat: add Storage measurement plugin
* fix: add boost as dep for dc_util
* feat: add wrappers to load parameters easily in new dc_util package
* Contributors: David Bensoussan
