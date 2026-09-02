.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* feat(dc_triggers): add Trigger plugin base, EdgeTrigger, and broadcast node
* feat(dc_measurements): move barcode/QR detection in-process via ZXing-C++ (`#123 <https://github.com/Minipada/ros2_data_collection/issues/123>`_)
* fix: condition_pub\_ is a normal publisher, could not publish when lifecycle
* fix+doc: list string equal url
* feat: add Camera measurement plugin
* feat: add StringStamped message which will be used to transmit data as JSON
* Contributors: David Bensoussan
