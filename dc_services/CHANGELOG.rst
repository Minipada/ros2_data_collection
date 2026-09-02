.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* build(tooling): run the hooks with prek and manage Python deps with uv
* fix(demos): drop dead Jazzy paths from the tb3_simulation demos and docs
* fix(demos): port the Nav2 QR-codes waypoint demo onto `#268 <https://github.com/Minipada/ros2_data_collection/issues/268>`_'s gz-sim simulation
* feat(dc_measurements): move barcode/QR detection in-process via ZXing-C++ (`#123 <https://github.com/Minipada/ros2_data_collection/issues/123>`_)
* feat+fix: add pre-commit for same version for all packages + fix
* fix+doc: list string equal url
* feat: add pre-commit and fix all errors
* fix: add zbar as dep to dc_services
* conf+feat: configure navigation for qrcodes demo
* feat: add service to detect barcodes on image
* feat: add service to save image
* feat: add service to draw on image
* Contributors: David Bensoussan
