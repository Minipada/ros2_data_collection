.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* feat(e2e): three-container robot topology with dc-uploader as its own container (`#447 <https://github.com/Minipada/ros2_data_collection/issues/447>`_)
* feat(dc_bridge): extract the Uploader into its own dc_uploader process (`#446 <https://github.com/Minipada/ros2_data_collection/issues/446>`_)
* docs(dc_bridge): explain shipper.managed's deployment modes
* feat(dc_bridge): add unmanaged-shipper mode and atomic config write (`#444 <https://github.com/Minipada/ros2_data_collection/issues/444>`_)
* chore: remove progress.txt session log, rely on git history
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* feat(uploader): optional thumbnail/preview generation for image and video Files
* docs(raw): correct the size-cap advice and the rate-limiter description
* feat(dc_bridge): collect any ROS topic with a generic-subscription mode (`#227 <https://github.com/Minipada/ros2_data_collection/issues/227>`_)
* feat(dc_bringup): auto-launch dc_mcap_writer so it configures like a Destination
* fix(bridge): send Record timestamps at nanosecond resolution (`#308 <https://github.com/Minipada/ros2_data_collection/issues/308>`_)
* feat(dc_measurements): move barcode/QR detection in-process via ZXing-C++ (`#123 <https://github.com/Minipada/ros2_data_collection/issues/123>`_)
* feat(dc_bridge): files retention policy — bounded local storage for un-uploaded Files (`#267 <https://github.com/Minipada/ros2_data_collection/issues/267>`_)
* docs: use RustFS instead of MinIO in DC 2.0 example Destinations
* style: drop issue/ADR provenance from dc_bridge launch comments
* chore: demolish embedded Fluent Bit — remove fluent_bit\_*, dc_destinations, flb\_* layer (`#250 <https://github.com/Minipada/ros2_data_collection/issues/250>`_)
* docs: remove stale Rust-pilot references from current source and docs
* feat: Uploader — verified File uploads with metadata Records (ADR-0005) (`#248 <https://github.com/Minipada/ros2_data_collection/issues/248>`_)
* feat: deterministic launch ordering — Vector/Bridge first, ready gate, then activation (`#247 <https://github.com/Minipada/ros2_data_collection/issues/247>`_)
* docs: recommend RustFS over discontinued MinIO for self-hosted s3 destinations
* feat: complete blessed Destinations (s3/file/console) + dc.<tag> passthrough contract (`#246 <https://github.com/Minipada/ros2_data_collection/issues/246>`_)
* feat: add dc_bridge config renderer + PostgreSQL blessed destination (`#245 <https://github.com/Minipada/ros2_data_collection/issues/245>`_)
* feat: add dc_bridge tracer bullet — Records flow to Vector console (`#244 <https://github.com/Minipada/ros2_data_collection/issues/244>`_)
* feat: port C++ core to ROS 2 Jazzy, ignore Fluent Bit packages (`#242 <https://github.com/Minipada/ros2_data_collection/issues/242>`_)
* feat: add dc_lifecycle package, to start nodes in requested order (`#220 <https://github.com/Minipada/ros2_data_collection/issues/220>`_)
* fix: typo in dc_bringup parameter
* fix+doc: list string equal url
* feat: can set to start group node in launchfile
* feat: can set to start dc services from bringup
* cleanup
* cleanup: remove unused dep and cmake tests
* feat: add group node in bringup
* fix: dependencies dc_bringup
* feat: add pre-commit and fix all errors
* feat: start draw, save image and barcode detection service in dc_bringup
* feat: add tb3_simulation in dc_demos
* feat: add Destination node and include in uptime configuration and bringup
* feat: add bringup package to start measurement as lifecycle node
* Contributors: David Bensoussan
