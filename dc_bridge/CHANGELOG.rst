.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* feat(e2e): three-container robot topology with dc-uploader as its own container (`#447 <https://github.com/Minipada/ros2_data_collection/issues/447>`_)
* feat(dc_bridge): extract the Uploader into its own dc_uploader process (`#446 <https://github.com/Minipada/ros2_data_collection/issues/446>`_)
* style: trim overly long comments in the split-topology changes
* fix(dc_bridge): resolve vector_forward_host via DNS, not literal IP only
* feat(dc_bridge): add unmanaged-shipper mode and atomic config write (`#444 <https://github.com/Minipada/ros2_data_collection/issues/444>`_)
* docs(dc_bridge): annotate the vector render test fixture
* feat(dc_bridge): bless vector as a Destination type (`#443 <https://github.com/Minipada/ros2_data_collection/issues/443>`_)
* feat(dc_bridge): split shipper.data_dir into separate Shipper and Uploader directories (`#441 <https://github.com/Minipada/ros2_data_collection/issues/441>`_)
* fix(dc_bridge): carry Measurement custom keys onto File metadata Records (`#419 <https://github.com/Minipada/ros2_data_collection/issues/419>`_)
* test(dc_bridge,e2e): prove the pipeline under degraded network conditions (`#366 <https://github.com/Minipada/ros2_data_collection/issues/366>`_)
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* feat(dc_bridge): map incident_id onto a Postgres column in the rendered Vector config
* feat(dc_measurements): stage Files into a FileScratchRing while incident capture is armed (`#290 <https://github.com/Minipada/ros2_data_collection/issues/290>`_)
* feat(uploader): optional thumbnail/preview generation for image and video Files
* docs(raw): correct the size-cap advice and the rate-limiter description
* feat(dc_bridge): collect any ROS topic with a generic-subscription mode (`#227 <https://github.com/Minipada/ros2_data_collection/issues/227>`_)
* fix(bridge): send Record timestamps at nanosecond resolution (`#308 <https://github.com/Minipada/ros2_data_collection/issues/308>`_)
* test(dc_bridge): make forwarder reconnect/ack tests robust to CI scheduling jitter
* feat(dc_bridge): files retention policy — bounded local storage for un-uploaded Files (`#267 <https://github.com/Minipada/ros2_data_collection/issues/267>`_)
* feat(dc_bridge): confirmed delivery from Bridge to Vector via shipper ingest acks (`#266 <https://github.com/Minipada/ros2_data_collection/issues/266>`_)
* feat(dc_bridge): durable Uploader intent queue — File uploads survive Bridge restarts (`#265 <https://github.com/Minipada/ros2_data_collection/issues/265>`_)
* fix(dc_bridge): stop supervisor_test flaking on loaded CI runners
* docs: remove stale Rust-pilot references from current source and docs
* fix(e2e): make the reference workload actually flow end-to-end
* feat: DC 2.0 Bridge C++ Phase 2 — aws_sdk_vendor + File Uploader (ADR-0005)
* feat: DC 2.0 S8 harness + CI, and revert the Rust Bridge to C++ (Phase 1)
* refactor: extract status-row builders into an uploader::status submodule
* feat: Uploader — verified File uploads with metadata Records (ADR-0005) (`#248 <https://github.com/Minipada/ros2_data_collection/issues/248>`_)
* feat: deterministic launch ordering — Vector/Bridge first, ready gate, then activation (`#247 <https://github.com/Minipada/ros2_data_collection/issues/247>`_)
* test: store-backed e2e tests hard-fail with instructions instead of skipping
* docs: recommend RustFS over discontinued MinIO for self-hosted s3 destinations
* feat: complete blessed Destinations (s3/file/console) + dc.<tag> passthrough contract (`#246 <https://github.com/Minipada/ros2_data_collection/issues/246>`_)
* feat: add dc_bridge config renderer + PostgreSQL blessed destination (`#245 <https://github.com/Minipada/ros2_data_collection/issues/245>`_)
* feat: add colcon-test-integrated integration tests for dc_bridge; fix Vector orphaning on shutdown
* fix: add ros2_data_collection_jazzy.repos for dc_bridge's reproducible Rust builds
* feat: add dc_bridge tracer bullet — Records flow to Vector console (`#244 <https://github.com/Minipada/ros2_data_collection/issues/244>`_)
* Contributors: David Bensoussan
