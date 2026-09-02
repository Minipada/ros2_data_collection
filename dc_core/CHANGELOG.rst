.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* feat(dc_measurements): rate-limit the IncidentReleaser buffered-window release (`#289 <https://github.com/Minipada/ros2_data_collection/issues/289>`_)
* feat(dc_measurements): add IncidentReleaser state machine with post-roll, cooldown and auto re-arm
* feat(dc_measurements): wire RecordRingBuffer into Measurement with flush-triggered release
* feat(dc_triggers): add Trigger plugin base, EdgeTrigger, and broadcast node
* refactor(core): extract the shared ConditionSet evaluator out of Measurement (`#284 <https://github.com/Minipada/ros2_data_collection/issues/284>`_)
* refactor(dc_measurements,dc_core): rename custom_params to custom_keys (`#186 <https://github.com/Minipada/ros2_data_collection/issues/186>`_)
* feat(dc_measurements): add gate_condition arming latch for Measurements (`#124 <https://github.com/Minipada/ros2_data_collection/issues/124>`_)
* chore: demolish embedded Fluent Bit — remove fluent_bit\_*, dc_destinations, flb\_* layer (`#250 <https://github.com/Minipada/ros2_data_collection/issues/250>`_)
* feat: can nest and flatten sample  (`#206 <https://github.com/Minipada/ros2_data_collection/issues/206>`_)
* feat: can override custom key or not (`#189 <https://github.com/Minipada/ros2_data_collection/issues/189>`_)
* fix: move custom parameters to measurement node (`#185 <https://github.com/Minipada/ros2_data_collection/issues/185>`_)
* fix: run id set in measurement and not in destination (`#181 <https://github.com/Minipada/ros2_data_collection/issues/181>`_)
* fix: plugins could not be loaded (`#9 <https://github.com/Minipada/ros2_data_collection/issues/9>`_)
* ci: add ci job (`#1 <https://github.com/Minipada/ros2_data_collection/issues/1>`_)
* fix: remove callback groups, not used
* fix: publishActive declared
* fix: typo
* doc: add uptime demo
* fix+doc: list string equal url
* fix: same_as_previous does not subscribe, data passed directly
* feat: add pre-commit and fix all errors
* conf+feat: configure navigation for qrcodes demo
* feat: add Map measurement plugin
* feat: add  custom_params in destinations
* feat: add run id and filter in destination
* refactor: filter initialization and flb debug moved
* feat: add base structure for Fluent Bit destinations and flb stdout plugin
* feat: can set max measurements when condition is enabled
* feat: add Destination node and include in uptime configuration and bringup
* feat: add Conditions, plugins to enable collect on condition. Add moving condition
* feat: add parameter to include parameter name in JSON measurement
* feat: add init_max_measurements to limit measurements to a certain number when starting the node
* feat add parameter init_collect for measurements to collect without waiting first tick
* feat: add tags in the JSON for each measurement plugin. It will be used to match to destinations
* feat: Add group key, used to group data in the future group node
* feat: add possibility to validate JSON schema and not send if not validated
* feat: add basics for the measurement class in new dc_core package
* feat: add dc_common package which has common class for all future dc packages
* Contributors: David Bensoussan
