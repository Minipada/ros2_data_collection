.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_measurements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* fix(dc_measurements): assert on lifecycle state, not a thrown exception
* style: fix clang-format violations in test_measurement_dummy.cpp
* feat(dc_measurements): resolve robot_name from literal, hostname, or file (`#442 <https://github.com/Minipada/ros2_data_collection/issues/442>`_)
* style(dc_measurements): apply clang-format to Mission adapter changes
* refactor(dc_measurements): share the Mission Record JSON Schema via a loader-backed $ref
* refactor(dc_measurements): deepen the Mission adapter lifecycle and pending-record queue
* fix(dc_bridge): carry Measurement custom keys onto File metadata Records (`#419 <https://github.com/Minipada/ros2_data_collection/issues/419>`_)
* fix(dc_measurements): keep PushServer's io_context alive between sends
* fix(dc_measurements): init rclcpp in the mission_open_rmf integration test
* feat(dc_measurements): add the Mission Measurement Open-RMF adapter (`#391 <https://github.com/Minipada/ros2_data_collection/issues/391>`_)
* feat(dc_measurements): add the Mission Measurement nav2 NavigateToPose adapter (`#387 <https://github.com/Minipada/ros2_data_collection/issues/387>`_)
* feat(dc_measurements): add the slam_toolbox quality Measurement (`#407 <https://github.com/Minipada/ros2_data_collection/issues/407>`_)
* fix(dc_measurements): rename fastdds_stats' processes field to process_names
* feat(dc_measurements): add the Fast DDS statistics Measurement (`#392 <https://github.com/Minipada/ros2_data_collection/issues/392>`_)
* fix(dc_measurements): use abort(), not canceled(), for the PREEMPTED test case
* feat(dc_measurements): add the Manipulation Measurement, a MoveIt MoveGroup adapter (`#393 <https://github.com/Minipada/ros2_data_collection/issues/393>`_)
* feat(dc_measurements): add the ros2_control status Measurement
* fix(dc_measurements): use GoalStatusArray.status_list, not .status (`#389 <https://github.com/Minipada/ros2_data_collection/issues/389>`_)
* fix(dc_measurements): use MissedWaypoint, not WaypointStatus, for FollowWaypoints (`#389 <https://github.com/Minipada/ros2_data_collection/issues/389>`_)
* feat(measurements): add mission_nav2_follow_waypoints Measurement (`#389 <https://github.com/Minipada/ros2_data_collection/issues/389>`_)
* fix(dc_measurements): remove redundant goal_handle->execute() in test fixture
* fix(dc_measurements): drop waypoint_statuses, absent from Jazzy's nav2_msgs
* feat(dc_measurements): add Mission Measurement NavigateThroughPoses adapter (`#388 <https://github.com/Minipada/ros2_data_collection/issues/388>`_)
* fix(dc_measurements): establish an OK baseline before a fault's first transition in tests
* style(dc_measurements): apply clang-format to the fault Measurement
* feat(dc_measurements): add the fault Measurement, raise/change/clear from diagnostics (`#365 <https://github.com/Minipada/ros2_data_collection/issues/365>`_)
* feat(dc_measurements): intervention Measurement for human takeovers (`#373 <https://github.com/Minipada/ros2_data_collection/issues/373>`_)
* feat(dc_measurements): add the battery Measurement, with charge sessions and cycles
* test(dc_measurements): cover one FlushEvent releasing several Measurements (`#345 <https://github.com/Minipada/ros2_data_collection/issues/345>`_)
* fix(dc_measurements): retract the QR in-plane rotation limitation (`#297 <https://github.com/Minipada/ros2_data_collection/issues/297>`_)
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* feat(dc_measurements): estimate the pose of detected codes (`#48 <https://github.com/Minipada/ros2_data_collection/issues/48>`_)
* fix(dc_measurements): constrain the Thermal Record's JSON schema
* test(dc_measurements): pace the released window in the File staging test
* feat(dc_measurements): stage Files into a FileScratchRing while incident capture is armed (`#290 <https://github.com/Minipada/ros2_data_collection/issues/290>`_)
* feat(dc_measurements): rate-limit the IncidentReleaser buffered-window release (`#289 <https://github.com/Minipada/ros2_data_collection/issues/289>`_)
* feat(dc_measurements): add IncidentReleaser state machine with post-roll, cooldown and auto re-arm
* feat(dc_measurements): wire RecordRingBuffer into Measurement with flush-triggered release
* refactor(core): extract the shared ConditionSet evaluator out of Measurement (`#284 <https://github.com/Minipada/ros2_data_collection/issues/284>`_)
* fix(simulation): spawn the robot, bridge joint_states, and retract a bad diagnosis (`#324 <https://github.com/Minipada/ros2_data_collection/issues/324>`_)
* fix(measurements): assert no-data measurements never publish, not that they publish empty
* fix(measurements): link yaml-cpp/ffmpeg into plugins that dlopen-failed in CI
* test(measurements): add gtest coverage for the remaining 11 measurement plugins
* test(conditions): add gtest coverage for the remaining condition plugins
* fix(conditions): fix 8 bugs found while adding condition-plugin test coverage
* test(conditions): add gtest coverage for the double_equal condition plugin
* test(measurements): add gtest coverage for the camera plugin
* refactor(dc_measurements,dc_core): rename custom_params to custom_keys (`#186 <https://github.com/Minipada/ros2_data_collection/issues/186>`_)
* refactor(dc_measurements,dc_util): unify parameter declaration on dc_util helpers (`#178 <https://github.com/Minipada/ros2_data_collection/issues/178>`_)
* feat(dc_measurements): add gate_condition arming latch for Measurements (`#124 <https://github.com/Minipada/ros2_data_collection/issues/124>`_)
* feat(dc_measurements): move barcode/QR detection in-process via ZXing-C++ (`#123 <https://github.com/Minipada/ros2_data_collection/issues/123>`_)
* feat(dc_measurements): add DrivingType measurement plugin
* feat(dc_measurements): add Thermal measurement plugin (`#100 <https://github.com/Minipada/ros2_data_collection/issues/100>`_)
* fix(dc_measurements): use a second, independently-named node for the Random seed test
* feat(dc_measurements): add Random measurement plugin (`#88 <https://github.com/Minipada/ros2_data_collection/issues/88>`_)
* fix(dc_measurements): detect serial hangup via poll(POLLHUP), not read()==0
* fix(dc_measurements): allocate the reconnect test's pty directly, not via a second socat
* fix(dc_measurements): use posix_spawn for the serial test's socat subprocess
* fix(dc_measurements): don't treat VMIN=0 termios read()=0 as serial disconnect
* feat(dc_measurements): add Serial Interface measurement plugin (`#60 <https://github.com/Minipada/ros2_data_collection/issues/60>`_)
* feat(dc_measurements): add Diagnostics measurement plugin (`#49 <https://github.com/Minipada/ros2_data_collection/issues/49>`_)
* chore: demolish embedded Fluent Bit — remove fluent_bit\_*, dc_destinations, flb\_* layer (`#250 <https://github.com/Minipada/ros2_data_collection/issues/250>`_)
* feat: deterministic launch ordering — Vector/Bridge first, ready gate, then activation (`#247 <https://github.com/Minipada/ros2_data_collection/issues/247>`_)
* feat: port C++ core to ROS 2 Jazzy, ignore Fluent Bit packages (`#242 <https://github.com/Minipada/ros2_data_collection/issues/242>`_)
* feat: stop measurement if it reached limit (`#231 <https://github.com/Minipada/ros2_data_collection/issues/231>`_)
* feat: add dc_lifecycle package, to start nodes in requested order (`#220 <https://github.com/Minipada/ros2_data_collection/issues/220>`_)
* fix: position plugin uses right timeout parameter (`#218 <https://github.com/Minipada/ros2_data_collection/issues/218>`_)
* feat: add grafana dashboard (`#209 <https://github.com/Minipada/ros2_data_collection/issues/209>`_)
* feat: can nest and flatten sample  (`#206 <https://github.com/Minipada/ros2_data_collection/issues/206>`_)
* feat: can save map as base64 img (`#207 <https://github.com/Minipada/ros2_data_collection/issues/207>`_)
* feat: save camera image as base64 (`#205 <https://github.com/Minipada/ros2_data_collection/issues/205>`_)
* fix: distance_traveled measurement checks tf is available (`#204 <https://github.com/Minipada/ros2_data_collection/issues/204>`_)
* fix: load base path in right place (`#202 <https://github.com/Minipada/ros2_data_collection/issues/202>`_)
* conf: better messages from condition when failed (`#199 <https://github.com/Minipada/ros2_data_collection/issues/199>`_)
* conf: better information message for activation of nodes (`#198 <https://github.com/Minipada/ros2_data_collection/issues/198>`_)
* feat: can override custom key or not (`#189 <https://github.com/Minipada/ros2_data_collection/issues/189>`_)
* fix: move custom parameters to measurement node (`#185 <https://github.com/Minipada/ros2_data_collection/issues/185>`_)
* fix: run id set in measurement and not in destination (`#181 <https://github.com/Minipada/ros2_data_collection/issues/181>`_)
* feat: add dummy measurement (`#145 <https://github.com/Minipada/ros2_data_collection/issues/145>`_)
* conf: group_key is optional, empty by default (`#136 <https://github.com/Minipada/ros2_data_collection/issues/136>`_)
* fix+test: fix test polling interval (`#109 <https://github.com/Minipada/ros2_data_collection/issues/109>`_)
* test: split test_os node and add test_m_parameters (`#107 <https://github.com/Minipada/ros2_data_collection/issues/107>`_)
* cleanup: Camera measurement (`#36 <https://github.com/Minipada/ros2_data_collection/issues/36>`_)
* fix: non blocking measurement subscriber (`#35 <https://github.com/Minipada/ros2_data_collection/issues/35>`_)
* conf: set include_measurement_name to true by default (`#32 <https://github.com/Minipada/ros2_data_collection/issues/32>`_)
* fix: same_as_previous condition does not crash with empty json (`#30 <https://github.com/Minipada/ros2_data_collection/issues/30>`_)
* feat: add camera name in camera (`#29 <https://github.com/Minipada/ros2_data_collection/issues/29>`_)
* fix: non inspected camera images are collected (`#28 <https://github.com/Minipada/ros2_data_collection/issues/28>`_)
* feat: add total memory in OS measurement (`#27 <https://github.com/Minipada/ros2_data_collection/issues/27>`_)
* feat: add tcp health measurement (`#26 <https://github.com/Minipada/ros2_data_collection/issues/26>`_)
* cleanup+doc: remove not used variables and fix comment on ram data (`#19 <https://github.com/Minipada/ros2_data_collection/issues/19>`_)
* fix: max_process handled properly in cpu measurement (`#22 <https://github.com/Minipada/ros2_data_collection/issues/22>`_)
* feat: also save map as png (`#20 <https://github.com/Minipada/ros2_data_collection/issues/20>`_)
* feat+doc: add computed field in cmd_vel (`#17 <https://github.com/Minipada/ros2_data_collection/issues/17>`_)
* fix: moving use and as in python and not && (`#16 <https://github.com/Minipada/ros2_data_collection/issues/16>`_)
* fix: when getting command, strip 0 unicode (`#13 <https://github.com/Minipada/ros2_data_collection/issues/13>`_)
* fix: plugins could not be loaded (`#9 <https://github.com/Minipada/ros2_data_collection/issues/9>`_)
* fix: add missing includes (`#7 <https://github.com/Minipada/ros2_data_collection/issues/7>`_)
* test: add test for uptime (`#5 <https://github.com/Minipada/ros2_data_collection/issues/5>`_)
* test: add test for os value (`#2 <https://github.com/Minipada/ros2_data_collection/issues/2>`_)
* ci: add ci job (`#1 <https://github.com/Minipada/ros2_data_collection/issues/1>`_)
* fix+ci: uuid included as lib
* test: add first test
* feat: throw an error if ffmpeg not installed in ip_camera measurement
* fix: double_equal condition returns data correctly
* fix: ignore init publish if init_max_measurements = -1
* fix: camera measurement only send measurement if field non empty
* fix+conf: update hooks and fix package.xml
* feat: catch JSON parsing error in conditions
* fix: include condition msg header
* fix: functions needed to be virtual or overridden
* cleanup: condition are UniqueInstance plugin
* fix: only save inspected image if there are result to inspection
* fix: rename remote_keys to remote_paths in camera measurement
* fix: condition_pub\_ is a normal publisher, could not publish when lifecycle
* fix: remove callback groups, not used
* feat: add parameter validation to ip camera measurement
* feat: change permission format from integer to string(rwx/int)
* fix: if measurement crash, does not collect measurement with init_collect
* feat: add parameter validation to network measurement
* feat: if error during measurement initialization, stop it from publishing
* fix: if topic_output is empty, use the measurement name properly
* fix: enabled is a bool, not a double
* fix: speed measurement typos
* feat: measurement callback with failed validation has access to json data
* fix: apply clang-format
* featL add ip_camera measurement
* fix+doc: list string equal url
* fix: same as previous cpp
* refactor: add naming convention for plugins
* refactor: edit start service message in camera.cpp
* fix string stamped measurement and can get all messages
* feat: add conditions (list bool, double, integer, string and string match)
* feat: add warning on problems in conditions
* feat: add condition for int, double and bool equal
* cleanup
* feat: add exist condition
* fix: same_as_previous does not subscribe, data passed directly
* feat: add same_as_previous condition
* refactor: move moving condition include to include directory
* doc: add basis for documentation
* cleanup: include and dependencies
* feat: add pre-commit and fix all errors
* fix: rename variable active to active\_
* conf+feat: configure navigation for qrcodes demo
* fix: conditions plugin parameter loaded in measurement node
* feat: start draw, save image and barcode detection service in dc_bringup
* feat: add Camera measurement plugin
* fix: load custom parameters in measurement server
* conf: add map in tb3 demo
* conf: add cmd_vel in tb3 demo
* feat: add Command Velocity measurement plugin
* feat: add Map measurement plugin
* feat: can set max measurements when condition is enabled
* feat: add Destination node and include in uptime configuration and bringup
* feat: add Distance Travaled plugin
* feat: add Speed plugin
* feat: add Position plugin
* cleanup: remove non needed class variable
* feat: add Conditions, plugins to enable collect on condition. Add moving condition
* feat: add parameter to include parameter name in JSON measurement
* feat: add init_max_measurements to limit measurements to a certain number when starting the node
* feat add parameter init_collect for measurements to collect without waiting first tick
* feat: add tags in the JSON for each measurement plugin. It will be used to match to destinations
* feat: add Permissions measurement plugin
* feat: add Storage measurement plugin
* feat: add Memory measurement plugin
* feat: add Network measurement plugin
* feat: add CPU measurement plugin
* feat: add OS measurement plugin
* feat: add StringStamped measurement plugin
* feat: Add group key, used to group data in the future group node
* feat: add bringup package to start measurement as lifecycle node
* feat: add possibility to validate JSON schema and not send if not validated
* feat: initialize measurement node with polling interval and uptime plugin
* Contributors: David Bensoussan
