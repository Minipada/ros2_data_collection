.. SPDX-FileCopyrightText: 2022-2026 David Bensoussan
.. SPDX-License-Identifier: MPL-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dc_group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-02)
------------------
* chore(license): declare copyright and license on every file (REUSE) (`#301 <https://github.com/Minipada/ros2_data_collection/issues/301>`_)
* build(tooling): run the hooks with prek and manage Python deps with uv
* feat(dc_group): lift a member's incident_id onto the merged Record
* feat(group): throttle the sync-timeout warnings per group
* feat(group): drop or emit a partial Record on sync timeout (`#126 <https://github.com/Minipada/ros2_data_collection/issues/126>`_)
* fix: remove unnecessary list of comprehension
* feat+fix: add pre-commit for same version for all packages + fix
* feat+doc: add include_group_name parameter in group node
* fix: remove tags in measurements when grouping them
* fix+doc: list string equal url
* feat: catch when groups is not initialized and exits the groups node
* cleanup: fix docstrings and run pre-commit
* refactor: dc_group upgraded to python3.10
* feat: add dc_group node
* Contributors: David Bensoussan
