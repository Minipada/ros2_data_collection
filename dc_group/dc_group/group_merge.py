# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Merge the Records of one synchronised window into a single Record payload.

Plain dicts in, plain dict out: no `rclpy`, no clock, no messages. `GroupServer` converts
at the edges and hands over already-parsed payloads, so these rules are testable without a
ROS install.

A member payload is whatever `json.loads` made of its Record — object, array, or bare
scalar (#514). Only an object can carry the envelope fields, so a non-object member is
merged as its value under its `group_key`, the same never-raise, never-drop rule as #508.
"""

from fnmatch import fnmatchcase

from .flatten import flatten, unflatten, unflatten_list

SEPARATOR = "."

# An entry holding any of these is a glob over the flattened key; anything else is a prefix.
_GLOB_SPECIALS = "*?["


def apply_exclude_keys(data_dict: dict, exclude_keys: list) -> dict:
    """Drop the flattened keys an `exclude_keys` entry names.

    Args:
        data_dict (dict): Flattened Record, filtered in place of being copied
        exclude_keys (list): Patterns applied in order, each narrowing the Record further.
            An entry holding no glob syntax is a key prefix; otherwise it is a glob over the
            whole flattened key (`fnmatch` syntax, case-sensitive, anchored at both ends)

    Returns:
        dict: The Record minus the excluded keys
    """
    for exclude_key in exclude_keys:
        if exclude_key == "":
            continue
        if not any(char in exclude_key for char in _GLOB_SPECIALS):
            data_dict = {k: v for k, v in data_dict.items() if not k.startswith(exclude_key)}
        else:
            data_dict = {k: v for k, v in data_dict.items() if not fnmatchcase(k, exclude_key)}
    return data_dict


def merge_records(
    parsed_payloads: list,
    group: str,
    group_cfg: dict,
    missing_inputs: list | None = None,
    collect_plugins: bool = True,
) -> dict:
    """Merge parsed Measurement payloads into one Record payload.

    Args:
        parsed_payloads (list): One entry per member Record, ordered by the input it came
            from: `{"group_key": <the member's group_key>, "data": <its parsed payload>}`.
            Two members sharing a `group_key` collapse into the last one's fields. A `data`
            that is not an object — a bare scalar or an array — is merged as it is
        group (str): Name of the group being published
        group_cfg (dict): The group's parameters — `exclude_keys`, `tags`, `nested_data`
            and `include_group_name`
        missing_inputs (list, optional): Input topics that contributed no Record. Empty for
            a complete Record, non-empty for a partial one. Defaults to None
        collect_plugins (bool, optional): Gather the members' `plugin` fields into a
            top-level `plugins` list. Defaults to True

    Returns:
        dict: The Record payload, ready to serialize
    """
    data_dict = {}
    plugins_list = []
    incident_id = None
    for payload in parsed_payloads:
        m_data = payload["data"]
        if isinstance(m_data, dict):
            # A copy: the pops below must not reach back into the caller's payload.
            m_data = dict(m_data)
            m_data.pop("tags", None)
            # `incident_id` is an envelope field, not measurement data (#291): merged under a
            # member's `group_key` it would become `<group_key>.incident_id`, which is no
            # column any Destination table has and so is silently dropped by the Postgres
            # sink. Lifted to the merged Record's top level instead, the same way `tags` is,
            # so a grouped incident stays queryable as `WHERE incident_id = ...`. One
            # FlushEvent mints one id for every Measurement listening, so the members of a
            # released window all carry the same one — first non-null wins, and a partial
            # Record built from a mix of released and live members still carries it.
            member_incident_id = m_data.pop("incident_id", None)
            if incident_id is None:
                incident_id = member_incident_id
            if collect_plugins and "plugin" in m_data:
                plugins_list.append(m_data["plugin"])
        # else: a payload that is not an object (#514) — a bare scalar or an array. It has
        # no `tags`, `incident_id` or `plugin` to lift, and `dict()` on it is what used to
        # raise from the callback. It goes under the member's `group_key` as it is: the
        # same never-raise, never-drop rule as #508, and the flatten round trip below
        # already brings a scalar or an array back unchanged.
        data_dict = data_dict | flatten(
            nested_dict={payload["group_key"]: m_data}, separator=SEPARATOR
        )

    data_dict = apply_exclude_keys(data_dict, group_cfg["exclude_keys"])
    if group_cfg["nested_data"]:
        flat_data = data_dict
        data_dict = unflatten_list(flat_dict=data_dict, separator=SEPARATOR)
        if not isinstance(data_dict, dict):
            # Every group_key is numeric, so the merged Record unflattened into a JSON array.
            # A Record is an object: its envelope (`tags`, `incident_id`, `plugins`) is made of
            # top-level keys, and a top-level key is all a `postgres` sink maps onto a column,
            # so an array would land in no column at all. The members stay nested under their
            # numeric group_key instead, keeping the rest of the Record intact.
            data_dict = unflatten(flat_data, separator=SEPARATOR)
    # A member's own Tags are dropped above: the Record carries the group's.
    data_dict["tags"] = group_cfg["tags"]
    # Only when a member actually carried one: a Record collected outside an incident must
    # leave the column NULL rather than write an explicit null into every grouped Record.
    if incident_id is not None:
        data_dict["incident_id"] = incident_id
    if collect_plugins and plugins_list:
        data_dict["plugins"] = plugins_list

    if group_cfg["include_group_name"]:
        data_dict["name"] = group

    # A Record assembled by the sync timeout rather than by the synchroniser is marked
    # so consumers can tell it apart. A complete Record carries neither key, keeping its
    # shape identical to what DC published before the timeout existed. The keys of the
    # inputs that never showed up are simply absent: the Group node only knows a missing
    # input's *topic*, never its `group_key` or its fields (both come from the Record
    # itself), so it cannot synthesise a correctly shaped null placeholder for it.
    if missing_inputs:
        data_dict["partial"] = True
        data_dict["missing_inputs"] = list(missing_inputs)

    return data_dict
