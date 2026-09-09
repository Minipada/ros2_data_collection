# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Tests for the Group merge rules (#497), extracted out of `GroupServer.publish_record`.

Plain dicts in, plain dicts out — no ROS, no executor, no clock. The live-graph side of the
same behaviour stays in `test_group_sync_timeout.py`, which CI drives on a real `GroupServer`.
"""

import json

import pytest
from dc_group.flatten import (
    check_if_numbers_are_consecutive,
    flatten,
    unflatten,
    unflatten_list,
)
from dc_group.group_merge import SEPARATOR, apply_exclude_keys, merge_records

GROUP = "test_group"


def cfg(**overrides):
    """The group parameters the merge reads, at their declared defaults."""
    group_cfg = {
        "exclude_keys": [""],
        "tags": ["dc"],
        "nested_data": True,
        "include_group_name": True,
    }
    group_cfg.update(overrides)
    return group_cfg


def payload(group_key, data):
    return {"group_key": group_key, "data": data}


def merge(payloads, **overrides):
    return merge_records(payloads, GROUP, cfg(**overrides))


# --- the merge itself ---------------------------------------------------------------------


def test_members_merge_under_their_own_group_keys():
    record = merge([payload("a", {"used": 12.0}), payload("b", {"free": 34.0})])

    assert record == {
        "a": {"used": 12.0},
        "b": {"free": 34.0},
        "tags": ["dc"],
        "name": GROUP,
    }


def test_members_sharing_a_group_key_collapse_into_the_last_one():
    # Pinning current behaviour: a member whose `group_key` equals another's silently
    # replaces it, because the merge is a dict union keyed by `group_key`.
    record = merge([payload("a", {"used": 1.0}), payload("a", {"used": 2.0})])

    assert record["a"] == {"used": 2.0}


def test_empty_member_data_is_kept_as_an_empty_object():
    record = merge([payload("a", {}), payload("b", {"free": 1.0})])

    assert record["a"] == {}
    assert record["b"] == {"free": 1.0}


def test_an_empty_group_key_leaves_the_members_keys_unprefixed():
    # Pinning current behaviour: `flatten` treats an empty key as no key at all, so a
    # member publishing an empty `group_key` lands at the top level of the Record.
    record = merge([payload("", {"used": 12.0})])

    assert record["used"] == 12.0
    assert "a" not in record


def test_merge_does_not_mutate_the_payloads_it_is_given():
    payloads = [
        payload("a", {"used": 12.0, "tags": ["x"], "incident_id": "incident-1"}),
        payload("b", {"free": 34.0}),
    ]

    merge(payloads, tags=[""])

    assert payloads[0]["data"] == {"used": 12.0, "tags": ["x"], "incident_id": "incident-1"}
    assert payloads[1]["data"] == {"free": 34.0}


# --- exclude_keys -------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("exclude_keys", "kept"),
    [
        # The declared default: nothing excluded.
        ([""], {"a.cpu", "a.cpuload", "ab.used", "b.free"}),
        # A `*`-free entry is a key prefix, so it takes the member's whole subtree — and
        # the *member* named `ab` with it, the prefix having no key boundary.
        (["a"], {"b.free"}),
        (["a.cpu"], {"ab.used", "b.free"}),
        (["b"], {"a.cpu", "a.cpuload", "ab.used"}),
        (["c"], {"a.cpu", "a.cpuload", "ab.used", "b.free"}),
        # Entries compose: each one narrows what is left.
        (["a", "b.free"], set()),
        # The prefix has no key boundary: `a.cpu` also catches `a.cpuload`.
        (["a.c"], {"ab.used", "b.free"}),
        # `*` in an entry matches when every `*`-separated fragment appears in the key.
        (["a.*"], {"ab.used", "b.free"}),
        (["*load"], {"a.cpu", "ab.used", "b.free"}),
        (["*.*"], set()),
        (["*"], set()),
        # The `b.` fragment lives inside the member named `ab`, so it takes that one too.
        (["a.*", "b.*"], set()),
    ],
)
def test_exclude_keys_filters_flattened_keys(exclude_keys, kept):
    payloads = [
        payload("a", {"cpu": 1.0, "cpuload": 2.0}),
        payload("ab", {"used": 3.0}),
        payload("b", {"free": 4.0}),
    ]

    record = merge(payloads, exclude_keys=exclude_keys, nested_data=False)

    assert set(record) == kept | {"tags", "name"}


def test_exclude_keys_glob_is_substring_matching_not_a_glob():
    # Pinning current behaviour: the `*` branch splits and checks substrings, so a
    # fragment matches anywhere in the key, in any position a glob would not allow.
    record = merge([payload("a", {"used": 1.0})], exclude_keys=["u*d"])

    assert set(record) == {"tags", "name"}


def test_apply_exclude_keys_returns_the_flattened_keys():
    flat = {"a.used": 1.0, "b.free": 2.0}

    assert apply_exclude_keys(flat, ["a"]) == {"b.free": 2.0}
    assert apply_exclude_keys(flat, []) == flat


# --- nested_data --------------------------------------------------------------------------


def test_nested_data_off_leaves_the_flattened_keys_in_the_record():
    record = merge([payload("a", {"used": 12.0}), payload("b", {"free": 34.0})], nested_data=False)

    assert record == {"a.used": 12.0, "b.free": 34.0, "tags": ["dc"], "name": GROUP}


def test_lists_survive_the_flatten_round_trip():
    record = merge([payload("a", {"samples": [1.0, 2.0]})])

    assert record == {"a": {"samples": [1.0, 2.0]}, "tags": ["dc"], "name": GROUP}


def test_deeply_nested_member_data_is_rebuilt():
    record = merge([payload("a", {"pose": {"position": {"x": 1.0}}})])

    assert record["a"] == {"pose": {"position": {"x": 1.0}}}


# --- plugins --------------------------------------------------------------------------------


def test_plugin_fields_are_collected_into_a_top_level_list():
    record = merge(
        [
            payload("a", {"plugin": "cpu", "used": 1.0}),
            payload("b", {"plugin": "memory", "free": 2.0}),
        ]
    )

    assert record["plugins"] == ["cpu", "memory"]
    # Collected, not lifted: the member keeps its own field.
    assert record["a"]["plugin"] == "cpu"


def test_plugins_key_is_absent_when_no_member_carries_one():
    record = merge([payload("a", {"used": 1.0})])

    assert "plugins" not in record


def test_plugins_key_is_absent_when_collection_is_off():
    record = merge_records(
        [payload("a", {"plugin": "cpu", "used": 1.0})], GROUP, cfg(), collect_plugins=False
    )

    assert "plugins" not in record
    assert record["a"] == {"plugin": "cpu", "used": 1.0}


def test_plugins_keep_the_member_order_and_their_duplicates():
    record = merge_records(
        [
            payload("a", {"plugin": "cpu"}),
            payload("b", {"plugin": "cpu"}),
            payload("c", {}),
        ],
        GROUP,
        cfg(),
    )

    assert record["plugins"] == ["cpu", "cpu"]


# --- tags -----------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "member_data",
    [{"used": 12.0, "tags": ["measurement-tag"]}, {"used": 12.0}],
    ids=["member-carries-tags", "member-has-no-tags"],
)
def test_record_carries_the_group_tags_not_the_members(member_data):
    record = merge([payload("a", member_data)], tags=["robot-7"])

    assert record["tags"] == ["robot-7"]


def test_tags_key_is_present_even_when_the_group_declares_none():
    record = merge([payload("a", {"used": 12.0})], tags=[""])

    assert record["tags"] == [""]


# --- incident_id ----------------------------------------------------------------------------


def test_incident_id_is_lifted_to_the_records_top_level():
    # Nested under the member's `group_key` it would become `a.incident_id`, which is no
    # column any Destination table has — the Postgres sink drops it silently (#291).
    record = merge(
        [
            payload("a", {"used": 12.0, "incident_id": "incident-42"}),
            payload("b", {"free": 34.0, "incident_id": "incident-42"}),
        ]
    )

    assert record["incident_id"] == "incident-42"
    # Lifted, not copied: it must not also stay behind inside the members' own data.
    assert record["a"] == {"used": 12.0}
    assert record["b"] == {"free": 34.0}


@pytest.mark.parametrize(
    ("member_ids", "expected"),
    [
        # One FlushEvent mints one id for every Measurement listening: all agree.
        (["incident-42", "incident-42"], "incident-42"),
        # First non-null wins, so a partial Record built from a mix of released and live
        # members still carries the id of the one that arrived.
        ([None, "incident-42"], "incident-42"),
        (["incident-42", None], "incident-42"),
    ],
)
def test_first_non_null_incident_id_wins(member_ids, expected):
    payloads = [
        payload(group_key, {"incident_id": incident_id} if incident_id else {})
        for group_key, incident_id in zip(["a", "b"], member_ids, strict=True)
    ]

    assert merge(payloads)["incident_id"] == expected


def test_incident_id_is_absent_when_no_member_carries_one():
    # A Record collected outside an incident leaves the column NULL, not empty or null.
    record = merge([payload("a", {"used": 12.0}), payload("b", {"free": 34.0})])

    assert "incident_id" not in record


def test_an_empty_string_incident_id_is_still_lifted():
    # Pinning current behaviour: the lift is on `is None`, so an empty string counts as an
    # id and shadows a real one arriving from a later member.
    record = merge(
        [payload("a", {"incident_id": ""}), payload("b", {"incident_id": "incident-42"})]
    )

    assert record["incident_id"] == ""


# --- the envelope ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "missing_inputs",
    [[], None],
    ids=["empty-list", "none"],
)
def test_a_complete_record_is_left_unmarked(missing_inputs):
    record = merge_records([payload("a", {"used": 12.0})], GROUP, cfg(), missing_inputs)

    assert "partial" not in record
    assert "missing_inputs" not in record


def test_a_partial_record_names_the_inputs_that_never_show_up():
    record = merge_records(
        [payload("a", {"used": 12.0})], GROUP, cfg(), missing_inputs=["/dc/test/b"]
    )

    assert record["partial"] is True
    assert record["missing_inputs"] == ["/dc/test/b"]


def test_missing_inputs_is_copied_not_aliased():
    missing = ["/dc/test/b"]

    record = merge_records([payload("a", {"used": 12.0})], GROUP, cfg(), missing)
    missing.append("/dc/test/c")

    assert record["missing_inputs"] == ["/dc/test/b"]


def test_a_complete_record_keeps_its_pre_timeout_shape():
    record = merge([payload("a", {"used": 12.0}), payload("b", {"free": 34.0})])

    assert "partial" not in record
    assert "missing_inputs" not in record


def test_envelope_keys_are_appended_after_the_merged_data():
    # The Group node serializes this dict straight to JSON, so its key order is the
    # column order every Destination sees.
    record = merge([payload("a", {"used": 12.0}), payload("b", {"free": 34.0})])

    assert list(record) == ["a", "b", "tags", "name"]


def test_envelope_keys_follow_the_same_order_without_the_optionals():
    record = merge_records(
        [payload("a", {"used": 12.0})],
        GROUP,
        cfg(include_group_name=False),
        missing_inputs=["/dc/test/b"],
    )

    assert list(record) == ["a", "tags", "partial", "missing_inputs"]


def test_serialized_record_is_stable():
    record = merge(
        [
            payload("a", {"used": 12.0, "incident_id": "incident-42", "plugin": "cpu"}),
            payload("b", {"free": 34.0, "incident_id": "incident-42"}),
        ],
        tags=["robot-7"],
    )

    assert json.dumps(record) == (
        '{"a": {"plugin": "cpu", "used": 12.0}, "b": {"free": 34.0}, "tags": ["robot-7"], '
        '"incident_id": "incident-42", "plugins": ["cpu"], "name": "test_group"}'
    )


# --- flatten / unflatten --------------------------------------------------------------------
# First tests for the vendored `amirziai/flatten` copy: it decides the key of every column
# every Destination sees, and upstream admits it "has not been tested extensively".


def test_flatten_turns_nesting_into_separator_joined_keys():
    assert flatten({"a": {"b": {"c": 1}}, "d": 2}, separator=SEPARATOR) == {
        "a.b.c": 1,
        "d": 2,
    }


def test_flatten_indexes_list_elements():
    assert flatten({"a": {"samples": [10, [20, 30]]}}, separator=SEPARATOR) == {
        "a.samples.0": 10,
        "a.samples.1.0": 20,
        "a.samples.1.1": 30,
    }


def test_flatten_flattens_tuples_like_lists():
    assert flatten({"a": (1, 2)}, separator=SEPARATOR) == {"a.0": 1, "a.1": 2}


def test_flatten_takes_an_empty_value_as_is():
    assert flatten({"a": {}, "b": {"c": []}}, separator=SEPARATOR) == {"a": {}, "b.c": []}


def test_flatten_of_an_empty_dict_is_empty():
    assert flatten({}, separator=SEPARATOR) == {}


def test_flatten_can_skip_root_keys():
    assert flatten({"a": 1, "b": 2}, root_keys_to_ignore=["a"], separator=SEPARATOR) == {"b": 2}


def test_flatten_requires_a_dictionary():
    with pytest.raises(AssertionError, match="flatten requires a dictionary input"):
        flatten([1, 2], separator=SEPARATOR)


def test_unflatten_rebuilds_nesting():
    assert unflatten({"a.b.c": 1, "a.d": 2, "e": 3}, separator=SEPARATOR) == {
        "a": {"b": {"c": 1}, "d": 2},
        "e": 3,
    }


def test_unflatten_requires_a_flat_dictionary():
    with pytest.raises(AssertionError, match="provided dict is not flat"):
        unflatten({"a": [1, 2]}, separator=SEPARATOR)


def test_unflatten_silently_drops_a_key_nested_under_another():
    # Pinning current behaviour: a scalar sitting at a prefix of another key is assumed to
    # be an artifact and skipped, so `a` disappears from the Record.
    assert unflatten({"a": 1, "a.b": 2}, separator=SEPARATOR) == {"a": {"b": 2}}


def test_unflatten_list_rebuilds_lists_from_consecutive_indices():
    assert unflatten_list({"a.samples.0": 10, "a.samples.1": 20}, separator=SEPARATOR) == {
        "a": {"samples": [10, 20]}
    }


def test_unflatten_list_rebuilds_lists_of_dicts():
    assert unflatten_list({"a.0.b": 1, "a.1.c": 2}, separator=SEPARATOR) == {
        "a": [{"b": 1}, {"c": 2}]
    }


def test_unflatten_list_keeps_a_dict_when_indices_are_not_consecutive():
    assert unflatten_list({"a.0": 1, "a.2": 3}, separator=SEPARATOR) == {"a": {"0": 1, "2": 3}}


def test_unflatten_list_keeps_a_dict_of_non_numeric_keys():
    assert unflatten_list({"a.x": 1, "a.y": 2}, separator=SEPARATOR) == {"a": {"x": 1, "y": 2}}


def test_unflatten_list_of_an_empty_dict_is_empty():
    assert unflatten_list({}, separator=SEPARATOR) == {}


def test_unflatten_list_crashes_on_a_zero_padded_index():
    # Pinning current behaviour: `00` is recognised as index 0 but then read back as the
    # string `"0"`, so the round trip raises. A Measurement nesting a zero-padded key under
    # a dict takes the Group node down with it.
    with pytest.raises(KeyError):
        unflatten_list({"a.00": 1}, separator=SEPARATOR)


def test_unflatten_list_crashes_on_a_top_level_list():
    # Pinning current behaviour: only nested lists are rebuilt — the root has no parent to
    # replace the list in, so the conversion raises instead.
    with pytest.raises(TypeError):
        unflatten_list({"0.a": 1, "1.b": 2}, separator=SEPARATOR)


def test_check_if_numbers_are_consecutive():
    assert check_if_numbers_are_consecutive([0, 1, 2])
    assert not check_if_numbers_are_consecutive([0, 2])
    # Vacuously true: an empty dict of indices counts as a list.
    assert check_if_numbers_are_consecutive([])
