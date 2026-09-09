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


# --- payloads that are not objects ----------------------------------------------------------
# A Measurement can publish any JSON value `json.loads` accepts, and `dict()` on the ones
# that are not objects used to raise out of the Group node's callback, killing it (#514).
# The rule, the same never-raise, never-drop one as #508: the value keeps its place under
# its `group_key`, and only an object can carry envelope fields.


@pytest.mark.parametrize(
    "raw",
    ['"hi"', "42", "3.25", "true", "false", "null", "[1, 2]", "[]", '{"used": 12.0}'],
)
def test_every_json_payload_merges_without_raising(raw):
    record = merge([payload("a", json.loads(raw))])

    assert record["name"] == GROUP


@pytest.mark.parametrize(
    ("raw", "value"),
    [
        ('"hi"', "hi"),
        ("42", 42),
        ("3.25", 3.25),
        ("true", True),
        ("false", False),
        ("null", None),
        ("[1, 2]", [1, 2]),
        ("[]", []),
    ],
)
def test_a_non_object_payload_is_kept_as_it_is_under_its_group_key(raw, value):
    record = merge([payload("a", json.loads(raw))])

    assert record["a"] == value


def test_an_array_payload_survives_the_flatten_round_trip():
    record = merge([payload("a", {"samples": [1.0, 2.0]}), payload("b", [1.0, 2.0])])

    assert record["a"] == {"samples": [1.0, 2.0]}
    assert record["b"] == [1.0, 2.0]


def test_an_array_payload_is_indexed_when_nesting_is_off():
    record = merge([payload("a", [1.0, 2.0])], nested_data=False)

    assert record == {"a.0": 1.0, "a.1": 2.0, "tags": ["dc"], "name": GROUP}


def test_a_non_object_member_carries_no_envelope_field():
    record = merge(
        [
            payload("a", "hi"),
            payload("b", {"plugin": "memory", "free": 1.0}),
        ]
    )

    # Nothing to lift off a scalar: no plugin of its own in the list.
    assert record["plugins"] == ["memory"]


def test_merge_does_not_mutate_a_non_object_payload():
    data = [1.0, 2.0]

    merge([payload("a", data)], tags=[""])

    assert data == [1.0, 2.0]


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
        # An entry holding `*` is a glob over the flattened key, anchored at both ends.
        (["a.*"], {"ab.used", "b.free"}),
        (["*load"], {"a.cpu", "ab.used", "b.free"}),
        (["*.*"], set()),
        (["*"], set()),
        # A glob has a key boundary the prefix form lacks: `b.*` leaves `ab.used` alone.
        (["b.*"], {"a.cpu", "a.cpuload", "ab.used"}),
        (["a.*", "b.*"], {"ab.used"}),
        # `?` and `[seq]` are glob syntax too, so they no longer match literally.
        (["a.?pu"], {"a.cpuload", "ab.used", "b.free"}),
        (["a.cp[u]load"], {"a.cpu", "ab.used", "b.free"}),
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


def test_exclude_keys_star_entry_is_a_glob_not_a_substring_match():
    # Was pinned as substring matching: a `u*d` fragment took `a.used` down with it, though a
    # glob has to start with a `u`. Fixed in #508 — the key survives, and so does its member.
    record = merge([payload("a", {"used": 1.0})], exclude_keys=["u*d"])

    assert record["a"] == {"used": 1.0}
    assert set(record) == {"a", "tags", "name"}


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
# `incident_id` rides the StringStamped envelope (#506), not the payload: GroupServer lifts
# it from the members' envelopes, and the merge neither sees nor lifts a payload key of that
# name — one inside a member's data is measurement data like any other.


def test_a_payload_level_incident_id_is_member_data_not_envelope():
    # Was lifted to the Record's top level when it lived inside the payload (#291); the
    # envelope field never enters the merge, so a key of that name in a member's data stays
    # under the member's group_key.
    record = merge(
        [
            payload("a", {"used": 12.0, "incident_id": "incident-42"}),
            payload("b", {"free": 34.0}),
        ]
    )

    assert record["a"] == {"used": 12.0, "incident_id": "incident-42"}
    assert "incident_id" not in record


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
        '{"a": {"incident_id": "incident-42", "plugin": "cpu", "used": 12.0}, '
        '"b": {"free": 34.0, "incident_id": "incident-42"}, "tags": ["robot-7"], '
        '"plugins": ["cpu"], "name": "test_group"}'
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


def test_unflatten_keeps_a_scalar_whose_key_prefixes_another():
    # Was pinned as a silent drop: `a` vanished from the Record because `a.b` nested under it.
    # A key that is also a prefix of another has no nested form — both cannot sit under it — so
    # the deeper key keeps its literal dotted key instead and nothing is dropped (#508).
    assert unflatten({"a": 1, "a.b": 2}, separator=SEPARATOR) == {"a": 1, "a.b": 2}


def test_unflatten_keeps_every_scalar_of_a_deeper_collision():
    # The rule holds all the way down: once `a.b` is a literal key, `a.b.c` cannot nest
    # under the scalar it holds either.
    assert unflatten({"a": 1, "a.b": 2, "a.b.c": 3}, separator=SEPARATOR) == {
        "a": 1,
        "a.b": 2,
        "a.b.c": 3,
    }


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


def test_unflatten_list_keeps_a_zero_padded_index_as_a_dict():
    # Was pinned as a KeyError: `00` is recognised as the index 0 but read back as the string
    # `"0"`. A list needs the canonical `0`..`n-1` keys, so the dict is left as it unflattened
    # — reshaped, not raised — and the Group node keeps its Record (#508).
    assert unflatten_list({"a.00": 1}, separator=SEPARATOR) == {"a": {"00": 1}}


def test_unflatten_list_rebuilds_a_top_level_list():
    # Was pinned as a TypeError: only nested lists were rebuilt, the root having no parent to
    # replace the list in. The root is converted like any other level now (#508).
    assert unflatten_list({"0.a": 1, "1.b": 2}, separator=SEPARATOR) == [{"a": 1}, {"b": 2}]


def test_a_record_whose_group_keys_are_all_numeric_stays_an_object():
    # Numeric group_keys would unflatten the merged Record into a JSON array. A Record is an
    # object — `tags` and `plugins` are top-level keys, which is all a `postgres` sink maps
    # onto columns — so the members stay nested under their group_key instead (#508).
    record = merge([payload("0", {"used": 1.0}), payload("1", {"free": 2.0})])

    assert record == {"0": {"used": 1.0}, "1": {"free": 2.0}, "tags": ["dc"], "name": GROUP}


def test_a_record_with_a_numeric_and_a_named_member_is_nested_normally():
    # A single numeric group_key among named ones is not enough to make the Record a list.
    record = merge([payload("0", {"used": 1.0}), payload("cpu", {"free": 2.0})])

    assert record["0"] == {"used": 1.0}
    assert record["cpu"] == {"free": 2.0}


def test_check_if_numbers_are_consecutive():
    assert check_if_numbers_are_consecutive([0, 1, 2])
    assert not check_if_numbers_are_consecutive([0, 2])
    # Vacuously true: an empty dict of indices counts as a list.
    assert check_if_numbers_are_consecutive([])
