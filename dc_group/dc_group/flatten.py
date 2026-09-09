# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

"""Flatten a nested payload to `{key: scalar}` and back (#508).

`GroupServer` calls these from a synchroniser callback, where an exception takes the node
down, so both directions are total: no payload a Measurement publishes makes them raise, and
no shape they cannot express silently drops a value — a key that cannot be nested keeps its
scalar and the key under it keeps its literal dotted key.
"""

from collections.abc import Iterable


def _unflatten_asserts(flat_dict, separator):
    assert isinstance(flat_dict, dict), "un_flatten requires dictionary input"
    assert isinstance(separator, str), "separator must be string"
    assert all(
        not value or not isinstance(value, Iterable) or isinstance(value, str)
        for value in flat_dict.values()
    ), "provided dict is not flat"


def _construct_key(
    previous_key: str, separator: str, new_key: str, replace_separators: str | None = None
):
    """Returns the new_key if no previous key exists, otherwise concatenates
    previous key, separator, and new_key

    Args:
        previous_key (str): Previous key
        separator (str): string to separate dictionary keys by
        new_key (str): New key
        replace_separators (str, optional): _description_. Defaults to None.

    Returns:
        str: New key
    """
    if replace_separators is not None:
        new_key = str(new_key).replace(separator, replace_separators)
    if previous_key:
        return f"{previous_key}{separator}{new_key}"
    else:
        return new_key


def flatten(
    nested_dict,
    separator="_",
    root_keys_to_ignore: list | None = None,
    replace_separators: str | None = None,
) -> dict:
    """Flattens a dictionary with nested structure to a dictionary with no
    hierarchy.
    Consider ignoring keys that you are not interested in to prevent
        unnecessary processing. This is specially true for very deep objects

    Args:
        nested_dict (str): dictionary we want to flatten
        separator (str, optional): string to separate dictionary keys by. Defaults to "_".
        root_keys_to_ignore (list, optional): set of root keys to ignore from flattening.
            Defaults to None.
        replace_separators (str, optional): Replace separators within keys. Defaults to None.

    Returns:
        dict: Flattened dictionary
    """
    assert isinstance(nested_dict, dict), "flatten requires a dictionary input"
    assert isinstance(separator, str), "separator must be string"

    if root_keys_to_ignore is None:
        root_keys_to_ignore = set()

    if len(nested_dict) == 0:
        return {}

    # This global dictionary stores the flattened keys and values and is
    # ultimately returned
    flattened_dict = {}

    def _flatten(object_: dict, key: str):
        """For dict, list and set objects_ calls itself on the elements and for other types
            assigns the object_ to the corresponding key in the global flattened_dict.

        Args:
            object_ (dict): object to flatten
            key (str): carries the concatenated key for the object_
        """
        # Empty object can't be iterated, take as is
        if not object_:
            flattened_dict[key] = object_
        # These object types support iteration
        elif isinstance(object_, dict):
            for object_key in object_:
                if not (not key and object_key in root_keys_to_ignore):
                    _flatten(
                        object_[object_key],
                        _construct_key(
                            key, separator, object_key, replace_separators=replace_separators
                        ),
                    )
        elif isinstance(object_, (list, set, tuple)):
            for index, item in enumerate(object_):
                _flatten(
                    item,
                    _construct_key(key, separator, index, replace_separators=replace_separators),
                )
        # Anything left take as is
        else:
            flattened_dict[key] = object_

    _flatten(nested_dict, None)
    return flattened_dict


def unflatten(flat_dict: dict, separator: str | None = "_") -> dict:
    """Creates a hierarchical dictionary from a flattened dictionary, assume no lists are present.

    A key that is also a *prefix* of another one has no nested form: `{"a": 1, "a.b": 2}`
    cannot put both a scalar and a dict under `a`. Rather than dropping the scalar, the deeper
    key is kept as its literal dotted key, so the round trip loses nothing — Postgres holds the
    result in one `jsonb` column, which accepts either shape.

    Args:
        flat_dict (dict): a dictionary with no hierarchy
        separator (str, optional): a string that separates keys. Defaults to "_".

    Returns:
        dict: a dictionary with hierarchy
    """
    _unflatten_asserts(flat_dict, separator)

    # This dictionary is mutated and returned
    unflattened_dict = {}

    for key in sorted(flat_dict):
        dic = unflattened_dict
        segments = key.split(separator)
        for depth, segment in enumerate(segments[:-1]):
            if segment not in dic:
                dic[segment] = {}
            child = dic[segment]
            if not isinstance(child, dict):
                # A scalar holds the prefix: nesting here would drop it, so the rest of the
                # path stays a literal key and both values survive.
                dic[separator.join(segments[depth:])] = flat_dict[key]
                break
            dic = child
        else:
            dic[segments[-1]] = flat_dict[key]
    return unflattened_dict


def check_if_numbers_are_consecutive(list_: list) -> bool:
    """Returns True if numbers in the list are consecutive

    Args:
        list_ (list): list of integers

    Returns:
        bool: Returns True if numbers in the list are consecutive
    """
    return all(
        True if second - first == 1 else False
        for first, second in zip(list_[:-1], list_[1:], strict=True)
    )


def _list_indices(object_) -> list:
    """Returns the indices `object_` needs to become a list, empty if it is not shaped like one."""
    try:
        indices = sorted(int(key) for key in object_)
    except (ValueError, TypeError):
        return []
    # Read back through the canonical `str(index)` form: `00` parses as the index 0 but is not
    # the key `0`, so a zero-padded index keeps the dict it unflattened to rather than raising
    # on a key that is not there.
    if set(object_) != {str(index) for index in indices}:
        return []
    return indices


def _convert_dict_to_list(object_):
    """Returns `object_` with every dict of consecutive indices inside it rebuilt as a list."""
    if not isinstance(object_, dict):
        return object_
    # Children first, so a list nested inside a list member is rebuilt too
    # https://github.com/amirziai/flatten/issues/15
    for key, value in object_.items():
        object_[key] = _convert_dict_to_list(value)
    indices = _list_indices(object_)
    if indices and indices[0] == 0 and check_if_numbers_are_consecutive(indices):
        return [object_[str(index)] for index in indices]
    return object_


def unflatten_list(flat_dict: dict, separator="_") -> dict | list:
    """Unflatten a dictionary, first assuming no lists exist and then tries to
    identify lists and replaces them.

    A dict is read as a list only when its keys are exactly the canonical indices `0` to
    `n - 1`, at the root as well as deeper: `{"0.a": 1, "1.b": 2}` comes back as
    `[{"a": 1}, {"b": 2}]`. Anything else stays the dict it unflattened to, so a payload a
    Measurement publishes can only ever reshape, never raise.

    Args:
        flat_dict (dict): dictionary with no hierarchy
        separator (str, optional): a string that separates keys. Defaults to "_".

    Returns:
        dict: a dictionary with hierarchy, or a list if the root itself is index-keyed
    """
    _unflatten_asserts(flat_dict, separator)

    # First unflatten the dictionary assuming no lists exist
    return _convert_dict_to_list(unflatten(flat_dict, separator))
