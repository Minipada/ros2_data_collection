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

    Args:
        flat_dict (dict): a dictionary with no hierarchy
        separator (str, optional): a string that separates keys. Defaults to "_".

    Returns:
        dict: a dictionary with hierarchy
    """
    _unflatten_asserts(flat_dict, separator)

    # This global dictionary is mutated and returned
    unflattened_dict = {}

    def _unflatten(dic, keys, value):
        for key in keys[:-1]:
            dic = dic.setdefault(key, {})

        dic[keys[-1]] = value

    list_keys = sorted(flat_dict.keys())
    for i, item in enumerate(list_keys):
        if i != len(list_keys) - 1:
            split_key = item.split(separator)
            next_split_key = list_keys[i + 1].split(separator)
            if not split_key == next_split_key[:-1]:
                _unflatten(unflattened_dict, item.split(separator), flat_dict[item])
            else:
                pass  # if key contained in next key, json will be invalid.
        else:
            #  last element
            _unflatten(unflattened_dict, item.split(separator), flat_dict[item])
    return unflattened_dict


def check_if_numbers_are_consecutive(list_: list) -> bool:
    """Returns True if numbers in the list are consecutive

    Args:
        list_ (list): list of integers

    Returns:
        bool: Returns True if numbers in the list are consecutive
    """
    return all(
        True if second - first == 1 else False for first, second in zip(list_[:-1], list_[1:])
    )


def unflatten_list(flat_dict: dict, separator="_") -> dict:
    """Unflatten a dictionary, first assuming no lists exist and then tries to
    identify lists and replaces them
    This is probably not very efficient and has not been tested extensively
    Feel free to add test cases or rewrite the logic
    Issues that stand out to me:
    - Sorting all the keys in the dictionary, which specially for the root
    dictionary can be a lot of keys
    - Checking that numbers are consecutive is O(N) in number of keys

    Args:
        flat_dict (dict): dictionary with no hierarchy
        separator (str, optional): a string that separates keys. Defaults to "_".

    Returns:
        dict: a dictionary with hierarchy
    """
    _unflatten_asserts(flat_dict, separator)

    # First unflatten the dictionary assuming no lists exist
    unflattened_dict = unflatten(flat_dict, separator)

    def _convert_dict_to_list(object_, parent_object, parent_object_key):
        if isinstance(object_, dict):
            for key in object_:
                if isinstance(object_[key], dict):
                    _convert_dict_to_list(object_[key], object_, key)
            try:
                keys = [int(key) for key in object_]
                keys.sort()
            except (ValueError, TypeError):
                keys = []
            keys_len = len(keys)

            if (
                keys_len > 0
                and sum(keys) == int(((keys_len - 1) * keys_len) / 2)
                and keys[0] == 0
                and keys[-1] == keys_len - 1
                and check_if_numbers_are_consecutive(keys)
            ):
                # The dictionary looks like a list so we're going to replace it
                parent_object[parent_object_key] = []
                for key_index, key in enumerate(keys):
                    parent_object[parent_object_key].append(object_[str(key)])
                    # The list item we just added might be a list itself
                    # https://github.com/amirziai/flatten/issues/15
                    _convert_dict_to_list(
                        parent_object[parent_object_key][-1],
                        parent_object[parent_object_key],
                        key_index,
                    )

    _convert_dict_to_list(unflattened_dict, None, None)
    return unflattened_dict
