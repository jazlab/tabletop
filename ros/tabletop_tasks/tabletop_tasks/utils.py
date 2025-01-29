from typing import Hashable, Iterable


def without_keys(d, keys: Iterable[Hashable]):
    """
    Return a new dictionary with the specified keys removed.
    """
    keys_set = set(keys)
    if keys_set.issubset(d.keys()):
        return {x: d[x] for x in d if x not in keys_set}

    keys_set = set(keys)
    if keys_set.issubset(d.keys()):
        return {x: d[x] for x in d if x not in keys_set}

    raise ValueError(f"Keys {keys} not found in dictionary {d}")
