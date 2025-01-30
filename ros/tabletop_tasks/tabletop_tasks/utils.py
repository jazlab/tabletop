from typing import Hashable, Iterable


def without_keys(d, keys: Iterable[Hashable]):
    """
    Return a new dictionary with the specified keys removed.
    """

    if not isinstance(keys, Iterable) or isinstance(keys, str):
        keys = [keys]
    keys_set = set(keys)
    if keys_set.issubset(d.keys()):
        return {x: d[x] for x in d if x not in keys_set}
    else:
        raise ValueError(f"Keys {keys} not found in dictionary {d}")
