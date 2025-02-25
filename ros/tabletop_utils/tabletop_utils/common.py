from collections.abc import Callable, Coroutine, Hashable, Iterable, Mapping
from inspect import iscoroutinefunction
from typing import Any


def is_iterable(obj: Any) -> bool:
    if isinstance(obj, (str, Mapping)):
        return False
    try:
        iter(obj)
    except Exception:
        return False
    return True


def without_keys(d, keys: Hashable | Iterable[Hashable]):
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


def create_coroutine_wrapper(
    fn: Callable[..., Any],
) -> Callable[..., Coroutine[Any, Any, Any]]:
    """
    Wrap a function in a coroutine.
    """
    if iscoroutinefunction(fn):
        raise ValueError("Function is already a coroutine")
    else:

        async def wrapper(*args, **kwargs):
            nonlocal fn
            return fn(*args, **kwargs)

        return wrapper
