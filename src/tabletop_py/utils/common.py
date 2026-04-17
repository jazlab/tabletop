"""Common utility functions and YAML helpers.

This module provides utility functions and classes for YAML processing
and common data manipulation operations used throughout the tabletop_py
package.

Classes:
    BracketedListDumper: YAML dumper that formats scalar sequences as
        bracketed flow-style lists.
    KwargYamlLoader: Abstract YAML loader for keyword argument-based
        custom constructors.

Functions:
    yaml_dump_string: Dump objects to YAML with bracketed list formatting.
    is_iterable: Check if an object is iterable (excluding strings/mappings).
    without_keys: Create a dictionary copy with specified keys removed.

Example:
    >>> yaml_dump_string({"items": [1, 2, 3]})
    'items: [1, 2, 3]\\n'

    >>> is_iterable([1, 2, 3])
    True

    >>> without_keys({"a": 1, "b": 2, "c": 3}, ["b"])
    {'a': 1, 'c': 3}
"""

from abc import ABCMeta, abstractmethod
from collections.abc import Callable, Hashable, Iterable, Mapping
from typing import Any, MutableMapping

import yaml


class BracketedListDumper(yaml.Dumper):
    """Custom YAML Dumper that formats scalar sequences as bracketed lists.

    When dumping YAML, sequences containing only scalar values (str, int,
    float, bool, None) are rendered in flow style [item1, item2, ...] rather
    than block style with one item per line.

    This produces more compact, readable output for configuration files
    with simple lists.

    Example:
        >>> yaml.dump({"pos": [1.0, 2.0, 3.0]}, Dumper=BracketedListDumper)
        'pos: [1.0, 2.0, 3.0]\\n'
    """

    def represent_sequence(self, tag, sequence, flow_style=None):
        """Represent a sequence, using flow style for scalar-only lists.

        Overrides the default represent_sequence to automatically use
        flow style (bracketed) for sequences containing only scalar values.

        Args:
            tag: YAML tag for the sequence.
            sequence: The sequence to represent.
            flow_style: Optional flow style override. If None, flow style
                is automatically determined based on sequence contents.

        Returns:
            YAML sequence node with appropriate flow style.
        """
        if all(
            isinstance(item, (str, int, float, bool, type(None)))
            for item in sequence
        ):
            return yaml.Dumper.represent_sequence(
                self, tag, sequence, flow_style=True
            )
        else:
            return yaml.Dumper.represent_sequence(
                self, tag, sequence, flow_style=flow_style
            )


class KwargYamlLoader(yaml.SafeLoader, metaclass=ABCMeta):
    """Abstract YAML loader for keyword argument-based custom constructors.

    Provides a framework for creating YAML loaders that can construct
    custom objects from YAML mappings using keyword arguments. Subclasses
    define tag-to-constructor mappings via get_kwarg_constructors().

    This enables declarative YAML configuration that instantiates Python
    objects directly.

    Example:
        class MyLoader(KwargYamlLoader):
            def get_kwarg_constructors(self):
                return {"!MyClass": MyClass}

        # In YAML:
        # !MyClass
        #   param1: value1
        #   param2: value2

    Attributes:
        Inherits from yaml.SafeLoader for secure YAML parsing.
    """

    def __init__(self, *args, **kwargs):
        """Initialize the loader and register custom constructors.

        Args:
            *args: Positional arguments passed to SafeLoader.
            **kwargs: Keyword arguments passed to SafeLoader.
        """
        for tag, fn in self.get_kwarg_constructors().items():
            self._add_mapping_constructor(tag, fn)

        super().__init__(*args, **kwargs)

    def _add_mapping_constructor(self, tag: str, fn: Callable):
        """Register a mapping constructor for a YAML tag.

        Args:
            tag: YAML tag (e.g., "!MyClass") to handle.
            fn: Callable to invoke with mapping contents as kwargs.
        """
        self.add_constructor(
            tag,
            constructor=lambda loader, node: self._mapping_constructor(
                loader,  # type: ignore[reportArgumentType]
                node,  # type: ignore[reportArgumentType]
                fn=fn,
            ),
        )

    @staticmethod
    def _mapping_constructor(
        loader: yaml.SafeLoader,
        node: yaml.nodes.MappingNode,
        *,
        fn: Callable,
    ):
        """Construct an object from a YAML mapping node.

        Args:
            loader: The YAML loader instance.
            node: The mapping node containing constructor arguments.
            fn: Callable to invoke with the mapping as keyword arguments.

        Returns:
            Object returned by fn(**mapping_contents).
        """
        return fn(**loader.construct_mapping(node, deep=True))  # type: ignore[reportCallIssue]

    @abstractmethod
    def get_kwarg_constructors(self) -> dict[str, Callable]:
        """Return a dictionary mapping YAML tags to constructor functions.

        Subclasses must implement this method to define which tags
        trigger which constructors.

        Returns:
            Dictionary mapping tag strings (e.g., "!MyClass") to
            callables that accept keyword arguments.
        """


def yaml_dump_string(d: Any, width: int = 80) -> str:
    """Dump an object to a YAML string with bracketed list formatting.

    Uses BracketedListDumper to produce compact YAML output where
    scalar-only sequences are rendered in flow style.

    Args:
        d: The object to dump to YAML.
        width: Maximum line width for the output (default 80).

    Returns:
        YAML string representation of the object.

    Example:
        >>> yaml_dump_string({"coords": [1.0, 2.0, 3.0]})
        'coords: [1.0, 2.0, 3.0]\\n'
    """
    return yaml.dump(d, Dumper=BracketedListDumper, width=width)


def is_iterable(obj: Any) -> bool:
    """Check if an object is iterable, excluding strings and mappings.

    Returns True for lists, tuples, sets, generators, etc., but False
    for strings and dict-like objects which are technically iterable
    but often need to be treated as single values.

    Args:
        obj: The object to check.

    Returns:
        True if obj is iterable (and not a string or mapping),
        False otherwise.

    Example:
        >>> is_iterable([1, 2, 3])
        True
        >>> is_iterable("hello")
        False
        >>> is_iterable({"a": 1})
        False
    """
    if isinstance(obj, (str, Mapping)):
        return False
    try:
        iter(obj)
    except Exception:
        return False
    return True


def without_keys(d, keys: Hashable | Iterable[Hashable]):
    """Return a new dictionary with the specified keys removed.

    Creates a shallow copy of the dictionary excluding the specified keys.
    All specified keys must exist in the dictionary.

    Args:
        d: The source dictionary.
        keys: A single key or iterable of keys to exclude. Strings are
            treated as single keys, not iterables of characters.

    Returns:
        New dictionary with the specified keys removed.

    Raises:
        ValueError: If any of the specified keys are not in the dictionary.

    Example:
        >>> without_keys({"a": 1, "b": 2, "c": 3}, ["a", "c"])
        {'b': 2}
        >>> without_keys({"x": 10}, "x")
        {}
    """
    if not isinstance(keys, Iterable) or isinstance(keys, str):
        keys = [keys]
    keys_set = set(keys)
    if keys_set.issubset(d.keys()):
        return {x: d[x] for x in d if x not in keys_set}
    else:
        raise ValueError(f"Keys {keys} not found in dictionary {d}")


def dict_update_recursive(d: MutableMapping, u: Mapping):
    for k, v in u.items():
        if isinstance(v, Mapping):
            d[k] = dict_update_recursive(d.get(k, {}), v)
        else:
            d[k] = v
    return d
