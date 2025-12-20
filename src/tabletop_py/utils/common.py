from abc import ABCMeta, abstractmethod
from collections.abc import Callable, Hashable, Iterable, Mapping
from typing import Any

import yaml


class BracketedListDumper(yaml.Dumper):
    """
    Custom YAML Dumper that formats scalar sequences as bracketed lists.
    """

    def represent_sequence(self, tag, sequence, flow_style=None):
        """
        Overrides the default represent_sequence to use flow style (bracketed)
        for sequences containing only scalar values.
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
    def __init__(self, *args, **kwargs):
        for tag, fn in self.get_kwarg_constructors().items():
            self._add_mapping_constructor(tag, fn)

        super().__init__(*args, **kwargs)

    def _add_mapping_constructor(self, tag: str, fn: Callable):
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
        return fn(**loader.construct_mapping(node, deep=True))  # type: ignore[reportCallIssue]

    @abstractmethod
    def get_kwarg_constructors(self) -> dict[str, Callable]:
        """Abstract method to return a dictionary mapping tags to constructor functions"""


def yaml_dump_string(d: Any, width: int = 80) -> str:
    return yaml.dump(d, Dumper=BracketedListDumper, width=width)


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
