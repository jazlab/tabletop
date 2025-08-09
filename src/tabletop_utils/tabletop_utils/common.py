from collections.abc import Hashable, Iterable, Mapping
from typing import Any

import yaml
from launch.substitution import Substitution


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


def print_substitutions(context, substitutions: dict[str, Substitution]):
    for name, substitution in substitutions.items():
        print(f"{name}: {substitution.perform(context)}")
