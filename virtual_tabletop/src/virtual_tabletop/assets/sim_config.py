"""Load the virtual tabletop's own config files (``config/*.yaml``).

- ``rig.yaml``: sim-only rig details (smartglass pane, part colours);
- ``subject.yaml``: the monkey (arm file, shoulder placements for the rig
  and RL models, RL actuation);
- ``buttons.yaml``: response buttons (loaded by
  :mod:`virtual_tabletop.buttons.config`).

Every key and its consumer: ``documentation/configuration.md``.
"""

from pathlib import Path

import yaml

from virtual_tabletop import paths


def load_rig_config(path: Path = paths.RIG_CONFIG) -> dict:
    return yaml.safe_load(path.read_text())


def load_subject_config(path: Path = paths.SUBJECT_CONFIG) -> dict:
    return yaml.safe_load(path.read_text())
