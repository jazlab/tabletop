"""Directory layout of the project (the one place that knows it).

::

    unsupervised_exploration/
      ue/                  the shared package
      research/            literature notes
      version_<v>/         one experiment generation
        configs/  runs/<name>/  logs/  report/{src,figures,videos,*.pdf}
        PLAN.md
      notebooks/  scripts/

A version is named by its suffix (``"0"``, ``"1"``); ``"v1"`` and
``"version_1"`` are accepted too.
"""

from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
RESEARCH = PROJECT / "research"
VERSION_PREFIX = "version_"


def version_dir(version) -> Path:
    """The directory of a version (``0``, ``"1"``, ``"v1"``, ...)."""
    v = str(version)
    for p in (VERSION_PREFIX, "v"):
        if v.startswith(p):
            v = v[len(p):]
    return PROJECT / f"{VERSION_PREFIX}{v}"


def version_of(path) -> Path | None:
    """The version directory containing ``path`` (or None)."""
    for p in [Path(path).resolve(), *Path(path).resolve().parents]:
        if p.name.startswith(VERSION_PREFIX) and p.parent == PROJECT:
            return p
    return None


def runs_dir(version) -> Path:
    return version_dir(version) / "runs"


def logs_dir(version) -> Path:
    return version_dir(version) / "logs"


def report_dir(version) -> Path:
    return version_dir(version) / "report"
