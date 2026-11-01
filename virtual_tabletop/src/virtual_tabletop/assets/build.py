"""Build every generated model (``vtt-build``).

::

    uv run vtt-build            # rig model + RL models (with and without rig)
    uv run vtt-build --clean    # first delete generated files (see below)

Writes ``<cache>/generated/tabletop.xml`` (:mod:`.rig_model`) and
``rl_tabletop.xml`` / ``rl_tabletop_rig.xml`` (:mod:`.rl_model`). Rebuild
after changing anything in ``config/`` or the arm file. ``--clean`` removes
the generated models and the scripted policy's press-path cache; downloads,
converted meshes and the motion library (slow to remake) are kept.
"""

import argparse
import shutil

from virtual_tabletop import paths
from virtual_tabletop.assets.rig_model import build_rig_model
from virtual_tabletop.assets.rl_model import build_rl_model

#: Regenerable outputs removed by ``--clean``.
CLEANABLE = (paths.GENERATED_DIR, paths.CACHE_DIR / "press_paths")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--clean", action="store_true", help="delete generated files first"
    )
    args = parser.parse_args()
    if args.clean:
        for path in CLEANABLE:
            shutil.rmtree(path, ignore_errors=True)
    print(build_rig_model())
    print(build_rl_model())
    print(build_rl_model(include_rig=True))


if __name__ == "__main__":
    main()
