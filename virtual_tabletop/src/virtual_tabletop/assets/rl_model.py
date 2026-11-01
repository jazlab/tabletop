"""The minimal model for the RL tasks.

The monkey arms (at the ``rl`` placement in ``config/subject.yaml``, driven
by joint position servos from its ``rl_actuation``), floor, lights and
cameras, and one mocap "stimulus" body per object geometry
(``stim/small_object``, ``stim/big_object``) that the env places and
colours each trial, each with its response button
(:mod:`virtual_tabletop.buttons`). No robot arms, planning volumes, object
grid or smartglass. ``include_rig`` adds the rig frame CAD (``rig.step``,
cosmetic).

Written to ``<cache>/generated/rl_tabletop[_rig].xml`` by ``vtt-build``.
"""

from pathlib import Path

import mujoco

from virtual_tabletop import paths
from virtual_tabletop.assets.parts import (
    MeshLibrary,
    add_environment,
    add_home_key,
    add_rig,
    add_subject,
    add_subject_servos,
    add_visual_meshes,
    write_model,
)
from virtual_tabletop.assets.rig_config import load_rig_scene
from virtual_tabletop.assets.sim_config import (
    load_rig_config,
    load_subject_config,
)
from virtual_tabletop.buttons.model import add_buttons

#: Mocap bodies of the RL model that hold the presented object, by mesh.
RL_STIMULI = ("small_object", "big_object")

#: Where unused stimulus bodies wait, out of sight.
RL_PARKED = (0.0, 0.0, -10.0)


def rl_model_path(include_rig: bool = False) -> Path:
    name = "rl_tabletop_rig.xml" if include_rig else "rl_tabletop.xml"
    return paths.GENERATED_DIR / name


def build_rl_spec(
    subject_config: dict | None = None, include_rig: bool = False
) -> mujoco.MjSpec:
    """Assemble the RL model as an (uncompiled) ``MjSpec``."""
    subject_config = subject_config or load_subject_config()
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    library = MeshLibrary(spec)
    add_environment(spec)
    spec.modelname = "tabletop_rl"
    scene = load_rig_scene()
    if include_rig:
        add_rig(
            spec,
            library,
            scene,
            load_rig_config(),
            primitives=False,
            meshes={"rig"},
        )
    meshes = {
        o.mesh_path.resolve().stem: o.mesh_path for o in scene.grid_objects
    }
    for stem in RL_STIMULI:
        body = spec.worldbody.add_body(
            name=f"stim/{stem}", pos=RL_PARKED, mocap=True
        )
        add_visual_meshes(
            body,
            library,
            meshes[stem],
            stem,
            rgba=(0.8, 0.8, 0.8, 1.0),
            conaffinity=1,  # the monkey can touch it
        )
    add_buttons(spec, RL_STIMULI)
    add_subject(spec, subject_config, "rl")
    add_subject_servos(spec, subject_config)
    add_home_key(spec, spec.compile())
    return spec


def build_rl_model(
    output: Path | None = None,
    subject_config: dict | None = None,
    include_rig: bool = False,
) -> Path:
    """Generate the RL model, writing MJCF to ``output``."""
    output = output or rl_model_path(include_rig)
    return write_model(build_rl_spec(subject_config, include_rig), output)


def load_rl_model(include_rig: bool = False) -> mujoco.MjModel:
    """Load the RL model, building it first if it doesn't exist."""
    path = rl_model_path(include_rig)
    if not path.exists():
        build_rl_model(path, include_rig=include_rig)
    return mujoco.MjModel.from_xml_path(str(path))
