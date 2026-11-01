"""Play a bag back in the MuJoCo rig model.

Interactive::

    uv run vtt-replay BAG

(On macOS, MuJoCo's passive viewer must run under ``mjpython``; ``vtt-replay``
re-launches itself that way when needed.)

Keys: space pause/resume, right/left arrow +/-2 s, N/P next/previous trial,
=/- faster/slower, R restart.

Offscreen video (plain ``python`` is fine)::

    uv run vtt-replay BAG --video out.mp4 [--camera subject] [--start S --end S]

Playback is kinematic: frames set ``qpos`` and mocap poses and call
``mj_forward``; nothing is simulated.
"""

import argparse
import functools
from pathlib import Path

import mujoco
import numpy as np

from virtual_tabletop import viewer
from virtual_tabletop.assets.rig_model import load_rig_model
from virtual_tabletop.assets.sim_config import load_rig_config
from virtual_tabletop.replay.timeline import Timeline, build_timeline


@functools.cache
def _glass_config() -> dict:
    return load_rig_config()["smartglass"]


def apply_frame(
    model: mujoco.MjModel, data: mujoco.MjData, timeline: Timeline, k: int
) -> None:
    """Put frame ``k`` of ``timeline`` into ``data`` and update kinematics."""
    data.qpos[timeline.qpos_adr] = timeline.robot_qpos[k]
    data.mocap_pos[:] = timeline.mocap_pos[k]
    data.mocap_quat[:] = timeline.mocap_quat[k]
    if timeline.glass_revealed is not None:
        glass = _glass_config()
        state = "revealed" if timeline.glass_revealed[k] else "occluded"
        model.geom_rgba[model.geom("smartglass").id] = glass[f"{state}_rgba"]
    mujoco.mj_forward(model, data)


def status_text(timeline: Timeline, k: int, speed: float = 1.0) -> str:
    """One-line description of frame ``k`` for overlays."""
    now = timeline.t[k]
    parts = [f"t={now:7.2f}s", f"x{speed:g}"]
    trial = timeline.trial_at(now)
    if trial is not None:
        parts.append(f"trial {trial.number} {trial.object_id}".rstrip())
    if timeline.glass_revealed is not None:
        parts.append(
            "glass:" + ("clear" if timeline.glass_revealed[k] else "opaque")
        )
    if timeline.reward_active is not None and timeline.reward_active[k]:
        parts.append("REWARD")
    if len(timeline.presses) and np.any(np.abs(timeline.presses - now) < 0.25):
        parts.append("PRESS")
    return "  ".join(parts)


def render_video(
    model: mujoco.MjModel,
    timeline: Timeline,
    output: Path,
    *,
    camera: str = "overview",
    start: float = 0.0,
    end: float | None = None,
    fps: int = 30,
    size: tuple[int, int] = (720, 1280),
) -> Path:
    """Render ``timeline`` between ``start`` and ``end`` to a video file."""
    import imageio.v2 as imageio

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    renderer = mujoco.Renderer(model, *size)
    end = timeline.t[-1] if end is None else end
    times = np.arange(start, end, 1.0 / fps)
    with imageio.get_writer(output, fps=fps, macro_block_size=8) as writer:
        for now in times:
            apply_frame(model, data, timeline, timeline.frame_at(now))
            renderer.update_scene(data, camera=camera)
            writer.append_data(renderer.render())
    renderer.close()
    return output


def run_viewer(model: mujoco.MjModel, timeline: Timeline) -> None:
    """Interactive playback in MuJoCo's passive viewer."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    playback = viewer.Playback()
    state = {"time": 0.0}
    starts = [trial.start for trial in timeline.trials]

    def on_key(key: int) -> None:
        if playback.on_key(key):
            return
        now = state["time"]
        if key == viewer.KEY_RIGHT:
            state["time"] = now + 2.0
        elif key == viewer.KEY_LEFT:
            state["time"] = max(0.0, now - 2.0)
        elif key == viewer.KEY_N:
            state["time"] = next((s for s in starts if s > now + 1e-3), now)
        elif key == viewer.KEY_P:
            state["time"] = next(
                (s for s in reversed(starts) if s < now - 0.5), 0.0
            )
        elif key == viewer.KEY_R:
            state["time"] = 0.0

    def update(wall_dt: float) -> str:
        state["time"] = min(
            state["time"] + playback.advance(wall_dt), timeline.t[-1]
        )
        k = timeline.frame_at(state["time"])
        apply_frame(model, data, timeline, k)
        return status_text(timeline, k, playback.speed)

    viewer.run_passive(model, data, update, on_key=on_key)


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a rig bag in MuJoCo.")
    parser.add_argument("bag", type=Path, help="bag directory or .mcap file")
    parser.add_argument("--video", type=Path, help="render to this file")
    parser.add_argument("--camera", default="overview")
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--end", type=float, default=None)
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args()
    if not args.video:
        viewer.ensure_mjpython("virtual_tabletop.replay.player")

    model = load_rig_model()
    timeline = build_timeline(args.bag, model, rate=args.rate)
    print(
        f"{len(timeline.t)} frames, {timeline.t[-1]:.1f} s, "
        f"{len(timeline.trials)} trials"
    )
    for trial in timeline.trials:
        print(
            f"  trial {trial.number:3d}  {trial.start:7.1f}-{trial.end:7.1f} s"
            f"  {trial.group_name:18s} {trial.object_id}"
        )
    if args.video:
        render_video(
            model,
            timeline,
            args.video,
            camera=args.camera,
            start=args.start,
            end=args.end,
        )
        print(args.video)
    else:
        run_viewer(model, timeline)


if __name__ == "__main__":
    main()
