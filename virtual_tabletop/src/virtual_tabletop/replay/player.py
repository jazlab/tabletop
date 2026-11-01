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
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

from virtual_tabletop.assets.build import load_model, load_sim_config
from virtual_tabletop.replay.timeline import Timeline, build_timeline

KEY_SPACE, KEY_RIGHT, KEY_LEFT = 32, 262, 263
KEY_N, KEY_P, KEY_R, KEY_EQUAL, KEY_MINUS = 78, 80, 82, 61, 45


@functools.cache
def _glass_config() -> dict:
    return load_sim_config()["smartglass"]


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
    import mujoco.viewer

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    state = {"time": 0.0, "speed": 1.0, "paused": False}
    starts = [trial.start for trial in timeline.trials]

    def on_key(key: int) -> None:
        now = state["time"]
        if key == KEY_SPACE:
            state["paused"] = not state["paused"]
        elif key == KEY_RIGHT:
            state["time"] = now + 2.0
        elif key == KEY_LEFT:
            state["time"] = max(0.0, now - 2.0)
        elif key == KEY_N:
            state["time"] = next((s for s in starts if s > now + 1e-3), now)
        elif key == KEY_P:
            state["time"] = next(
                (s for s in reversed(starts) if s < now - 0.5), 0.0
            )
        elif key == KEY_EQUAL:
            state["speed"] = min(state["speed"] * 2, 16.0)
        elif key == KEY_MINUS:
            state["speed"] = max(state["speed"] / 2, 0.125)
        elif key == KEY_R:
            state["time"] = 0.0

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as v:
        last = time.monotonic()
        while v.is_running():
            now = time.monotonic()
            if not state["paused"]:
                state["time"] += (now - last) * state["speed"]
            last = now
            state["time"] = min(state["time"], timeline.t[-1])
            k = timeline.frame_at(state["time"])
            # Never call viewer methods (set_texts, sync) while holding its
            # lock: they take the same lock and would deadlock the viewer.
            with v.lock():
                apply_frame(model, data, timeline, k)
            v.set_texts(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    status_text(timeline, k, state["speed"]),
                    "",
                )
            )
            v.sync()
            time.sleep(1 / 60)


def _ensure_mjpython() -> None:
    """On macOS, re-run this command under mjpython (the viewer needs it)."""
    if sys.platform != "darwin" or os.environ.get("VTT_UNDER_MJPYTHON"):
        return
    mjpython = Path(sys.executable).with_name("mjpython")
    os.environ["VTT_UNDER_MJPYTHON"] = "1"
    os.execv(
        mjpython,
        [str(mjpython), "-m", "virtual_tabletop.replay.player", *sys.argv[1:]],
    )


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
        _ensure_mjpython()

    model = load_model()
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
