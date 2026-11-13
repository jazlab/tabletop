"""Videos that show the hand versions' joint ranges (used by
notebooks/hand_ranges_demo.ipynb).

The arm hangs at rest, the object is parked out of view, and the hand is
posed kinematically (no physics), so the videos show exactly what each
version allows. For one joint, both versions sweep the same angle profile
through the *version 0* range: the version 1 hand is clamped at its own
limits, so it visibly stops where the new limit is. The camera looks
along the joint's rotation axis, so flexion is seen in profile.
"""

import base64

import imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from ..sim.hand import get_hand
from ..sim.model import PREFIX, PlayModel

FPS = 25


class HandPoser:
    """A hand version's model, posed kinematically and rendered from a
    camera that looks at the hand."""

    def __init__(self, hand: str, size=(360, 360)):
        self.pm = PlayModel(2, 0, hand=hand)
        self.hand = get_hand(hand)
        self.m = self.pm.model
        self.d = mujoco.MjData(self.m)
        self.names = [n.removeprefix(PREFIX) for n in self.pm.joint_names]
        self.renderer = mujoco.Renderer(self.m, *size)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.palm = self.m.body(PREFIX + "palm").id
        self.followers = {
            self.names.index(f): (self.names.index(lead), r)
            for f, lead, r in self.hand.couplings}

    def range_deg(self, joint: str) -> tuple[float, float]:
        j = self.m.joint(PREFIX + joint).id
        lo, hi = np.degrees(self.m.jnt_range[j])
        return float(lo), float(hi)

    def pose(self, angles_deg: dict) -> dict:
        """Set joint angles (degrees; clamped to this version's range).
        Coupled followers are set from their leaders. Returns the angles
        actually applied."""
        self.d.qpos[:] = self.m.qpos0
        applied = {}
        for name, a in angles_deg.items():
            lo, hi = self.range_deg(name)
            applied[name] = float(np.clip(a, lo, hi))
            jid = self.m.joint(PREFIX + name).id
            self.d.qpos[self.m.jnt_qposadr[jid]] = np.radians(applied[name])
        for f, (lead, ratio) in self.followers.items():
            ql = self.d.qpos[self.pm.qadr[lead]]
            lo, hi = self.m.jnt_range[self.m.joint(
                PREFIX + self.names[f]).id]
            self.d.qpos[self.pm.qadr[f]] = np.clip(ratio * ql, lo, hi)
            applied[self.names[f]] = float(np.degrees(
                self.d.qpos[self.pm.qadr[f]]))
        mujoco.mj_forward(self.m, self.d)
        return applied

    def look(self, joint: str | None, distance=0.085, view=None):
        """Aim the camera at the hand along ``joint``'s axis (or ``view``,
        a world-frame viewing direction)."""
        if view is None:
            jid = self.m.joint(PREFIX + joint).id
            v = self.d.xaxis[jid].copy()
        else:
            v = np.asarray(view, float)
        v /= np.linalg.norm(v)
        self.cam.lookat[:] = (self.d.geom_xpos[self.pm.palm_geom]
                              + 0.4 * (self.d.geom_xpos[self.pm.tip_geoms[2]]
                                       - self.d.geom_xpos[self.pm.palm_geom]))
        self.cam.distance = distance
        self.cam.azimuth = float(np.degrees(np.arctan2(v[1], v[0])))
        self.cam.elevation = float(np.degrees(np.arcsin(np.clip(v[2], -1,
                                                                1))))

    def render(self) -> np.ndarray:
        self.renderer.update_scene(self.d, camera=self.cam)
        return self.renderer.render()


def _label(img, lines, colour=(255, 255, 255), bar=(0, 0, 0)):
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, im.width, 14 * len(lines) + 4], fill=bar)
    for i, t in enumerate(lines):
        dr.text((4, 2 + 14 * i), t, fill=colour)
    return np.asarray(im)


def sweep_profile(lo, hi, seconds=4.0):
    """0 -> hi -> lo -> 0 (degrees), smooth, ``seconds`` long."""
    n = int(seconds * FPS)
    t = np.linspace(0, 1, n)
    a = np.where(t < 0.25, hi * np.sin(t / 0.25 * np.pi / 2),
                 np.where(t < 0.75,
                          hi + (lo - hi) * (1 - np.cos((t - 0.25) / 0.5
                                                       * np.pi)) / 2,
                          lo * np.cos((t - 0.75) / 0.25 * np.pi / 2)))
    return a


def joint_video(joint: str, posers: dict, base_pose=None) -> list:
    """Side-by-side frames: each version sweeps ``joint`` through the
    widest (version 0) range; each panel is labelled with the applied
    angle, the version's range, and LIMIT while clamped."""
    base_pose = base_pose or {}
    lo = min(p.range_deg(joint)[0] for p in posers.values())
    hi = max(p.range_deg(joint)[1] for p in posers.values())
    frames = []
    for a in sweep_profile(lo, hi):
        tiles = []
        for name, p in posers.items():
            applied = p.pose({**base_pose, joint: a})
            p.look(joint)
            r = p.range_deg(joint)
            at_limit = abs(applied[joint] - a) > 0.5
            tiles.append(_label(p.render(), [
                f"{name}: {joint}",
                f"angle {applied[joint]:+6.1f} deg   range "
                f"[{r[0]:+.0f}, {r[1]:+.0f}]" + ("   LIMIT" if at_limit
                                                  else "")],
                bar=(150, 30, 30) if at_limit else (0, 0, 0)))
        frames.append(np.concatenate(tiles, 1))
    return frames


def sequence_video(keyframes: list[tuple[str, dict]], posers: dict,
                   view_joint: str | None = None, view=None,
                   seconds_per=1.5) -> list:
    """Interpolate between named whole-hand poses (degrees), for every
    version side by side."""
    frames = []
    n = int(seconds_per * FPS)
    for (n0, k0), (n1, k1) in zip(keyframes[:-1], keyframes[1:]):
        keys = set(k0) | set(k1)
        for s in np.linspace(0, 1, n):
            w = (1 - np.cos(s * np.pi)) / 2
            pose = {k: (1 - w) * k0.get(k, 0) + w * k1.get(k, 0)
                    for k in keys}
            tiles = []
            for name, p in posers.items():
                p.pose(pose)
                p.look(view_joint, view=view)
                tiles.append(_label(p.render(), [f"{name}: {n0} -> {n1}"]))
            frames.append(np.concatenate(tiles, 1))
    return frames


def to_html_video(frames: list, width=None) -> str:
    """An inline (base64 mp4) HTML5 video, for notebooks."""
    import io
    buf = io.BytesIO()
    imageio.mimsave(buf, frames, format="mp4", fps=FPS, macro_block_size=1)
    b64 = base64.b64encode(buf.getvalue()).decode()
    w = f' width="{width}"' if width else ""
    return (f'<video controls loop autoplay muted{w}>'
            f'<source src="data:video/mp4;base64,{b64}" type="video/mp4">'
            f'</video>')
