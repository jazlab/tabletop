# Model monkey arm

`right_arm_fingers_v0.xml` is a right arm: 7 arm joints (shoulder
abduction/flexion/rotation, elbow flexion, radial pronation, wrist
flexion/abduction) plus 16 finger joints (thumb abduction and 3 flexions; 3
flexions per other finger), limits enforced, with a joint `<motor>` each (the
RL model replaces them with position servos) and no muscles. A `fingertip`
site at the middle fingertip is the reaching point. The left arm is mirrored
from it at build time (`assets/subject.py`); placement and actuation are in
`config/subject.yaml`.

Bone meshes (`bones/`, git LFS) were forked from the Saxena Lab muSim
musculoskeletal model (https://github.com/saxenalab-neuro/muSim); the hand
meshes are in `bones/hand_bones/`. The arm came from earlier virtual-tabletop
work (branch `nwatters/virtual_tabletop`), with its hand file inlined
(MuJoCo's `<include>` drops the included file's top element, which removed
the `palm` body) and the fingertip site added. An older muscle-driven arm
(39 muscles, no fingers) was removed on 2026-10-31; it is in git history.
