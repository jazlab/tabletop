# Model Monkey Arm

This model monkey arm was forked and modified from the Saxena Lab muSim
musculoskeletal model (https://github.com/saxenalab-neuro/muSim).

Two right-arm models (the left arm is mirrored from it at build time):

- `right_arm_fingers_v0.xml` (used by default): 7 arm joints plus 16 finger
  joints (thumb abduction + 3 flexions; 3 flexions per other finger), each
  with a joint `<motor>`; no muscles. A `fingertip` site at the middle
  fingertip is the reaching point. Copied from `tabletop_v1` (branch
  `nwatters/modeling`, also on `nwatters/virtual_tabletop`), with the hand
  file `right_hand_v0.xml` inlined (MuJoCo's `<include>` drops the included
  file's top element, which removed the `palm` body) and the site added.
  Hand meshes: `bones/hand_bones/`.
- `right_arm_v0.xml`: 7 joints and 39 muscles, no fingers.
