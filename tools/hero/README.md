# Hero animation generator

The homepage hero is a Franka FR3 with a UMI-style gripper, drawn as a halftone dot grid from frames
ray-cast against the real FR3 geometry (MuJoCo Menagerie `franka_fr3`). No OpenGL is needed.

Pipeline (Python with `mujoco`, `numpy`, `Pillow`; the menagerie `fr3.xml` and its `assets/` next to the scripts):

1. `build_scene.py` attaches a real-scale UMI-like gripper (primitives, orange finger pads) to the FR3 and writes `fr3_umi.xml`.
2. `traj.py` plans the motion in joint space with an orientation-constrained damped-least-squares IK:
   three idle reaches, then pick the mirrored letter, flip it (roll +90 to -90 deg) and place it. Writes `traj.json`.
   Pick/place/idle targets are given as canvas fractions; the canvas is mirrored so the base sits on the left.
3. `export.py` renders every frame with `halftone.py` (orthographic rays along +y, normals from depth differences,
   lighting from the upper left) into `atlas.png` (R = luminance, G = mask 0/127/254, B = how face-on) plus `hero.json`.
   Convert the atlas to lossless WebP and copy both to `images/hero/`.

`js/hero-arm.js` plays the atlas on `#hero-arm`, draws the gripper trace, and moves the DOM letter `#hero-R`
between its wall position, the gripper tip and its place in the headline.
