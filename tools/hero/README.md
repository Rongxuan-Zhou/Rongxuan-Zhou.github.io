# Hero animation generator

The homepage hero is a Franka FR3 with a UMI-style gripper, drawn in the browser as a fine halftone (one dot per cell)
from a pre-rendered video. The frames are ray-cast against the real FR3 visual meshes (MuJoCo Menagerie `franka_fr3`)
with their materials, so the white links, black joint rings and base details are the real geometry. No OpenGL is needed.

Pipeline (Python with `mujoco`, `numpy`, `Pillow`, plus `ffmpeg`; the menagerie `fr3.xml` and its `assets/` next to the scripts):

1. `build_scene.py` attaches a real-scale UMI-like gripper (primitives, orange finger pads) and a green LED ring to the FR3
   and writes `fr3_umi.xml`.
2. `traj.py` plans the motion in joint space with an orientation-constrained damped-least-squares IK (random restarts).
   The gripper stays in the picture plane: approach angle `phi` (0 down, +90 right, -90 left). Sequence: factory pose,
   idle reaches, grasp the mirrored R from its right side (phi -90), swing to +90 while carrying (a 180-degree flip),
   place it in the headline, release; later grasp it on the floor from above (phi 0) and swing to -90 to put it back
   mirrored. Targets are canvas fractions; a reachability scan picked them. Writes `traj.json`.
3. `export.py` renders every frame with `halftone.py` (orthographic rays along +y against visual geoms only, albedo x
   lighting) to PNG, encodes `hero.mp4` (H.264) and `hero.webm` (VP9) at 400x250, 15 fps, and writes `hero.json`
   (per-frame gripper tip, phi and letter state). Pixel coding: grey = arm luminance, orange = finger pads, green = LED.
   Copy the two videos and the JSON to `images/hero/`.

`js/hero-arm.js` decodes the hidden `#hero-video` frame by frame onto an offscreen canvas, draws the dots on `#hero-arm`
(ink, orange, breathing green), keeps the gripper trace, and moves the DOM letter `#hero-R`. The letter's fall is a
small planar rigid-body simulation in the browser (gravity scaled to the canvas, restitution, settling on its side).
