# Hero animation generator

The homepage hero is a Franka FR3 with the UMI robot-side gripper, rendered offline from the real FR3 meshes
(MuJoCo Menagerie `franka_fr3`) by ray casting (no OpenGL) and drawn in the browser as a fine halftone.

Story: the "R" of the headline starts mirrored at its home. The arm grasps it from above, pulls it off the glass, flips it
180 degrees about the vertical axis, the R slips at the end of the flip and falls, the arm reacts after 120 ms, tracks it,
grabs it on the floor, seats it at home and lets go; the page then turns the arm into mist that drifts past the name and off
the page. Plays once.

Requirements: Python with `mujoco`, `numpy`, `Pillow`; `ffmpeg`; the menagerie `fr3.xml` and its `assets/` next to the scripts;
the page font for the R: `raleway800.ttf` (Raleway ExtraBold, from the Google Fonts CSS for `Raleway:800`).

1. `glyph.py` rasterizes the R and derives its physical model (70 mm ink height): ink box, centroid, area moments, and three
   convex parts (stem, bowl, leg) for contact. Writes `glyph.json`.
2. `build_scene.py` adds the UMI gripper as deployed on Franka (after the UMI teaser photo) as one connected assembly: flange
   adapter, mount plate, gripper body, jaw guide, jaws, white finger mounts, orange Fin Ray fingers (inner wall, slanted back
   wall, base, tip cap, 9 ribs) with black pads, GoPro with lens; plus the base LED ring at the link0/link1
   seam and two cables. Writes `fr3_umi.xml`. `gripper_check.py` renders the gripper at 0.6 mm to look for gaps.
3. `ikcore.py`: damped-least-squares IK on the fingertip site with a full orientation target (in-plane tilt `phi`, yaw `psi`).
4. `fall.py`: the R's drop in MuJoCo, constrained to a plane parallel to the glass; acrylic, restitution about 0.4.
5. `traj.py` plans the sequence as IK knots, runs the drop from the release pose, plans the floor grasp from the rest pose,
   and interpolates joints with PCHIP. Writes `traj.json` (per-frame robot state and letter pose, plus the 240 Hz fall track).
6. `export.py` renders every frame with `halftone.py` (2x2 supersampled, 560x350 cells) twice: the scene, and the gripper parts
   in front of the letter during the flip. The two are stacked vertically and encoded as `hero.mp4` / `hero.webm` with
   `hero.json`. Pixel coding: grey = luminance (<= 240), orange = Fin Ray fingers, green = LED ring, pure white = nothing.
   `hero.json` also holds the per-frame trail point: the TCP (`umi_tip`, midway between the fingertips), projected on the
   same render grid as the video. `python export.py --meta-only` rewrites only `hero.json` (no rendering).
   Copy the three files to `images/hero/`.

`js/hero-arm.js` decodes the hidden `#hero-video`, stamps the dots into `#hero-arm` (below the text) and `#hero-front`
(above it), draws the floor line and the cable continuation, sizes the headline so the R is 70 mm in world scale, builds the
R as a CSS 3D block (20 stacked glyph layers, perspective centred on its home), replays the letter pose, and runs the dissolve.
