"""FR3 + the UMI robot-side gripper as deployed on Franka (after the UMI teaser photo): flange adapter, mount plate,
parallel-gripper body, front jaw guide, two jaws with white 3D-printed finger mounts, orange Fin Ray fingers
(straight inner wall, slanted back wall, base plate, tip cap, 9 ribs) with black fingertip pads, and a GoPro with its lens
on the camera-facing side; the GoPro's side mirrors are left out (at this scale they read as two stray plates). Every part
touches its neighbour. Gripper frame: z = approach, y = closing axis, x = away from the viewer. Plus the base LED ring at the link0/link1 seam and two cables leaving the base."""
import mujoco, numpy as np
spec = mujoco.MjSpec.from_file("fr3.xml")
site = next(s for s in spec.sites if s.name == "attachment_site")
link7 = site.parent
G = mujoco.mjtGeom
def add(body, name, typ, pos, size, rgba, quat=None):
    g = body.add_geom(name=name, type=typ, pos=list(map(float, pos)), size=list(map(float, size)), rgba=rgba, contype=0, conaffinity=0)
    if quat is not None: g.quat = list(map(float, quat))
    return g
def box(b, n, p, s, c, q=None): return add(b, n, G.mjGEOM_BOX, p, s, c, q)
def cyl(b, n, p, r, h, c, q=None): return add(b, n, G.mjGEOM_CYLINDER, p, [r, h, 0], c, q)
def qx(a): return [np.cos(a/2), np.sin(a/2), 0, 0]
def seg(b, n, ya, za, yb, zb, half_t, half_w, c):          # a wall in the y-z plane from (ya,za) to (yb,zb)
    L = np.hypot(yb - ya, zb - za); ang = np.arctan2(-(yb - ya), zb - za)        # rotate the box's z axis onto the segment
    return box(b, n, [0, (ya + yb)/2, (za + zb)/2], [half_w, half_t, L/2 + half_t], c, qx(ang))
K = lambda v: [v, v, v, 1]
dark, body_c, metal, black, white, orange, glass, mirror = K(0.22), K(0.15), K(0.42), K(0.05), K(0.95), [0.95, 0.42, 0.08, 1], K(0.35), K(0.8)
g = link7.add_body(name="umi_body", pos=[0, 0, 0.107])
cyl(g, "umi_adapter", [0, 0, 0.005], 0.0315, 0.005, dark)                          # z 0.000-0.010
box(g, "umi_plate", [0, 0, 0.015], [0.032, 0.040, 0.005], dark)                    # z 0.010-0.020
box(g, "umi_gripper", [0, 0, 0.056], [0.026, 0.073, 0.036], body_c)                # z 0.020-0.092, 146 mm along the closing axis
box(g, "umi_guide", [0, 0, 0.096], [0.020, 0.070, 0.004], metal)                   # z 0.092-0.100
# GoPro on the viewer side (-x), touching the gripper body
box(g, "umi_gopro", [-0.043, 0, 0.056], [0.017, 0.0355, 0.026], black)             # x -0.060..-0.026, z 0.030-0.082
cyl(g, "umi_lens", [-0.043, 0, 0.086], 0.011, 0.004, glass)                         # z 0.082-0.090
# fingers: body origin on the inner (gripping) plane at full open; positive slide closes
Y_OPEN, Z0, Z1 = 0.045, 0.134, 0.242
for i, sg, tag in ((1, -1, "a"), (2, 1, "b")):
    f = g.add_body(name=f"umi_finger_{tag}", pos=[0, sg*Y_OPEN, 0.100])
    f.add_joint(name=f"umi_slide{i}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, -sg, 0], range=[0, 0.02])
    box(f, f"umi_{tag}_jaw", [0, sg*0.012, -0.004], [0.016, 0.012, 0.005], metal)            # z 0.091-0.101 (inside the guide)
    box(f, f"umi_{tag}_mount", [0, sg*0.014, 0.017], [0.014, 0.014, 0.017], white)           # z 0.100-0.134
    z0, z1 = Z0 - 0.100, Z1 - 0.100                                                           # finger local z range
    t = 0.0013; w = 0.0125
    box(f, f"umi_{tag}_fin_base", [0, sg*0.014, z0 + t], [w, 0.014, t], orange)               # closes the triangle at the base
    box(f, f"umi_{tag}_fin_inner", [0, sg*t, (z0 + z1)/2], [w, t, (z1 - z0)/2], orange)       # straight inner wall
    seg(f, f"umi_{tag}_fin_back", sg*(0.028 - t), z0, sg*(2*t), z1, t, w, orange)             # slanted back wall
    box(f, f"umi_{tag}_fin_tip", [0, sg*0.002, z1], [w, 0.002, 0.0025], orange)               # tip cap
    for k in range(9):                                                                        # ribs, tilted 12 deg toward the base
        z = z0 + (k + 0.6) * (z1 - z0) / 9.6
        yb = 0.028 - 0.026 * (z - z0) / (z1 - z0)                                             # back wall position at this z
        seg(f, f"umi_{tag}_fin_rib{k}", sg*t, z, sg*(yb - t), z - 0.12*(yb - t), 0.0008, w, orange)
    box(f, f"umi_{tag}_pad", [0, -sg*0.0012, z1 - 0.024], [w, 0.0012, 0.022], black)          # rubber pad on the gripping face
link7.add_site(name="umi_tip", pos=[0, 0, 0.107 + Z1])
# LED ring at the base/shoulder seam
link0 = next(bd for bd in spec.bodies if bd.name == "fr3_link0")
cyl(link0, "led_ring", [0, 0, 0.1405], 0.0595, 0.004, [0.1, 1, 0.3, 1])
# cables from the base connector panel, down and off the bottom of the picture
world = spec.worldbody
pts = [(-0.150, 0.0, 0.055), (-0.200, 0.0, 0.045), (-0.240, 0.0, 0.010), (-0.262, 0.0, -0.05), (-0.270, 0.0, -0.15), (-0.272, 0.0, -0.40)]
for nm, dy, r, rgba in (("power", -0.010, 0.0055, black), ("ethernet", 0.012, 0.0032, K(0.25))):
    for k in range(len(pts) - 1):
        a = np.array(pts[k]) + [0, dy, 0]; b = np.array(pts[k+1]) + [0, dy, 0]
        world.add_geom(name=f"cable_{nm}_{k}", type=G.mjGEOM_CAPSULE, fromto=list(map(float, [*a, *b])), size=[r, 0, 0], rgba=rgba, contype=0, conaffinity=0)
box(world, "cable_plug", [-0.146, -0.010, 0.055], [0.012, 0.009, 0.009], black)
spec.compile(); open("fr3_umi.xml", "w").write(spec.to_xml())
m = mujoco.MjModel.from_xml_path("fr3_umi.xml")
print("geoms", m.ngeom, "joints", m.njnt, "umi geoms", sum(1 for i in range(m.ngeom) if m.geom(i).name.startswith("umi")), "tip site", m.site("umi_tip").id)
