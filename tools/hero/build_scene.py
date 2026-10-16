"""FR3 + a UMI-style gripper built from primitives at real scale, the base LED ring at the link0/link1 seam,
and two cables leaving the base. Gripper frame: z = approach (flange axis), fingers close along local y."""
import mujoco, numpy as np
spec = mujoco.MjSpec.from_file("fr3.xml")
site = next(s for s in spec.sites if s.name == "attachment_site")
link7 = site.parent
def box(body, name, pos, size, rgba, quat=None):
    g = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size, rgba=rgba, contype=0, conaffinity=0)
    if quat is not None: g.quat = quat
    return g
def cyl(body, name, pos, r, h, rgba, quat=None):
    g = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=pos, size=[r, h, 0], rgba=rgba, contype=0, conaffinity=0)
    if quat is not None: g.quat = quat
    return g
def quat_from_axis_angle(axis, ang):
    axis = np.asarray(axis, float); axis /= np.linalg.norm(axis); s = np.sin(ang/2)
    return [np.cos(ang/2), axis[0]*s, axis[1]*s, axis[2]*s]
dark = [0.12, 0.12, 0.13, 1]; grey = [0.35, 0.35, 0.37, 1]; black = [0.05, 0.05, 0.05, 1]; orange = [0.95, 0.45, 0.10, 1]; glass = [0.6, 0.65, 0.7, 1]
g = link7.add_body(name="umi_body", pos=[0, 0, 0.107])
cyl(g, "umi_adapter", [0, 0, 0.006], 0.036, 0.006, grey)
box(g, "umi_palm", [0, 0, 0.034], [0.046, 0.030, 0.022], dark)                 # printed body
box(g, "umi_bridge", [0, 0, 0.060], [0.046, 0.012, 0.006], dark)               # finger rail
# GoPro block beside the fingers, lens looking along the approach, with the two side mirrors
box(g, "umi_cam", [0.078, 0, 0.040], [0.016, 0.030, 0.028], grey)
box(g, "umi_lens", [0.078, 0, 0.072], [0.010, 0.010, 0.004], black)
for i, sgn in ((1, 1), (2, -1)):
    box(g, f"umi_mirror{i}", [0.082, sgn*0.046, 0.075], [0.014, 0.001, 0.016], glass, quat=quat_from_axis_angle([0, 0, 1], sgn*0.6))
# Fin Ray fingers: a straight pad wall facing the object, a slanted back wall, and ribs between them.
FIN_Z0, FIN_Z1, FIN_W = 0.066, 0.156, 0.012                                     # 90 mm long, 24 mm wide
for i, sgn, tag in ((1, -1, "near"), (2, 1, "far")):                            # near = toward the camera (-y)
    f = g.add_body(name=f"umi_{tag}_finger", pos=[0, sgn*0.040, 0])
    f.add_joint(name=f"umi_slide{i}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0], range=[-0.035, 0.0])
    box(f, f"umi_{tag}_base", [0, 0, 0.060], [0.010, 0.009, 0.008], black)
    L = FIN_Z1 - FIN_Z0; zc = 0.5*(FIN_Z0 + FIN_Z1)
    box(f, f"umi_{tag}_pad", [0, sgn*-0.0012, zc], [FIN_W, 0.0012, L/2], orange)                    # inner wall
    back_y0, back_y1 = sgn*0.020, sgn*0.0025                                                         # slanted outer wall
    ang = np.arctan2(back_y1 - back_y0, L)
    box(f, f"umi_{tag}_back", [0, 0.5*(back_y0+back_y1), zc], [FIN_W, 0.0012, 0.5*np.hypot(L, back_y1-back_y0)], orange, quat=quat_from_axis_angle([1, 0, 0], ang))
    for k in range(6):                                                                                # ribs
        z = FIN_Z0 + (k + 0.5) * L / 6
        yb = back_y0 + (back_y1 - back_y0) * (z - FIN_Z0) / L
        box(f, f"umi_{tag}_rib{k}", [0, 0.5*(yb + sgn*-0.0012), z], [FIN_W, abs(yb - sgn*-0.0012)/2, 0.0009], orange)
# LED ring at the base/shoulder seam
link0 = next(bd for bd in spec.bodies if bd.name == "fr3_link0")
cyl(link0, "led_ring", [0, 0, 0.1405], 0.0595, 0.004, [0.1, 1, 0.3, 1])
# cables from the base connectors, down and off the bottom of the picture
world = spec.worldbody
pts = [(-0.150, 0.0, 0.055), (-0.200, 0.0, 0.045), (-0.240, 0.0, 0.010), (-0.262, 0.0, -0.05), (-0.270, 0.0, -0.15), (-0.272, 0.0, -0.40)]
for c, (dy, r, rgba, nm) in enumerate(((-0.010, 0.0055, black, "power"), (0.012, 0.0032, [0.25, 0.25, 0.28, 1], "ethernet"))):
    for k in range(len(pts) - 1):
        a = np.array(pts[k]) + [0, dy, 0]; b = np.array(pts[k+1]) + [0, dy, 0]
        world.add_geom(name=f"cable_{nm}_{k}", type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[*a, *b], size=[r, 0, 0], rgba=rgba, contype=0, conaffinity=0)
box(world, "cable_plug", [-0.146, -0.010, 0.055], [0.012, 0.009, 0.009], black)
spec.compile(); open("fr3_umi.xml", "w").write(spec.to_xml())
m = mujoco.MjModel.from_xml_path("fr3_umi.xml")
print("geoms", m.ngeom, "joints", m.njnt, "near-finger geoms", sum(1 for i in range(m.ngeom) if m.geom(i).name.startswith("umi_near")), "cables", sum(1 for i in range(m.ngeom) if m.geom(i).name.startswith("cable")))
