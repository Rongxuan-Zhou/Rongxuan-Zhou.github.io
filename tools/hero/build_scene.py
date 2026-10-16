"""Attach a UMI-like gripper (primitives, real-scale) to the FR3 attachment site and save a combined MJCF."""
import mujoco, re
spec = mujoco.MjSpec.from_file("fr3.xml")
site = next(s for s in spec.sites if s.name == "attachment_site")
link7 = site.parent
g = link7.add_body(name="umi_body", pos=[0, 0, 0.107])
def box(body, name, pos, size, rgba, quat=None):
    geom = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size, rgba=rgba, contype=0, conaffinity=0)
    if quat is not None: geom.quat = quat
    return geom
grey=[0.25,0.25,0.27,1]; orange=[0.95,0.45,0.1,1]; black=[0.08,0.08,0.08,1]
# UMI: flange adapter, main body with the GoPro block on the "top" (+x of the gripper frame), two fingers along +z (approach)
box(g, "umi_adapter", [0,0,0.012], [0.035,0.035,0.012], grey)
box(g, "umi_palm", [0,0,0.045], [0.052,0.034,0.024], black)
box(g, "umi_cam", [0.075,0,0.03], [0.026,0.032,0.026], grey)         # fisheye GoPro block
box(g, "umi_cam_lens", [0.105,0,0.03], [0.006,0.012,0.012], black)
# fingers: two sliders along y, each a flat orange finger extending along +z
for i,sgn in ((1,1),(2,-1)):
    f = g.add_body(name=f"umi_finger{i}", pos=[0, sgn*0.04, 0.069])
    j = f.add_joint(name=f"umi_slide{i}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0,1,0], range=[-0.035,0.0])
    box(f, f"umi_finger{i}_bar", [0,0,0.0], [0.012,0.006,0.012], black)
    box(f, f"umi_finger{i}_pad", [0,sgn*-0.004,0.05], [0.014,0.006,0.045], orange)
link0 = next(bd for bd in spec.bodies if bd.name == "fr3_link0")
led = link0.add_geom(name="led_ring", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0,0,0.295], size=[0.083,0.006,0], rgba=[0.1,1,0.3,1], contype=0, conaffinity=0)
spec.compile()
xml = spec.to_xml()
open("fr3_umi.xml","w").write(xml)
m = mujoco.MjModel.from_xml_path("fr3_umi.xml")
print("geoms", m.ngeom, "joints", m.njnt, "umi geoms:", [m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.startswith("umi")])
