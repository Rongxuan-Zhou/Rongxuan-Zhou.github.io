"""High-resolution orthographic render of the gripper alone, to look for floating or broken parts."""
import mujoco, numpy as np, sys
from PIL import Image
from halftone import Scene, LIGHT
sc = Scene(); m, d = sc.m, sc.d
q = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]); sc.set_q(q, float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
c = sc.tip(); res = 0.0006
xs = c[0] + np.arange(-0.17, 0.17, res); zs = c[2] + np.arange(0.30, -0.05, -res)
img = np.full((len(zs), len(xs), 3), 255, np.uint8); gid = np.zeros(1, np.int32); vec = np.array([0, 1.0, 0])
names = [m.geom(i).name for i in range(m.ngeom)]
rgba = np.array([m.mat_rgba[m.geom_matid[i]] if m.geom_matid[i] >= 0 else m.geom_rgba[i] for i in range(m.ngeom)])
for j, z in enumerate(zs):
    for i, x in enumerate(xs):
        dd = mujoco.mj_ray(m, d, np.array([x, -2.0, z]), vec, sc.groups, 1, -1, gid)
        if dd >= 0: img[j, i] = (rgba[gid[0], :3] * 235 + 10).astype(np.uint8)
Image.fromarray(img).save("gripper_hi.png"); print("tip", np.round(c, 3), img.shape)
