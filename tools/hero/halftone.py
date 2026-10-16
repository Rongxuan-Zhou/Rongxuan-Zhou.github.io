"""Orthographic side-view halftone data from MuJoCo via ray casting: luminance + mask per cell, no OpenGL."""
import mujoco, numpy as np
NX, NY = 160, 100
X0, X1 = -1.05, 0.45          # world x window (m); base at the right
Z0, Z1 = -0.03, 0.9075        # world z window (m); aspect 1.5:0.9375 = 1.6
LIGHT = np.array([-0.5, -0.55, 0.65]); LIGHT /= np.linalg.norm(LIGHT)
class Scene:
    def __init__(self, xml="fr3_umi.xml"):
        self.m = mujoco.MjModel.from_xml_path(xml); self.d = mujoco.MjData(self.m)
        self.orange = np.array([self.m.geom(i).name.endswith("_pad") for i in range(self.m.ngeom)])
        self.tip_body = self.m.body("umi_body").id
        self.xs = X0 + (np.arange(NX) + 0.5) * (X1 - X0) / NX
        self.zs = Z1 - (np.arange(NY) + 0.5) * (Z1 - Z0) / NY      # row 0 at the top
    def set_q(self, q, finger=0.0):
        self.d.qpos[:7] = q; self.d.qpos[7] = -finger; self.d.qpos[8] = -finger
        mujoco.mj_forward(self.m, self.d)
    def tip(self):
        p = self.d.xpos[self.tip_body] + self.d.xmat[self.tip_body].reshape(3,3) @ np.array([0,0,0.114])
        return p
    def to_cell(self, p):
        return ((p[0]-X0)/(X1-X0)*NX, (Z1-p[2])/(Z1-Z0)*NY)
    def render(self):
        m, d = self.m, self.d
        depth = np.full((NY, NX), np.inf); gid = np.full((NY, NX), -1, dtype=int)
        geomid = np.zeros(1, dtype=np.int32); vec = np.array([0.0, 1.0, 0.0])
        for j, z in enumerate(self.zs):
            for i, x in enumerate(self.xs):
                dist = mujoco.mj_ray(m, d, np.array([x, -2.0, z]), vec, None, 1, -1, geomid)
                if dist >= 0: depth[j, i] = dist; gid[j, i] = geomid[0]
        hit = np.isfinite(depth)
        dx = (X1 - X0) / NX; dz = (Z1 - Z0) / NY
        D = np.where(hit, depth, np.nan)
        # one-sided finite differences that ignore background neighbours
        def grad(axis, h):
            fwd = np.roll(D, -1, axis=axis); bwd = np.roll(D, 1, axis=axis)
            gf = (fwd - D) / h; gb = (D - bwd) / h
            g = np.where(np.isfinite(gf) & np.isfinite(gb), 0.5*(gf+gb), np.where(np.isfinite(gf), gf, np.where(np.isfinite(gb), gb, 0.0)))
            return np.nan_to_num(g)
        Dx = grad(1, dx); Dz = -grad(0, dz)          # rows go down, z goes up
        Dx = np.clip(Dx, -3, 3); Dz = np.clip(Dz, -3, 3)
        n = np.stack([2.2*Dx, -np.ones_like(Dx), 2.2*Dz], -1); n /= np.linalg.norm(n, axis=-1, keepdims=True)
        lum = 0.16 + 0.84 * np.clip(n @ LIGHT, 0, None)
        rim = np.clip(1.0 / np.sqrt(1 + Dx**2 + Dz**2), 0, 1)       # how face-on the surface is
        lum = np.where(hit, lum, -1.0)
        mask = np.where(hit, np.where(self.orange[np.clip(gid,0,None)], 2, 1), 0)
        return lum, rim, mask
def preview(lum, rim, mask, path, pitch=4, dark=False, tip=None):
    from PIL import Image, ImageDraw
    W, H = NX*pitch, NY*pitch
    im = Image.new("RGB", (W, H), (20,20,22) if dark else (255,255,255)); dr = ImageDraw.Draw(im)
    rng = np.random.default_rng(0); grain = 0.9 + 0.2*rng.random((NY, NX))
    for j in range(NY):
        for i in range(NX):
            if mask[j,i] == 0: continue
            l = lum[j,i]
            c = (0.06 + 0.94*l**1.7) if dark else (0.07 + 0.83*(1-l)**1.25)
            c *= 0.3 + 0.7*np.sqrt(rim[j,i])
            r = pitch/2*1.02*np.sqrt(c)*grain[j,i]
            if r < 0.28: continue
            cx, cy = (i+0.5)*pitch, (j+0.5)*pitch
            col = (236,112,20) if mask[j,i]==2 else ((230,230,230) if dark else (20,20,20))
            dr.ellipse([cx-r, cy-r, cx+r, cy+r], fill=col)
    if tip is not None:
        tx, ty = tip[0]*pitch, tip[1]*pitch; dr.line([tx-8,ty,tx+8,ty], fill=(53,82,242)); dr.line([tx,ty-8,tx,ty+8], fill=(53,82,242))
    im.save(path)
if __name__ == "__main__":
    import time, sys
    sc = Scene(); t=time.time()
    for name, q, f in [("home", [0,0,0,-1.57079,0,1.57079,-0.7853], 0.0), ("reach", [0.0,-0.6,0.0,-2.3,0.0,1.7,0.8], 0.03)]:
        sc.set_q(np.array(q), f); lum, rim, mask = sc.render()
        tip = sc.to_cell(sc.tip()); print(name, "hit cells", int((mask>0).sum()), "orange", int((mask==2).sum()), "tip cell", np.round(tip,1), "tip world", np.round(sc.tip(),3))
        preview(lum, rim, mask, f"preview_{name}.png", tip=tip)
    print("render time per frame %.2fs" % ((time.time()-t)/2))
