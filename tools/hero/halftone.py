"""Orthographic side-view shading from MuJoCo via ray casting against the visual meshes only:
luminance (albedo x lighting) + material mask per cell. No OpenGL."""
import mujoco, numpy as np
NX, NY = 400, 250
X0, X1 = -0.42, 1.18          # world x window (m); base near the left, arm reaches to the right
Z0, Z1 = -0.03, 0.97          # world z window (m); 1.6 x 1.0
LIGHT = np.array([-0.45, -0.6, 0.65]); LIGHT /= np.linalg.norm(LIGHT)
class Scene:
    def __init__(self, xml="fr3_umi.xml"):
        self.m = mujoco.MjModel.from_xml_path(xml); self.d = mujoco.MjData(self.m)
        m = self.m
        self.orange = np.array([m.geom(i).name.endswith("_pad") for i in range(m.ngeom)])
        self.led = np.array([m.geom(i).name == "led_ring" for i in range(m.ngeom)])
        rgba = np.array([m.mat_rgba[m.geom_matid[i]] if m.geom_matid[i] >= 0 else m.geom_rgba[i] for i in range(m.ngeom)])
        self.albedo = 0.2126*rgba[:,0] + 0.7152*rgba[:,1] + 0.0722*rgba[:,2]
        self.groups = np.zeros(6, dtype=np.uint8); self.groups[[0, 2]] = 1      # my primitives (0) + menagerie visual meshes (2); no collision hulls (3)
        self.tip_body = m.body("umi_body").id
        self.xs = X0 + (np.arange(NX) + 0.5) * (X1 - X0) / NX
        self.zs = Z1 - (np.arange(NY) + 0.5) * (Z1 - Z0) / NY
    def set_q(self, q, finger=0.0):
        self.d.qpos[:7] = q; self.d.qpos[7] = -finger; self.d.qpos[8] = -finger
        mujoco.mj_forward(self.m, self.d)
    def tip(self):
        return self.d.xpos[self.tip_body] + self.d.xmat[self.tip_body].reshape(3,3) @ np.array([0,0,0.114])
    def to_cell(self, p):
        return ((p[0]-X0)/(X1-X0)*NX, (Z1-p[2])/(Z1-Z0)*NY)
    def render(self):
        m, d = self.m, self.d
        depth = np.full((NY, NX), np.inf); gid = np.full((NY, NX), -1, dtype=int)
        geomid = np.zeros(1, dtype=np.int32); vec = np.array([0.0, 1.0, 0.0])
        for j, z in enumerate(self.zs):
            for i, x in enumerate(self.xs):
                dist = mujoco.mj_ray(m, d, np.array([x, -2.0, z]), vec, self.groups, 1, -1, geomid)
                if dist >= 0: depth[j, i] = dist; gid[j, i] = geomid[0]
        hit = np.isfinite(depth)
        dx = (X1 - X0) / NX; dz = (Z1 - Z0) / NY
        D = np.where(hit, depth, np.nan)
        def grad(axis, h):
            fwd = np.roll(D, -1, axis=axis); bwd = np.roll(D, 1, axis=axis)
            gf = (fwd - D) / h; gb = (D - bwd) / h
            g = np.where(np.isfinite(gf) & np.isfinite(gb), 0.5*(gf+gb), np.where(np.isfinite(gf), gf, np.where(np.isfinite(gb), gb, 0.0)))
            return np.nan_to_num(g)
        Dx = np.clip(grad(1, dx), -4, 4); Dz = np.clip(-grad(0, dz), -4, 4)
        n = np.stack([1.6*Dx, -np.ones_like(Dx), 1.6*Dz], -1); n /= np.linalg.norm(n, axis=-1, keepdims=True)
        diffuse = np.clip(n @ LIGHT, 0, None)
        spec = np.clip(n @ LIGHT, 0, None) ** 24
        g = np.clip(gid, 0, None)
        alb = self.albedo[g]
        lum = alb * (0.22 + 0.78 * diffuse) + 0.25 * spec
        lum = np.where(hit, np.clip(lum, 0, 1), -1.0)
        mask = np.where(hit, np.where(self.led[g], 3, np.where(self.orange[g], 2, 1)), 0)
        return lum, mask
def preview(lum, mask, path, pitch=2, tip=None):
    from PIL import Image, ImageDraw
    W, H = NX*pitch, NY*pitch
    im = Image.new("RGB", (W, H), (255,255,255)); dr = ImageDraw.Draw(im)
    for j in range(NY):
        for i in range(NX):
            if mask[j,i] == 0: continue
            c = 0.05 + 0.95*(1-lum[j,i])**1.5
            r = pitch/2*1.05*np.sqrt(c)
            if r < 0.25: continue
            cx, cy = (i+0.5)*pitch, (j+0.5)*pitch
            col = (236,112,20) if mask[j,i]==2 else ((40,210,90) if mask[j,i]==3 else (20,20,20))
            dr.ellipse([cx-r, cy-r, cx+r, cy+r], fill=col)
    if tip is not None:
        tx, ty = tip[0]*pitch, tip[1]*pitch; dr.line([tx-8,ty,tx+8,ty], fill=(53,82,242)); dr.line([tx,ty-8,tx,ty+8], fill=(53,82,242))
    im.save(path)
if __name__ == "__main__":
    import time
    sc = Scene(); t = time.time()
    sc.set_q(np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]), 0.0); lum, mask = sc.render()
    print("factory pose: hit", int((mask>0).sum()), "orange", int((mask==2).sum()), "led", int((mask==3).sum()), "tip cell", np.round(sc.to_cell(sc.tip()),1), "%.2fs" % (time.time()-t))
    preview(lum, mask, "preview_factory.png", pitch=2, tip=sc.to_cell(sc.tip()))
    from PIL import Image
    Image.fromarray((np.clip(np.where(mask>0, lum, 1.0),0,1)*255).astype(np.uint8)).save("lum_factory.png")
