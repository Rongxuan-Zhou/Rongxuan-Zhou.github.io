"""Rasterize the headline's "R" (Raleway 800, the font the page uses) and derive its physical model:
ink box, centroid, area moments, and three convex parts (stem, bowl, leg) for contact. Units: metres, ink height 70 mm.
Frame: origin at the ink-box centre, x right, z up (world), letter upright."""
import numpy as np, json
from PIL import Image, ImageDraw, ImageFont
H_M = 0.070                                  # ink height of the R in the world
font = ImageFont.truetype("raleway800.ttf", 1600)
img = Image.new("L", (1800, 1800), 0); ImageDraw.Draw(img).text((100, 0), "R", fill=255, font=font)
a = np.array(img) > 127
ys, xs = np.nonzero(a); x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
a = a[y0:y1, x0:x1]; hpx, wpx = a.shape; s = H_M / hpx                     # metres per pixel
W_M = wpx * s
yy, xx = np.nonzero(a)
X = (xx + 0.5) * s - W_M / 2; Z = H_M / 2 - (yy + 0.5) * s                  # centred, z up
A = a.sum() * s * s
cx, cz = X.mean(), Z.mean()
Izz = ((X - cx) ** 2).sum() * s * s; Ixx = ((Z - cz) ** 2).sum() * s * s    # area moments (m^4)
def hull(px, pz):
    P = sorted(set(zip(np.round(px, 6), np.round(pz, 6))))
    def cross(o, a_, b): return (a_[0]-o[0])*(b[1]-o[1]) - (a_[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]
# pixel corners for exact hulls
def corners(mask_rows, mask_cols):
    pts_x, pts_z = [], []
    for dx in (0, 1):
        for dz in (0, 1):
            pts_x.append((mask_cols + dx) * s - W_M / 2); pts_z.append(H_M / 2 - (mask_rows + dz) * s)
    return np.concatenate(pts_x), np.concatenate(pts_z)
# split: stem = columns left of the first column where the bowl's hole starts; bowl = rows above the leg's start; leg = rest
col_fill = a.mean(axis=0)
stem_right = int(np.argmax(col_fill < 0.80 * col_fill.max())) if (col_fill < 0.80 * col_fill.max()).any() else wpx // 3
row_has_hole = []
for r in range(hpx):
    row = a[r, stem_right:]
    filled = np.nonzero(row)[0]
    row_has_hole.append(len(filled) > 0 and (filled.max() - filled.min() + 1) > len(filled) + 2)
row_has_hole = np.array(row_has_hole)
hole_rows = np.nonzero(row_has_hole[: int(0.75 * hpx)])[0]
bowl_bottom = int(hole_rows.max() + 0.10 * hpx) if len(hole_rows) else int(0.55 * hpx)
parts = {}
for name, sel in (("stem", xx < stem_right), ("bowl", (xx >= stem_right - 2) & (yy <= bowl_bottom)), ("leg", (xx >= stem_right - 2) & (yy > bowl_bottom - 4))):
    px, pz = corners(yy[sel], xx[sel]); parts[name] = hull(px, pz)
out = {"height": H_M, "width": W_M, "area": A, "centroid": [cx, cz], "Ixx_area": Ixx, "Izz_area": Izz,
       "parts": {k: [list(map(float, p)) for p in v] for k, v in parts.items()}, "stem_right_frac": stem_right / wpx, "bowl_bottom_frac": bowl_bottom / hpx}
json.dump(out, open("glyph.json", "w"), indent=1)
print("R: %.1f x %.1f mm, area %.0f mm^2, centroid (%.1f, %.1f) mm from box centre; stem ends at %.0f%% width; bowl ends at %.0f%% height" % (
      W_M*1e3, H_M*1e3, A*1e6, cx*1e3, cz*1e3, 100*stem_right/wpx, 100*bowl_bottom/hpx))
print({k: len(v) for k, v in parts.items()}, "hull vertices")
# preview of the parts
pv = Image.new("RGB", (wpx // 4, hpx // 4), "white"); d = ImageDraw.Draw(pv)
for k, col in (("stem", (200, 60, 60)), ("bowl", (60, 160, 60)), ("leg", (60, 60, 200))):
    d.polygon([((p[0] + W_M/2)/s/4, (H_M/2 - p[1])/s/4) for p in parts[k]], outline=col)
small = Image.fromarray((a[::4, ::4] * 60).astype(np.uint8)).convert("RGB")
Image.blend(small.resize(pv.size), pv, 0.6).save("glyph_parts.png")
