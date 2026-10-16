import json, numpy as np, time
from PIL import Image
from halftone import Scene, NX, NY, preview
sc = Scene(); T = json.load(open("traj.json")); F = T["frames"]; n = len(F)
cols = 16; rows = int(np.ceil(n / cols))
atlas = np.zeros((rows*NY, cols*NX, 3), dtype=np.uint8)
t = time.time(); keep = {}
for k, fr in enumerate(F):
    sc.set_q(np.array(fr["q"]), fr["f"]); lum, rim, mask = sc.render()
    lum, rim, mask = lum[:, ::-1], rim[:, ::-1], mask[:, ::-1]        # mirror: base on the left
    r, c = divmod(k, cols); tile = atlas[r*NY:(r+1)*NY, c*NX:(c+1)*NX]
    tile[..., 0] = np.where(mask > 0, np.clip(lum, 0, 1)*255, 0).astype(np.uint8)
    tile[..., 1] = (mask * 85).astype(np.uint8)
    tile[..., 2] = 0   # rim is derived in the browser from the mask's neighbours
    if k in (0, 60, 150, 200, 250, 300, 360, n-1): keep[k] = (lum, rim, mask, [NX - fr["tip"][0], fr["tip"][1]])
Image.fromarray(atlas).save("atlas.png", optimize=True)
meta = {"fps": T["fps"], "nx": NX, "ny": NY, "cols": cols, "n": n, "pick": T["pick"], "place": T["place"], "floor": T["floor"],
        "frames": [{"tip": [round(NX - f["tip"][0],2), f["tip"][1]], "theta": f["theta"], "state": f["state"], "f": round(f["f"],3), "press": f.get("press",0)} for f in F]}
json.dump(meta, open("hero.json","w"), separators=(",",":"))
import os; print("rendered", n, "frames in %.1fs" % (time.time()-t), "atlas", atlas.shape, os.path.getsize("atlas.png")//1024, "KB json", os.path.getsize("hero.json")//1024, "KB")
# contact sheet
tiles = []
for k,(lum,rim,mask,tip) in keep.items():
    preview(lum, rim, mask, f"k_{k}.png", pitch=3, tip=tip); tiles.append(Image.open(f"k_{k}.png"))
w,h = tiles[0].size; sheet = Image.new("RGB", (w*4+30, h*2+10), "gray")
for i,im in enumerate(tiles): sheet.paste(im, ((i%4)*(w+10), (i//4)*(h+10)))
sheet.save("sheet.png"); print("sheet", sheet.size)
