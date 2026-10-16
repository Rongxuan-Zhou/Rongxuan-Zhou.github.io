"""Render every trajectory frame with halftone.py and encode the result as video (MP4 + WebM) plus hero.json.
Pixel coding (decoded in the browser): arm = grey (R=G=B=luminance), orange finger pads = (255, 110, 0) scaled by
luminance, green LED ring = (0, 255, 90), background = pure white. Chroma subsampling blurs edges a little; the decoder
uses hue thresholds so a few boundary cells are the worst case."""
import json, numpy as np, time, subprocess, os, shutil
from PIL import Image
from halftone import Scene, NX, NY, preview
FF = os.path.expanduser("~/miniconda3/envs/tacforce/bin/ffmpeg")
sc = Scene(); T = json.load(open("traj.json")); F = T["frames"]; n = len(F)
shutil.rmtree("frames", ignore_errors=True); os.makedirs("frames")
t = time.time(); keep = {}
for k, fr in enumerate(F):
    sc.set_q(np.array(fr["q"]), fr["f"]); lum, mask = sc.render()
    img = np.full((NY, NX, 3), 255, dtype=np.uint8)
    L = (np.clip(lum, 0, 1) * 255).astype(np.uint8)
    arm = mask == 1; pad = mask == 2; led = mask == 3
    for c in range(3): img[..., c][arm] = L[arm]
    img[..., 0][pad] = np.clip(60 + 0.76 * L[pad], 0, 255).astype(np.uint8); img[..., 1][pad] = (0.43 * L[pad]).astype(np.uint8); img[..., 2][pad] = 0
    img[..., 0][led] = 0; img[..., 1][led] = 255; img[..., 2][led] = 90
    Image.fromarray(img).save(f"frames/f{k:04d}.png")
    if k in (0, 60, 150, 190, 230, 280, 340, n - 1): keep[k] = (lum, mask, fr["tip"])
print("rendered", n, "frames in %.1fs" % (time.time() - t))
fps = T["fps"]
subprocess.run([FF, "-y", "-v", "error", "-framerate", str(fps), "-i", "frames/f%04d.png", "-c:v", "libx264", "-preset", "slow", "-crf", "16",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", "hero.mp4"], check=True)
subprocess.run([FF, "-y", "-v", "error", "-framerate", str(fps), "-i", "frames/f%04d.png", "-c:v", "libvpx-vp9", "-b:v", "0", "-crf", "24",
                "-pix_fmt", "yuv420p", "-row-mt", "1", "hero.webm"], check=True)
meta = {"fps": fps, "nx": NX, "ny": NY, "n": n, "pick": T["pick"], "place": T["place"], "floor": T["floor"],
        "letter_half_cells": T["letter_half_cells"], "letter_half_w_cells": T["letter_half_w_cells"], "floor_angle": T.get("floor_angle", 90),
        "frames": [{"tip": f["tip"], "phi": f["phi"], "state": f["state"], "f": round(f["f"], 3)} for f in F]}
json.dump(meta, open("hero.json", "w"), separators=(",", ":"))
print("mp4", os.path.getsize("hero.mp4") // 1024, "KB  webm", os.path.getsize("hero.webm") // 1024, "KB  json", os.path.getsize("hero.json") // 1024, "KB")
tiles = []
for k, (lum, mask, tip) in keep.items():
    preview(lum, mask, f"k_{k}.png", pitch=3, tip=tip); tiles.append(Image.open(f"k_{k}.png"))
w, h = tiles[0].size; sheet = Image.new("RGB", (w * 4 + 30, h * 2 + 10), "gray")
for i, im in enumerate(tiles): sheet.paste(im, ((i % 4) * (w + 10), (i // 4) * (h + 10)))
sheet.save("sheet.png"); print("sheet", sheet.size)
