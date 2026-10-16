"""Render every frame (base layer + front-of-letter layer), pack them one above the other, encode MP4 + WebM, write hero.json.
Pixel coding: grey = arm luminance (capped at 240), orange = Fin Ray fingers, green = base LED ring, pure white = nothing."""
import json, numpy as np, time, subprocess, os, shutil
from PIL import Image
from halftone import Scene, NX, NY
FF = os.path.expanduser("~/miniconda3/envs/tacforce/bin/ffmpeg")
sc = Scene(); T = json.load(open("traj.json")); F = T["frames"]; n = len(F); TH = T["glyph"]["thickness"]
def encode(lum, mask):
    img = np.full((NY, NX, 3), 255, dtype=np.uint8); L = (np.clip(lum, 0, 0.94) * 255).astype(np.uint8)   # 255 is reserved for "nothing"
    arm, pad, led = mask == 1, mask == 2, mask == 3
    for c in range(3): img[..., c][arm] = L[arm]
    t = (L[pad] / 255.0)[:, None]
    img[pad] = ((np.array([236, 112, 20]) / 255.0 * (1 - t) + np.array([1.0, 0.85, 0.6]) * t) * 255).astype(np.uint8)
    img[led] = (0, 255, 90)
    return img
shutil.rmtree("frames", ignore_errors=True); os.makedirs("frames")
t0 = time.time(); has_front = []
for k, fr in enumerate(F):
    sc.set_q(np.array(fr["q"]), fr["f"]); L = fr["letter"]
    fy = (L["y"] - TH / 2 - 0.001) if L["st"] == "held" else None
    out = sc.render(front_y=fy)
    (lum, mask), (flum, fmask) = (out if fy is not None else (out, (np.full((NY, NX), -1.0), np.zeros((NY, NX), int))))
    has_front.append(bool((fmask > 0).any()))
    Image.fromarray(np.vstack([encode(lum, mask), encode(flum, fmask)])).save(f"frames/f{k:04d}.png")
    if k % 40 == 0: print(f"  frame {k}/{n}  {time.time()-t0:.0f}s", flush=True)
print("rendered", n, "frames in %.0fs; frames with a front layer: %d" % (time.time() - t0, sum(has_front)))
fps = T["fps"]
subprocess.run([FF, "-y", "-v", "error", "-framerate", str(fps), "-i", "frames/f%04d.png", "-c:v", "libx264", "-preset", "slow", "-crf", "15",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", "hero.mp4"], check=True)
subprocess.run([FF, "-y", "-v", "error", "-framerate", str(fps), "-i", "frames/f%04d.png", "-c:v", "libvpx-vp9", "-b:v", "0", "-crf", "22",
                "-pix_fmt", "yuv420p", "-row-mt", "1", "hero.webm"], check=True)
prev = None
for f in F:
    y = f["letter"]["yaw"]
    if prev is not None:
        while y - prev > 180: y -= 360
        while y - prev < -180: y += 360
    f["letter"]["yaw"] = y; prev = y
meta = {k: T[k] for k in ("fps", "t_drop", "t_held2", "t_release", "home", "glyph", "depth", "fall")}
meta.update({"nx": NX, "ny": NY, "world": {"x0": -0.42, "x1": 1.18, "z0": -0.03, "z1": 0.97}, "n": n,
             "frames": [{"tip": f["tip"], "L": {k: (round(v, 5) if isinstance(v, float) else v) for k, v in f["letter"].items()}, "front": hf}
                        for f, hf in zip(F, has_front)]})
json.dump(meta, open("hero.json", "w"), separators=(",", ":"))
print("mp4", os.path.getsize("hero.mp4") // 1024, "KB  webm", os.path.getsize("hero.webm") // 1024, "KB  json", os.path.getsize("hero.json") // 1024, "KB")
