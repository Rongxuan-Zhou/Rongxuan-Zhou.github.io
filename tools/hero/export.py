"""Render every frame (base layer + front-of-letter layer), pack them one above the other, encode MP4 + WebM, write hero.json.
Pixel coding: grey = arm luminance (capped at 240), orange = Fin Ray fingers, green = base LED ring, pure white = nothing.
The trail point of each frame (the TCP: umi_tip, midway between the fingertips) is projected here, on the render grid, so it
always lands on the gripper in the video. --meta-only rewrites hero.json from traj.json without rendering or encoding; the
front-layer flags then come from the existing hero.json."""
import json, numpy as np, time, subprocess, os, shutil, sys
from PIL import Image
from halftone import Scene, NX, NY, X0, X1, Z0, Z1
FF = os.path.expanduser("~/miniconda3/envs/tacforce/bin/ffmpeg")
META_ONLY = "--meta-only" in sys.argv
sc = Scene(); T = json.load(open("traj.json")); F = T["frames"]; n = len(F); TH = T["glyph"]["thickness"]
def encode(lum, mask):
    img = np.full((NY, NX, 3), 255, dtype=np.uint8); L = (np.clip(lum, 0, 0.94) * 255).astype(np.uint8)   # 255 is reserved for "nothing"
    arm, pad, led = mask == 1, mask == 2, mask == 3
    for c in range(3): img[..., c][arm] = L[arm]
    t = (L[pad] / 255.0)[:, None]
    img[pad] = ((np.array([236, 112, 20]) / 255.0 * (1 - t) + np.array([1.0, 0.85, 0.6]) * t) * 255).astype(np.uint8)
    img[led] = (0, 255, 90)
    return img
tips = []
for fr in F:
    sc.set_q(np.array(fr["q"]), fr["f"]); tips.append([round(float(v), 2) for v in sc.to_cell(sc.tip())])
if META_ONLY:
    old = json.load(open("hero.json")); assert old["n"] == n and old["nx"] == NX and old["ny"] == NY, "re-render: the frame set changed"
    has_front = [f["front"] for f in old["frames"]]
else:
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
meta.update({"nx": NX, "ny": NY, "world": {"x0": X0, "x1": X1, "z0": Z0, "z1": Z1}, "n": n,
             "frames": [{"tip": tp, "L": {k: (round(v, 5) if isinstance(v, float) else v) for k, v in f["letter"].items()}, "front": hf}
                        for f, tp, hf in zip(F, tips, has_front)]})
json.dump(meta, open("hero.json", "w"), separators=(",", ":"))
print("json", os.path.getsize("hero.json") // 1024, "KB" + ("" if META_ONLY else "  mp4 %d KB  webm %d KB" % (os.path.getsize("hero.mp4") // 1024, os.path.getsize("hero.webm") // 1024)))
