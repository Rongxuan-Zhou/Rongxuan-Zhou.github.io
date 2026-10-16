"""Hero motion: the R starts mirrored at its home in the headline; the arm grasps it from above, pulls it off the glass,
flips it 180 deg about the vertical, the R slips at the end of the flip and falls (MuJoCo rigid-body drop), the arm reacts
after 120 ms, tracks it down, grasps it on the floor, carries it home, seats it on the glass and lets go.
Joint trajectory: PCHIP through IK knots (continuous velocity, no overshoot). Output: per-frame robot state and letter pose."""
import numpy as np, json
from ikcore import sc, m, d, ik, target_rot, cell_to_world, world_to_cell, lo, hi
from fall import drop
FPS = 30
g = json.load(open("glyph.json")); W_R, H_R = g["width"], g["height"]; T_R = 0.020
HOME_C = (0.58, 0.45); HOME = cell_to_world(*HOME_C)                     # ink-box centre of the R on the glass (y = 0)
DEPTH = 0.045                                                            # pulled off the glass for the flip and the fall
TIP_DY, TIP_DZ = 0.004, -0.020                                           # fingertip relative to the letter centre (top-down)
Y_OPEN = 0.045
def close_for(width, squeeze=0.0006):                                    # slide so the pads (2.4 mm proud) just touch
    return Y_OPEN - 0.0024 - width / 2 + squeeze
OPEN, CLOSED = 0.0, close_for(W_R)
FACTORY = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
SLIP = dict(vx=0.05, w=0.25)
knots = []                                                               # (t, q, finger, label)
def at(t, q, f, label=""): knots.append((t, np.array(q, float), float(f), label))
def solve(p, phi, psi, q_ref, **kw):
    q, ep, er = ik(p, target_rot(phi, psi), q_ref, **kw)
    assert ep < 3e-3 and er < 0.03, (p, phi, psi, ep, er)
    return q
t = 0.0
at(t, FACTORY, OPEN, "factory"); t += 0.5; at(t, FACTORY, OPEN)
grasp1 = HOME + np.array([0, TIP_DY, TIP_DZ])
q_hover = solve(grasp1 + np.array([0.007, 0, 0.055]), 0, 0, FACTORY)    # policy pre-grasp, 7 mm off in x
t += 1.05; at(t, q_hover, OPEN, "pre-grasp")
q_near = solve(grasp1 + np.array([0.002, 0, 0.018]), 0, 0, q_hover); t += 0.30; at(t, q_near, OPEN)
q_g1 = solve(grasp1, 0, 0, q_near); t += 0.30; at(t, q_g1, OPEN, "contact")
t += 0.22; at(t, q_g1, CLOSED, "closed1"); T_HELD = t
pull = grasp1 + np.array([0, DEPTH, 0.004])
q_pull = solve(pull, 0, 0, q_g1); t += 0.32; at(t, q_pull, CLOSED, "pulled")
q = q_pull; T_FLIP0 = t
for k in range(1, 7):                                                    # flip: psi 0 -> -pi in 30 deg steps
    psi = -np.pi * k / 6
    q = solve(pull, 0, psi, q, restarts=8); at(t + 0.92 * k / 6, q, CLOSED, f"flip{k}")
t += 0.92; T_DROP = t; q_flipped = q
# ---- the drop (letter centre starts at the flipped grasp's letter centre) ----
c0 = pull - np.array([0, TIP_DY, TIP_DZ])                                 # letter centre at the drop
track, settle, mass = drop(c0[0], c0[2], dur=2.0, rate=240, solref=(0.004, 0.8), friction=0.6, **SLIP)
track[:, 1] += c0[0]; track[:, 2] += c0[2]
rest = track[-1]; assert abs(np.degrees(rest[3])) < 1, rest
print(f"drop: settles at x {rest[1]:.3f} z {rest[2]:.4f} after {settle:.2f} s; mass {mass*1e3:.0f} g")
def letter_at(tt): i = min(len(track) - 1, int(round(tt * 240))); return track[i]
at(T_DROP + 0.05, q_flipped, 0.004, "slip")                               # grip gives way
at(T_DROP + 0.12, q_flipped, OPEN, "react")                               # 120 ms latency, fingers open
for tt, lead in ((0.32, 0.20), (0.52, 0.12)):                             # track the falling letter from above
    L = letter_at(tt); p = np.array([L[1], c0[1] + TIP_DY, max(L[2], rest[2]) + lead])
    q = solve(p, 0, -np.pi, q); at(T_DROP + tt, q, OPEN, "track")
floor_c = np.array([rest[1], c0[1], rest[2]])
grasp2 = floor_c + np.array([0, TIP_DY, TIP_DZ])
q = solve(grasp2 + np.array([0, 0, 0.05]), 0, -np.pi, q); at(T_DROP + settle + 0.05, q, OPEN, "hover2")
q = solve(grasp2, 0, -np.pi, q); at(T_DROP + settle + 0.30, q, OPEN, "contact2"); T_C2 = T_DROP + settle + 0.30
at(T_C2 + 0.20, q, CLOSED, "closed2"); T_HELD2 = T_C2 + 0.20
q = solve(grasp2 + np.array([0, 0, 0.07]), 0, -np.pi, q); at(T_HELD2 + 0.30, q, CLOSED, "lift")
q = solve(pull + np.array([-0.004, 0, 0.012]), 0, -np.pi, q); at(T_HELD2 + 1.00, q, CLOSED, "back")
q = solve(pull, 0, -np.pi, q); at(T_HELD2 + 1.25, q, CLOSED, "aligned")
q = solve(grasp1, 0, -np.pi, q); at(T_HELD2 + 1.50, q, CLOSED, "seated"); T_SEAT = T_HELD2 + 1.50
at(T_SEAT + 0.20, q, OPEN, "released"); T_REL = T_SEAT + 0.20
at(T_REL + 0.15, q, OPEN, "end")
# ---- PCHIP through the knots ----
K = sorted(knots, key=lambda k: k[0]); Tk = np.array([k[0] for k in K]); Qk = np.array([k[1] for k in K]); Fk = np.array([k[2] for k in K])
def pchip(ts, ys, tq):
    ys = np.asarray(ys, float).reshape(len(ts), -1); n = len(ts); h = np.diff(ts); out = np.zeros((len(tq), ys.shape[1]))
    for c in range(ys.shape[1]):
        y = ys[:, c]; dl = np.diff(y) / h; dd = np.zeros(n)
        for k in range(1, n - 1):
            if dl[k-1] * dl[k] > 0:
                w1, w2 = 2*h[k] + h[k-1], h[k] + 2*h[k-1]; dd[k] = (w1 + w2) / (w1/dl[k-1] + w2/dl[k])
        i = np.clip(np.searchsorted(ts, tq, side="right") - 1, 0, n - 2); s = (tq - ts[i]) / h[i]
        out[:, c] = (2*s**3 - 3*s**2 + 1)*y[i] + (s**3 - 2*s**2 + s)*h[i]*dd[i] + (-2*s**3 + 3*s**2)*y[i+1] + (s**3 - s**2)*h[i]*dd[i+1]
    return out
tq = np.arange(0, Tk[-1] + 1e-9, 1.0 / FPS)
Qs = pchip(Tk, Qk, tq); Fs = pchip(Tk, Fk, tq)[:, 0]
assert (Qs >= lo - 1e-6).all() and (Qs <= hi + 1e-6).all()
# ---- per-frame letter pose: centre (world x, z), depth y, in-plane angle th (about +y, = CSS rotate), yaw (= CSS rotateY) ----
frames = []; body = sc.tip_body; off1 = off2 = None
for i, tt in enumerate(tq):
    sc.set_q(Qs[i], Fs[i]); tip = sc.tip(); R = d.xmat[body].reshape(3, 3)
    psi = np.arctan2(R[1, 1], R[0, 1])                                     # closing axis direction in the horizontal plane
    if tt < T_HELD - 1e-9:
        L = dict(x=HOME[0], z=HOME[2], y=0.0, th=0.0, yaw=180.0, st="home")
    elif tt < T_DROP - 1e-9:
        if off1 is None: off1 = R.T @ (HOME - tip)
        c = tip + R @ off1; L = dict(x=c[0], z=c[2], y=c[1], th=0.0, yaw=180.0 + np.degrees(psi), st="held")
    elif tt < T_HELD2 - 1e-9:
        L = dict(x=None, z=None, y=c0[1], th=None, yaw=0.0, st="fall")        # filled in the browser from the track
    elif tt < T_REL - 1e-9:
        if off2 is None: off2 = R.T @ (floor_c - tip)
        c = tip + R @ off2; L = dict(x=c[0], z=c[2], y=c[1], th=0.0, yaw=0.0, st="held2")
    else:
        L = dict(x=HOME[0], z=HOME[2], y=0.0, th=0.0, yaw=0.0, st="seated")
    frames.append({"q": Qs[i].tolist(), "f": float(Fs[i]), "tip": world_to_cell(tip).round(2).tolist(), "letter": L})
print("frames", len(frames), "duration %.2f s" % tq[-1], "| drop at %.2f s, regrasp %.2f s, release %.2f s" % (T_DROP, T_HELD2, T_REL))
print("seat error vs home: %.2f mm" % (1e3 * np.linalg.norm(np.array([frames[-1]["letter"]["x"], frames[-1]["letter"]["z"]]) - HOME[[0, 2]])))
json.dump({"fps": FPS, "t_drop": T_DROP, "t_held2": T_HELD2, "t_release": T_REL, "home": list(HOME_C),
           "glyph": {"width": W_R, "height": H_R, "thickness": T_R}, "depth": DEPTH,
           "fall": [[round(a, 5) for a in r] for r in track.tolist()], "frames": frames}, open("traj.json", "w"))
