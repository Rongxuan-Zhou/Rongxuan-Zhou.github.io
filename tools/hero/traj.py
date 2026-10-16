"""Plan the hero motion in joint space: idle reaches, then pick the mirrored letter, flip it and place it.
Gripper approach axis points +y (into the wall the letters sit on); roll about it flips the letter."""
import mujoco, numpy as np, json
from halftone import Scene, NX, NY, X0, X1, Z0, Z1
sc = Scene(); m, d = sc.m, sc.d
Y_WALL = -0.40                                  # wall plane in front of the base (+y)
def cell_to_world(cx, cy):                     # canvas fractions -> wall point; the canvas is mirrored (base on the left)
    cx = 1.0 - cx
    return np.array([X0 + cx*(X1-X0), Y_WALL, Z1 - cy*(Z1-Z0)])
PICK  = cell_to_world(0.42, 0.88)              # where the mirrored R lies
PLACE = cell_to_world(0.46, 0.64)              # where the headline's R belongs
IDLE  = [cell_to_world(0.54, 0.30), cell_to_world(0.66, 0.50), cell_to_world(0.46, 0.40)]
def target_rot(theta):
    z = np.array([0,-1.0,-0.55]); z /= np.linalg.norm(z)                 # toward the camera, tilted down
    x0 = np.array([np.cos(theta), 0, np.sin(theta)]); x0 -= z*np.dot(x0,z); x = x0/np.linalg.norm(x0); y = np.cross(z, x)
    return np.stack([x, y, z], 1)
lo, hi = m.jnt_range[:7,0], m.jnt_range[:7,1]
def ik(p_des, theta, q0, iters=300, restarts=12):
    best = None; rng = np.random.default_rng(3)
    for r in range(restarts):
        q0r = q0 if r == 0 else np.clip(q0 + rng.normal(0, 0.6, 7), lo+0.05, hi-0.05)
        res = _ik(p_des, theta, q0r, iters)
        if best is None or (res[1] + 0.3*res[2]) < (best[1] + 0.3*best[2]): best = res
        if best[1] < 2e-3 and best[2] < 0.02: break
    sc.set_q(best[0]); return best
def _ik(p_des, theta, q0, iters=300):
    q = q0.copy(); R_des = target_rot(theta); body = sc.tip_body
    jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
    for _ in range(iters):
        sc.set_q(q); p = sc.tip(); R = d.xmat[body].reshape(3,3)
        e_p = p_des - p; e_r = 0.5*sum(np.cross(R[:,i], R_des[:,i]) for i in range(3))
        e = np.concatenate([e_p, 0.6*e_r])
        if np.linalg.norm(e_p) < 5e-4 and np.linalg.norm(e_r) < 2e-3: break
        mujoco.mj_jac(m, d, jacp, jacr, p, body)
        J = np.vstack([jacp[:, :7], 0.6*jacr[:, :7]])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3*np.eye(6), e)
        q = np.clip(q + np.clip(dq, -0.2, 0.2), lo+0.02, hi-0.02)
    sc.set_q(q); return q, np.linalg.norm(p_des - sc.tip()), np.linalg.norm(e_r)
smooth = lambda t: t*t*t*(10 + t*(-15 + 6*t))
FPS = 15
segs = []   # (duration, q_from, q_to, finger_from, finger_to, letter_state, theta_from, theta_to)
q_home = np.array([np.pi, -0.5, 0, -2.0, 0, 1.6, 0.8])
q, ep, er = ik(IDLE[0], 0.0, q_home); print("idle0 err", round(ep,4), round(er,4))
keys = [("move", IDLE[0], 0.0, 0.0, 2.4), ("dwell", IDLE[0], 0.0, 0.028, 1.2),
        ("move", IDLE[1], 0.0, 0.0, 2.4), ("dwell", IDLE[1], 0.0, 0.028, 1.2),
        ("move", IDLE[2], 0.0, 0.0, 2.2), ("dwell", IDLE[2], 0.0, 0.028, 1.0),
        ("move", PICK + np.array([0,0.08,0]), np.pi/2, 0.0, 2.6),     # hover in front of the letter, rolled 180 (letter is upside down)
        ("move", PICK, np.pi/2, 0.0, 1.0), ("grasp", PICK, np.pi/2, 0.034, 0.8),
        ("carry", PLACE, -np.pi/2, 0.034, 3.2),                              # lift, roll back to 0 and translate: the flip
        ("release", PLACE, -np.pi/2, 0.0, 0.8), ("move", PLACE + np.array([0,0.1,0]), -np.pi/2, 0.0, 1.2),
        ("move", IDLE[1], 0.0, 0.0, 2.6)]
frames = []; qc = q; fc = 0.0; thc = 0.0; held = False; placed = False; errs = []
for kind, p, th, fg, dur in keys:
    qn, ep, er = ik(p, th, qc); errs.append((kind, round(ep,4), round(er,4)))
    n = max(1, int(round(dur*FPS)))
    for k in range(n):
        s = smooth((k+1)/n)
        qi = qc + (qn-qc)*s; fi = fc + (fg-fc)*s; thi = thc + (th-thc)*s
        sc.set_q(qi, fi); tip = sc.to_cell(sc.tip())
        if kind == "grasp" and k == n-1: held = True
        state = "placed" if placed else ("held" if held else "wall")
        frames.append({"q": qi.tolist(), "f": fi, "tip": [round(tip[0],2), round(tip[1],2)], "theta": round(float(thi),4), "state": state, "kind": kind})
        if kind == "release" and k == n-1: held = False; placed = True
    qc, fc, thc = qn, fg, th
print("frames", len(frames), "sec", len(frames)/FPS); print("ik errors", errs)
json.dump({"fps": FPS, "nx": NX, "ny": NY, "pick": [0.42,0.88], "place": [0.46,0.64], "frames": frames}, open("traj.json","w"))
