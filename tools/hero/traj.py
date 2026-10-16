"""Plan the hero motion in joint space. Gripper approach points toward the camera (tilted down); roll about it flips the letter.
Sequence: idle reaches -> pick the mirrored R, flip, place it in the headline -> it falls -> pick it up from the floor,
carry it back to the far spot (flipping it back), press it against the wall, release -> idle again."""
import mujoco, numpy as np, json
from halftone import Scene, NX, NY, X0, X1, Z0, Z1
sc = Scene(); m, d = sc.m, sc.d
Y_WALL = -0.40
def cell_to_world(cx, cy, dy=0.0):              # canvas fractions -> wall point; canvas mirrored (base on the left)
    cx = 1.0 - cx
    return np.array([X0 + cx*(X1-X0), Y_WALL + dy, Z1 - cy*(Z1-Z0)])
C_PICK, C_PLACE, C_FLOOR = (0.42, 0.88), (0.46, 0.64), (0.40, 0.93)
PICK, PLACE, FLOOR = cell_to_world(*C_PICK), cell_to_world(*C_PLACE), cell_to_world(*C_FLOOR)
IDLE = [cell_to_world(0.54, 0.30), cell_to_world(0.66, 0.50), cell_to_world(0.46, 0.40)]
BACK = np.array([0, 0.08, 0])                   # hover offset away from the wall
TILT = -0.31                                    # how the letter lands on the floor (rad, in-plane)
def target_rot(theta):
    z = np.array([0,-1.0,-0.55]); z /= np.linalg.norm(z)
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
OPEN, CLOSED = 0.0, 0.034
q_home = np.array([np.pi, -0.5, 0, -2.0, 0, 1.6, 0.8])
q, ep, er = ik(IDLE[0], 0.0, q_home)
#        kind       target             roll        finger  dur   letter-state-after
keys = [("move",   IDLE[0],           0.0,        OPEN,   2.4, None), ("dwell", IDLE[0], 0.0, 0.028, 1.2, None),
        ("move",   IDLE[1],           0.0,        OPEN,   2.4, None), ("dwell", IDLE[1], 0.0, 0.028, 1.2, None),
        ("move",   IDLE[2],           0.0,        OPEN,   2.2, None), ("dwell", IDLE[2], 0.0, 0.028, 1.0, None),
        ("move",   PICK + BACK,       np.pi/2,    OPEN,   2.6, None),
        ("move",   PICK,              np.pi/2,    OPEN,   0.9, None), ("grasp", PICK, np.pi/2, CLOSED, 0.7, "held"),
        ("carry",  PLACE,            -np.pi/2,    CLOSED, 3.2, None),
        ("release",PLACE,            -np.pi/2,    OPEN,   0.5, "falling"),
        ("move",   PLACE + BACK*0.6, -np.pi/2,    OPEN,   0.5, None),
        ("move",   FLOOR + BACK,     -np.pi/2+TILT, OPEN, 1.6, None),
        ("move",   FLOOR,            -np.pi/2+TILT, OPEN, 0.7, None), ("grasp", FLOOR, -np.pi/2+TILT, CLOSED, 0.6, "held2"),
        ("carry",  PICK + BACK*0.5,   np.pi/2,    CLOSED, 3.0, None),
        ("press",  PICK + np.array([0,-0.035,0]), np.pi/2, CLOSED, 0.6, "pressed"),
        ("hold",   PICK + np.array([0,-0.035,0]), np.pi/2, CLOSED, 0.9, None),
        ("move",   PICK,              np.pi/2,    CLOSED, 0.4, None),
        ("release",PICK,              np.pi/2,    OPEN,   0.5, "wall"),
        ("move",   PICK + BACK,       np.pi/2,    OPEN,   0.8, None),
        ("move",   IDLE[1],           0.0,        OPEN,   2.6, None)]
frames = []; qc = q; fc = OPEN; thc = 0.0; state = "wall"; errs = []
for kind, p, th, fg, dur, after in keys:
    qn, ep, er = ik(p, th, qc); errs.append((kind, round(ep,4), round(er,4)))
    n = max(1, int(round(dur*FPS)))
    for k in range(n):
        s = smooth((k+1)/n)
        qi = qc + (qn-qc)*s; fi = fc + (fg-fc)*s; thi = thc + (th-thc)*s
        sc.set_q(qi, fi); tip = sc.to_cell(sc.tip())
        press = s if kind == "press" else (1.0 if kind == "hold" else 0.0)
        frames.append({"q": qi.tolist(), "f": fi, "tip": [round(tip[0],2), round(tip[1],2)], "theta": round(float(thi),4), "state": state, "kind": kind, "press": round(press,3)})
        if after and k == n-1: state = after
    qc, fc, thc = qn, fg, th
print("frames", len(frames), "sec", len(frames)/FPS)
bad = [e for e in errs if e[1] > 0.005 or e[2] > 0.03]; print("ik errors over tolerance:", bad if bad else "none")
json.dump({"fps": FPS, "nx": NX, "ny": NY, "pick": list(C_PICK), "place": list(C_PLACE), "floor": list(C_FLOOR), "tilt": TILT, "frames": frames}, open("traj.json","w"))
