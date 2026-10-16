"""Hero motion. Camera looks along +y; the arm faces +x (right). The gripper stays in the x-z plane (side view of the UMI):
approach direction at angle phi in the plane (phi=0 down, +pi/2 pointing right, -pi/2 pointing left), fingers split along the in-plane normal.
Sequence: factory pose -> idle reaches -> grasp the mirrored R from its right side (phi=-pi/2), carry it while swinging to phi=+pi/2
(a 180-degree in-plane flip), place it in the headline, release -> the R falls (simulated in the browser) -> grasp it on the floor
(phi=+pi/2), carry it back swinging to -pi/2 (mirrored again), stick it on the wall at the far spot, release -> idle -> factory pose."""
import mujoco, numpy as np, json
from halftone import Scene, NX, NY, X0, X1, Z0, Z1
sc = Scene(); m, d = sc.m, sc.d
Y_PLANE = 0.0                                   # letters live in the arm's own x-z plane
def cell_to_world(cx, cy):
    return np.array([X0 + cx*(X1-X0), Y_PLANE, Z1 - cy*(Z1-Z0)])
C_PICK, C_PLACE, C_FLOOR = (0.40, 0.80), (0.58, 0.45), (0.60, 0.90)
PICK, PLACE, FLOOR = cell_to_world(*C_PICK), cell_to_world(*C_PLACE), cell_to_world(*C_FLOOR)
IDLE = [cell_to_world(0.58, 0.26), cell_to_world(0.70, 0.40), cell_to_world(0.50, 0.34)]
LETTER_HALF = 0.045                             # half height of the letter (m): the grasp point is its centre, the tip stops at the edge
LETTER_HALF_W = 0.034                           # half width, for the letter lying on its side
def target_rot(phi):
    z = np.array([np.sin(phi), 0.0, -np.cos(phi)])          # approach: down at phi=0, right at +pi/2, left at -pi/2
    x = np.array([np.cos(phi), 0.0, np.sin(phi)])           # finger split axis, in plane
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)
def grasp_pose(center, phi, half=LETTER_HALF):               # tip sits `half` before the centre along the approach
    return center - half * np.array([np.sin(phi), 0.0, -np.cos(phi)])
lo, hi = m.jnt_range[:7,0], m.jnt_range[:7,1]
def ik(p_des, phi, q0, iters=300, restarts=14):
    best = None; rng = np.random.default_rng(5)
    for r in range(restarts):
        q0r = q0 if r == 0 else np.clip(q0 + rng.normal(0, 0.7, 7), lo+0.05, hi-0.05)
        res = _ik(p_des, phi, q0r, iters)
        if best is None or (res[1] + 0.3*res[2]) < (best[1] + 0.3*best[2]): best = res
        if best[1] < 2e-3 and best[2] < 0.02: break
    sc.set_q(best[0]); return best
def _ik(p_des, phi, q0, iters=300):
    q = q0.copy(); R_des = target_rot(phi); body = sc.tip_body
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
OPEN, CLOSED = 0.0, 0.030
FACTORY = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
L, R_ = -np.pi/2, np.pi/2
BACK = 0.10                                     # hover distance before a grasp, along the approach
def hover(center, phi, half=LETTER_HALF): return grasp_pose(center, phi, half) - BACK * np.array([np.sin(phi), 0.0, -np.cos(phi)])
#        kind      target                       phi   finger  dur  state-after
keys = [("hold",   None,                        None, OPEN,   1.6, None),                       # factory position
        ("move",   IDLE[0],                     0.0,  OPEN,   2.4, None), ("dwell", IDLE[0], 0.0, 0.026, 1.0, None),
        ("move",   IDLE[1],                     0.3,  OPEN,   2.2, None), ("dwell", IDLE[1], 0.3, 0.026, 0.9, None),
        ("move",   hover(PICK, L),              L,    OPEN,   2.6, None),
        ("move",   grasp_pose(PICK, L),         L,    OPEN,   0.9, None), ("grasp", grasp_pose(PICK, L), L, CLOSED, 0.6, "held"),
        ("carry",  grasp_pose(PLACE, R_),       R_,   CLOSED, 3.4, None),
        ("release",grasp_pose(PLACE, R_),       R_,   OPEN,   0.5, "falling"),
        ("move",   hover(PLACE, R_),            R_,   OPEN,   0.8, None),
        ("move",   hover(FLOOR, 0.0, LETTER_HALF_W), 0.0, OPEN, 1.8, None),
        ("move",   grasp_pose(FLOOR, 0.0, LETTER_HALF_W), 0.0, OPEN, 0.8, None), ("grasp", grasp_pose(FLOOR, 0.0, LETTER_HALF_W), 0.0, CLOSED, 0.6, "held2"),
        ("carry",  grasp_pose(PICK, L),         L,    CLOSED, 3.4, None),
        ("release",grasp_pose(PICK, L),         L,    OPEN,   0.5, "wall"),
        ("move",   hover(PICK, L),              L,    OPEN,   0.8, None),
        ("move",   IDLE[2],                     0.0,  OPEN,   2.2, None),
        ("factory",None,                        None, OPEN,   2.4, None)]
frames = []; qc = FACTORY.copy(); fc = OPEN; phc = 0.0; state = "wall"; errs = []
sc.set_q(qc)
for kind, p, phi, fg, dur, after in keys:
    if kind in ("hold", "factory"): qn, ep, er, phn = FACTORY.copy(), 0.0, 0.0, phc if kind == "hold" else 0.0
    else: qn, ep, er = ik(p, phi, qc); phn = phi; errs.append((kind, round(ep,4), round(er,4)))
    n = max(1, int(round(dur*FPS)))
    for k in range(n):
        s = smooth((k+1)/n)
        qi = qc + (qn-qc)*s; fi = fc + (fg-fc)*s; phi_i = phc + (phn-phc)*s
        sc.set_q(qi, fi); tip = sc.to_cell(sc.tip())
        frames.append({"q": qi.tolist(), "f": fi, "tip": [round(tip[0],2), round(tip[1],2)], "phi": round(float(phi_i),4), "state": state, "kind": kind})
        if after and k == n-1: state = after
    qc, fc, phc = qn, fg, phn
print("frames", len(frames), "sec", round(len(frames)/FPS,1))
bad = [e for e in errs if e[1] > 0.005 or e[2] > 0.03]; print("ik errors over tolerance:", bad if bad else "none")
json.dump({"fps": FPS, "nx": NX, "ny": NY, "pick": list(C_PICK), "place": list(C_PLACE), "floor": list(C_FLOOR),
           "letter_half_cells": LETTER_HALF/(Z1-Z0)*NY, "letter_half_w_cells": LETTER_HALF_W/(Z1-Z0)*NY, "floor_angle": 90, "frames": frames}, open("traj.json","w"))
