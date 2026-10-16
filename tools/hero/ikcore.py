"""IK for the gripper tip with a full orientation target.
Orientation: start from the top-down grasp (approach down, fingers closing along world x, GoPro toward the viewer),
tilt in the picture plane by phi (0 down, +pi/2 approach pointing right), then yaw about the world vertical by psi."""
import mujoco, numpy as np
from halftone import Scene, NX, NY, X0, X1, Z0, Z1
sc = Scene(); m, d = sc.m, sc.d
lo, hi = m.jnt_range[:7, 0].copy(), m.jnt_range[:7, 1].copy()
def Rz(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def target_rot(phi=0.0, psi=0.0):
    z = np.array([np.sin(phi), 0.0, -np.cos(phi)])          # approach
    y = np.array([np.cos(phi), 0.0, np.sin(phi)])           # closing axis
    x = np.cross(y, z)                                      # = +y world: away from the viewer
    return Rz(psi) @ np.stack([x, y, z], 1)
def cell_to_world(cx, cy, y=0.0): return np.array([X0 + cx*(X1-X0), y, Z1 - cy*(Z1-Z0)])
def world_to_cell(p): return np.array([(p[0]-X0)/(X1-X0)*NX, (Z1-p[2])/(Z1-Z0)*NY])
def _ik(p_des, R_des, q0, iters=400):
    q = q0.copy(); body = sc.tip_body; jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
    for _ in range(iters):
        sc.set_q(q); p = sc.tip(); R = d.xmat[body].reshape(3, 3)
        e_p = p_des - p; e_r = 0.5*sum(np.cross(R[:, i], R_des[:, i]) for i in range(3))
        if np.linalg.norm(e_p) < 3e-4 and np.linalg.norm(e_r) < 1.5e-3: break
        mujoco.mj_jacSite(m, d, jacp, jacr, sc.tip_site)
        J = np.vstack([jacp[:, :7], 0.6*jacr[:, :7]]); e = np.concatenate([e_p, 0.6*e_r])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3*np.eye(6), e)
        q = np.clip(q + np.clip(dq, -0.2, 0.2), lo + 0.02, hi - 0.02)
    sc.set_q(q); R = d.xmat[body].reshape(3, 3)
    return q, float(np.linalg.norm(p_des - sc.tip())), float(np.linalg.norm(0.5*sum(np.cross(R[:, i], R_des[:, i]) for i in range(3))))
def ik(p_des, R_des, q0, restarts=14, seed=5, stay_close=True):
    """Warm-started DLS IK with random restarts; prefers the solution closest to q0 among the good ones."""
    rng = np.random.default_rng(seed); best = None; good = []
    for r in range(restarts):
        q0r = q0 if r == 0 else np.clip(q0 + rng.normal(0, 0.7, 7), lo + 0.05, hi - 0.05)
        res = _ik(p_des, R_des, q0r)
        if res[1] < 2e-3 and res[2] < 0.02: good.append(res)
        if best is None or res[1] + 0.3*res[2] < best[1] + 0.3*best[2]: best = res
        if r == 0 and good and stay_close: break
    if good: best = min(good, key=lambda g: np.linalg.norm(g[0] - q0))
    sc.set_q(best[0]); return best
