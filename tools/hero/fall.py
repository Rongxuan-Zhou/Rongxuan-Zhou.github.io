"""Rigid-body drop of the "R" block in MuJoCo, constrained to a plane parallel to the glass (slide x, slide z, hinge y).
Collision: the glyph's three convex parts (stem, bowl, leg) extruded to 20 mm; inertia from the actual glyph area; acrylic.
Floor: z = 0, hard contact with a small bounce."""
import mujoco, numpy as np, json
g = json.load(open("glyph.json"))
T = 0.020; RHO = 1190.0
A = g["area"]; mass = RHO * A * T
cxg, czg = g["centroid"]
Ixx = RHO * T * g["Ixx_area"] + mass * T * T / 12; Izz = RHO * T * g["Izz_area"] + mass * T * T / 12
Iyy = RHO * T * (g["Ixx_area"] + g["Izz_area"])
def mesh_xml(name, poly):
    v = []
    for x, z in poly:
        v += [x, -T/2, z, x, T/2, z]
    return f'<mesh name="{name}" vertex="{" ".join(f"{a:.6f}" for a in v)}"/>'
def model(x0, z0, solref=(0.003, 0.30), friction=0.55):
    meshes = "\n".join(mesh_xml(k, v) for k, v in g["parts"].items())
    geoms = "\n".join(f'<geom type="mesh" mesh="{k}" rgba=".1 .1 .1 1" friction="{friction} 0.005 0.0001" solref="{solref[0]} {solref[1]}" solimp="0.95 0.99 0.001"/>' for k in g["parts"])
    return f"""<mujoco><option timestep="0.0002" integrator="implicitfast" cone="elliptic" gravity="0 0 -9.81"/>
<asset>{meshes}</asset>
<worldbody><geom type="plane" size="3 3 .1" friction="{friction} 0.005 0.0001" solref="{solref[0]} {solref[1]}" solimp="0.95 0.99 0.001"/>
<body name="R" pos="{x0} 0.045 {z0}">
  <joint name="x" type="slide" axis="1 0 0"/><joint name="z" type="slide" axis="0 0 1"/><joint name="th" type="hinge" axis="0 1 0"/>
  <inertial pos="{cxg} 0 {czg}" mass="{mass}" diaginertia="{Ixx} {Iyy} {Izz}"/>
  {geoms}
</body></worldbody></mujoco>"""
def drop(x0, z0, vx=0.0, w=0.0, dur=2.2, rate=240, **kw):
    m = mujoco.MjModel.from_xml_string(model(x0, z0, **kw)); d = mujoco.MjData(m)
    # initial velocity of the body frame origin (the ink-box centre) given the centroid's motion and spin
    d.qvel[:] = [vx, 0.0, w]
    track = []; n = int(round(1.0 / rate / m.opt.timestep)); settle = None
    for k in range(int(dur * rate) + 1):
        track.append([k / rate, float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])])
        sp = abs(d.qvel[0]) + abs(d.qvel[1]) + 0.03 * abs(d.qvel[2])
        if settle is None and k > 0.15 * rate and sp < 0.004 and d.ncon > 0: settle = k / rate
        if settle is not None and sp > 0.02: settle = None
        for _ in range(n): mujoco.mj_step(m, d)
    tr = np.array(track); th = tr[:, 3]
    land = next((r[0] for r in tr if r[2] - z0 < -0.40), None)
    return tr, settle, mass
if __name__ == "__main__":
    import sys
    x0, z0 = float(sys.argv[1]), float(sys.argv[2])
    print(f"mass {drop(x0, z0, dur=0.01)[2]*1e3:.1f} g, Iyy {Iyy*1e6:.2f} g*m^2... ")
    for w in (-2.0, -1.2, -0.6, 0.6, 1.2, 2.0):
        for vx in (0.0, 0.06):
            tr, st, _ = drop(x0, z0, vx=vx, w=w)
            th = np.degrees(tr[:, 3]); end = tr[-1]
            print(f"w {w:+.1f} vx {vx:.2f}: end x {end[1]:.3f} z {end[2]:.4f} th {np.degrees(end[3]):+6.1f}  max|th| {np.abs(th).max():5.1f}  settle {st}")
