# step_reach_sim

What one inchworm step of the Tether arm can reach, and what it costs in motor torque.
Model: `src/terra_arm_description/urdf/arm.urdf` (v0.2.0). Results and videos are written up in
tether_context `notes/2026-10-28-john-step-reach-sim.md`.

Everything assumes the robot only steps between mounted female docks (`dock_female.urdf`), with both
grippers on the dock axis. Torques are quasi-static gravity torques, no dynamics.

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

Video rendering needs `ffmpeg`. On a headless Linux box MuJoCo renders with OSMesa
(`apt install libosmesa6`); `render.py` sets `MUJOCO_GL=osmesa` there by default.

## Scripts

| Script | What it does | Output |
|---|---|---|
| `model.py` | URDF loader: forward kinematics, meshes, masses | |
| `step_reach.py` | IK (gripper axes held normal to the surface), mesh self-collision, ledge terrain, gravity torque per motor, load on the anchored gripper | |
| `sweep.py <EE link>` | Grid over step length r and height h for one anchored end (about 3 min on 4 cores) | `results/<link>.npz` |
| `plot.py`, `final.py` | Foothold maps, max-step-vs-height envelope, extreme poses | `out/` |
| `slopes.py`, `plot_slopes.py` | Step length vs surface angle (floor to wall to ceiling) and step direction | `out/slopes.json`, `out/fig4_slopes.png` |
| `corners.py inside\|outside`, `plot_corners.py` | Floor to wall and over-an-edge transitions | `results/corner_*.npz`, `out/fig5_corners.png` |
| `mass_sens.py` | Same limits under alternative mass assumptions | stdout |
| `plan.py` | Dock-to-dock step planner: lift out, collision-checked swing, lower in | |
| `render.py <scenario>` | Plans and renders one test video with a torque overlay (`--preview` for one frame) | `videos/<scenario>.mp4` |

Run from this folder, in this order for the plots: `sweep.py EndEffectorAssembly_v3_2`,
`sweep.py EndEffectorAssembly_v3_1`, `plot.py ...`, `final.py`.

## Assumptions to revisit

- **Masses.** The URDF masses were corrected on 2026-10-31 (`apply_measured_masses.py`): datasheet motors,
  weighed batteries, slip rings and gripper servos, CAD aluminium and TPU for the rest. The arm is 14.35 kg.
  Wiring, ESP boards and fasteners are not modelled. The `datasheet_motors` mass model in `step_reach.py`
  now equals the URDF; results before 2026-10-31 were run at 19.8 kg.
- **Motor limits** (`SPEC` in `step_reach.py`): DM-J10010-2EC 40 Nm rated / 150 Nm peak, DM-J4340P-2EC 9 / 27 Nm,
  from the user manuals. The URDF link names say `DM-J10010L`, but the motor is the plain 10010.
- **Joint ranges** come only from mesh collisions. Cable limits on the non-slip-ring joints are not modelled.
- **Lift-out distance** before a swing is 12 cm (`LIFT` in `plan.py`), because the CAD fin-ray fingers stick
  out 8 cm past the dock frame.
- **Gripper hold strength** is not modelled. The scripts report the load; whether the gripper survives it is
  a bench test.
