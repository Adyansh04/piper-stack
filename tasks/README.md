# Tasks

Each task the arm learns is data in a folder of its own, not a package: `tasks/<name>/task.yaml`
describes its parts, where they are and how they look, and `cad/` holds their models. The
simulator (`sim/`) and Isaac Lab (`lab/`) read the same files.

| Task | What the arm does |
|---|---|
| [`assembly`](assembly/task.yaml) | Builds a threaded assembly of printed parts on a base in four stages: the keyed ring onto the post, the collar screwed onto the boss (2.6 turns), the pin into the boss, the cap locked onto the collar with a 30 degree twist. |
| [`usb_c`](usb_c/task.yaml) | Takes a USB-C plug from its holder and plugs it into a socket on a printed board, with the cable attached. |

## Layout of a task

- `task.yaml`: the parts, their materials, the scene and the preview views (below).
- `cad/<part>.step`: the model to edit or print. `cad/<part>.stl`: the mesh the simulator uses.
  Both are in millimetres, with the part's frame at its bottom centre and z up (the USB-C socket
  and plug: at the opening and at the tip, z along the insertion).

## task.yaml

| Key | Holds |
|---|---|
| `materials` | By name, for the parts to refer to: friction, restitution, and Newton's contact stiffness, damping and torsional and rolling friction. |
| `contact` | `gap_m`: how far from a surface contacts are looked for; `sdf_band_m`: how far a mesh's signed distance field reaches. |
| `print` | Density and the share of a part a print fills, for the masses of printed parts. |
| `parts` | Each part: `body` (`static` or `dynamic`), `material`, `mass_kg` (optional, wins over the printed mass) and `shapes`. |
| `parts.<part>.shapes` | Each mesh: `collision` (`sdf`, `convex_hull`, `convex_decomposition` or `none`), `sdf_resolution` (cells along the mesh's longest side), `colour`, `metallic`, `roughness`. |
| `cable` | A cable from a dynamic part (its plug) to an anchor: radius, mass per metre, bending and torsional stiffness, damping, and its path at the start. |
| `scene` | Poses: `fixed` for static parts, `start` and `goal` for dynamic ones. |
| `stages` | The steps of the task in order, each one part put at its goal on its mate (below). |
| `preview` | The arm's joints and the camera views for the preview images. |

A pose is `xyz` in metres and `rpy` in degrees (roll, pitch and yaw about the fixed axes, as in
URDF), in the robot's world frame (its base at the origin on the table top) or in the frame of the
part named by `in`. Use `sdf` collision for anything that mates closely (threads, slots, pockets,
the connector); it is the most accurate and the costliest.

Each stage names its `part` (whose goal is `in` another part, its mate) and gives:

| Key | Holds |
|---|---|
| `grasp` | The TCP's pose in the part's frame (`xyz`, `rpy`; the gripper points down the TCP's z and the fingers close along its y), `opening_m` (each finger's opening before it closes) and `symmetry` (2: the grasp also holds the part turned half a turn). |
| `approach_m` | How far above where its motion starts the part comes down from. |
| `release_m` | How far above its path the part moves and is let go. |
| `motion` | `insert`, `screw` (with `screw: {pitch_m, turns}`, a right-hand thread from `turns` above the seat) or `twist` (with `twist_deg`, negative clockwise seen from above). |
| `contacts` | Pairs of parts or robot links that may touch while the stage runs. |

`piper_skills`' `run_stage` runs a stage on the arm through MoveIt.

The simulator turns a task into USD files once per change of the task (`sim/task_usd.py`, cached
in `data/sim/tasks/`): each part with its collision for PhysX and Newton, its mass properties from
its meshes and its materials, and the cable as a chain of capsules jointed with springs of the
cable's stiffness (PhysX). Under Newton a convex hull does not collide with a static SDF mesh, so
parts that touch the fixture use `sdf`. `TASK=<name>` puts a task's parts in the simulator
([simulation guide](../docs/guides/simulation.md)).

## Preview images

`sim/preview.py` renders a task's parts with the arm and the table, headless, from the task's
views and the overhead camera's (`sim/config/sim.yaml`). In the sim container:

```bash
/isaac-sim/python.sh /piper-stack/sim/preview.py --task /piper-stack/tasks/assembly --state goal --out /piper-stack/data/preview/assembly
```

`--state start` lets the parts settle where they start; `goal` holds them where they end. A task
with a cable renders its start only, since the cable's path describes the start. It logs how far
each part moved while settling: a part that moved far fell through something or was thrown out of
an overlap. `PHYSICS_ENGINE=newton` renders on Newton.

## Printing

PLA with a 0.4 mm nozzle. Fits have 0.5 to 1 mm of clearance on the diameter, chamfered lead-ins
and coarse threads, so the parts print without supports in the orientation of their CAD frame (the
cap upside down).

| Part | Fits |
|---|---|
| `fixture` | Holds every loose part in a pocket at a known pose; the base sits in it and takes the threading torque. |
| `base` | M29x3.5 boss (thread on the lower 9 mm, 25.4 mm pilot above), keyed post with one flat. |
| `ring`, `collar`, `pin`, `cap` | Ring: D bore on the post. Collar: M30x3.5 inside, two bayonet slots outside. Pin: 8 mm in an 8.6 mm hole. Cap: two lugs in the collar's slots. |
| `board`, `socket`, `plug_holder`, `cable_anchor` | The USB-C demo board, its socket (a USB Type-C receptacle's opening, 8.34 by 2.56 mm), the plug's holder and the cable's anchor. |
