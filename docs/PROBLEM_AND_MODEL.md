# Problem And Model

This repository models a planar RRR welding robot operating in a fixed 2D environment. The geometry in `scene.py` and the weld registry in `welds.py` are the locked problem definition for this collaborator package.

## Robot base convention

- Base frame origin: `(0.0, 0.0)` mm
- Positive `x`: right
- Positive `y`: up
- Joint units: radians in code and artifacts
- Zero-pose forward kinematics: `fk((0, 0, 0)) = (340, 0, 0)`

## Link lengths

The active robot dimensions from `scene.py` are:
- `L1 = 170 mm`
- `L2 = 160 mm`
- `L3 = 10 mm`

## Tool and clearance assumptions

- Torch diameter: `10 mm`
- Scene clearance assumption: `1 mm`
- Active inflated obstacle radius used by Task 2: `6 mm`
- Task 2 polygonizes circles with `48` sides before collision checks
- The active collision model checks only Link 3 and the torch-tip segment

These assumptions are part of the frozen Task1/Task2 package and should be preserved when reading or extending the branch.

## Depot convention

- The depot is fixed at `(300.0, 0.0)`.
- Task 1 solves a depot-anchored cycle that starts at `DEPOT` and returns to `DEPOT`.
- Task 2 consumes that depot-anchored order and builds one path segment per consecutive pair in the cycle.

## Object and weld naming

The fixed scene contains:
- `C1`: circle centered at `(60, 140)` with radius `25`
- `P1`: regular hexagon centered at `(150, 140)` with inradius `20`
- `C2`: circle centered at `(240, 140)` with radius `35`
- `C3`: circle centered at `(150, 40)` with radius `30`
- `P2`: regular hexagon centered at `(250, 40)` with inradius `20`

Naming conventions:
- Circle objects use `C*`
- Hexagon objects use `P*`
- Weld ids use `OBJECT_ID_W<index>`, for example `C1_W0` or `P2_W5`
- Task 2 lifted states extend those visit ids with branch labels such as `P2_W4_B0`

## Weld convention

- Circles expose weld normals at `0`, `90`, `180`, and `270` degrees.
- Hexagons expose weld normals at `30`, `90`, `150`, `210`, `270`, and `330` degrees.
- Weld points are placed on the object boundary offset by the scene inflation radius.
- Weld normals point outward from the object.
- The tool heading is the inward-facing Link-3 direction:
  `tool_heading_deg = (alpha_deg + 180) % 360`

## Locked assumptions for collaborators

- `scene.py` is the authoritative environment definition.
- `welds.py` deterministically defines the weld registry.
- Task 1 ignores collision and solves Euclidean weld ordering only.
- Task 2 is the first collision-aware stage and uses the active `collision.py` model.
- `artifacts/task1_hamiltonian_solution.json` is the authoritative Task 1 output for Task 2.
- `artifacts/task2_cartesian_paths.json` is the authoritative upstream artifact for Task 3.
