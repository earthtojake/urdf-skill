# URDF Validation

Use this reference when validating generated URDF files.

## Structural Checks

Validate that:

- the root element is `<robot>`
- the robot has a non-empty name
- every link has a unique non-empty name
- every joint has a unique non-empty name
- every joint has valid parent and child links
- parent/child links exist
- each child link has at most one parent
- the graph has exactly one root link
- the graph is connected and acyclic
- the tree has exactly `links - 1` joints unless the design intentionally uses a different structure and the validator supports it

## Joint Checks

Supported joint types are:

- `fixed`
- `continuous`
- `revolute`
- `prismatic`

For revolute and prismatic joints, validate lower and upper limits. Confirm axes and origins match the intended kinematic behavior.

## Inertial Checks

For each physical link with mass or geometry, prefer an explicit `inertial` block:

- `origin` is the center of mass in the link frame
- `mass` is positive
- `inertia` defines `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, and `izz`
- diagonal inertia values are positive
- inertia values satisfy basic triangle inequalities

Frame-only links may intentionally omit `inertial`.

## Mesh Checks

Validate that visual and collision mesh references:

- are non-empty
- point to supported mesh formats
- resolve from the generated URDF location or package URI convention
- refer to files that exist

Collision geometry may also use supported URDF primitives such as box, cylinder, or sphere. Prefer simplified collision geometry over detailed visual meshes when the URDF is intended for physics simulation.

If mesh references changed, confirm the corresponding mesh outputs were regenerated separately.

## Tooling

`scripts/gen_urdf/cli.py --summary` prints a compact robot/link/joint summary after regeneration.

The URDF source reader also validates XML structure with `yourdfpy`, which must be installed in the active Python environment.
