# URDF Workflow

Use this reference when editing robot description structure, mesh references, or generated URDF output.

## Edit Loop

1. Find the Python source that defines `gen_urdf()`.
2. Treat that Python source as source of truth and the `.urdf` file as generated.
3. For physical robot links, prioritize the standard trio: `inertial`, `visual`, and `collision`.
4. Edit links, joints, limits, axes, origins, inertials, materials, visual/collision geometry, and mesh filenames deliberately.
5. Keep visual and collision mesh references tied to the source assembly or instance payload when the project uses generated assembly meshes.
6. Regenerate only the explicit URDF target with `scripts/gen_urdf/cli.py <source-file>`.
7. Use `--summary` for a compact robot/link/joint check.
8. If mesh outputs changed, regenerate affected mesh/render assets with the owning project's mesh workflow.

## Standard Link Tags

Use these tags for each link that represents physical robot geometry:

- `inertial`: mass, center of mass, and inertia tensor used by simulators.
- `visual`: display geometry and optional material.
- `collision`: contact geometry used by physics and planning.

Frame-only links, such as `base_footprint` or tool-center marker frames, may intentionally omit these tags when they represent no physical mass or geometry.

For movable physical links, avoid zero or missing mass unless the target simulator explicitly supports that modeling choice. If exact mass properties are unavailable, use a documented approximation and make the approximation easy to replace later.

## Explorer Pose Metadata

When a downstream UI or simulator adapter needs named robot poses, default joint values, or other consumer-specific description data, encode it as generator-owned sidecar metadata rather than non-standard XML inside the generated URDF.

- Return optional `explorer_metadata` from `gen_urdf()`; the generator writes it to `.<urdf filename>/explorer.json`.
- Choose a host-defined metadata schema and document the expected keys.
- If storing default values or pose presets, document the units expected by that consumer.
- Keep the source-of-truth in the Python file that defines `gen_urdf()`, then regenerate the explicit `.urdf` target.

Example:

```json
{
  "schemaVersion": 1,
  "kind": "example-urdf-consumer",
  "defaultJoints": {
    "elbow": 45
  },
  "poses": [
    {
      "name": "home",
      "joints": {
        "shoulder": 0,
        "elbow": 45
      }
    }
  ]
}
```

## Mesh References

URDF mesh filenames should be stable from the generated URDF file's perspective or use a package URI convention understood by the consumer.

When using package URIs, confirm the consuming environment resolves the package root the same way as the generated URDF expects.

Do not use generated URDF XML as the source of truth for mesh placement. Prefer deriving visual mesh references from the same source data that owns the mesh instance placement.

## Collision Geometry

Add collision geometry under each `<link>` that should participate in physics or contact. Do not encode collision behavior on joints.

Use one or more `<collision>` blocks per link. The `<origin>` is expressed in the link frame, just like `<visual>`, and mesh scales must match the units of the exported mesh:

```xml
<link name="forearm_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <mesh filename="STL/forearm.stl" scale="0.001 0.001 0.001" />
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <mesh filename="STL/forearm_collision.stl" scale="0.001 0.001 0.001" />
    </geometry>
  </collision>
</link>
```

Prefer simplified collision geometry over detailed visual meshes. Good options, from simplest to most specific:

- Primitive `<box>`, `<cylinder>`, or `<sphere>` geometry when it approximates the part well.
- A coarse, closed STL collision mesh exported from CAD.
- The visual STL as a temporary fallback for loading and smoke tests.

In generator sources, model collisions explicitly rather than hand-editing generated URDF. A common pattern is to add a `collisions` collection beside `visuals` in each link spec and emit it with the same mesh filename, origin, and scale helper code used for visual meshes.
