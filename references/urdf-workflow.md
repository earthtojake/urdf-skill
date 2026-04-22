# URDF Workflow

Use this reference when editing robot description structure, mesh references, or generated URDF output.

## Edit Loop

1. Find the Python source that defines `gen_urdf()`.
2. Treat that Python source as source of truth and the `.urdf` file as generated.
3. Edit links, joints, limits, axes, origins, inertials, materials, and mesh filenames deliberately.
4. Keep visual mesh references tied to the source assembly or instance payload when the project uses generated assembly meshes.
5. Regenerate only the explicit URDF target with `scripts/gen_urdf <source-file>`.
6. Use `--summary` for a compact robot/link/joint check.
7. If mesh outputs changed, use the CAD skill to regenerate affected STEP/STL/render assets separately.

## Mesh References

URDF mesh filenames should be stable from the generated URDF file's perspective or use a package URI convention understood by the consumer.

When using package URIs, confirm the consuming environment resolves the package root the same way as the generated URDF expects.

Do not use URDF XML as the source of truth for CAD placement. Prefer deriving visual mesh references from the same assembly/source data that owns the CAD instance payload.
