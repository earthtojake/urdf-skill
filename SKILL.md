---
name: urdf
description: URDF generation and validation for robot model outputs. Use when the agent needs to create, edit, regenerate, inspect, or validate `.urdf` files, `gen_urdf()` envelopes, robot links, joints, joint limits, parent/child kinematic structure, visual or collision mesh references, collision geometry, or URDF-specific XML validation. Use the owning CAD or mesh workflow for STEP/STP, STL/3MF/DXF exports, render images, GLB/topology sidecars, CAD Explorer links, and @cad geometry references.
---

# URDF

Use this skill for robot description outputs. URDF work is intentionally separate from ordinary CAD generation because the correctness questions are kinematic, XML, and mesh-reference oriented rather than primarily geometric.

Consumer-specific metadata is allowed when a downstream UI or simulator adapter expects it. Treat that metadata as generator-owned extension data rather than standard URDF.

Defer CAD handoff details to the CAD skill: read `.agents/skills/cad/SKILL.md`, then only load its `references/rendering-and-explorer.md` when a URDF needs a CAD Explorer link. If the local CAD skill is unavailable, use [cad-skill](https://github.com/earthtojake/cad-skill) as the fallback reference. Keep URDF-specific generation and `explorer_metadata` contracts in this skill.

## Workflow

1. Treat the Python source that defines `gen_urdf()` as source of truth. Treat the configured `.urdf` file as generated.
2. For the `gen_urdf()` envelope contract, read `references/generator-contract.md`.
3. For robot description edits, read `references/urdf-workflow.md`.
4. Prioritize complete physical links: include `inertial`, `visual`, and `collision` for each link that represents physical robot geometry. Frame-only links may intentionally omit them.
5. Edit links, joints, limits, axes, origins, inertials, materials, visual/collision geometry, mesh filenames, and any consumer-specific sidecar metadata deliberately.
6. Regenerate only the explicit URDF target with `scripts/gen_urdf/cli.py`.
7. Use `--summary` for a compact robot/link/joint check after regeneration.
8. For validation expectations, read `references/validation.md`.
9. If URDF mesh references depend on changed CAD, mesh, or render outputs, regenerate only the affected explicit targets with the owning CAD or mesh workflow.
10. For CAD Explorer handoff of generated `.urdf` entries, use the CAD skill's Explorer handoff rules rather than duplicating link syntax here.

## Commands

Run with the Python environment for the project or workspace. If the environment lacks the URDF validation runtime packages, install this skill's script dependencies from `requirements.txt`. Invoke the tool as a filesystem script, for example `python <urdf-skill>/scripts/gen_urdf/cli.py ...`. Relative target paths are resolved from the current working directory; the tool does not prepend a project root.

- URDF sidecars: `scripts/gen_urdf/cli.py`

The command interface is target-explicit. Pass the Python generator that defines `gen_urdf()`; use `--summary` for a compact robot/link/joint check.

## References

- URDF generation: `references/gen-urdf.md`
- Generator contract: `references/generator-contract.md`
- URDF edit workflow: `references/urdf-workflow.md`
- URDF validation: `references/validation.md`
