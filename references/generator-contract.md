# URDF Generator Contract

Use this reference when creating or editing Python sources that generate URDF files.

## Source Of Truth

The Python source that defines `gen_urdf()` is source of truth. The configured `.urdf` file is generated and should not be hand-edited.

## Envelope Contract

`gen_urdf()` must be a top-level zero-argument function returning an envelope with:

- `xml`: complete URDF XML as a string
- `urdf_output`: relative path to the generated `.urdf` file
- `explorer_metadata` (optional): JSON-serializable object written to `.<urdf filename>/explorer.json` for consumer-specific metadata that should not be embedded in standard URDF XML

The `urdf_output` path:

- is relative to the owning Python source
- must use POSIX `/` separators
- must end in `.urdf`
- is resolved as a file path, not through a harness root

The host project may impose its own layout policy, but the URDF skill runtime does not hardcode a project directory.

## Runtime Behavior

`scripts/gen_urdf/cli.py` runs only `gen_urdf()`. It does not regenerate external mesh, CAD, or render artifacts.

If URDF visual or collision mesh references depend on updated mesh outputs, regenerate those targets separately with the owning project's CAD or mesh workflow.
