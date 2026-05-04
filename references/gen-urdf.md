# gen_urdf

Regenerates explicit URDF outputs from Python sources with envelope-returning `gen_urdf()` functions.

```bash
python <urdf-skill>/scripts/gen_urdf/cli.py path/to/assembly.py
python <urdf-skill>/scripts/gen_urdf/cli.py path/to/assembly.py --summary
```

Targets must be explicit generated Python source files whose `gen_urdf()` returns an envelope with `xml` and `urdf_output`; see `references/generator-contract.md`.

Relative targets resolve from the current working directory.

This tool runs only `gen_urdf()` and does not regenerate CAD, mesh/export, GLB/topology, or render outputs.
