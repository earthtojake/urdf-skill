from __future__ import annotations

import argparse
import importlib.util
import inspect
import json
import sys
from collections.abc import Sequence
from pathlib import PurePosixPath
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from urdf_source import UrdfSourceError, read_urdf_source


def generate_urdf_targets(targets: Sequence[str], *, summary: bool = False) -> int:
    generated_paths = [_generate_target(target) for target in targets]
    if summary:
        _print_summaries(generated_paths)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="gen_urdf",
        description="Generate explicit URDF targets from envelope-returning Python sources.",
    )
    parser.add_argument(
        "targets",
        nargs="+",
        help="Explicit Python source file defining gen_urdf() to generate.",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Print a compact summary for generated outputs.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    return generate_urdf_targets(args.targets, summary=args.summary)


def _generate_target(target: str) -> Path:
    script_path = Path(target).resolve()
    if script_path.suffix.lower() != ".py":
        raise ValueError(f"{_display_path(script_path)} must be a Python source file")
    if not script_path.is_file():
        raise FileNotFoundError(f"Python source not found: {_display_path(script_path)}")

    module = _load_generator_module(script_path)
    generator = getattr(module, "gen_urdf", None)
    if not callable(generator):
        raise RuntimeError(f"{_display_path(script_path)} does not define callable gen_urdf()")
    if inspect.signature(generator).parameters:
        raise ValueError(f"{_display_path(script_path)} gen_urdf() must not accept arguments")

    envelope = generator()
    if not isinstance(envelope, dict):
        raise TypeError(f"{_display_path(script_path)} gen_urdf() must return a generator envelope dict")

    output_path = _resolve_urdf_output(envelope.get("urdf_output"), script_path=script_path)
    _write_urdf_payload(envelope, output_path=output_path, script_path=script_path)
    if not output_path.exists():
        raise RuntimeError(f"{_display_path(script_path)} did not write {_display_path(output_path)}")
    return output_path


def _load_generator_module(script_path: Path) -> object:
    module_name = (
        "_urdf_tool_"
        + _display_path(script_path).replace("/", "_").replace("\\", "_").replace("-", "_").replace(".", "_")
    )
    module_spec = importlib.util.spec_from_file_location(module_name, script_path)
    if module_spec is None or module_spec.loader is None:
        raise RuntimeError(f"Failed to load generator module from {_display_path(script_path)}")

    module = importlib.util.module_from_spec(module_spec)
    original_sys_path = list(sys.path)
    search_paths = [
        str(Path.cwd().resolve()),
        str(script_path.parent),
    ]
    for candidate in reversed(search_paths):
        if candidate not in sys.path:
            sys.path.insert(0, candidate)

    try:
        sys.modules[module_name] = module
        module_spec.loader.exec_module(module)
    finally:
        sys.path[:] = original_sys_path

    return module


def _resolve_urdf_output(raw_value: object, *, script_path: Path) -> Path:
    if not isinstance(raw_value, str) or not raw_value.strip():
        raise ValueError(f"{_display_path(script_path)} gen_urdf() envelope urdf_output must be a non-empty string")
    value = raw_value.strip()
    if "\\" in value:
        raise ValueError(f"{_display_path(script_path)} gen_urdf() envelope urdf_output must use POSIX '/' separators")
    pure = PurePosixPath(value)
    if pure.is_absolute() or any(part in {"", "."} for part in pure.parts):
        raise ValueError(f"{_display_path(script_path)} gen_urdf() envelope urdf_output must be relative")
    output_path = (script_path.parent / Path(*pure.parts)).resolve()
    if output_path.suffix.lower() != ".urdf":
        raise ValueError(f"{_display_path(script_path)} gen_urdf() envelope urdf_output must end in .urdf")
    return output_path


def _write_urdf_payload(envelope: dict[str, object], *, output_path: Path, script_path: Path) -> None:
    xml = envelope.get("xml")
    if not isinstance(xml, str):
        raise TypeError(
            f"{_display_path(script_path)} gen_urdf() envelope field 'xml' must be a string, "
            f"got {type(xml).__name__}"
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    text = xml if xml.endswith("\n") else xml + "\n"
    output_path.write_text(text, encoding="utf-8")
    print(f"Wrote URDF: {output_path}")
    _write_explorer_metadata_payload(envelope, output_path=output_path, script_path=script_path)


def _write_explorer_metadata_payload(envelope: dict[str, object], *, output_path: Path, script_path: Path) -> None:
    if "explorer_metadata" not in envelope or envelope.get("explorer_metadata") is None:
        return
    explorer_metadata = envelope.get("explorer_metadata")
    if not isinstance(explorer_metadata, dict):
        raise TypeError(
            f"{_display_path(script_path)} gen_urdf() envelope field 'explorer_metadata' must be an object, "
            f"got {type(explorer_metadata).__name__}"
        )
    explorer_dir = output_path.parent / f".{output_path.name}"
    explorer_dir.mkdir(parents=True, exist_ok=True)
    explorer_metadata_path = explorer_dir / "explorer.json"
    text = json.dumps(explorer_metadata, indent=2, sort_keys=True)
    explorer_metadata_path.write_text(text + "\n", encoding="utf-8")
    print(f"Wrote URDF explorer metadata: {explorer_metadata_path}")


def _print_summaries(paths: Sequence[Path]) -> None:
    for path in paths:
        try:
            urdf_source = read_urdf_source(path)
        except (UrdfSourceError, ValueError) as exc:
            print(f"summary {_display_path(path)}: unavailable ({exc})")
            continue
        print(
            f"{_display_path(path)}: robot={urdf_source.robot_name} "
            f"links={len(urdf_source.links)} joints={len(urdf_source.joints)}"
        )


def _display_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


if __name__ == "__main__":
    raise SystemExit(main())
