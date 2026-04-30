from __future__ import annotations

from dataclasses import dataclass
from math import degrees
from pathlib import Path
import xml.etree.ElementTree as ET

URDF_SUFFIX = ".urdf"
SUPPORTED_JOINT_TYPES = {"fixed", "continuous", "revolute", "prismatic"}
SUPPORTED_MESH_SUFFIXES = {".stl"}
SUPPORTED_COLLISION_PRIMITIVES = {"box", "cylinder", "sphere"}


class UrdfSourceError(ValueError):
    pass


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    min_value_deg: float | None
    max_value_deg: float | None


@dataclass(frozen=True)
class UrdfSource:
    file_ref: str
    source_path: Path
    robot_name: str
    root_link: str
    links: tuple[str, ...]
    joints: tuple[UrdfJoint, ...]
    mesh_paths: tuple[Path, ...]
    visual_mesh_paths: tuple[Path, ...] = ()
    collision_mesh_paths: tuple[Path, ...] = ()


def file_ref_from_urdf_path(urdf_path: Path) -> str:
    resolved = urdf_path.resolve()
    if resolved.suffix.lower() != URDF_SUFFIX:
        raise UrdfSourceError(f"{resolved} is not a URDF source file")
    return _relative_to_repo(resolved)


def read_urdf_source(urdf_path: Path) -> UrdfSource:
    resolved_path = urdf_path.resolve()
    if resolved_path.suffix.lower() != URDF_SUFFIX:
        raise UrdfSourceError(f"{resolved_path} is not a URDF source file")

    try:
        root = ET.fromstring(resolved_path.read_text(encoding="utf-8"))
    except (OSError, ET.ParseError) as exc:
        raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} could not be parsed as URDF XML") from exc

    if root.tag != "robot":
        raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} root element must be <robot>")
    robot_name = str(root.attrib.get("name") or "").strip()
    if not robot_name:
        raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} robot name is required")

    link_names = []
    for link_element in root.findall("link"):
        name = str(link_element.attrib.get("name") or "").strip()
        if not name:
            raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} link name is required")
        link_names.append(name)
    if not link_names:
        raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} must define at least one link")
    _raise_on_duplicates(link_names, source_path=resolved_path, label="link")
    link_name_set = set(link_names)
    _validate_link_inertials(root, source_path=resolved_path)

    visual_mesh_paths: list[Path] = []
    collision_mesh_paths: list[Path] = []
    for link_element in root.findall("link"):
        visual_mesh_paths.extend(
            _geometry_mesh_paths(
                link_element,
                element_name="visual",
                source_path=resolved_path,
                require_mesh=True,
            )
        )
        collision_mesh_paths.extend(
            _geometry_mesh_paths(
                link_element,
                element_name="collision",
                source_path=resolved_path,
                require_mesh=False,
            )
        )

    joints = []
    joint_names = []
    parent_by_child: dict[str, str] = {}
    children = set()
    joints_by_parent: dict[str, list[str]] = {}
    for joint_element in root.findall("joint"):
        name = str(joint_element.attrib.get("name") or "").strip()
        if not name:
            raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} joint name is required")
        joint_names.append(name)
        joint_type = str(joint_element.attrib.get("type") or "").strip().lower()
        if joint_type not in SUPPORTED_JOINT_TYPES:
            raise UrdfSourceError(
                f"{_relative_to_repo(resolved_path)} joint {name!r} uses unsupported type {joint_type!r}"
            )
        parent_element = joint_element.find("parent")
        child_element = joint_element.find("child")
        parent_link = str(parent_element.attrib.get("link") if parent_element is not None else "").strip()
        child_link = str(child_element.attrib.get("link") if child_element is not None else "").strip()
        if not parent_link or not child_link:
            raise UrdfSourceError(
                f"{_relative_to_repo(resolved_path)} joint {name!r} must define parent and child links"
            )
        if parent_link not in link_name_set:
            raise UrdfSourceError(
                f"{_relative_to_repo(resolved_path)} joint {name!r} references missing parent link {parent_link!r}"
            )
        if child_link not in link_name_set:
            raise UrdfSourceError(
                f"{_relative_to_repo(resolved_path)} joint {name!r} references missing child link {child_link!r}"
            )
        if child_link in parent_by_child:
            raise UrdfSourceError(
                f"{_relative_to_repo(resolved_path)} link {child_link!r} has multiple parents"
            )
        parent_by_child[child_link] = parent_link
        children.add(child_link)
        joints_by_parent.setdefault(parent_link, []).append(child_link)
        min_value_deg, max_value_deg = _joint_limits_deg(joint_element, joint_type=joint_type, source_path=resolved_path)
        joints.append(
            UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent_link=parent_link,
                child_link=child_link,
                min_value_deg=min_value_deg,
                max_value_deg=max_value_deg,
            )
        )

    _raise_on_duplicates(joint_names, source_path=resolved_path, label="joint")
    root_candidates = [link_name for link_name in link_names if link_name not in children]
    if len(root_candidates) != 1:
        raise UrdfSourceError(
            f"{_relative_to_repo(resolved_path)} must form a single rooted tree; found roots {root_candidates!r}"
        )
    root_link = root_candidates[0]

    visited: set[str] = set()
    visiting: set[str] = set()

    def visit(link_name: str) -> None:
        if link_name in visited:
            return
        if link_name in visiting:
            raise UrdfSourceError(f"{_relative_to_repo(resolved_path)} joint graph contains a cycle")
        visiting.add(link_name)
        for child_link in joints_by_parent.get(link_name, ()):
            visit(child_link)
        visiting.remove(link_name)
        visited.add(link_name)

    visit(root_link)
    if visited != link_name_set:
        missing_links = sorted(link_name_set - visited)
        raise UrdfSourceError(
            f"{_relative_to_repo(resolved_path)} leaves links disconnected from the root: {missing_links!r}"
        )
    if len(joints) != len(link_names) - 1:
        raise UrdfSourceError(
            f"{_relative_to_repo(resolved_path)} must form a tree with exactly links-1 joints"
        )

    _validate_with_yourdfpy(resolved_path)

    return UrdfSource(
        file_ref=file_ref_from_urdf_path(resolved_path),
        source_path=resolved_path,
        robot_name=robot_name,
        root_link=root_link,
        links=tuple(link_names),
        joints=tuple(joints),
        mesh_paths=tuple(visual_mesh_paths + collision_mesh_paths),
        visual_mesh_paths=tuple(visual_mesh_paths),
        collision_mesh_paths=tuple(collision_mesh_paths),
    )


def _validate_link_inertials(root: ET.Element, *, source_path: Path) -> None:
    for link_element in root.findall("link"):
        link_name = str(link_element.attrib.get("name") or "").strip()
        inertial_element = link_element.find("inertial")
        if inertial_element is None:
            continue
        mass_element = inertial_element.find("mass")
        if mass_element is None:
            raise UrdfSourceError(
                f"{_relative_to_repo(source_path)} link {link_name!r} inertial requires <mass>"
            )
        mass = _required_float_attr(
            mass_element,
            "value",
            source_path=source_path,
            label=f"link {link_name!r} inertial mass",
        )
        if mass <= 0.0:
            raise UrdfSourceError(
                f"{_relative_to_repo(source_path)} link {link_name!r} inertial mass must be positive"
            )

        inertia_element = inertial_element.find("inertia")
        if inertia_element is None:
            raise UrdfSourceError(
                f"{_relative_to_repo(source_path)} link {link_name!r} inertial requires <inertia>"
            )
        ixx = _required_float_attr(
            inertia_element,
            "ixx",
            source_path=source_path,
            label=f"link {link_name!r} inertia ixx",
        )
        ixy = _required_float_attr(
            inertia_element,
            "ixy",
            source_path=source_path,
            label=f"link {link_name!r} inertia ixy",
        )
        ixz = _required_float_attr(
            inertia_element,
            "ixz",
            source_path=source_path,
            label=f"link {link_name!r} inertia ixz",
        )
        iyy = _required_float_attr(
            inertia_element,
            "iyy",
            source_path=source_path,
            label=f"link {link_name!r} inertia iyy",
        )
        iyz = _required_float_attr(
            inertia_element,
            "iyz",
            source_path=source_path,
            label=f"link {link_name!r} inertia iyz",
        )
        izz = _required_float_attr(
            inertia_element,
            "izz",
            source_path=source_path,
            label=f"link {link_name!r} inertia izz",
        )
        _validate_inertia_values(
            link_name,
            ixx=ixx,
            ixy=ixy,
            ixz=ixz,
            iyy=iyy,
            iyz=iyz,
            izz=izz,
            source_path=source_path,
        )


def _required_float_attr(
    element: ET.Element,
    attr_name: str,
    *,
    source_path: Path,
    label: str,
) -> float:
    try:
        value = element.attrib[attr_name]
    except KeyError as exc:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} {label} requires {attr_name!r}"
        ) from exc
    try:
        return float(value)
    except ValueError as exc:
        raise UrdfSourceError(f"{_relative_to_repo(source_path)} {label} is invalid") from exc


def _validate_inertia_values(
    link_name: str,
    *,
    ixx: float,
    ixy: float,
    ixz: float,
    iyy: float,
    iyz: float,
    izz: float,
    source_path: Path,
) -> None:
    del ixy, ixz, iyz
    if ixx <= 0.0 or iyy <= 0.0 or izz <= 0.0:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} link {link_name!r} inertia diagonal values must be positive"
        )
    tolerance = 1e-12
    if ixx + iyy + tolerance < izz or ixx + izz + tolerance < iyy or iyy + izz + tolerance < ixx:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} link {link_name!r} inertia violates triangle inequalities"
        )


def _geometry_mesh_paths(
    link_element: ET.Element,
    *,
    element_name: str,
    source_path: Path,
    require_mesh: bool,
) -> list[Path]:
    mesh_paths: list[Path] = []
    for geometry_owner in link_element.findall(element_name):
        geometry_element = geometry_owner.find("geometry")
        if geometry_element is None:
            continue
        mesh_element = geometry_element.find("mesh")
        if mesh_element is None:
            if require_mesh:
                raise UrdfSourceError(
                    f"{_relative_to_repo(source_path)} only mesh {element_name} geometry is supported"
                )
            if not any(geometry_element.find(tag) is not None for tag in SUPPORTED_COLLISION_PRIMITIVES):
                supported = ", ".join(sorted((*SUPPORTED_COLLISION_PRIMITIVES, "mesh")))
                raise UrdfSourceError(
                    f"{_relative_to_repo(source_path)} {element_name} geometry must use one of: {supported}"
                )
            continue
        mesh_paths.append(_validated_mesh_path(mesh_element, source_path=source_path))
    return mesh_paths


def _validated_mesh_path(mesh_element: ET.Element, *, source_path: Path) -> Path:
    filename = str(mesh_element.attrib.get("filename") or "").strip()
    if not filename:
        raise UrdfSourceError(f"{_relative_to_repo(source_path)} mesh filename is required")
    mesh_path = _resolve_mesh_path(filename, source_path=source_path)
    if mesh_path.suffix.lower() not in SUPPORTED_MESH_SUFFIXES:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} only STL mesh geometry is supported: {filename!r}"
        )
    if not mesh_path.is_file():
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} references missing mesh file: {filename!r}"
        )
    return mesh_path


def _validate_with_yourdfpy(source_path: Path) -> None:
    try:
        from yourdfpy import URDF
    except ModuleNotFoundError as exc:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} requires yourdfpy for URDF validation; "
            "install it in the active Python environment"
        ) from exc

    try:
        urdf = URDF.load(
            str(source_path),
            build_scene_graph=False,
            build_collision_scene_graph=False,
            load_meshes=False,
            load_collision_meshes=False,
        )
        is_valid = urdf.validate()
    except Exception as exc:
        raise UrdfSourceError(f"{_relative_to_repo(source_path)} failed yourdfpy validation: {exc}") from exc

    if not is_valid:
        errors = "; ".join(str(error) for error in urdf.errors) or "unknown error"
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} failed yourdfpy validation: {errors}"
        )


def _joint_limits_deg(
    joint_element: ET.Element,
    *,
    joint_type: str,
    source_path: Path,
) -> tuple[float | None, float | None]:
    if joint_type == "fixed":
        return 0.0, 0.0
    if joint_type == "continuous":
        return -180.0, 180.0
    limit_element = joint_element.find("limit")
    if limit_element is None:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} {joint_type} joint {joint_element.attrib.get('name', '')!r} requires <limit>"
        )
    try:
        lower = float(limit_element.attrib["lower"])
        upper = float(limit_element.attrib["upper"])
    except KeyError as exc:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} {joint_type} joint {joint_element.attrib.get('name', '')!r} requires lower and upper limits"
        ) from exc
    except ValueError as exc:
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} {joint_type} joint {joint_element.attrib.get('name', '')!r} has invalid limits"
        ) from exc
    if joint_type == "prismatic":
        return lower, upper
    return degrees(lower), degrees(upper)


def _resolve_mesh_path(filename: str, *, source_path: Path) -> Path:
    if filename.startswith("package://"):
        relative = filename.removeprefix("package://").lstrip("/")
        return (Path.cwd() / relative).resolve()
    return (source_path.parent / filename).resolve()


def _raise_on_duplicates(values: list[str], *, source_path: Path, label: str) -> None:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
            continue
        seen.add(value)
    if duplicates:
        duplicate_text = ", ".join(repr(item) for item in sorted(duplicates))
        raise UrdfSourceError(
            f"{_relative_to_repo(source_path)} {label} names contain duplicates {duplicate_text}"
        )


def _relative_to_repo(path: Path) -> str:
    try:
        return path.resolve().relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()
