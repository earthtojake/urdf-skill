import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from urdf_source import UrdfSourceError, read_urdf_source


class UrdfSourceTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tempdir = tempfile.TemporaryDirectory(prefix="tmp-urdf-source-")
        self.temp_root = Path(self._tempdir.name)

    def tearDown(self) -> None:
        self._tempdir.cleanup()

    def _cad_ref(self, name: str) -> str:
        return (self.temp_root / f"{name}.urdf").resolve().as_posix()

    def _write_mesh(self, name: str) -> Path:
        mesh_path = self.temp_root / f"{name}.stl"
        mesh_path.write_text("solid empty\nendsolid empty\n", encoding="utf-8")
        return mesh_path

    def _write_urdf(self, name: str, body: str) -> Path:
        urdf_path = self.temp_root / f"{name}.urdf"
        urdf_path.write_text(body.strip() + "\n", encoding="utf-8")
        script_path = self.temp_root / f"{name}.py"
        if not script_path.exists():
            script_path.write_text(
                "\n".join(
                    [
                        "def gen_step():",
                        f"    return {{'instances': [], 'step_output': {f'{name}.step'!r}}}",
                        "",
                        "def gen_urdf():",
                        f"    return {{'xml': '', 'urdf_output': {f'{name}.urdf'!r}}}",
                        "",
                    ]
                ),
                encoding="utf-8",
            )
        return urdf_path

    def test_read_urdf_source_accepts_valid_mesh_robot(self) -> None:
        mesh_path = self._write_mesh("base")
        source_path = self._write_urdf(
            "robot",
            f"""
            <robot name="sample-robot">
              <link name="base_link">
                <visual>
                  <geometry>
                    <mesh filename="{mesh_path.name}" scale="0.001 0.001 0.001" />
                  </geometry>
                </visual>
              </link>
            </robot>
            """,
        )

        source = read_urdf_source(source_path)

        self.assertEqual(self._cad_ref("robot"), source.file_ref)
        self.assertEqual("sample-robot", source.robot_name)
        self.assertEqual("base_link", source.root_link)
        self.assertEqual(("base_link",), source.links)
        self.assertEqual(0, len(source.joints))
        self.assertEqual((mesh_path.resolve(),), source.mesh_paths)

    def test_read_urdf_source_validates_with_yourdfpy_without_loading_meshes(self) -> None:
        source_path = self._write_urdf(
            "robot",
            """
            <robot name="sample-robot">
              <link name="base_link" />
            </robot>
            """,
        )
        urdf = Mock()
        urdf.validate.return_value = True
        urdf.errors = []

        with patch("yourdfpy.URDF.load", return_value=urdf) as load:
            read_urdf_source(source_path)

        load.assert_called_once()
        self.assertEqual(str(source_path.resolve()), load.call_args.args[0])
        self.assertFalse(load.call_args.kwargs["build_scene_graph"])
        self.assertFalse(load.call_args.kwargs["build_collision_scene_graph"])
        self.assertFalse(load.call_args.kwargs["load_meshes"])
        self.assertFalse(load.call_args.kwargs["load_collision_meshes"])
        urdf.validate.assert_called_once_with()

    def test_read_urdf_source_reports_yourdfpy_validation_errors(self) -> None:
        source_path = self._write_urdf(
            "robot",
            """
            <robot name="sample-robot">
              <link name="base_link" />
            </robot>
            """,
        )
        urdf = Mock()
        urdf.validate.return_value = False
        urdf.errors = ["bad joint"]

        with patch("yourdfpy.URDF.load", return_value=urdf):
            with self.assertRaisesRegex(UrdfSourceError, "bad joint"):
                read_urdf_source(source_path)

    def test_read_urdf_source_rejects_duplicate_links(self) -> None:
        mesh_path = self._write_mesh("base")
        source_path = self._write_urdf(
            "robot",
            f"""
            <robot name="sample-robot">
              <link name="base_link">
                <visual>
                  <geometry>
                    <mesh filename="{mesh_path.name}" />
                  </geometry>
                </visual>
              </link>
              <link name="base_link" />
            </robot>
            """,
        )

        with self.assertRaisesRegex(UrdfSourceError, "duplicates"):
            read_urdf_source(source_path)

    def test_read_urdf_source_rejects_missing_mesh(self) -> None:
        source_path = self._write_urdf(
            "robot",
            """
            <robot name="sample-robot">
              <link name="base_link">
                <visual>
                  <geometry>
                    <mesh filename="does-not-exist.stl" />
                  </geometry>
                </visual>
              </link>
            </robot>
            """,
        )

        with self.assertRaisesRegex(UrdfSourceError, "missing mesh file"):
            read_urdf_source(source_path)

    def test_read_urdf_source_rejects_unsupported_joint_type(self) -> None:
        mesh_path = self._write_mesh("base")
        source_path = self._write_urdf(
            "robot",
            f"""
            <robot name="sample-robot">
              <link name="base_link">
                <visual>
                  <geometry>
                    <mesh filename="{mesh_path.name}" />
                  </geometry>
                </visual>
              </link>
              <link name="arm_link" />
              <joint name="base_to_arm" type="prismatic">
                <parent link="base_link" />
                <child link="arm_link" />
              </joint>
            </robot>
            """,
        )

        with self.assertRaisesRegex(UrdfSourceError, "unsupported type"):
            read_urdf_source(source_path)

    def test_file_ref_ignores_neighbor_step_toml(self) -> None:
        source_path = self._write_urdf(
            "robot",
            """
            <robot name="sample-robot">
              <link name="base_link" />
            </robot>
            """,
        )
        stale_path = self.temp_root / "robot.step.toml"
        stale_path.write_text(
            "\n".join(
                [
                    'kind = "part"',
                    'source = "robot.urdf"',
                ]
            )
            + "\n",
            encoding="utf-8",
        )

        source = read_urdf_source(source_path)

        self.assertEqual(source_path.resolve().as_posix(), source.file_ref)


if __name__ == "__main__":
    unittest.main()
