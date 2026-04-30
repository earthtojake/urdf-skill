import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from gen_urdf import cli


class GenUrdfCliTests(unittest.TestCase):
    def test_requires_explicit_target(self) -> None:
        with self.assertRaises(SystemExit) as cm:
            cli.main([])
        self.assertEqual(2, cm.exception.code)

    def test_rejects_root_option(self) -> None:
        with self.assertRaises(SystemExit) as cm:
            cli.main(["--root", "sample_root"])
        self.assertEqual(2, cm.exception.code)

    def test_passes_targets_in_order(self) -> None:
        with mock.patch.object(cli, "generate_urdf_targets", return_value=0) as generate:
            self.assertEqual(0, cli.main(["sample_robot.py", "other.py", "--summary"]))

        generate.assert_called_once_with(["sample_robot.py", "other.py"], summary=True)

    def test_generates_urdf_without_external_tool_imports(self) -> None:
        with tempfile.TemporaryDirectory(prefix="tmp-gen-urdf-") as tempdir:
            source_path = Path(tempdir) / "sample_robot.py"
            source_path.write_text(
                "\n".join(
                    [
                        "def gen_urdf():",
                        "    return {",
                        "        'xml': '<robot name=\"sample\"><link name=\"base_link\" /></robot>',",
                        "        'urdf_output': 'sample_robot.urdf',",
                        "    }",
                        "",
                    ]
                ),
                encoding="utf-8",
            )

            self.assertEqual(0, cli.generate_urdf_targets([str(source_path)]))

            self.assertEqual(
                '<robot name="sample"><link name="base_link" /></robot>\n',
                (Path(tempdir) / "sample_robot.urdf").read_text(encoding="utf-8"),
            )

    def test_generates_explorer_metadata_sidecar_when_present(self) -> None:
        with tempfile.TemporaryDirectory(prefix="tmp-gen-urdf-") as tempdir:
            source_path = Path(tempdir) / "sample_robot.py"
            source_path.write_text(
                "\n".join(
                    [
                        "def gen_urdf():",
                        "    return {",
                        "        'xml': '<robot name=\"sample\"><link name=\"base_link\" /></robot>',",
                        "        'urdf_output': 'sample_robot.urdf',",
                        "        'explorer_metadata': {",
                        "            'schemaVersion': 1,",
                        "            'kind': 'example-urdf-consumer',",
                        "            'defaultJoints': {'joint_1': 90},",
                        "            'poses': [],",
                        "        },",
                        "    }",
                        "",
                    ]
                ),
                encoding="utf-8",
            )

            self.assertEqual(0, cli.generate_urdf_targets([str(source_path)]))

            sidecar_path = Path(tempdir) / ".sample_robot.urdf" / "explorer.json"
            self.assertEqual(
                {
                    "schemaVersion": 1,
                    "kind": "example-urdf-consumer",
                    "defaultJoints": {"joint_1": 90},
                    "poses": [],
                },
                json.loads(sidecar_path.read_text(encoding="utf-8")),
            )

if __name__ == "__main__":
    unittest.main()
