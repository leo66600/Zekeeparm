"""Verify source and installed nodes locate the same perception modules."""

import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from zekeep_shadow import paths


class PerceptionRootTest(unittest.TestCase):
    def test_source_and_installed_locations_find_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            root = workspace / "src" / "zekeep_grasp"
            (root / "utils").mkdir(parents=True)
            (root / "utils" / "camera_utils.py").touch()
            for location in (
                "src/zekeep_shadow/zekeep_shadow/paths.py",
                "install/zekeep_shadow/lib/python3.10/site-packages/zekeep_shadow/paths.py",
            ):
                with self.subTest(location=location), patch.dict(os.environ, {"ZKEEP_WS": ""}), \
                        patch.object(paths, "__file__", str(workspace / location)):
                    self.assertEqual(paths.default_perception_root(), str(root))

    def test_explicit_workspace_takes_precedence(self):
        with patch.dict(os.environ, {"ZKEEP_WS": "~/custom-workspace"}):
            self.assertEqual(
                paths.default_perception_root(),
                str(Path.home() / "custom-workspace/src/zekeep_grasp"),
            )

    def test_external_install_can_use_current_workspace(self):
        with patch.dict(os.environ, {"ZKEEP_WS": ""}), \
                patch.object(paths, "__file__", "/unrelated/zekeep_shadow/paths.py"), \
                patch.object(Path, "cwd", return_value=Path("/custom-workspace")):
            self.assertEqual(paths.default_perception_root(), "/custom-workspace/src/zekeep_grasp")
