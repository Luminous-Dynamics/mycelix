#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from tests.conductor.test_holochain_real_driver import CONTROL_SOURCE, CONTRACT, hash_file

ROOT = Path(__file__).resolve().parent
TOOL = ROOT / "holochain_driver_config.py"


class HolochainDriverConfigTests(unittest.TestCase):
    def control(self, root: Path) -> Path:
        path = root / "control.py"
        path.write_text(CONTROL_SOURCE, encoding="utf-8")
        path.chmod(0o755)
        return path

    def test_create_and_verify(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); control = self.control(root); config = root / "config.json"
            result = subprocess.run([
                str(TOOL), "--create", "--config", str(config), "--control", str(control),
                "--scenario-contract", str(CONTRACT), "--step-timeout-seconds", "30",
            ], text=True, capture_output=True, timeout=30, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(config.read_text())
            self.assertEqual(value["control_executable_sha256"], hash_file(control))
            self.assertEqual(config.stat().st_mode & 0o777, 0o444)
            self.assertIn("runtime_lock_hash=", result.stdout)

    def test_existing_output_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); control = self.control(root); config = root / "config.json"; config.write_text("{}")
            result = subprocess.run([
                str(TOOL), "--create", "--config", str(config), "--control", str(control),
                "--scenario-contract", str(CONTRACT),
            ], text=True, capture_output=True, timeout=30, check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("already exists", result.stderr)

    def test_control_drift_is_rejected_on_verify(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); control = self.control(root); config = root / "config.json"
            create = subprocess.run([
                str(TOOL), "--create", "--config", str(config), "--control", str(control),
                "--scenario-contract", str(CONTRACT),
            ], text=True, capture_output=True, timeout=30, check=False)
            self.assertEqual(create.returncode, 0, create.stderr)
            control.write_text(control.read_text() + "\n# drift\n"); control.chmod(0o755)
            result = subprocess.run([str(TOOL), "--verify", "--config", str(config)], text=True, capture_output=True, timeout=30, check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("hash mismatch", result.stderr)


if __name__ == "__main__":
    unittest.main()
