#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b3b2aq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)

class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self):
        self.assertEqual(Q.PRODUCT_COMMIT, "1eca62e9732372cbae1230d957123c57c0f8a49d")
        self.assertEqual(Q.PRODUCT_PARENT, "10eab4ec3c5833d9230fc9bfde89547a0cea1246")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 8)

    def test_sqlite_pin_and_offline_gate_are_frozen(self):
        self.assertIn("--offline", Q.COMMANDS[1])
        self.assertIn("--offline", Q.COMMANDS[2])
        self.assertIn("warnings", Q.COMMANDS[2])

    def test_dependency_lineage_is_exact(self):
        self.assertEqual(len(Q.PRODUCT_BLOBS), 3)
        self.assertEqual(len(Q.B3B1_BLOBS), 3)
        self.assertEqual(len(Q.B3A_BLOBS), 3)

if __name__ == "__main__":
    unittest.main()
