#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b3aq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)

class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self):
        self.assertEqual(Q.PRODUCT_COMMIT, "1a5fdee47750c2f90e76e4e9dd4374603b56cfd7")
        self.assertEqual(Q.PRODUCT_PARENT, "c5c892ab096c5454f8e0bcad87940e0e7ac1a849")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 10)

    def test_offline_gate_is_frozen(self):
        self.assertEqual(Q.COMMANDS[0], ("cargo", "fmt", "--check", "--all"))
        self.assertIn("--offline", Q.COMMANDS[1])
        self.assertIn("--offline", Q.COMMANDS[2])
        self.assertIn("warnings", Q.COMMANDS[2])

    def test_authority_shortcuts_are_source_prohibited(self):
        self.assertEqual(Q.FORBIDDEN_AUTHORITY_FIELDS, ("pub unspent:", "pub consumed:", "pub spent:"))

    def test_product_blob_set_is_exact(self):
        self.assertEqual(len(Q.PRODUCT_BLOBS), 3)
        self.assertIn("mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/src/lib.rs", Q.PRODUCT_BLOBS)

if __name__ == "__main__": unittest.main()
