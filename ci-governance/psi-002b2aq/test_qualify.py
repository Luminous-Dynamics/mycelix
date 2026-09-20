#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b2aq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "d63eb7c2dec46c3b0b6faeee3062bdfded0ba3b6")
        self.assertEqual(Q.PRODUCT_PARENT, "c5c892ab096c5454f8e0bcad87940e0e7ac1a849")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 10)

    def test_offline_execution_vector(self) -> None:
        self.assertEqual(
            Q.COMMANDS,
            (
                ("cargo", "fmt", "--check", "--all"),
                ("cargo", "test", "--offline", "--workspace"),
                ("cargo", "clippy", "--offline", "--workspace", "--all-targets", "--", "-D", "warnings"),
            ),
        )

    def test_git_override_is_rejected(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_REPLACE_REF_BASE": "refs/replace"})
        Q.reject_git_env_overrides({})

    def test_lock_ceiling_remains_structural(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "StructuralOnly")
        self.assertFalse(authority["provider_cryptographically_verified"])
        self.assertFalse(authority["registry_authenticated"])
        self.assertFalse(authority["registry_current"])
        self.assertFalse(authority["composition_qualified"])
        self.assertFalse(authority["production_admission"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')


if __name__ == "__main__":
    unittest.main()
