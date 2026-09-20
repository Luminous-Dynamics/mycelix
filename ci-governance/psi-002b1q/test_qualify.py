#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b1q", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "c5c892ab096c5454f8e0bcad87940e0e7ac1a849")
        self.assertEqual(Q.PRODUCT_PARENT, "4cd3bbc47c27a3f11df7b3308ea0888adbf2322e")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 13)

    def test_command_vector_is_offline(self) -> None:
        self.assertEqual(
            Q.COMMANDS,
            (
                ("cargo", "fmt", "--check", "--all"),
                ("cargo", "test", "--offline", "--workspace"),
                ("cargo", "clippy", "--offline", "--workspace", "--all-targets", "--", "-D", "warnings"),
            ),
        )

    def test_git_override_fails_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_DIR": "/tmp/not-the-checkout"})
        Q.reject_git_env_overrides({})

    def test_lock_ceiling_cannot_promote_privacy(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "StructuralOnly")
        for key in (
            "query_token_unlinkability_established",
            "abuse_resistance_established",
            "transport_unlinkability_established",
            "registry_authenticity_established",
            "registry_currentness_established",
            "composition_qualified",
            "production_admission",
            "application_authority",
        ):
            self.assertFalse(authority[key])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')


if __name__ == "__main__":
    unittest.main()
