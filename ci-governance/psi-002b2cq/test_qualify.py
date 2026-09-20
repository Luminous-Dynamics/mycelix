#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b2cq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "cb2124800d98d1e51d21db37c44e0fe650611802")
        self.assertEqual(Q.PRODUCT_PARENT, "d63eb7c2dec46c3b0b6faeee3062bdfded0ba3b6")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 9)

    def test_offline_commands_are_frozen(self) -> None:
        self.assertEqual(
            Q.COMMANDS,
            (
                ("cargo", "fmt", "--check", "--all"),
                ("cargo", "test", "--offline", "--all-targets"),
                ("cargo", "clippy", "--offline", "--all-targets", "--all-features", "--", "-D", "warnings"),
            ),
        )

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env({"GIT_INDEX_FILE": "/tmp/evil"})
        Q.reject_git_env({})

    def test_authority_ceiling_remains_structural_only(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "StructuralCurrentnessClaimExecutionOnly")
        self.assertFalse(authority["provider_evidence_verified"])
        self.assertFalse(authority["trusted_clock_established"])
        self.assertFalse(authority["completeness_established"])
        self.assertFalse(authority["registry_current"])
        self.assertFalse(authority["composition_qualified"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')


if __name__ == "__main__":
    unittest.main()
