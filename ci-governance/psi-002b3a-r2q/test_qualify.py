#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b3ar2q", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "e32b54c86d602989955820fe1be5cbe88490e1e5")
        self.assertEqual(Q.PRODUCT_TREE, "e24774507e1a7c29d757b5ed9c7a033952ab8775")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 11)

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
            Q.reject_git_env_overrides({"GIT_OBJECT_DIRECTORY": "/tmp/evil"})
        Q.reject_git_env_overrides({})

    def test_authority_ceiling_is_non_promoting(self) -> None:
        lock = Q.expected_lock(
            {"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"}
        )
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "StructuralRfcSemanticsOnly")
        self.assertFalse(authority["privacy_pass_backend_qualified"])
        self.assertFalse(authority["token_cryptographically_verified"])
        self.assertFalse(authority["token_nonce_cryptographically_bound"])
        self.assertFalse(authority["challenge_digest_cryptographically_bound"])
        self.assertFalse(authority["atomic_single_use_established"])
        self.assertFalse(authority["query_credit_granted"])
        self.assertFalse(authority["production_admission"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')


if __name__ == "__main__":
    unittest.main()
