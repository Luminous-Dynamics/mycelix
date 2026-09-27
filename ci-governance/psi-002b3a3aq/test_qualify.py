#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("a3aq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_subject_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "b4985d90813a3cb303eb5d253531b7fc981a3c13")
        self.assertEqual(Q.PRODUCT_PARENT, "e32b54c86d602989955820fe1be5cbe88490e1e5")
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

    def test_authority_ceiling_is_non_promoting(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "StructuralIssuerDirectoryOnly")
        for field in (
            "directory_payload_authenticated",
            "retrieval_provider_trusted",
            "directory_freshness_established",
            "trusted_clock_not_before_established",
            "issuer_key_admitted",
            "issuer_key_current",
            "token_verified",
            "query_credit_granted",
            "production_admission",
            "application_authority",
        ):
            self.assertFalse(authority[field])

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_REPLACE_REF_BASE": "refs/evil"})
        Q.reject_git_env_overrides({})

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"z": 1, "a": 2}), b'{"a":2,"z":1}\n')


if __name__ == "__main__":
    unittest.main()
