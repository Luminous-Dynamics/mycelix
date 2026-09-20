#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002b2bq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "d6c8acf5ad2e88f7454fe161f75e2446df5e2c26")
        self.assertEqual(Q.PRODUCT_PARENT, "d63eb7c2dec46c3b0b6faeee3062bdfded0ba3b6")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 8)

    def test_commands_are_offline_and_warnings_denied(self) -> None:
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
            Q.reject_git_env({"GIT_OBJECT_DIRECTORY": "/tmp/evil"})
        Q.reject_git_env({})

    def test_authority_ceiling_does_not_promote_currentness_or_producer(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "ConsumerProviderVerificationExecutionOnly")
        self.assertFalse(authority["xenia_producer_qualified"])
        self.assertFalse(authority["registry_current"])
        self.assertFalse(authority["psi_security_established"])
        self.assertFalse(authority["composition_qualified"])
        self.assertFalse(authority["production_admission"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"z": 1, "a": 2}), b'{"a":2,"z":1}\n')


if __name__ == "__main__":
    unittest.main()
