#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi002aq", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "4cd3bbc47c27a3f11df7b3308ea0888adbf2322e")
        self.assertEqual(Q.PRODUCT_PARENT, "41a26efa89435fbc328bb5ac68b9e971f4b162cd")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 11)

    def test_offline_commands_are_frozen(self) -> None:
        self.assertEqual(
            Q.COMMANDS,
            (
                ("cargo", "fmt", "--check", "--all"),
                ("cargo", "test", "--offline", "--workspace"),
                ("cargo", "clippy", "--offline", "--workspace", "--all-targets", "--", "-D", "warnings"),
            ),
        )

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_OBJECT_DIRECTORY": "/tmp/evil"})
        Q.reject_git_env_overrides({})

    def test_lock_authority_ceiling_is_non_promoting(self) -> None:
        lock = Q.expected_lock(
            {
                "README.md": "a",
                "qualify.py": "b",
                "test_qualify.py": "c",
            }
        )
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "ExperimentalExecutionOnly")
        self.assertFalse(authority["voprf_backend_qualified"])
        self.assertFalse(authority["psi_security_established"])
        self.assertFalse(authority["enumeration_resistance_established"])
        self.assertFalse(authority["client_anonymity_established"])
        self.assertFalse(authority["production_admission"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')

    def test_voprf_manifest_is_exactly_pinned_in_static_contract(self) -> None:
        cargo = Q.PRODUCT_BLOBS[
            "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/Cargo.toml"
        ]
        self.assertEqual(cargo, "d4d45713f0dd15f6d2ad0558dd24ba5a05ed6c94")


if __name__ == "__main__":
    unittest.main()
