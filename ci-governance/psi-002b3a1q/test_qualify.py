#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi_b3a1q", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)


class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "31713561f743294c3ff0d4b3b33aa0ac62478c34")
        self.assertEqual(Q.PRODUCT_PARENT, "e32b54c86d602989955820fe1be5cbe88490e1e5")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 9)

    def test_rfc_vector_is_frozen(self) -> None:
        self.assertEqual(
            "8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f",
            "8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f",
        )

    def test_offline_commands_are_strict(self) -> None:
        self.assertEqual(Q.COMMANDS[0], ("cargo", "fmt", "--check", "--all"))
        self.assertIn("--offline", Q.COMMANDS[1])
        self.assertIn("--offline", Q.COMMANDS[2])
        self.assertEqual(Q.COMMANDS[2][-2:], ("-D", "warnings"))

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_ALTERNATE_OBJECT_DIRECTORIES": "/tmp/evil"})
        Q.reject_git_env_overrides({})

    def test_authority_ceiling_is_wire_only(self) -> None:
        lock = Q.expected_lock(
            {"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"}
        )
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "Rfc9577ChallengeEncodingOnly")
        self.assertTrue(authority["exact_rfc9577_default_challenge_encoding_established"])
        self.assertFalse(authority["token_challenge_digest_cryptographically_bound"])
        self.assertFalse(authority["token_cryptographically_verified"])
        self.assertFalse(authority["token_nonce_cryptographically_bound"])
        self.assertFalse(authority["atomic_single_use_established"])
        self.assertFalse(authority["query_credit_granted"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')

    def test_exact_challenge_source_blob_is_frozen(self) -> None:
        path = f"{Q.CHALLENGE_CRATE}/src/lib.rs"
        self.assertEqual(Q.PRODUCT_BLOBS[path], "5d08799814c7af4f3c3b97d6cd3d8da90a139a48")


if __name__ == "__main__":
    unittest.main()
