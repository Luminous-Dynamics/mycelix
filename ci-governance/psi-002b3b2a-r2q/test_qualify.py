#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi_b3b2a_r2q", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)

class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "e48e3bc6e9cb203084d2a5aad61c5ead1ada515d")
        self.assertEqual(Q.PRODUCT_PARENT, "be4fabe15a7f9180db13d85725a020566d3f9095")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 9)

    def test_exact_sqlite_dependency_is_frozen(self) -> None:
        cargo_path = f"{Q.SQLITE_CRATE}/Cargo.toml"
        self.assertEqual(Q.PRODUCT_BLOBS[cargo_path], "7d3801e316ccbe5f1874e71c448c6f50fc31da68")

    def test_offline_commands_are_strict(self) -> None:
        self.assertIn("--offline", Q.COMMANDS[1])
        self.assertIn("--offline", Q.COMMANDS[2])
        self.assertEqual(Q.COMMANDS[2][-2:], ("-D", "warnings"))

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_INDEX_FILE": "/tmp/index"})
        Q.reject_git_env_overrides({})

    def test_authority_ceiling_is_sqlite_profile_only(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "SqliteWalFullNonceReplayStoreOnly")
        self.assertTrue(authority["sqlite_atomic_single_use_established"])
        self.assertTrue(authority["sqlite_full_sync_profile_established"])
        self.assertFalse(authority["privacy_pass_backend_qualified"])
        self.assertFalse(authority["global_store_uniqueness_established"])
        self.assertFalse(authority["hardware_power_loss_durability_established"])
        self.assertFalse(authority["safe_compaction_established"])
        self.assertFalse(authority["query_credit_granted"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')

if __name__ == "__main__":
    unittest.main()
