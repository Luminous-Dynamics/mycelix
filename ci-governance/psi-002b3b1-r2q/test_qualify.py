#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("psi_b3b1_r2q", HERE / "qualify.py")
assert SPEC and SPEC.loader
Q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(Q)

class QualifierSourceTests(unittest.TestCase):
    def test_exact_product_identity(self) -> None:
        self.assertEqual(Q.PRODUCT_COMMIT, "be4fabe15a7f9180db13d85725a020566d3f9095")
        self.assertEqual(Q.PRODUCT_PARENT, "e32b54c86d602989955820fe1be5cbe88490e1e5")
        self.assertEqual(Q.EXPECTED_TEST_COUNT, 9)

    def test_command_vector_is_offline_and_strict(self) -> None:
        self.assertEqual(Q.COMMANDS[0], ("cargo", "fmt", "--check", "--all"))
        self.assertIn("--offline", Q.COMMANDS[1])
        self.assertIn("--offline", Q.COMMANDS[2])
        self.assertEqual(Q.COMMANDS[2][-2:], ("-D", "warnings"))

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(Q.QualificationError):
            Q.reject_git_env_overrides({"GIT_OBJECT_DIRECTORY": "/tmp/evil"})
        Q.reject_git_env_overrides({})

    def test_authority_ceiling_is_process_local_only(self) -> None:
        lock = Q.expected_lock({"README.md": "a", "qualify.py": "b", "test_qualify.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "ProcessLocalAtomicSpendReferenceOnly")
        self.assertTrue(authority["process_local_atomic_single_use_established"])
        self.assertFalse(authority["durable_single_use_established"])
        self.assertFalse(authority["multi_process_single_use_established"])
        self.assertFalse(authority["crash_safe_single_use_established"])
        self.assertFalse(authority["privacy_pass_backend_qualified"])
        self.assertFalse(authority["query_credit_granted"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(Q.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')

    def test_exact_nonce_bound_source_blob_is_frozen(self) -> None:
        path = f"{Q.SPEND_CRATE}/src/lib.rs"
        self.assertEqual(Q.PRODUCT_BLOBS[path], "130b639f499a858b7ad4be1a44de8f484bf07da9")

if __name__ == "__main__":
    unittest.main()
