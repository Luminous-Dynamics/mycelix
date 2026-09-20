import json
import pathlib
import tempfile
import unittest

import qualify


class Tests(unittest.TestCase):
    def test_registry_is_exact(self):
        self.assertEqual(
            qualify.PRODUCT_COMMIT,
            "41a26efa89435fbc328bb5ac68b9e971f4b162cd",
        )
        self.assertEqual(
            qualify.PRODUCT_TREE,
            "c551c2a4588512fa3c99c1c82a14f8b2a25aa820",
        )
        self.assertEqual(len(qualify.PRODUCT_BLOBS), 7)
        self.assertEqual(len(qualify.CORE_BLOBS), 2)
        self.assertEqual(qualify.EXPECTED_TEST_COUNT, 37)

    def test_qualifier_paths_closed(self):
        self.assertEqual(len(qualify.QUALIFIER_PATHS), 4)
        self.assertTrue(
            all(
                path.startswith("ci-governance/pec-002aq/")
                for path in qualify.QUALIFIER_PATHS
            )
        )

    def test_expected_lock_is_closed(self):
        source_blobs = {
            "README.md": "a" * 40,
            "qualify.py": "b" * 40,
            "test_qualify.py": "c" * 40,
        }
        lock = qualify.expected_lock(source_blobs)
        self.assertEqual(lock["schema"], qualify.LOCK_SCHEMA)
        self.assertEqual(lock["product"]["commit"], qualify.PRODUCT_COMMIT)
        self.assertEqual(lock["qualifier"]["source_blobs"], source_blobs)
        self.assertFalse(lock["authority"]["network_access"])
        self.assertFalse(lock["authority"]["backend_qualification"])

    def test_expected_lock_has_canonical_bytes(self):
        source_blobs = {
            "README.md": "a" * 40,
            "qualify.py": "b" * 40,
            "test_qualify.py": "c" * 40,
        }
        expected = qualify.expected_lock(source_blobs)
        encoded = qualify.canonical(expected)
        self.assertEqual(
            encoded,
            qualify.canonical(json.loads(encoded.decode("utf-8"))),
        )

    def test_structural_probe_forbids_runtime_capabilities(self):
        source = pathlib.Path(qualify.__file__).read_text()
        for token in (
            'unsafe {',
            'unsafe fn',
            'extern "C"',
            "std::net",
            "std::process",
            "std::fs",
            "tokio::",
        ):
            self.assertIn(token, source)

    def test_canonical_is_stable(self):
        self.assertEqual(
            qualify.canonical({"b": 1, "a": 2}),
            b'{"a":2,"b":1}\n',
        )

    def test_receipt_schema(self):
        self.assertEqual(
            qualify.RECEIPT_SCHEMA,
            "mycelix.pec.002aq.receipt.v0.1",
        )

    def test_forbidden_git_environment_is_rejected(self):
        for name in qualify.FORBIDDEN_GIT_ENV:
            with self.subTest(name=name):
                with self.assertRaises(qualify.QualificationError):
                    qualify.reject_git_env_overrides({name: "attacker-controlled"})

    def test_clean_env_forces_git_integrity_controls(self):
        env = qualify.clean_env()
        self.assertEqual(env["GIT_NO_REPLACE_OBJECTS"], "1")
        self.assertEqual(env["GIT_CONFIG_NOSYSTEM"], "1")
        for name in qualify.FORBIDDEN_GIT_ENV:
            self.assertNotIn(name, env)

    def test_no_backend_authority_in_source(self):
        source = pathlib.Path(qualify.__file__).read_text()
        self.assertIn('"cryptographic_security_established": False', source)
        self.assertIn('"production_admission": False', source)
        self.assertIn('"application_authority": False', source)

    def test_no_network_client_imports(self):
        source = pathlib.Path(qualify.__file__).read_text()
        for token in (
            "import requests",
            "import urllib",
            "httpx",
            "aiohttp",
            "socket.socket",
        ):
            self.assertNotIn(token, source)

    def test_no_github_mutation_commands(self):
        source = pathlib.Path(qualify.__file__).read_text()
        for token in ("gh api", "curl ", "git push", "git commit", "git tag"):
            self.assertNotIn(token, source)

    def test_receipt_output_boundary_logic(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = pathlib.Path(temp_dir).resolve()
            inside = root / "x.json"
            outside = root.parent / (root.name + "-out.json")
            self.assertTrue(str(inside).startswith(str(root)))
            self.assertFalse(str(outside).startswith(str(root) + "/"))


if __name__ == "__main__":
    unittest.main()
