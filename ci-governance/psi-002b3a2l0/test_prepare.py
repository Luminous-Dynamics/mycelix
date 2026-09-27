#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import pathlib
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("a2pl0", HERE / "prepare.py")
assert SPEC and SPEC.loader
P = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(P)


class PreparationSourceTests(unittest.TestCase):
    def test_exact_product_and_backend_identity(self) -> None:
        self.assertEqual(P.PRODUCT_COMMIT, "ea7e0d6c73f0bb775e6dd9f4497f51237749d3fe")
        self.assertEqual(P.PRODUCT_TREE, "5587844d471ca2640160a13aa04521f766221d0e")
        self.assertEqual(P.BACKEND_COMMIT, "5ff5f57a62877f42313d6600b53e0d4ee4e4e452")
        self.assertEqual(P.BACKEND_TREE, "014106e30cddf78dafd6cb4ae7c6e20c755e862d")

    def test_backend_relevant_source_blobs_are_frozen(self) -> None:
        for path in (
            "Cargo.toml",
            "Cargo.lock",
            "src/lib.rs",
            "src/auth/authenticate.rs",
            "src/auth/authorize.rs",
            "src/public_tokens/mod.rs",
            "src/public_tokens/server.rs",
            "src/public_tokens/request.rs",
            "src/public_tokens/response.rs",
            "tests/kat_vectors/public_go.json",
        ):
            self.assertIn(path, P.BACKEND_BLOBS)
            self.assertEqual(len(P.BACKEND_BLOBS[path]), 40)

    def test_git_environment_overrides_fail_closed(self) -> None:
        with self.assertRaises(P.PreparationError):
            P.reject_git_env_overrides({"GIT_OBJECT_DIRECTORY": "/tmp/redirect"})
        P.reject_git_env_overrides({})

    def test_backend_resolution_requires_exact_git_commit(self) -> None:
        exact = [{
            "name": "privacypass",
            "version": "0.2.0-pre.3",
            "source": "git+https://github.com/raphaelrobert/privacypass?rev=5ff5f57a62877f42313d6600b53e0d4ee4e4e452#5ff5f57a62877f42313d6600b53e0d4ee4e4e452",
            "checksum": None,
            "dependencies": [],
        }]
        self.assertTrue(P.require_exact_backend_resolution(exact).endswith("#" + P.BACKEND_COMMIT))

        wrong = [dict(exact[0], source="git+https://github.com/raphaelrobert/privacypass#" + "0" * 40)]
        with self.assertRaises(P.PreparationError):
            P.require_exact_backend_resolution(wrong)

    def test_package_records_are_canonicalized(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            lock = pathlib.Path(temp) / "Cargo.lock"
            lock.write_text(
                'version = 4\n\n[[package]]\nname = "z"\nversion = "1.0.0"\nsource = "registry+x"\nchecksum = "aa"\ndependencies = ["b", "a"]\n\n[[package]]\nname = "a"\nversion = "2.0.0"\n',
                encoding="utf-8",
            )
            version, packages = P.package_records(lock)
            self.assertEqual(version, 4)
            self.assertEqual([row["name"] for row in packages], ["a", "z"])
            self.assertEqual(packages[1]["dependencies"], ["a", "b"])

    def test_seed_cargo_home_is_explicit_copy(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            seed = root / "seed"
            destination = root / "isolated"
            seed.mkdir()
            destination.mkdir()
            (seed / "marker").write_text("seeded", encoding="utf-8")
            self.assertTrue(P.seed_isolated_cargo_home(seed, destination))
            self.assertEqual((destination / "marker").read_text(encoding="utf-8"), "seeded")
            self.assertFalse(P.seed_isolated_cargo_home(None, destination))

    def test_self_lock_authority_is_non_promoting(self) -> None:
        lock = P.expected_lock({"README.md": "a", "prepare.py": "b", "test_prepare.py": "c"})
        authority = lock["authority"]
        self.assertEqual(authority["scope"], "DependencyCapsulePreparationOnly")
        self.assertFalse(authority["dependency_graph_frozen"])
        self.assertFalse(authority["backend_executed"])
        self.assertFalse(authority["token_verified"])
        self.assertFalse(authority["query_credit_granted"])
        self.assertFalse(authority["production_admission"])
        self.assertFalse(authority["application_authority"])

    def test_canonical_json_is_stable(self) -> None:
        self.assertEqual(P.canonical({"b": 1, "a": 2}), b'{"a":2,"b":1}\n')


if __name__ == "__main__":
    unittest.main()
