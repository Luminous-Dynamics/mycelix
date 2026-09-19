#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
PRODUCER_PATH = HERE / "forge_changed_paths.py"

spec = importlib.util.spec_from_file_location("forge_changed_paths", PRODUCER_PATH)
producer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(producer)


class ForgeChangeSetTests(unittest.TestCase):
    def git(self, root: Path, *args: str, input_bytes: bytes | None = None) -> bytes:
        env = os.environ.copy()
        env["GIT_CONFIG_NOSYSTEM"] = "1"
        completed = subprocess.run(
            ["git", "-C", str(root), *args],
            input=input_bytes,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            check=True,
        )
        return completed.stdout

    def repo(self):
        temp = tempfile.TemporaryDirectory()
        root = Path(temp.name)
        self.git(root, "init", "-q")
        self.git(root, "config", "user.email", "test@example.invalid")
        self.git(root, "config", "user.name", "Forge Test")
        return temp, root

    def commit_all(self, root: Path, message: str) -> str:
        self.git(root, "add", "-A")
        self.git(root, "commit", "-q", "-m", message)
        return self.git(root, "rev-parse", "HEAD").decode().strip()

    def base_repo(self):
        temp, root = self.repo()
        p = root / "adapters" / "forge-hermetic-host" / "src"
        p.mkdir(parents=True)
        (p / "lib.rs").write_text("pub fn v1() {}\n", encoding="utf-8")
        base = self.commit_all(root, "base")
        return temp, root, base

    def test_modified_file(self):
        temp, root, base = self.base_repo()
        try:
            target = root / "adapters/forge-hermetic-host/src/lib.rs"
            target.write_text("pub fn v2() {}\n", encoding="utf-8")
            head = self.commit_all(root, "modify")
            result = producer.derive_change_set(root, base, head)
            self.assertEqual(result["changed_paths"], ["adapters/forge-hermetic-host/src/lib.rs"])
            self.assertEqual(result["changed_path_count"], 1)
        finally:
            temp.cleanup()

    def test_added_file(self):
        temp, root, base = self.base_repo()
        try:
            target = root / "adapters/forge-hermetic-host/src/new.rs"
            target.write_text("pub fn added() {}\n", encoding="utf-8")
            head = self.commit_all(root, "add")
            result = producer.derive_change_set(root, base, head)
            self.assertEqual(result["changed_paths"], ["adapters/forge-hermetic-host/src/new.rs"])
        finally:
            temp.cleanup()

    def test_deleted_file(self):
        temp, root, base = self.base_repo()
        try:
            (root / "adapters/forge-hermetic-host/src/lib.rs").unlink()
            head = self.commit_all(root, "delete")
            result = producer.derive_change_set(root, base, head)
            self.assertEqual(result["changed_paths"], ["adapters/forge-hermetic-host/src/lib.rs"])
        finally:
            temp.cleanup()

    def test_rename_is_old_plus_new(self):
        temp, root, base = self.base_repo()
        try:
            old = root / "adapters/forge-hermetic-host/src/lib.rs"
            new = root / "adapters/forge-hermetic-host/src/renamed.rs"
            old.rename(new)
            head = self.commit_all(root, "rename")
            result = producer.derive_change_set(root, base, head)
            self.assertEqual(
                result["changed_paths"],
                [
                    "adapters/forge-hermetic-host/src/lib.rs",
                    "adapters/forge-hermetic-host/src/renamed.rs",
                ],
            )
        finally:
            temp.cleanup()

    def test_paths_are_sorted_independent_of_creation_order(self):
        temp, root, base = self.base_repo()
        try:
            src = root / "adapters/forge-hermetic-host/src"
            (src / "z.rs").write_text("z\n", encoding="utf-8")
            (src / "a.rs").write_text("a\n", encoding="utf-8")
            head = self.commit_all(root, "two files")
            result = producer.derive_change_set(root, base, head)
            self.assertEqual(
                result["changed_paths"],
                [
                    "adapters/forge-hermetic-host/src/a.rs",
                    "adapters/forge-hermetic-host/src/z.rs",
                ],
            )
        finally:
            temp.cleanup()

    def test_newline_filename_preserved_by_nul_parser(self):
        temp, root, base = self.base_repo()
        try:
            name = "line\nbreak.rs"
            target = root / "adapters/forge-hermetic-host/src" / name
            target.write_text("x\n", encoding="utf-8")
            head = self.commit_all(root, "newline")
            result = producer.derive_change_set(root, base, head)
            self.assertEqual(
                result["changed_paths"],
                ["adapters/forge-hermetic-host/src/line\nbreak.rs"],
            )
        finally:
            temp.cleanup()

    def test_equal_base_head_is_valid_empty_set(self):
        temp, root, base = self.base_repo()
        try:
            result = producer.derive_change_set(root, base, base)
            self.assertEqual(result["changed_paths"], [])
            self.assertEqual(result["changed_path_count"], 0)
        finally:
            temp.cleanup()

    def test_invalid_oid_refused(self):
        temp, root, base = self.base_repo()
        try:
            with self.assertRaises(producer.ChangeSetError):
                producer.derive_change_set(root, "nope", base)
        finally:
            temp.cleanup()

    def test_nonexistent_oid_refused(self):
        temp, root, base = self.base_repo()
        try:
            missing = "0" * 40
            with self.assertRaises(producer.ChangeSetError):
                producer.derive_change_set(root, missing, base)
        finally:
            temp.cleanup()

    def test_non_commit_oid_refused(self):
        temp, root, base = self.base_repo()
        try:
            blob = self.git(root, "hash-object", "adapters/forge-hermetic-host/src/lib.rs").decode().strip()
            with self.assertRaises(producer.ChangeSetError):
                producer.derive_change_set(root, blob, base)
        finally:
            temp.cleanup()

    def test_non_ancestor_refused(self):
        temp, root, base = self.base_repo()
        try:
            self.git(root, "checkout", "-q", "--orphan", "other")
            self.git(root, "rm", "-q", "-rf", ".")
            (root / "other.txt").write_text("other\n", encoding="utf-8")
            other = self.commit_all(root, "other root")
            with self.assertRaises(producer.ChangeSetError):
                producer.derive_change_set(root, base, other)
        finally:
            temp.cleanup()

    def test_evidence_commitment_is_deterministic(self):
        temp, root, base = self.base_repo()
        try:
            target = root / "adapters/forge-hermetic-host/src/lib.rs"
            target.write_text("pub fn v2() {}\n", encoding="utf-8")
            head = self.commit_all(root, "modify")
            a = producer.derive_change_set(root, base, head)
            b = producer.derive_change_set(root, base, head)
            self.assertEqual(a, b)
            self.assertRegex(a["evidence_commitment"], r"^[0-9a-f]{64}$")
        finally:
            temp.cleanup()

    def test_commitment_changes_with_subject(self):
        temp, root, base = self.base_repo()
        try:
            target = root / "adapters/forge-hermetic-host/src/lib.rs"
            target.write_text("v2\n", encoding="utf-8")
            head1 = self.commit_all(root, "one")
            target.write_text("v3\n", encoding="utf-8")
            head2 = self.commit_all(root, "two")
            a = producer.derive_change_set(root, base, head1)
            b = producer.derive_change_set(root, base, head2)
            self.assertNotEqual(a["evidence_commitment"], b["evidence_commitment"])
        finally:
            temp.cleanup()

    def test_repository_remains_clean_after_derivation(self):
        temp, root, base = self.base_repo()
        try:
            target = root / "adapters/forge-hermetic-host/src/new.rs"
            target.write_text("new\n", encoding="utf-8")
            head = self.commit_all(root, "new")
            before = self.git(root, "status", "--porcelain=v1", "-z")
            producer.derive_change_set(root, base, head)
            after = self.git(root, "status", "--porcelain=v1", "-z")
            self.assertEqual(before, b"")
            self.assertEqual(after, before)
        finally:
            temp.cleanup()

    def test_diff_profile_is_frozen(self):
        self.assertEqual(producer.DIFF_PROFILE, "git-tree-diff-no-renames-v1")


if __name__ == "__main__":
    unittest.main()
