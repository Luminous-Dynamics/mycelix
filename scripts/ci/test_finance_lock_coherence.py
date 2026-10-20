#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("finance_lock_coherence.py")
spec = importlib.util.spec_from_file_location("finance_lock_coherence", SCRIPT)
assert spec is not None and spec.loader is not None
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


class Fixture:
    def __init__(self, workspace: str, app: str, lock: str):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root / "Cargo.toml").write_text(workspace, encoding="utf-8")
        (self.root / "app").mkdir()
        (self.root / "app" / "Cargo.toml").write_text(app, encoding="utf-8")
        (self.root / "Cargo.lock").write_text(lock, encoding="utf-8")

    def verify(self):
        return mod.verify(self.root / "Cargo.toml", self.root / "Cargo.lock")

    def close(self):
        self.tmp.cleanup()


WORKSPACE = """\
[workspace]
members = ["app"]
resolver = "2"
"""

APP_BASE = """\
[package]
name = "app"
version = "0.1.0"
edition = "2024"
"""


def lock(*packages: str) -> str:
    return "version = 4\n\n" + "\n\n".join(packages) + "\n"


def package(name: str, version: str = "0.1.0", *, source: str | None = None,
            checksum: str | None = None, deps: list[str] | None = None) -> str:
    lines = ["[[package]]", f'name = "{name}"', f'version = "{version}"']
    if source is not None:
        lines.append(f'source = "{source}"')
    if checksum is not None:
        lines.append(f'checksum = "{checksum}"')
    if deps is not None:
        lines.append("dependencies = [")
        lines.extend(f' "{d}",' for d in deps)
        lines.append("]")
    return "\n".join(lines)


REGISTRY = "registry+https://github.com/rust-lang/crates.io-index"
GIT = "git+https://example.invalid/repo?rev=abc#abcdef"
CHECKSUM = "0" * 64


class FinanceLockCoherenceTests(unittest.TestCase):
    def check(self, workspace: str, app: str, lock_text: str):
        fixture = Fixture(workspace, app, lock_text)
        try:
            return fixture.verify()
        finally:
            fixture.close()

    def assert_problem(self, receipt, kind: str):
        self.assertEqual(receipt["status"], "FAIL", receipt)
        self.assertIn(kind, {p["kind"] for p in receipt["problems"]}, receipt)

    def test_clean_registry_dependency_passes(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\nserde = "1"\n',
            lock(
                package("app", deps=["serde"]),
                package("serde", "1.0.0", source=REGISTRY, checksum=CHECKSUM),
            ),
        )
        self.assertEqual(receipt["status"], "PASS", receipt)

    def test_missing_workspace_package_fails(self):
        receipt = self.check(WORKSPACE, APP_BASE, lock(package("other")))
        self.assert_problem(receipt, "MissingWorkspacePackage")

    def test_missing_direct_edge_fails(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\ndep = { path = "../dep" }\n',
            lock(package("app", deps=[]), package("dep")),
        )
        self.assert_problem(receipt, "MissingDirectDependencyEdge")

    def test_path_dependency_cannot_resolve_to_registry_package(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\ndep = { path = "../dep" }\n',
            lock(
                package("app", deps=[f"dep 1.0.0 ({REGISTRY})"]),
                package("dep", "1.0.0", source=REGISTRY, checksum=CHECKSUM),
            ),
        )
        self.assert_problem(receipt, "PathSourceIdentityMismatch")

    def test_registry_dependency_cannot_resolve_to_local_package(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\ndep = "1"\n',
            lock(package("app", deps=["dep"]), package("dep", "1.0.0")),
        )
        self.assert_problem(receipt, "RegistrySourceIdentityMismatch")

    def test_crates_io_dependency_requires_checksum(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\nserde = "1"\n',
            lock(
                package("app", deps=["serde"]),
                package("serde", "1.0.0", source=REGISTRY),
            ),
        )
        self.assert_problem(receipt, "MissingRegistryChecksum")

    def test_renamed_registry_dependency_passes(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\nalias = { package = "real", version = "1" }\n',
            lock(
                package("app", deps=["real"]),
                package("real", "1.0.0", source=REGISTRY, checksum=CHECKSUM),
            ),
        )
        self.assertEqual(receipt["status"], "PASS", receipt)

    def test_target_specific_dependency_is_checked(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + "\n[target.'cfg(unix)'.dependencies]\ndep = { path = \"../dep\" }\n",
            lock(package("app", deps=["dep"]), package("dep")),
        )
        self.assertEqual(receipt["status"], "PASS", receipt)
        self.assertEqual(receipt["manifest_direct_edges_checked"], 1)

    def test_dev_dependency_is_checked(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dev-dependencies]\ndep = { path = "../dep" }\n',
            lock(package("app", deps=["dep"]), package("dep")),
        )
        self.assertEqual(receipt["status"], "PASS", receipt)
        self.assertEqual(receipt["manifest_direct_edges_checked"], 1)

    def test_workspace_inherited_path_dependency_passes(self):
        workspace = WORKSPACE + '\n[workspace.dependencies]\ndep = { path = "dep" }\n'
        app = APP_BASE + '\n[dependencies]\ndep = { workspace = true }\n'
        receipt = self.check(
            workspace,
            app,
            lock(package("app", deps=["dep"]), package("dep")),
        )
        self.assertEqual(receipt["status"], "PASS", receipt)

    def test_name_only_edge_with_multiple_versions_is_ambiguous(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\nserde = "1"\n',
            lock(
                package("app", deps=["serde"]),
                package("serde", "1.0.0", source=REGISTRY, checksum=CHECKSUM),
                package("serde", "2.0.0", source=REGISTRY, checksum="1" * 64),
            ),
        )
        self.assert_problem(receipt, "AmbiguousDirectDependencyEdge")

    def test_git_source_mismatch_fails(self):
        receipt = self.check(
            WORKSPACE,
            APP_BASE + '\n[dependencies]\ndep = { git = "https://example.invalid/wanted" }\n',
            lock(
                package("app", deps=[f"dep 1.0.0 ({GIT})"]),
                package("dep", "1.0.0", source=GIT),
            ),
        )
        self.assert_problem(receipt, "GitSourceIdentityMismatch")


if __name__ == "__main__":
    unittest.main(verbosity=2)
