#!/usr/bin/env python3
"""Negative self-tests for verify_capsule.py.

These tests operate on a synthetic, minimally valid capsule assembled from the
verifier's expected evidence format. They exercise the verifier as a separate
consumer and never invoke the lock-capsule generator or validator logic.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VERIFIER = ROOT / "tools" / "verify_capsule.py"

FILES = [
    "Cargo.toml",
    "Cargo.lock",
    "Cargo.lock.sha256",
    "cargo-metadata.canonical.json",
    "cargo-tree.txt",
    "cargo-tree-offline.txt",
    "dependency-source-inventory.v1.json",
    "dependency-source-inventory.v1.sha256",
    "cargo-lock-source-projection.v1.json",
    "cargo-lock-source-projection.v1.sha256",
    "provenance.v1.json",
    "provenance.v1.sha256",
    "source-commit.txt",
    "source-tree.txt",
    "source-status.txt",
    "workflow-sha256.txt",
    "runner-uname.txt",
    "runner-os-release.txt",
    "runner-python-version.txt",
    "rustc-1.96.0.txt",
    "cargo-1.96.0.txt",
]

def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()

def canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode() + b"\n"

def write_sidecar(root: Path, filename: str) -> None:
    (root / filename).write_text(
        f"{sha256_bytes((root / filename.removesuffix('.sha256')).read_bytes())}  "
        f"{filename.removesuffix('.sha256')}\n"
    )

def make_capsule(root: Path) -> None:
    manifest = """[package]
name = "psi-002b3a3k0v-f2s1-validator"
version = "0.1.0"
edition = "2024"
rust-version = "1.96.0"
publish = false
[dependencies]
jsonschema = { version = "=0.58.2", default-features = false }
[workspace]
resolver = "3"
"""
    (root / "Cargo.toml").write_text(manifest)
    (root / "Cargo.lock").write_text(
        """version = 4

[[package]]
name = "psi-002b3a3k0v-f2s1-validator"
version = "0.1.0"
"""
    )
    (root / "Cargo.lock.sha256").write_text(
        f"{sha256_bytes((root / 'Cargo.lock').read_bytes())}  Cargo.lock\n"
    )

    metadata = {
        "packages": [
            {
                "id": "path+file:///workspace#psi-002b3a3k0v-f2s1-validator@0.1.0",
                "name": "psi-002b3a3k0v-f2s1-validator",
                "version": "0.1.0",
                "source": None,
                "checksum": None,
                "edition": "2024",
                "rust_version": "1.96.0",
                "manifest_path": "Cargo.toml",
            }
        ],
        "workspace_root": "/workspace",
        "target_directory": "/target",
    }
    (root / "cargo-metadata.canonical.json").write_bytes(canonical(metadata))
    (root / "cargo-tree.txt").write_text("psi-002b3a3k0v-f2s1-validator v0.1.0\n")
    (root / "cargo-tree-offline.txt").write_text((root / "cargo-tree.txt").read_text())

    package_projection = [
        {
            "name": "psi-002b3a3k0v-f2s1-validator",
            "version": "0.1.0",
            "source": None,
            "checksum": None,
        }
    ]
    (root / "cargo-lock-source-projection.v1.json").write_bytes(canonical(package_projection))
    (root / "cargo-lock-source-projection.v1.sha256").write_text(
        f"{sha256_bytes((root / 'cargo-lock-source-projection.v1.json').read_bytes())}  "
        "cargo-lock-source-projection.v1.json\n"
    )

    inventory = {
        "schema": "psi-002b3a3k0v-f2s1b1-dependency-source-inventory.v1",
        "toolchain": "1.96.0",
        "manifest_sha256": sha256_bytes((root / "Cargo.toml").read_bytes()),
        "lock_sha256": sha256_bytes((root / "Cargo.lock").read_bytes()),
        "packages": [
            {
                "id": "path+file:///workspace#psi-002b3a3k0v-f2s1-validator@0.1.0",
                "name": "psi-002b3a3k0v-f2s1-validator",
                "version": "0.1.0",
                "source": None,
                "checksum": None,
                "edition": "2024",
                "rust_version": "1.96.0",
                "manifest_path": "Cargo.toml",
            }
        ],
    }
    (root / "dependency-source-inventory.v1.json").write_bytes(canonical(inventory))
    (root / "dependency-source-inventory.v1.sha256").write_text(
        f"{sha256_bytes((root / 'dependency-source-inventory.v1.json').read_bytes())}  "
        "dependency-source-inventory.v1.json\n"
    )

    (root / "source-commit.txt").write_text("a" * 40 + "\n")
    (root / "source-tree.txt").write_text("b" * 40 + "\n")
    (root / "workflow-sha256.txt").write_text("c" * 64 + "  workflow.yml\n")
    (root / "source-status.txt").write_text("")
    (root / "rustc-1.96.0.txt").write_text("rustc 1.96.0\n")
    (root / "cargo-1.96.0.txt").write_text("cargo 1.96.0\n")
    (root / "runner-uname.txt").write_text("synthetic runner\n")
    (root / "runner-os-release.txt").write_text("synthetic os\n")
    (root / "runner-python-version.txt").write_text("Python 3\n")

    provenance = {
        "schema": "psi-002b3a3k0v-f2s1b1-provenance.v1",
        "qualification": "PSI-002B3A3K0V-F2S1",
        "package": "psi-002b3a3k0v-f2s1-validator",
        "source": {
            "commit_sha": "a" * 40,
            "tree_sha": "b" * 40,
            "workflow_sha256": "c" * 64,
            "manifest_sha256": sha256_bytes((root / "Cargo.toml").read_bytes()),
            "lock_sha256": sha256_bytes((root / "Cargo.lock").read_bytes()),
        },
        "toolchain": {
            "rust": "1.96.0",
            "rustc_version_sha256": sha256_bytes((root / "rustc-1.96.0.txt").read_bytes()),
            "cargo_version_sha256": sha256_bytes((root / "cargo-1.96.0.txt").read_bytes()),
        },
        "runner": {
            "os_release_sha256": sha256_bytes((root / "runner-os-release.txt").read_bytes()),
            "uname_sha256": sha256_bytes((root / "runner-uname.txt").read_bytes()),
            "python_version_sha256": sha256_bytes((root / "runner-python-version.txt").read_bytes()),
            "platform_machine": "synthetic",
        },
    }
    (root / "provenance.v1.json").write_bytes(canonical(provenance))
    (root / "provenance.v1.sha256").write_text(
        f"{sha256_bytes((root / 'provenance.v1.json').read_bytes())}  provenance.v1.json\n"
    )

    receipt = {
        "schema": "psi-002b3a3k0v-f2s1b1-evidence-receipt.v1",
        "pre_receipt_manifest_sha256": "",
        "artifacts": {
            name: sha256_bytes((root / name).read_bytes())
            for name in [
                "cargo-metadata.canonical.json",
                "cargo-tree.txt",
                "cargo-tree-offline.txt",
                "dependency-source-inventory.v1.json",
                "cargo-lock-source-projection.v1.json",
                "provenance.v1.json",
                "Cargo.toml",
                "Cargo.lock",
            ]
        },
    }
    base = FILES[:]
    pre_lines = [
        f"{sha256_bytes((root / name).read_bytes())}  {name}"
        for name in base
    ]
    (root / "manifest.pre-receipt.sha256").write_text("\n".join(pre_lines) + "\n")
    receipt["pre_receipt_manifest_sha256"] = sha256_bytes(
        (root / "manifest.pre-receipt.sha256").read_bytes()
    )
    (root / "evidence-receipt.v1.json").write_bytes(canonical(receipt))
    (root / "evidence-receipt.v1.sha256").write_text(
        f"{sha256_bytes((root / "evidence-receipt.v1.json").read_bytes())}  evidence-receipt.v1.json\n"
    )

    # The verifier checks the exact manifest after all files exist.
    lines = [
        f"{sha256_bytes((root / name).read_bytes())}  {name}"
        for name in sorted(FILES + ["evidence-receipt.v1.json", "evidence-receipt.v1.sha256", "manifest.pre-receipt.sha256"])
    ]
    (root / "manifest.sha256").write_text("\n".join(lines) + "\n")

def refresh_final_integrity(root: Path) -> None:
    receipt_path = root / "evidence-receipt.v1.json"
    receipt = json.loads(receipt_path.read_text())
    receipt["pre_receipt_manifest_sha256"] = sha256_bytes(
        (root / "manifest.pre-receipt.sha256").read_bytes()
    )
    receipt_path.write_bytes(canonical(receipt))
    write_sidecar(root, "evidence-receipt.v1.sha256")

    lines = [
        f"{sha256_bytes((root / name).read_bytes())}  {name}"
        for name in sorted(FILES + [
            "evidence-receipt.v1.json",
            "evidence-receipt.v1.sha256",
            "manifest.pre-receipt.sha256",
        ])
    ]
    (root / "manifest.sha256").write_text("\n".join(lines) + "\n")

def expect_fail(base: Path, name: str, mutate) -> None:
    case = base.parent / name
    shutil.copytree(base, case, symlinks=True)
    mutate(case)
    result = subprocess.run(
        [sys.executable, str(VERIFIER), str(case)],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        raise AssertionError(f"{name}: verifier unexpectedly accepted tampered capsule")
    if "CAPSULE VERIFY: FAIL:" not in result.stderr:
        raise AssertionError(f"{name}: missing structured failure output: {result.stderr!r}")

def main() -> int:
    with tempfile.TemporaryDirectory(prefix="psi-capsule-negative-") as tmp:
        base = Path(tmp) / "base"
        base.mkdir()
        make_capsule(base)

        baseline = subprocess.run(
            [sys.executable, str(VERIFIER), str(base)],
            capture_output=True,
            text=True,
        )
        if baseline.returncode != 0:
            raise AssertionError(f"baseline capsule rejected: {baseline.stderr}")

        expect_fail(base, "alter-provenance", lambda p: (p / "provenance.v1.json").write_text("{}"))
        expect_fail(base, "alter-lock", lambda p: (p / "Cargo.lock").write_text((p / "Cargo.lock").read_text() + "# tampered\n"))
        expect_fail(base, "alter-manifest", lambda p: (p / "Cargo.toml").write_text((p / "Cargo.toml").read_text().replace('version = "0.1.0"', 'version = "0.1.1"')))
        expect_fail(base, "missing-file", lambda p: (p / "Cargo.lock").unlink())
        expect_fail(base, "extra-file", lambda p: (p / "unexpected.txt").write_text("unexpected\n"))
        expect_fail(base, "symlink", lambda p: ((p / "link").symlink_to(p / "Cargo.toml")))
        expect_fail(base, "duplicate-entry", lambda p: (p / "manifest.sha256").write_text((p / "manifest.sha256").read_text() + (p / "manifest.sha256").read_text().splitlines()[0] + "\n"))
        expect_fail(base, "digest-mismatch", lambda p: (p / "Cargo.lock.sha256").write_text("0" * 64 + "  Cargo.lock\n"))\n\n        def reorder_pre_receipt(p: Path) -> None:
            lines = (p / "manifest.pre-receipt.sha256").read_text().splitlines()
            (p / "manifest.pre-receipt.sha256").write_text("\n".join(reversed(lines)) + "\n")
            refresh_final_integrity(p)
        expect_fail(base, "pre-receipt-reordered", reorder_pre_receipt)

        def duplicate_pre_receipt(p: Path) -> None:
            lines = (p / "manifest.pre-receipt.sha256").read_text().splitlines()
            (p / "manifest.pre-receipt.sha256").write_text("\n".join(lines + [lines[0]]) + "\n")
            refresh_final_integrity(p)
        expect_fail(base, "pre-receipt-duplicate", duplicate_pre_receipt)

        def unexpected_pre_receipt(p: Path) -> None:
            lines = (p / "manifest.pre-receipt.sha256").read_text().splitlines()
            lines.append("0" * 64 + "  unexpected.txt")
            (p / "manifest.pre-receipt.sha256").write_text("\n".join(lines) + "\n")
            refresh_final_integrity(p)
        expect_fail(base, "pre-receipt-unexpected", unexpected_pre_receipt)

        def omitted_pre_receipt(p: Path) -> None:
            lines = (p / "manifest.pre-receipt.sha256").read_text().splitlines()
            (p / "manifest.pre-receipt.sha256").write_text("\n".join(lines[:-1]) + "\n")
            refresh_final_integrity(p)
        expect_fail(base, "pre-receipt-omitted", omitted_pre_receipt)

        print("CAPSULE NEGATIVE TESTS: PASS (tamper rejection only; no qualification claim)")
        return 0

if __name__ == "__main__":
    raise SystemExit(main())
