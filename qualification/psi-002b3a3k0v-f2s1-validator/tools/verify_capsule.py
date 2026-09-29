#!/usr/bin/env python3
"""Dependency-free verifier for the PSI-002B3A3K0V-F2S1 lock capsule.

This verifier consumes only the packaged evidence. It intentionally does not
import the generator, Cargo, the validator crate, or any currentness/crypto
logic. All canonical JSON checks use Python stdlib serialization.
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
import tomllib
from pathlib import Path

BASE_ALLOWLIST = {
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
    "evidence-receipt.v1.json",
    "evidence-receipt.v1.sha256",
    "manifest.pre-receipt.sha256",
}
RECEIPT_FILES = {
    "evidence-receipt.v1.json",
    "evidence-receipt.v1.sha256",
    "manifest.pre-receipt.sha256",
}
ALLOWLIST = BASE_ALLOWLIST | RECEIPT_FILES
HEX64 = re.compile(r"^[0-9a-f]{64}$")
HEX40 = re.compile(r"^[0-9a-f]{40}$")


def fail(message: str) -> None:
    raise SystemExit(f"CAPSULE VERIFY: FAIL: {message}")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid JSON {path.name}: {exc}")


def canonical_json(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode() + b"\n"


def read_sidecar(path: Path, expected_name: str) -> str:
    parts = path.read_text().strip().split()
    if len(parts) != 2 or parts[1] != expected_name or not HEX64.fullmatch(parts[0]):
        fail(f"malformed digest sidecar {path.name}")
    return parts[0]


def verify_manifest(root: Path) -> None:
    entries = list(root.iterdir())
    if any(p.is_symlink() for p in entries):
        fail("capsule contains a symlink")
    if any(not p.is_file() for p in entries):
        fail("capsule contains a non-regular file")
    actual = {p.name for p in entries if p.name != "manifest.sha256"}
    if actual != ALLOWLIST:
        fail(f"exact file-set mismatch: missing={sorted(ALLOWLIST - actual)}, extra={sorted(actual)}")

    seen = []
    for line in (root / "manifest.sha256").read_text().splitlines():
        parts = line.split("  ", 1)
        if len(parts) != 2 or not HEX64.fullmatch(parts[0]):
            fail("malformed manifest entry")
        recorded, name = parts
        if name not in ALLOWLIST:
            fail(f"manifest names non-allowlisted file {name}")
        if name in seen:
            fail(f"duplicate manifest path {name}")
        seen.append(name)
        if digest(root / name) != recorded:
            fail(f"manifest digest mismatch for {name}")
    if seen != sorted(ALLOWLIST):
        fail("manifest ordering or file set mismatch")
    if "manifest.sha256" in seen:
        fail("manifest must not self-reference")


def verify_pre_receipt_manifest(root: Path) -> None:
    path = root / "manifest.pre-receipt.sha256"
    seen = []
    try:
        lines = path.read_text().splitlines()
    except OSError as exc:
        fail(f"cannot read pre-receipt manifest: {exc}")

    for line in lines:
        parts = line.split("  ", 1)
        if len(parts) != 2 or not HEX64.fullmatch(parts[0]):
            fail("malformed pre-receipt manifest entry")
        recorded, name = parts
        if name not in BASE_ALLOWLIST:
            fail(f"pre-receipt manifest names non-base file {name}")
        if name in seen:
            fail(f"duplicate pre-receipt manifest path {name}")
        seen.append(name)
        if digest(root / name) != recorded:
            fail(f"pre-receipt manifest digest mismatch for {name}")

    if seen != sorted(BASE_ALLOWLIST):
        fail("pre-receipt manifest ordering or file set mismatch")
    if "manifest.pre-receipt.sha256" in seen:
        fail("pre-receipt manifest must not self-reference")


def verify_provenance(root: Path) -> None:
    p = read_json(root / "provenance.v1.json")
    if p.get("schema") != "psi-002b3a3k0v-f2s1b1-provenance.v1":
        fail("unexpected provenance schema")
    if p.get("qualification") != "PSI-002B3A3K0V-F2S1":
        fail("unexpected qualification")
    if p.get("package") != "psi-002b3a3k0v-f2s1-validator":
        fail("unexpected package")

    source = p.get("source", {})
    for key in ("commit_sha", "tree_sha"):
        if not HEX40.fullmatch(source.get(key, "")):
            fail(f"invalid provenance {key}")
    for key in ("workflow_sha256", "manifest_sha256", "lock_sha256"):
        if not HEX64.fullmatch(source.get(key, "")):
            fail(f"invalid provenance {key}")

    if source["commit_sha"] != (root / "source-commit.txt").read_text().strip():
        fail("provenance commit does not match source-commit.txt")
    if source["tree_sha"] != (root / "source-tree.txt").read_text().strip():
        fail("provenance tree does not match source-tree.txt")
    if source["workflow_sha256"] != (root / "workflow-sha256.txt").read_text().split()[0]:
        fail("provenance workflow digest mismatch")
    if source["manifest_sha256"] != digest(root / "Cargo.toml"):
        fail("provenance manifest digest mismatch")
    if source["lock_sha256"] != digest(root / "Cargo.lock"):
        fail("provenance lock digest mismatch")

    toolchain = p.get("toolchain", {})
    if toolchain.get("rust") != "1.96.0":
        fail("unexpected Rust toolchain identity")
    if toolchain.get("rustc_version_sha256") != digest(root / "rustc-1.96.0.txt"):
        fail("rustc evidence digest mismatch")
    if toolchain.get("cargo_version_sha256") != digest(root / "cargo-1.96.0.txt"):
        fail("cargo evidence digest mismatch")

    runner = p.get("runner", {})
    for key, filename in (
        ("os_release_sha256", "runner-os-release.txt"),
        ("uname_sha256", "runner-uname.txt"),
        ("python_version_sha256", "runner-python-version.txt"),
    ):
        if runner.get(key) != digest(root / filename):
            fail(f"runner evidence digest mismatch: {key}")

    if read_sidecar(root / "provenance.v1.sha256", "provenance.v1.json") != digest(root / "provenance.v1.json"):
        fail("provenance sidecar digest mismatch")
    if canonical_json(p) != (root / "provenance.v1.json").read_bytes():
        fail("provenance JSON is not in canonical form")


def verify_lock_and_inventory(root: Path) -> None:
    lock = tomllib.loads((root / "Cargo.lock").read_text())
    inv = read_json(root / "dependency-source-inventory.v1.json")
    if inv.get("schema") != "psi-002b3a3k0v-f2s1b1-dependency-source-inventory.v1":
        fail("unexpected dependency inventory schema")
    if inv.get("toolchain") != "1.96.0":
        fail("dependency inventory toolchain mismatch")
    if inv.get("manifest_sha256") != digest(root / "Cargo.toml"):
        fail("dependency inventory manifest digest mismatch")
    if inv.get("lock_sha256") != digest(root / "Cargo.lock"):
        fail("dependency inventory lock digest mismatch")

    def package_tuple(p):
        try:
            return (p["name"], p["version"], p.get("source"), p.get("checksum"))
        except (KeyError, TypeError):
            fail("malformed dependency package record")

    lock_packages = sorted(
        package_tuple(p) for p in lock.get("package", [])
    )
    inv_packages = sorted(
        package_tuple(p) for p in inv.get("packages", [])
    )
    if lock_packages != inv_packages:
        fail("dependency inventory does not match Cargo.lock")

    projection = read_json(root / "cargo-lock-source-projection.v1.json")
    expected_projection = [
        {"name": n, "version": v, "source": s, "checksum": c}
        for n, v, s, c in lock_packages
    ]
    if projection != expected_projection:
        fail("lock source projection does not match Cargo.lock")
    if canonical_json(projection) != (root / "cargo-lock-source-projection.v1.json").read_bytes():
        fail("lock source projection is not canonical")

    if read_sidecar(root / "dependency-source-inventory.v1.sha256", "dependency-source-inventory.v1.json") != digest(root / "dependency-source-inventory.v1.json"):
        fail("dependency inventory sidecar mismatch")
    if read_sidecar(root / "cargo-lock-source-projection.v1.sha256", "cargo-lock-source-projection.v1.json") != digest(root / "cargo-lock-source-projection.v1.json"):
        fail("lock projection sidecar mismatch")
    if read_sidecar(root / "Cargo.lock.sha256", "Cargo.lock") != digest(root / "Cargo.lock"):
        fail("Cargo.lock sidecar mismatch")


def verify_evidence_receipt(root: Path) -> None:
    receipt = read_json(root / "evidence-receipt.v1.json")
    if receipt.get("schema") != "psi-002b3a3k0v-f2s1b1-evidence-receipt.v1":
        fail("unexpected evidence receipt schema")
    expected = {
        "cargo-metadata.canonical.json",
        "cargo-tree.txt",
        "cargo-tree-offline.txt",
        "dependency-source-inventory.v1.json",
        "cargo-lock-source-projection.v1.json",
        "provenance.v1.json",
        "Cargo.toml",
        "Cargo.lock",
    }
    files = receipt.get("artifacts")
    if not isinstance(files, dict) or set(files) != expected:
        fail("evidence receipt artifact set mismatch")
    for name in expected:
        if files[name] != digest(root / name):
            fail(f"evidence receipt digest mismatch: {name}")
    pre_manifest_digest = digest(root / "manifest.pre-receipt.sha256")
    if receipt.get("pre_receipt_manifest_sha256") != pre_manifest_digest:
        fail("evidence receipt pre-receipt manifest binding mismatch")
    if read_sidecar(root / "evidence-receipt.v1.sha256", "evidence-receipt.v1.json") != digest(root / "evidence-receipt.v1.json"):
        fail("evidence receipt sidecar mismatch")
    if canonical_json(receipt) != (root / "evidence-receipt.v1.json").read_bytes():
        fail("evidence receipt is not canonical")


def verify_manifest_metadata(root: Path) -> None:
    manifest = tomllib.loads((root / "Cargo.toml").read_text())
    package = manifest.get("package", {})
    if package.get("name") != "psi-002b3a3k0v-f2s1-validator":
        fail("unexpected package name")
    if package.get("rust-version") != "1.96.0":
        fail("unexpected package rust-version")
    dep = manifest.get("dependencies", {}).get("jsonschema")
    if not isinstance(dep, dict) or dep.get("version") != "=0.58.2" or dep.get("default-features") is not False:
        fail("jsonschema dependency boundary mismatch")
    if manifest.get("workspace", {}).get("resolver") != "3":
        fail("workspace resolver mismatch")


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} CAPSULE_DIR", file=sys.stderr)
        return 2
    root = Path(sys.argv[1])
    if not root.is_dir():
        fail(f"not a directory: {root}")
    verify_manifest(root)
    verify_manifest_metadata(root)
    verify_pre_receipt_manifest(root)
    verify_provenance(root)
    verify_lock_and_inventory(root)
    verify_evidence_receipt(root)
    print("CAPSULE VERIFY: PASS (evidence integrity only; no schema/currentness/crypto qualification)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
