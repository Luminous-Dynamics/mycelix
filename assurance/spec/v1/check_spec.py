#!/usr/bin/env python3
"""Verify the frozen ASSURE-V1 specification package using only stdlib."""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import tomllib

HERE = Path(__file__).resolve().parent
LOCK_PATH = HERE / "SPEC.lock"


def fail(message: str) -> None:
    print(f"ASSURE-V1 SPEC CHECK FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def sha256_bytes(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()


def load_toml(path: Path) -> dict:
    with path.open("rb") as handle:
        return tomllib.load(handle)


def require_unique(values: list[object], label: str) -> None:
    if len(values) != len(set(values)):
        fail(f"duplicate {label}")


def check_registry(path: Path, table: str, *, code_field: str | None = None) -> None:
    doc = load_toml(path)
    if doc.get("version") != 1:
        fail(f"{path.name}: expected version = 1")
    rows = doc.get(table)
    if not isinstance(rows, list) or not rows:
        fail(f"{path.name}: missing [[{table}]] entries")

    ids = [row.get("id") for row in rows]
    require_unique(ids, f"IDs in {path.name}")
    if any(not isinstance(value, int) or value <= 0 for value in ids):
        fail(f"{path.name}: IDs must be positive integers")

    if code_field is None:
        names = [row.get("name") for row in rows]
        require_unique(names, f"names in {path.name}")
        if any(not isinstance(value, str) or not value for value in names):
            fail(f"{path.name}: names must be non-empty strings")
    else:
        codes = [row.get(code_field) for row in rows]
        require_unique(codes, f"{code_field} values in {path.name}")
        if any(not isinstance(value, str) or not value for value in codes):
            fail(f"{path.name}: {code_field} values must be non-empty strings")
        allowed_stages = {"decode", "structure", "admission"}
        stages = [row.get("stage") for row in rows]
        if any(stage not in allowed_stages for stage in stages):
            fail(f"{path.name}: unknown failure stage")


def main() -> None:
    lock = load_toml(LOCK_PATH)
    if lock.get("protocol") != "MYCELIX-ASSURE":
        fail("unexpected protocol name")
    if lock.get("major") != 1 or lock.get("minor") != 0:
        fail("unexpected protocol version")
    if lock.get("hash") != "sha256":
        fail("ASSURE-V1 requires SHA-256")

    order = lock.get("order")
    expected_files = lock.get("files")
    if not isinstance(order, list) or not order:
        fail("SPEC.lock has no file order")
    if not isinstance(expected_files, dict):
        fail("SPEC.lock has no [files] table")
    require_unique(order, "file names in SPEC.lock order")
    if set(order) != set(expected_files):
        fail("SPEC.lock order and [files] table disagree")

    root = hashlib.sha256()
    domain = lock.get("spec_domain")
    if not isinstance(domain, str) or not domain:
        fail("SPEC.lock has no spec_domain")
    root.update(domain.encode("utf-8"))

    for name in order:
        path = HERE / name
        if not path.is_file():
            fail(f"missing normative file: {name}")
        digest = sha256_bytes(path.read_bytes())
        expected = expected_files.get(name)
        if digest.hex() != expected:
            fail(f"hash mismatch for {name}: got {digest.hex()}, expected {expected}")
        root.update(name.encode("utf-8"))
        root.update(b"\x00")
        root.update(digest)

    expected_root = lock.get("spec_root_sha256")
    if root.hexdigest() != expected_root:
        fail(f"spec root mismatch: got {root.hexdigest()}, expected {expected_root}")

    check_registry(HERE / "node-tags.toml", "tag")
    check_registry(HERE / "edge-tags.toml", "tag")
    check_registry(HERE / "failure-codes.toml", "failure", code_field="code")

    domains = load_toml(HERE / "domain-separators.toml")
    if domains.get("version") != 1:
        fail("domain-separators.toml: expected version = 1")
    separators = domains.get("separator")
    if not isinstance(separators, dict) or not separators:
        fail("domain-separators.toml: missing [separator] table")
    values = list(separators.values())
    require_unique(values, "domain separator values")
    if any(not isinstance(value, str) or not value.startswith("MYCELIX-ASSURE/V1/") for value in values):
        fail("all domain separators must use the MYCELIX-ASSURE/V1/ prefix")
    if separators.get("spec") != domain:
        fail("SPEC.lock spec_domain does not match domain-separators.toml")

    print(f"ASSURE-V1 specification OK: sha256:{expected_root}")


if __name__ == "__main__":
    main()
