#!/usr/bin/env python3
"""Verify the frozen JIT-1A payload manifest without relying on git."""

from __future__ import annotations

import hashlib
from pathlib import Path
import tomllib

HERE = Path(__file__).resolve().parent
LOCK = tomllib.loads((HERE / "JIT1A.lock").read_text())
DOMAIN = b"MYCELIX-JIT-QUAL/V0.1/JIT1A-PAYLOAD"

EXPECTED_PARENT = "108a4505904d49f2f93924776e31e9def1c880ab"
EXPECTED_JIT0_ROOT = "78a8d7d525252b31ff24245514ab9288caec6bea56fba02349e4415a0a55546c"
EXPECTED_PROFILE = "9c9bea898f07a068e8d4c4be7d39adc81bf11a5b4ea9b9fc19d44b90c42a6a9b"


def git_blob_oid(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def fail(message: str) -> None:
    raise SystemExit(message)


if LOCK.get("protocol") != "MYCELIX-JIT-QUAL/JIT1A":
    fail("unexpected JIT-1A protocol")
if LOCK.get("version") != 1:
    fail("unexpected JIT-1A lock version")
if LOCK.get("status") != "exact-execution-pending":
    fail("unexpected JIT-1A status")
if LOCK.get("parent_jit0_commit") != EXPECTED_PARENT:
    fail("JIT-0 parent mismatch")
if LOCK.get("jit0_root_sha256") != EXPECTED_JIT0_ROOT:
    fail("JIT-0 root mismatch")
if LOCK.get("qualification_profile_sha256") != EXPECTED_PROFILE:
    fail("qualification profile mismatch")

files = LOCK.get("files")
if not isinstance(files, dict) or not files:
    fail("missing JIT-1A payload files")

actual: list[tuple[str, str]] = []
for rel, expected_oid in sorted(files.items()):
    if not isinstance(rel, str) or not isinstance(expected_oid, str):
        fail("malformed file manifest")
    path = HERE / rel
    if not path.is_file():
        fail(f"missing payload file: {rel}")
    oid = git_blob_oid(path.read_bytes())
    if oid != expected_oid:
        fail(f"Git blob mismatch: {rel}")
    actual.append((rel, oid))

h = hashlib.sha256()
h.update(DOMAIN)
for rel, oid in actual:
    name = rel.encode("utf-8")
    h.update(len(name).to_bytes(4, "big"))
    h.update(name)
    h.update(bytes.fromhex(oid))
manifest = h.hexdigest()
if manifest != LOCK.get("payload_manifest_sha256"):
    fail("payload manifest SHA-256 mismatch")

print(f"jit1a_payload_manifest_sha256={manifest}")
print(f"jit1a_payload_file_count={len(actual)}")
