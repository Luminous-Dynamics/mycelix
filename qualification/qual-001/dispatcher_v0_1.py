#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

POINTER_SCHEMA = "mycelix.qual.current-verifier.v0.1"

class DispatchError(RuntimeError):
    pass

def require(condition: bool, message: str) -> None:
    if not condition:
        raise DispatchError(message)

def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()

def validate_rel(value: str) -> None:
    p = Path(value)
    require(value != "", "empty path")
    require(not p.is_absolute(), f"absolute path rejected: {value}")
    require(".." not in p.parts, f"path traversal rejected: {value}")

def safe_file(root: Path, rel: str) -> Path:
    validate_rel(rel)
    p = root / rel
    require(p.exists() and not p.is_symlink() and p.is_file(), f"invalid file: {rel}")
    resolved = p.resolve()
    rr = root.resolve()
    require(rr == resolved or rr in resolved.parents, f"path escaped root: {rel}")
    return p

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verifier-root", type=Path, required=True)
    ap.add_argument("--subject-root", type=Path, required=True)
    ap.add_argument("--current-pointer", type=Path, required=True)
    ap.add_argument("--expected-verifier-head", required=True)
    ap.add_argument("--subject-base", required=True)
    ap.add_argument("--subject-head", required=True)
    ap.add_argument("--receipt-out", type=Path, required=True)
    args = ap.parse_args()

    expected_pointer = safe_file(args.verifier_root, "qualification/qual-001/current-verifier.json")
    require(args.current_pointer.resolve() == expected_pointer.resolve(), "unexpected current-pointer path")
    pointer_raw = expected_pointer.read_bytes()
    pointer = json.loads(pointer_raw)
    require(pointer.get("schema") == POINTER_SCHEMA, "wrong current-verifier pointer schema")
    require(set(pointer) == {"schema","profile","bundle_path","bundle_sha256"}, "unexpected pointer fields")
    bundle_path = pointer["bundle_path"]
    bundle_sha = pointer["bundle_sha256"]
    profile = pointer["profile"]
    require(isinstance(profile, str) and profile, "invalid pointer profile")
    require(isinstance(bundle_sha, str) and len(bundle_sha) == 64, "invalid pointer bundle digest")

    bundle_file = safe_file(args.verifier_root, bundle_path)
    require(sha256_bytes(bundle_file.read_bytes()) == bundle_sha, "current bundle digest mismatch")
    bundle = json.loads(bundle_file.read_text())
    require(bundle.get("profile") == profile, "pointer/bundle profile mismatch")
    components = bundle.get("components")
    require(isinstance(components, dict) and "verifier" in components and "gate_manifest" in components,
            "bundle missing dispatch components")

    verifier_rel = components["verifier"]["path"]
    gate_rel = components["gate_manifest"]["path"]
    verifier_file = safe_file(args.verifier_root, verifier_rel)
    gate_file = safe_file(args.verifier_root, gate_rel)

    cmd = [
        sys.executable, "-B", str(verifier_file),
        "--verifier-root", str(args.verifier_root),
        "--subject-root", str(args.subject_root),
        "--manifest", str(gate_file),
        "--bundle-manifest", str(bundle_file),
        "--current-pointer", str(args.current_pointer),
        "--expected-verifier-head", args.expected_verifier_head,
        "--subject-base", args.subject_base,
        "--subject-head", args.subject_head,
        "--receipt-out", str(args.receipt_out),
    ]
    subprocess.run(cmd, check=True)

if __name__ == "__main__":
    main()
