#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import stat
import subprocess
from pathlib import Path

MANIFEST_SCHEMA = "mycelix.qual.gate-manifest.v0.1"
BUNDLE_SCHEMA = "mycelix.qual.verifier-bundle.v0.1"
RECEIPT_SCHEMA = "mycelix.qual.independent-verifier.receipt.v0.1"


class VerificationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise VerificationError(message)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_json(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def git(root: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(root), *args],
        text=True,
        stderr=subprocess.STDOUT,
    ).strip()


def load_gate_manifest(path: Path) -> tuple[dict, str]:
    raw = path.read_bytes()
    data = json.loads(raw)
    require(data.get("schema") == MANIFEST_SCHEMA, "wrong gate-manifest schema")
    require(data.get("receipt_schema") == RECEIPT_SCHEMA, "wrong receipt schema")
    require(data.get("subject_execution") is False, "v0.1 static profile must not execute subject code")
    require(data.get("same_repository_subject_required") is True, "v0.1 requires same-repository subject")
    require(isinstance(data.get("max_changed_paths"), int) and data["max_changed_paths"] > 0, "invalid path bound")
    require(isinstance(data.get("max_changed_bytes"), int) and data["max_changed_bytes"] > 0, "invalid byte bound")
    return data, sha256_bytes(raw)


def validate_repo_relative_path(value: str) -> None:
    path = Path(value)
    require(value != "", "empty changed path")
    require(not path.is_absolute(), f"absolute changed path rejected: {value}")
    require(".." not in path.parts, f"path traversal rejected: {value}")


def load_and_verify_bundle(
    verifier_root: Path,
    bundle_manifest_path: Path,
    expected_profile: str,
) -> tuple[dict, str, dict[str, str]]:
    raw = bundle_manifest_path.read_bytes()
    bundle = json.loads(raw)
    require(bundle.get("schema") == BUNDLE_SCHEMA, "wrong verifier-bundle schema")
    require(bundle.get("profile") == expected_profile, "verifier-bundle profile mismatch")

    files = bundle.get("files")
    require(isinstance(files, list) and files, "verifier-bundle file census missing")
    seen: set[str] = set()
    actual: dict[str, str] = {}

    for entry in files:
        require(isinstance(entry, dict), "invalid verifier-bundle entry")
        path_value = entry.get("path")
        expected_sha = entry.get("sha256")
        require(isinstance(path_value, str) and path_value, "invalid verifier-bundle path")
        require(isinstance(expected_sha, str) and len(expected_sha) == 64, "invalid verifier-bundle digest")
        validate_repo_relative_path(path_value)
        require(path_value not in seen, f"duplicate verifier-bundle path: {path_value}")
        seen.add(path_value)

        path = verifier_root / path_value
        require(path.is_file() and not path.is_symlink(), f"verifier-bundle file missing or non-regular: {path_value}")
        digest = sha256_bytes(path.read_bytes())
        require(digest == expected_sha, f"verifier-bundle byte drift: {path_value}")
        actual[path_value] = digest

    required_paths = {
        ".github/workflows/qual-001-authoritative.yml",
        "qualification/qual-001/gate-manifest-v0.1.json",
        "qualification/qual-001/verifier_v0_1.py",
    }
    require(seen == required_paths, f"unexpected verifier-bundle census: {sorted(seen)}")
    return bundle, sha256_bytes(raw), actual


def validate_changed_paths(changed: list[str], manifest: dict) -> None:
    require(len(changed) <= manifest["max_changed_paths"], "changed-path bound exceeded")
    exact = set(manifest.get("forbidden_subject_exact_paths", []))
    prefixes = tuple(manifest.get("forbidden_subject_prefixes", []))
    for item in changed:
        validate_repo_relative_path(item)
        require(item not in exact, f"subject attempted to change verifier-owned path: {item}")
        require(not item.startswith(prefixes), f"subject attempted to shadow verifier-owned prefix: {item}")


def changed_paths(subject_root: Path, base: str, head: str) -> list[str]:
    output = git(subject_root, "diff", "--name-only", "--no-renames", base, head)
    return [line for line in output.splitlines() if line]


def changed_file_bytes(subject_root: Path, changed: list[str], max_bytes: int) -> int:
    total = 0
    root = subject_root.resolve()
    for item in changed:
        validate_repo_relative_path(item)
        path = subject_root / item
        if not path.exists() and not path.is_symlink():
            continue
        require(not path.is_symlink(), f"subject symlink rejected: {item}")
        resolved = path.resolve()
        require(root == resolved or root in resolved.parents, f"subject path escaped checkout: {item}")
        mode = path.stat().st_mode
        require(stat.S_ISREG(mode), f"non-regular subject path rejected: {item}")
        total += path.stat().st_size
        require(total <= max_bytes, "changed-byte bound exceeded")
    return total


def verify(
    verifier_root: Path,
    subject_root: Path,
    manifest_path: Path,
    bundle_manifest_path: Path,
    expected_verifier_head: str,
    subject_base: str,
    subject_head: str,
) -> dict:
    manifest, manifest_sha = load_gate_manifest(manifest_path)
    _, bundle_sha, bundle_files = load_and_verify_bundle(
        verifier_root,
        bundle_manifest_path,
        manifest["profile"],
    )

    require(
        bundle_files["qualification/qual-001/gate-manifest-v0.1.json"] == manifest_sha,
        "gate-manifest digest disagrees with verifier bundle",
    )

    actual_verifier_head = git(verifier_root, "rev-parse", "HEAD")
    actual_subject_head = git(subject_root, "rev-parse", "HEAD")
    require(actual_verifier_head == expected_verifier_head, "verifier checkout head mismatch")
    require(actual_subject_head == subject_head, "subject checkout head mismatch")

    git(subject_root, "cat-file", "-e", f"{subject_base}^{{commit}}")

    changed = changed_paths(subject_root, subject_base, subject_head)
    validate_changed_paths(changed, manifest)
    total_bytes = changed_file_bytes(subject_root, changed, manifest["max_changed_bytes"])
    changed_digest = sha256_bytes(("\n".join(changed) + ("\n" if changed else "")).encode())

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "qualification_pass": True,
        "subject_head": subject_head,
        "subject_tree": git(subject_root, "rev-parse", f"{subject_head}^{{tree}}"),
        "subject_base": subject_base,
        "verifier_head": actual_verifier_head,
        "verifier_tree": git(verifier_root, "rev-parse", "HEAD^{tree}"),
        "verifier_bundle_schema": BUNDLE_SCHEMA,
        "verifier_bundle_sha256": bundle_sha,
        "authoritative_workflow_sha256": bundle_files[".github/workflows/qual-001-authoritative.yml"],
        "verifier_source_sha256": bundle_files["qualification/qual-001/verifier_v0_1.py"],
        "gate_manifest_profile": manifest["profile"],
        "gate_manifest_sha256": manifest_sha,
        "changed_paths_sha256": changed_digest,
        "changed_path_count": len(changed),
        "changed_bytes": total_bytes,
        "subject_code_executed": False,
        "candidate_local_verifier_authoritative": False,
        "os_network_sandbox_claimed": False,
    }

    required = set(manifest.get("required_receipt_fields", []))
    missing = sorted(required - receipt.keys())
    require(not missing, f"receipt missing required fields: {missing}")
    return receipt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verifier-root", type=Path, required=True)
    parser.add_argument("--subject-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--bundle-manifest", type=Path, required=True)
    parser.add_argument("--expected-verifier-head", required=True)
    parser.add_argument("--subject-base", required=True)
    parser.add_argument("--subject-head", required=True)
    parser.add_argument("--receipt-out", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    receipt = verify(
        args.verifier_root,
        args.subject_root,
        args.manifest,
        args.bundle_manifest,
        args.expected_verifier_head,
        args.subject_base,
        args.subject_head,
    )
    args.receipt_out.write_text(canonical_json(receipt))
    print("QUAL-001 static subject verification gates: PASS")


if __name__ == "__main__":
    main()
