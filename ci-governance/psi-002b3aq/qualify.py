#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import tempfile
from typing import Any

PRODUCT_COMMIT = "1a5fdee47750c2f90e76e4e9dd4374603b56cfd7"
PRODUCT_TREE = "e9dcaca88577dfd53781c343c07f107ecb259467"
PRODUCT_PARENT = "c5c892ab096c5454f8e0bcad87940e0e7ac1a849"
PRODUCT_PARENT_TREE = "64a9c02021611de3e8352045de98f3202e28d644"
PREFIX = "ci-governance/psi-002b3aq"
PATHS = (
    f"{PREFIX}/README.md",
    f"{PREFIX}/lock.json",
    f"{PREFIX}/qualify.py",
    f"{PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/Cargo.toml": "ba44563a663b3e44a41477fdf6fa50dde7dde1dd",
    "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/README.md": "907d300c5e1692db32281d5a296589dbe8c1d8d6",
    "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/src/lib.rs": "107d5b4063270a209b87db4fb6f3e59c8e3d02d3",
}
EXPECTED_TEST_COUNT = 10
LOCK_SCHEMA = "mycelix.psi.002b3aq.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b3aq.receipt.v0.1"
FORBIDDEN_AUTHORITY_FIELDS = ("pub unspent:", "pub consumed:", "pub spent:")
COMMANDS = (
    ("cargo", "fmt", "--check", "--all"),
    ("cargo", "test", "--offline", "--all-targets"),
    ("cargo", "clippy", "--offline", "--all-targets", "--all-features", "--", "-D", "warnings"),
)
FORBIDDEN_GIT_ENV = (
    "GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE", "GIT_OBJECT_DIRECTORY",
    "GIT_ALTERNATE_OBJECT_DIRECTORIES", "GIT_REPLACE_REF_BASE",
)

class QualificationError(RuntimeError):
    pass

def canonical(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()

def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def clean_env() -> dict[str, str]:
    return {**{k: v for k, v in os.environ.items() if not k.startswith("GIT_")}, "GIT_NO_REPLACE_OBJECTS": "1", "GIT_CONFIG_NOSYSTEM": "1"}

def reject_git_env_overrides() -> None:
    present = [name for name in FORBIDDEN_GIT_ENV if os.environ.get(name)]
    if present:
        raise QualificationError(f"forbidden Git environment overrides: {present}")

def run(args: list[str], cwd: pathlib.Path, *, text: bool = True) -> subprocess.CompletedProcess[Any]:
    return subprocess.run(args, cwd=cwd, env=clean_env(), text=text, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)

def git(repo: pathlib.Path, *args: str) -> str:
    result = run(["git", *args], repo)
    if result.returncode:
        raise QualificationError(result.stderr.strip())
    return result.stdout.strip()

def git_bytes(repo: pathlib.Path, *args: str) -> bytes:
    result = run(["git", *args], repo, text=False)
    if result.returncode:
        raise QualificationError(result.stderr.decode(errors="replace"))
    return result.stdout

def verify_git_indirection(repo: pathlib.Path) -> None:
    git_dir = pathlib.Path(git(repo, "rev-parse", "--git-dir"))
    if not git_dir.is_absolute():
        git_dir = (repo / git_dir).resolve()
    for rel in ("info/grafts", "objects/info/alternates"):
        path = git_dir / rel
        if path.exists() and path.read_bytes().strip():
            raise QualificationError(f"Git indirection present: {rel}")
    if git(repo, "for-each-ref", "--format=%(refname)", "refs/replace"):
        raise QualificationError("Git replace refs present")

def verify_checkout(repo: pathlib.Path) -> tuple[str, str]:
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of product")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^{{tree}}") != PRODUCT_TREE:
        raise QualificationError("product tree mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^") != PRODUCT_PARENT:
        raise QualificationError("product parent mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_PARENT}^{{tree}}") != PRODUCT_PARENT_TREE:
        raise QualificationError("product parent tree mismatch")
    changed = tuple(sorted(git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines()))
    if changed != tuple(sorted(PATHS)):
        raise QualificationError(f"qualifier path set mismatch: {changed}")
    for path, oid in PRODUCT_BLOBS.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise QualificationError(f"product blob mismatch: {path}")
    return head, git(repo, "rev-parse", "HEAD^{tree}")

def expected_lock(repo: pathlib.Path) -> dict[str, Any]:
    source_blobs = {
        "README.md": git(repo, "rev-parse", f"HEAD:{PREFIX}/README.md"),
        "qualify.py": git(repo, "rev-parse", f"HEAD:{PREFIX}/qualify.py"),
        "test_qualify.py": git(repo, "rev-parse", f"HEAD:{PREFIX}/test_qualify.py"),
    }
    return {
        "schema": LOCK_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "parent": PRODUCT_PARENT, "parent_tree": PRODUCT_PARENT_TREE},
        "product_blobs": PRODUCT_BLOBS,
        "qualifier_source_blobs": source_blobs,
        "expected_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "StructuralExecutionOnly",
            "privacy_pass_backend_qualified": False,
            "token_cryptographically_verified": False,
            "atomic_single_use_established": False,
            "query_credit_granted": False,
            "anonymous_rate_limit_established": False,
            "enumeration_resistance_established": False,
            "production_admission": False,
            "application_authority": False,
        },
    }

def verify_lock(repo: pathlib.Path) -> str:
    raw = git_bytes(repo, "show", f"HEAD:{PREFIX}/lock.json")
    expected = expected_lock(repo)
    try:
        actual = json.loads(raw.decode())
    except Exception as exc:
        raise QualificationError(f"invalid lock JSON: {exc}") from exc
    if actual != expected:
        raise QualificationError("lock contract mismatch")
    if raw != canonical(expected):
        raise QualificationError("lock bytes are not canonical")
    return sha256(raw)

def read_product(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")

def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_product(repo, "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/Cargo.toml").decode()
    source = read_product(repo, "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/src/lib.rs").decode()
    for token in ("privacy-pass", "privacypass", "reqwest", "tokio", "holochain", "xenia"):
        if token in cargo.lower():
            raise QualificationError(f"unexpected runtime/backend dependency: {token}")
    required = (
        "AtomicSingleUseRequired", "ReadyForBackendVerification", "0x0001", "0x0002",
        "rfc9578-voprf-p384-sha384-token-type-0001",
        "rfc9578-blind-rsa-2048-sha384-token-type-0002",
        "redemption_context_sha256", "max_identifiers_per_credit",
        "token_cryptographically_verified", "token_unspent_verified",
        "token_atomically_consumed", "query_credit_granted",
        "anonymous_rate_limit_established", "enumeration_resistance_established",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required theorem token absent: {token}")
    for forbidden in FORBIDDEN_AUTHORITY_FIELDS:
        if forbidden in source:
            raise QualificationError(f"caller-controlled spend state present: {forbidden}")
    for overclaim in (
        "token_cryptographically_verified(&self) -> bool { true",
        "token_unspent_verified(&self) -> bool { true",
        "token_atomically_consumed(&self) -> bool { true",
        "query_credit_granted(&self) -> bool { true",
    ):
        if overclaim in source:
            raise QualificationError(f"authority overclaim present: {overclaim}")
    count = source.count("#[test]")
    if count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {count}")
    return {"registered_tests": count, "probe_profile": "psi-002b3aq-v0.1"}

def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    crate = root / "psi-privacy-pass-credit-core"
    marker = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/"
    for path in PRODUCT_BLOBS:
        relative = path.split(marker, 1)[1]
        destination = crate / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(read_product(repo, path))
    return crate

def execute(args: tuple[str, ...], cwd: pathlib.Path) -> dict[str, Any]:
    result = run(list(args), cwd)
    record = {"argv": list(args), "returncode": result.returncode, "stdout_sha256": sha256(result.stdout.encode()), "stderr_sha256": sha256(result.stderr.encode())}
    if result.returncode:
        raise QualificationError(f"command failed: {args}")
    return record

def qualify(repo: pathlib.Path, receipt_path: pathlib.Path) -> dict[str, Any]:
    reject_git_env_overrides()
    repo = repo.resolve(); receipt_path = receipt_path.resolve()
    try:
        receipt_path.relative_to(repo)
    except ValueError:
        pass
    else:
        raise QualificationError("receipt must be outside checkout")
    verify_git_indirection(repo)
    qualifier_commit, qualifier_tree = verify_checkout(repo)
    lock_sha256 = verify_lock(repo)
    probes = static_probes(repo)
    with tempfile.TemporaryDirectory(prefix="psi-002b3aq-") as directory:
        crate = materialize(repo, pathlib.Path(directory))
        records = [execute(command, crate) for command in COMMANDS]
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit or git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier identity changed during execution")
    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "blobs": PRODUCT_BLOBS},
        "qualifier": {"commit": qualifier_commit, "tree": qualifier_tree, "lock_sha256": lock_sha256},
        "probes": probes,
        "commands": records,
        "authority_scope": "StructuralExecutionOnly",
        "structural_source_compiled": True,
        "registered_tests_passed": True,
        "privacy_pass_backend_qualified": False,
        "token_cryptographically_verified": False,
        "atomic_single_use_established": False,
        "query_credit_granted": False,
        "anonymous_rate_limit_established": False,
        "enumeration_resistance_established": False,
        "production_admission": False,
        "application_authority": False,
    }
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    receipt_path.write_bytes(canonical(receipt))
    return receipt

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=pathlib.Path, default=pathlib.Path.cwd())
    parser.add_argument("--receipt", type=pathlib.Path, required=True)
    args = parser.parse_args()
    try:
        receipt = qualify(args.repo, args.receipt)
    except (QualificationError, OSError) as exc:
        print(f"PSI-002B3AQ FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
