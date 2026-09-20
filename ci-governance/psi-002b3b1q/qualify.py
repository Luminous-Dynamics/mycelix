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

PRODUCT_COMMIT = "10eab4ec3c5833d9230fc9bfde89547a0cea1246"
PRODUCT_TREE = "8aa6fa930e17761dd8d23e23160a78342e5744ca"
PRODUCT_PARENT = "1a5fdee47750c2f90e76e4e9dd4374603b56cfd7"
PRODUCT_PARENT_TREE = "e9dcaca88577dfd53781c343c07f107ecb259467"
PREFIX = "ci-governance/psi-002b3b1q"
PATHS = (
    f"{PREFIX}/README.md",
    f"{PREFIX}/lock.json",
    f"{PREFIX}/qualify.py",
    f"{PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/Cargo.toml": "6763b9c6598ef710a062fe3444d6b07db51c4f53",
    "mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/README.md": "75bce76b3ccd10f4fa1055ebbd7cd61ab19cbe9b",
    "mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/src/lib.rs": "bf19c57689682e673e9e26e2d37e430fe2242e3e",
}
B3A_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/Cargo.toml": "ba44563a663b3e44a41477fdf6fa50dde7dde1dd",
    "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/README.md": "907d300c5e1692db32281d5a296589dbe8c1d8d6",
    "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/src/lib.rs": "107d5b4063270a209b87db4fb6f3e59c8e3d02d3",
}
EXPECTED_TEST_COUNT = 8
LOCK_SCHEMA = "mycelix.psi.002b3b1q.lock.v0.1"
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
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    return env

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
        raise QualificationError("dirty checkout")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct product child")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^{{tree}}") != PRODUCT_TREE:
        raise QualificationError("product tree mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^") != PRODUCT_PARENT:
        raise QualificationError("product parent mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_PARENT}^{{tree}}") != PRODUCT_PARENT_TREE:
        raise QualificationError("product parent tree mismatch")
    changed = tuple(sorted(git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines()))
    if changed != tuple(sorted(PATHS)):
        raise QualificationError(f"qualifier path set mismatch: {changed}")
    for path, oid in {**PRODUCT_BLOBS, **B3A_BLOBS}.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise QualificationError(f"blob mismatch: {path}")
    return head, git(repo, "rev-parse", "HEAD^{tree}")

def expected_lock(repo: pathlib.Path) -> dict[str, Any]:
    return {
        "schema": LOCK_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "parent": PRODUCT_PARENT,
            "parent_tree": PRODUCT_PARENT_TREE,
        },
        "product_blobs": PRODUCT_BLOBS,
        "b3a_blobs": B3A_BLOBS,
        "qualifier_source_blobs": {
            "README.md": git(repo, "rev-parse", f"HEAD:{PREFIX}/README.md"),
            "qualify.py": git(repo, "rev-parse", f"HEAD:{PREFIX}/qualify.py"),
            "test_qualify.py": git(repo, "rev-parse", f"HEAD:{PREFIX}/test_qualify.py"),
        },
        "expected_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "ProcessLocalReferenceExecutionOnly",
            "durable_single_use_established": False,
            "multi_process_single_use_established": False,
            "crash_safe_single_use_established": False,
            "privacy_pass_backend_qualified": False,
            "query_credit_granted": False,
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

def probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_product(repo, "mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/Cargo.toml").decode()
    source = read_product(repo, "mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/src/lib.rs").decode()
    for token in ("rusqlite", "sqlite", "rocksdb", "sled", "redb", "tokio", "reqwest", "holochain", "xenia"):
        if token in cargo.lower():
            raise QualificationError(f"unexpected durable/network dependency: {token}")
    required = (
        "ProcessLocalAtomicSpendStoreV1",
        "Mutex<BTreeSet<String>>",
        "concurrent_race_has_exactly_one_winner",
        "Barrier::new(16)",
        "ProcessLocalConsumedQueryTokenV1",
        "process_local_atomic_single_use_established",
        "durable_single_use_established",
        "multi_process_single_use_established",
        "crash_safe_single_use_established",
        "privacy_pass_token_cryptographically_verified",
        "query_credit_granted",
        "evaluate_redemption_structure_v1",
        "ReadyForBackendVerification",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required theorem token absent: {token}")
    if "Deserialize)]\npub struct ProcessLocalConsumedQueryTokenV1" in source:
        raise QualificationError("positive became deserializable")
    for overclaim in (
        "durable_single_use_established(&self) -> bool { true",
        "multi_process_single_use_established(&self) -> bool { true",
        "crash_safe_single_use_established(&self) -> bool { true",
        "privacy_pass_token_cryptographically_verified(&self) -> bool { true",
        "query_credit_granted(&self) -> bool { true",
    ):
        if overclaim in source:
            raise QualificationError(f"authority overclaim present: {overclaim}")
    count = source.count("#[test]")
    if count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {count}")
    return {"registered_tests": count, "probe_profile": "psi-002b3b1q-v0.1"}

def materialize(repo: pathlib.Path, tmp: pathlib.Path) -> pathlib.Path:
    for crate_name, blobs in (("psi-privacy-pass-credit-core", B3A_BLOBS), ("psi-query-credit-spend-core", PRODUCT_BLOBS)):
        marker = f"mycelix-workspace/mycelix-core/libs/{crate_name}/"
        root = tmp / crate_name
        for path in blobs:
            relative = path.split(marker, 1)[1]
            destination = root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(read_product(repo, path))
    return tmp / "psi-query-credit-spend-core"

def execute(command: tuple[str, ...], cwd: pathlib.Path) -> dict[str, Any]:
    result = run(list(command), cwd)
    record = {
        "argv": list(command),
        "returncode": result.returncode,
        "stdout_sha256": sha256(result.stdout.encode()),
        "stderr_sha256": sha256(result.stderr.encode()),
    }
    if result.returncode:
        raise QualificationError(f"command failed: {command}")
    return record

def qualify(repo: pathlib.Path, receipt: pathlib.Path) -> dict[str, Any]:
    reject_git_env_overrides()
    repo = repo.resolve()
    receipt = receipt.resolve()
    try:
        receipt.relative_to(repo)
    except ValueError:
        pass
    else:
        raise QualificationError("receipt must be outside checkout")
    verify_git_indirection(repo)
    head, tree = verify_checkout(repo)
    lock_sha256 = verify_lock(repo)
    probe_result = probes(repo)
    with tempfile.TemporaryDirectory(prefix="psi-002b3b1q-") as directory:
        crate = materialize(repo, pathlib.Path(directory))
        records = [execute(command, crate) for command in COMMANDS]
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated")
    if git(repo, "rev-parse", "HEAD") != head or git(repo, "rev-parse", "HEAD^{tree}") != tree:
        raise QualificationError("qualifier identity changed")
    output = {
        "schema": "mycelix.psi.002b3b1q.receipt.v0.1",
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE},
        "qualifier": {"commit": head, "tree": tree, "lock_sha256": lock_sha256},
        "probes": probe_result,
        "commands": records,
        "authority_scope": "ProcessLocalReferenceExecutionOnly",
        "process_local_atomic_single_use_tested": True,
        "durable_single_use_established": False,
        "multi_process_single_use_established": False,
        "crash_safe_single_use_established": False,
        "privacy_pass_backend_qualified": False,
        "query_credit_granted": False,
        "production_admission": False,
        "application_authority": False,
    }
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_bytes(canonical(output))
    return output

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=pathlib.Path, default=pathlib.Path.cwd())
    parser.add_argument("--receipt", type=pathlib.Path, required=True)
    args = parser.parse_args()
    try:
        result = qualify(args.repo, args.receipt)
    except (QualificationError, OSError) as exc:
        print(f"PSI-002B3B1Q FAIL: {exc}")
        return 1
    print(canonical(result).decode(), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
