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

PRODUCT_COMMIT = "e48e3bc6e9cb203084d2a5aad61c5ead1ada515d"
PRODUCT_TREE = "2bcc38014ec8a7b5ed74763631317c439cecf6f7"
PRODUCT_PARENT = "be4fabe15a7f9180db13d85725a020566d3f9095"
PRODUCT_PARENT_TREE = "0c2c17db883f1df9ba7ff753bb85d5ecf182e7d6"
SQLITE_CRATE = "mycelix-workspace/mycelix-core/libs/psi-query-credit-sqlite-spend"
SPEND_CRATE = "mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core"
CREDIT_CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core"
QUALIFIER_PREFIX = "ci-governance/psi-002b3b2a-r2q"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    f"{SQLITE_CRATE}/Cargo.toml": "7d3801e316ccbe5f1874e71c448c6f50fc31da68",
    f"{SQLITE_CRATE}/README.md": "2707c1d8ac17a4878b3dac8dacdaa97a1e15785a",
    f"{SQLITE_CRATE}/src/lib.rs": "52662dfed0ea698dbd48623bc1e6b97e9328932f",
}
SPEND_BLOBS = {
    f"{SPEND_CRATE}/Cargo.toml": "d2844d3b12f633b71898a012f233b8a2e27177d2",
    f"{SPEND_CRATE}/README.md": "441cecb8bcaa9535efb66c97286297e907ef4039",
    f"{SPEND_CRATE}/src/lib.rs": "130b639f499a858b7ad4be1a44de8f484bf07da9",
}
CREDIT_BLOBS = {
    f"{CREDIT_CRATE}/Cargo.toml": "3c40e8d3070a62320768df8030a3e81420da12df",
    f"{CREDIT_CRATE}/README.md": "24104d6badbd5d61f2a9acccbc99699916cddcaa",
    f"{CREDIT_CRATE}/src/lib.rs": "484d89b25bf27fd66e5346ab4eeab9e5cc6fe9f2",
}
EXPECTED_TEST_COUNT = 9
LOCK_SCHEMA = "mycelix.psi.002b3b2a.r2q.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b3b2a.r2q.receipt.v0.1"
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

def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def canonical(obj: Any) -> bytes:
    return (json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()

def clean_env() -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    return env

def reject_git_env_overrides(env: dict[str, str] | None = None) -> None:
    source = os.environ if env is None else env
    present = sorted(name for name in FORBIDDEN_GIT_ENV if source.get(name))
    if present:
        raise QualificationError(f"forbidden Git environment override(s): {present}")

def sh(args: list[str], cwd: pathlib.Path, check: bool = True, *, text: bool = True) -> subprocess.CompletedProcess[Any]:
    return subprocess.run(args, cwd=cwd, env=clean_env(), text=text, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check)

def git(repo: pathlib.Path, *args: str) -> str:
    return sh(["git", *args], repo).stdout.strip()

def git_bytes(repo: pathlib.Path, *args: str) -> bytes:
    return sh(["git", *args], repo, text=False).stdout

def reject_git_indirection(repo: pathlib.Path) -> None:
    git_dir = pathlib.Path(git(repo, "rev-parse", "--git-dir"))
    if not git_dir.is_absolute():
        git_dir = (repo / git_dir).resolve()
    for rel in ("info/grafts", "objects/info/alternates"):
        path = git_dir / rel
        if path.exists() and path.read_bytes().strip():
            raise QualificationError(f"git indirection present: {rel}")
    if git(repo, "for-each-ref", "--format=%(refname)", "refs/replace").strip():
        raise QualificationError("git replace refs present")

def verify_checkout(repo: pathlib.Path) -> tuple[str, str]:
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of SQLite r2")
    changed = tuple(sorted(filter(None, git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines())))
    if changed != tuple(sorted(QUALIFIER_PATHS)):
        raise QualificationError(f"qualifier path set mismatch: {changed}")
    return head, git(repo, "rev-parse", "HEAD^{tree}")

def verify_product(repo: pathlib.Path) -> None:
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^{{tree}}") != PRODUCT_TREE:
        raise QualificationError("product tree mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^") != PRODUCT_PARENT:
        raise QualificationError("product parent mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_PARENT}^{{tree}}") != PRODUCT_PARENT_TREE:
        raise QualificationError("product parent tree mismatch")
    for path, oid in {**PRODUCT_BLOBS, **SPEND_BLOBS, **CREDIT_BLOBS}.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise QualificationError(f"blob mismatch: {path}")

def expected_lock(source_blobs: dict[str, str]) -> dict[str, Any]:
    return {
        "schema": LOCK_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "parent": PRODUCT_PARENT, "parent_tree": PRODUCT_PARENT_TREE},
        "product_blobs": PRODUCT_BLOBS,
        "spend_blobs": SPEND_BLOBS,
        "credit_blobs": CREDIT_BLOBS,
        "qualifier": {"paths": list(QUALIFIER_PATHS), "source_blobs": source_blobs},
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(c) for c in COMMANDS],
        "authority": {
            "scope": "SqliteWalFullNonceReplayStoreOnly",
            "network_access": False,
            "sqlite_atomic_single_use_established": True,
            "sqlite_full_sync_profile_established": True,
            "token_nonce_cryptographically_bound": False,
            "privacy_pass_backend_qualified": False,
            "global_store_uniqueness_established": False,
            "hardware_power_loss_durability_established": False,
            "safe_compaction_established": False,
            "query_credit_granted": False,
            "production_admission": False,
            "application_authority": False,
        },
    }

def verify_lock(repo: pathlib.Path) -> str:
    source_blobs = {
        "README.md": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/README.md"),
        "qualify.py": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/qualify.py"),
        "test_qualify.py": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/test_qualify.py"),
    }
    raw = git_bytes(repo, "show", f"HEAD:{QUALIFIER_PREFIX}/lock.json")
    actual = json.loads(raw.decode("utf-8"))
    expected = expected_lock(source_blobs)
    if actual != expected:
        raise QualificationError("lock contract mismatch")
    if raw != canonical(expected):
        raise QualificationError("lock bytes are not canonical")
    return sha256(raw)

def read_product_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")

def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_product_blob(repo, f"{SQLITE_CRATE}/Cargo.toml").decode()
    source = read_product_blob(repo, f"{SQLITE_CRATE}/src/lib.rs").decode()
    required_cargo = (
        'psi-query-credit-spend-core = { path = "../psi-query-credit-spend-core" }',
        'rusqlite = { version = "=0.39.0", default-features = false, features = ["bundled"] }',
    )
    for token in required_cargo:
        if token not in cargo:
            raise QualificationError(f"required Cargo binding absent: {token}")
    required = (
        'pub const SQLITE_SPEND_PROFILE_V2: &str = "sqlite-wal-full-query-token-spend-v2-nonce-bound";',
        "pub const SQLITE_SCHEMA_VERSION_V2: i64 = 2;",
        "pub const SQLITE_BUSY_TIMEOUT_MS: u64 = 5_000;",
        "pub const SQLITE_SYNCHRONOUS_FULL_CODE: i64 = 2;",
        'connection.query_row("PRAGMA journal_mode=WAL"',
        'connection.pragma_update(None, "synchronous", SQLITE_SYNCHRONOUS_FULL_CODE)',
        'connection.pragma_query_value(None, "busy_timeout"',
        'connection.pragma_query_value(None, "user_version"',
        "TransactionBehavior::Immediate",
        "INSERT OR IGNORE INTO query_token_spends_v2",
        "transaction.commit()?;",
        "same_nonce_different_token_artifact_is_still_replay",
        "replay_fails_after_close_and_reopen",
        "two_connections_racing_one_database_have_one_winner",
        "safe_compaction_established(&self) -> bool { false }",
        "query_credit_granted(&self) -> bool { false }",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required SQLite r2 token absent: {token}")
    schema = source.split('const SCHEMA_SQL: &str = r#"', 1)[1].split('"#;', 1)[0]
    if "token_nonce_sha256" not in schema:
        raise QualificationError("nonce digest absent from persistent schema")
    if "token_sha256" in schema or "authenticator" in schema or "token_bytes" in schema:
        raise QualificationError("whole-token/raw-token material present in persistent schema")
    commit_pos = source.find("transaction.commit()?;")
    positive_pos = source.find("Ok(SqliteFullSyncConsumedQueryTokenV2 {")
    if commit_pos < 0 or positive_pos < 0 or commit_pos > positive_pos:
        raise QualificationError("positive can be constructed before commit")
    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")
    return {"committed_test_count": test_count, "source_probe_set": "psi-002b3b2a-r2q-v0.1", "rusqlite": "0.39.0", "schema_version": 2}

def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    groups = (
        ("psi-privacy-pass-credit-core", CREDIT_BLOBS),
        ("psi-query-credit-spend-core", SPEND_BLOBS),
        ("psi-query-credit-sqlite-spend", PRODUCT_BLOBS),
    )
    targets: dict[str, pathlib.Path] = {}
    for crate_name, blobs in groups:
        target = root / crate_name
        targets[crate_name] = target
        marker = f"/libs/{crate_name}/"
        for source_path in blobs:
            relative = source_path.split(marker, 1)[1]
            destination = target / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(read_product_blob(repo, source_path))
    return targets["psi-query-credit-sqlite-spend"]

def run_cmd(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
    completed = sh(args, cwd, check=False)
    return {"argv": args, "returncode": completed.returncode, "stdout_sha256": sha256(completed.stdout.encode()), "stderr_sha256": sha256(completed.stderr.encode())}

def qualify(repo: pathlib.Path, receipt_path: pathlib.Path) -> dict[str, Any]:
    reject_git_env_overrides()
    repo = repo.resolve()
    receipt_path = receipt_path.resolve()
    try:
        receipt_path.relative_to(repo)
    except ValueError:
        pass
    else:
        raise QualificationError("receipt must be outside checkout")
    reject_git_indirection(repo)
    qualifier_commit, qualifier_tree = verify_checkout(repo)
    verify_product(repo)
    lock_sha256 = verify_lock(repo)
    probes = static_probes(repo)
    with tempfile.TemporaryDirectory(prefix="psi-002b3b2a-r2q-") as td:
        cwd = materialize(repo, pathlib.Path(td))
        commands = [run_cmd(list(c), cwd) for c in COMMANDS]
        failed = next((r for r in commands if r["returncode"] != 0), None)
        if failed:
            raise QualificationError(f"command failed: {failed['argv']}")
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit or git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier identity changed during qualification")
    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "blobs": PRODUCT_BLOBS, "spend_blobs": SPEND_BLOBS, "credit_blobs": CREDIT_BLOBS},
        "qualifier": {"commit": qualifier_commit, "tree": qualifier_tree, "lock_sha256": lock_sha256},
        "probes": probes,
        "commands": commands,
        "authority_scope": "SqliteWalFullNonceReplayStoreOnly",
        "sqlite_source_compiled": True,
        "registered_tests_passed": True,
        "sqlite_atomic_single_use_established": True,
        "sqlite_full_sync_profile_established": True,
        "token_nonce_cryptographically_bound": False,
        "privacy_pass_backend_qualified": False,
        "global_store_uniqueness_established": False,
        "hardware_power_loss_durability_established": False,
        "safe_compaction_established": False,
        "query_credit_granted": False,
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
    except (QualificationError, subprocess.CalledProcessError, OSError, json.JSONDecodeError) as exc:
        print(f"PSI-002B3B2A r2Q FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
