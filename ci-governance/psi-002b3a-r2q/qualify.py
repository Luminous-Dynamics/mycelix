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

PRODUCT_COMMIT = "e32b54c86d602989955820fe1be5cbe88490e1e5"
PRODUCT_TREE = "e24774507e1a7c29d757b5ed9c7a033952ab8775"
PRODUCT_PARENT = "c5c892ab096c5454f8e0bcad87940e0e7ac1a849"
PRODUCT_PARENT_TREE = "64a9c02021611de3e8352045de98f3202e28d644"
CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core"
QUALIFIER_PREFIX = "ci-governance/psi-002b3a-r2q"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    f"{CRATE}/Cargo.toml": "3c40e8d3070a62320768df8030a3e81420da12df",
    f"{CRATE}/README.md": "24104d6badbd5d61f2a9acccbc99699916cddcaa",
    f"{CRATE}/src/lib.rs": "484d89b25bf27fd66e5346ab4eeab9e5cc6fe9f2",
}
EXPECTED_TEST_COUNT = 11
LOCK_SCHEMA = "mycelix.psi.002b3a.r2q.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b3a.r2q.receipt.v0.1"
COMMANDS = (
    ("cargo", "fmt", "--check", "--all"),
    ("cargo", "test", "--offline", "--all-targets"),
    ("cargo", "clippy", "--offline", "--all-targets", "--all-features", "--", "-D", "warnings"),
)
FORBIDDEN_GIT_ENV = (
    "GIT_DIR",
    "GIT_WORK_TREE",
    "GIT_INDEX_FILE",
    "GIT_OBJECT_DIRECTORY",
    "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    "GIT_REPLACE_REF_BASE",
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
    return subprocess.run(
        args,
        cwd=cwd,
        env=clean_env(),
        text=text,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


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
        raise QualificationError("qualifier is not direct child of B3A r2")
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
    for path, oid in PRODUCT_BLOBS.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise QualificationError(f"product blob mismatch: {path}")


def expected_lock(source_blobs: dict[str, str]) -> dict[str, Any]:
    return {
        "schema": LOCK_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "parent": PRODUCT_PARENT,
            "parent_tree": PRODUCT_PARENT_TREE,
        },
        "product_blobs": PRODUCT_BLOBS,
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
        },
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "StructuralRfcSemanticsOnly",
            "network_access": False,
            "privacy_pass_backend_qualified": False,
            "token_cryptographically_verified": False,
            "token_nonce_cryptographically_bound": False,
            "challenge_digest_cryptographically_bound": False,
            "atomic_single_use_established": False,
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


def read_product(repo: pathlib.Path, relative: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{CRATE}/{relative}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_product(repo, "Cargo.toml").decode("utf-8")
    source = read_product(repo, "src/lib.rs").decode("utf-8")
    readme = read_product(repo, "README.md").decode("utf-8")

    for token in ('name = "psi-privacy-pass-credit-core"', 'version = "0.2.0"', 'serde = { version = "1.0"', 'sha2 = "0.10"'):
        if token not in cargo:
            raise QualificationError(f"required Cargo token absent: {token}")
    for token in ("privacypass", "reqwest", "tokio", "rusqlite", "holochain", "xenia"):
        if token in cargo.lower():
            raise QualificationError(f"runtime/crypto dependency forbidden in structural crate: {token}")

    required = (
        "AtomicSingleUseRequired",
        "ReadyForBackendVerification",
        "token_sha256",
        "token_nonce_sha256",
        "token_challenge_digest_sha256",
        "PrivateVoprfP384Sha384",
        "PublicBlindRsa2048Sha384",
        "0x0001",
        "0x0002",
        "token_cryptographically_verified",
        "token_nonce_cryptographically_bound",
        "challenge_digest_cryptographically_bound",
        "token_unspent_verified",
        "token_atomically_consumed",
        "query_credit_granted",
        "anonymous_rate_limit_established",
        "enumeration_resistance_established",
        "application_authority_granted",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required r2 source token absent: {token}")

    forbidden_field_fragments = ("pub unspent:", "pub consumed:", "pub spent:")
    lowered = source.lower()
    for fragment in forbidden_field_fragments:
        if fragment in lowered:
            raise QualificationError(f"caller-controlled spend authority field present: {fragment}")

    if "token_nonce_sha256" not in readme or "token_challenge_digest_sha256" not in readme:
        raise QualificationError("r2 identity split missing from README")
    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")

    return {
        "source_probe_set": "psi-002b3a-r2q-v0.1",
        "committed_test_count": test_count,
        "nonce_replay_subject_declared": True,
        "challenge_digest_subject_declared": True,
    }


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    crate = root / "psi-privacy-pass-credit-core"
    for relative in ("Cargo.toml", "README.md", "src/lib.rs"):
        destination = crate / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(read_product(repo, relative))
    return crate


def run_cmd(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
    completed = sh(args, cwd, check=False)
    return {
        "argv": args,
        "returncode": completed.returncode,
        "stdout_sha256": sha256(completed.stdout.encode()),
        "stderr_sha256": sha256(completed.stderr.encode()),
    }


def require_success(record: dict[str, Any]) -> None:
    if record["returncode"] != 0:
        raise QualificationError(f"command failed: {record['argv']}")


def qualify(repo: pathlib.Path, receipt_output: pathlib.Path) -> dict[str, Any]:
    reject_git_env_overrides()
    repo = repo.resolve()
    receipt_output = receipt_output.resolve()
    try:
        receipt_output.relative_to(repo)
    except ValueError:
        pass
    else:
        raise QualificationError("receipt output must be outside checkout")

    reject_git_indirection(repo)
    qualifier_commit, qualifier_tree = verify_checkout(repo)
    verify_product(repo)
    lock_sha256 = verify_lock(repo)
    probes = static_probes(repo)

    with tempfile.TemporaryDirectory(prefix="psi-002b3a-r2q-") as temp_dir:
        crate = materialize(repo, pathlib.Path(temp_dir))
        versions = {
            "rustc": run_cmd(["rustc", "--version"], crate),
            "cargo": run_cmd(["cargo", "--version"], crate),
        }
        for record in versions.values():
            require_success(record)
        commands = [run_cmd(list(command), crate) for command in COMMANDS]
        for record in commands:
            require_success(record)

    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit or git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier identity changed during qualification")

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "blobs": PRODUCT_BLOBS},
        "qualifier": {"commit": qualifier_commit, "tree": qualifier_tree, "lock_sha256": lock_sha256},
        "probes": probes,
        "versions": versions,
        "commands": commands,
        "authority_scope": "StructuralRfcSemanticsOnly",
        "structural_source_compiled": True,
        "registered_tests_passed": True,
        "privacy_pass_backend_qualified": False,
        "token_cryptographically_verified": False,
        "token_nonce_cryptographically_bound": False,
        "challenge_digest_cryptographically_bound": False,
        "atomic_single_use_established": False,
        "query_credit_granted": False,
        "production_admission": False,
        "application_authority": False,
    }
    receipt_output.parent.mkdir(parents=True, exist_ok=True)
    receipt_output.write_bytes(canonical(receipt))
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=pathlib.Path, default=pathlib.Path.cwd())
    parser.add_argument("--receipt", type=pathlib.Path, required=True)
    args = parser.parse_args()
    try:
        receipt = qualify(args.repo, args.receipt)
    except (QualificationError, subprocess.CalledProcessError, OSError, json.JSONDecodeError) as exc:
        print(f"PSI-002B3A-r2Q FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
