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

PRODUCT_COMMIT = "31713561f743294c3ff0d4b3b33aa0ac62478c34"
PRODUCT_TREE = "6b8bcbf3080a1abe8f1f1ab1f8a539ec59184329"
PRODUCT_PARENT = "e32b54c86d602989955820fe1be5cbe88490e1e5"
PRODUCT_PARENT_TREE = "e24774507e1a7c29d757b5ed9c7a033952ab8775"
CHALLENGE_CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-rfc9577-challenge"
CREDIT_CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core"
QUALIFIER_PREFIX = "ci-governance/psi-002b3a1q"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    f"{CHALLENGE_CRATE}/Cargo.toml": "b594108d26bc60cca41519a47ba221c798730739",
    f"{CHALLENGE_CRATE}/README.md": "3cde7299a1760f1595d1275073342010b3e0073b",
    f"{CHALLENGE_CRATE}/src/lib.rs": "5d08799814c7af4f3c3b97d6cd3d8da90a139a48",
}
DEPENDENCY_BLOBS = {
    f"{CREDIT_CRATE}/Cargo.toml": "3c40e8d3070a62320768df8030a3e81420da12df",
    f"{CREDIT_CRATE}/README.md": "24104d6badbd5d61f2a9acccbc99699916cddcaa",
    f"{CREDIT_CRATE}/src/lib.rs": "484d89b25bf27fd66e5346ab4eeab9e5cc6fe9f2",
}
EXPECTED_TEST_COUNT = 9
LOCK_SCHEMA = "mycelix.psi.002b3a1q.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b3a1q.receipt.v0.1"
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


def sh(
    args: list[str],
    cwd: pathlib.Path,
    check: bool = True,
    *,
    text: bool = True,
) -> subprocess.CompletedProcess[Any]:
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
    for relative in ("info/grafts", "objects/info/alternates"):
        candidate = git_dir / relative
        if candidate.exists() and candidate.read_bytes().strip():
            raise QualificationError(f"git indirection present: {relative}")
    if git(repo, "for-each-ref", "--format=%(refname)", "refs/replace").strip():
        raise QualificationError("git replace refs present")


def verify_checkout(repo: pathlib.Path) -> tuple[str, str]:
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of B3A1")
    changed = tuple(
        sorted(filter(None, git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines()))
    )
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
    for path, oid in {**PRODUCT_BLOBS, **DEPENDENCY_BLOBS}.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise QualificationError(f"blob mismatch: {path}")


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
        "dependency_blobs": DEPENDENCY_BLOBS,
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
        },
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "Rfc9577ChallengeEncodingOnly",
            "network_access": False,
            "exact_rfc9577_default_challenge_encoding_established": True,
            "token_challenge_digest_cryptographically_bound": False,
            "token_cryptographically_verified": False,
            "token_nonce_cryptographically_bound": False,
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
    try:
        actual = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QualificationError(f"invalid lock JSON: {exc}") from exc
    expected = expected_lock(source_blobs)
    if actual != expected:
        raise QualificationError("lock contract mismatch")
    if raw != canonical(expected):
        raise QualificationError("lock bytes are not canonical")
    return sha256(raw)


def read_product_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_product_blob(repo, f"{CHALLENGE_CRATE}/Cargo.toml").decode("utf-8")
    source = read_product_blob(repo, f"{CHALLENGE_CRATE}/src/lib.rs").decode("utf-8")
    if 'psi-privacy-pass-credit-core = { path = "../psi-privacy-pass-credit-core" }' not in cargo:
        raise QualificationError("exact B3A r2 dependency absent")
    for token in ("privacypass", "reqwest", "tokio", "holochain", "xenia"):
        if token.lower() in cargo.lower():
            raise QualificationError(f"forbidden runtime/backend dependency present: {token}")

    required = (
        '"8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f"',
        '"0002000e6973737565722e6578616d706c6520476ac2c935f458e9b2d7af32dacfbd22dd6023ef5887a789f1abe004e79bb5bb000e6f726967696e2e6578616d706c65"',
        "assert_eq!(wire.len(), 67);",
        "out.extend_from_slice(&token_type.code().to_be_bytes());",
        "out.extend_from_slice(&issuer_len.to_be_bytes());",
        "out.push(context_len);",
        "out.extend_from_slice(&origin_len.to_be_bytes());",
        "let expected = policy.challenge_binding()?;",
        "if challenge != &expected",
        "exact_rfc9577_default_challenge_encoding_established(&self) -> bool",
        "token_challenge_digest_cryptographically_bound(&self) -> bool",
        "query_credit_granted(&self) -> bool",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required RFC source token absent: {token}")

    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")
    return {
        "committed_test_count": test_count,
        "source_probe_set": "psi-002b3a1q-v0.1",
        "rfc_vector_1_wire_length": 67,
        "rfc_vector_1_sha256": "8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f",
    }


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    groups = (
        ("psi-privacy-pass-credit-core", DEPENDENCY_BLOBS),
        ("psi-privacy-pass-rfc9577-challenge", PRODUCT_BLOBS),
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
    return targets["psi-privacy-pass-rfc9577-challenge"]


def run_cmd(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
    completed = sh(args, cwd, check=False)
    return {
        "argv": args,
        "returncode": completed.returncode,
        "stdout_sha256": sha256(completed.stdout.encode()),
        "stderr_sha256": sha256(completed.stderr.encode()),
    }


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

    with tempfile.TemporaryDirectory(prefix="psi-002b3a1q-") as temp_dir:
        cwd = materialize(repo, pathlib.Path(temp_dir))
        commands = [run_cmd(list(command), cwd) for command in COMMANDS]
        failed = next((record for record in commands if record["returncode"] != 0), None)
        if failed:
            raise QualificationError(f"command failed: {failed['argv']}")

    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit:
        raise QualificationError("HEAD changed during qualification")
    if git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier tree changed during qualification")

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "blobs": PRODUCT_BLOBS,
            "dependency_blobs": DEPENDENCY_BLOBS,
        },
        "qualifier": {
            "commit": qualifier_commit,
            "tree": qualifier_tree,
            "lock_sha256": lock_sha256,
        },
        "probes": probes,
        "commands": commands,
        "authority_scope": "Rfc9577ChallengeEncodingOnly",
        "source_compiled": True,
        "registered_tests_passed": True,
        "exact_rfc9577_default_challenge_encoding_established": True,
        "token_challenge_digest_cryptographically_bound": False,
        "token_cryptographically_verified": False,
        "token_nonce_cryptographically_bound": False,
        "atomic_single_use_established": False,
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
    except (QualificationError, subprocess.CalledProcessError, OSError) as exc:
        print(f"PSI-002B3A1Q FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
