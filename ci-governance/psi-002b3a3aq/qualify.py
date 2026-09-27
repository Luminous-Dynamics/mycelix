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

PRODUCT_COMMIT = "b4985d90813a3cb303eb5d253531b7fc981a3c13"
PRODUCT_TREE = "79a900c8cd536b77e9f77be6fd17eec26b399f41"
PRODUCT_PARENT = "e32b54c86d602989955820fe1be5cbe88490e1e5"
PRODUCT_PARENT_TREE = "e24774507e1a7c29d757b5ed9c7a033952ab8775"
CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-issuer-directory-core"
B3A = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core"
QUALIFIER_PREFIX = "ci-governance/psi-002b3a3aq"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    f"{CRATE}/Cargo.toml": "d81bbc3070d0f86e8338df8a36fd87bfb7f9516c",
    f"{CRATE}/README.md": "1ebbb0a251ad7c25e8a940bad70a71d5e551f4ce",
    f"{CRATE}/src/lib.rs": "dc91418ecf6ae47ef929b938a97fc05f47d6a166",
}
B3A_BLOBS = {
    f"{B3A}/Cargo.toml": "3c40e8d3070a62320768df8030a3e81420da12df",
    f"{B3A}/README.md": "24104d6badbd5d61f2a9acccbc99699916cddcaa",
    f"{B3A}/src/lib.rs": "484d89b25bf27fd66e5346ab4eeab9e5cc6fe9f2",
}
EXPECTED_TEST_COUNT = 11
LOCK_SCHEMA = "mycelix.psi.002b3a3aq.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b3a3aq.receipt.v0.1"
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


def reject_git_env_overrides(env: dict[str, str] | None = None) -> None:
    source = os.environ if env is None else env
    present = sorted(name for name in FORBIDDEN_GIT_ENV if source.get(name))
    if present:
        raise QualificationError(f"forbidden Git environment override(s): {present}")


def clean_env() -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    env["CARGO_NET_OFFLINE"] = "true"
    return env


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
    reject_git_indirection(repo)
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of A3A product")
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
    for path, oid in {**PRODUCT_BLOBS, **B3A_BLOBS}.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise QualificationError(f"source blob mismatch: {path}")


def expected_lock(source_blobs: dict[str, str]) -> dict[str, Any]:
    return {
        "schema": LOCK_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "parent": PRODUCT_PARENT,
            "parent_tree": PRODUCT_PARENT_TREE,
            "blobs": PRODUCT_BLOBS,
        },
        "b3a_blobs": B3A_BLOBS,
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
        },
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "StructuralIssuerDirectoryOnly",
            "directory_payload_authenticated": False,
            "retrieval_provider_trusted": False,
            "directory_freshness_established": False,
            "trusted_clock_not_before_established": False,
            "issuer_key_admitted": False,
            "issuer_key_current": False,
            "token_verified": False,
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
        raise QualificationError("lock bytes are not canonical JSON")
    return sha256(raw)


def read_product_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_product_blob(repo, f"{CRATE}/Cargo.toml").decode("utf-8")
    source = read_product_blob(repo, f"{CRATE}/src/lib.rs").decode("utf-8")
    required_cargo = (
        'psi-privacy-pass-credit-core = { path = "../psi-privacy-pass-credit-core" }',
        'serde = { version = "1.0", features = ["derive"] }',
        'sha2 = "0.10"',
    )
    for token in required_cargo:
        if token not in cargo:
            raise QualificationError(f"required Cargo binding absent: {token}")
    for token in ("reqwest", "tokio", "holochain", "xenia", "chrono", "time ="):
        if token.lower() in cargo.lower():
            raise QualificationError(f"forbidden runtime/dependency token: {token}")

    required_source = (
        'pub const ISSUER_DIRECTORY_MEDIA_TYPE: &str = "application/private-token-issuer-directory";',
        "StructurallyConsistentIssuerDirectoryKeyObservationV1",
        "sha256_hex(&self.public_key_spki_der) != self.token_key_id_sha256",
        "&self.token_key_id_sha256[62..64]",
        "for key in &self.token_keys",
        "observation.response_body_sha256 != query_policy.issuer_configuration_sha256",
        "ExactFullKeyAbsent",
        "truncated_id_collision_is_surfaced_without_equating_full_keys",
        "not_before_is_retained_but_never_mints_currentness",
        "friendly_http_cache_claims_do_not_establish_freshness",
        "pub const fn directory_payload_authenticated(&self) -> bool { false }",
        "pub const fn directory_freshness_established(&self) -> bool { false }",
        "pub const fn issuer_key_admitted_under_service_policy(&self) -> bool { false }",
        "pub const fn issuer_key_current_under_service_policy(&self) -> bool { false }",
        "pub const fn token_cryptographically_verified(&self) -> bool { false }",
        "pub const fn query_credit_granted(&self) -> bool { false }",
        "pub const fn application_authority_granted(&self) -> bool { false }",
    )
    for token in required_source:
        if token not in source:
            raise QualificationError(f"required source ratchet absent: {token}")

    forbidden_source = (
        "SystemTime",
        "Utc::now",
        "Local::now",
        "Instant::now",
        "reqwest::",
        "tokio::",
        "hdk::",
        "holochain",
        "issuer_key_current_under_service_policy(&self) -> bool { true }",
        "directory_freshness_established(&self) -> bool { true }",
    )
    for token in forbidden_source:
        if token in source:
            raise QualificationError(f"forbidden source token present: {token}")

    count = source.count("#[test]")
    if count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {count}")
    return {
        "source_probe_set": "psi-002b3a3aq-v0.1",
        "committed_test_count": count,
        "ambient_clock_api_absent": True,
        "network_runtime_dependency_absent": True,
    }


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    groups = (("psi-privacy-pass-credit-core", B3A, B3A_BLOBS), ("psi-privacy-pass-issuer-directory-core", CRATE, PRODUCT_BLOBS))
    for crate_name, prefix, files in groups:
        base = workspace / crate_name
        marker = prefix + "/"
        for source_path in files:
            if not source_path.startswith(marker):
                continue
            relative = source_path[len(marker):]
            target = base / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(read_product_blob(repo, source_path))
    return workspace / "psi-privacy-pass-issuer-directory-core"


def run_command(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
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

    qualifier_commit, qualifier_tree = verify_checkout(repo)
    verify_product(repo)
    lock_sha256 = verify_lock(repo)
    probes = static_probes(repo)

    with tempfile.TemporaryDirectory(prefix="psi-002b3a3aq-") as temp:
        crate = materialize(repo, pathlib.Path(temp))
        versions = {
            "rustc": run_command(["rustc", "--version"], crate),
            "cargo": run_command(["cargo", "--version"], crate),
        }
        for record in versions.values():
            require_success(record)
        commands = [run_command(list(command), crate) for command in COMMANDS]
        for record in commands:
            require_success(record)
        lock_path = crate / "Cargo.lock"
        if not lock_path.exists():
            raise QualificationError("Cargo.lock not produced")
        cargo_lock_sha256 = sha256(lock_path.read_bytes())

    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit:
        raise QualificationError("HEAD changed during qualification")
    if git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier tree changed during qualification")

    receipt: dict[str, Any] = {
        "schema": RECEIPT_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "parent": PRODUCT_PARENT,
            "blobs": PRODUCT_BLOBS,
            "b3a_blobs": B3A_BLOBS,
        },
        "qualifier": {
            "commit": qualifier_commit,
            "tree": qualifier_tree,
            "lock_sha256": lock_sha256,
        },
        "probes": probes,
        "versions": versions,
        "commands": commands,
        "cargo_lock_sha256": cargo_lock_sha256,
        "authority_scope": "StructuralIssuerDirectoryOnly",
        "directory_payload_authenticated": False,
        "retrieval_provider_trusted": False,
        "directory_freshness_established": False,
        "trusted_clock_not_before_established": False,
        "issuer_key_admitted": False,
        "issuer_key_current": False,
        "token_verified": False,
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
    except (QualificationError, OSError, subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        print(f"PSI-002B3A3AQ FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
