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

PRODUCT_COMMIT = "3193bcf7b92feed1f0599dca47ecafbfa178404f"
PRODUCT_TREE = "9e834e47ab0c97f2813e9191bff84efa080b8520"
PRODUCT_PARENT = "d63eb7c2dec46c3b0b6faeee3062bdfded0ba3b6"
PRODUCT_PARENT_TREE = "16ad4fe0139d261edaebbcaf25fdafba583c3853"
PREFIX = "ci-governance/psi-002b2b-r2q"
QUALIFIER_PATHS = (
    f"{PREFIX}/README.md",
    f"{PREFIX}/lock.json",
    f"{PREFIX}/qualify.py",
    f"{PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-xenia-registry-provider-provenance/Cargo.toml": "f5c46d62797a2a727850ec865cd8a346fc54f655",
    "mycelix-workspace/mycelix-core/libs/psi-xenia-registry-provider-provenance/README.md": "fa4dc2a9d62361f4bd75107d9711c6f2afe2183e",
    "mycelix-workspace/mycelix-core/libs/psi-xenia-registry-provider-provenance/src/lib.rs": "c2fd031712dbf89bb3a4b1abfcc809911054889b",
}
DEPENDENCY_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/Cargo.toml": "eff3c73ca452c7395160ae0c19a95a8a66dca466",
    "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/src/lib.rs": "e90ce4ca4a1c8be451a9d8d7003a9a802d13cfe6",
}
EXPECTED_TEST_COUNT = 6
LOCK_SCHEMA = "mycelix.psi.002b2b.r2q.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b2b.r2q.receipt.v0.1"
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


def reject_git_env(env: dict[str, str] | None = None) -> None:
    source = os.environ if env is None else env
    present = sorted(name for name in FORBIDDEN_GIT_ENV if source.get(name))
    if present:
        raise QualificationError(f"forbidden Git environment overrides: {present}")


def clean_env() -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    return env


def run(args: list[str], cwd: pathlib.Path, *, check: bool = False, text: bool = True) -> subprocess.CompletedProcess[Any]:
    return subprocess.run(args, cwd=cwd, env=clean_env(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=text, check=check)


def git(repo: pathlib.Path, *args: str) -> str:
    return run(["git", *args], repo, check=True).stdout.strip()


def git_bytes(repo: pathlib.Path, *args: str) -> bytes:
    return run(["git", *args], repo, check=True, text=False).stdout


def reject_git_indirection(repo: pathlib.Path) -> None:
    git_dir = pathlib.Path(git(repo, "rev-parse", "--git-dir"))
    if not git_dir.is_absolute():
        git_dir = (repo / git_dir).resolve()
    for rel in ("info/grafts", "objects/info/alternates"):
        path = git_dir / rel
        if path.exists() and path.read_bytes().strip():
            raise QualificationError(f"git indirection present: {rel}")
    if git(repo, "for-each-ref", "--format=%(refname)", "refs/replace"):
        raise QualificationError("git replace refs present")


def verify_checkout(repo: pathlib.Path) -> tuple[str, str]:
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of B2B r2")
    paths = tuple(sorted(git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines()))
    if paths != tuple(sorted(QUALIFIER_PATHS)):
        raise QualificationError(f"unexpected qualifier paths: {paths}")
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
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "parent": PRODUCT_PARENT, "parent_tree": PRODUCT_PARENT_TREE},
        "product_blobs": PRODUCT_BLOBS,
        "dependency_blobs": DEPENDENCY_BLOBS,
        "qualifier": {"paths": list(QUALIFIER_PATHS), "source_blobs": source_blobs},
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(c) for c in COMMANDS],
        "authority": {
            "scope": "PolicyTrustedProviderProvenanceExecutionOnly",
            "provider_signatures_verified": True,
            "provider_identity_trusted_under_policy": True,
            "producer_contract_qualified": False,
            "registry_authenticated": False,
            "registry_current": False,
            "psi_security_established": False,
            "composition_qualified": False,
            "production_admission": False,
            "application_authority": False,
        },
    }


def verify_lock(repo: pathlib.Path) -> str:
    source_blobs = {
        "README.md": git(repo, "rev-parse", f"HEAD:{PREFIX}/README.md"),
        "qualify.py": git(repo, "rev-parse", f"HEAD:{PREFIX}/qualify.py"),
        "test_qualify.py": git(repo, "rev-parse", f"HEAD:{PREFIX}/test_qualify.py"),
    }
    raw = git_bytes(repo, "show", f"HEAD:{PREFIX}/lock.json")
    try:
        actual = json.loads(raw.decode())
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QualificationError(f"invalid lock JSON: {exc}") from exc
    expected = expected_lock(source_blobs)
    if actual != expected or raw != canonical(expected):
        raise QualificationError("lock contract mismatch")
    return sha256(raw)


def blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = blob(repo, "mycelix-workspace/mycelix-core/libs/psi-xenia-registry-provider-provenance/Cargo.toml").decode()
    source = blob(repo, "mycelix-workspace/mycelix-core/libs/psi-xenia-registry-provider-provenance/src/lib.rs").decode()

    for token in (
        'ed25519-dalek = "2"',
        'ml-dsa = { version = "0.1.1", default-features = true }',
        'psi-registry-evidence-core = { path = "../psi-registry-evidence-core" }',
    ):
        if token not in cargo:
            raise QualificationError(f"required dependency binding absent: {token}")
    for forbidden in ("reqwest", "tokio", "holochain", "xenia-peer"):
        if forbidden.lower() in cargo.lower():
            raise QualificationError(f"unexpected runtime/provider dependency: {forbidden}")

    required = (
        'b"xenia-mycelix-registry-provider-attestation-v1\\0"',
        'b"xenia-mycelix-registry/verifier-identity/v1\\0"',
        'b"registry-authenticity-established-v1"',
        "PolicyTrustedXeniaRegistryProviderProvenanceV1",
        "bind_structurally_consistent_observation_v1(subject, policy, observation)",
        "provider_signatures_verified(&self) -> bool { true }",
        "provider_identity_trusted_under_policy(&self) -> bool { true }",
        "producer_contract_qualified(&self) -> bool { false }",
        "registry_authenticated(&self) -> bool { false }",
        "registry_current(&self) -> bool { false }",
        "verify_ed25519(trusted_verifier, envelope, &transcript)?",
        "verify_ml_dsa_65(trusted_verifier, envelope, &transcript)?",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required provenance token absent: {token}")

    if "registry_authenticated(&self) -> bool { true }" in source:
        raise QualificationError("consumer provenance layer promotes registry authentication")
    if "producer_contract_qualified(&self) -> bool { true }" in source:
        raise QualificationError("consumer provenance layer promotes producer qualification")

    start = source.index("pub struct XeniaRegistryProviderAttestedReceiptV1")
    end = source.index("impl XeniaRegistryProviderAttestedReceiptV1", start)
    if "current" in source[start:end].lower():
        raise QualificationError("portable provider envelope unexpectedly carries currentness")

    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")
    return {"probe_profile": "psi-002b2b-r2q-v0.1", "committed_test_count": test_count}


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    libs = root / "libs"
    for crate_name, files in (("psi-registry-evidence-core", DEPENDENCY_BLOBS), ("psi-xenia-registry-provider-provenance", PRODUCT_BLOBS)):
        marker = f"/libs/{crate_name}/"
        base = libs / crate_name
        for source_path in files:
            if marker not in source_path or source_path.endswith("README.md"):
                continue
            relative = source_path.split(marker, 1)[1]
            target = base / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(blob(repo, source_path))
    workdir = libs / "psi-xenia-registry-provider-provenance"
    if not workdir.exists():
        raise QualificationError("isolated product materialization failed")
    return workdir


def record(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
    result = run(args, cwd)
    return {"argv": args, "returncode": result.returncode, "stdout_sha256": sha256(result.stdout.encode()), "stderr_sha256": sha256(result.stderr.encode())}


def require_success(item: dict[str, Any]) -> None:
    if item["returncode"] != 0:
        raise QualificationError(f"command failed: {item['argv']}")


def qualify(repo: pathlib.Path, receipt_path: pathlib.Path) -> dict[str, Any]:
    reject_git_env()
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

    with tempfile.TemporaryDirectory(prefix="psi-002b2b-r2q-") as temp:
        workdir = materialize(repo, pathlib.Path(temp))
        versions = {"rustc": record(["rustc", "--version"], workdir), "cargo": record(["cargo", "--version"], workdir)}
        for item in versions.values():
            require_success(item)
        commands = [record(list(command), workdir) for command in COMMANDS]
        for item in commands:
            require_success(item)
        lockfile = workdir / "Cargo.lock"
        if not lockfile.exists():
            raise QualificationError("Cargo.lock not produced")
        cargo_lock_sha256 = sha256(lockfile.read_bytes())

    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit or git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier identity changed during qualification")

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "blobs": PRODUCT_BLOBS, "dependency_blobs": DEPENDENCY_BLOBS},
        "qualifier": {"commit": qualifier_commit, "tree": qualifier_tree, "lock_sha256": lock_sha256},
        "probes": probes,
        "versions": versions,
        "commands": commands,
        "cargo_lock_sha256": cargo_lock_sha256,
        "authority_scope": "PolicyTrustedProviderProvenanceExecutionOnly",
        "source_compiled": True,
        "registered_tests_passed": True,
        "provider_signatures_verified": True,
        "provider_identity_trusted_under_policy": True,
        "producer_contract_qualified": False,
        "registry_authenticated": False,
        "registry_current": False,
        "psi_security_established": False,
        "composition_qualified": False,
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
        print(f"PSI-002B2B-R2Q FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
