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

PRODUCT_COMMIT = "d63eb7c2dec46c3b0b6faeee3062bdfded0ba3b6"
PRODUCT_TREE = "16ad4fe0139d261edaebbcaf25fdafba583c3853"
PRODUCT_PARENT = "c5c892ab096c5454f8e0bcad87940e0e7ac1a849"
PRODUCT_PARENT_TREE = "64a9c02021611de3e8352045de98f3202e28d644"
QUALIFIER_PREFIX = "ci-governance/psi-002b2aq"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/Cargo.toml": "eff3c73ca452c7395160ae0c19a95a8a66dca466",
    "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/README.md": "e90ce4ca4a1c8be451a9d8d7003a9a802d13cfe6",
    "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/src/lib.rs": "a006dea8d913f8cf14f79bbd568ed3164a390c40",
}
EXPECTED_TEST_COUNT = 10
LOCK_SCHEMA = "mycelix.psi.002b2aq.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b2aq.receipt.v0.1"
COMMANDS = (
    ("cargo", "fmt", "--check", "--all"),
    ("cargo", "test", "--offline", "--workspace"),
    ("cargo", "clippy", "--offline", "--workspace", "--all-targets", "--", "-D", "warnings"),
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
    return (json.dumps(obj, sort_keys=True, separators=(",", ":")) + "\n").encode()


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


def verify_checkout(repo: pathlib.Path) -> tuple[str, str]:
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of PSI-002B2A")
    paths = tuple(sorted(git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines()))
    if paths != tuple(sorted(QUALIFIER_PATHS)):
        raise QualificationError(f"qualifier path set mismatch: {paths}")
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
            raise QualificationError(f"blob mismatch: {path}")


def expected_lock(source_blobs: dict[str, str]) -> dict[str, Any]:
    return {
        "schema": LOCK_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "parent": PRODUCT_PARENT, "parent_tree": PRODUCT_PARENT_TREE},
        "product_blobs": PRODUCT_BLOBS,
        "qualifier": {"paths": list(QUALIFIER_PATHS), "source_blobs": source_blobs},
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "StructuralOnly",
            "provider_cryptographically_verified": False,
            "registry_authenticated": False,
            "registry_current": False,
            "composition_qualified": False,
            "production_admission": False,
        },
    }


def verify_lock(repo: pathlib.Path) -> str:
    source_blobs = {
        "README.md": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/README.md"),
        "qualify.py": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/qualify.py"),
        "test_qualify.py": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/test_qualify.py"),
    }
    raw = git_bytes(repo, "show", f"HEAD:{QUALIFIER_PREFIX}/lock.json")
    expected = expected_lock(source_blobs)
    if json.loads(raw.decode()) != expected or raw != canonical(expected):
        raise QualificationError("lock mismatch")
    return sha256(raw)


def read_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo = read_blob(repo, "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/Cargo.toml").decode()
    source = read_blob(repo, "mycelix-workspace/mycelix-core/libs/psi-registry-evidence-core/src/lib.rs").decode()
    if 'sha2 = "0.10"' not in cargo or 'serde = { version = "1.0", features = ["derive"] }' not in cargo:
        raise QualificationError("expected dependency surface missing")
    for token in ("holochain", "xenia", "reqwest", "tokio", "ed25519", "ml-dsa"):
        if token.lower() in cargo.lower():
            raise QualificationError(f"provider/runtime dependency present: {token}")
    required = (
        "RegistrySnapshotSubjectV1",
        "RegistryTrustPolicyV1",
        "TrustedProviderVerifierV1",
        "RegistryProviderObservationV1",
        "StructurallyConsistentRegistryObservationV1",
        "provider_cryptographically_verified",
        "registry_authenticated",
        "registry_current",
        "VerifierNotTrustedForProvider",
        "ObservationSubjectMismatch",
        "ObservationSequenceMismatch",
        "REGISTRY_SUBJECT_DOMAIN",
        "REGISTRY_TRUST_POLICY_DOMAIN",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required token absent: {token}")
    if "AuthenticatedRegistrySnapshot" in source:
        raise QualificationError("unauthorized positive type name introduced")
    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")
    return {"committed_test_count": test_count, "probe_set": "psi-002b2aq-v0.1"}


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    (workspace / "Cargo.toml").write_text('[workspace]\nmembers=["psi-registry-evidence-core"]\nresolver="2"\n', encoding="utf-8")
    marker = "/libs/psi-registry-evidence-core/"
    for source_path in PRODUCT_BLOBS:
        destination = workspace / "psi-registry-evidence-core" / source_path.split(marker, 1)[1]
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(read_blob(repo, source_path))
    return workspace


def run_cmd(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
    completed = sh(args, cwd, check=False)
    return {"argv": args, "returncode": completed.returncode, "stdout_sha256": sha256(completed.stdout.encode()), "stderr_sha256": sha256(completed.stderr.encode())}


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
    head, tree = verify_checkout(repo)
    verify_product(repo)
    lock_sha256 = verify_lock(repo)
    probes = static_probes(repo)
    with tempfile.TemporaryDirectory(prefix="psi-002b2aq-") as temp_dir:
        workspace = materialize(repo, pathlib.Path(temp_dir))
        versions = {"rustc": run_cmd(["rustc", "--version"], workspace), "cargo": run_cmd(["cargo", "--version"], workspace)}
        commands = [run_cmd(list(command), workspace) for command in COMMANDS]
        for record in [*versions.values(), *commands]:
            if record["returncode"] != 0:
                raise QualificationError(f"command failed: {record['argv']}")
        lock_path = workspace / "Cargo.lock"
        if not lock_path.exists():
            raise QualificationError("Cargo.lock not produced")
        cargo_lock_sha256 = sha256(lock_path.read_bytes())
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated")
    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product_commit": PRODUCT_COMMIT,
        "qualifier_commit": head,
        "qualifier_tree": tree,
        "lock_sha256": lock_sha256,
        "probes": probes,
        "versions": versions,
        "commands": commands,
        "cargo_lock_sha256": cargo_lock_sha256,
        "authority_scope": "StructuralOnly",
        "source_compiled": True,
        "registered_tests_passed": True,
        "provider_cryptographically_verified": False,
        "registry_authenticated": False,
        "registry_current": False,
        "composition_qualified": False,
        "production_admission": False,
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
        print(f"PSI-002B2AQ FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
