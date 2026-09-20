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

PRODUCT_COMMIT = "41a26efa89435fbc328bb5ac68b9e971f4b162cd"
PRODUCT_TREE = "c551c2a4588512fa3c99c1c82a14f8b2a25aa820"
PRODUCT_PARENT = "46d3410d0a69aa382b9bbd0e8e30769770314f32"
PRODUCT_PARENT_TREE = "b72753610381547829628c75d5d22a0c7d49fbee"
QUALIFIER_PATHS = (
    "ci-governance/pec-002aq/README.md",
    "ci-governance/pec-002aq/lock.json",
    "ci-governance/pec-002aq/qualify.py",
    "ci-governance/pec-002aq/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/Cargo.toml": "040282ab6f4d754a0febb62a8a608f7f83cb2bc2",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/README.md": "5c754601588ea644356e2de953c0d21960661f92",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/lib.rs": "2b139217703a4679708ac6af5cf5dd1c4b21cd56",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/mpc.rs": "e6d5d7ec80c039d5b14947de33dcec080cf301e7",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/fhe.rs": "2b622787a9b0d2b27847826c0673aaa5d7d9734c",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/psi.rs": "c638c0dae8c91fc7c30c2cfd822462281a784c4d",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/pir.rs": "0fb1854c26a34d09e69a73b51943abe9dc1147b8",
}
CORE_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/privacy-computation-core/Cargo.toml": "fcd5c53418df9d291fab0238dd622e9c1b9a81b4",
    "mycelix-workspace/mycelix-core/libs/privacy-computation-core/src/lib.rs": "3d0908e5e1da437dbc8dec457479475e6916be50",
}
EXPECTED_TEST_COUNT = 37
LOCK_SCHEMA = "mycelix.pec.002aq.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.pec.002aq.receipt.v0.1"
COMMANDS = (
    ("cargo", "fmt", "--check", "--all"),
    ("cargo", "test", "--offline", "--workspace"),
    ("cargo", "clippy", "--offline", "--workspace", "--all-targets", "--", "-D", "warnings"),
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
    return (
        json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n"
    ).encode()


def reject_git_env_overrides(env: dict[str, str] | None = None) -> None:
    source = os.environ if env is None else env
    present = sorted(name for name in FORBIDDEN_GIT_ENV if source.get(name))
    if present:
        raise QualificationError(f"forbidden Git environment override(s): {present}")


def clean_env() -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    return env


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
    parent = git(repo, "rev-parse", "HEAD^")
    if parent != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of product")
    paths = tuple(
        sorted(git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines())
    )
    if paths != tuple(sorted(QUALIFIER_PATHS)):
        raise QualificationError(f"qualifier path set mismatch: {paths}")
    return head, git(repo, "rev-parse", "HEAD^{tree}")


def verify_product(repo: pathlib.Path) -> None:
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^{{tree}}") != PRODUCT_TREE:
        raise QualificationError("product tree mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^") != PRODUCT_PARENT:
        raise QualificationError("product parent mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_PARENT}^{{tree}}") != PRODUCT_PARENT_TREE:
        raise QualificationError("parent tree mismatch")
    for path, oid in {**PRODUCT_BLOBS, **CORE_BLOBS}.items():
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
        "core_blobs": CORE_BLOBS,
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
        },
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "StructuralOnly",
            "network_access": False,
            "git_mutation": False,
            "github_mutation": False,
            "backend_qualification": False,
            "production_admission": False,
            "application_authority": False,
        },
    }


def verify_lock(repo: pathlib.Path) -> str:
    prefix = "ci-governance/pec-002aq"
    source_blobs = {
        "README.md": git(repo, "rev-parse", f"HEAD:{prefix}/README.md"),
        "qualify.py": git(repo, "rev-parse", f"HEAD:{prefix}/qualify.py"),
        "test_qualify.py": git(repo, "rev-parse", f"HEAD:{prefix}/test_qualify.py"),
    }
    raw = git_bytes(repo, "show", f"HEAD:{prefix}/lock.json")
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


def read_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    text = {path: read_blob(repo, path).decode("utf-8") for path in PRODUCT_BLOBS}
    cargo_path = next(path for path in text if path.endswith("Cargo.toml"))
    cargo = text[cargo_path]
    if "privacy-computation-core" not in cargo or "serde" not in cargo:
        raise QualificationError("expected dependencies missing")
    forbidden = (
        "tfhe",
        "openfhe",
        "seal",
        "voprf",
        "mp-spdz",
        "feldman-dkg",
        "reqwest",
        "ring =",
        "openssl",
    )
    low = cargo.lower()
    if any(token in low for token in forbidden):
        raise QualificationError("concrete crypto/network dependency found")
    joined = "\n".join(text.values())
    required = (
        "cryptographic_security_established",
        "production_admission_granted",
        "application_authority_granted",
        "ZeroRoundInteractive",
        "MpcOutputDisclosureUnspecified",
        "MpcNoOutputDisclosureConflict",
        "FheKeyParticipantMismatch",
        "PsiAggregateSpecRequired",
        "PsiInconsistentResultLeakage",
        "PirParticipantCountOverflow",
        "checked_add(1)",
        "aggregate_value",
        "value_domain.trim().is_empty()",
    )
    for token in required:
        if token not in joined:
            raise QualificationError(f"required structural token absent: {token}")
    if "saturating_add(1)" in joined:
        raise QualificationError("saturating PIR topology arithmetic returned")
    if "feldman_dkg" in joined or "feldman-dkg" in joined:
        raise QualificationError("DKG relabeled/imported")
    forbidden_capabilities = (
        'unsafe {',
        'unsafe fn',
        'extern "C"',
        "std::net",
        "std::process",
        "std::fs",
        "tokio::",
    )
    if any(token in joined for token in forbidden_capabilities):
        raise QualificationError("authority-bearing runtime capability found in structural crate")
    test_count = sum(
        source.count("#[test]") for path, source in text.items() if path.endswith(".rs")
    )
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")
    return {
        "committed_test_count": test_count,
        "structural_probe_set": "pec-002aq-v0.1",
    }


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    (workspace / "Cargo.toml").write_text(
        '[workspace]\nmembers=["privacy-computation-core","privacy-protocol-profiles"]\nresolver="2"\n',
        encoding="utf-8",
    )
    for prefix, files in (
        ("privacy-computation-core", CORE_BLOBS),
        ("privacy-protocol-profiles", PRODUCT_BLOBS),
    ):
        base = workspace / prefix
        for source_path in files:
            marker = f"/libs/{prefix}/"
            if marker not in source_path:
                continue
            relative = source_path.split(marker, 1)[1]
            destination = base / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(read_blob(repo, source_path))
    return workspace


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

    with tempfile.TemporaryDirectory(prefix="pec-002aq-") as temp_dir:
        workspace = materialize(repo, pathlib.Path(temp_dir))
        versions = {
            "rustc": run_cmd(["rustc", "--version"], workspace),
            "cargo": run_cmd(["cargo", "--version"], workspace),
        }
        for version in versions.values():
            require_success(version)

        commands = [run_cmd(list(command), workspace) for command in COMMANDS]
        for record in commands:
            require_success(record)

        cargo_lock = workspace / "Cargo.lock"
        if not cargo_lock.exists():
            raise QualificationError("Cargo.lock not produced")
        cargo_lock_sha256 = sha256(cargo_lock.read_bytes())

    receipt: dict[str, Any] = {
        "schema": RECEIPT_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "parent": PRODUCT_PARENT,
            "blobs": PRODUCT_BLOBS,
            "core_blobs": CORE_BLOBS,
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
        "authority_scope": "StructuralOnly",
        "mpc_backend_qualified": False,
        "fhe_backend_qualified": False,
        "psi_backend_qualified": False,
        "pir_backend_qualified": False,
        "cryptographic_security_established": False,
        "production_admission": False,
        "application_authority": False,
        "proposition": (
            "Exact PEC-002A Rust structural profile source compiled and its "
            "registered structural corpus passed in the bound local environment."
        ),
        "nonclaims": [
            "cryptographic backend security",
            "MPC malicious security",
            "FHE parameter adequacy",
            "PSI/VOPRF security",
            "PIR/ORAM security",
            "production admission",
            "application authority",
        ],
    }
    receipt["receipt_commitment_sha256"] = sha256(canonical(receipt))
    receipt_output.parent.mkdir(parents=True, exist_ok=True)
    receipt_output.write_bytes(canonical(receipt))
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".")
    parser.add_argument("--receipt-output", required=True)
    args = parser.parse_args()
    try:
        qualify(pathlib.Path(args.repo), pathlib.Path(args.receipt_output))
    except (QualificationError, subprocess.CalledProcessError, OSError) as exc:
        print(f"PEC-002AQ FAIL: {exc}", file=os.sys.stderr)
        return 1
    print("PEC-002AQ PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
