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

PRODUCT_COMMIT = "c5c892ab096c5454f8e0bcad87940e0e7ac1a849"
PRODUCT_TREE = "64a9c02021611de3e8352045de98f3202e28d644"
PRODUCT_PARENT = "4cd3bbc47c27a3f11df7b3308ea0888adbf2322e"
PRODUCT_PARENT_TREE = "a0bb868eb897268ec8bfe60297b223fea48e976a"
QUALIFIER_PREFIX = "ci-governance/psi-002b1q"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-contact-discovery-policy/Cargo.toml": "32809e5172d65754cb943d4e06e459b96aaa34c0",
    "mycelix-workspace/mycelix-core/libs/psi-contact-discovery-policy/README.md": "45311548190baf6fc16553b022b0289dae1a9a04",
    "mycelix-workspace/mycelix-core/libs/psi-contact-discovery-policy/src/lib.rs": "e05e5555788f760f8afe39cbbd1fbc7f96a9c130",
}
CORE_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/privacy-computation-core/Cargo.toml": "fcd5c53418df9d291fab0238dd622e9c1b9a81b4",
    "mycelix-workspace/mycelix-core/libs/privacy-computation-core/src/lib.rs": "3d0908e5e1da437dbc8dec457479475e6916be50",
}
EXPECTED_TEST_COUNT = 13
LOCK_SCHEMA = "mycelix.psi.002b1q.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002b1q.receipt.v0.1"
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
        raise QualificationError("qualifier is not direct child of PSI-002B1")
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
        "qualifier": {"paths": list(QUALIFIER_PATHS), "source_blobs": source_blobs},
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "StructuralOnly",
            "query_token_unlinkability_established": False,
            "abuse_resistance_established": False,
            "transport_unlinkability_established": False,
            "registry_authenticity_established": False,
            "registry_currentness_established": False,
            "composition_qualified": False,
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
    expected = expected_lock(source_blobs)
    try:
        actual = json.loads(raw.decode())
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QualificationError(f"invalid lock JSON: {exc}") from exc
    if actual != expected or raw != canonical(expected):
        raise QualificationError("lock contract/bytes mismatch")
    return sha256(raw)


def read_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo_path = "mycelix-workspace/mycelix-core/libs/psi-contact-discovery-policy/Cargo.toml"
    source_path = "mycelix-workspace/mycelix-core/libs/psi-contact-discovery-policy/src/lib.rs"
    readme_path = "mycelix-workspace/mycelix-core/libs/psi-contact-discovery-policy/README.md"
    cargo = read_blob(repo, cargo_path).decode()
    source = read_blob(repo, source_path).decode()
    readme = read_blob(repo, readme_path).decode()

    if 'privacy-computation-core = { path = "../privacy-computation-core" }' not in cargo:
        raise QualificationError("PEC dependency missing")
    for token in ("reqwest", "tokio", "holochain", "privacy-pass", "ohttp", "voprf"):
        if token.lower() in cargo.lower():
            raise QualificationError(f"concrete protocol/runtime dependency present: {token}")

    required = (
        'pub const PSI_002A_SUBJECT: &str = "4cd3bbc47c27a3f11df7b3308ea0888adbf2322e";',
        'pub const POLICY_PROFILE: &str = "psi-contact-discovery-composition-v1";',
        "AuthorizationPresentation::DeclaredUnlinkableToken",
        "ReplayProtection::ScopedNullifier",
        "TransportModel::ObliviousRelay",
        "TrafficAnalysisOverclaim",
        "RegistryKeyEpochMismatch",
        "RegistryAuthenticityRequired",
        "RegistryCurrentnessRequired",
        "PersistentDerivedTagsForbidden",
        "SemanticAuthority::StructuralOnly",
        "abuse_resistance_established",
        "transport_unlinkability_established",
        "registry_authenticity_established",
        "registry_currentness_established",
        "composition_qualified",
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required semantic token absent: {token}")

    if "PrivacyPassRfc9577" in source or "OhttpRfc9458" in source:
        raise QualificationError("standards adapter name baked into semantic enum")
    if "source exists\n!= source compiles" not in readme:
        raise QualificationError("source/compile nonclaim missing")

    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")
    return {"committed_test_count": test_count, "probe_set": "psi-002b1q-v0.1"}


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    (workspace / "Cargo.toml").write_text(
        '[workspace]\nmembers=["privacy-computation-core","psi-contact-discovery-policy"]\nresolver="2"\n',
        encoding="utf-8",
    )
    groups = (
        ("privacy-computation-core", CORE_BLOBS),
        ("psi-contact-discovery-policy", PRODUCT_BLOBS),
    )
    for crate_name, files in groups:
        marker = f"/libs/{crate_name}/"
        for source_path in files:
            if marker not in source_path:
                continue
            destination = workspace / crate_name / source_path.split(marker, 1)[1]
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

    with tempfile.TemporaryDirectory(prefix="psi-002b1q-") as temp_dir:
        workspace = materialize(repo, pathlib.Path(temp_dir))
        versions = {
            "rustc": run_cmd(["rustc", "--version"], workspace),
            "cargo": run_cmd(["cargo", "--version"], workspace),
        }
        for value in versions.values():
            require_success(value)
        commands = [run_cmd(list(command), workspace) for command in COMMANDS]
        for record in commands:
            require_success(record)
        lock_path = workspace / "Cargo.lock"
        if not lock_path.exists():
            raise QualificationError("Cargo.lock not produced")
        cargo_lock_sha256 = sha256(lock_path.read_bytes())

    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout mutated")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit:
        raise QualificationError("HEAD changed")
    if git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier tree changed")

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "product": {"commit": PRODUCT_COMMIT, "tree": PRODUCT_TREE, "parent": PRODUCT_PARENT},
        "qualifier": {"commit": qualifier_commit, "tree": qualifier_tree, "lock_sha256": lock_sha256},
        "probes": probes,
        "versions": versions,
        "commands": commands,
        "cargo_lock_sha256": cargo_lock_sha256,
        "authority_scope": "StructuralOnly",
        "source_compiled": True,
        "registered_tests_passed": True,
        "query_token_unlinkability_established": False,
        "abuse_resistance_established": False,
        "transport_unlinkability_established": False,
        "registry_authenticity_established": False,
        "registry_currentness_established": False,
        "composition_qualified": False,
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
    except (QualificationError, subprocess.CalledProcessError, OSError) as exc:
        print(f"PSI-002B1Q FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
