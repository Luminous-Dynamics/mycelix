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

PRODUCT_COMMIT = "4cd3bbc47c27a3f11df7b3308ea0888adbf2322e"
PRODUCT_TREE = "a0bb868eb897268ec8bfe60297b223fea48e976a"
PRODUCT_PARENT = "41a26efa89435fbc328bb5ac68b9e971f4b162cd"
PRODUCT_PARENT_TREE = "c551c2a4588512fa3c99c1c82a14f8b2a25aa820"
QUALIFIER_PREFIX = "ci-governance/psi-002aq"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/qualify.py",
    f"{QUALIFIER_PREFIX}/test_qualify.py",
)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/Cargo.toml": "d4d45713f0dd15f6d2ad0558dd24ba5a05ed6c94",
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/README.md": "7587e053534fc871fbb0d0437d153ae9f3d57ae0",
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/src/lib.rs": "2e2bddb57c0b6d5e0ccd1a686a6b5cb0775903cd",
}
CORE_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/privacy-computation-core/Cargo.toml": "fcd5c53418df9d291fab0238dd622e9c1b9a81b4",
    "mycelix-workspace/mycelix-core/libs/privacy-computation-core/src/lib.rs": "3d0908e5e1da437dbc8dec457479475e6916be50",
}
PROFILE_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/Cargo.toml": "040282ab6f4d754a0febb62a8a608f7f83cb2bc2",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/README.md": "5c754601588ea644356e2de953c0d21960661f92",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/fhe.rs": "2b622787a9b0d2b27847826c0673aaa5d7d9734c",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/lib.rs": "2b139217703a4679708ac6af5cf5dd1c4b21cd56",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/mpc.rs": "e6d5d7ec80c039d5b14947de33dcec080cf301e7",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/pir.rs": "0fb1854c26a34d09e69a73b51943abe9dc1147b8",
    "mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles/src/psi.rs": "c638c0dae8c91fc7c30c2cfd822462281a784c4d",
}
EXPECTED_TEST_COUNT = 11
LOCK_SCHEMA = "mycelix.psi.002aq.lock.v0.1"
RECEIPT_SCHEMA = "mycelix.psi.002aq.receipt.v0.1"
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
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise QualificationError("qualifier is not direct child of PSI-002A product")
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
    for path, oid in {**PRODUCT_BLOBS, **CORE_BLOBS, **PROFILE_BLOBS}.items():
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
        "profile_blobs": PROFILE_BLOBS,
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
        },
        "expected_committed_test_count": EXPECTED_TEST_COUNT,
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "ExperimentalExecutionOnly",
            "network_access": False,
            "git_mutation": False,
            "github_mutation": False,
            "voprf_backend_qualified": False,
            "psi_security_established": False,
            "enumeration_resistance_established": False,
            "client_anonymity_established": False,
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


def read_blob(repo: pathlib.Path, path: str) -> bytes:
    return git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{path}")


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    cargo_path = "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/Cargo.toml"
    source_path = "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/src/lib.rs"
    readme_path = "mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/README.md"
    cargo = read_blob(repo, cargo_path).decode("utf-8")
    source = read_blob(repo, source_path).decode("utf-8")
    readme = read_blob(repo, readme_path).decode("utf-8")

    required_cargo = (
        'privacy-computation-core = { path = "../privacy-computation-core" }',
        'privacy-protocol-profiles = { path = "../privacy-protocol-profiles" }',
        'voprf = "=0.5.0"',
        'rand_core = "0.6"',
    )
    for token in required_cargo:
        if token not in cargo:
            raise QualificationError(f"required Cargo binding absent: {token}")
    for token in ("reqwest", "tokio", "holochain", "hc_zome", "std::net", "openfhe", "tfhe"):
        if token.lower() in cargo.lower():
            raise QualificationError(f"forbidden runtime/dependency token in experiment Cargo.toml: {token}")

    required_source = (
        'pub const PROTOCOL_ID: &str = "PSI-002A";',
        'pub const BACKEND_VERSION: &str = "0.5.0";',
        'pub const PEC_PROFILE_SUBJECT: &str = "41a26efa89435fbc328bb5ac68b9e971f4b162cd";',
        'pub const SYNTHETIC_PREFIX: &str = "syn-contact-v1:";',
        "QualificationState::Experimental",
        "VoprfClient::<Suite>::blind",
        "server.blind_evaluate",
        ".finalize(",
        "server.get_public_key()",
        "server.evaluate(&input)",
        "oracle_tag_for_guess",
        "DuplicateCanonicalIdentifier",
        "MAX_SET_SIZE",
        "ProofVerification",
        "psi_security_established",
        "enumeration_resistance_established",
        "client_anonymity_established",
        "production_admission_granted",
        "application_authority_granted",
        "service_domain",
        "session_domain",
        "append_field(&mut out, config.service_domain.as_bytes())",
        "append_field(&mut out, config.session_domain.as_bytes())",
    )
    for token in required_source:
        if token not in source:
            raise QualificationError(f"required experiment source token absent: {token}")

    forbidden_source = (
        "std::net",
        "reqwest::",
        "tokio::net",
        "hdk::",
        "holochain",
        "ProductionAdmitted",
    )
    for token in forbidden_source:
        if token in source:
            raise QualificationError(f"forbidden experiment source token present: {token}")

    if "source exists\n!= source compiles" not in readme:
        raise QualificationError("source/compile nonclaim absent from experiment README")

    test_count = source.count("#[test]")
    if test_count != EXPECTED_TEST_COUNT:
        raise QualificationError(f"test count mismatch: {test_count}")

    return {
        "committed_test_count": test_count,
        "source_probe_set": "psi-002aq-v0.1",
        "voprf_dependency": "0.5.0",
        "synthetic_namespace": "syn-contact-v1:",
    }


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    (workspace / "Cargo.toml").write_text(
        '[workspace]\nmembers=["privacy-computation-core","privacy-protocol-profiles","psi-voprf-experiment"]\nresolver="2"\n',
        encoding="utf-8",
    )
    groups = (
        ("privacy-computation-core", CORE_BLOBS),
        ("privacy-protocol-profiles", PROFILE_BLOBS),
        ("psi-voprf-experiment", PRODUCT_BLOBS),
    )
    for crate_name, files in groups:
        base = workspace / crate_name
        marker = f"/libs/{crate_name}/"
        for source_path in files:
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

    with tempfile.TemporaryDirectory(prefix="psi-002aq-") as temp_dir:
        workspace = materialize(repo, pathlib.Path(temp_dir))
        versions = {
            "rustc": run_cmd(["rustc", "--version"], workspace),
            "cargo": run_cmd(["cargo", "--version"], workspace),
        }
        for record in versions.values():
            require_success(record)
        commands = [run_cmd(list(command), workspace) for command in COMMANDS]
        for record in commands:
            require_success(record)
        cargo_lock = workspace / "Cargo.lock"
        if not cargo_lock.exists():
            raise QualificationError("Cargo.lock not produced")
        cargo_lock_sha256 = sha256(cargo_lock.read_bytes())

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
            "core_blobs": CORE_BLOBS,
            "profile_blobs": PROFILE_BLOBS,
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
        "authority_scope": "ExperimentalExecutionOnly",
        "source_compiled": True,
        "registered_tests_passed": True,
        "voprf_backend_qualified": False,
        "psi_security_established": False,
        "enumeration_resistance_established": False,
        "client_anonymity_established": False,
        "production_admission": False,
        "application_authority": False,
        "proposition": (
            "Exact PSI-002A synthetic VOPRF experiment source compiled and its registered "
            "source corpus passed in the bound offline environment."
        ),
        "nonclaims": [
            "independent RFC 9497 backend qualification",
            "PSI cryptographic security",
            "malicious security",
            "enumeration resistance",
            "client anonymity",
            "registry freshness or authenticity",
            "production admission",
            "application authority",
        ],
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
        print(f"PSI-002AQ FAIL: {exc}")
        return 1
    print(canonical(receipt).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
