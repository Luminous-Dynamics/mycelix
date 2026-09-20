#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import tomllib
from typing import Any

PRODUCT_COMMIT = "2be72da2acfd9903bfca168035c9ee087059f46f"
PRODUCT_TREE = "d5f793cfb99e08ab3412ea7d3e144babbf52479e"
PRODUCT_PARENT = "41a26efa89435fbc328bb5ac68b9e971f4b162cd"
PRODUCT_PARENT_TREE = "c551c2a4588512fa3c99c1c82a14f8b2a25aa820"
PREFIX = "ci-governance/psi-002aq"
QUALIFIER_NAMES = ("README.md", "lock.json", "qualify.py", "test_qualify.py")
QUALIFIER_PATHS = tuple(f"{PREFIX}/{name}" for name in QUALIFIER_NAMES)
PRODUCT_BLOBS = {
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experimental/Cargo.toml": "65f1a5b36e6bf2fc608c8068e4263a29fb5d2f2b",
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experimental/README.md": "17a2efe8fb402ad841115177af0e0fe7b0b64131",
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experimental/BACKEND.lock.json": "701bd1a19a63a9ec7c994c22048713074d663396",
    "mycelix-workspace/mycelix-core/libs/psi-voprf-experimental/src/lib.rs": "03f4dfc93d86ec7c0548a7e7536012b005909543",
}
PROFILE_BLOBS = {
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
EXPECTED_TESTS = 13
VOPRF_VERSION = "0.5.0"
VOPRF_CHECKSUM = "28f59c30c76e2fea54cdece6a054e2662feffa7ab19658a7887524265ee39470"
VOPRF_UPSTREAM = "f0531f0812387cd6be01923b21e2157399a9b295"
RUSTC_RELEASE = "1.96.0"
CARGO_RELEASE = "1.96.0"
LOCK_SCHEMA = "mycelix.psi.002aq.lock.v0.4"
RECEIPT_SCHEMA = "mycelix.psi.002aq.receipt.v0.4"
PYTHON_INVOCATION = ("python3", "-I", "-S", "-B")
COMMANDS = (
    ("cargo", "fmt", "--check", "--all"),
    ("cargo", "generate-lockfile", "--offline"),
    ("cargo", "test", "--offline", "--locked", "--workspace"),
    ("cargo", "clippy", "--offline", "--locked", "--workspace", "--all-targets", "--", "-D", "warnings"),
    ("cargo", "tree", "--offline", "--locked", "-e", "normal,dev"),
)
FORBIDDEN_GIT_ENV = (
    "GIT_DIR",
    "GIT_WORK_TREE",
    "GIT_INDEX_FILE",
    "GIT_OBJECT_DIRECTORY",
    "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    "GIT_COMMON_DIR",
    "GIT_REPLACE_REF_BASE",
    "GIT_CONFIG_GLOBAL",
    "GIT_CONFIG_SYSTEM",
    "GIT_CONFIG_NOSYSTEM",
    "GIT_CONFIG_COUNT",
    "GIT_CONFIG_PARAMETERS",
    "GIT_NAMESPACE",
    "GIT_SHALLOW_FILE",
)
DANGEROUS_LOCAL_CONFIG_KEYS = (
    "core.worktree",
    "extensions.worktreeconfig",
    "core.attributesfile",
    "core.hookspath",
    "core.fsmonitor",
    "core.alternaterefscommand",
    "include.path",
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
    bad = sorted(
        key
        for key, value in source.items()
        if value
        and (
            key in FORBIDDEN_GIT_ENV
            or key.startswith("GIT_CONFIG_KEY_")
            or key.startswith("GIT_CONFIG_VALUE_")
        )
    )
    if bad:
        raise QualificationError(f"forbidden Git environment override(s): {bad}")


def clean_env() -> dict[str, str]:
    env = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
    env.update(
        GIT_NO_REPLACE_OBJECTS="1",
        GIT_CONFIG_NOSYSTEM="1",
        GIT_CONFIG_GLOBAL=os.devnull,
        CARGO_NET_OFFLINE="true",
    )
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


def verify_python_flags(flags: Any, dont_write_bytecode: bool) -> None:
    required = {
        "isolated": 1,
        "ignore_environment": 1,
        "no_user_site": 1,
        "safe_path": True,
        "no_site": 1,
    }
    for name, expected in required.items():
        if getattr(flags, name, None) != expected:
            raise QualificationError(
                f"Python must run isolated via {' '.join(PYTHON_INVOCATION)}; {name} mismatch"
            )
    if not dont_write_bytecode:
        raise QualificationError("Python bytecode writes must be disabled with -B")


def verify_python_runtime() -> dict[str, Any]:
    verify_python_flags(sys.flags, sys.dont_write_bytecode)
    return {
        "version": sys.version,
        "executable": sys.executable,
        "isolated": bool(sys.flags.isolated),
        "ignore_environment": bool(sys.flags.ignore_environment),
        "no_user_site": bool(sys.flags.no_user_site),
        "safe_path": bool(getattr(sys.flags, "safe_path", False)),
        "no_site": bool(sys.flags.no_site),
        "dont_write_bytecode": bool(sys.dont_write_bytecode),
    }


def verify_repo_root(repo: pathlib.Path) -> None:
    actual = pathlib.Path(git(repo, "rev-parse", "--show-toplevel")).resolve()
    if actual != repo.resolve():
        raise QualificationError(f"repository root mismatch: {actual} != {repo.resolve()}")


def local_config_keys(repo: pathlib.Path) -> tuple[str, ...]:
    git_dir = pathlib.Path(git(repo, "rev-parse", "--git-dir"))
    if not git_dir.is_absolute():
        git_dir = (repo / git_dir).resolve()
    config = git_dir / "config"
    if not config.exists():
        return ()
    completed = sh(
        ["git", "config", "--file", str(config), "--no-includes", "--name-only", "--list"],
        repo,
        check=False,
    )
    if completed.returncode != 0:
        raise QualificationError("unable to inspect local Git config")
    return tuple(line.strip().lower() for line in completed.stdout.splitlines() if line.strip())


def reject_dangerous_local_config(repo: pathlib.Path) -> None:
    bad = []
    for key in local_config_keys(repo):
        if key in DANGEROUS_LOCAL_CONFIG_KEYS or (
            key.startswith("includeif.") and key.endswith(".path")
        ):
            bad.append(key)
    if bad:
        raise QualificationError(f"dangerous local Git config key(s): {sorted(bad)}")


def reject_git_indirection(repo: pathlib.Path) -> None:
    verify_repo_root(repo)
    reject_dangerous_local_config(repo)
    git_dir = pathlib.Path(git(repo, "rev-parse", "--git-dir"))
    if not git_dir.is_absolute():
        git_dir = (repo / git_dir).resolve()
    for relative in ("info/grafts", "objects/info/alternates"):
        path = git_dir / relative
        if path.exists() and path.read_bytes().strip():
            raise QualificationError(f"git indirection present: {relative}")
    if git(repo, "for-each-ref", "--format=%(refname)", "refs/replace").strip():
        raise QualificationError("git replace refs present")


def verify_execution_path(
    repo: pathlib.Path,
    script_path: pathlib.Path | None = None,
) -> str:
    canonical_working_path = repo / PREFIX / "qualify.py"
    if canonical_working_path.is_symlink() or not canonical_working_path.is_file():
        raise QualificationError("canonical qualifier must be a regular non-symlink file")
    canonical_path = canonical_working_path.resolve()
    supplied = pathlib.Path(__file__) if script_path is None else script_path
    supplied_path = supplied.resolve()
    if supplied_path != canonical_path:
        raise QualificationError(
            f"executing qualifier path mismatch: {supplied_path} != {canonical_path}"
        )
    return str(canonical_path)


def verify_working_qualifier_files(repo: pathlib.Path) -> dict[str, dict[str, str]]:
    evidence: dict[str, dict[str, str]] = {}
    for name, relative in zip(QUALIFIER_NAMES, QUALIFIER_PATHS, strict=True):
        path = repo / relative
        if path.is_symlink() or not path.is_file():
            raise QualificationError(f"qualifier working file is not a regular file: {relative}")
        working = path.read_bytes()
        committed = git_bytes(repo, "show", f"HEAD:{relative}")
        if working != committed:
            raise QualificationError(f"qualifier working bytes differ from HEAD: {relative}")
        evidence[name] = {
            "blob": git(repo, "rev-parse", f"HEAD:{relative}"),
            "sha256": sha256(working),
        }
    return evidence


def verify_checkout(repo: pathlib.Path) -> tuple[str, str]:
    verify_repo_root(repo)
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout is dirty")
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
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
    for path, oid in {**PRODUCT_BLOBS, **PROFILE_BLOBS, **CORE_BLOBS}.items():
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
        "profile_blobs": PROFILE_BLOBS,
        "core_blobs": CORE_BLOBS,
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
            "execution": {
                "python_argv_prefix": list(PYTHON_INVOCATION),
                "canonical_script_path": f"{PREFIX}/qualify.py",
                "working_files_must_equal_head": True,
                "global_git_config_disabled": True,
                "system_git_config_disabled": True,
            },
        },
        "toolchain": {
            "rustc_release": RUSTC_RELEASE,
            "cargo_release": CARGO_RELEASE,
        },
        "expected_product_test_count": EXPECTED_TESTS,
        "backend": {
            "crate": "voprf",
            "version": VOPRF_VERSION,
            "crates_io_checksum_sha256": VOPRF_CHECKSUM,
            "upstream_commit": VOPRF_UPSTREAM,
        },
        "commands": [list(command) for command in COMMANDS],
        "authority": {
            "scope": "ExperimentalSyntheticOnly",
            "network_access": False,
            "git_mutation": False,
            "github_mutation": False,
            "generic_psi_security": False,
            "enumeration_resistance": False,
            "real_data_admission": False,
            "production_admission": False,
            "application_authority": False,
        },
        "postflight": {
            "checkout_immutable": True,
            "product_reverified": True,
            "qualifier_lock_reverified": True,
            "qualifier_working_bytes_reverified": True,
            "executing_path_reverified": True,
            "static_probes_reverified": True,
        },
    }


def verify_lock(repo: pathlib.Path) -> str:
    source_blobs = {
        name: git(repo, "rev-parse", f"HEAD:{PREFIX}/{name}")
        for name in ("README.md", "qualify.py", "test_qualify.py")
    }
    raw = git_bytes(repo, "show", f"HEAD:{PREFIX}/lock.json")
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


def verify_backend_lock_bytes(raw: bytes) -> dict[str, Any]:
    try:
        lock = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QualificationError(f"invalid backend lock: {exc}") from exc
    backend = lock.get("backend", {})
    required = {
        "crate": "voprf",
        "crate_version": VOPRF_VERSION,
        "crates_io_checksum_sha256": VOPRF_CHECKSUM,
        "repository": "https://github.com/facebook/voprf",
        "tag": "v0.5.0",
        "commit": VOPRF_UPSTREAM,
        "default_features": False,
        "ciphersuite": "ristretto255-SHA512",
        "specification": "RFC 9497",
    }
    for key, value in required.items():
        if backend.get(key) != value:
            raise QualificationError(f"backend lock mismatch: {key}")
    if backend.get("features") != ["ristretto255-ciphersuite"]:
        raise QualificationError("backend feature set mismatch")
    if lock.get("data_profile", {}).get("synthetic_only") is not True:
        raise QualificationError("synthetic-only lock missing")
    for key in (
        "generic_psi_security_established",
        "enumeration_resistance_established",
        "client_anonymity_established",
        "transport_privacy_established",
        "registry_authenticity_established",
        "registry_freshness_established",
        "key_lifecycle_qualified",
        "wire_format_qualified",
        "real_data_admitted",
        "production_admitted",
        "application_authority_granted",
    ):
        if lock.get("authority_ceiling", {}).get(key) is not False:
            raise QualificationError(f"authority ceiling widened: {key}")
    return lock


def static_source_probes(source: str, cargo: str, backend_lock: bytes) -> dict[str, Any]:
    verify_backend_lock_bytes(backend_lock)
    if 'voprf = { version = "=0.5.0"' not in cargo:
        raise QualificationError("voprf version not exact-pinned")
    if (
        'default-features = false' not in cargo
        or 'features = ["ristretto255-ciphersuite"]' not in cargo
    ):
        raise QualificationError("voprf feature profile mismatch")
    if '"danger"' in cargo:
        raise QualificationError("danger feature enabled")
    required = (
        "pub struct SyntheticIdentifier(String);",
        'starts_with("synthetic-contact-")',
        "pub struct BlindedRequest",
        "pub struct PendingQuery",
        "pub fn prepare_query",
        "pub fn evaluate_blinded",
        "request: &BlindedRequest",
        "VoprfClient::<Ristretto255>::blind",
        ".blind_evaluate(",
        ".finalize(",
        "tag_set_commitment",
        "raw_identifier_snapshot_hash_emitted: false",
        "server_request_contains_only_blinded_elements: true",
        "simulate_online_membership_guess",
        "offline_enumeration_resistance_established: false",
        "online_enumeration_abuse_resistance_established: false",
        "real_data_admitted: false",
        "production_admitted: false",
        "application_authority_granted: false",
        VOPRF_UPSTREAM,
    )
    for token in required:
        if token not in source:
            raise QualificationError(f"required source invariant absent: {token}")
    match = re.search(r"pub fn evaluate_blinded[^\{]+\{", source, re.S)
    if not match:
        raise QualificationError("server evaluation signature not found")
    signature = match.group(0)
    if any(token in signature for token in ("SyntheticIdentifier", "PendingQuery", "Vec<u8>")):
        raise QualificationError("server evaluation accepts raw/state material")
    for token in (
        "fn snapshot_commitment(",
        "hasher.update(identifier.0",
        "raw_identifier_snapshot_hash_emitted: true",
        "real_data_admitted: true",
        "production_admitted: true",
        "application_authority_granted: true",
    ):
        if token in source:
            raise QualificationError(f"forbidden source pattern present: {token}")
    tests = source.count("#[test]")
    if tests != EXPECTED_TESTS:
        raise QualificationError(f"product test count mismatch: {tests}")
    return {
        "product_test_count": tests,
        "role_boundary": "client-blind/server-blinded-only/client-finalize",
        "raw_identifier_snapshot_hash_emitted": False,
        "online_enumeration_risk_test_present": True,
    }


def verify_voprf_cargo_lock(raw: bytes) -> dict[str, str]:
    try:
        parsed = tomllib.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
        raise QualificationError(f"invalid Cargo.lock: {exc}") from exc
    matches = [
        package
        for package in parsed.get("package", [])
        if package.get("name") == "voprf" and package.get("version") == VOPRF_VERSION
    ]
    if len(matches) != 1:
        raise QualificationError(
            f"expected exactly one voprf {VOPRF_VERSION} package, got {len(matches)}"
        )
    package = matches[0]
    if package.get("checksum") != VOPRF_CHECKSUM:
        raise QualificationError("voprf crates.io checksum mismatch")
    if not package.get("source", "").startswith("registry+"):
        raise QualificationError("voprf did not resolve from registry")
    return {
        "version": package["version"],
        "source": package["source"],
        "checksum": package["checksum"],
    }


def static_probes(repo: pathlib.Path) -> dict[str, Any]:
    source_path = next(path for path in PRODUCT_BLOBS if path.endswith("src/lib.rs"))
    cargo_path = next(path for path in PRODUCT_BLOBS if path.endswith("Cargo.toml"))
    backend_path = next(path for path in PRODUCT_BLOBS if path.endswith("BACKEND.lock.json"))
    return static_source_probes(
        read_blob(repo, source_path).decode("utf-8"),
        read_blob(repo, cargo_path).decode("utf-8"),
        read_blob(repo, backend_path),
    )


def materialize(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    (workspace / "Cargo.toml").write_text(
        '[workspace]\nmembers=["privacy-computation-core","privacy-protocol-profiles","psi-voprf-experimental"]\nresolver="2"\n',
        encoding="utf-8",
    )
    for prefix, files in (
        ("privacy-computation-core", CORE_BLOBS),
        ("privacy-protocol-profiles", PROFILE_BLOBS),
        ("psi-voprf-experimental", PRODUCT_BLOBS),
    ):
        marker = f"/libs/{prefix}/"
        for source_path in files:
            if marker not in source_path:
                continue
            destination = workspace / prefix / source_path.split(marker, 1)[1]
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(read_blob(repo, source_path))
    return workspace


def run_cmd(args: list[str], cwd: pathlib.Path) -> dict[str, Any]:
    completed = sh(args, cwd, check=False)
    return {
        "argv": args,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "stdout_sha256": sha256(completed.stdout.encode()),
        "stderr_sha256": sha256(completed.stderr.encode()),
    }


def require_success(record: dict[str, Any]) -> None:
    if record["returncode"] != 0:
        raise QualificationError(f"command failed: {record['argv']}")


def verify_toolchain_versions(rustc: dict[str, Any], cargo: dict[str, Any]) -> dict[str, str]:
    require_success(rustc)
    require_success(cargo)
    release_match = re.search(r"^release:\s*(\S+)\s*$", rustc["stdout"], re.M)
    if not release_match or release_match.group(1) != RUSTC_RELEASE:
        observed = release_match.group(1) if release_match else "missing"
        raise QualificationError(
            f"rustc release mismatch: expected {RUSTC_RELEASE}, observed {observed}"
        )
    cargo_match = re.match(r"^cargo\s+(\S+)", cargo["stdout"].strip())
    if not cargo_match or cargo_match.group(1) != CARGO_RELEASE:
        observed = cargo_match.group(1) if cargo_match else "missing"
        raise QualificationError(
            f"cargo release mismatch: expected {CARGO_RELEASE}, observed {observed}"
        )
    return {
        "rustc_release": release_match.group(1),
        "cargo_release": cargo_match.group(1),
    }


def runtime_versions(workspace: pathlib.Path) -> tuple[dict[str, Any], dict[str, str]]:
    rustc = run_cmd(["rustc", "-Vv"], workspace)
    cargo = run_cmd(["cargo", "--version"], workspace)
    verified = verify_toolchain_versions(rustc, cargo)
    evidence = {
        "rustc": {
            "argv": rustc["argv"],
            "returncode": rustc["returncode"],
            "stdout_sha256": rustc["stdout_sha256"],
            "stderr_sha256": rustc["stderr_sha256"],
        },
        "cargo": {
            "argv": cargo["argv"],
            "returncode": cargo["returncode"],
            "stdout_sha256": cargo["stdout_sha256"],
            "stderr_sha256": cargo["stderr_sha256"],
        },
    }
    return evidence, verified


def postflight(
    repo: pathlib.Path,
    qualifier_commit: str,
    qualifier_tree: str,
    lock_sha: str,
    initial_probes: dict[str, Any],
    initial_working_files: dict[str, dict[str, str]],
) -> dict[str, Any]:
    reject_git_indirection(repo)
    verify_execution_path(repo)
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise QualificationError("checkout changed during qualification")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit:
        raise QualificationError("HEAD changed during qualification")
    if git(repo, "rev-parse", "HEAD^{tree}") != qualifier_tree:
        raise QualificationError("qualifier tree changed during qualification")
    final_working_files = verify_working_qualifier_files(repo)
    if final_working_files != initial_working_files:
        raise QualificationError("qualifier working bytes changed during qualification")
    verify_product(repo)
    if verify_lock(repo) != lock_sha:
        raise QualificationError("qualifier lock changed during qualification")
    final_probes = static_probes(repo)
    if final_probes != initial_probes:
        raise QualificationError("static probes changed during qualification")
    return {
        "checkout_immutable": True,
        "executing_path_reverified": True,
        "qualifier_working_bytes_reverified": True,
        "product_reverified": True,
        "qualifier_lock_reverified": True,
        "static_probes_reverified": True,
    }


def qualify(repo: pathlib.Path, receipt_output: pathlib.Path) -> dict[str, Any]:
    reject_git_env_overrides()
    python_runtime = verify_python_runtime()
    repo = repo.resolve()
    receipt_output = receipt_output.resolve()
    try:
        receipt_output.relative_to(repo)
    except ValueError:
        pass
    else:
        raise QualificationError("receipt output must be outside checkout")

    reject_git_indirection(repo)
    verify_execution_path(repo)
    qualifier_commit, qualifier_tree = verify_checkout(repo)
    working_files = verify_working_qualifier_files(repo)
    verify_product(repo)
    lock_sha = verify_lock(repo)
    probes = static_probes(repo)

    with tempfile.TemporaryDirectory(prefix="psi-002aq-") as temp_dir:
        workspace = materialize(repo, pathlib.Path(temp_dir))
        versions, verified_toolchain = runtime_versions(workspace)
        records = []
        for command in COMMANDS:
            record = run_cmd(list(command), workspace)
            require_success(record)
            records.append(
                {
                    "argv": record["argv"],
                    "returncode": record["returncode"],
                    "stdout_sha256": record["stdout_sha256"],
                    "stderr_sha256": record["stderr_sha256"],
                }
            )
            if command[:2] == ("cargo", "generate-lockfile"):
                lock = workspace / "Cargo.lock"
                if not lock.exists():
                    raise QualificationError("Cargo.lock not produced")
                verify_voprf_cargo_lock(lock.read_bytes())
        raw_lock = (workspace / "Cargo.lock").read_bytes()
        resolved = verify_voprf_cargo_lock(raw_lock)

    post = postflight(
        repo,
        qualifier_commit,
        qualifier_tree,
        lock_sha,
        probes,
        working_files,
    )
    receipt: dict[str, Any] = {
        "schema": RECEIPT_SCHEMA,
        "product": {
            "commit": PRODUCT_COMMIT,
            "tree": PRODUCT_TREE,
            "parent": PRODUCT_PARENT,
            "blobs": PRODUCT_BLOBS,
            "profile_blobs": PROFILE_BLOBS,
            "core_blobs": CORE_BLOBS,
        },
        "qualifier": {
            "commit": qualifier_commit,
            "tree": qualifier_tree,
            "lock_sha256": lock_sha,
            "working_files": working_files,
            "executed_path": str((repo / PREFIX / "qualify.py").resolve()),
        },
        "backend": {
            "upstream_commit": VOPRF_UPSTREAM,
            "resolved_registry_package": resolved,
        },
        "python_runtime": python_runtime,
        "toolchain": verified_toolchain,
        "versions": versions,
        "probes": probes,
        "postflight": post,
        "commands": records,
        "cargo_lock_sha256": sha256(raw_lock),
        "qualification": "Experimental",
        "synthetic_only": True,
        "generic_psi_security_established": False,
        "offline_enumeration_resistance_established": False,
        "online_enumeration_abuse_resistance_established": False,
        "client_anonymity_established": False,
        "transport_privacy_established": False,
        "registry_authenticity_established": False,
        "registry_freshness_established": False,
        "key_lifecycle_qualified": False,
        "wire_format_qualified": False,
        "real_data_admitted": False,
        "production_admitted": False,
        "application_authority_granted": False,
        "proposition": (
            "Exact PSI-002A synthetic VOPRF experiment source compiled and its "
            "registered synthetic role-boundary corpus passed under the exact "
            "Rust 1.96.0 offline qualification profile."
        ),
        "nonclaims": [
            "generic PSI security",
            "enumeration resistance",
            "client anonymity",
            "transport privacy",
            "registry authenticity/freshness",
            "operational key lifecycle",
            "network wire-format safety",
            "real-data admission",
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
        print(f"PSI-002AQ FAIL: {exc}", file=sys.stderr)
        return 1
    print("PSI-002AQ PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
