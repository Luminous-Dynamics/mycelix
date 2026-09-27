#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import tempfile
import tomllib
from typing import Any

PRODUCT_COMMIT = "ea7e0d6c73f0bb775e6dd9f4497f51237749d3fe"
PRODUCT_TREE = "5587844d471ca2640160a13aa04521f766221d0e"
PRODUCT_PARENT = "31713561f743294c3ff0d4b3b33aa0ac62478c34"
PRODUCT_PARENT_TREE = "6b8bcbf3080a1abe8f1f1ab1f8a539ec59184329"

A2P_CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-rfc9578-public-preflight"
B3A_CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core"
B3A1_CRATE = "mycelix-workspace/mycelix-core/libs/psi-privacy-pass-rfc9577-challenge"

PRODUCT_BLOBS = {
    f"{A2P_CRATE}/Cargo.toml": "025f83c1d4dbcaf9f889eb56072392c0a1f5ae31",
    f"{A2P_CRATE}/README.md": "e6cd02d17a10f372754dfa12ed2e32e7c9284700",
    f"{A2P_CRATE}/src/lib.rs": "5162c6be8a96b3f1dd348297f4323ebca209eb90",
    f"{A2P_CRATE}/fixtures/public_go_vector_0.json": "9f0b50ecff497d6912a9f944283a156ffd07c5e8",
    f"{A2P_CRATE}/tests/wrong_key.rs": "3a72f0cf1b7d4495a3bfa269aa3e2cdebf86899d",
}
B3A_BLOBS = {
    f"{B3A_CRATE}/Cargo.toml": "3c40e8d3070a62320768df8030a3e81420da12df",
    f"{B3A_CRATE}/README.md": "24104d6badbd5d61f2a9acccbc99699916cddcaa",
    f"{B3A_CRATE}/src/lib.rs": "484d89b25bf27fd66e5346ab4eeab9e5cc6fe9f2",
}
B3A1_BLOBS = {
    f"{B3A1_CRATE}/Cargo.toml": "b594108d26bc60cca41519a47ba221c798730739",
    f"{B3A1_CRATE}/README.md": "3cde7299a1760f1595d1275073342010b3e0073b",
    f"{B3A1_CRATE}/src/lib.rs": "5d08799814c7af4f3c3b97d6cd3d8da90a139a48",
}

BACKEND_REPOSITORY = "https://github.com/raphaelrobert/privacypass"
BACKEND_COMMIT = "5ff5f57a62877f42313d6600b53e0d4ee4e4e452"
BACKEND_TREE = "014106e30cddf78dafd6cb4ae7c6e20c755e862d"
BACKEND_BLOBS = {
    "Cargo.toml": "3ffe177fb74b01c6cd4286ca7f43fa22005b569d",
    "Cargo.lock": "2e4aada5f0bd813d16938542cf4d7e608afc9401",
    "src/lib.rs": "d4834c8b2bf76cf984bd78b8cdc718a8fb1941db",
    "src/auth/authenticate.rs": "4e1931d7f794c1007f9e9c21577670d2e6b1077b",
    "src/auth/authorize.rs": "78ab70e3406b3ff2e2a9d411455cd52f7af98114",
    "src/public_tokens/mod.rs": "75c19b760163a0273e322b2db2ed5a067bfaf3b8",
    "src/public_tokens/server.rs": "79eca61a911531e2afe2f974c01598b8f152b67f",
    "src/public_tokens/request.rs": "451a9fa51166083ac117094e9f6a14e46217a067",
    "src/public_tokens/response.rs": "aac0b161fba60414d3a541dc34d032fc86434882",
    "tests/kat_vectors/public_go.json": "66de7b7cbde433ae9f41f938095473fe1d4aca43",
}

QUALIFIER_PREFIX = "ci-governance/psi-002b3a2l0"
QUALIFIER_PATHS = (
    f"{QUALIFIER_PREFIX}/README.md",
    f"{QUALIFIER_PREFIX}/lock.json",
    f"{QUALIFIER_PREFIX}/prepare.py",
    f"{QUALIFIER_PREFIX}/test_prepare.py",
)
LOCK_SCHEMA = "mycelix.psi.002b3a2l0.lock.v0.1"
CAPSULE_SCHEMA = "mycelix.psi.002b3a2l.candidate.v0.1"
FORBIDDEN_GIT_ENV = (
    "GIT_DIR",
    "GIT_WORK_TREE",
    "GIT_INDEX_FILE",
    "GIT_OBJECT_DIRECTORY",
    "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    "GIT_REPLACE_REF_BASE",
)


class PreparationError(RuntimeError):
    pass


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical(obj: Any) -> bytes:
    return (json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()


def reject_git_env_overrides(env: dict[str, str] | None = None) -> None:
    source = os.environ if env is None else env
    present = sorted(name for name in FORBIDDEN_GIT_ENV if source.get(name))
    if present:
        raise PreparationError(f"forbidden Git environment override(s): {present}")


def clean_env(extra: dict[str, str] | None = None) -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    if extra:
        env.update(extra)
    return env


def run(
    args: list[str],
    cwd: pathlib.Path,
    *,
    env: dict[str, str] | None = None,
    check: bool = True,
    text: bool = True,
) -> subprocess.CompletedProcess[Any]:
    return subprocess.run(
        args,
        cwd=cwd,
        env=clean_env(env),
        text=text,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def git(repo: pathlib.Path, *args: str) -> str:
    return run(["git", *args], repo).stdout.strip()


def git_bytes(repo: pathlib.Path, *args: str) -> bytes:
    return run(["git", *args], repo, text=False).stdout


def reject_git_indirection(repo: pathlib.Path) -> None:
    git_dir = pathlib.Path(git(repo, "rev-parse", "--git-dir"))
    if not git_dir.is_absolute():
        git_dir = (repo / git_dir).resolve()
    for rel in ("info/grafts", "objects/info/alternates"):
        path = git_dir / rel
        if path.exists() and path.read_bytes().strip():
            raise PreparationError(f"git indirection present: {rel}")
    if git(repo, "for-each-ref", "--format=%(refname)", "refs/replace").strip():
        raise PreparationError("git replace refs present")


def verify_clean_checkout(repo: pathlib.Path) -> None:
    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise PreparationError(f"checkout is dirty: {repo}")


def verify_mycelix_checkout(repo: pathlib.Path) -> tuple[str, str]:
    reject_git_indirection(repo)
    verify_clean_checkout(repo)
    head = git(repo, "rev-parse", "HEAD")
    if git(repo, "rev-parse", "HEAD^") != PRODUCT_COMMIT:
        raise PreparationError("A2PL0 is not a direct child of exact A2P product")
    changed = tuple(sorted(filter(None, git(repo, "diff", "--name-only", PRODUCT_COMMIT, head).splitlines())))
    if changed != tuple(sorted(QUALIFIER_PATHS)):
        raise PreparationError(f"A2PL0 path set mismatch: {changed}")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^{{tree}}") != PRODUCT_TREE:
        raise PreparationError("A2P product tree mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_COMMIT}^") != PRODUCT_PARENT:
        raise PreparationError("A2P product parent mismatch")
    if git(repo, "rev-parse", f"{PRODUCT_PARENT}^{{tree}}") != PRODUCT_PARENT_TREE:
        raise PreparationError("A2P product parent tree mismatch")
    for path, oid in {**PRODUCT_BLOBS, **B3A_BLOBS, **B3A1_BLOBS}.items():
        if git(repo, "rev-parse", f"{PRODUCT_COMMIT}:{path}") != oid:
            raise PreparationError(f"Mycelix source blob mismatch: {path}")
    return head, git(repo, "rev-parse", "HEAD^{tree}")


def verify_backend_checkout(repo: pathlib.Path) -> None:
    reject_git_indirection(repo)
    verify_clean_checkout(repo)
    if git(repo, "rev-parse", "HEAD") != BACKEND_COMMIT:
        raise PreparationError("backend HEAD mismatch")
    if git(repo, "rev-parse", "HEAD^{tree}") != BACKEND_TREE:
        raise PreparationError("backend tree mismatch")
    for path, oid in BACKEND_BLOBS.items():
        if git(repo, "rev-parse", f"HEAD:{path}") != oid:
            raise PreparationError(f"backend source blob mismatch: {path}")


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
        "b3a1_blobs": B3A1_BLOBS,
        "backend": {
            "repository": BACKEND_REPOSITORY,
            "commit": BACKEND_COMMIT,
            "tree": BACKEND_TREE,
            "blobs": BACKEND_BLOBS,
        },
        "qualifier": {
            "paths": list(QUALIFIER_PATHS),
            "source_blobs": source_blobs,
        },
        "authority": {
            "scope": "DependencyCapsulePreparationOnly",
            "dependency_graph_frozen": False,
            "backend_executed": False,
            "token_verified": False,
            "query_credit_granted": False,
            "production_admission": False,
            "application_authority": False,
        },
    }


def verify_self_lock(repo: pathlib.Path) -> str:
    source_blobs = {
        "README.md": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/README.md"),
        "prepare.py": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/prepare.py"),
        "test_prepare.py": git(repo, "rev-parse", f"HEAD:{QUALIFIER_PREFIX}/test_prepare.py"),
    }
    raw = git_bytes(repo, "show", f"HEAD:{QUALIFIER_PREFIX}/lock.json")
    actual = json.loads(raw.decode("utf-8"))
    expected = expected_lock(source_blobs)
    if actual != expected:
        raise PreparationError("A2PL0 self-lock semantic mismatch")
    if raw != canonical(expected):
        raise PreparationError("A2PL0 self-lock is not canonical JSON")
    return sha256(raw)


def materialize_crate(repo: pathlib.Path, product_path: str, destination: pathlib.Path, paths: dict[str, str]) -> None:
    marker = product_path + "/"
    for source_path in paths:
        if not source_path.startswith(marker):
            continue
        relative = source_path[len(marker):]
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(git_bytes(repo, "show", f"{PRODUCT_COMMIT}:{source_path}"))


def materialize_workspace(repo: pathlib.Path, root: pathlib.Path) -> pathlib.Path:
    workspace = root / "workspace"
    workspace.mkdir()
    materialize_crate(repo, B3A_CRATE, workspace / "psi-privacy-pass-credit-core", B3A_BLOBS)
    materialize_crate(repo, B3A1_CRATE, workspace / "psi-privacy-pass-rfc9577-challenge", B3A1_BLOBS)
    materialize_crate(repo, A2P_CRATE, workspace / "psi-privacy-pass-rfc9578-public-preflight", PRODUCT_BLOBS)
    return workspace / "psi-privacy-pass-rfc9578-public-preflight"


def tool_version(args: list[str], cwd: pathlib.Path) -> str:
    return run(args, cwd).stdout.strip()


def package_records(lock_path: pathlib.Path) -> tuple[int, list[dict[str, Any]]]:
    parsed = tomllib.loads(lock_path.read_text(encoding="utf-8"))
    lock_version = int(parsed.get("version", 0))
    packages: list[dict[str, Any]] = []
    for package in parsed.get("package", []):
        packages.append(
            {
                "name": package["name"],
                "version": package["version"],
                "source": package.get("source"),
                "checksum": package.get("checksum"),
                "dependencies": sorted(package.get("dependencies", [])),
            }
        )
    packages.sort(key=lambda row: (row["name"], row["version"], row["source"] or ""))
    return lock_version, packages


def require_exact_backend_resolution(packages: list[dict[str, Any]]) -> str:
    candidates = [p for p in packages if p["name"] == "privacypass" and p["version"] == "0.2.0-pre.3"]
    if len(candidates) != 1:
        raise PreparationError(f"expected one resolved privacypass package, found {len(candidates)}")
    source = candidates[0]["source"] or ""
    if not source.startswith("git+https://github.com/raphaelrobert/privacypass"):
        raise PreparationError(f"unexpected privacypass source: {source}")
    if not source.endswith("#" + BACKEND_COMMIT):
        raise PreparationError(f"privacypass source does not resolve exact commit: {source}")
    return source


def host_from_rustc_verbose(text: str) -> str:
    for line in text.splitlines():
        if line.startswith("host: "):
            return line.removeprefix("host: ").strip()
    raise PreparationError("rustc -vV output does not contain host triple")


def seed_isolated_cargo_home(seed: pathlib.Path | None, destination: pathlib.Path) -> bool:
    if seed is None:
        return False
    seed = seed.resolve()
    if not seed.is_dir():
        raise PreparationError("seed Cargo home is not a directory")
    shutil.copytree(seed, destination, dirs_exist_ok=True, symlinks=True)
    return True


def prepare(
    repo: pathlib.Path,
    backend_repo: pathlib.Path,
    output_dir: pathlib.Path,
    *,
    allow_network: bool,
    seed_cargo_home: pathlib.Path | None,
) -> dict[str, Any]:
    reject_git_env_overrides()
    repo = repo.resolve()
    backend_repo = backend_repo.resolve()
    output_dir = output_dir.resolve()
    for protected in (repo, backend_repo):
        try:
            output_dir.relative_to(protected)
        except ValueError:
            pass
        else:
            raise PreparationError("output directory must be outside both Git checkouts")
    if output_dir.exists():
        raise PreparationError("output directory already exists")

    qualifier_commit, qualifier_tree = verify_mycelix_checkout(repo)
    verify_backend_checkout(backend_repo)
    self_lock_sha256 = verify_self_lock(repo)

    with tempfile.TemporaryDirectory(prefix="psi-002b3a2l0-") as temp:
        manifest_dir = materialize_workspace(repo, pathlib.Path(temp))
        output_dir.mkdir(parents=True)
        cargo_home = output_dir / "cargo-home"
        cargo_home.mkdir()
        seed_used = seed_isolated_cargo_home(seed_cargo_home, cargo_home)

        cargo_env = {"CARGO_HOME": str(cargo_home)}
        if not allow_network:
            cargo_env["CARGO_NET_OFFLINE"] = "true"

        rustc_verbose = tool_version(["rustc", "-vV"], manifest_dir)
        cargo_version = tool_version(["cargo", "--version", "--verbose"], manifest_dir)

        generate = run(["cargo", "generate-lockfile"], manifest_dir, env=cargo_env, check=False)
        if generate.returncode != 0:
            raise PreparationError(f"cargo generate-lockfile failed: {generate.stderr[-4000:]}")
        lock_path = manifest_dir / "Cargo.lock"
        if not lock_path.exists():
            raise PreparationError("Cargo.lock was not generated")

        lock_bytes = lock_path.read_bytes()
        (output_dir / "Cargo.lock.candidate").write_bytes(lock_bytes)
        lock_version, packages = package_records(lock_path)
        backend_source = require_exact_backend_resolution(packages)

        fetch = run(["cargo", "fetch", "--locked"], manifest_dir, env=cargo_env, check=False)
        if fetch.returncode != 0:
            raise PreparationError(f"cargo fetch --locked failed: {fetch.stderr[-4000:]}")

        offline_env = {"CARGO_HOME": str(cargo_home), "CARGO_NET_OFFLINE": "true"}
        offline = run(
            ["cargo", "metadata", "--locked", "--offline", "--format-version", "1", "--no-deps"],
            manifest_dir,
            env=offline_env,
            check=False,
        )
        if offline.returncode != 0:
            raise PreparationError(f"offline locked metadata probe failed: {offline.stderr[-4000:]}")

        capsule: dict[str, Any] = {
            "schema": CAPSULE_SCHEMA,
            "product": {
                "commit": PRODUCT_COMMIT,
                "tree": PRODUCT_TREE,
                "parent": PRODUCT_PARENT,
                "blobs": PRODUCT_BLOBS,
                "b3a_blobs": B3A_BLOBS,
                "b3a1_blobs": B3A1_BLOBS,
            },
            "preparer": {
                "commit": qualifier_commit,
                "tree": qualifier_tree,
                "self_lock_sha256": self_lock_sha256,
            },
            "backend": {
                "repository": BACKEND_REPOSITORY,
                "commit": BACKEND_COMMIT,
                "tree": BACKEND_TREE,
                "blobs": BACKEND_BLOBS,
                "resolved_cargo_source": backend_source,
            },
            "cargo_lock": {
                "sha256": sha256(lock_bytes),
                "version": lock_version,
                "packages": packages,
            },
            "toolchain": {
                "rustc_verbose": rustc_verbose,
                "cargo_version": cargo_version,
                "host": host_from_rustc_verbose(rustc_verbose),
            },
            "acquisition": {
                "network_allowed": allow_network,
                "seed_cargo_home_supplied": seed_used,
                "isolated_cargo_home": True,
                "locked_fetch_succeeded": True,
                "offline_locked_metadata_probe_succeeded": True,
            },
            "authority": {
                "scope": "CandidateDependencyCapsuleOnly",
                "dependency_graph_frozen": False,
                "backend_executed": False,
                "token_verified": False,
                "query_credit_granted": False,
                "production_admission": False,
                "application_authority": False,
            },
        }
        (output_dir / "capsule.json").write_bytes(canonical(capsule))

    if git(repo, "status", "--porcelain", "--untracked-files=all"):
        raise PreparationError("Mycelix checkout mutated during preparation")
    if git(repo, "rev-parse", "HEAD") != qualifier_commit:
        raise PreparationError("Mycelix HEAD changed during preparation")
    if git(backend_repo, "status", "--porcelain", "--untracked-files=all"):
        raise PreparationError("backend checkout mutated during preparation")
    if git(backend_repo, "rev-parse", "HEAD") != BACKEND_COMMIT:
        raise PreparationError("backend HEAD changed during preparation")

    return capsule


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=pathlib.Path, default=pathlib.Path.cwd())
    parser.add_argument("--backend-repo", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument("--seed-cargo-home", type=pathlib.Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--allow-network", action="store_true")
    mode.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    try:
        capsule = prepare(
            args.repo,
            args.backend_repo,
            args.output_dir,
            allow_network=bool(args.allow_network),
            seed_cargo_home=args.seed_cargo_home,
        )
    except (PreparationError, OSError, subprocess.CalledProcessError, json.JSONDecodeError, tomllib.TOMLDecodeError) as exc:
        print(f"PSI-002B3A2L0 FAIL: {exc}")
        return 1
    print(canonical(capsule).decode(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
