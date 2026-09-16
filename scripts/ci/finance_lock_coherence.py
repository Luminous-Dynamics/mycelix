#!/usr/bin/env python3
"""Static Cargo workspace manifest <-> lock structural coherence verifier.

This tool deliberately does not resolve dependencies or rewrite Cargo.lock.
It checks whether the checked-in lock structurally represents the checked-in
workspace manifests closely enough to justify proceeding to `cargo --locked`.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


DEP_TABLES = ("dependencies", "dev-dependencies", "build-dependencies")


@dataclass(frozen=True)
class DependencySpec:
    alias: str
    package: str
    source_kind: str  # path | registry | git
    source_value: str | None
    owner_manifest: str
    table: str


@dataclass(frozen=True)
class Problem:
    kind: str
    manifest: str
    package: str
    dependency: str | None
    detail: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "manifest": self.manifest,
            "package": self.package,
            "dependency": self.dependency,
            "detail": self.detail,
        }


def load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as f:
        return tomllib.load(f)


def iter_dep_tables(manifest: dict[str, Any]) -> Iterable[tuple[str, dict[str, Any]]]:
    for name in DEP_TABLES:
        table = manifest.get(name)
        if isinstance(table, dict):
            yield name, table

    target = manifest.get("target")
    if not isinstance(target, dict):
        return

    for target_name, target_cfg in target.items():
        if not isinstance(target_cfg, dict):
            continue
        for dep_name in DEP_TABLES:
            table = target_cfg.get(dep_name)
            if isinstance(table, dict):
                yield f"target.{target_name}.{dep_name}", table


def effective_dep_spec(
    alias: str,
    raw: Any,
    workspace_deps: dict[str, Any],
    owner_manifest: str,
    table: str,
) -> DependencySpec:
    if isinstance(raw, dict) and raw.get("workspace") is True:
        inherited = workspace_deps.get(alias)
        if inherited is None:
            raise ValueError(
                f"{owner_manifest}: {table}.{alias} uses workspace=true "
                "but workspace.dependencies has no matching entry"
            )
        # Package rename can be specified at the member site even when the
        # dependency source/version comes from workspace.dependencies.
        merged: Any
        if isinstance(inherited, dict):
            merged = dict(inherited)
            for key, value in raw.items():
                if key != "workspace":
                    merged[key] = value
        elif isinstance(inherited, str):
            overrides = {key: value for key, value in raw.items() if key != "workspace"}
            merged = {"version": inherited, **overrides} if overrides else inherited
        else:
            merged = inherited
        raw = merged

    if isinstance(raw, str):
        return DependencySpec(
            alias=alias,
            package=alias,
            source_kind="registry",
            source_value=None,
            owner_manifest=owner_manifest,
            table=table,
        )

    if not isinstance(raw, dict):
        raise ValueError(f"{owner_manifest}: unsupported dependency form for {table}.{alias}")

    package = str(raw.get("package", alias))
    if "path" in raw:
        return DependencySpec(
            alias, package, "path", str(raw["path"]), owner_manifest, table
        )
    if "git" in raw:
        return DependencySpec(
            alias, package, "git", str(raw["git"]), owner_manifest, table
        )
    return DependencySpec(
        alias, package, "registry", None, owner_manifest, table
    )


LOCK_DEP_RE = re.compile(r"^(?P<name>\S+)(?: (?P<version>\S+)(?: \((?P<source>.+)\))?)?$")


@dataclass(frozen=True)
class LockDepRef:
    name: str
    version: str | None
    source: str | None


def parse_lock_dep_ref(dep: str) -> LockDepRef | None:
    match = LOCK_DEP_RE.match(dep)
    if match is None:
        return None
    return LockDepRef(
        name=match.group("name"),
        version=match.group("version"),
        source=match.group("source"),
    )


def select_lock_candidates(
    owner_deps: list[Any],
    package_name: str,
    lock_by_name: dict[str, list[dict[str, Any]]],
) -> tuple[list[LockDepRef], list[dict[str, Any]]]:
    refs: list[LockDepRef] = []
    selected: list[dict[str, Any]] = []
    candidates = lock_by_name.get(package_name, [])

    for raw_ref in owner_deps:
        if not isinstance(raw_ref, str):
            continue
        ref = parse_lock_dep_ref(raw_ref)
        if ref is None or ref.name != package_name:
            continue
        refs.append(ref)
        for candidate in candidates:
            if ref.version is not None and str(candidate.get("version")) != ref.version:
                continue
            if ref.source is not None and candidate.get("source") != ref.source:
                continue
            selected.append(candidate)

    # Deduplicate by object identity represented as stable package tuple.
    unique: dict[tuple[Any, Any, Any, Any], dict[str, Any]] = {}
    for entry in selected:
        key = (
            entry.get("name"),
            entry.get("version"),
            entry.get("source"),
            entry.get("checksum"),
        )
        unique[key] = entry
    return refs, list(unique.values())


def registry_source(source: Any) -> bool:
    return isinstance(source, str) and source.startswith("registry+")


def git_source(source: Any) -> bool:
    return isinstance(source, str) and source.startswith("git+")


def path_source(source: Any) -> bool:
    return source is None


def resolve_member_paths(workspace_manifest_path: Path, workspace: dict[str, Any]) -> list[Path]:
    root = workspace_manifest_path.parent
    members = workspace.get("workspace", {}).get("members", [])
    if not isinstance(members, list):
        raise ValueError("workspace.members must be a list")

    out: list[Path] = []
    for member in members:
        if not isinstance(member, str):
            raise ValueError("workspace.members entries must be strings")
        if any(ch in member for ch in "*?["):
            raise ValueError(
                f"globbed workspace member {member!r} is unsupported by this "
                "network-independent verifier; expand it or extend the verifier explicitly"
            )
        manifest = (root / member / "Cargo.toml").resolve()
        out.append(manifest)
    return out


def verify(workspace_manifest_path: Path, lock_path: Path) -> dict[str, Any]:
    workspace_manifest_path = workspace_manifest_path.resolve()
    lock_path = lock_path.resolve()
    workspace = load_toml(workspace_manifest_path)
    lock = load_toml(lock_path)

    workspace_deps = workspace.get("workspace", {}).get("dependencies", {})
    if not isinstance(workspace_deps, dict):
        workspace_deps = {}

    lock_packages = lock.get("package", [])
    if not isinstance(lock_packages, list):
        raise ValueError("Cargo.lock package must be a list")

    lock_by_name: dict[str, list[dict[str, Any]]] = {}
    for entry in lock_packages:
        if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
            continue
        lock_by_name.setdefault(entry["name"], []).append(entry)

    problems: list[Problem] = []
    member_manifests = resolve_member_paths(workspace_manifest_path, workspace)
    member_names: set[str] = set()
    direct_edges_checked = 0

    for manifest_path in member_manifests:
        rel_manifest = str(manifest_path.relative_to(workspace_manifest_path.parent))
        if not manifest_path.exists():
            problems.append(
                Problem(
                    "MissingWorkspaceManifest",
                    rel_manifest,
                    "<unknown>",
                    None,
                    "workspace member Cargo.toml does not exist",
                )
            )
            continue

        manifest = load_toml(manifest_path)
        package_table = manifest.get("package", {})
        owner = package_table.get("name") if isinstance(package_table, dict) else None
        owner_version = package_table.get("version") if isinstance(package_table, dict) else None
        if not isinstance(owner, str) or not owner:
            problems.append(
                Problem(
                    "MissingPackageName",
                    rel_manifest,
                    "<unknown>",
                    None,
                    "workspace member has no [package].name",
                )
            )
            continue
        member_names.add(owner)

        owner_candidates = [
            e
            for e in lock_by_name.get(owner, [])
            if path_source(e.get("source"))
            and (
                not isinstance(owner_version, str)
                or str(e.get("version")) == owner_version
            )
        ]
        if len(owner_candidates) == 0:
            problems.append(
                Problem(
                    "MissingWorkspacePackage",
                    rel_manifest,
                    owner,
                    None,
                    "no source-less/local package stanza with this workspace package name exists in Cargo.lock",
                )
            )
            owner_lock = None
        elif len(owner_candidates) > 1:
            problems.append(
                Problem(
                    "AmbiguousWorkspacePackage",
                    rel_manifest,
                    owner,
                    None,
                    f"{len(owner_candidates)} source-less/local lock entries share this package name",
                )
            )
            owner_lock = None
        else:
            owner_lock = owner_candidates[0]

        for table_name, dep_table in iter_dep_tables(manifest):
            for alias, raw in dep_table.items():
                direct_edges_checked += 1
                try:
                    spec = effective_dep_spec(
                        alias, raw, workspace_deps, rel_manifest, table_name
                    )
                except ValueError as exc:
                    problems.append(
                        Problem(
                            "InvalidDependencySpec",
                            rel_manifest,
                            owner,
                            alias,
                            str(exc),
                        )
                    )
                    continue

                candidates = lock_by_name.get(spec.package, [])
                if not candidates:
                    problems.append(
                        Problem(
                            "MissingDependencyPackage",
                            rel_manifest,
                            owner,
                            spec.package,
                            "declared direct dependency has no package stanza in Cargo.lock",
                        )
                    )
                    continue

                selected_candidates: list[dict[str, Any]] = []
                if owner_lock is not None:
                    owner_deps = owner_lock.get("dependencies", [])
                    if not isinstance(owner_deps, list):
                        owner_deps = []
                    refs, selected_candidates = select_lock_candidates(
                        owner_deps, spec.package, lock_by_name
                    )
                    if not refs:
                        problems.append(
                            Problem(
                                "MissingDirectDependencyEdge",
                                rel_manifest,
                                owner,
                                spec.package,
                                f"{table_name}.{alias} declares {spec.package!r}, "
                                "but the owner lock stanza has no matching direct dependency edge",
                            )
                        )
                    elif not selected_candidates:
                        problems.append(
                            Problem(
                                "UnresolvableDirectDependencyEdge",
                                rel_manifest,
                                owner,
                                spec.package,
                                "owner lock dependency reference does not resolve to a matching package stanza",
                            )
                        )
                    elif len(selected_candidates) > 1:
                        # A name-only edge may legitimately be unambiguous to Cargo only
                        # when there is one candidate. Multiple candidates mean this
                        # static verifier cannot prove which source identity is selected.
                        if any(ref.version is None and ref.source is None for ref in refs):
                            problems.append(
                                Problem(
                                    "AmbiguousDirectDependencyEdge",
                                    rel_manifest,
                                    owner,
                                    spec.package,
                                    "owner lock dependency reference is not specific enough to prove source identity",
                                )
                            )
                else:
                    # Owner package itself is incoherent; source checks still use all
                    # candidates only to report additional useful diagnostics.
                    selected_candidates = candidates

                source_candidates = selected_candidates if selected_candidates else candidates

                if spec.source_kind == "path":
                    if not any(path_source(e.get("source")) for e in source_candidates):
                        problems.append(
                            Problem(
                                "PathSourceIdentityMismatch",
                                rel_manifest,
                                owner,
                                spec.package,
                                "selected lock dependency is not source-less/local as required by the path dependency",
                            )
                        )
                elif spec.source_kind == "registry":
                    registry_candidates = [
                        e for e in source_candidates if registry_source(e.get("source"))
                    ]
                    if not registry_candidates:
                        problems.append(
                            Problem(
                                "RegistrySourceIdentityMismatch",
                                rel_manifest,
                                owner,
                                spec.package,
                                "selected lock dependency is not registry-sourced",
                            )
                        )
                    elif any(
                        isinstance(e.get("source"), str)
                        and "crates.io" in e["source"]
                        and not isinstance(e.get("checksum"), str)
                        for e in registry_candidates
                    ):
                        problems.append(
                            Problem(
                                "MissingRegistryChecksum",
                                rel_manifest,
                                owner,
                                spec.package,
                                "selected crates.io registry package is missing checksum",
                            )
                        )
                elif spec.source_kind == "git":
                    matching = [
                        e
                        for e in source_candidates
                        if git_source(e.get("source"))
                        and spec.source_value is not None
                        and spec.source_value in str(e.get("source"))
                    ]
                    if not matching:
                        problems.append(
                            Problem(
                                "GitSourceIdentityMismatch",
                                rel_manifest,
                                owner,
                                spec.package,
                                f"selected git dependency has no lock source compatible with {spec.source_value!r}",
                            )
                        )

    kind_counts: dict[str, int] = {}
    for p in problems:
        kind_counts[p.kind] = kind_counts.get(p.kind, 0) + 1

    return {
        "workspace_manifest": str(workspace_manifest_path),
        "lockfile": str(lock_path),
        "workspace_members_checked": len(member_manifests),
        "workspace_package_names_seen": len(member_names),
        "manifest_direct_edges_checked": direct_edges_checked,
        "problem_count": len(problems),
        "problem_counts": dict(sorted(kind_counts.items())),
        "problems": [p.as_dict() for p in problems],
        "status": "PASS" if not problems else "FAIL",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace-manifest",
        type=Path,
        default=Path("mycelix-finance/Cargo.toml"),
    )
    parser.add_argument(
        "--lockfile",
        type=Path,
        default=Path("mycelix-finance/Cargo.lock"),
    )
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    try:
        receipt = verify(args.workspace_manifest, args.lockfile)
    except (OSError, ValueError, tomllib.TOMLDecodeError) as exc:
        print(f"finance-lock-coherence: fatal: {exc}", file=sys.stderr)
        return 2

    if args.as_json:
        print(json.dumps(receipt, sort_keys=True, indent=2))
    else:
        print(
            "finance-lock-coherence: "
            f"members={receipt['workspace_members_checked']} "
            f"direct_edges={receipt['manifest_direct_edges_checked']} "
            f"problems={receipt['problem_count']} "
            f"status={receipt['status']}"
        )
        for problem in receipt["problems"]:
            dep = (
                f" dependency={problem['dependency']}"
                if problem["dependency"] is not None
                else ""
            )
            print(
                f"- {problem['kind']}: manifest={problem['manifest']} "
                f"package={problem['package']}{dep}: {problem['detail']}"
            )

    return 0 if receipt["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
