#!/usr/bin/env python3
"""Inventory Holochain-bearing Cargo workspace authority across an exact checkout.

This is a static structural census, not resolved-graph qualification. It sees every
source Cargo manifest, models explicit/implicit workspace subjects, follows local
path dependencies, and maps reviewed authority anchors to those structural subjects.
Unknown Holochain-bearing subjects remain unclassified until deliberately reviewed.
"""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
import glob
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tomllib

from holochain_release_family import HOLOCHAIN_RELEASE_COUPLED

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_POLICY = ROOT / "mycelix-workspace/holochain-workspace-authority.toml"

# Generated/vendor trees are not source authority subjects. Keep this list small and
# visible: adding an exclusion changes the census theorem and requires review.
SKIP_PARTS = {
    ".git",
    ".direnv",
    "node_modules",
    "target",
    "target-native",
}

FAMILY_EXACT = set(HOLOCHAIN_RELEASE_COUPLED) | {
    "hdk",
    "hdi",
    "hdk_derive",
    "holochain_client",
    "holo_hash",
    "holochain_chc",
    "holochain_serialized_bytes",
    "holochain_serialized_bytes_derive",
    "lair_keystore",
    "kitsune2",
}
FAMILY_PREFIXES = (
    "holochain_",
    "holochain-wasmer-",
    "holochain_wasmer_",
    "kitsune2_",
    "lair_keystore_",
)
INVALID_FAMILY_RE = re.compile(
    r'''(?mx)
    ^\s*(?:
        hdk|hdi|hdk_derive|holo_hash|holochain_client|holochain_chc|
        holochain_serialized_bytes(?:_derive)?|holochain(?:[_-][A-Za-z0-9_-]+)?|
        kitsune2(?:_[A-Za-z0-9_-]+)?|lair_keystore(?:_[A-Za-z0-9_-]+)?
    )\s*=|
    \bpackage\s*=\s*["'](?:
        hdk|hdi|hdk_derive|holo_hash|holochain_client|holochain_chc|
        holochain_serialized_bytes(?:_derive)?|holochain(?:[_-][A-Za-z0-9_-]+)?|
        kitsune2(?:_[A-Za-z0-9_-]+)?|lair_keystore(?:_[A-Za-z0-9_-]+)?
    )["']
    '''
)


def rel(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def is_holochain_family(name: str) -> bool:
    return name in FAMILY_EXACT or name.startswith(FAMILY_PREFIXES)


def manifest_paths() -> list[Path]:
    found = []
    for path in ROOT.rglob("Cargo.toml"):
        relative = path.relative_to(ROOT)
        if any(part in SKIP_PARTS for part in relative.parts):
            continue
        found.append(path.resolve())
    return sorted(found, key=rel)


def dependency_tables(document: dict):
    for table_name in ("dependencies", "dev-dependencies", "build-dependencies"):
        table = document.get(table_name)
        if isinstance(table, dict):
            yield table_name, table

    target = document.get("target")
    if isinstance(target, dict):
        for target_name, target_table in target.items():
            if not isinstance(target_table, dict):
                continue
            for table_name in ("dependencies", "dev-dependencies", "build-dependencies"):
                table = target_table.get(table_name)
                if isinstance(table, dict):
                    yield f"target.{target_name}.{table_name}", table


def dependency_record(manifest: Path, key: str, value) -> dict:
    actual = key
    path = None
    workspace = False
    version = None
    if isinstance(value, str):
        version = value
    elif isinstance(value, dict):
        actual = str(value.get("package", key))
        version = value.get("version")
        workspace = bool(value.get("workspace", False))
        raw_path = value.get("path")
        if raw_path is not None:
            candidate = (manifest.parent / str(raw_path)).resolve()
            if candidate.is_dir():
                candidate = candidate / "Cargo.toml"
            path = candidate
    return {
        "key": key,
        "package": actual,
        "version": version,
        "workspace": workspace,
        "path": path,
    }


def expand_workspace_patterns(root_manifest: Path, patterns: list[str]) -> set[Path]:
    root_dir = root_manifest.parent
    results: set[Path] = set()
    for pattern in patterns:
        for match in glob.glob(str(root_dir / pattern)):
            path = Path(match).resolve()
            if path.is_dir():
                path = path / "Cargo.toml"
            if path.name == "Cargo.toml" and path.is_file():
                results.add(path)
    return results


def git_head() -> str:
    proc = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return proc.stdout.strip()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--allow-unclassified",
        action="store_true",
        help="measurement mode: emit unclassified Holochain subjects without failing",
    )
    args = parser.parse_args()

    policy_path = args.policy.resolve()
    policy = tomllib.loads(policy_path.read_text())
    if policy.get("schema") != 1:
        raise SystemExit(f"unsupported workspace-authority policy schema: {policy.get('schema')!r}")
    if policy.get("state") not in {"measurement-only", "closed"}:
        raise SystemExit(f"unsupported workspace-authority policy state: {policy.get('state')!r}")

    manifests = manifest_paths()
    manifest_set = set(manifests)
    parsed: dict[Path, dict] = {}
    parse_errors: dict[Path, str] = {}
    suspicious_invalid: list[str] = []
    manifest_evidence: dict[str, dict] = {}
    path_edges: dict[Path, set[Path]] = defaultdict(set)
    direct_family: dict[Path, set[str]] = defaultdict(set)

    for manifest in manifests:
        raw = manifest.read_text(errors="replace")
        evidence = {
            "sha256": sha256(manifest),
            "parse_status": "ok",
            "package": None,
            "declares_workspace": False,
            "package_workspace": None,
            "direct_holochain_dependencies": [],
            "path_dependencies": [],
        }
        try:
            document = tomllib.loads(raw)
        except tomllib.TOMLDecodeError as exc:
            parse_errors[manifest] = str(exc)
            evidence["parse_status"] = "invalid"
            evidence["parse_error"] = str(exc)
            if INVALID_FAMILY_RE.search(raw):
                suspicious_invalid.append(rel(manifest))
            manifest_evidence[rel(manifest)] = evidence
            continue

        parsed[manifest] = document
        package = document.get("package")
        if isinstance(package, dict):
            evidence["package"] = package.get("name")
            evidence["package_workspace"] = package.get("workspace")
        evidence["declares_workspace"] = isinstance(document.get("workspace"), dict)

        family_rows = []
        path_rows = []
        for table_name, table in dependency_tables(document):
            for key, value in sorted(table.items()):
                dep = dependency_record(manifest, key, value)
                actual = dep["package"]
                if is_holochain_family(actual):
                    direct_family[manifest].add(actual)
                    family_rows.append(
                        {
                            "table": table_name,
                            "key": key,
                            "package": actual,
                            "version": dep["version"],
                            "workspace": dep["workspace"],
                        }
                    )
                dep_path = dep["path"]
                if dep_path is not None:
                    try:
                        dep_rel = rel(dep_path)
                    except ValueError:
                        dep_rel = str(dep_path)
                    path_rows.append({"table": table_name, "key": key, "manifest": dep_rel})
                    if dep_path in manifest_set:
                        path_edges[manifest].add(dep_path)

        evidence["direct_holochain_dependencies"] = family_rows
        evidence["path_dependencies"] = path_rows
        manifest_evidence[rel(manifest)] = evidence

    explicit_roots = {m for m, d in parsed.items() if isinstance(d.get("workspace"), dict)}
    root_members: dict[Path, set[Path]] = {}
    root_excludes: dict[Path, set[Path]] = {}
    for root in sorted(explicit_roots, key=rel):
        document = parsed[root]
        workspace = document["workspace"]
        members = expand_workspace_patterns(root, list(workspace.get("members", [])))
        excludes = expand_workspace_patterns(root, list(workspace.get("exclude", [])))
        members -= excludes
        if isinstance(document.get("package"), dict):
            members.add(root)
        root_members[root] = {m for m in members if m in parsed}
        root_excludes[root] = excludes

    # Conservatively close explicit roots over local path dependencies that remain
    # below that root and are not excluded/nested workspace roots. Resolved Cargo
    # metadata in a later gate remains authoritative over this static approximation.
    for root in sorted(explicit_roots, key=rel):
        root_dir = root.parent.resolve()
        queue = deque(root_members[root])
        seen = set(root_members[root])
        while queue:
            current = queue.popleft()
            for dep in path_edges.get(current, set()):
                try:
                    dep.relative_to(root_dir)
                except ValueError:
                    continue
                if dep in root_excludes[root] or (dep in explicit_roots and dep != root):
                    continue
                if dep not in seen:
                    seen.add(dep)
                    queue.append(dep)
        root_members[root] = seen

    # Workspace root manifests are authority subjects even when they are virtual and
    # contain no [package]. Treat the root itself as a structural claim so an outer
    # workspace accidentally claiming a nested workspace becomes an explicit conflict.
    claims: dict[Path, set[Path]] = defaultdict(set)
    for root in explicit_roots:
        claims[root].add(root)
    for root, members in root_members.items():
        for member in members:
            claims[member].add(root)

    workspace_conflicts = {
        rel(m): sorted(rel(r) for r in roots)
        for m, roots in claims.items()
        if len(roots) > 1
    }

    subjects: dict[Path, set[Path]] = {root: set(members) for root, members in root_members.items()}
    manifest_subject: dict[Path, Path] = {}
    for manifest, roots in claims.items():
        if len(roots) == 1:
            manifest_subject[manifest] = next(iter(roots))

    # Every parseable package outside an explicit workspace is an implicit singleton
    # Cargo subject. Explicit virtual workspace roots were already mapped above.
    for manifest, document in parsed.items():
        if not isinstance(document.get("package"), dict):
            continue
        if manifest not in manifest_subject:
            subjects.setdefault(manifest, {manifest})
            manifest_subject[manifest] = manifest

    authorities: dict[Path, set[str]] = defaultdict(set)
    anchor_errors: list[str] = []
    bindings_evidence = []
    for binding in policy.get("binding", []):
        authority = binding.get("authority")
        anchor_raw = binding.get("anchor")
        if not isinstance(authority, str) or not isinstance(anchor_raw, str):
            anchor_errors.append(f"invalid authority binding: {binding!r}")
            continue
        anchor = (ROOT / anchor_raw).resolve()
        if anchor not in parsed:
            anchor_errors.append(f"authority anchor missing/unparseable: {anchor_raw}")
            continue
        subject = manifest_subject.get(anchor)
        if subject is None:
            anchor_errors.append(f"authority anchor has no unambiguous Cargo subject: {anchor_raw}")
            continue
        authorities[subject].add(authority)
        bindings_evidence.append(
            {
                "authority": authority,
                "anchor": anchor_raw,
                "workspace_root": rel(subject),
                "reason": binding.get("reason"),
            }
        )

    subject_rows = []
    unclassified = []
    double_authority = []
    holochain_subject_count = 0
    non_holochain_isolated = []

    for subject, members in sorted(subjects.items(), key=lambda item: rel(item[0])):
        queue = deque(members)
        closure = set(members)
        family_names: set[str] = set()
        direct_names: set[str] = set()
        while queue:
            current = queue.popleft()
            current_family = direct_family.get(current, set())
            family_names.update(current_family)
            if current in members:
                direct_names.update(current_family)
            for dep in path_edges.get(current, set()):
                if dep not in closure:
                    closure.add(dep)
                    queue.append(dep)

        auth = sorted(authorities.get(subject, set()))
        if family_names:
            holochain_subject_count += 1
            if len(auth) == 0:
                classification = "unclassified"
                unclassified.append(rel(subject))
            elif len(auth) == 1:
                classification = auth[0]
            else:
                classification = "double-authority"
                double_authority.append({"workspace_root": rel(subject), "authorities": auth})
        else:
            classification = "non_holochain"
            root_workspace = (ROOT / "mycelix-workspace/Cargo.toml").resolve()
            if subject in explicit_roots and subject != root_workspace:
                non_holochain_isolated.append(rel(subject))

        subject_rows.append(
            {
                "workspace_root": rel(subject),
                "explicit_workspace": subject in explicit_roots,
                "member_count": len(members),
                "members": sorted(rel(m) for m in members),
                "local_path_closure_count": len(closure),
                "direct_holochain_families": sorted(direct_names),
                "closure_holochain_families": sorted(family_names),
                "authorities": auth,
                "classification": classification,
            }
        )

    closed = not (
        unclassified
        or double_authority
        or workspace_conflicts
        or suspicious_invalid
        or anchor_errors
    )

    evidence = {
        "schema": 1,
        "authority": "measurement-only" if args.allow_unclassified else "closed-inventory-gate",
        "repository_head": git_head(),
        "policy": rel(policy_path),
        "policy_sha256": sha256(policy_path),
        "inventory_tool_sha256": sha256(Path(__file__).resolve()),
        "manifest_count": len(manifests),
        "parseable_manifest_count": len(parsed),
        "invalid_manifest_count": len(parse_errors),
        "explicit_workspace_root_count": len(explicit_roots),
        "cargo_subject_count": len(subjects),
        "holochain_bearing_subject_count": holochain_subject_count,
        "unclassified_holochain_workspace_count": len(unclassified),
        "duplicate_authority_count": len(double_authority),
        "workspace_conflict_count": len(workspace_conflicts),
        "suspicious_invalid_manifest_count": len(suspicious_invalid),
        "anchor_error_count": len(anchor_errors),
        "closed": closed,
        "bindings": sorted(bindings_evidence, key=lambda row: (row["authority"], row["anchor"])),
        "unclassified_holochain_workspaces": sorted(unclassified),
        "duplicate_authority": double_authority,
        "workspace_conflicts": workspace_conflicts,
        "suspicious_invalid_manifests": sorted(suspicious_invalid),
        "invalid_manifests": {rel(path): error for path, error in sorted(parse_errors.items(), key=lambda item: rel(item[0]))},
        "anchor_errors": anchor_errors,
        "non_holochain_isolated_workspaces": sorted(non_holochain_isolated),
        "subjects": subject_rows,
        "manifests": manifest_evidence,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")

    print(
        "Holochain workspace census: "
        f"manifests={len(manifests)} subjects={len(subjects)} "
        f"holochain={holochain_subject_count} unclassified={len(unclassified)} "
        f"double_authority={len(double_authority)} conflicts={len(workspace_conflicts)} "
        f"closed={closed}"
    )
    for workspace in sorted(unclassified):
        print(f"UNCLASSIFIED {workspace}")

    hard_failures = []
    if double_authority:
        hard_failures.append("one or more Holochain workspaces have multiple authorities")
    if workspace_conflicts:
        hard_failures.append("one or more manifests are structurally claimed by multiple workspaces")
    if suspicious_invalid:
        hard_failures.append("invalid manifests contain dependency-like Holochain-family tokens")
    if anchor_errors:
        hard_failures.append("authority policy contains invalid/unresolved anchors")
    if unclassified and not args.allow_unclassified:
        hard_failures.append("one or more Holochain workspaces are unclassified")

    if hard_failures:
        raise SystemExit("Workspace authority census failed:\n- " + "\n- ".join(hard_failures))

    if unclassified:
        print("Measurement-only census is NOT CLOSED; review unclassified workspaces before qualification.")
    else:
        print("Workspace authority census is structurally closed for this exact checkout.")


if __name__ == "__main__":
    main()
