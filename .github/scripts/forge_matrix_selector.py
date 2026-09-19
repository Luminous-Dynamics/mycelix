#!/usr/bin/env python3
"""Deterministic, fail-closed Forge matrix selection oracle.

This tool has no GitHub mutation authority. It consumes one frozen trusted
profile plus changed repository paths and proposes an admitted Forge CI matrix.

The live CLI verifies that the checkout still contains the exact workflow and
Cargo-manifest Git blobs named by the profile before it can propose a subset.
Any source-binding drift fails closed to the full frozen matrix.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Iterable


PROFILE_SCHEMA = "mycelix-forge-matrix-profile-v1"
SELECTION_SCHEMA = "mycelix-forge-matrix-selection-v1"
_SHA1_RE = re.compile(r"^[0-9a-f]{40}$")


class ProfileError(ValueError):
    pass


def _canonical_paths(paths: Iterable[str]) -> list[str]:
    result: set[str] = set()
    for raw in paths:
        if not isinstance(raw, str) or not raw:
            raise ValueError("changed paths must be non-empty strings")
        if "\\" in raw or raw.startswith("/") or any(part == ".." for part in raw.split("/")):
            raise ValueError(f"non-canonical changed path: {raw!r}")
        normalized = raw.removeprefix("./")
        if not normalized or normalized.startswith("/"):
            raise ValueError(f"non-canonical changed path: {raw!r}")
        result.add(normalized)
    return sorted(result)


def _canonical_profile_path(raw: object, *, field: str) -> str:
    if not isinstance(raw, str) or not raw:
        raise ProfileError(f"{field} must be a non-empty string")
    if "\\" in raw or raw.startswith("/") or any(part in ("", ".", "..") for part in raw.split("/")):
        raise ProfileError(f"{field} must be a canonical repository-relative path")
    return raw


def _require_sha1(raw: object, *, field: str) -> str:
    if not isinstance(raw, str) or _SHA1_RE.fullmatch(raw) is None:
        raise ProfileError(f"{field} must be a lowercase 40-hex Git SHA-1")
    return raw


def git_blob_sha(data: bytes, object_format: str = "sha1") -> str:
    """Return the Git blob object ID for exact bytes under the frozen suite."""
    if object_format != "sha1":
        raise ProfileError(f"unsupported Git object format: {object_format!r}")
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def load_profile(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        profile = json.load(handle)
    validate_profile(profile)
    return profile


def validate_profile(profile: dict) -> None:
    if not isinstance(profile, dict):
        raise ProfileError("profile must be an object")
    if profile.get("schema") != PROFILE_SCHEMA:
        raise ProfileError("unsupported profile schema")
    if profile.get("profile_version") != 1:
        raise ProfileError("unsupported profile version")
    if profile.get("git_object_format") != "sha1":
        raise ProfileError("v1 requires Git SHA-1 object identifiers")

    workflow_path = _canonical_profile_path(profile.get("workflow_path"), field="workflow_path")
    if workflow_path != ".github/workflows/forge.yml":
        raise ProfileError("unexpected workflow path")
    _require_sha1(profile.get("workflow_blob_sha"), field="workflow_blob_sha")

    lanes = profile.get("lanes")
    if not isinstance(lanes, list) or not lanes:
        raise ProfileError("lanes must be a non-empty list")

    ids: list[str] = []
    names: set[str] = set()
    roots: set[str] = set()
    lane_by_id: dict[str, dict] = {}

    for lane in lanes:
        if not isinstance(lane, dict):
            raise ProfileError("lane must be an object")
        lane_id = lane.get("id")
        name = lane.get("name")
        root = lane.get("path")
        cargo = lane.get("cargo_dependencies")
        semantic = lane.get("semantic_dependencies")

        if not isinstance(lane_id, str) or not lane_id:
            raise ProfileError("lane id must be non-empty")
        if not isinstance(name, str) or not name:
            raise ProfileError(f"lane {lane_id!r} has invalid name")
        root = _canonical_profile_path(root, field=f"lane {lane_id!r} path")
        _require_sha1(
            lane.get("cargo_manifest_blob_sha"),
            field=f"lane {lane_id!r} cargo_manifest_blob_sha",
        )
        if not isinstance(cargo, list) or not all(isinstance(x, str) and x for x in cargo):
            raise ProfileError(f"lane {lane_id!r} has invalid cargo dependencies")
        if not isinstance(semantic, list) or not all(isinstance(x, str) and x for x in semantic):
            raise ProfileError(f"lane {lane_id!r} has invalid semantic dependencies")

        if lane_id in lane_by_id:
            raise ProfileError(f"duplicate lane id: {lane_id}")
        if name in names:
            raise ProfileError(f"duplicate lane name: {name}")
        if root in roots:
            raise ProfileError(f"duplicate lane path: {root}")

        ids.append(lane_id)
        names.add(name)
        roots.add(root)
        lane_by_id[lane_id] = lane

    known = set(ids)
    for lane_id, lane in lane_by_id.items():
        deps = list(lane["cargo_dependencies"]) + list(lane["semantic_dependencies"])
        if len(deps) != len(set(deps)):
            raise ProfileError(f"duplicate dependency for lane: {lane_id}")
        for dep in deps:
            if dep not in known:
                raise ProfileError(f"unknown dependency {dep!r} for lane {lane_id!r}")
            if dep == lane_id:
                raise ProfileError(f"self dependency for lane: {lane_id}")

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str) -> None:
        if node in visited:
            return
        if node in visiting:
            raise ProfileError("dependency cycle detected")
        visiting.add(node)
        lane = lane_by_id[node]
        for dep in list(lane["cargo_dependencies"]) + list(lane["semantic_dependencies"]):
            visit(dep)
        visiting.remove(node)
        visited.add(node)

    for lane_id in ids:
        visit(lane_id)

    prefixes = profile.get("forge_prefixes")
    if not isinstance(prefixes, list) or not prefixes:
        raise ProfileError("forge_prefixes must be non-empty strings")
    for index, prefix in enumerate(prefixes):
        _canonical_profile_path(prefix.rstrip("/"), field=f"forge_prefixes[{index}]")

    full_paths = profile.get("full_matrix_paths")
    if not isinstance(full_paths, list) or not full_paths:
        raise ProfileError("full_matrix_paths must be non-empty strings")
    for index, path in enumerate(full_paths):
        _canonical_profile_path(path, field=f"full_matrix_paths[{index}]")


def source_binding_errors(repo_root: Path, profile: dict) -> list[str]:
    """Verify the checkout matches every frozen workflow/manifest Git blob."""
    validate_profile(profile)
    root = repo_root.resolve()
    errors: list[str] = []
    object_format = profile["git_object_format"]

    subjects: list[tuple[str, str, str]] = [
        ("workflow", profile["workflow_path"], profile["workflow_blob_sha"])
    ]
    subjects.extend(
        (
            f"lane:{lane['id']}:cargo_manifest",
            f"{lane['path']}/Cargo.toml",
            lane["cargo_manifest_blob_sha"],
        )
        for lane in profile["lanes"]
    )

    for label, relative_path, expected in subjects:
        candidate = root / relative_path
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(root)
            data = resolved.read_bytes()
        except (OSError, ValueError) as exc:
            errors.append(f"{label}:unreadable:{relative_path}:{type(exc).__name__}")
            continue
        actual = git_blob_sha(data, object_format)
        if actual != expected:
            errors.append(f"{label}:blob_mismatch:{relative_path}:{expected}:{actual}")

    return errors


def _reverse_graph(profile: dict) -> dict[str, set[str]]:
    reverse = {lane["id"]: set() for lane in profile["lanes"]}
    for lane in profile["lanes"]:
        for dep in list(lane["cargo_dependencies"]) + list(lane["semantic_dependencies"]):
            reverse[dep].add(lane["id"])
    return reverse


def _lane_for_path(path: str, profile: dict) -> str | None:
    matches = []
    for lane in profile["lanes"]:
        root = lane["path"]
        if path == root or path.startswith(root + "/"):
            matches.append(lane["id"])
    if len(matches) > 1:
        raise ProfileError(f"overlapping lane roots for path {path!r}")
    return matches[0] if matches else None


def _full(
    profile: dict,
    paths: list[str],
    reason: str,
    *,
    binding_errors: list[str] | None = None,
) -> dict:
    result = {
        "schema": SELECTION_SCHEMA,
        "disposition": "full_matrix",
        "reason": reason,
        "changed_paths": paths,
        "seed_lanes": [],
        "selected": [
            {"id": lane["id"], "name": lane["name"], "path": lane["path"]}
            for lane in profile["lanes"]
        ],
    }
    if binding_errors is not None:
        result["binding_errors"] = list(binding_errors)
    return result


def _select_unbound(changed_paths: Iterable[str], profile: dict) -> dict:
    """Pure graph oracle used by tests after profile validation.

    This helper is intentionally not an authority path. Production callers must
    use select_matrix(), which first verifies frozen checkout byte bindings.
    """
    validate_profile(profile)
    try:
        paths = _canonical_paths(changed_paths)
    except ValueError:
        return _full(profile, [], "non_canonical_changed_path")

    full_paths = set(profile["full_matrix_paths"])
    if any(path in full_paths for path in paths):
        return _full(profile, paths, "selector_profile_or_workflow_changed")

    seed_ids: set[str] = set()
    for path in paths:
        lane_id = _lane_for_path(path, profile)
        if lane_id is not None:
            if path.endswith("/Cargo.toml"):
                return _full(profile, paths, "forge_dependency_manifest_changed")
            seed_ids.add(lane_id)
            continue

        if any(path.startswith(prefix) for prefix in profile["forge_prefixes"]):
            return _full(profile, paths, "unknown_forge_path")

    if not seed_ids:
        return {
            "schema": SELECTION_SCHEMA,
            "disposition": "no_forge_change",
            "reason": "no_known_forge_path_changed",
            "changed_paths": paths,
            "seed_lanes": [],
            "selected": [],
        }

    reverse = _reverse_graph(profile)
    selected = set(seed_ids)
    pending = list(seed_ids)
    while pending:
        current = pending.pop()
        for consumer in reverse[current]:
            if consumer not in selected:
                selected.add(consumer)
                pending.append(consumer)

    ordered_ids = [lane["id"] for lane in profile["lanes"] if lane["id"] in selected]
    lane_by_id = {lane["id"]: lane for lane in profile["lanes"]}
    return {
        "schema": SELECTION_SCHEMA,
        "disposition": "selected_matrix",
        "reason": "transitive_reverse_dependency_closure",
        "changed_paths": paths,
        "seed_lanes": [lane["id"] for lane in profile["lanes"] if lane["id"] in seed_ids],
        "selected": [
            {"id": lane_id, "name": lane_by_id[lane_id]["name"], "path": lane_by_id[lane_id]["path"]}
            for lane_id in ordered_ids
        ],
    }


def select_matrix(changed_paths: Iterable[str], profile: dict, repo_root: Path) -> dict:
    """Bound selection path: verify frozen source bytes, then run the graph oracle."""
    validate_profile(profile)
    try:
        paths = _canonical_paths(changed_paths)
    except ValueError:
        return _full(profile, [], "non_canonical_changed_path")

    errors = source_binding_errors(repo_root, profile)
    if errors:
        return _full(
            profile,
            paths,
            "frozen_source_binding_mismatch",
            binding_errors=errors,
        )
    return _select_unbound(paths, profile)


def _default_profile() -> Path:
    return Path(__file__).resolve().parent.parent / "ci" / "forge_matrix_profile_v1.json"


def _repository_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*", help="repository-relative changed paths")
    args = parser.parse_args(argv)

    try:
        profile = load_profile(_default_profile())
        result = select_matrix(args.paths, profile, _repository_root())
    except (OSError, json.JSONDecodeError, ProfileError, ValueError) as exc:
        # A future workflow integration MUST interpret selector refusal as
        # FullMatrix, never as permission to skip lanes.
        print(f"forge matrix selector refused input: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
