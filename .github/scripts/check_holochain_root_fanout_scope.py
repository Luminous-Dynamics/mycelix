#!/usr/bin/env python3
"""Independently verify the canonical-root Holochain reverse-impact scope.

The inventory discovers affected members by computing each workspace member's forward
dependency closure and asking whether it reaches a direct seed. This verifier uses the
dual algorithm: independently derive the changed canonical dependency set from the
committed-vs-materialized Cargo.toml diff, identify direct seeds again, build Cargo's
reverse resolved graph, and require that resulting workspace-member set to equal the
inventory exactly.

It also independently recomputes the impacted guest-WASM set and checks the emitted
manifest lists. Qualification fails closed on any scope disagreement.
"""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
import json
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"
ROOT_MANIFEST = WORKSPACE / "Cargo.toml"
CONTRACT = WORKSPACE / "holochain-cohort.toml"

DEPENDENCY_TABLES = ("dependencies", "dev-dependencies", "build-dependencies")


def load_toml(path: Path) -> dict:
    return tomllib.loads(path.read_text())


def committed_root_manifest() -> dict:
    proc = subprocess.run(
        ["git", "show", "HEAD:mycelix-workspace/Cargo.toml"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise SystemExit("cannot read committed root Cargo.toml:\n" + proc.stderr)
    return tomllib.loads(proc.stdout)


def changed_root_dependencies(baseline: dict, current: dict) -> dict[str, dict]:
    before = baseline["workspace"]["dependencies"]
    after = current["workspace"]["dependencies"]
    changed: dict[str, dict] = {}
    for name in sorted(set(before) | set(after)):
        old = before.get(name)
        new = after.get(name)
        if old == new:
            continue
        if old is None or new is None:
            raise SystemExit(
                f"candidate added/removed canonical workspace dependency {name!r}; "
                "scope verifier permits only in-place cohort changes"
            )
        changed[name] = {"before": old, "after": new}
    return changed


def cargo_metadata() -> dict:
    proc = subprocess.run(
        [
            "cargo",
            "metadata",
            "--locked",
            "--format-version",
            "1",
            "--manifest-path",
            str(ROOT_MANIFEST),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise SystemExit("root cargo metadata --locked failed:\n" + proc.stderr)
    return json.loads(proc.stdout)


def inherited_changed_dependencies(table: dict, changed_names: set[str]) -> set[str]:
    found: set[str] = set()
    for alias, value in table.items():
        if not isinstance(value, dict) or value.get("workspace") is not True:
            continue
        package = value.get("package", alias)
        if package in changed_names:
            found.add(package)
    return found


def direct_changed_dependencies(manifest: dict, changed_names: set[str]) -> set[str]:
    found: set[str] = set()
    for table_name in DEPENDENCY_TABLES:
        table = manifest.get(table_name, {})
        if isinstance(table, dict):
            found.update(inherited_changed_dependencies(table, changed_names))

    targets = manifest.get("target", {})
    if isinstance(targets, dict):
        for target_table in targets.values():
            if not isinstance(target_table, dict):
                continue
            for table_name in DEPENDENCY_TABLES:
                table = target_table.get(table_name, {})
                if isinstance(table, dict):
                    found.update(inherited_changed_dependencies(table, changed_names))
    return found


def forward_closure(start: str, nodes: dict[str, dict]) -> set[str]:
    seen: set[str] = set()
    queue: deque[str] = deque([start])
    while queue:
        package_id = queue.popleft()
        if package_id in seen:
            continue
        seen.add(package_id)
        node = nodes.get(package_id)
        if node is None:
            continue
        for dep in node.get("deps", []):
            dep_id = dep.get("pkg")
            if dep_id and dep_id not in seen:
                queue.append(dep_id)
    return seen


def reverse_closure(seeds: set[str], nodes: dict[str, dict]) -> set[str]:
    reverse: dict[str, set[str]] = defaultdict(set)
    for node_id, node in nodes.items():
        for dep in node.get("deps", []):
            dep_id = dep.get("pkg")
            if dep_id:
                reverse[dep_id].add(node_id)

    seen: set[str] = set()
    queue: deque[str] = deque(sorted(seeds))
    while queue:
        package_id = queue.popleft()
        if package_id in seen:
            continue
        seen.add(package_id)
        for dependent in sorted(reverse.get(package_id, ())):
            if dependent not in seen:
                queue.append(dependent)
    return seen


def repo_manifest(path: str) -> str:
    manifest = Path(path).resolve()
    try:
        relative = manifest.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise SystemExit(f"manifest escaped repository: {manifest}") from exc
    return str(relative)


def read_manifest_list(path: Path) -> list[str]:
    return [line for line in path.read_text().splitlines() if line]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--manifest-list", type=Path, required=True)
    parser.add_argument("--wasm-manifest-list", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    inventory = json.loads(args.inventory.read_text())
    if inventory.get("schema") != 3:
        raise SystemExit(f"expected fanout inventory schema 3, got {inventory.get('schema')!r}")
    if inventory.get("scope_model") != "candidate-diff-seeded-locked-resolved-reverse-impact-closure":
        raise SystemExit(f"unexpected scope model: {inventory.get('scope_model')!r}")

    contract = load_toml(CONTRACT)
    if contract.get("state") != "aligned":
        raise SystemExit("fanout scope verification requires aligned candidate state")
    rust = contract.get("rust", {})

    changed = changed_root_dependencies(committed_root_manifest(), load_toml(ROOT_MANIFEST))
    if not changed:
        raise SystemExit("independent scope verifier observed no canonical dependency changes")
    unexpected = sorted(set(changed) - set(rust))
    if unexpected:
        raise SystemExit(
            "candidate changed canonical dependencies outside the Holochain cohort contract:\n- "
            + "\n- ".join(unexpected)
        )
    if inventory.get("changed_root_workspace_dependencies") != changed:
        raise SystemExit("inventory changed-root dependency census disagrees with independent candidate diff")
    if inventory.get("changed_root_workspace_dependency_count") != len(changed):
        raise SystemExit("inventory changed_root_workspace_dependency_count mismatch")
    changed_names = set(changed)

    metadata = cargo_metadata()
    packages = {package["id"]: package for package in metadata.get("packages", [])}
    member_ids = set(metadata.get("workspace_members", []))
    resolve = metadata.get("resolve") or {}
    nodes = {node["id"]: node for node in resolve.get("nodes", [])}

    direct_ids: set[str] = set()
    crate_types: dict[str, set[str]] = {}
    for package_id in member_ids:
        package = packages.get(package_id)
        if package is None:
            raise SystemExit(f"workspace member missing package metadata: {package_id}")
        manifest = load_toml(Path(package["manifest_path"]))
        if direct_changed_dependencies(manifest, changed_names):
            direct_ids.add(package_id)
        lib = manifest.get("lib", {})
        values = lib.get("crate-type", []) if isinstance(lib, dict) else []
        crate_types[package_id] = set(values) if isinstance(values, list) else set()

    if not direct_ids:
        raise SystemExit("independent scope verifier found no direct changed-pin seeds")

    reverse_impacted = reverse_closure(direct_ids, nodes) & member_ids

    inventory_items = inventory.get("affected_members", [])
    inventory_manifests = {item["manifest"] for item in inventory_items}
    expected_manifests = {
        repo_manifest(packages[package_id]["manifest_path"])
        for package_id in reverse_impacted
    }

    failures: list[str] = []
    if inventory_manifests != expected_manifests:
        missing_inventory = sorted(expected_manifests - inventory_manifests)
        extra_inventory = sorted(inventory_manifests - expected_manifests)
        if missing_inventory:
            failures.append(
                "inventory missed reverse-impacted manifests: " + ", ".join(missing_inventory)
            )
        if extra_inventory:
            failures.append(
                "inventory contains non-impacted manifests: " + ", ".join(extra_inventory)
            )

    expected_direct_manifests = {
        repo_manifest(packages[package_id]["manifest_path"])
        for package_id in direct_ids
    }
    observed_direct_manifests = {
        item["manifest"] for item in inventory_items if item.get("impact_kind") == "direct"
    }
    if observed_direct_manifests != expected_direct_manifests:
        failures.append("inventory direct/transitive classification disagrees with independent seed census")

    observed_transitive = {
        item["manifest"] for item in inventory_items if item.get("impact_kind") == "transitive"
    }
    expected_transitive = expected_manifests - expected_direct_manifests
    if observed_transitive != expected_transitive:
        failures.append("inventory transitive classification disagrees with reverse closure")

    expected_wasm: set[str] = set()
    for package_id in reverse_impacted:
        if "cdylib" not in crate_types.get(package_id, set()):
            continue
        reachable = forward_closure(package_id, nodes)
        names = {
            packages[reachable_id]["name"]
            for reachable_id in reachable
            if reachable_id in packages
        }
        if names & {"hdk", "hdi"}:
            expected_wasm.add(repo_manifest(packages[package_id]["manifest_path"]))

    observed_wasm = {
        item["manifest"] for item in inventory_items if item.get("wasm_surface") is True
    }
    if observed_wasm != expected_wasm:
        failures.append("inventory WASM classification disagrees with independent resolved closure")

    manifest_list = read_manifest_list(args.manifest_list)
    wasm_manifest_list = read_manifest_list(args.wasm_manifest_list)
    if manifest_list != sorted(expected_manifests):
        failures.append("affected-manifests.txt is not the exact sorted reverse-impact set")
    if wasm_manifest_list != sorted(expected_wasm):
        failures.append("wasm-manifests.txt is not the exact sorted impacted guest set")

    if inventory.get("direct_affected_member_count") != len(expected_direct_manifests):
        failures.append("direct_affected_member_count mismatch")
    if inventory.get("transitive_affected_member_count") != len(expected_transitive):
        failures.append("transitive_affected_member_count mismatch")
    if inventory.get("affected_member_count") != len(expected_manifests):
        failures.append("affected_member_count mismatch")
    if inventory.get("wasm_affected_member_count") != len(expected_wasm):
        failures.append("wasm_affected_member_count mismatch")

    proof = {
        "schema": 2,
        "algorithm": "candidate-diff-seeded-reverse-resolved-closure",
        "changed_root_workspace_dependencies": changed,
        "changed_root_workspace_dependency_count": len(changed),
        "direct_seed_count": len(expected_direct_manifests),
        "transitive_affected_count": len(expected_transitive),
        "affected_member_count": len(expected_manifests),
        "wasm_affected_count": len(expected_wasm),
        "direct_seed_manifests": sorted(expected_direct_manifests),
        "transitive_affected_manifests": sorted(expected_transitive),
        "affected_manifests": sorted(expected_manifests),
        "wasm_affected_manifests": sorted(expected_wasm),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(proof, indent=2, sort_keys=True) + "\n")

    if failures:
        raise SystemExit("Root fanout scope verification failed:\n- " + "\n- ".join(failures))

    print(
        f"Independent candidate diff found {len(changed)} changed canonical dependencies. "
        "Root fanout scope OK: "
        f"{len(expected_direct_manifests)} direct + {len(expected_transitive)} transitive = "
        f"{len(expected_manifests)} affected; {len(expected_wasm)} WASM surfaces."
    )


if __name__ == "__main__":
    main()
