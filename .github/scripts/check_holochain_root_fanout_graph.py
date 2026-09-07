#!/usr/bin/env python3
"""Verify every root-workspace impact member resolves only the qualified cohort.

The fanout inventory contains the complete candidate-diff-seeded reverse-impact closure:
direct members that inherit actually changed canonical pins plus transitive workspace
consumers whose locked resolved closure reaches those direct seeds. This checker
traverses every affected member's locked dependency closure and rejects mixed protocol-
family generations that compilation alone could otherwise tolerate.
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path
import subprocess
import tomllib

from holochain_release_family import UnclassifiedFamilyPackage, expected_family_version

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"
ROOT_MANIFEST = WORKSPACE / "Cargo.toml"
CONTRACT = WORKSPACE / "holochain-cohort.toml"


def load_contract() -> dict:
    return tomllib.loads(CONTRACT.read_text())


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


def closure(start: str, nodes: dict[str, dict]) -> set[str]:
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


def normalize_manifest(path: str) -> str:
    manifest = Path(path).resolve()
    try:
        relative = manifest.relative_to(ROOT.resolve())
    except ValueError:
        return str(manifest)
    return str(relative)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    inventory = json.loads(args.inventory.read_text())
    failures: list[str] = []
    if inventory.get("schema") != 3:
        failures.append(f"inventory schema {inventory.get('schema')!r} != 3")
    if inventory.get("scope_model") != "candidate-diff-seeded-locked-resolved-reverse-impact-closure":
        failures.append(f"unexpected inventory scope model {inventory.get('scope_model')!r}")
    if not inventory.get("changed_root_workspace_dependencies"):
        failures.append("inventory contains no changed canonical workspace dependency evidence")

    contract = load_contract()
    if contract.get("state") != "aligned":
        raise SystemExit("root resolved-graph qualification requires aligned candidate state")
    if not contract["policy"].get("require_rust_nix_alignment"):
        raise SystemExit("aligned candidate must require Rust/Nix alignment")

    metadata = cargo_metadata()
    packages = {package["id"]: package for package in metadata.get("packages", [])}
    resolve = metadata.get("resolve") or {}
    nodes = {node["id"]: node for node in resolve.get("nodes", [])}

    by_manifest: dict[str, str] = {}
    for package_id, package in packages.items():
        by_manifest[normalize_manifest(package["manifest_path"])] = package_id

    evidence: dict[str, dict] = {}

    affected = inventory.get("affected_members", [])
    if len(affected) != inventory.get("affected_member_count"):
        failures.append("inventory affected_member_count does not match affected_members length")

    direct_count = sum(1 for item in affected if item.get("impact_kind") == "direct")
    transitive_count = sum(1 for item in affected if item.get("impact_kind") == "transitive")
    invalid_kinds = sorted(
        {item.get("impact_kind") for item in affected}
        - {"direct", "transitive"}
    )
    if invalid_kinds:
        failures.append(f"invalid impact kinds in inventory: {invalid_kinds!r}")
    if direct_count != inventory.get("direct_affected_member_count"):
        failures.append("inventory direct_affected_member_count mismatch")
    if transitive_count != inventory.get("transitive_affected_member_count"):
        failures.append("inventory transitive_affected_member_count mismatch")
    if direct_count + transitive_count != len(affected):
        failures.append("inventory direct + transitive counts do not close over affected members")

    seen_manifests: set[str] = set()
    for item in affected:
        manifest = item["manifest"]
        if manifest in seen_manifests:
            failures.append(f"duplicate affected manifest: {manifest}")
            continue
        seen_manifests.add(manifest)

        seeds = item.get("direct_seed_manifests", [])
        if not isinstance(seeds, list) or not seeds:
            failures.append(f"{manifest}: no direct seed provenance")
        if item.get("impact_kind") == "direct" and manifest not in seeds:
            failures.append(f"{manifest}: direct member does not cite itself as a direct seed")
        if item.get("impact_kind") == "transitive" and manifest in seeds:
            failures.append(f"{manifest}: transitive member incorrectly cites itself as a direct seed")

        package_id = by_manifest.get(manifest)
        if package_id is None:
            failures.append(f"{manifest}: affected member missing from locked cargo metadata")
            continue

        reachable = closure(package_id, nodes)
        observed: dict[str, set[str]] = {}
        for reachable_id in reachable:
            package = packages.get(reachable_id)
            if package is None:
                failures.append(f"{manifest}: unresolved metadata package id {reachable_id}")
                continue
            name = package["name"]
            try:
                expected = expected_family_version(name, contract)
            except UnclassifiedFamilyPackage as exc:
                failures.append(f"{manifest}:{exc}")
                continue
            if expected is None:
                continue
            version = package["version"]
            observed.setdefault(name, set()).add(version)
            if version != expected:
                failures.append(
                    f"{manifest}:{name}: resolved {version}, expected cohort {expected}"
                )

        if not observed:
            failures.append(f"{manifest}: dependency closure contains no tracked Holochain-family package")

        evidence[manifest] = {
            "package": item["package"],
            "domain": item["domain"],
            "pulse": item["pulse"],
            "impact_kind": item["impact_kind"],
            "direct_seed_manifests": seeds,
            "wasm_surface": item["wasm_surface"],
            "closure_package_count": len(reachable),
            "tracked_packages": {
                name: sorted(versions) for name, versions in sorted(observed.items())
            },
        }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")

    print(
        f"Resolved root fanout graph census: {len(evidence)} affected member closures, "
        f"{sum(item['closure_package_count'] for item in evidence.values())} total closure observations."
    )

    if failures:
        raise SystemExit(
            "Root fanout resolved Holochain graph qualification failed:\n- "
            + "\n- ".join(failures)
        )

    print("Every direct and transitive root impact member resolves only the qualified Holochain-family cohort.")


if __name__ == "__main__":
    main()
