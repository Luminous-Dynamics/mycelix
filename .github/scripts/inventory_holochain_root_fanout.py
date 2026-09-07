#!/usr/bin/env python3
"""Inventory the complete root-workspace impact of canonical Holochain pin changes.

The direct seeds are root workspace members that inherit one of the exact canonical
Holochain-family pins changed by the 0.6.3 materializer. The actual blast radius is
larger: any root workspace member whose *locked resolved dependency closure* reaches
one of those seeds can observe API/type changes caused by the canonical pin move.

This script therefore records both direct and transitive impact, using Cargo's real
workspace membership and resolve graph rather than directory heuristics.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"
ROOT_MANIFEST = WORKSPACE / "Cargo.toml"
CONTRACT = WORKSPACE / "holochain-cohort.toml"

DEPENDENCY_TABLES = ("dependencies", "dev-dependencies", "build-dependencies")

# Keep this exactly aligned with the canonical-root replacements performed by
# materialize_holochain_0_6_3_pulse.py. Unchanged family members (for example
# serialized-bytes, Kitsune2, or Lair) are not direct seeds merely because they
# are part of the compatibility contract; they are still observed downstream by
# the resolved-graph theorem.
ROOT_PIN_CHANGES = {
    "hdk",
    "hdi",
    "holochain",
    "holochain_client",
    "holochain_types",
    "holochain_zome_types",
    "holo_hash",
    "holochain_integrity_types",
    "holochain_state",
    "holochain_p2p",
    "holochain_keystore",
    "holochain_sqlite",
}


def load_toml(path: Path) -> dict:
    return tomllib.loads(path.read_text())


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


def inherited_dependencies(table: dict, tracked: set[str], scope: str) -> list[dict[str, str]]:
    found: list[dict[str, str]] = []
    for alias, value in sorted(table.items()):
        if not isinstance(value, dict) or value.get("workspace") is not True:
            continue
        package = value.get("package", alias)
        if package not in tracked:
            continue
        found.append({"alias": alias, "package": package, "scope": scope})
    return found


def manifest_inherited_dependencies(manifest: dict, tracked: set[str]) -> list[dict[str, str]]:
    found: list[dict[str, str]] = []
    for table_name in DEPENDENCY_TABLES:
        table = manifest.get(table_name, {})
        if isinstance(table, dict):
            found.extend(inherited_dependencies(table, tracked, table_name))

    targets = manifest.get("target", {})
    if isinstance(targets, dict):
        for target_name, target_table in sorted(targets.items()):
            if not isinstance(target_table, dict):
                continue
            for table_name in DEPENDENCY_TABLES:
                table = target_table.get(table_name, {})
                if isinstance(table, dict):
                    found.extend(
                        inherited_dependencies(
                            table,
                            tracked,
                            f"target.{target_name}.{table_name}",
                        )
                    )
    return found


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


def domain_for(relative_manifest: Path) -> str:
    parts = relative_manifest.parts
    return parts[0] if parts else "."


def normalized_manifest(path: str) -> tuple[Path, str]:
    manifest_path = Path(path).resolve()
    try:
        relative = manifest_path.relative_to(WORKSPACE.resolve())
    except ValueError:
        raise SystemExit(f"workspace member escaped mycelix-workspace: {manifest_path}")
    return relative, str(Path("mycelix-workspace") / relative)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest-list", type=Path)
    parser.add_argument("--wasm-manifest-list", type=Path)
    args = parser.parse_args()

    contract = load_toml(CONTRACT)
    if contract.get("state") != "aligned":
        raise SystemExit("root fanout inventory requires the materialized aligned candidate")

    rust = contract.get("rust", {})
    missing_contract_members = sorted(ROOT_PIN_CHANGES - set(rust))
    if missing_contract_members:
        raise SystemExit(
            "changed root pins missing from cohort contract:\n- "
            + "\n- ".join(missing_contract_members)
        )

    metadata = cargo_metadata()
    member_ids = set(metadata.get("workspace_members", []))
    packages = {package["id"]: package for package in metadata.get("packages", [])}
    resolve = metadata.get("resolve") or {}
    nodes = {node["id"]: node for node in resolve.get("nodes", [])}

    missing_nodes = sorted(member_ids - set(nodes))
    if missing_nodes:
        raise SystemExit(
            "root cargo metadata resolve graph is missing workspace members:\n- "
            + "\n- ".join(missing_nodes)
        )

    git_head = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()

    member_info: dict[str, dict] = {}
    direct_ids: set[str] = set()

    for package_id in sorted(member_ids):
        package = packages.get(package_id)
        if package is None:
            raise SystemExit(f"workspace member missing package metadata: {package_id}")

        relative, manifest_string = normalized_manifest(package["manifest_path"])
        manifest = load_toml(Path(package["manifest_path"]))
        inherited = manifest_inherited_dependencies(manifest, ROOT_PIN_CHANGES)
        if inherited:
            direct_ids.add(package_id)

        lib = manifest.get("lib", {})
        crate_types = lib.get("crate-type", []) if isinstance(lib, dict) else []
        member_info[package_id] = {
            "package": package["name"],
            "version": package["version"],
            "manifest": manifest_string,
            "domain": domain_for(relative),
            "pulse": relative.parts[0] == "mycelix-pulse" if relative.parts else False,
            "crate_types": sorted(crate_types) if isinstance(crate_types, list) else [],
            "inherited_changed_dependencies": inherited,
        }

    if not direct_ids:
        raise SystemExit(
            "root fanout inventory found no direct workspace members inheriting changed Holochain pins"
        )

    direct_manifest_by_id = {
        package_id: member_info[package_id]["manifest"] for package_id in direct_ids
    }

    affected: list[dict] = []
    for package_id in sorted(member_ids):
        reachable = closure(package_id, nodes)
        reached_direct_ids = sorted(reachable & direct_ids)
        if not reached_direct_ids:
            continue

        info = dict(member_info[package_id])
        direct = package_id in direct_ids
        reachable_names = {
            packages[reachable_id]["name"]
            for reachable_id in reachable
            if reachable_id in packages
        }
        wasm_surface = (
            "cdylib" in info["crate_types"]
            and bool(reachable_names & {"hdk", "hdi"})
        )

        info.update(
            {
                "impact_kind": "direct" if direct else "transitive",
                "wasm_surface": wasm_surface,
                "dependency_closure_package_count": len(reachable),
                "direct_seed_manifests": sorted(
                    direct_manifest_by_id[seed_id] for seed_id in reached_direct_ids
                ),
            }
        )
        affected.append(info)

    affected.sort(key=lambda item: (item["domain"], item["manifest"], item["package"]))
    direct = [item for item in affected if item["impact_kind"] == "direct"]
    transitive = [item for item in affected if item["impact_kind"] == "transitive"]
    non_pulse = [item for item in affected if not item["pulse"]]
    wasm = [item for item in affected if item["wasm_surface"]]
    domains = sorted({item["domain"] for item in affected})
    non_pulse_domains = sorted({item["domain"] for item in non_pulse})

    if len(direct) != len(direct_ids):
        raise SystemExit(
            f"direct impact cardinality mismatch: affected={len(direct)} seeds={len(direct_ids)}"
        )
    if not non_pulse:
        raise SystemExit(
            "root fanout inventory unexpectedly found no non-Pulse consumers; scope theorem needs review"
        )

    evidence = {
        "schema": 2,
        "qualified_checkout_head": git_head,
        "root_manifest": "mycelix-workspace/Cargo.toml",
        "contract_state": contract["state"],
        "scope_model": "locked-resolved-reverse-impact-closure",
        "changed_root_workspace_dependencies": {
            name: rust[name] for name in sorted(ROOT_PIN_CHANGES)
        },
        "workspace_member_count": len(member_ids),
        "direct_affected_member_count": len(direct),
        "transitive_affected_member_count": len(transitive),
        "affected_member_count": len(affected),
        "non_pulse_affected_member_count": len(non_pulse),
        "wasm_affected_member_count": len(wasm),
        "affected_domains": domains,
        "non_pulse_affected_domains": non_pulse_domains,
        "affected_members": affected,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")

    if args.manifest_list:
        args.manifest_list.parent.mkdir(parents=True, exist_ok=True)
        args.manifest_list.write_text(
            "".join(item["manifest"] + "\n" for item in affected)
        )

    if args.wasm_manifest_list:
        args.wasm_manifest_list.parent.mkdir(parents=True, exist_ok=True)
        args.wasm_manifest_list.write_text(
            "".join(item["manifest"] + "\n" for item in wasm)
        )

    print(
        f"Root Holochain fanout: {len(direct)} direct + {len(transitive)} transitive = "
        f"{len(affected)} affected workspace members across {len(domains)} domains; "
        f"{len(non_pulse)} non-Pulse members across {len(non_pulse_domains)} domains; "
        f"{len(wasm)} WASM surfaces."
    )
    for domain in non_pulse_domains:
        direct_count = sum(
            1 for item in non_pulse
            if item["domain"] == domain and item["impact_kind"] == "direct"
        )
        transitive_count = sum(
            1 for item in non_pulse
            if item["domain"] == domain and item["impact_kind"] == "transitive"
        )
        print(f"  non-Pulse {domain}: {direct_count} direct, {transitive_count} transitive")


if __name__ == "__main__":
    main()
