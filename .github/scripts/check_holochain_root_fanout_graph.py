#!/usr/bin/env python3
"""Verify every root-workspace fanout member resolves only the qualified cohort.

The root fanout inventory identifies workspace members whose direct Holochain-family
requirements are inherited from [workspace.dependencies]. This checker traverses each
such member's *locked resolved dependency closure* and rejects mixed protocol-family
generations that compilation alone could otherwise tolerate.
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"
ROOT_MANIFEST = WORKSPACE / "Cargo.toml"
CONTRACT = WORKSPACE / "holochain-cohort.toml"


def load_contract() -> dict:
    return tomllib.loads(CONTRACT.read_text())


def expected_version(name: str, rust: dict[str, str]) -> str | None:
    if name == "hdi":
        return rust["hdi"]
    if name in {"hdk", "hdk_derive"}:
        return rust["hdk"]
    if name == "holochain_client":
        return rust["holochain_client"]
    if name == "holo_hash":
        return rust["holo_hash"]
    if name in {"holochain_serialized_bytes", "holochain_serialized_bytes_derive"}:
        return rust["holochain_serialized_bytes"]
    if name.startswith("holochain_wasmer_"):
        return rust["holochain_wasmer_host"]
    if name == "holochain" or name.startswith("holochain_"):
        return rust["holochain"]
    if name == "kitsune2" or name.startswith("kitsune2_"):
        return rust["kitsune2"]
    if name == "lair_keystore" or name.startswith("lair_keystore_"):
        return rust["lair_keystore"]
    return None


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
    contract = load_contract()
    if contract.get("state") != "aligned":
        raise SystemExit("root resolved-graph qualification requires aligned candidate state")
    if not contract["policy"].get("require_rust_nix_alignment"):
        raise SystemExit("aligned candidate must require Rust/Nix alignment")

    rust = contract["rust"]
    metadata = cargo_metadata()
    packages = {package["id"]: package for package in metadata.get("packages", [])}
    resolve = metadata.get("resolve") or {}
    nodes = {node["id"]: node for node in resolve.get("nodes", [])}

    by_manifest: dict[str, str] = {}
    for package_id, package in packages.items():
        by_manifest[normalize_manifest(package["manifest_path"])] = package_id

    failures: list[str] = []
    evidence: dict[str, dict] = {}

    affected = inventory.get("affected_members", [])
    if len(affected) != inventory.get("affected_member_count"):
        failures.append("inventory affected_member_count does not match affected_members length")

    for item in affected:
        manifest = item["manifest"]
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
            expected = expected_version(name, rust)
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

    print("Every affected root member resolves only the qualified Holochain-family cohort.")


if __name__ == "__main__":
    main()
