#!/usr/bin/env python3
"""Inventory root-workspace packages affected by canonical Holochain pin changes.

This is deliberately a scope theorem, not a compiler theorem. It asks Cargo for the
actual root workspace membership, then inspects each member manifest for Holochain-
family dependencies inherited from [workspace.dependencies]. The result is a stable
machine-readable blast-radius census used by the root fanout qualification workflow.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"
ROOT_MANIFEST = WORKSPACE / "Cargo.toml"
CONTRACT = WORKSPACE / "holochain-cohort.toml"

DEPENDENCY_TABLES = ("dependencies", "dev-dependencies", "build-dependencies")


def load_toml(path: Path) -> dict:
    return tomllib.loads(path.read_text())


def cargo_metadata_no_deps() -> dict:
    proc = subprocess.run(
        [
            "cargo",
            "metadata",
            "--locked",
            "--no-deps",
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
        raise SystemExit(
            "root cargo metadata --locked --no-deps failed:\n" + proc.stderr
        )
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


def domain_for(relative_manifest: Path) -> str:
    parts = relative_manifest.parts
    return parts[0] if parts else "."


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest-list", type=Path)
    args = parser.parse_args()

    contract = load_toml(CONTRACT)
    if contract.get("state") != "aligned":
        raise SystemExit("root fanout inventory requires the materialized aligned candidate")

    rust = contract.get("rust", {})
    tracked = set(rust)
    metadata = cargo_metadata_no_deps()
    member_ids = set(metadata.get("workspace_members", []))
    packages = metadata.get("packages", [])

    git_head = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()

    affected: list[dict] = []
    for package in packages:
        if package.get("id") not in member_ids:
            continue
        manifest_path = Path(package["manifest_path"]).resolve()
        try:
            relative = manifest_path.relative_to(WORKSPACE.resolve())
        except ValueError:
            raise SystemExit(f"workspace member escaped mycelix-workspace: {manifest_path}")

        manifest = load_toml(manifest_path)
        inherited = manifest_inherited_dependencies(manifest, tracked)
        if not inherited:
            continue

        affected.append(
            {
                "package": package["name"],
                "version": package["version"],
                "manifest": str(Path("mycelix-workspace") / relative),
                "domain": domain_for(relative),
                "pulse": relative.parts[0] == "mycelix-pulse" if relative.parts else False,
                "inherited_dependencies": inherited,
            }
        )

    affected.sort(key=lambda item: (item["domain"], item["manifest"], item["package"]))
    non_pulse = [item for item in affected if not item["pulse"]]
    domains = sorted({item["domain"] for item in affected})
    non_pulse_domains = sorted({item["domain"] for item in non_pulse})

    if not affected:
        raise SystemExit(
            "root fanout inventory found no workspace members inheriting tracked Holochain dependencies"
        )
    if not non_pulse:
        raise SystemExit(
            "root fanout inventory unexpectedly found no non-Pulse consumers; scope theorem needs review"
        )

    evidence = {
        "schema": 1,
        "qualified_checkout_head": git_head,
        "root_manifest": "mycelix-workspace/Cargo.toml",
        "contract_state": contract["state"],
        "tracked_workspace_dependencies": {name: rust[name] for name in sorted(tracked)},
        "workspace_member_count": len(member_ids),
        "affected_member_count": len(affected),
        "non_pulse_affected_member_count": len(non_pulse),
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

    print(
        f"Root Holochain fanout: {len(affected)} affected workspace members across "
        f"{len(domains)} domains; {len(non_pulse)} members across "
        f"{len(non_pulse_domains)} non-Pulse domains."
    )
    for domain in non_pulse_domains:
        count = sum(1 for item in non_pulse if item["domain"] == domain)
        print(f"  non-Pulse {domain}: {count}")


if __name__ == "__main__":
    main()
