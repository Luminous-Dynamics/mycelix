#!/usr/bin/env python3
"""Audit the canonical Rust source closure without invoking Cargo.

The snapshot intentionally lives inside a larger Luminous Dynamics workspace.
This guard makes those external path dependencies explicit and detects drift.
Use --require-present for an actual build/release lane; the default declaration
mode remains useful on a standalone review snapshot and fails only on undeclared
or stale path edges.
"""

from __future__ import annotations

import argparse
import json
import sys
import tomllib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/canonical-source-closure.json"


@dataclass(frozen=True, order=True)
class PathEdge:
    manifest: str
    name: str
    target: str
    present: bool
    external: bool
    features: tuple[str, ...]
    package: str | None


def relative_to_root(path: Path) -> str:
    resolved = path.resolve(strict=False)
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(Path("..") / resolved.relative_to(ROOT.parent))


def dependency_tables(document: dict[str, Any]) -> Iterable[dict[str, Any]]:
    for key in ("dependencies", "dev-dependencies", "build-dependencies"):
        table = document.get(key)
        if isinstance(table, dict):
            yield table
    targets = document.get("target", {})
    if isinstance(targets, dict):
        for target in targets.values():
            if not isinstance(target, dict):
                continue
            for key in ("dependencies", "dev-dependencies", "build-dependencies"):
                table = target.get(key)
                if isinstance(table, dict):
                    yield table


def load_toml(path: Path) -> dict[str, Any]:
    return tomllib.loads(path.read_text(encoding="utf-8"))


def discover(config: dict[str, Any]) -> list[PathEdge]:
    queue = [ROOT / root for root in config["roots"]]
    visited: set[Path] = set()
    edges: set[PathEdge] = set()

    while queue:
        manifest = queue.pop().resolve(strict=False)
        if manifest in visited:
            continue
        visited.add(manifest)
        if not manifest.is_file():
            raise FileNotFoundError(f"missing canonical manifest: {relative_to_root(manifest)}")
        document = load_toml(manifest)
        manifest_rel = relative_to_root(manifest)

        workspace = document.get("workspace")
        if isinstance(workspace, dict):
            for member in workspace.get("members", []):
                member_manifest = (manifest.parent / member / "Cargo.toml").resolve(strict=False)
                queue.append(member_manifest)

        for table in dependency_tables(document):
            for name, spec in table.items():
                if not isinstance(spec, dict) or not isinstance(spec.get("path"), str):
                    continue
                target = (manifest.parent / spec["path"]).resolve(strict=False)
                try:
                    target.relative_to(ROOT)
                    external = False
                except ValueError:
                    external = True
                target_manifest = target if target.name == "Cargo.toml" else target / "Cargo.toml"
                present = target_manifest.is_file()
                package = None
                if present:
                    target_document = load_toml(target_manifest)
                    package_table = target_document.get("package")
                    if isinstance(package_table, dict) and isinstance(package_table.get("name"), str):
                        package = package_table["name"]
                features = tuple(sorted(str(item) for item in spec.get("features", []) if isinstance(item, str)))
                edge = PathEdge(
                    manifest=manifest_rel,
                    name=name,
                    target=relative_to_root(target),
                    present=present,
                    external=external,
                    features=features,
                    package=package,
                )
                edges.add(edge)
                if not external and target_manifest.is_file():
                    queue.append(target_manifest)
    return sorted(edges)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-present", action="store_true")
    parser.add_argument("--json", action="store_true", dest="json_output")
    args = parser.parse_args()

    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    edges = discover(config)
    external = [edge for edge in edges if edge.external]
    declarations = config["external_path_dependencies"]
    declared = {
        (item["manifest"], item["name"], item["target"])
        for item in declarations
    }
    observed = {(edge.manifest, edge.name, edge.target) for edge in external}
    declaration_by_edge = {
        (item["manifest"], item["name"], item["target"]): item
        for item in declarations
    }

    errors: list[str] = []
    for undeclared in sorted(observed - declared):
        errors.append(f"undeclared external path dependency: {undeclared}")
    for stale in sorted(declared - observed):
        errors.append(f"stale external path declaration: {stale}")
    for edge in external:
        declaration = declaration_by_edge.get((edge.manifest, edge.name, edge.target))
        if declaration is None:
            continue
        expected_features = tuple(sorted(declaration.get("features", [])))
        if edge.features != expected_features:
            errors.append(
                f"external dependency feature drift: {edge.name}: "
                f"observed={list(edge.features)} expected={list(expected_features)}"
            )
        expected_package = declaration.get("package")
        if not isinstance(expected_package, str) or not expected_package:
            errors.append(f"external dependency declaration lacks package identity: {edge.name}")
        elif edge.present and edge.package != expected_package:
            errors.append(
                f"external dependency package mismatch: {edge.name}: "
                f"observed={edge.package!r} expected={expected_package!r}"
            )
    if args.require_present:
        for edge in edges:
            if not edge.present:
                errors.append(
                    f"missing path dependency for release: {edge.manifest} -> {edge.name} ({edge.target})"
                )

    report = {
        "roots": config["roots"],
        "edges": [asdict(edge) for edge in edges],
        "external_declared": len(declared),
        "external_present": sum(edge.present for edge in external),
        "status": "failed" if errors else "passed",
        "errors": errors,
    }
    if args.json_output:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print("Canonical source-closure audit", report["status"])
        print(f"  path edges: {len(edges)}")
        print(f"  external declared: {len(declared)}")
        print(f"  external present: {report['external_present']}/{len(external)}")
        for edge in external:
            state = "present" if edge.present else "missing"
            package = edge.package or "unresolved"
            features = ",".join(edge.features) or "none"
            print(
                f"  - {edge.manifest}: {edge.name} -> {edge.target} "
                f"[{state}; package={package}; features={features}]"
            )
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
