#!/usr/bin/env python3
"""Measurement-only Holochain 0.6 -> 0.7 migration census.

This tool scans the exact checked-out repository tree. It does not edit manifests,
resolve Cargo graphs, or claim that any discovered match requires a specific fix.
It produces deterministic JSON evidence for review by MYC-HOLO-007A/007B.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tomllib
from typing import Any, Iterable

SCHEMA = "mycelix.holochain07-migration-census.v1"
TARGET_PROFILE = {
    "holochain": "0.7.0",
    "hdk": "0.7.0",
    "hdi": "0.8.0",
    "holochain_client": "0.9.0",
    "kitsune2": "0.5.0",
    "lair_keystore": "0.7.1",
}

SKIP_DIR_NAMES = {
    ".git",
    ".direnv",
    ".devenv",
    "target",
    "node_modules",
    "dist",
    "result",
}

HOLOCHAIN_DEPENDENCY_NAMES = {
    "hdk",
    "hdi",
    "hdk_derive",
    "holochain",
    "holochain_client",
    "holochain_types",
    "holochain_zome_types",
    "holochain_integrity_types",
    "holochain_serialized_bytes",
    "holo_hash",
    "holochain_keystore",
    "holochain_nonce",
    "holochain_trace",
    "holochain_util",
    "lair_keystore",
    "kitsune2",
}

RUST_PATTERNS: dict[str, re.Pattern[str]] = {
    "flatop_store_entry": re.compile(r"\bFlatOp::StoreEntry\b"),
    "flatop_store_record": re.compile(r"\bFlatOp::StoreRecord\b"),
    "flatop_register_update": re.compile(r"\bFlatOp::RegisterUpdate\b"),
    "flatop_register_delete": re.compile(r"\bFlatOp::RegisterDelete\b"),
    "flatop_register_create_link": re.compile(r"\bFlatOp::RegisterCreateLink\b"),
    "flatop_register_delete_link": re.compile(r"\bFlatOp::RegisterDeleteLink\b"),
    "flatop_register_agent_activity": re.compile(r"\bFlatOp::RegisterAgentActivity\b"),
    "entry_creation_action": re.compile(r"\bEntryCreationAction\b"),
    "old_action_create_match": re.compile(r"\bAction::Create\s*\("),
    "old_action_update_match": re.compile(r"\bAction::Update\s*\("),
    "old_action_delete_match": re.compile(r"\bAction::Delete\s*\("),
    "old_action_create_link_match": re.compile(r"\bAction::CreateLink\s*\("),
    "old_action_delete_link_match": re.compile(r"\bAction::DeleteLink\s*\("),
    "signal_action_callback": re.compile(r"\bsignal_action\b"),
    "get_agent_activity": re.compile(r"\bget_agent_activity\s*\("),
    "agent_activity_type": re.compile(r"\bAgentActivity\b"),
}

CONFIG_PATTERNS: dict[str, re.Pattern[str]] = {
    "signal_url": re.compile(r"(^|\s)signal_url\s*:"),
    "webrtc_config": re.compile(r"(^|\s)webrtc_config\s*:"),
    "chc_url": re.compile(r"(^|\s)chc_url\s*:"),
    "db_sync_strategy": re.compile(r"(^|\s)db_sync_strategy\s*:"),
}

CARGO_TEXT_PATTERNS: dict[str, re.Pattern[str]] = {
    "feature_sqlite_encrypted": re.compile(r"[\"']sqlite-encrypted[\"']"),
    "feature_wasmer_sys": re.compile(r"[\"']wasmer_sys[\"']"),
    "feature_transport_iroh": re.compile(r"[\"']transport-iroh[\"']"),
}

JS_PACKAGES = {
    "@holochain/client",
    "@holochain/tryorama",
    "@holochain-open-dev/tryorama",
    "@holochain/hc-spin",
}

CANARY_FILES = [
    "mycelix-workspace/mycelix-commons/zomes/mesh-time/integrity/src/lib.rs",
    "mycelix-workspace/mycelix-commons/zomes/mesh-time/integrity/Cargo.toml",
    "mycelix-workspace/mycelix-commons/zomes/mesh-time/coordinator/src/lib.rs",
    "mycelix-workspace/mycelix-commons/zomes/mesh-time/coordinator/Cargo.toml",
]


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def git(*args: str, cwd: Path) -> str:
    return subprocess.check_output(["git", *args], cwd=cwd, text=True).strip()


def sha256_file(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def walk_files(root: Path) -> Iterable[Path]:
    for current, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIR_NAMES)
        current_path = Path(current)
        for name in sorted(files):
            yield current_path / name


def rel(root: Path, path: Path) -> str:
    return path.relative_to(root).as_posix()


def dependency_tables(document: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    tables: list[tuple[str, dict[str, Any]]] = []
    for name in ("dependencies", "dev-dependencies", "build-dependencies"):
        table = document.get(name)
        if isinstance(table, dict):
            tables.append((name, table))

    workspace = document.get("workspace")
    if isinstance(workspace, dict):
        for name in ("dependencies",):
            table = workspace.get(name)
            if isinstance(table, dict):
                tables.append((f"workspace.{name}", table))

    targets = document.get("target")
    if isinstance(targets, dict):
        for target_name, target_table in targets.items():
            if not isinstance(target_table, dict):
                continue
            for name in ("dependencies", "dev-dependencies", "build-dependencies"):
                table = target_table.get(name)
                if isinstance(table, dict):
                    tables.append((f"target.{target_name}.{name}", table))
    return tables


def is_holochain_family(name: str) -> bool:
    return (
        name in HOLOCHAIN_DEPENDENCY_NAMES
        or name.startswith("holochain_")
        or name.startswith("kitsune2")
        or name.startswith("lair_")
    )


def normalize_dependency(value: Any) -> dict[str, Any]:
    if isinstance(value, str):
        return {"version": value}
    if isinstance(value, dict):
        keep = (
            "version",
            "workspace",
            "path",
            "git",
            "rev",
            "branch",
            "tag",
            "package",
            "default-features",
            "features",
        )
        return {key: value[key] for key in keep if key in value}
    return {"raw_type": type(value).__name__}


def scan_cargo(root: Path, path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    dependencies: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    try:
        document = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, tomllib.TOMLDecodeError) as error:
        return [], [{"path": rel(root, path), "error": str(error)}]

    package = document.get("package")
    package_name = package.get("name") if isinstance(package, dict) else None
    for table_name, table in dependency_tables(document):
        for dependency_name, value in sorted(table.items()):
            normalized_name = dependency_name
            if isinstance(value, dict) and isinstance(value.get("package"), str):
                normalized_name = value["package"]
            if not is_holochain_family(normalized_name):
                continue
            dependencies.append(
                {
                    "manifest": rel(root, path),
                    "package": package_name,
                    "table": table_name,
                    "dependency_key": dependency_name,
                    "package_name": normalized_name,
                    "spec": normalize_dependency(value),
                }
            )
    return dependencies, errors


def line_matches(
    root: Path,
    path: Path,
    patterns: dict[str, re.Pattern[str]],
) -> list[dict[str, Any]]:
    matches: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError):
        return matches
    for line_number, line in enumerate(lines, start=1):
        for pattern_name, pattern in patterns.items():
            if pattern.search(line):
                matches.append(
                    {
                        "pattern": pattern_name,
                        "path": rel(root, path),
                        "line": line_number,
                        "text": line.strip()[:240],
                    }
                )
    return matches


def scan_package_json(root: Path, path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    pins: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        return [], [{"path": rel(root, path), "error": str(error)}]
    for table_name in ("dependencies", "devDependencies", "peerDependencies", "optionalDependencies"):
        table = document.get(table_name)
        if not isinstance(table, dict):
            continue
        for name in sorted(JS_PACKAGES):
            if name in table:
                pins.append(
                    {
                        "manifest": rel(root, path),
                        "table": table_name,
                        "package": name,
                        "version": table[name],
                    }
                )
    return pins, errors


def canary_snapshot(root: Path) -> list[dict[str, Any]]:
    snapshot: list[dict[str, Any]] = []
    for relative in CANARY_FILES:
        path = root / relative
        if not path.is_file():
            raise SystemExit(f"required 007B canary file missing: {relative}")
        snapshot.append(
            {
                "path": relative,
                "git_blob": git("hash-object", relative, cwd=root),
                "sha256": sha256_file(path),
            }
        )
    return snapshot


def pattern_counts(matches: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for match in matches:
        counts[match["pattern"]] = counts.get(match["pattern"], 0) + 1
    return dict(sorted(counts.items()))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="holochain07-migration-census.json")
    args = parser.parse_args()

    root = repository_root()
    script = Path(__file__).resolve()
    head = git("rev-parse", "HEAD", cwd=root)

    cargo_dependencies: list[dict[str, Any]] = []
    cargo_errors: list[dict[str, Any]] = []
    rust_matches: list[dict[str, Any]] = []
    config_matches: list[dict[str, Any]] = []
    cargo_text_matches: list[dict[str, Any]] = []
    js_pins: list[dict[str, Any]] = []
    json_errors: list[dict[str, Any]] = []
    manifest_count = 0
    rust_file_count = 0

    for path in walk_files(root):
        if path.name == "Cargo.toml":
            manifest_count += 1
            deps, errors = scan_cargo(root, path)
            cargo_dependencies.extend(deps)
            cargo_errors.extend(errors)
            cargo_text_matches.extend(line_matches(root, path, CARGO_TEXT_PATTERNS))
        elif path.suffix == ".rs":
            rust_file_count += 1
            rust_matches.extend(line_matches(root, path, RUST_PATTERNS))
        elif path.suffix in {".yaml", ".yml"}:
            config_matches.extend(line_matches(root, path, CONFIG_PATTERNS))
        elif path.name == "package.json":
            pins, errors = scan_package_json(root, path)
            js_pins.extend(pins)
            json_errors.extend(errors)

    cargo_dependencies.sort(key=lambda item: (item["manifest"], item["table"], item["dependency_key"]))
    rust_matches.sort(key=lambda item: (item["path"], item["line"], item["pattern"]))
    config_matches.sort(key=lambda item: (item["path"], item["line"], item["pattern"]))
    cargo_text_matches.sort(key=lambda item: (item["path"], item["line"], item["pattern"]))
    js_pins.sort(key=lambda item: (item["manifest"], item["table"], item["package"]))

    evidence: dict[str, Any] = {
        "schema": SCHEMA,
        "evidence_authority": "measurement-only",
        "product_materialization_authorized": False,
        "repository_head": head,
        "tool_path": rel(root, script),
        "tool_sha256": sha256_file(script),
        "candidate_target_profile": TARGET_PROFILE,
        "canary": {
            "name": "mesh-time",
            "files": canary_snapshot(root),
        },
        "summary": {
            "cargo_manifest_count": manifest_count,
            "rust_file_count": rust_file_count,
            "holochain_dependency_observation_count": len(cargo_dependencies),
            "legacy_rust_pattern_match_count": len(rust_matches),
            "legacy_rust_pattern_counts": pattern_counts(rust_matches),
            "conductor_config_match_count": len(config_matches),
            "cargo_feature_match_count": len(cargo_text_matches),
            "javascript_pin_count": len(js_pins),
            "cargo_parse_error_count": len(cargo_errors),
            "package_json_parse_error_count": len(json_errors),
        },
        "holochain_dependencies": cargo_dependencies,
        "legacy_rust_patterns": rust_matches,
        "conductor_config_patterns": config_matches,
        "cargo_feature_patterns": cargo_text_matches,
        "javascript_pins": js_pins,
        "parse_errors": {
            "cargo": cargo_errors,
            "package_json": json_errors,
        },
        "nonclaims": [
            "not a resolved Cargo dependency graph",
            "not proof that every textual match requires a source change",
            "not Holochain 0.7 compatibility qualification",
            "not product migration authority",
            "not deployment or state-continuity evidence",
        ],
    }

    output = root / args.output
    output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(evidence["summary"], indent=2, sort_keys=True))
    print(f"wrote {output.relative_to(root)}")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        print(f"git command failed: {error}", file=sys.stderr)
        raise SystemExit(error.returncode) from error
