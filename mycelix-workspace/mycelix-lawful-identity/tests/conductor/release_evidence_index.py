#!/usr/bin/env python3
"""Build or verify one deterministic index over all conductor evidence manifests."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from evidence_manifest import (
    EvidenceError,
    canonical_json_bytes,
    contract_hash,
    load_and_validate_manifest,
    load_json_object,
    manifest_hash,
    require,
    require_hash32,
    require_text,
    sha256_b64,
)

ROOT = Path(__file__).resolve().parent
CONTRACT_PATH = ROOT / "settlement_scenarios_v2.json"
INDEX_SCHEMA = "mycelix-lawful-identity-release-evidence-index-v1"
INDEX_FILENAME = "release-evidence-index.json"


class IndexError(RuntimeError):
    pass


def load_contract() -> dict[str, Any]:
    value = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or not isinstance(value.get("scenarios"), list):
        raise IndexError("scenario contract is malformed")
    return value


def build_index(evidence_root: Path) -> dict[str, Any]:
    evidence_root = evidence_root.resolve(strict=True)
    if not evidence_root.is_dir() or evidence_root.is_symlink():
        raise IndexError("evidence root must be a non-symlink directory")
    contract = load_contract()
    expected_contract_hash = contract_hash(CONTRACT_PATH)
    scenario_entries: list[dict[str, Any]] = []
    shared_dna: str | None = None
    shared_happ: str | None = None
    shared_adapter: str | None = None
    expected_dirs: set[str] = set()

    for scenario in contract["scenarios"]:
        scenario_id = require_text(scenario.get("id"), "scenario.id")
        expected_state = require_text(scenario.get("expected_state"), "scenario.expected_state")
        expected_dirs.add(scenario_id)
        manifest_path = evidence_root / scenario_id / "manifest.json"
        raw = load_json_object(manifest_path, 512 * 1024, "scenario evidence manifest")
        dna_hash = require_text(raw.get("dna_hash"), "dna_hash", 4096)
        _, summary = load_and_validate_manifest(
            manifest_path,
            evidence_root,
            expected_contract_hash=expected_contract_hash,
            expected_scenario_id=scenario_id,
            expected_state=expected_state,
            expected_dna_hash=dna_hash,
        )
        happ_hash = require_text(raw.get("happ_hash"), "happ_hash", 4096)
        adapter_hash = require_hash32(raw.get("adapter_release_hash"), "adapter_release_hash")
        if shared_dna is None:
            shared_dna, shared_happ, shared_adapter = dna_hash, happ_hash, adapter_hash
        require(dna_hash == shared_dna, "scenario manifests use different DNA hashes")
        require(happ_hash == shared_happ, "scenario manifests use different hApp hashes")
        require(adapter_hash == shared_adapter, "scenario manifests use different adapter releases")
        scenario_entries.append(
            {
                "scenario_id": scenario_id,
                "expected_state": expected_state,
                "manifest_path": f"{scenario_id}/manifest.json",
                "manifest_hash": summary["manifest_hash"],
            }
        )

    allowed_top_level = expected_dirs | {INDEX_FILENAME}
    for child in evidence_root.iterdir():
        require(not child.is_symlink(), f"evidence root contains a symlink: {child.name}")
        require(child.name in allowed_top_level, f"unexpected top-level evidence path: {child.name}")

    require(shared_dna is not None, "scenario contract contains no scenarios")
    return {
        "schema": INDEX_SCHEMA,
        "scenario_contract_hash": expected_contract_hash,
        "dna_hash": shared_dna,
        "happ_hash": shared_happ,
        "adapter_release_hash": shared_adapter,
        "scenario_count": len(scenario_entries),
        "scenarios": scenario_entries,
    }


def output_path(evidence_root: Path, requested: Path | None) -> Path:
    path = requested if requested is not None else evidence_root / INDEX_FILENAME
    path = path.resolve(strict=False)
    root = evidence_root.resolve(strict=True)
    if not path.is_relative_to(root):
        raise IndexError("index output must remain inside evidence root")
    if path.name != INDEX_FILENAME:
        raise IndexError(f"index output filename must be {INDEX_FILENAME}")
    if path.exists() and path.is_symlink():
        raise IndexError("index output may not be a symlink")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    try:
        root = args.evidence_root.resolve(strict=True)
        target = output_path(root, args.output)
        expected = build_index(root)
        expected_hash = manifest_hash(expected)
        if args.verify:
            actual = load_json_object(target, 512 * 1024, "release evidence index")
            require(actual == expected, "release evidence index content mismatch")
        else:
            target.write_bytes(canonical_json_bytes(expected) + b"\n")
        print(f"release evidence index: PASS ({len(expected['scenarios'])} scenarios)")
        print(f"release_evidence_index_hash_b64={expected_hash}")
        return 0
    except (EvidenceError, IndexError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"release evidence index: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
