#!/usr/bin/env python3
"""Compare Integral external-source generation manifests without deciding authority.

This tool intentionally performs no network access. A collector or human produces an
observed manifest; this tool compares it with a frozen baseline and reports drift.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SourceView:
    source_id: str
    status: str
    identity: dict[str, Any]


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        doc = json.load(handle)
    if not isinstance(doc, dict) or not isinstance(doc.get("sources"), list):
        raise ValueError(f"{path}: manifest must contain a sources array")
    ids: set[str] = set()
    for source in doc["sources"]:
        if not isinstance(source, dict):
            raise ValueError(f"{path}: each source must be an object")
        source_id = source.get("source_id")
        if not isinstance(source_id, str) or not source_id.strip():
            raise ValueError(f"{path}: every source requires a non-empty source_id")
        if source_id in ids:
            raise ValueError(f"{path}: duplicate source_id {source_id}")
        ids.add(source_id)
        identity = source.get("identity")
        if not isinstance(identity, dict):
            raise ValueError(f"{path}: {source_id} requires an identity object")
        kind = identity.get("kind")
        if kind == "github_blob":
            sha = identity.get("blob_sha")
            if not isinstance(sha, str) or len(sha) != 40:
                raise ValueError(f"{path}: {source_id} requires a 40-character blob_sha")
        elif kind == "web_observation":
            token = identity.get("observation_token")
            if not isinstance(token, str) or not token:
                raise ValueError(f"{path}: {source_id} requires observation_token")
        else:
            raise ValueError(f"{path}: {source_id} has unsupported identity kind {kind!r}")
    return doc


def index_sources(doc: dict[str, Any]) -> dict[str, SourceView]:
    return {
        source["source_id"]: SourceView(
            source_id=source["source_id"],
            status=str(source.get("status", "")),
            identity=source["identity"],
        )
        for source in doc["sources"]
    }


def exact_identity(identity: dict[str, Any]) -> bool:
    return identity.get("kind") == "github_blob"


def compare(baseline: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any]:
    base = index_sources(baseline)
    obs = index_sources(observed)
    rows: list[dict[str, Any]] = []

    for source_id in sorted(set(base) | set(obs)):
        left = base.get(source_id)
        right = obs.get(source_id)
        if left is None:
            rows.append({"source_id": source_id, "classification": "new_source"})
            continue
        if right is None:
            rows.append({"source_id": source_id, "classification": "missing_source"})
            continue

        if left.identity != right.identity:
            rows.append(
                {
                    "source_id": source_id,
                    "classification": "changed_generation",
                    "baseline_identity": left.identity,
                    "observed_identity": right.identity,
                    "status_changed": left.status != right.status,
                }
            )
            continue

        if left.status != right.status:
            rows.append(
                {
                    "source_id": source_id,
                    "classification": "status_changed",
                    "baseline_status": left.status,
                    "observed_status": right.status,
                }
            )
            continue

        if not exact_identity(left.identity):
            rows.append(
                {
                    "source_id": source_id,
                    "classification": "unverifiable_web_observation",
                    "reason": "identity has no qualified content commitment",
                }
            )
            continue

        rows.append({"source_id": source_id, "classification": "unchanged"})

    counts: dict[str, int] = {}
    for row in rows:
        key = row["classification"]
        counts[key] = counts.get(key, 0) + 1

    requires_review = any(
        row["classification"]
        in {"new_source", "missing_source", "changed_generation", "status_changed"}
        for row in rows
    )

    return {
        "profile": "myc-int-017n-source-drift-report-v1",
        "requires_review": requires_review,
        "counts": counts,
        "sources": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--observed", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = compare(load_manifest(args.baseline), load_manifest(args.observed))
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 2 if report["requires_review"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
