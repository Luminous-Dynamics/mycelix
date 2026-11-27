#!/usr/bin/env python3
"""Test-only OCEL 2.0 differential round-trip oracle using pinned PM4Py.

This is intentionally outside the Rust production/runtime dependency graph.
It compares normalized OCEL-standard fields, not raw JSON formatting.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import pathlib
import sys
from typing import Any

EXPECTED_PM4PY = "2.7.23.15"


def canonical_value(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def normalize_time(value: Any) -> Any:
    if not isinstance(value, str):
        return value
    candidate = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = dt.datetime.fromisoformat(candidate)
    except ValueError:
        return value
    if parsed.tzinfo is None:
        return value
    normalized = parsed.astimezone(dt.timezone.utc).isoformat(timespec="auto")
    return normalized.removesuffix("+00:00") + "Z"


def declared_types(data: dict[str, Any], key: str) -> dict[str, dict[str, str]]:
    declarations = data.get(key)
    if not isinstance(declarations, list):
        raise ValueError(f"{key} must be an array")
    result: dict[str, dict[str, str]] = {}
    for declaration in declarations:
        name = declaration["name"]
        if name in result:
            raise ValueError(f"duplicate {key} declaration: {name}")
        attrs = declaration.get("attributes", [])
        result[name] = {item["name"]: item["type"] for item in attrs}
    return result


def typed_value(value: Any, kind: str | None) -> Any:
    if kind == "time":
        return normalize_time(value)
    if kind == "integer" and value is not None:
        return int(value)
    if kind == "float" and value is not None:
        return float(value)
    if kind == "boolean" and value is not None:
        return bool(value)
    return value


def normalize_inventory(path: pathlib.Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    event_types = declared_types(data, "eventTypes")
    object_types = declared_types(data, "objectTypes")

    normalized_event_types = [
        (name, sorted(attrs.items())) for name, attrs in sorted(event_types.items())
    ]
    normalized_object_types = [
        (name, sorted(attrs.items())) for name, attrs in sorted(object_types.items())
    ]

    events = []
    for event in data.get("events", []):
        event_type = event["type"]
        schema = event_types[event_type]
        attributes = sorted(
            (
                item["name"],
                canonical_value(typed_value(item["value"], schema.get(item["name"]))),
            )
            for item in event.get("attributes", [])
        )
        relationships = sorted(
            (item["objectId"], item.get("qualifier"))
            for item in event.get("relationships", [])
        )
        events.append(
            (
                event["id"],
                event_type,
                normalize_time(event["time"]),
                attributes,
                relationships,
            )
        )

    objects = []
    for obj in data.get("objects", []):
        object_type = obj["type"]
        schema = object_types[object_type]
        attributes = sorted(
            (
                item["name"],
                normalize_time(item["time"]),
                canonical_value(typed_value(item["value"], schema.get(item["name"]))),
            )
            for item in obj.get("attributes", [])
        )
        relationships = sorted(
            (item["objectId"], item.get("qualifier"))
            for item in obj.get("relationships", [])
        )
        objects.append((obj["id"], object_type, attributes, relationships))

    return {
        "event_types": normalized_event_types,
        "object_types": normalized_object_types,
        "events": sorted(events),
        "objects": sorted(objects),
    }


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=pathlib.Path, required=True)
    parser.add_argument("--roundtrip", type=pathlib.Path, required=True)
    parser.add_argument("--expected-pm4py", default=EXPECTED_PM4PY)
    args = parser.parse_args()

    try:
        import pm4py
    except ImportError as exc:
        print(f"PM4Py import failed: {exc}", file=sys.stderr)
        return 2

    actual_version = getattr(pm4py, "__version__", "unknown")
    if actual_version != args.expected_pm4py:
        print(
            f"PM4Py version mismatch: expected {args.expected_pm4py}, got {actual_version}",
            file=sys.stderr,
        )
        return 2

    try:
        source_inventory = normalize_inventory(args.source)
        ocel = pm4py.read_ocel2_json(str(args.source))
        args.roundtrip.parent.mkdir(parents=True, exist_ok=True)
        pm4py.write_ocel2_json(ocel, str(args.roundtrip))
        roundtrip_inventory = normalize_inventory(args.roundtrip)
    except Exception as exc:  # external tool failures must be visible to the gate
        print(f"PM4Py OCEL 2.0 round-trip failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    if source_inventory != roundtrip_inventory:
        print(
            json.dumps(
                {
                    "gate": "fail",
                    "reason": "normalized OCEL inventory differs after PM4Py round-trip",
                    "source": source_inventory,
                    "roundtrip": roundtrip_inventory,
                },
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1

    inventory_bytes = canonical_value(source_inventory).encode("utf-8")
    report = {
        "gate": "pass",
        "pm4py_version": actual_version,
        "source_sha256": sha256_file(args.source),
        "roundtrip_sha256": sha256_file(args.roundtrip),
        "normalized_inventory_sha256": hashlib.sha256(inventory_bytes).hexdigest(),
        "event_count": len(source_inventory["events"]),
        "object_count": len(source_inventory["objects"]),
        "event_type_count": len(source_inventory["event_types"]),
        "object_type_count": len(source_inventory["object_types"]),
        "preserved": [
            "declared event/object types",
            "event IDs, types and timestamps",
            "typed event attributes",
            "event-to-object IDs and qualifiers",
            "object IDs and types",
            "typed, timestamped object attributes",
            "object-to-object IDs and qualifiers",
        ],
        "claim_boundary": (
            "Synthetic OCEL interchange only; not source authenticity, "
            "temporal authority, owner-core qualification or field evidence."
        ),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
