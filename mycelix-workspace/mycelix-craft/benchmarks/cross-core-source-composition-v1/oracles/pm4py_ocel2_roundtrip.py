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
import re
import sys
from typing import Any

EXPECTED_PM4PY = "2.7.23.15"


def canonical_value(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


_RFC3339_TIME = re.compile(
    r"^\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}"
    r"(?:\\.\\d{1,6})?(?:Z|[+-]\\d{2}:\\d{2})$"
)


def normalize_time(value: Any) -> str:
    if not isinstance(value, str) or not _RFC3339_TIME.fullmatch(value):
        raise ValueError(f"invalid OCEL timestamp: {value!r}")
    candidate = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = dt.datetime.fromisoformat(candidate)
    except ValueError as exc:
        raise ValueError(f"invalid OCEL timestamp: {value!r}") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError(f"OCEL timestamp must include an explicit UTC offset: {value!r}")
    normalized = parsed.astimezone(dt.timezone.utc).isoformat(timespec="auto")
    return normalized.removesuffix("+00:00") + "Z"


def require_keys(record: Any, allowed: set[str], context: str) -> dict[str, Any]:
    if not isinstance(record, dict):
        raise ValueError(f"{context} must be an object")
    unknown = sorted(set(record) - allowed)
    if unknown:
        raise ValueError(
            f"unsupported extension field(s) at {context}: {', '.join(unknown)}"
        )
    return record


def semantic_type(kind: str) -> str:
    # PM4Py uses equivalent implementation labels for standard OCEL value types.
    aliases = {
        "date": "time",
        "datetime": "time",
        "int": "integer",
        "double": "float",
        "bool": "boolean",
    }
    if not isinstance(kind, str):
        raise ValueError("OCEL attribute declaration type must be a string")
    normalized = aliases.get(kind.lower(), kind.lower())
    if normalized not in {"string", "time", "integer", "float", "boolean"}:
        raise ValueError(f"unsupported OCEL attribute type: {kind!r}")
    return normalized


def declared_types(data: dict[str, Any], key: str) -> dict[str, dict[str, str]]:
    declarations = data.get(key)
    if not isinstance(declarations, list):
        raise ValueError(f"{key} must be an array")
    result: dict[str, dict[str, str]] = {}
    for index, raw_declaration in enumerate(declarations):
        declaration = require_keys(
            raw_declaration, {"name", "attributes"}, f"{key}[{index}]"
        )
        name = declaration.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{key}[{index}].name must be a non-empty string")
        if name in result:
            raise ValueError(f"duplicate {key} declaration: {name}")
        attrs = declaration.get("attributes", [])
        if not isinstance(attrs, list):
            raise ValueError(f"{key}[{index}].attributes must be an array")
        schema: dict[str, str] = {}
        for attr_index, raw_attribute in enumerate(attrs):
            attribute = require_keys(
                raw_attribute,
                {"name", "type"},
                f"{key}[{index}].attributes[{attr_index}]",
            )
            attr_name = attribute.get("name")
            if not isinstance(attr_name, str) or not attr_name:
                raise ValueError("OCEL attribute declaration name must be non-empty")
            if attr_name in schema:
                raise ValueError(
                    f"duplicate attribute declaration in {key} type {name}: {attr_name}"
                )
            schema[attr_name] = semantic_type(attribute.get("type"))
        result[name] = schema
    return result


def typed_value(value: Any, kind: str | None) -> Any:
    if kind is None:
        raise ValueError("OCEL attribute has no declared type")
    if kind == "time":
        return normalize_time(value)
    if kind == "string":
        if not isinstance(value, str):
            raise ValueError(f"expected string OCEL value, got {type(value).__name__}")
        return value
    if kind == "integer":
        if type(value) is not int:
            raise ValueError(f"expected integer OCEL value, got {type(value).__name__}")
        return value
    if kind == "float":
        if type(value) not in {int, float}:
            raise ValueError(f"expected numeric OCEL value, got {type(value).__name__}")
        return value
    if kind == "boolean":
        if type(value) is not bool:
            raise ValueError(f"expected boolean OCEL value, got {type(value).__name__}")
        return value
    raise ValueError(f"unsupported OCEL value type: {kind!r}")


def normalize_inventory(path: pathlib.Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    require_keys(data, {"eventTypes", "objectTypes", "events", "objects"}, "OCEL root")
    for key in ("eventTypes", "objectTypes", "events", "objects"):
        if not isinstance(data.get(key), list):
            raise ValueError(f"{key} must be an array")
    event_types = declared_types(data, "eventTypes")
    object_types = declared_types(data, "objectTypes")

    normalized_event_types = [
        (name, sorted(attrs.items())) for name, attrs in sorted(event_types.items())
    ]
    normalized_object_types = [
        (name, sorted(attrs.items())) for name, attrs in sorted(object_types.items())
    ]

    events = []
    event_ids: set[str] = set()
    for event_index, raw_event in enumerate(data["events"]):
        event = require_keys(
            raw_event,
            {"id", "type", "time", "attributes", "relationships"},
            f"events[{event_index}]",
        )
        event_id = event.get("id")
        event_type = event.get("type")
        if not isinstance(event_id, str) or not event_id:
            raise ValueError(f"events[{event_index}].id must be non-empty")
        if event_id in event_ids:
            raise ValueError(f"duplicate event ID: {event_id}")
        event_ids.add(event_id)
        if not isinstance(event_type, str) or event_type not in event_types:
            raise ValueError(f"unknown event type for {event_id}: {event_type!r}")
        schema = event_types[event_type]
        raw_attributes = event.get("attributes", [])
        raw_relationships = event.get("relationships", [])
        if not isinstance(raw_attributes, list) or not isinstance(raw_relationships, list):
            raise ValueError(f"event attributes/relationships must be arrays: {event_id}")
        attributes = []
        seen_event_attributes: set[str] = set()
        for attribute_index, raw_attribute in enumerate(raw_attributes):
            item = require_keys(
                raw_attribute,
                {"name", "value"},
                f"events[{event_index}].attributes[{attribute_index}]",
            )
            name = item.get("name")
            if not isinstance(name, str) or name not in schema:
                raise ValueError(f"undeclared event attribute on {event_id}: {name!r}")
            if name in seen_event_attributes:
                raise ValueError(f"duplicate event attribute on {event_id}: {name}")
            seen_event_attributes.add(name)
            attributes.append((name, canonical_value(typed_value(item.get("value"), schema[name]))))
        relationships = []
        for relationship_index, raw_relationship in enumerate(raw_relationships):
            item = require_keys(
                raw_relationship,
                {"objectId", "qualifier"},
                f"events[{event_index}].relationships[{relationship_index}]",
            )
            object_id = item.get("objectId")
            qualifier = item.get("qualifier")
            if not isinstance(object_id, str) or not object_id:
                raise ValueError(f"invalid event relationship endpoint on {event_id}")
            if qualifier is not None and not isinstance(qualifier, str):
                raise ValueError(f"invalid event relationship qualifier on {event_id}")
            relationships.append((object_id, qualifier))
        events.append(
            (
                event_id,
                event_type,
                normalize_time(event.get("time")),
                sorted(attributes),
                sorted(relationships),
            )
        )

    objects = []
    object_ids: set[str] = set()
    pending_object_relationships: list[tuple[str, str]] = []
    for object_index, raw_object in enumerate(data["objects"]):
        obj = require_keys(
            raw_object,
            {"id", "type", "attributes", "relationships"},
            f"objects[{object_index}]",
        )
        object_id = obj.get("id")
        object_type = obj.get("type")
        if not isinstance(object_id, str) or not object_id:
            raise ValueError(f"objects[{object_index}].id must be non-empty")
        if object_id in object_ids:
            raise ValueError(f"duplicate object ID: {object_id}")
        object_ids.add(object_id)
        if not isinstance(object_type, str) or object_type not in object_types:
            raise ValueError(f"unknown object type for {object_id}: {object_type!r}")
        schema = object_types[object_type]
        raw_attributes = obj.get("attributes", [])
        raw_relationships = obj.get("relationships", [])
        if not isinstance(raw_attributes, list) or not isinstance(raw_relationships, list):
            raise ValueError(f"object attributes/relationships must be arrays: {object_id}")
        attributes = []
        # Repeated object attribute names are valid OCEL history when their
        # timestamps differ; do not collapse them into a dictionary.
        for attribute_index, raw_attribute in enumerate(raw_attributes):
            item = require_keys(
                raw_attribute,
                {"name", "time", "value"},
                f"objects[{object_index}].attributes[{attribute_index}]",
            )
            name = item.get("name")
            if not isinstance(name, str) or name not in schema:
                raise ValueError(f"undeclared object attribute on {object_id}: {name!r}")
            attributes.append(
                (
                    name,
                    normalize_time(item.get("time")),
                    canonical_value(typed_value(item.get("value"), schema[name])),
                )
            )
        relationships = []
        for relationship_index, raw_relationship in enumerate(raw_relationships):
            item = require_keys(
                raw_relationship,
                {"objectId", "qualifier"},
                f"objects[{object_index}].relationships[{relationship_index}]",
            )
            target_id = item.get("objectId")
            qualifier = item.get("qualifier")
            if not isinstance(target_id, str) or not target_id:
                raise ValueError(f"invalid object relationship endpoint on {object_id}")
            if qualifier is not None and not isinstance(qualifier, str):
                raise ValueError(f"invalid object relationship qualifier on {object_id}")
            relationships.append((target_id, qualifier))
            pending_object_relationships.append((object_id, target_id))
        objects.append((object_id, object_type, sorted(attributes), sorted(relationships)))

    known_object_ids = object_ids
    for source_id, target_id in pending_object_relationships:
        if target_id not in known_object_ids:
            raise ValueError(f"unknown object relationship target: {source_id} -> {target_id}")
    for event in events:
        for target_id, _qualifier in event[4]:
            if target_id not in known_object_ids:
                raise ValueError(f"unknown event relationship target: {event[0]} -> {target_id}")

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
        "extension_policy": (
            "Restricted profile: unknown top-level and record fields are rejected; "
            "no extension-field preservation is claimed."
        ),
        "coercions_applied": [],
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
