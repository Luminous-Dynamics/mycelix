#!/usr/bin/env python3
"""Standard-library-only validation for the synthetic home reference fixture.

This is a CI/test verifier only. No production home-design runtime depends on
Python. A full JSON Schema implementation is intentionally not vendored here;
this verifier checks the published schema's supported subset plus the semantic
cross-references that JSON Schema cannot conveniently express.
"""
from __future__ import annotations

from copy import deepcopy
import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = ROOT / "fixtures" / "johannesburg-adaptable-home-v1.json"
SCHEMA_PATH = ROOT / "fixtures" / "home-design-reference-fixture-1.0.0.schema.json"


def fail(path: str, message: str) -> None:
    raise ValueError(f"{path}: {message}")


def json_type_matches(value: Any, expected: str) -> bool:
    if expected == "null":
        return value is None
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return (isinstance(value, int) or isinstance(value, float)) and not isinstance(value, bool)
    if expected == "string":
        return isinstance(value, str)
    return False


def validate(value: Any, schema: dict[str, Any], path: str = "$") -> None:
    expected_type = schema.get("type")
    if expected_type is not None:
        choices = expected_type if isinstance(expected_type, list) else [expected_type]
        if not any(json_type_matches(value, choice) for choice in choices):
            fail(path, f"expected type {choices}, got {type(value).__name__}")

    if "const" in schema and value != schema["const"]:
        fail(path, f"expected constant {schema['const']!r}, got {value!r}")
    if "enum" in schema and value not in schema["enum"]:
        fail(path, f"value {value!r} is not in enum {schema['enum']!r}")

    if isinstance(value, str):
        if len(value) < schema.get("minLength", 0):
            fail(path, "string shorter than minLength")
        if "pattern" in schema and re.search(schema["pattern"], value) is None:
            fail(path, f"string does not match pattern {schema['pattern']!r}")

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            fail(path, f"value is below minimum {schema['minimum']}")
        if "maximum" in schema and value > schema["maximum"]:
            fail(path, f"value is above maximum {schema['maximum']}")
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
            fail(path, f"value is not above exclusiveMinimum {schema['exclusiveMinimum']}")

    if isinstance(value, dict):
        missing = set(schema.get("required", [])) - set(value)
        if missing:
            fail(path, f"missing required properties {sorted(missing)}")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            unexpected = set(value) - set(properties)
            if unexpected:
                fail(path, f"unexpected properties {sorted(unexpected)}")
        for key, child in properties.items():
            if key in value:
                validate(value[key], child, f"{path}.{key}")
        extra_schema = schema.get("additionalProperties")
        if isinstance(extra_schema, dict):
            for key in set(value) - set(properties):
                validate(value[key], extra_schema, f"{path}.{key}")
        if len(value) < schema.get("minProperties", 0):
            fail(path, "object has fewer properties than minProperties")

    if isinstance(value, list):
        if len(value) < schema.get("minItems", 0):
            fail(path, "array shorter than minItems")
        if "maxItems" in schema and len(value) > schema["maxItems"]:
            fail(path, "array longer than maxItems")
        if schema.get("uniqueItems"):
            encoded = [json.dumps(item, sort_keys=True, separators=(",", ":")) for item in value]
            if len(set(encoded)) != len(encoded):
                fail(path, "array items are not unique")
        if "prefixItems" in schema:
            prefix = schema["prefixItems"]
            if len(value) < len(prefix):
                fail(path, "array shorter than prefixItems")
            if schema.get("items") is False and len(value) != len(prefix):
                fail(path, "array has items beyond prefixItems")
            for index, child in enumerate(prefix):
                validate(value[index], child, f"{path}[{index}]")
        elif isinstance(schema.get("items"), dict):
            for index, item in enumerate(value):
                validate(item, schema["items"], f"{path}[{index}]")

    if "anyOf" in schema:
        errors = []
        for alternative in schema["anyOf"]:
            try:
                validate(value, alternative, path)
                break
            except ValueError as error:
                errors.append(str(error))
        else:
            fail(path, "no anyOf alternative matched: " + " | ".join(errors))


def descendants(start: str, adjacency: dict[str, list[str]]) -> set[str]:
    visited = {start}
    queue = [start]
    while queue:
        current = queue.pop(0)
        for child in adjacency.get(current, []):
            if child not in visited:
                visited.add(child)
                queue.append(child)
    return visited


def validate_semantics(fixture: dict[str, Any]) -> None:
    for key in ("candidate_style_families", "candidate_construction_families"):
        family_ids = [item["id"] for item in fixture[key]]
        if len(set(family_ids)) != len(family_ids):
            fail(f"$.{key}", "family IDs must be unique")

    ids = fixture["stable_artifact_ids"]
    values = list(ids.values())
    if len(set(values)) != len(values):
        fail("$.stable_artifact_ids", "artifact IDs must be unique")
    known = set(values)
    edges = [tuple(edge) for edge in fixture["dependency_edges"]]
    if len(set(edges)) != len(edges):
        fail("$.dependency_edges", "duplicate dependency edge")
    adjacency: dict[str, list[str]] = {}
    for source, dependent in edges:
        if source not in known or dependent not in known:
            fail("$.dependency_edges", f"edge references undeclared artifact: {(source, dependent)!r}")
        if source == dependent:
            fail("$.dependency_edges", f"self-dependency is forbidden: {source!r}")
        adjacency.setdefault(source, []).append(dependent)

    scenarios = {item["id"]: item for item in fixture["change_scenarios"]}
    if len(scenarios) != len(fixture["change_scenarios"]):
        fail("$.change_scenarios", "scenario IDs must be unique")
    for scenario in fixture["change_scenarios"]:
        changed = scenario.get("changed_id")
        if changed is None:
            continue
        if changed not in known:
            fail(f"scenario:{scenario['id']}", f"unknown changed_id {changed!r}")
        closure = descendants(changed, adjacency)
        expected = set(scenario.get("expected_to_invalidate", []))
        actual_dependents = closure - {changed}
        if expected != actual_dependents:
            fail(
                f"scenario:{scenario['id']}.expected_to_invalidate",
                "expected invalidation set differs from dependency closure: "
                f"missing={sorted(actual_dependents - expected)}, "
                f"extra={sorted(expected - actual_dependents)}",
            )
        for key in ("expected_to_remain_current_until_other_change", "expected_not_to_invalidate"):
            excluded = set(scenario.get(key, []))
            overlap = excluded & closure
            if overlap:
                fail(
                    f"scenario:{scenario['id']}.{key}",
                    f"items incorrectly considered unaffected: {sorted(overlap)}",
                )

    evidence_boundary = fixture["evidence_boundary"]
    if evidence_boundary["source_files_are_synthetic"] is not True:
        fail("$.evidence_boundary", "fixture must remain marked synthetic")
    if any(evidence_boundary[key] for key in (
        "design_geometry_included", "IFC_file_included", "engineering_solvers_included",
        "legal_rule_pack_included", "code_compliance_claim", "automatic_construction_or_device_execution"
    )):
        fail("$.evidence_boundary", "fixture must not imply real design or compliance evidence")


def expect_rejected(name: str, operation: Any) -> None:
    try:
        operation()
    except (ValueError, KeyError, TypeError):
        return
    fail(f"negative_control:{name}", "mutated invalid fixture was incorrectly accepted")


def run_negative_controls(fixture: dict[str, Any], schema: dict[str, Any]) -> int:
    missing_field = deepcopy(fixture)
    del missing_field["owner_brief"]
    expect_rejected("missing_required_owner_brief", lambda: validate(missing_field, schema))

    false_authority = deepcopy(fixture)
    false_authority["jurisdiction_profile"]["authority_verified"] = True
    expect_rejected("unverified_authority_claim", lambda: validate(false_authority, schema))

    false_compliance = deepcopy(fixture)
    false_compliance["evidence_boundary"]["code_compliance_claim"] = True
    expect_rejected("unsupported_compliance_claim", lambda: validate(false_compliance, schema))

    dangling_edge = deepcopy(fixture)
    dangling_edge["dependency_edges"][0][1] = "artifact:undeclared"
    expect_rejected("dangling_dependency", lambda: validate_semantics(dangling_edge))

    stale_finding_marked_current = deepcopy(fixture)
    stale_finding_marked_current["change_scenarios"][0][
        "expected_to_remain_current_until_other_change"
    ].append("finding:energy-rule")
    expect_rejected(
        "dependent_finding_marked_unaffected",
        lambda: validate_semantics(stale_finding_marked_current),
    )

    duplicate_family = deepcopy(fixture)
    duplicate_family["candidate_style_families"][1]["id"] = duplicate_family[
        "candidate_style_families"
    ][0]["id"]
    expect_rejected("duplicate_style_family_id", lambda: validate_semantics(duplicate_family))

    incomplete_invalidation = deepcopy(fixture)
    incomplete_invalidation["change_scenarios"][0]["expected_to_invalidate"].remove(
        "permit:submission"
    )
    expect_rejected(
        "missing_transitive_invalidation",
        lambda: validate_semantics(incomplete_invalidation),
    )
    return 7


def main() -> int:
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        fail("$schema", "unexpected JSON Schema dialect")
    if schema.get("$id") != "urn:luminous-dynamics:schema:home-design-reference-fixture:1.0.0":
        fail("$id", "unexpected or unversioned fixture schema identity")
    validate(fixture, schema)
    validate_semantics(fixture)
    negative_controls = run_negative_controls(fixture, schema)

    print(
        "HOME_FIXTURE_VALIDATION=PASS "
        f"scenario={fixture['scenario_id']} "
        f"styles={len(fixture['candidate_style_families'])} "
        f"construction_families={len(fixture['candidate_construction_families'])} "
        f"edges={len(fixture['dependency_edges'])} "
        f"scenarios={len(fixture['change_scenarios'])} "
        f"negative_controls={negative_controls}"
    )
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, json.JSONDecodeError, ValueError, KeyError, TypeError) as error:
        print(f"HOME_FIXTURE_VALIDATION=FAIL {error}", file=sys.stderr)
        sys.exit(1)
