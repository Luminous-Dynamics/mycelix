#!/usr/bin/env python3
"""Standard-library-only validation for the synthetic home reference fixture.

This is a CI/test verifier only. No production home-design runtime depends on
Python. A full JSON Schema implementation is intentionally not vendored here;
this verifier checks the published schema's supported subset plus the semantic
cross-references that JSON Schema cannot conveniently express.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import re
import sys
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = ROOT / "fixtures" / "johannesburg-adaptable-home-v1.json"
SCHEMA_PATH = ROOT / "fixtures" / "home-design-reference-fixture-1.0.0.schema.json"
CAPABILITY_REGISTRY_PATH = ROOT / "fixtures" / "jurisdiction-capability-cards-synthetic-v1.json"
CAPABILITY_SCHEMA_PATH = ROOT / "fixtures" / "jurisdiction-capability-cards-1.0.0.schema.json"
SOURCE_WATCH_PATH = ROOT / "fixtures" / "jurisdiction-source-watch-za-johannesburg-2026-v1.json"
SOURCE_WATCH_SCHEMA_PATH = ROOT / "fixtures" / "jurisdiction-source-watch-1.0.0.schema.json"
REQUIRED_CAPABILITY_DOMAINS = {
    "residential_building",
    "planning_and_land_use",
    "structural_safety",
    "fire_and_life_safety",
    "energy_and_thermal",
    "accessibility",
    "water_and_sanitation",
    "electrical_installations",
    "permit_and_inspection_process",
    "heritage_and_conservation",
    "environmental_and_site_hazards",
}
SHA256_PATTERN = re.compile(r"^[0-9a-fA-F]{64}$")


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


SUPPORTED_SCHEMA_KEYWORDS = {
    "$id",
    "$schema",
    "additionalProperties",
    "anyOf",
    "const",
    "description",
    "enum",
    "exclusiveMinimum",
    "items",
    "maxItems",
    "maximum",
    "minItems",
    "minLength",
    "minProperties",
    "minimum",
    "pattern",
    "prefixItems",
    "properties",
    "required",
    "title",
    "type",
    "uniqueItems",
}


def validate_schema_supported(schema: Any, path: str = "$schema") -> None:
    """Reject schema keywords this minimal validator does not implement."""
    if isinstance(schema, bool):
        return
    if not isinstance(schema, dict):
        fail(path, "schema must be an object or boolean")

    unsupported = set(schema) - SUPPORTED_SCHEMA_KEYWORDS
    if unsupported:
        fail(path, f"unsupported JSON Schema keywords {sorted(unsupported)}")

    valid_types = {"array", "boolean", "integer", "null", "number", "object", "string"}
    if "type" in schema:
        types = schema["type"] if isinstance(schema["type"], list) else [schema["type"]]
        if not types or any(not isinstance(item, str) or item not in valid_types for item in types):
            fail(path, "type must name one or more supported JSON Schema primitive types")
        if len(set(types)) != len(types):
            fail(path, "type array must not repeat a type")
    if "required" in schema and (
        not isinstance(schema["required"], list)
        or any(not isinstance(item, str) for item in schema["required"])
    ):
        fail(path, "required must be an array of property names")
    if "enum" in schema and (
        not isinstance(schema["enum"], list) or not schema["enum"]
    ):
        fail(path, "enum must be a non-empty array")
    for keyword in ("minimum", "maximum", "exclusiveMinimum"):
        if keyword in schema and (
            not isinstance(schema[keyword], (int, float))
            or isinstance(schema[keyword], bool)
        ):
            fail(path, f"{keyword} must be numeric")
    for keyword in ("maxItems", "minItems", "minLength", "minProperties"):
        if keyword in schema and (
            not isinstance(schema[keyword], int)
            or isinstance(schema[keyword], bool)
            or schema[keyword] < 0
        ):
            fail(path, f"{keyword} must be a non-negative integer")
    if "pattern" in schema:
        if not isinstance(schema["pattern"], str):
            fail(path, "pattern must be a string")
        try:
            re.compile(schema["pattern"])
        except re.error as error:
            fail(path, f"invalid schema regular expression: {error}")
    if "uniqueItems" in schema and not isinstance(schema["uniqueItems"], bool):
        fail(path, "uniqueItems must be boolean")

    properties = schema.get("properties", {})
    if "properties" in schema and not isinstance(properties, dict):
        fail(path, "properties must be an object")
    for key, child in properties.items():
        validate_schema_supported(child, f"{path}.properties.{key}")

    items = schema.get("items")
    if "items" in schema and not isinstance(items, (dict, bool)):
        fail(path, "items must be a schema object or boolean")
    if isinstance(items, dict):
        validate_schema_supported(items, f"{path}.items")

    extra = schema.get("additionalProperties")
    if "additionalProperties" in schema and not isinstance(extra, (dict, bool)):
        fail(path, "additionalProperties must be a schema object or boolean")
    if isinstance(extra, dict):
        validate_schema_supported(extra, f"{path}.additionalProperties")

    prefix = schema.get("prefixItems", [])
    if "prefixItems" in schema and not isinstance(prefix, list):
        fail(path, "prefixItems must be an array of schemas")
    for index, child in enumerate(prefix):
        validate_schema_supported(child, f"{path}.prefixItems[{index}]")

    alternatives = schema.get("anyOf", [])
    if "anyOf" in schema and not isinstance(alternatives, list):
        fail(path, "anyOf must be an array of schemas")
    for index, child in enumerate(alternatives):
        validate_schema_supported(child, f"{path}.anyOf[{index}]")


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

        prefix = schema.get("prefixItems", [])
        if "prefixItems" in schema:
            if len(value) < len(prefix):
                fail(path, "array shorter than prefixItems")
            for index, child in enumerate(prefix):
                validate(value[index], child, f"{path}[{index}]")

        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index in range(len(prefix), len(value)):
                validate(value[index], item_schema, f"{path}[{index}]")
        elif item_schema is False and len(value) > len(prefix):
            fail(path, "array has items beyond its allowed item schemas")

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
    queue = deque([start])
    while queue:
        current = queue.popleft()
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

    # A dependency graph is a DAG, not merely a graph whose traversal happens
    # to terminate. Reject cycles before using the closure as invalidation proof.
    nodes = set(values)
    indegree = {node: 0 for node in nodes}
    for source, dependents in adjacency.items():
        for dependent in dependents:
            indegree[dependent] += 1
    queue = deque(sorted(node for node, degree in indegree.items() if degree == 0))
    processed = 0
    while queue:
        current = queue.popleft()
        processed += 1
        for dependent in adjacency.get(current, []):
            indegree[dependent] -= 1
            if indegree[dependent] == 0:
                queue.append(dependent)
    if processed != len(nodes):
        fail("$.dependency_edges", "cyclic dependency graph is forbidden")

    scenarios = {item["id"]: item for item in fixture["change_scenarios"]}
    if len(scenarios) != len(fixture["change_scenarios"]):
        fail("$.change_scenarios", "scenario IDs must be unique")
    for scenario in fixture["change_scenarios"]:
        single_root = scenario.get("changed_id")
        multiple_roots = scenario.get("changed_ids")
        if single_root is not None and multiple_roots is not None:
            fail(f"scenario:{scenario['id']}", "use changed_id or changed_ids, not both")
        if multiple_roots is not None:
            roots = set(multiple_roots)
        elif single_root is not None:
            roots = {single_root}
        else:
            continue
        if not roots:
            fail(f"scenario:{scenario['id']}", "at least one changed root is required")
        unknown_roots = roots - known
        if unknown_roots:
            fail(f"scenario:{scenario['id']}", f"unknown changed roots {sorted(unknown_roots)}")

        closure: set[str] = set()
        for root in roots:
            closure.update(descendants(root, adjacency))
        expected = set(scenario.get("expected_to_invalidate", []))
        actual_dependents = closure - roots
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


def validate_capability_registry(registry: dict[str, Any]) -> None:
    """Cross-check global rollout claims beyond the portable schema shape."""
    seen_jurisdictions: set[str] = set()
    for card in registry["capability_cards"]:
        path = f"capability_card:{card['jurisdiction_id']}"
        jurisdiction = card["jurisdiction_id"]
        if jurisdiction in seen_jurisdictions:
            fail("$.capability_cards", f"duplicate jurisdiction ID {jurisdiction!r}")
        seen_jurisdictions.add(jurisdiction)

        for field in (
            "subject_sha256",
            "source_registry_snapshot_sha256",
            "test_corpus_sha256",
        ):
            if not SHA256_PATTERN.fullmatch(card[field]):
                fail(f"{path}.{field}", "expected a 64-character SHA-256 hex digest")
        if card["source_registry_checked_on_day"] > card["registry_as_of_day"]:
            fail(path, "source-registry check date cannot be in the future")

        if not card["card_author_id"].strip():
            fail(path, "capability-card author is required")
        if not card["accountable_maintainer"].strip():
            fail(path, "accountable maintainer is required")
        if not card["update_policy"].strip():
            fail(path, "source update policy is required")
        if any(not gap.strip() for gap in card["known_scope_gaps"]):
            fail(path, "known scope gaps must be explicit, nonblank records")

        coverage_by_domain: dict[str, dict[str, Any]] = {}
        for record in card["coverage"]:
            domain = record["domain"]
            if domain in coverage_by_domain:
                fail(path, f"duplicate coverage record for {domain}")
            coverage_by_domain[domain] = record
            disposition = record["disposition"]
            if disposition == "covered":
                if not record["module_ids"] or any(not value.strip() for value in record["module_ids"]):
                    fail(path, f"{domain}: covered state requires module IDs")
                if len(set(record["module_ids"])) != len(record["module_ids"]):
                    fail(path, f"{domain}: module IDs must be unique")
                if not SHA256_PATTERN.fullmatch(record["source_snapshot_sha256"]):
                    fail(path, f"{domain}: source snapshot digest is malformed")
            elif disposition == "explicitly_not_applicable":
                if not record["rationale"].strip() or not record["decision_reference"].strip():
                    fail(path, f"{domain}: not-applicable needs rationale and decision reference")
            elif disposition == "missing":
                if not record["gap_description"].strip():
                    fail(path, f"{domain}: missing coverage needs an explicit gap")

        if card["declared_status"] in {"researching", "pilot", "supported"}:
            missing_domains = REQUIRED_CAPABILITY_DOMAINS - set(coverage_by_domain)
            if missing_domains:
                fail(
                    path,
                    "research/pilot/support profiles must explicitly classify every regulatory "
                    f"domain; omitted={sorted(missing_domains)}",
                )

        reviewer_ids: set[str] = set()
        for review in card["independent_reviews"]:
            reviewer = review["reviewer_id"]
            if not reviewer.strip() or reviewer in reviewer_ids:
                fail(path, f"reviewer ID is blank or duplicate: {reviewer!r}")
            reviewer_ids.add(reviewer)
            if reviewer in {card["card_author_id"], card["accountable_maintainer"]}:
                fail(path, "card author and accountable maintainer cannot count as independent reviewers")
            if review["reviewed_subject_sha256"] != card["subject_sha256"]:
                fail(path, f"reviewer {reviewer!r} did not review this exact card subject")

        if card["declared_status"] == "supported":
            missing = REQUIRED_CAPABILITY_DOMAINS - set(coverage_by_domain)
            if missing:
                fail(path, f"supported claim omits regulatory domains {sorted(missing)}")
            gaps = [
                domain for domain, record in coverage_by_domain.items()
                if domain in REQUIRED_CAPABILITY_DOMAINS and record["disposition"] == "missing"
            ]
            if gaps:
                fail(path, f"supported claim has missing domain coverage {sorted(gaps)}")
            if card["tested_case_count"] <= 0:
                fail(path, "supported claim requires a nonempty independent test corpus")
            if len(reviewer_ids) < 2:
                fail(path, "supported claim requires at least two independent reviewers")
            if not card["independent_reviews"] or any(
                review["disposition"] != "supports_bounded_support_claim"
                for review in card["independent_reviews"]
            ):
                fail(path, "supported claim requires explicit supportive reviews; dissent must not be erased")


def run_capability_negative_controls(
    registry: dict[str, Any], schema: dict[str, Any]
) -> int:
    base = deepcopy(registry["capability_cards"][0])
    base["declared_status"] = "supported"
    base["tested_case_count"] = 120
    base["coverage"] = [
        {
            "domain": domain,
            "disposition": "covered",
            "module_ids": [f"synthetic:{domain}:v1"],
            "source_snapshot_sha256": "d" * 64,
        }
        for domain in sorted(REQUIRED_CAPABILITY_DOMAINS)
    ]
    base["independent_reviews"] = [
        {
            "reviewer_id": reviewer,
            "review_scope": "Independent source and scope review",
            "reviewed_subject_sha256": base["subject_sha256"],
            "disposition": "supports_bounded_support_claim",
            "rationale": "Synthetic test-only review",
            "conflict_of_interest_disclosure": "Synthetic disclosure",
        }
        for reviewer in ("reviewer-a", "reviewer-b")
    ]
    base["known_scope_gaps"] = []
    supported_fixture = deepcopy(registry)
    supported_fixture["capability_cards"] = [base]
    validate(supported_fixture, schema)
    validate_capability_registry(supported_fixture)

    missing_domain = deepcopy(supported_fixture)
    missing_domain["capability_cards"][0]["coverage"] = [
        item for item in missing_domain["capability_cards"][0]["coverage"]
        if item["domain"] != "accessibility"
    ]
    expect_rejected(
        "supported_jurisdiction_missing_domain",
        lambda: validate_capability_registry(missing_domain),
    )

    inconclusive_review = deepcopy(supported_fixture)
    inconclusive_review["capability_cards"][0]["independent_reviews"][1][
        "disposition"
    ] = "inconclusive"
    expect_rejected(
        "supported_jurisdiction_inconclusive_review",
        lambda: validate_capability_registry(inconclusive_review),
    )

    duplicate_jurisdiction = deepcopy(registry)
    duplicate_jurisdiction["capability_cards"][1]["jurisdiction_id"] = (
        duplicate_jurisdiction["capability_cards"][0]["jurisdiction_id"]
    )
    expect_rejected(
        "duplicate_jurisdiction_id",
        lambda: validate_capability_registry(duplicate_jurisdiction),
    )

    wrong_snapshot = deepcopy(supported_fixture)
    wrong_snapshot["capability_cards"][0]["independent_reviews"][0][
        "reviewed_subject_sha256"
    ] = "e" * 64
    expect_rejected(
        "reviewed_different_capability_snapshot",
        lambda: validate_capability_registry(wrong_snapshot),
    )

    author_as_reviewer = deepcopy(supported_fixture)
    author = author_as_reviewer["capability_cards"][0]["card_author_id"]
    author_as_reviewer["capability_cards"][0]["independent_reviews"][0]["reviewer_id"] = author
    expect_rejected(
        "capability_author_claimed_as_independent",
        lambda: validate_capability_registry(author_as_reviewer),
    )

    incomplete_research_coverage = deepcopy(registry)
    incomplete_research_coverage["capability_cards"][0]["coverage"].pop()
    expect_rejected(
        "research_profile_omits_a_regulatory_domain",
        lambda: validate_capability_registry(incomplete_research_coverage),
    )
    return 6


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

    unsupported_keyword_schema = deepcopy(schema)
    unsupported_keyword_schema["properties"]["label"]["not"] = {"const": ""}
    expect_rejected(
        "unsupported_schema_keyword",
        lambda: validate_schema_supported(unsupported_keyword_schema),
    )

    malformed_keyword_schema = deepcopy(schema)
    malformed_keyword_schema["properties"]["label"]["items"] = "not-a-schema"
    expect_rejected(
        "malformed_schema_keyword_value",
        lambda: validate_schema_supported(malformed_keyword_schema),
    )

    forbidden_array_items = deepcopy(schema)
    forbidden_array_items["properties"]["owner_brief"]["properties"]["priorities"]["items"] = False
    expect_rejected(
        "array_item_constraint_is_enforced",
        lambda: validate(fixture, forbidden_array_items),
    )

    cyclic_graph = deepcopy(fixture)
    cyclic_graph["dependency_edges"].append(
        ["finding:energy-rule", "rulepack:za-jhb:energy"]
    )
    expect_rejected("cyclic_dependency_graph", lambda: validate_semantics(cyclic_graph))

    incomplete_multi_root = deepcopy(fixture)
    combined = next(
        item for item in incomplete_multi_root["change_scenarios"]
        if item["id"] == "simultaneous-floorplan-and-energy-rule-revision"
    )
    combined["expected_to_invalidate"].remove("finding:energy-rule")
    expect_rejected(
        "incomplete_multi_root_invalidation",
        lambda: validate_semantics(incomplete_multi_root),
    )

    ambiguous_roots = deepcopy(fixture)
    combined = next(
        item for item in ambiguous_roots["change_scenarios"]
        if item["id"] == "simultaneous-floorplan-and-energy-rule-revision"
    )
    combined["changed_id"] = combined["changed_ids"][0]
    expect_rejected("ambiguous_changed_root_fields", lambda: validate_semantics(ambiguous_roots))

    duplicate_change_root = deepcopy(fixture)
    combined = next(
        item for item in duplicate_change_root["change_scenarios"]
        if item["id"] == "simultaneous-floorplan-and-energy-rule-revision"
    )
    combined["changed_ids"].append(combined["changed_ids"][0])
    expect_rejected("duplicate_changed_root", lambda: validate(duplicate_change_root, schema))
    return 14



def validate_source_watch(watch: dict[str, Any]) -> None:
    """Keep an official-locator inventory separate from verified legal rules."""
    if watch["declared_support_status"] != "researching":
        fail("$.declared_support_status", "source-watch fixture must remain research-only")
    if watch["source_bytes_verified"] is not False:
        fail("$.source_bytes_verified", "unmaterialized source bytes cannot be declared verified")
    if watch["legal_applicability_evaluated"] is not False:
        fail("$.legal_applicability_evaluated", "this source-watch fixture has no legal-applicability review")
    if watch["binding_rules_emitted"] is not False:
        fail("$.binding_rules_emitted", "locator-only source watch must not emit binding rule modules")

    allowed_hosts = {"www.gov.za", "www.thedtic.gov.za"}
    seen: set[str] = set()
    roles: dict[str, list[dict[str, Any]]] = {}
    for record in watch["source_records"]:
        path = f"source_watch:{record['source_id']}"
        source_id = record["source_id"]
        if source_id in seen:
            fail(path, "source IDs must be unique")
        seen.add(source_id)

        parsed = urlparse(record["official_locator"])
        if parsed.scheme != "https" or parsed.hostname not in allowed_hosts or not parsed.path:
            fail(path, "source locator must use an allowlisted official government HTTPS host")
        if record["legal_effect_assessment"] != "not_evaluated":
            fail(path, "source locator does not establish legal effect or applicability")
        if record["source_bytes_verified"] is not False or record["binding_claim_emitted"] is not False:
            fail(path, "unverified locator must not become verified bytes or a binding rule")
        digest = record["source_sha256"]
        if digest and not SHA256_PATTERN.fullmatch(digest):
            fail(path, "source digest must be empty until bytes are preserved, or a SHA-256 hex digest")
        if not digest and record["source_bytes_verified"]:
            fail(path, "source cannot be verified without a recorded digest")
        if record["registry_status"] == "proposal_for_public_comment":
            if record["source_role"] != "draft_bill_public_comment":
                fail(path, "proposal status requires the draft-bill source role")
            if record["binding_claim_emitted"]:
                fail(path, "proposal cannot be emitted as a binding rule")
        roles.setdefault(record["source_role"], []).append(record)

    if not roles.get("existing_act_locator"):
        fail("$.source_records", "preserve an official locator for the existing Act")
    if not roles.get("draft_bill_public_comment"):
        fail("$.source_records", "proposed amendment must be separately recorded")
    extensions = roles.get("comment_deadline_extension_notice", [])
    if len(extensions) != 1:
        fail("$.source_records", "exactly one current consultation extension notice is expected in this fixture")
    if extensions[0]["consultation_deadline"] != "2026-11-12":
        fail("$.source_records", "consultation deadline must match the recorded official extension notice")
    if roles["draft_bill_public_comment"][0]["consultation_deadline"] != "2026-11-12":
        fail("$.source_records", "proposal card should point to the separately recorded deadline extension")


def run_source_watch_negative_controls(
    watch: dict[str, Any], schema: dict[str, Any]
) -> int:
    promoted_proposal = deepcopy(watch)
    bill = next(
        item for item in promoted_proposal["source_records"]
        if item["source_role"] == "draft_bill_public_comment"
    )
    bill["binding_claim_emitted"] = True
    expect_rejected(
        "proposal_silently_promoted_to_binding",
        lambda: (validate(promoted_proposal, schema), validate_source_watch(promoted_proposal)),
    )

    unapproved_host = deepcopy(watch)
    unapproved_host["source_records"][0]["official_locator"] = "https://example.com/act.pdf"
    expect_rejected(
        "untrusted_source_host",
        lambda: (validate(unapproved_host, schema), validate_source_watch(unapproved_host)),
    )

    false_verified_bytes = deepcopy(watch)
    false_verified_bytes["source_records"][0]["source_bytes_verified"] = True
    expect_rejected(
        "unverified_bytes_claimed_verified",
        lambda: (validate(false_verified_bytes, schema), validate_source_watch(false_verified_bytes)),
    )

    missing_deadline = deepcopy(watch)
    extension = next(
        item for item in missing_deadline["source_records"]
        if item["source_role"] == "comment_deadline_extension_notice"
    )
    extension["consultation_deadline"] = ""
    expect_rejected(
        "consultation_extension_without_deadline",
        lambda: validate_source_watch(missing_deadline),
    )
    return 4


def main() -> int:
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        fail("$schema", "unexpected JSON Schema dialect")
    if schema.get("$id") != "urn:luminous-dynamics:schema:home-design-reference-fixture:1.0.0":
        fail("$id", "unexpected or unversioned fixture schema identity")
    validate_schema_supported(schema)
    validate(fixture, schema)
    validate_semantics(fixture)
    home_negative_controls = run_negative_controls(fixture, schema)

    capability_registry = json.loads(CAPABILITY_REGISTRY_PATH.read_text(encoding="utf-8"))
    capability_schema = json.loads(CAPABILITY_SCHEMA_PATH.read_text(encoding="utf-8"))
    if capability_schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        fail("$capability_schema", "unexpected JSON Schema dialect")
    if capability_schema.get("$id") != "urn:luminous-dynamics:schema:jurisdiction-capability-cards:1.0.0":
        fail("$capability_schema.$id", "unexpected or unversioned capability schema identity")
    validate_schema_supported(capability_schema)
    validate(capability_registry, capability_schema)
    validate_capability_registry(capability_registry)
    capability_negative_controls = run_capability_negative_controls(
        capability_registry, capability_schema
    )

    source_watch = json.loads(SOURCE_WATCH_PATH.read_text(encoding="utf-8"))
    source_watch_schema = json.loads(SOURCE_WATCH_SCHEMA_PATH.read_text(encoding="utf-8"))
    if source_watch_schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        fail("$source_watch_schema", "unexpected JSON Schema dialect")
    if source_watch_schema.get("$id") != "urn:luminous-dynamics:schema:jurisdiction-source-watch:1.0.0":
        fail("$source_watch_schema.$id", "unexpected source-watch schema identity")
    validate_schema_supported(source_watch_schema)
    validate(source_watch, source_watch_schema)
    validate_source_watch(source_watch)
    source_watch_negative_controls = run_source_watch_negative_controls(
        source_watch, source_watch_schema
    )

    print(
        "HOME_FIXTURE_VALIDATION=PASS "
        f"scenario={fixture['scenario_id']} "
        f"styles={len(fixture['candidate_style_families'])} "
        f"construction_families={len(fixture['candidate_construction_families'])} "
        f"edges={len(fixture['dependency_edges'])} "
        f"scenarios={len(fixture['change_scenarios'])} "
        f"home_negative_controls={home_negative_controls} "
        f"jurisdictions={len(capability_registry['capability_cards'])} "
        f"capability_negative_controls={capability_negative_controls} "
        f"source_watch_records={len(source_watch['source_records'])} "
        f"source_watch_negative_controls={source_watch_negative_controls} "
        f"negative_controls={home_negative_controls + capability_negative_controls + source_watch_negative_controls}"
    )
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, json.JSONDecodeError, ValueError, KeyError, TypeError) as error:
        print(f"HOME_FIXTURE_VALIDATION=FAIL {error}", file=sys.stderr)
        sys.exit(1)
