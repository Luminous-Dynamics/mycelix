#!/usr/bin/env python3
"""Standard-library JSON Schema subset and cross-record conformance checker."""

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "fixtures" / "construction-project-trace-v0.1.schema.json"
FIXTURE_PATH = ROOT / "fixtures" / "pavilion-trace-v0.1.json"
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def resolve_ref(root, ref):
    if not ref.startswith("#/"):
        raise ValueError(f"unsupported external schema reference: {ref}")
    value = root
    for token in ref[2:].split("/"):
        value = value[token.replace("~1", "/").replace("~0", "~")]
    return value


def check_schema(value, schema, root, path="$"):
    if "$ref" in schema:
        check_schema(value, resolve_ref(root, schema["$ref"]), root, path)
        return

    expected = schema.get("type")
    allowed = expected if isinstance(expected, list) else [expected] if expected else []
    if allowed:
        kinds = {
            "object": lambda item: isinstance(item, dict),
            "array": lambda item: isinstance(item, list),
            "string": lambda item: isinstance(item, str),
            "integer": lambda item: isinstance(item, int) and not isinstance(item, bool),
            "boolean": lambda item: isinstance(item, bool),
            "null": lambda item: item is None,
        }
        if not any(kinds[kind](value) for kind in allowed):
            raise AssertionError(f"{path}: expected {expected}, got {type(value).__name__}")

    if "const" in schema and value != schema["const"]:
        raise AssertionError(f"{path}: expected constant {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        raise AssertionError(f"{path}: value not in enum")

    if isinstance(value, str):
        if len(value) < schema.get("minLength", 0):
            raise AssertionError(f"{path}: string too short")
        if "pattern" in schema and re.search(schema["pattern"], value) is None:
            raise AssertionError(f"{path}: pattern mismatch")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if value < schema.get("minimum", value):
            raise AssertionError(f"{path}: below minimum")

    if isinstance(value, dict):
        missing = set(schema.get("required", [])) - set(value)
        if missing:
            raise AssertionError(f"{path}: missing properties {sorted(missing)}")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            extra = set(value) - set(properties)
            if extra:
                raise AssertionError(f"{path}: unknown properties {sorted(extra)}")
        for key, child in properties.items():
            if key in value:
                check_schema(value[key], child, root, f"{path}.{key}")

    if isinstance(value, list) and "items" in schema:
        for index, child in enumerate(value):
            check_schema(child, schema["items"], root, f"{path}[{index}]")


def unique_index(items, key, label):
    result = {}
    for item in items:
        identifier = item[key]
        if identifier in result:
            raise AssertionError(f"duplicate {label}: {identifier}")
        result[identifier] = item
    return result


def require_artifact(artifacts, reference, role=None, subject=None):
    if reference not in artifacts:
        raise AssertionError(f"missing source artifact: {reference}")
    artifact = artifacts[reference]
    if role is not None and artifact["artifact_role"] != role:
        raise AssertionError(f"{reference}: expected role {role}")
    if subject is not None and artifact["subject_ref"] != subject:
        raise AssertionError(f"{reference}: expected subject {subject}")
    if not HEX_SHA256.fullmatch(artifact["sha256"]):
        raise AssertionError(f"{reference}: invalid digest")
    return artifact


def require_evidence(evidence, reference, subject, event_at):
    if reference not in evidence:
        raise AssertionError(f"missing evidence: {reference}")
    item = evidence[reference]
    if item["subject_ref"] != subject:
        raise AssertionError(f"{reference}: evidence subject mismatch")
    if item["status"] != "verified" or item["use_permission"] != "permitted":
        raise AssertionError(f"{reference}: evidence is not usable")
    if item["available_at"] > event_at or item["valid_from"] > event_at:
        raise AssertionError(f"{reference}: evidence was not available at decision time")
    if item["valid_until"] is not None and event_at > item["valid_until"]:
        raise AssertionError(f"{reference}: evidence expired at decision time")


def require_authority(authorities, reference, signer, role, scope, event_at):
    if reference not in authorities:
        raise AssertionError(f"missing authority assertion: {reference}")
    item = authorities[reference]
    if (item["signer_id"], item["role"], item["scope_ref"]) != (signer, role, scope):
        raise AssertionError(f"{reference}: authority scope mismatch")
    if item["status"] != "active":
        raise AssertionError(f"{reference}: authority is not active")
    if item["available_at"] > event_at or item["valid_from"] > event_at:
        raise AssertionError(f"{reference}: authority not available at decision time")
    if item["valid_until"] is not None and event_at > item["valid_until"]:
        raise AssertionError(f"{reference}: authority expired at decision time")


def validate_graph(trace):
    artifacts = unique_index(trace["artifact_references"], "reference", "artifact reference")
    evidence = unique_index(trace["evidence_assertions"], "reference", "evidence reference")
    authorities = unique_index(trace["authority_assertions"], "reference", "authority reference")
    approvals = unique_index(trace["approvals"], "approval_id", "approval")
    batches = unique_index(trace["material_batches"], "batch_id", "material batch")
    custody = unique_index(trace["custody_receipts"], "receipt_id", "custody receipt")
    manufacturing = unique_index(trace["manufacturing_receipts"], "manufacturing_id", "manufacturing receipt")
    inspections = unique_index(trace["inspections"], "inspection_id", "inspection receipt")

    require_artifact(artifacts, trace["current_design_artifact_ref"], "design_revision")
    if trace["site_context_artifact_ref"] is not None:
        require_artifact(artifacts, trace["site_context_artifact_ref"], "site_context")
    if trace["portfolio_alternative_artifact_ref"] is not None:
        require_artifact(artifacts, trace["portfolio_alternative_artifact_ref"], "portfolio_alternative")

    accepted_approvals = set()
    for approval in approvals.values():
        require_artifact(artifacts, approval["subject_artifact_ref"], "design_revision")
        if not approval["supporting_artifact_refs"]:
            raise AssertionError(f"{approval['approval_id']}: no supporting engineering artifacts")
        engineering_count = 0
        for reference in approval["supporting_artifact_refs"]:
            artifact = require_artifact(artifacts, reference, subject=approval["subject_artifact_ref"])
            engineering_count += artifact["artifact_role"] == "engineering_receipt"
        if not engineering_count:
            raise AssertionError(f"{approval['approval_id']}: no exact-design engineering receipt")
        require_authority(
            authorities, approval["authority_ref"], approval["signer_id"],
            approval["role"], approval["scope_ref"], approval["decided_at"]
        )
        if approval["decision"] != "affirmed" or any(not item["satisfied"] for item in approval["conditions"]):
            raise AssertionError(f"{approval['approval_id']}: participant did not affirm all conditions")
        if not approval["evidence_refs"]:
            raise AssertionError(f"{approval['approval_id']}: no supporting evidence")
        for reference in approval["evidence_refs"]:
            require_evidence(evidence, reference, approval["scope_ref"], approval["decided_at"])
        if approval["subject_artifact_ref"] == trace["current_design_artifact_ref"]:
            accepted_approvals.add(approval["approval_id"])

    for batch in batches.values():
        if not batch["supplier_assertion"] or not batch["certification_evidence_ref"] or not batch["certification_scope_ref"]:
            raise AssertionError(f"{batch['batch_id']}: supplier assertion lacks independent certification")
        require_evidence(evidence, batch["certification_evidence_ref"], batch["certification_scope_ref"], batch["declared_at"])

    custody_by_batch = {}
    for receipt in custody.values():
        if receipt["batch_id"] not in batches:
            raise AssertionError(f"{receipt['receipt_id']}: unknown batch")
        require_evidence(evidence, receipt["evidence_ref"], receipt["receipt_id"], receipt["occurred_at"])
        custody_by_batch.setdefault(receipt["batch_id"], []).append(receipt)

    for batch_id, batch in batches.items():
        chain = sorted(custody_by_batch.get(batch_id, []), key=lambda item: (item["occurred_at"], item["receipt_id"]))
        if not chain:
            raise AssertionError(f"{batch_id}: custody chain missing")
        if chain[0]["predecessor_receipt_id"] is not None or chain[0]["from_party_id"] != batch["supplier_party_id"]:
            raise AssertionError(f"{batch_id}: invalid first custody receipt")
        for index, receipt in enumerate(chain[1:], start=1):
            previous = chain[index - 1]
            if receipt["predecessor_receipt_id"] != previous["receipt_id"] or receipt["from_party_id"] != previous["to_party_id"]:
                raise AssertionError(f"{batch_id}: broken custody predecessor")
            if receipt["occurred_at"] < previous["occurred_at"]:
                raise AssertionError(f"{batch_id}: custody time reversal")
        if chain[-1]["to_party_id"] != batch["receiving_site_party_id"]:
            raise AssertionError(f"{batch_id}: custody chain misses receiving site")

    for item in manufacturing.values():
        if item["design_artifact_ref"] != trace["current_design_artifact_ref"]:
            raise AssertionError(f"{item['manufacturing_id']}: manufacturing targets non-current design")
        if not any(
            approval["subject_artifact_ref"] == item["design_artifact_ref"]
            and approval["approval_id"] in accepted_approvals
            for approval in approvals.values()
        ):
            raise AssertionError(f"{item['manufacturing_id']}: missing current participant commitment")
        require_artifact(artifacts, item["design_artifact_ref"], "design_revision")
        require_artifact(artifacts, item["manufacturing_artifact_ref"], "manufacturing_record", item["design_artifact_ref"])
        require_artifact(artifacts, item["component_artifact_ref"], "fabricated_component", item["manufacturing_artifact_ref"])
        batch = batches[item["material_batch_id"]]
        last_custody = max(r["occurred_at"] for r in custody_by_batch[item["material_batch_id"]])
        if last_custody > item["manufactured_at"]:
            raise AssertionError(f"{item['manufacturing_id']}: manufacture predates delivery")
        require_authority(
            authorities, item["authority_ref"], item["manufacturer_id"],
            item["role"], item["scope_ref"], item["manufactured_at"]
        )
        if not item["evidence_refs"]:
            raise AssertionError(f"{item['manufacturing_id']}: evidence missing")
        for reference in item["evidence_refs"]:
            require_evidence(evidence, reference, item["scope_ref"], item["manufactured_at"])

    for inspection in inspections.values():
        if inspection["manufacturing_id"] not in manufacturing:
            raise AssertionError(f"{inspection['inspection_id']}: manufacturing record missing")
        manufacturing_item = manufacturing[inspection["manufacturing_id"]]
        if inspection["manufacturing_artifact_ref"] != manufacturing_item["manufacturing_artifact_ref"]:
            raise AssertionError(f"{inspection['inspection_id']}: manufacturing artifact continuity mismatch")
        if inspection["component_artifact_ref"] != manufacturing_item["component_artifact_ref"]:
            raise AssertionError(f"{inspection['inspection_id']}: component identity mismatch")
        require_artifact(artifacts, inspection["manufacturing_artifact_ref"], "manufacturing_record", manufacturing_item["design_artifact_ref"])
        require_artifact(artifacts, inspection["component_artifact_ref"], "fabricated_component", inspection["manufacturing_artifact_ref"])
        require_artifact(artifacts, inspection["inspection_artifact_ref"], "inspection_record", inspection["component_artifact_ref"])
        require_artifact(artifacts, inspection["installation_artifact_ref"], "installation_record", inspection["component_artifact_ref"])
        require_artifact(artifacts, inspection["as_built_artifact_ref"], "as_built_asset", inspection["component_artifact_ref"])
        if inspection["inspected_at"] < manufacturing_item["manufactured_at"]:
            raise AssertionError(f"{inspection['inspection_id']}: inspection predates manufacture")
        require_authority(
            authorities, inspection["authority_ref"], inspection["inspector_id"],
            inspection["role"], inspection["scope_ref"], inspection["inspected_at"]
        )
        if inspection["claimed_outcome"] != "conforming" or not inspection["evidence_refs"]:
            raise AssertionError(f"{inspection['inspection_id']}: no conforming evidence claim")
        for reference in inspection["evidence_refs"]:
            require_evidence(evidence, reference, inspection["scope_ref"], inspection["inspected_at"])


def expect_rejected(label, mutate, fixture, schema):
    candidate = json.loads(json.dumps(fixture))
    mutate(candidate)
    try:
        check_schema(candidate, schema, schema)
        validate_graph(candidate)
    except (AssertionError, KeyError):
        return
    raise AssertionError(f"negative control accepted: {label}")


def main():
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    check_schema(fixture, schema, schema)
    validate_graph(fixture)

    expect_rejected(
        "missing external source subject",
        lambda item: item["artifact_references"][0].pop("subject_ref"),
        fixture, schema
    )
    expect_rejected(
        "wrong engineering receipt design",
        lambda item: next(a for a in item["artifact_references"] if a["reference"] == "sol-atlas:engineering:receipt-r2").update(subject_ref="sol-atlas:artifact:design-r1"),
        fixture, schema
    )
    expect_rejected(
        "wrong material certificate scope",
        lambda item: item["material_batches"][0].update(certification_scope_ref="claim:other-batch"),
        fixture, schema
    )
    expect_rejected(
        "late evidence",
        lambda item: next(e for e in item["evidence_assertions"] if e["reference"] == "evidence:design-r2").update(available_at=999),
        fixture, schema
    )
    expect_rejected(
        "broken custody predecessor",
        lambda item: item["custody_receipts"][1].update(predecessor_receipt_id="missing-receipt"),
        fixture, schema
    )
    expect_rejected(
        "inspection component swap",
        lambda item: item["inspections"][0].update(component_artifact_ref="fabricator:component:other"),
        fixture, schema
    )
    expect_rejected(
        "revoked material certificate",
        lambda item: next(e for e in item["evidence_assertions"] if e["reference"] == "evidence:batch-cert-01").update(status="revoked"),
        fixture, schema
    )
    print("construction provenance fixture: schema + ownership + authority + custody + lifecycle controls passed")


if __name__ == "__main__":
    main()
