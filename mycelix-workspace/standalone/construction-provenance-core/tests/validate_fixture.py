#!/usr/bin/env python3
"""Standard-library fixture validator for the declared JSON Schema keyword subset."""

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "fixtures" / "construction-project-trace-v0.1.schema.json"
FIXTURE_PATH = ROOT / "fixtures" / "pavilion-trace-v0.1.json"


def resolve_ref(root, ref):
    if not ref.startswith("#/"):
        raise ValueError(f"unsupported external schema reference: {ref}")
    value = root
    for token in ref[2:].split("/"):
        value = value[token.replace("~1", "/").replace("~0", "~")]
    return value


def check(value, schema, root, path="$"):
    if "$ref" in schema:
        check(value, resolve_ref(root, schema["$ref"]), root, path)
        return
    expected = schema.get("type")
    allowed = expected if isinstance(expected, list) else [expected] if expected else []
    if allowed:
        kinds = {
            "object": lambda v: isinstance(v, dict),
            "array": lambda v: isinstance(v, list),
            "string": lambda v: isinstance(v, str),
            "integer": lambda v: isinstance(v, int) and not isinstance(v, bool),
            "boolean": lambda v: isinstance(v, bool),
            "null": lambda v: v is None,
        }
        if not any(kinds[k](value) for k in allowed):
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
    if isinstance(value, int) and not isinstance(value, bool):
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
                check(value[key], child, root, f"{path}.{key}")
    if isinstance(value, list) and "items" in schema:
        for index, child in enumerate(value):
            check(child, schema["items"], root, f"{path}[{index}]")


def main():
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    check(fixture, schema, schema)

    assert fixture["schema_version"] == 1
    assert fixture["current_design_revision_id"] == "design-r2"
    assert fixture["site_context"]["owner_system"] == "sol-atlas-leptos"
    assert fixture["site_context"]["artifact_role"] == "site_context"
    assert fixture["external_portfolio_context"]["owner_system"] == "sol-atlas-leptos"
    assert fixture["external_portfolio_context"]["artifact_role"] == "portfolio_alternative"
    assert len(fixture["design_revisions"]) == 2
    assert fixture["design_revisions"][1]["supersedes_revision_id"] == "design-r1"
    for design in fixture["design_revisions"]:
        assert design["source_artifact"]["owner_system"] == "symthaea"
        assert design["source_artifact"]["artifact_role"] == "design_revision"
        assert design["source_artifact"]["revision_ref"] == design["revision_id"]
        assert design["source_artifact"]["sha256"] == design["sha256"]
        assert design["ifc_model_export"]["artifact_role"] == "ifc_model_export"
        assert design["ifc_model_export"]["revision_ref"] == design["revision_id"]
        assert design["drawing_package"]["artifact_role"] == "drawing_package"
        assert design["drawing_package"]["revision_ref"] == design["revision_id"]
    for run in fixture["simulation_runs"]:
        assert run["execution_artifact"]["owner_system"] == "symtropy"
        assert run["execution_artifact"]["artifact_role"] == "simulation_run"
        assert run["execution_artifact"]["revision_ref"] == run["run_id"]
        assert run["execution_artifact"]["sha256"] == run["result_sha256"]
    assert fixture["approvals"][1]["simulation_run_id"] == "sim-r2"
    assert fixture["approvals"][1]["design_sha256"] == fixture["design_revisions"][1]["sha256"]
    evidence_by_ref = {item["reference"]: item for item in fixture["evidence_assertions"]}
    assert fixture["material_batches"][0]["supplier_assertion"] is not None
    assert fixture["material_batches"][0]["certification_evidence_ref"] is not None
    batch = fixture["material_batches"][0]
    assert evidence_by_ref[batch["certification_evidence_ref"]]["subject_ref"] == batch["certification_scope_ref"]
    for approval in fixture["approvals"]:
        for reference in approval["evidence_refs"]:
            assert evidence_by_ref[reference]["subject_ref"] == approval["scope_ref"]
    for receipt in fixture["custody_receipts"]:
        assert evidence_by_ref[receipt["evidence_ref"]]["subject_ref"] == receipt["receipt_id"]
    for inspection in fixture["inspections"]:
        for reference in inspection["evidence_refs"]:
            assert evidence_by_ref[reference]["subject_ref"] == inspection["inspection_id"]
    assert fixture["custody_receipts"][1]["predecessor_receipt_id"] == fixture["custody_receipts"][0]["receipt_id"]
    assert fixture["inspections"][0]["terminal_custody_receipt_id"] == fixture["custody_receipts"][-1]["receipt_id"]

    # Adversarial shape controls: reject omitted mandatory identity and malformed digest.
    broken = json.loads(json.dumps(fixture))
    del broken["approvals"][1]["scope_ref"]
    try:
        check(broken, schema, schema)
    except AssertionError:
        pass
    else:
        raise AssertionError("missing approval scope was accepted")

    broken = json.loads(json.dumps(fixture))
    broken["design_revisions"][1]["sha256"] = "not-a-digest"
    try:
        check(broken, schema, schema)
    except AssertionError:
        pass
    else:
        raise AssertionError("malformed design digest was accepted")

    print("construction trace fixture: schema subset + cross-record controls passed")


if __name__ == "__main__":
    main()
