#!/usr/bin/env python3
"""Test-only Draft 2020-12 schema audit for MFG-PROC-001B.

This script validates contract structure only. It does not fetch owner records,
verify signers, resolve capacity/calendar references, or qualify production.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError
from referencing import Registry, Resource

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_DIR = ROOT / "schemas"
OFFER_SCHEMA_PATH = SCHEMA_DIR / "mfg_proc_001b" / "manufacturing_resource_offer_v0_1.schema.json"
PROCESS_SCHEMA_PATH = SCHEMA_DIR / "mfg_proc_001a" / "typed_process_refs_v0_1.schema.json"
FIXTURE_PATH = ROOT / "tests" / "fixtures" / "mfg_proc_001b_resource_offer_v0_1.json"


def read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise AssertionError(f"expected JSON object in {path}")
    return value


def expect_invalid(validator: Draft202012Validator, value: dict, label: str) -> None:
    try:
        validator.validate(value)
    except ValidationError:
        return
    raise AssertionError(f"negative schema control was accepted: {label}")


def main() -> None:
    offer_schema = read_json(OFFER_SCHEMA_PATH)
    process_schema = read_json(PROCESS_SCHEMA_PATH)
    Draft202012Validator.check_schema(offer_schema)
    Draft202012Validator.check_schema(process_schema)

    registry = (
        Registry()
        .with_resource(offer_schema["$id"], Resource.from_contents(offer_schema))
        .with_resource(process_schema["$id"], Resource.from_contents(process_schema))
    )
    validator = Draft202012Validator(offer_schema, registry=registry)

    fixture = read_json(FIXTURE_PATH)
    template = fixture["template_offer"]
    cases = fixture["cases"]
    if fixture.get("case_count") != len(cases) or len(cases) != 17:
        raise AssertionError("expected exactly 17 frozen, uniquely identified cases")
    case_ids = [case.get("id") for case in cases]
    if len(set(case_ids)) != len(case_ids):
        raise AssertionError("fixture case IDs must be unique")

    validator.validate(template)
    if template.get("publication_state") not in {"Published", "Withdrawn"}:
        raise AssertionError("template contains a non-provider publication state")
    if template.get("lifecycle_event_refs") != []:
        raise AssertionError("template must explicitly carry an empty lifecycle-event reference set")

    negative_controls = [
        ("missing stable offer ID", lambda d: d.pop("offer_id")),
        ("unknown availability boolean", lambda d: d.update({"available_now": True})),
        ("provider self-asserts qualified capability", lambda d: d.update({"qualified_for_production": True})),
        ("provider self-asserts reservation", lambda d: d.update({"reserved_capacity": True})),
        ("provider self-asserts execution", lambda d: d.update({"executed_output": True})),
        ("provider self-asserts Disputed", lambda d: d.update({"publication_state": "Disputed"})),
        ("provider self-asserts Superseded", lambda d: d.update({"publication_state": "Superseded"})),
        ("invalid publication enum", lambda d: d.update({"publication_state": "Expired"})),
        ("missing lifecycle event set", lambda d: d.pop("lifecycle_event_refs")),
        ("malformed owner digest", lambda d: d["resource_subject_ref"].update({"payload_sha256": "bad"})),
        ("zero-length/empty offer identity", lambda d: d.update({"offer_id": ""})),
    ]
    for label, mutate in negative_controls:
        candidate = copy.deepcopy(template)
        mutate(candidate)
        expect_invalid(validator, candidate, label)

    print(
        "MFG_PROC_001B_SCHEMA_AUDIT "
        f"cases={len(cases)} negative_controls={len(negative_controls)} "
        "positive_template=valid publication_states=Published|Withdrawn"
    )


if __name__ == "__main__":
    main()
