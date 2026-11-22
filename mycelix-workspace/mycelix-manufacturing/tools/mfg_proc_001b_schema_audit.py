#!/usr/bin/env python3
"""Test-only Draft 2020-12 conformance runner for MFG-PROC-001B.

This does not participate in production routing, identity, availability, or qualification.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "schemas/mfg_proc_001b/manufacturing_resource_offer_v0_1.schema.json"
EXAMPLE_PATH = ROOT / "tests/fixtures/mfg_proc_001b_resource_offer_example_v0_1.json"
FIXTURE_PATH = ROOT / "tests/fixtures/mfg_proc_001b_resource_offers_v0_1.json"


def read_json(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def set_pointer(document: dict, pointer: str, value) -> None:
    if not pointer.startswith("/"):
        raise ValueError(f"unsupported JSON pointer: {pointer}")
    pieces = [part.replace("~1", "/").replace("~0", "~") for part in pointer[1:].split("/")]
    target = document
    for part in pieces[:-1]:
        target = target[part]
    target[pieces[-1]] = copy.deepcopy(value)


def apply_mutations(base: dict, mutations: list[dict]) -> dict:
    result = copy.deepcopy(base)
    for mutation in mutations:
        if mutation.get("op") != "replace":
            raise ValueError(f"unsupported mutation operation: {mutation}")
        set_pointer(result, mutation["path"], mutation["value"])
    return result


def expect_invalid(validator, document: dict, label: str) -> None:
    try:
        validator.validate(document)
    except ValidationError:
        return
    raise AssertionError(f"negative schema control was accepted: {label}")


def main() -> None:
    schema = read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema)

    example = read_json(EXAMPLE_PATH)
    validator.validate(example)

    corpus = read_json(FIXTURE_PATH)
    base = corpus["base_offer"]
    validator.validate(base)
    if len(corpus["cases"]) != 16:
        raise AssertionError("expected exactly 16 frozen resource-offer cases")

    for case in corpus["cases"]:
        candidate = apply_mutations(base, case.get("mutations", []))
        validator.validate(candidate)
        for additional in case.get("additional_records", []):
            record = copy.deepcopy(base if additional.get("base_offer") else additional.get("record", {}))
            for pointer, value in additional.get("overrides", {}).items():
                set_pointer(record, pointer, value)
            validator.validate(record)

    negative = [
        ("provider ref missing", lambda d: d.pop("provider_ref")),
        ("unknown availability boolean", lambda d: d.update({"available": True})),
        ("provider asserts qualified", lambda d: d.update({"qualified": True})),
        ("provider asserts reserved", lambda d: d.update({"reserved": True})),
        ("provider-set lifecycle state Expired", lambda d: d.update({"publication_state": "Expired"})),
        ("provider-set lifecycle state Disputed", lambda d: d.update({"publication_state": "Disputed"})),
        ("provider-set lifecycle state Superseded", lambda d: d.update({"publication_state": "Superseded"})),
        ("verified declaration self-assertion", lambda d: d["declaration"].update({"verification_state": "verified"})),
        ("capacity zero", lambda d: d["declared_capacity"].update({"quantity": 0})),
        ("malformed digest", lambda d: d["resource_ref"].update({"content_digest_sha256": "no-digest"})),
        ("extra execution assertion", lambda d: d.update({"executed": True})),
    ]
    for label, mutate in negative:
        candidate = copy.deepcopy(base)
        mutate(candidate)
        expect_invalid(validator, candidate, label)

    print(
        "MFG-PROC-001B_SCHEMA_AUDIT "
        f"cases={len(corpus['cases'])} "
        f"schema_negative_controls={len(negative)} "
        "example=valid base_offer=valid lifecycle=derived"
    )


if __name__ == "__main__":
    main()
