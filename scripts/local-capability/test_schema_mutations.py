#!/usr/bin/env python3
"""Targeted negative controls for the Local Capability Profile schema."""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "schemas/local-capability-profile-v0.1.schema.json"
FIXTURE_PATH = ROOT / "fixtures/local-capability/profiles/semi-arid-low-cash.json"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    with SCHEMA_PATH.open("r", encoding="utf-8") as stream:
        schema = json.load(stream)
    with FIXTURE_PATH.open("r", encoding="utf-8") as stream:
        baseline = json.load(stream)

    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    if list(validator.iter_errors(baseline)):
        fail("mutation baseline is not schema-valid")

    mutations = {
        "unknown top-level property": lambda value: value.update({"unreviewed_score": 1}),
        "unexpected precise coordinate": lambda value: value["place"].update({"latitude": -26.2}),
        "unknown nested property": lambda value: value["resource_envelope"]["electricity"].update({"vendor_part": "x"}),
        "invalid resource enum substitution": lambda value: value["resource_envelope"]["electricity"].update({"value": "industrial_supply_chain"}),
        "malformed source URI": lambda value: value["sources"][0].update({"url": "not a URI"}),
        "wrong-typed evidence references": lambda value: value["priorities"][0].update({"evidence_refs": "source-id"}),
    }
    for name, mutate in mutations.items():
        candidate = copy.deepcopy(baseline)
        mutate(candidate)
        errors = list(validator.iter_errors(candidate))
        if not errors:
            fail(f"mutation was unexpectedly accepted: {name}")
        print(f"PASS mutation control: {name}")

    print(f"PASS: {len(mutations)} independent schema mutations rejected.")


if __name__ == "__main__":
    main()
