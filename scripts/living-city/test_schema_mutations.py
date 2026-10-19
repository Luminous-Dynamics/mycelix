#!/usr/bin/env python3
"""Mutation tests proving selected schema constraints reject targeted bad records.

Python is CI-only validation glue, not a production runtime.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "schemas/living-city-concept-v0.1.schema.json"
FIXTURE_PATH = ROOT / "fixtures/living-city/concepts/johannesburg-highveld-rewoven.v0.1.json"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"FAIL: {message}")


def main() -> None:
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    baseline = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    validator = Draft202012Validator(schema, format_checker=FormatChecker())

    baseline_errors = list(validator.iter_errors(baseline))
    require(not baseline_errors, f"baseline fixture must validate, got {len(baseline_errors)} errors")

    malformed_uri = copy.deepcopy(baseline)
    malformed_uri["sources"][0]["url"] = "not a valid URI with spaces"
    uri_errors = list(validator.iter_errors(malformed_uri))
    require(
        any(list(error.path) == ["sources", 0, "url"] and error.validator == "format"
            for error in uri_errors),
        "malformed source URL was not rejected by URI format assertion",
    )
    print("PASS mutation: malformed source URI rejected")

    unknown_top_level = copy.deepcopy(baseline)
    unknown_top_level["unreviewed_claim"] = "must not silently enter the contract"
    top_errors = list(validator.iter_errors(unknown_top_level))
    require(
        any(error.validator == "additionalProperties" and "unreviewed_claim" in error.message
            for error in top_errors),
        "unknown top-level property was not rejected",
    )
    print("PASS mutation: unknown top-level property rejected")

    unknown_nested = copy.deepcopy(baseline)
    unknown_nested["place"]["invented_precision"] = "unverified"
    nested_errors = list(validator.iter_errors(unknown_nested))
    require(
        any(list(error.path) == ["place"] and error.validator == "additionalProperties"
            and "invented_precision" in error.message for error in nested_errors),
        "unknown nested property was not rejected",
    )
    print("PASS mutation: unknown nested property rejected")
    print("PASS: 1 valid baseline and 3 targeted schema mutations")


if __name__ == "__main__":
    main()
