#!/usr/bin/env python3
"""Validate the Mycelix food traceability V1 schema and fail-closed invariants."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "docs/food/schemas/food-traceability-event.v1.schema.json"
EXAMPLE_PATH = ROOT / "docs/food/examples/eggs-produced.v1.json"


def main() -> int:
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    example = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))

    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema, format_checker=FormatChecker())

    def valid(name: str, instance: dict) -> None:
        errors = sorted(validator.iter_errors(instance), key=lambda err: list(map(str, err.path)))
        if errors:
            formatted = "\n".join(f"  {list(err.path)}: {err.message}" for err in errors)
            raise AssertionError(f"{name} should be valid:\n{formatted}")
        print(f"PASS valid: {name}")

    def invalid(name: str, instance: dict) -> None:
        errors = list(validator.iter_errors(instance))
        if not errors:
            raise AssertionError(f"{name} should be invalid but validated successfully")
        print(f"PASS rejects: {name}")

    valid("canonical synthetic egg lot", example)

    unknown = copy.deepcopy(example)
    unknown["subjects"][0]["quantity"]["status"] = "unknown"
    unknown["subjects"][0]["quantity"].pop("value")
    valid("unknown amount carries no numeric value", unknown)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["status"] = "unknown"
    invalid("unknown amount cannot smuggle a numeric value", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "-1"
    invalid("negative amount", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "1e3"
    invalid("exponent notation is not canonical decimal", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "NaN"
    invalid("NaN is not a quantity", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCode"] = "dzn"
    invalid("unit codes use canonical uppercase syntax", bad)

    bad = copy.deepcopy(example)
    bad["unexpected"] = "must not be silently accepted"
    invalid("unknown top-level property", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "PLANNED"
    invalid("planned event cannot use observed quantity state", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "FORECASTED"
    invalid("forecast event cannot use observed quantity state", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["status"] = "forecast"
    invalid("observed production event cannot carry forecast quantity state", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "TRANSFORMED"
    invalid("transformation requires explicit input and output lots", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "HANDLING_OBSERVED"
    invalid("handling event requires handling observations", bad)

    handling_base = copy.deepcopy(example)
    handling_base["eventType"] = "HANDLING_OBSERVED"
    handling = {
        "parameter": "temperature",
        "status": "observed",
        "value": "4.2",
        "observedAt": "2026-10-12T06:01:00+02:00",
        "receivedAt": "2026-10-12T06:01:05+02:00",
        "sensorRef": "urn:mycelix:sensor:cold-room-01",
        "calibrationStatus": "current",
        "unitCode": "CEL",
        "unitCodeSystem": "UNECE_REC20",
        "unitCodeListVersion": "REC20-REV17-2021",
    }
    handling_base["handlingObservations"] = [handling]
    valid("well-formed observed handling measurement", handling_base)

    bad = copy.deepcopy(handling_base)
    bad["handlingObservations"][0].pop("value")
    invalid("observed handling measurement requires a value", bad)

    bad = copy.deepcopy(handling_base)
    bad["handlingObservations"][0]["status"] = "missing"
    invalid("missing handling sample cannot claim a value", bad)

    bad = copy.deepcopy(example)
    bad["evidenceRefs"][0]["status"] = "verified"
    invalid("verified evidence requires verifier and procedure references", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "CORRECTION"
    invalid("correction requires supersession link and reason", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "RETRACTION"
    bad["supersedesEventId"] = example["eventId"]
    invalid("retraction requires an explicit reason", bad)

    print("PASS: JSON Schema draft 2020-12 meta-validation and 17 contract cases")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, json.JSONDecodeError, OSError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
