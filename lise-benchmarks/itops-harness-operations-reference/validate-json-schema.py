#!/usr/bin/env python3
"""Validate the public ITOPS harness-operations fixture against Draft 2020-12."""

from __future__ import annotations

import json
import sys
from importlib.metadata import version
from pathlib import Path

from jsonschema import Draft202012Validator


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: validate-json-schema.py SCHEMA INSTANCE", file=sys.stderr)
        return 2

    schema_path = Path(sys.argv[1])
    instance_path = Path(sys.argv[2])
    try:
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        instance = json.loads(instance_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"schema/input read failed: {exc}", file=sys.stderr)
        return 2

    installed = version("jsonschema")
    if installed != "4.26.0":
        print(
            f"unsupported jsonschema version {installed}; expected 4.26.0",
            file=sys.stderr,
        )
        return 2

    try:
        Draft202012Validator.check_schema(schema)
    except Exception as exc:  # jsonschema raises several schema-validation exceptions
        print(f"Draft 2020-12 schema is invalid: {exc}", file=sys.stderr)
        return 1

    validator = Draft202012Validator(schema)
    errors = sorted(
        validator.iter_errors(instance),
        key=lambda error: (list(map(str, error.absolute_path)), error.message),
    )
    if errors:
        for error in errors:
            location = "/" + "/".join(map(str, error.absolute_path))
            print(f"INVALID {location}: {error.message}", file=sys.stderr)
        return 1

    print(
        "PASS: Draft 2020-12 schema meta-validation and instance conformance "
        f"(jsonschema {installed})."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
