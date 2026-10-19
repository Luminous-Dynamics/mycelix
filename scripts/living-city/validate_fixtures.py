#!/usr/bin/env python3
"""Fail-closed JSON Schema and negative-control checks for Living-City fixtures.

Python is used only as CI validation glue; this is not a production runtime.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = ROOT / "fixtures/living-city/manifest.v0.1.json"

def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)

def load_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"{path.relative_to(ROOT)} is not readable valid JSON: {exc}")

def expected_required_violation(description: str):
    if description == "climate.profile_status is required":
        return ["climate"], "profile_status"
    if description == "cultural_review.status is required":
        return ["cultural_review"], "status"
    if description == "sources[].rights_status is required":
        return None, "rights_status"
    fail(f"unrecognized expected_violation in manifest: {description!r}")

def main() -> None:
    manifest = load_json(MANIFEST_PATH)
    if manifest.get("schema_dialect") != "https://json-schema.org/draft/2020-12/schema":
        fail("manifest schema_dialect is not Draft 2020-12")
    schema_rel = manifest.get("schema_path")
    if not isinstance(schema_rel, str) or not schema_rel or Path(schema_rel).is_absolute() or ".." in Path(schema_rel).parts:
        fail(f"schema_path must be a safe repository-relative path: {schema_rel!r}")
    schema_path = ROOT / schema_rel
    if not schema_path.is_file():
        fail(f"schema file missing: {schema_rel!r}")
    schema = load_json(schema_path)
    try:
        Draft202012Validator.check_schema(schema)
    except Exception as exc:
        fail(f"schema does not validate against Draft 2020-12 meta-schema: {exc}")

    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    fixtures = manifest.get("fixtures")
    if not isinstance(fixtures, list) or not fixtures:
        fail("manifest fixtures must be a non-empty array")

    seen_paths = set()
    valid_count = invalid_count = 0
    for entry in fixtures:
        if not isinstance(entry, dict):
            fail(f"manifest fixture entry must be an object: {entry!r}")
        rel = entry.get("path")
        if not isinstance(rel, str) or rel in seen_paths:
            fail(f"fixture path missing, invalid, or duplicated: {rel!r}")
        rel_path = Path(rel)
        if rel_path.is_absolute() or ".." in rel_path.parts:
            fail(f"fixture path must stay within repository: {rel!r}")
        seen_paths.add(rel)
        path = ROOT / rel
        if not path.is_file():
            fail(f"fixture file missing: {rel}")
        instance = load_json(path)
        errors = sorted(validator.iter_errors(instance), key=lambda e: (list(map(str, e.path)), e.message))

        if entry.get("expect_valid") is True:
            if errors:
                details = "; ".join(f"{list(e.path)}: {e.message}" for e in errors)
                fail(f"positive fixture {rel} rejected: {details}")
            if instance.get("concept_id") != entry.get("concept_id"):
                fail(f"positive fixture {rel} concept_id does not match manifest")
            print(f"PASS positive: {rel}")
            valid_count += 1
            continue

        if entry.get("expect_valid") is not False:
            fail(f"{rel}: expect_valid must be boolean")
        description = entry.get("expected_violation")
        if not isinstance(description, str):
            fail(f"negative fixture {rel} lacks expected_violation")
        expected_parent, expected_field = expected_required_violation(description)
        matching = []
        for error in errors:
            if error.validator != "required" or expected_field not in error.validator_value:
                continue
            if expected_parent is not None and list(error.path) != expected_parent:
                continue
            if expected_parent is None:
                if len(error.path) != 2 or error.path[0] != "sources" or not isinstance(error.path[1], int):
                    continue
            matching.append(error)
        if len(matching) != 1:
            details = "; ".join(f"{list(e.path)} [{e.validator}]: {e.message}" for e in errors) or "no validation errors"
            fail(f"negative fixture {rel} did not fail for its intended required field ({description}): {details}")
        if len(errors) != 1:
            details = "; ".join(f"{list(e.path)} [{e.validator}]: {e.message}" for e in errors)
            fail(f"negative fixture {rel} has unexpected additional validation errors: {details}")
        print(f"PASS negative control: {rel} -> {description}")
        invalid_count += 1

    discovered_paths = {
        path.relative_to(ROOT).as_posix()
        for folder in (ROOT / "fixtures/living-city/concepts", ROOT / "fixtures/living-city/invalid")
        if folder.is_dir()
        for path in folder.rglob("*.json")
    }
    if discovered_paths != seen_paths:
        missing_from_manifest = sorted(discovered_paths - seen_paths)
        missing_from_disk = sorted(seen_paths - discovered_paths)
        fail(f"manifest/disk fixture drift: unlisted files={missing_from_manifest}; missing files={missing_from_disk}")

    if valid_count == 0 or invalid_count == 0:
        fail(f"need positive and negative controls; found {valid_count} positive, {invalid_count} negative")
    print(f"PASS: Draft 2020-12 schema; {valid_count} positive fixtures; {invalid_count} intended negative controls; URI format assertion enabled.")

if __name__ == "__main__":
    main()
