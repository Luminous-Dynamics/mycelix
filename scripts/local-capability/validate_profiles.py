#!/usr/bin/env python3
"""Fail-closed validation for Local Capability Profile schema and semantic controls.

Python is CI validation glue only; this script is not a production service.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = ROOT / "fixtures/local-capability/manifest.v0.1.json"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"{path.relative_to(ROOT)} is not readable valid JSON: {exc}")


def semantic_errors(profile: dict) -> list[str]:
    errors: list[str] = []
    engagement = profile["engagement"]
    mode = engagement["mode"]
    status = profile["status"]
    source_ids = [source["source_id"] for source in profile["sources"]]

    if len(source_ids) != len(set(source_ids)):
        errors.append("duplicate_source_id")

    evidence_refs: list[str] = []
    evidence_refs.extend(profile["context"]["climate_profile"]["evidence_refs"])
    for observation in profile["resource_envelope"].values():
        evidence_refs.extend(observation["evidence_refs"])
    for capability in profile["capabilities"]:
        evidence_refs.extend(capability["evidence_refs"])
    for priority in profile["priorities"]:
        evidence_refs.extend(priority["evidence_refs"])
    if any(reference not in source_ids for reference in evidence_refs):
        errors.append("dangling_evidence_reference")

    if status in {"community_reviewed", "pilot_tested", "measured_outcomes"}:
        if (
            mode not in {"community_requested", "community_co_designed"}
            or engagement["authorization_status"] != "granted"
            or profile["safety_gates"]["community_consent"] != "passed"
        ):
            errors.append("community_maturity_requires_consent")

    if status == "measured_outcomes" and not any(
        source["use"] == "measured_outcome" for source in profile["sources"]
    ):
        errors.append("measured_outcomes_requires_source")

    if any(
        priority["basis"] in {"community_stated", "co_designed"}
        for priority in profile["priorities"]
    ) and (
        mode not in {"community_requested", "community_co_designed"}
        or engagement["authorization_status"] != "granted"
    ):
        errors.append("community_priority_requires_authorization")

    if mode == "protective_no_contact":
        if (
            profile["place"]["scope"] != "generalized_context"
            or profile["place"]["location_precision"] != "not_collected"
        ):
            errors.append("no_contact_specific_location")

        climate = profile["context"]["climate_profile"]
        has_resource_claims = any(
            observation["value"] != "unknown"
            or observation["basis"] != "unknown"
            or observation["evidence_refs"]
            for observation in profile["resource_envelope"].values()
        )
        has_local_context = (
            profile["status"] != "illustrative"
            or profile["place"]["basis"] != "illustrative"
            or profile["context"]["settlement_type"] != "generalized_context"
            or climate["classification"] != "unknown"
            or climate["basis"] != "unknown"
            or climate["evidence_refs"]
            or climate["hazards_to_investigate"]
            or has_resource_claims
            or profile["capabilities"]
            or profile["priorities"]
            or profile["sources"]
            or engagement["restricted_knowledge_present"]
        )
        if has_local_context:
            errors.append("no_contact_local_context")

        if (
            engagement["authorization_status"] != "not_sought"
            or engagement["location_handling"] != "not_collected"
            or engagement["intervention_policy"] != "no_intervention_without_authorized_request"
            or profile["safety_gates"]["community_consent"] != "not_assessed"
        ):
            errors.append("no_contact_must_not_authorize_intervention")

    if mode != "protective_no_contact" and not profile["priorities"]:
        errors.append("profile_requires_priorities")
    if mode == "protective_no_contact" and profile["priorities"]:
        errors.append("no_contact_priorities_prohibited")

    return errors


def main() -> None:
    manifest = load_json(MANIFEST_PATH)
    if manifest.get("schema_dialect") != "https://json-schema.org/draft/2020-12/schema":
        fail("manifest schema_dialect is not Draft 2020-12")
    schema_rel = manifest.get("schema_path")
    if (
        not isinstance(schema_rel, str)
        or not schema_rel
        or Path(schema_rel).is_absolute()
        or ".." in Path(schema_rel).parts
    ):
        fail(f"schema_path must be a safe repository-relative path: {schema_rel!r}")

    schema_path = ROOT / schema_rel
    if not schema_path.is_file():
        fail(f"schema file missing: {schema_rel!r}")
    schema = load_json(schema_path)
    try:
        Draft202012Validator.check_schema(schema)
    except Exception as exc:
        fail(f"schema fails Draft 2020-12 meta-schema: {exc}")

    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    fixtures = manifest.get("fixtures")
    if not isinstance(fixtures, list) or not fixtures:
        fail("manifest fixtures must be a non-empty array")

    seen_paths: set[str] = set()
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
        profile = load_json(path)
        schema_errors = sorted(
            validator.iter_errors(profile),
            key=lambda error: (list(map(str, error.path)), error.message),
        )
        if schema_errors:
            details = "; ".join(
                f"{list(error.path)} [{error.validator}]: {error.message}"
                for error in schema_errors
            )
            fail(f"fixture {rel} fails schema validation: {details}")
        if profile.get("profile_id") != entry.get("profile_id"):
            fail(f"fixture {rel} profile_id does not match manifest")

        errors = semantic_errors(profile)
        if entry.get("expect_valid") is True:
            if errors:
                fail(f"positive fixture {rel} has semantic errors: {errors}")
            print(f"PASS positive: {rel}")
            valid_count += 1
            continue

        if entry.get("expect_valid") is not False:
            fail(f"{rel}: expect_valid must be boolean")
        expected = entry.get("expected_violation")
        if not isinstance(expected, str):
            fail(f"{rel}: expected_violation must be a string")
        if errors != [expected]:
            fail(
                f"negative fixture {rel} expected exactly [{expected!r}], "
                f"observed {errors!r}"
            )
        print(f"PASS negative control: {rel} -> {expected}")
        invalid_count += 1

    discovered_paths = {
        path.relative_to(ROOT).as_posix()
        for folder in (
            ROOT / "fixtures/local-capability/profiles",
            ROOT / "fixtures/local-capability/invalid",
        )
        if folder.is_dir()
        for path in folder.rglob("*.json")
    }
    if discovered_paths != seen_paths:
        fail(
            "manifest/disk fixture drift: "
            f"unlisted files={sorted(discovered_paths - seen_paths)}; "
            f"missing files={sorted(seen_paths - discovered_paths)}"
        )

    if valid_count == 0 or invalid_count == 0:
        fail(f"need positive and negative controls; found {valid_count} positive, {invalid_count} negative")
    print(
        "PASS: Draft 2020-12 schema; "
        f"{valid_count} positive profiles; {invalid_count} intended semantic negative controls; "
        "URI format checking enabled."
    )


if __name__ == "__main__":
    main()
