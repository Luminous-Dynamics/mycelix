#!/usr/bin/env python3
"""Fail-closed source/evidence structure and reference checks. CI-only, no network access."""
from __future__ import annotations
import copy
import json
import sys
from datetime import date
from pathlib import Path
from urllib.parse import urlsplit
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SOURCE_SCHEMA_PATH = ROOT / "schemas/living-city-source-v0.1.schema.json"
EVIDENCE_SCHEMA_PATH = ROOT / "schemas/living-city-evidence-v0.1.schema.json"
SOURCE_FIXTURE_PATH = ROOT / "fixtures/living-city/evidence/sources.v0.1.json"
CLAIMS_DIR = ROOT / "fixtures/living-city/evidence/claims"
CONCEPT_MANIFEST_PATH = ROOT / "fixtures/living-city/manifest.v0.1.json"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"{path.relative_to(ROOT)} is not readable valid JSON: {exc}")


def errors_for(validator: Draft202012Validator, instance: dict):
    return sorted(validator.iter_errors(instance), key=lambda e: (list(map(str, e.path)), e.message))


def describe(errors) -> str:
    return "; ".join(f"{list(e.path)} [{e.validator}]: {e.message}" for e in errors) or "no validation errors"


def safe_https_uri(value: str) -> bool:
    try:
        parts = urlsplit(value)
        return parts.scheme == "https" and bool(parts.hostname) and parts.username is None and parts.password is None
    except ValueError:
        return False


def temporal_order(start, end, context: str) -> list[str]:
    if start is not None and end is not None and date.fromisoformat(start) > date.fromisoformat(end):
        return [f"{context}: start date is after end date"]
    return []

def resolution_errors(value, qualifier: str, context: str) -> list[str]:
    if value is None and qualifier in ("exact", "approximate"):
        return [f"{context}: resolution qualifier mismatch; numeric value required"]
    if value is not None and qualifier in ("unknown", "not_applicable"):
        return [f"{context}: resolution qualifier mismatch; qualifier conflicts with numeric value"]
    return []


def source_semantic_errors(source: dict) -> list[str]:
    errors = []
    geo = source["geographic_coverage"]
    errors.extend(resolution_errors(geo["spatial_resolution_m"], geo["spatial_resolution_qualifier"], source["source_id"]))
    if not safe_https_uri(source["landing_url"]):
        errors.append(f"{source['source_id']}: landing_url must be HTTPS without embedded credentials")
    if source["reuse"]["status"] == "open_license_verified" and not source["reuse"]["license_url"]:
        errors.append(f"{source['source_id']}: open_license_verified requires a license_url")
    errors.extend(temporal_order(source["temporal_coverage"]["start_date"], source["temporal_coverage"]["end_date"], source["source_id"]))
    return errors


def claim_semantic_errors(claim: dict, sources_by_id: dict, concepts_by_key: dict) -> list[str]:
    errors = []
    for ref in claim["source_refs"]:
        if ref not in sources_by_id:
            errors.append(f"{claim['evidence_id']}: unknown source_ref {ref!r}")
    cref = claim["concept_ref"]
    key = (cref["concept_id"], cref["concept_version"])
    concept = concepts_by_key.get(key)
    if concept is None:
        errors.append(f"{claim['evidence_id']}: unresolved concept/version {key!r}")
    elif cref["hypothesis_id"] not in {x["hypothesis_id"] for x in concept["design_hypotheses"]}:
        errors.append(f"{claim['evidence_id']}: unresolved hypothesis {cref['hypothesis_id']!r} for {key!r}")
    coverage = claim["coverage"]
    errors.extend(resolution_errors(coverage["spatial_resolution_m"], coverage["spatial_resolution_qualifier"], claim["evidence_id"]))
    errors.extend(temporal_order(coverage["temporal_start"], coverage["temporal_end"], claim["evidence_id"]))
    method = claim["method"]
    if claim["evidence_kind"] == "simulation" and (method["kind"] != "simulation" or not method["model_or_protocol_version"]):
        errors.append(f"{claim['evidence_id']}: simulation requires simulation method and explicit model_or_protocol_version")
    if claim["evidence_kind"] == "observed_measurement":
        if method["kind"] != "instrument_measurement":
            errors.append(f"{claim['evidence_id']}: observed_measurement requires instrument_measurement method")
        if coverage["temporal_start"] is None or coverage["temporal_end"] is None:
            errors.append(f"{claim['evidence_id']}: observed_measurement requires a documented temporal window")
    review = claim["review"]
    if review["status"] == "reviewed" and (not review["reviewer_ref"] or not review["decided_at"]):
        errors.append(f"{claim['evidence_id']}: reviewed status requires reviewer_ref and decided_at")
    uncertainty = claim["uncertainty"]
    bounds = uncertainty["bounds"]
    if uncertainty["classification"] == "source_reported_range":
        if bounds is None:
            errors.append(f"{claim['evidence_id']}: source_reported_range requires numeric bounds")
        elif bounds["lower"] > bounds["upper"]:
            errors.append(f"{claim['evidence_id']}: uncertainty lower bound exceeds upper bound")
    if claim["evidence_id"] in claim["supersedes"]:
        errors.append(f"{claim['evidence_id']}: record cannot supersede itself")
    return errors


def main() -> None:
    source_schema = load_json(SOURCE_SCHEMA_PATH)
    evidence_schema = load_json(EVIDENCE_SCHEMA_PATH)
    for name, schema in [("source", source_schema), ("evidence", evidence_schema)]:
        try:
            Draft202012Validator.check_schema(schema)
        except Exception as exc:
            fail(f"{name} schema fails Draft 2020-12 meta-schema: {exc}")
    source_v = Draft202012Validator(source_schema, format_checker=FormatChecker())
    evidence_v = Draft202012Validator(evidence_schema, format_checker=FormatChecker())

    manifest = load_json(CONCEPT_MANIFEST_PATH)
    concepts = {}
    for entry in manifest.get("fixtures", []):
        if entry.get("expect_valid") is True:
            item = load_json(ROOT / entry["path"])
            key = (item["concept_id"], item["version"])
            if key in concepts:
                fail(f"duplicate concept version in concept fixtures: {key!r}")
            concepts[key] = item

    sources = load_json(SOURCE_FIXTURE_PATH)
    if not isinstance(sources, list) or not sources:
        fail("source fixture must be a non-empty JSON array")
    by_source = {}
    for index, source in enumerate(sources):
        errors = errors_for(source_v, source)
        if errors:
            fail(f"source fixture index {index} invalid: {describe(errors)}")
        sid = source["source_id"]
        if sid in by_source:
            fail(f"duplicate source_id: {sid!r}")
        semantic = source_semantic_errors(source)
        if semantic:
            fail("; ".join(semantic))
        by_source[sid] = source
        print(f"PASS source: {sid}")

    paths = sorted(CLAIMS_DIR.glob("*.json"))
    if not paths:
        fail("no evidence claim fixtures found")
    by_evidence = set()
    claims = []
    for path in paths:
        claim = load_json(path)
        errors = errors_for(evidence_v, claim)
        if errors:
            fail(f"claim {path.relative_to(ROOT)} invalid: {describe(errors)}")
        eid = claim["evidence_id"]
        if eid in by_evidence:
            fail(f"duplicate evidence_id: {eid!r}")
        semantic = claim_semantic_errors(claim, by_source, concepts)
        if semantic:
            fail("; ".join(semantic))
        by_evidence.add(eid)
        claims.append(claim)
        print(f"PASS claim: {eid}")

    bad_uri = copy.deepcopy(sources[0])
    bad_uri["landing_url"] = "not a URI with spaces"
    errors = errors_for(source_v, bad_uri)
    if not any(list(e.path) == ["landing_url"] and e.validator == "format" for e in errors):
        fail("mutation: malformed source URI not rejected at landing_url")
    print("PASS mutation: malformed source URI rejected")

    unknown_prop = copy.deepcopy(claims[0])
    unknown_prop["unreviewed_precision"] = "must not enter silently"
    errors = errors_for(evidence_v, unknown_prop)
    if not any(e.validator == "additionalProperties" and "unreviewed_precision" in e.message for e in errors):
        fail("mutation: unknown evidence property not rejected")
    print("PASS mutation: unknown evidence property rejected")

    unknown_source = copy.deepcopy(claims[0])
    unknown_source["source_refs"] = ["source-that-does-not-exist"]
    if not any("unknown source_ref" in e for e in claim_semantic_errors(unknown_source, by_source, concepts)):
        fail("mutation: unresolved source reference not rejected")
    print("PASS mutation: unresolved source reference rejected")

    unknown_hypothesis = copy.deepcopy(claims[0])
    unknown_hypothesis["concept_ref"]["hypothesis_id"] = "not-a-real-hypothesis"
    if not any("unresolved hypothesis" in e for e in claim_semantic_errors(unknown_hypothesis, by_source, concepts)):
        fail("mutation: unresolved hypothesis reference not rejected")
    print("PASS mutation: unresolved concept hypothesis rejected")

    bad_resolution = copy.deepcopy(claims[0])
    bad_resolution["coverage"]["spatial_resolution_m"] = None
    bad_resolution["coverage"]["spatial_resolution_qualifier"] = "exact"
    if not any("resolution qualifier mismatch" in e for e in claim_semantic_errors(bad_resolution, by_source, concepts)):
        fail("mutation: contradictory spatial-resolution qualifier was not rejected")
    print("PASS mutation: approximate/unknown resolution cannot be mislabeled as exact")

    print(f"PASS: {len(by_source)} sources; {len(by_evidence)} claims; concept/source references resolve; 5 targeted mutation controls.")


if __name__ == "__main__":
    main()
