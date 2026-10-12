#!/usr/bin/env python3
"""Validate a regenerative observation receipt against its catalog contract.

A valid receipt is still an unreviewed source observation. This validator does not
retrieve sources, verify their claims, infer causality, or qualify an indicator.
"""
from __future__ import annotations

import json
import math
import re
import sys
from datetime import date
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
CATALOG_PATH = ROOT / "docs" / "economy-atlas" / "regenerative-indicators.v1.json"
SHA256 = re.compile(r"^[0-9a-f]{64}$")
STATUSES = {
    "observed", "estimated", "below_detection_limit", "not_collected",
    "not_applicable", "suppressed", "not_computable", "unknown",
}
MISSING_STATUSES = {"not_collected", "not_applicable", "suppressed", "not_computable", "unknown"}


def _real_iso_date(value: object) -> bool:
    if not isinstance(value, str):
        return False
    try:
        return date.fromisoformat(value).isoformat() == value
    except ValueError:
        return False


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_observation(record: object, catalog: object) -> list[str]:
    if not isinstance(record, dict):
        return ["observation record root must be an object"]
    if not isinstance(catalog, dict):
        return ["indicator catalog root must be an object"]
    errors: list[str] = []
    if record.get("schema_version") != "1.0.0":
        errors.append("schema_version must be 1.0.0")
    if record.get("record_type") != "regenerative_observation_record":
        errors.append("record_type must be regenerative_observation_record")
    if record.get("qualification_status") != "unreviewed_source_observation":
        errors.append("qualification_status must remain unreviewed_source_observation")

    for field in ("observation_id", "indicator_id", "native_unit", "denominator_definition"):
        if not isinstance(record.get(field), str) or not record[field].strip():
            errors.append(f"{field} must be a non-empty string")

    catalog_digest = record.get("indicator_catalog_sha256")
    if not isinstance(catalog_digest, str) or not SHA256.fullmatch(catalog_digest):
        errors.append("indicator_catalog_sha256 must be a lowercase SHA-256 digest")

    indicator_ids = {
        i.get("indicator_id")
        for i in catalog.get("indicators", [])
        if isinstance(i, dict) and isinstance(i.get("indicator_id"), str)
    } if isinstance(catalog.get("indicators"), list) else set()
    indicator_id = record.get("indicator_id")
    if isinstance(indicator_id, str) and indicator_id not in indicator_ids:
        errors.append(f"indicator_id is not present in the candidate catalog: {indicator_id}")

    source = record.get("source")
    if not isinstance(source, dict):
        errors.append("source must be an object")
        source = {}
    for field in ("publisher", "dataset_id", "series_id", "source_revision", "license_or_access_policy"):
        if not isinstance(source.get(field), str) or not source[field].strip():
            errors.append(f"source.{field} must be a non-empty string")
    url = source.get("source_url")
    parsed = urlparse(url) if isinstance(url, str) else None
    if parsed is None or parsed.scheme != "https" or not parsed.hostname:
        errors.append("source.source_url must be HTTPS with a hostname")
    if not _real_iso_date(source.get("retrieved_on")):
        errors.append("source.retrieved_on must be a real YYYY-MM-DD date")
    if not isinstance(source.get("artifact_sha256"), str) or not SHA256.fullmatch(source["artifact_sha256"]):
        errors.append("source.artifact_sha256 must be a lowercase SHA-256 digest")

    geography = record.get("geography")
    if not isinstance(geography, dict):
        errors.append("geography must be an object")
        geography = {}
    for field in ("name", "geography_id", "boundary_id", "boundary_version", "spatial_scope"):
        if not isinstance(geography.get(field), str) or not geography[field].strip():
            errors.append(f"geography.{field} must be a non-empty string")

    period = record.get("period")
    if not isinstance(period, dict):
        errors.append("period must be an object")
        period = {}
    for field in ("start_date", "end_date"):
        if not _real_iso_date(period.get(field)):
            errors.append(f"period.{field} must be a real YYYY-MM-DD date")
    if _real_iso_date(period.get("start_date")) and _real_iso_date(period.get("end_date")):
        if date.fromisoformat(period["start_date"]) > date.fromisoformat(period["end_date"]):
            errors.append("period.start_date must not be after period.end_date")
    if not isinstance(period.get("temporal_resolution"), str) or not period["temporal_resolution"].strip():
        errors.append("period.temporal_resolution must be a non-empty string")

    status = record.get("observation_status")
    if not isinstance(status, str) or status not in STATUSES:
        errors.append("observation_status must be an allowed explicit evidence/missingness state")
        status = None
    measurement = record.get("measurement")
    if not isinstance(measurement, dict):
        errors.append("measurement must be an object")
        measurement = {}
    kind = measurement.get("kind")
    value = measurement.get("value")
    if status in {"observed", "estimated"}:
        if kind != "point" or not _finite_number(value):
            errors.append("observed/estimated measurements require a finite numeric point value")
        if "censoring" in measurement:
            errors.append("point measurements must not include censoring metadata")
    elif status == "below_detection_limit":
        censoring = measurement.get("censoring")
        if kind != "censored" or value is not None or not isinstance(censoring, dict):
            errors.append("below_detection_limit requires censored kind, null value and censoring metadata")
        else:
            if not isinstance(censoring.get("operator"), str) or censoring.get("operator") not in {"<", "<="}:
                errors.append("censoring.operator must be < or <=")
            if not _finite_number(censoring.get("threshold")):
                errors.append("censoring.threshold must be finite")
            if not isinstance(censoring.get("unit"), str) or not censoring["unit"].strip():
                errors.append("censoring.unit must be non-empty")
    elif status in MISSING_STATUSES:
        if kind != "missing" or value is not None or "censoring" in measurement:
            errors.append("missing states require kind=missing, null value, and no censoring object")
    else:
        errors.append("observation_status cannot be inferred from measurement data")

    transformation = record.get("transformation")
    if not isinstance(transformation, dict):
        errors.append("transformation must be an object")
        transformation = {}
    if not isinstance(transformation.get("kind"), str) or transformation.get("kind") not in {"identity", "unit_conversion", "aggregation", "normalization", "model_estimate"}:
        errors.append("transformation.kind must be an explicit allowed method")
    for field in ("method_id", "version", "description"):
        if not isinstance(transformation.get(field), str) or not transformation[field].strip():
            errors.append(f"transformation.{field} must be a non-empty string")
    if not isinstance(transformation.get("parameters"), dict):
        errors.append("transformation.parameters must be an object, including an empty object for identity")
    if not isinstance(transformation.get("input_sha256"), str) or not SHA256.fullmatch(transformation["input_sha256"]):
        errors.append("transformation.input_sha256 must be a lowercase SHA-256 digest")

    coverage = record.get("coverage")
    if not isinstance(coverage, dict):
        errors.append("coverage must be an object")
        coverage = {}
    coverage_state = coverage.get("state")
    fraction = coverage.get("coverage_fraction")
    if not isinstance(coverage_state, str) or coverage_state not in {"complete", "partial", "unknown", "not_applicable"}:
        errors.append("coverage.state must be explicit")
    if fraction is not None and (not _finite_number(fraction) or fraction < 0 or fraction > 1):
        errors.append("coverage.coverage_fraction must be null or a number in [0, 1]")
    if coverage_state == "complete" and fraction != 1:
        errors.append("complete coverage requires coverage_fraction=1")
    if coverage_state == "partial" and (
        not _finite_number(fraction) or fraction <= 0 or fraction >= 1
    ):
        errors.append("partial coverage requires a finite coverage_fraction strictly between 0 and 1")
    if coverage_state in {"unknown", "not_applicable"} and fraction is not None:
        errors.append("unknown/not_applicable coverage requires a null fraction")
    if not isinstance(coverage.get("definition"), str) or not coverage["definition"].strip():
        errors.append("coverage.definition must explain the coverage denominator")

    uncertainty = record.get("uncertainty")
    if not isinstance(uncertainty, dict):
        errors.append("uncertainty must be an object")
        uncertainty = {}
    uncertainty_status = uncertainty.get("status")
    if not isinstance(uncertainty_status, str) or uncertainty_status not in {"quantified", "qualitative", "not_assessed"}:
        errors.append("uncertainty.status must be explicit")
    for field in ("method", "description"):
        if not isinstance(uncertainty.get(field), str) or not uncertainty[field].strip():
            errors.append(f"uncertainty.{field} must be non-empty; use an explicit not-assessed explanation when needed")
    if uncertainty_status == "quantified":
        low, high = uncertainty.get("lower_bound"), uncertainty.get("upper_bound")
        if not _finite_number(low) or not _finite_number(high) or low > high:
            errors.append("quantified uncertainty requires finite ordered lower_bound and upper_bound")
        if not isinstance(uncertainty.get("bounds_unit"), str) or not uncertainty["bounds_unit"].strip():
            errors.append("quantified uncertainty requires bounds_unit")

    flags = record.get("quality_flags")
    if not isinstance(flags, list) or any(not isinstance(flag, str) or not flag.strip() for flag in flags):
        errors.append("quality_flags must be a list of non-empty strings")
    elif len(flags) != len(set(flags)):
        errors.append("quality_flags must not contain duplicates")
    return errors


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: validate_regenerative_observation.py <observation-record.json>", file=sys.stderr)
        return 2
    try:
        record = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
        catalog = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot read record or catalog: {exc}", file=sys.stderr)
        return 1
    errors = validate_observation(record, catalog)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"Observation receipt rejected: {len(errors)} violation(s).", file=sys.stderr)
        return 1
    print("PASS: observation receipt structure is valid.")
    print("NOTE: the source artifact was not fetched or independently authenticated; the receipt remains unreviewed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
