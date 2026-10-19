#!/usr/bin/env python3
"""Fail-closed validation for candidate dataset rights and ingestion gates.

This validates the discovery manifest and cross-links only. It does not download
sources, give legal advice, or approve any dataset for use.
"""
from __future__ import annotations

import json
import sys
from datetime import date
import math
from pathlib import Path
from urllib.parse import urlparse

def finite_coordinate(value: object) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError):
        return False


ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "docs" / "economy-atlas" / "regenerative-dataset-candidates.v1.json"
INDICATOR_CATALOG_PATH = ROOT / "docs" / "economy-atlas" / "regenerative-indicators.v1.json"
CANDIDATE_STATUSES = {
    "license_not_verified",
    "license_metadata_conflict",
    "restricted_noncommercial_use",
    "license_declared_access_private",
}
LICENSE_STATUSES = {"unverified", "conflicting", "restricted", "verified"}
COMMERCIAL_STATES = {"unknown", "permitted", "prohibited", "permission_required"}
REDISTRIBUTION_STATES = {
    "unknown", "permitted", "prohibited_unless_permission_granted", "permission_required"
}
MODIFICATION_STATES = {"unknown", "permitted", "prohibited", "permission_required"}


def real_date(value: object) -> bool:
    if not isinstance(value, str):
        return False
    try:
        return date.fromisoformat(value).isoformat() == value
    except ValueError:
        return False


def https_url(value: object) -> bool:
    if not isinstance(value, str):
        return False
    parsed = urlparse(value)
    return parsed.scheme == "https" and bool(parsed.hostname)


def validate_manifest(manifest: object, indicator_catalog: object) -> list[str]:
    if not isinstance(manifest, dict):
        return ["dataset candidate catalog root must be an object"]
    if not isinstance(indicator_catalog, dict):
        return ["indicator catalog root must be an object"]
    errors: list[str] = []
    if manifest.get("schema_version") != "1.0.0":
        errors.append("schema_version must be 1.0.0")
    if manifest.get("catalog_id") != "mycelix.regenerative-dataset-candidates":
        errors.append("catalog_id must be mycelix.regenerative-dataset-candidates")
    if manifest.get("catalog_status") != "source_discovery_only":
        errors.append("catalog_status must remain source_discovery_only")

    qualification = manifest.get("qualification")
    required_false_flags = {
        "source_artifacts_downloaded": False,
        "real_observations_imported": False,
        "candidate_ingestion_gates_passed": False,
        "dataset_rights_independently_verified": False,
    }
    if qualification != required_false_flags:
        errors.append("all qualification flags must remain false until explicit ingestion review")

    rules = manifest.get("global_ingestion_rules")
    if not isinstance(rules, list) or not rules or any(not isinstance(rule, str) or len(rule.strip()) < 10 for rule in rules):
        errors.append("global_ingestion_rules must contain substantive strings")

    indicator_ids = {
        item.get("indicator_id")
        for item in indicator_catalog.get("indicators", [])
        if isinstance(item, dict) and isinstance(item.get("indicator_id"), str)
    } if isinstance(indicator_catalog.get("indicators"), list) else set()

    candidates = manifest.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        errors.append("candidates must be a non-empty list")
        candidates = []
    candidate_ids: set[str] = set()
    all_finding_ids: set[str] = set()
    for index, candidate in enumerate(candidates):
        location = f"candidates[{index}]"
        if not isinstance(candidate, dict):
            errors.append(f"{location} must be an object")
            continue
        candidate_id = candidate.get("candidate_id")
        if not isinstance(candidate_id, str) or not candidate_id.strip():
            errors.append(f"{location}.candidate_id must be a non-empty string")
        elif candidate_id in candidate_ids:
            errors.append(f"duplicate candidate_id: {candidate_id}")
        else:
            candidate_ids.add(candidate_id)

        for key in ("title", "publisher", "reported_availability"):
            if key in candidate and (not isinstance(candidate[key], str) or not candidate[key].strip()):
                errors.append(f"{location}.{key} must be a non-empty string")
        if not https_url(candidate.get("dataset_url")):
            errors.append(f"{location}.dataset_url must be HTTPS with a hostname")

        status = candidate.get("discovery_status")
        if not isinstance(status, str) or status not in CANDIDATE_STATUSES:
            errors.append(f"{location}.discovery_status is invalid or missing")
        links = candidate.get("related_indicator_ids")
        if not isinstance(links, list) or not links:
            errors.append(f"{location}.related_indicator_ids must be a non-empty list")
        else:
            for link in links:
                if not isinstance(link, str) or link not in indicator_ids:
                    errors.append(f"{location} refers to an unknown or malformed indicator_id: {link}")

        reported_shape = candidate.get("reported_data_shape")
        if not isinstance(reported_shape, dict):
            errors.append(f"{location}.reported_data_shape must be an object")
        elif candidate_id == "south-africa.swam-satellite-water-quality.2026.v1":
            if "Western Cape" not in candidate.get("spatial_coverage", {}).get("geography", ""):
                errors.append(f"{location}: SWAM geographic scope must remain explicitly Western Cape")
            if "target_geography_mismatch_for_johannesburg_pilot" not in candidate.get("ingestion_gate", {}).get("reason_codes", []):
                errors.append(f"{location}: SWAM must not be qualified as a Johannesburg/Gauteng source")
            if "https://github.com/DalinJiang/EO-Africa-SWAM" not in candidate.get("license_assessment", {}).get("evidence_urls", []):
                errors.append(f"{location}: linked project code/data license-scope evidence is required")
            finding_ids_for_swam = {
                finding.get("finding_id")
                for finding in candidate.get("known_findings", [])
                if isinstance(finding, dict) and isinstance(finding.get("finding_id"), str)
            }
            required_swam_findings = {
                "SWAM-PREVIEW-DATE-WINDOW-001",
                "SWAM-MULTI-OVERPASS-DAY-001",
                "SWAM-RIETVLEI-SITE-IDENTITY-001",
                "SWAM-DATA-CODE-LICENSE-SCOPE-001",
            }
            if not required_swam_findings.issubset(finding_ids_for_swam):
                errors.append(f"{location}: required SWAM date, overpass, site-identity and license-scope findings must be preserved")
        elif candidate_id == "south-africa.national-water-chemistry.1972-2011.v1":
            if "reported_sample_count" in reported_shape or "reported_station_count" in reported_shape:
                errors.append(f"{location}: approximate source counts must not be encoded as exact numeric counts")
            if not isinstance(reported_shape.get("reported_sample_count_description"), str) or "more than 500,000" not in reported_shape["reported_sample_count_description"].lower():
                errors.append(f"{location}: preserve the source's 'more than 500,000' count wording")
            if not isinstance(reported_shape.get("reported_station_count_description"), str) or "over 2,000" not in reported_shape["reported_station_count_description"].lower():
                errors.append(f"{location}: preserve the source's 'over 2,000' station wording")

        known_findings = candidate.get("known_findings", [])
        finding_ids = {
            finding.get("finding_id")
            for finding in known_findings
            if isinstance(finding, dict) and isinstance(finding.get("finding_id"), str)
        } if isinstance(known_findings, list) else set()
        if candidate_id == "south-africa.jukskei-water-quality-study.2021.v1":
            if status != "license_declared_access_private":
                errors.append(f"{location}: Jukskei study must retain its declared-license/private-access state")
            if not {"JUKSKEI-PRIVATE-SHARE-ACCESS-001", "JUKSKEI-SIX-SAMPLE-WINDOW-001"}.issubset(finding_ids):
                errors.append(f"{location}: Jukskei access and short-sample-window findings must be preserved")
            shape = candidate.get("reported_data_shape", {})
            if not isinstance(shape, dict) or shape.get("paper_sample_count") != 6:
                errors.append(f"{location}: preserve the paper's six-sample study design as source-reported metadata")
            if not isinstance(candidate.get("license_assessment"), dict) or candidate["license_assessment"].get("license_status") != "unverified":
                errors.append(f"{location}: private-share license status must remain unverified until file-level review")
        if candidate_id == "south-africa.jukskei-river-microbial-data.2025.v1":
            spatial_record = candidate.get("spatial_coverage", {})
            sites = spatial_record.get("entities", []) if isinstance(spatial_record, dict) else []
            expected_coordinates = {
                "A": (-26.18147, 28.10237),
                "B": (-26.16475, 28.13029),
                "C": (-26.08208, 28.10873),
                "D": (-26.06715, 28.10767),
                "E": (-25.98690, 27.99731),
            }
            seen_sites: set[str] = set()
            if not isinstance(sites, list) or len(sites) != 5:
                errors.append(f"{location}: ASM Jukskei candidate must preserve its five source-reported site records")
                sites = []
            for site_index, site in enumerate(sites):
                site_location = f"{location}.spatial_coverage.entities[{site_index}]"
                if not isinstance(site, dict):
                    errors.append(f"{site_location} must be a site object with source and normalized coordinates")
                    continue
                site_id = site.get("site")
                if not isinstance(site_id, str) or site_id not in expected_coordinates:
                    errors.append(f"{site_location}.site must be one of the source site IDs A-E")
                    continue
                if site_id in seen_sites:
                    errors.append(f"{location}: duplicate ASM site ID {site_id}")
                seen_sites.add(site_id)
                raw_coordinates = site.get("coordinates_as_published")
                if not isinstance(raw_coordinates, str) or not raw_coordinates.strip():
                    errors.append(f"{site_location}.coordinates_as_published must preserve the paper's original coordinate notation")
                normalized = site.get("coordinates_decimal_degrees")
                if not isinstance(normalized, dict):
                    errors.append(f"{site_location}.coordinates_decimal_degrees must be an object")
                    continue
                latitude = normalized.get("latitude")
                longitude = normalized.get("longitude")
                if not finite_coordinate(latitude):
                    errors.append(f"{site_location}.coordinates_decimal_degrees.latitude must be finite")
                    continue
                if not finite_coordinate(longitude):
                    errors.append(f"{site_location}.coordinates_decimal_degrees.longitude must be finite")
                    continue
                expected_latitude, expected_longitude = expected_coordinates[site_id]
                if latitude != expected_latitude or longitude != expected_longitude:
                    errors.append(f"{site_location}: normalized coordinates must match the source-pinned decimal conversion for site {site_id}")
                if not (-27.0 <= latitude <= -25.0 and 27.0 <= longitude <= 29.0):
                    errors.append(f"{site_location}: normalized coordinate falls outside the broad published Gauteng study-area bounds")
                conversion_note = normalized.get("conversion_note")
                if not isinstance(conversion_note, str) or "coordinates_as_published" not in conversion_note:
                    errors.append(f"{site_location}: coordinate conversion rule must be documented")
            if seen_sites != set(expected_coordinates):
                errors.append(f"{location}: ASM site IDs must be exactly A, B, C, D and E")
            if status != "license_not_verified":
                errors.append(f"{location}: ASM Jukskei supplemental data must remain license_not_verified")
            if not {"ASM-JUKSKEI-SUPPLEMENT-REUSE-RIGHTS-001", "ASM-JUKSKEI-12-MONTH-WINDOW-001"}.issubset(finding_ids):
                errors.append(f"{location}: ASM supplemental rights and date-window findings must be preserved")
            shape = candidate.get("reported_data_shape", {})
            if not isinstance(shape, dict) or shape.get("study_sample_count") != 60 or shape.get("site_count") != 5:
                errors.append(f"{location}: preserve the article's source-reported 60-sample/five-site design")
            temporal = candidate.get("temporal_coverage", {})
            if isinstance(temporal, dict) and (temporal.get("start_date") is not None or temporal.get("end_date") is not None):
                errors.append(f"{location}: do not invent ASM study date bounds before inspecting the sample records")
            license_info = candidate.get("license_assessment", {})
            if isinstance(license_info, dict) and license_info.get("license_status") != "unverified":
                errors.append(f"{location}: supplemental-data reuse rights must remain unverified")
        if candidate_id == "south-africa.up-hartbeespoort-roodeplaat-water-quality.2025.v1":
            if status != "license_not_verified":
                errors.append(f"{location}: University of Pretoria dataset must remain license_not_verified")
            if "UP-CATCHMENTS-LICENSE-METADATA-001" not in finding_ids:
                errors.append(f"{location}: University of Pretoria license/access finding must be preserved")
            temporal = candidate.get("temporal_coverage", {})
            if isinstance(temporal, dict) and (temporal.get("start_date") is not None or temporal.get("end_date") is not None):
                errors.append(f"{location}: do not invent a time window before source-file inspection")

        coverage = candidate.get("temporal_coverage")
        if not isinstance(coverage, dict):
            errors.append(f"{location}.temporal_coverage must be an object")
        else:
            start, end = coverage.get("start_date"), coverage.get("end_date")
            if start is not None and not real_date(start):
                errors.append(f"{location}.temporal_coverage.start_date must be a real YYYY-MM-DD date or null")
            if end is not None and not real_date(end):
                errors.append(f"{location}.temporal_coverage.end_date must be a real YYYY-MM-DD date or null")
            if real_date(start) and real_date(end) and date.fromisoformat(start) > date.fromisoformat(end):
                errors.append(f"{location}.temporal_coverage starts after it ends")
            if not isinstance(coverage.get("basis"), str) or not coverage["basis"].strip():
                errors.append(f"{location}.temporal_coverage.basis must explain the claim")

        spatial = candidate.get("spatial_coverage")
        if not isinstance(spatial, dict) or not all(
            isinstance(spatial.get(key), str) and spatial[key].strip()
            for key in ("geography", "caveat")
        ):
            errors.append(f"{location}.spatial_coverage must explain geography and caveats")

        measures = candidate.get("reported_measures")
        if not isinstance(measures, list) or not measures:
            errors.append(f"{location}.reported_measures must be a non-empty list")
        else:
            for j, measure in enumerate(measures):
                if not isinstance(measure, dict) or not all(
                    isinstance(measure.get(key), str) and measure[key].strip()
                    for key in ("name", "native_unit", "method_summary")
                ):
                    errors.append(f"{location}.reported_measures[{j}] must include name, native unit and method summary")

        license_info = candidate.get("license_assessment")
        if not isinstance(license_info, dict):
            errors.append(f"{location}.license_assessment must be an object")
            license_info = {}
        license_status = license_info.get("license_status")
        if not isinstance(license_status, str) or license_status not in LICENSE_STATUSES:
            errors.append(f"{location}.license_assessment.license_status is invalid")
        consistency = {
            "license_not_verified": "unverified",
            "license_metadata_conflict": "conflicting",
            "restricted_noncommercial_use": "restricted",
            "license_declared_access_private": "unverified",
        }
        if isinstance(status, str) and status in consistency and license_status != consistency[status]:
            errors.append(f"{location}: discovery_status conflicts with license_assessment.license_status")
        for key, allowed in (
            ("commercial_reuse", COMMERCIAL_STATES),
            ("redistribution", REDISTRIBUTION_STATES),
            ("modification", MODIFICATION_STATES),
        ):
            if not isinstance(license_info.get(key), str) or license_info.get(key) not in allowed:
                errors.append(f"{location}.license_assessment.{key} is invalid")
        for key in ("displayed_access_status", "assessment_note"):
            if not isinstance(license_info.get(key), str) or not license_info[key].strip():
                errors.append(f"{location}.license_assessment.{key} must be explicit")
        evidence_urls = license_info.get("evidence_urls")
        if not isinstance(evidence_urls, list) or not evidence_urls:
            errors.append(f"{location}.license_assessment.evidence_urls must be non-empty")
        elif any(not https_url(url) for url in evidence_urls):
            errors.append(f"{location}.license_assessment.evidence_urls must contain only HTTPS URLs")

        gate = candidate.get("ingestion_gate")
        if not isinstance(gate, dict):
            errors.append(f"{location}.ingestion_gate must be an object")
            gate = {}
        if gate.get("status") != "blocked":
            errors.append(f"{location}.ingestion_gate.status must remain blocked")
        if gate.get("ingestion_authorized") is not False:
            errors.append(f"{location}.ingestion_gate.ingestion_authorized must be false")
        if gate.get("values_imported") is not False:
            errors.append(f"{location}.ingestion_gate.values_imported must be false")
        for key in ("reason_codes", "required_evidence"):
            values = gate.get(key)
            if not isinstance(values, list) or not values or any(not isinstance(value, str) or not value.strip() for value in values):
                errors.append(f"{location}.ingestion_gate.{key} must be a non-empty string list")
        reason_codes = gate.get("reason_codes")
        if isinstance(reason_codes, list) and all(isinstance(value, str) for value in reason_codes):
            if len(reason_codes) != len(set(reason_codes)):
                errors.append(f"{location}.ingestion_gate.reason_codes must be unique")

        findings = candidate.get("known_findings")
        if not isinstance(findings, list):
            errors.append(f"{location}.known_findings must be a list")
        else:
            for finding_index, finding in enumerate(findings):
                finding_location = f"{location}.known_findings[{finding_index}]"
                if not isinstance(finding, dict):
                    errors.append(f"{finding_location} must be an object")
                    continue
                finding_id = finding.get("finding_id")
                if not isinstance(finding_id, str) or not finding_id.strip():
                    errors.append(f"{finding_location}.finding_id must be a non-empty string")
                elif finding_id in all_finding_ids:
                    errors.append(f"duplicate finding_id across dataset catalog: {finding_id}")
                else:
                    all_finding_ids.add(finding_id)
                severity = finding.get("severity")
                if not isinstance(severity, str) or severity not in {"low", "medium", "high", "critical"}:
                    errors.append(f"{finding_location}.severity is invalid")
                for field in ("observation", "required_resolution"):
                    if not isinstance(finding.get(field), str) or len(finding[field].strip()) < (20 if field == "observation" else 10):
                        errors.append(f"{finding_location}.{field} must be substantive")
        suitability = candidate.get("suitability")
        if not isinstance(suitability, dict) or not all(
            isinstance(suitability.get(key), str) and suitability[key].strip()
            for key in ("best_initial_use", "not_suitable_for", "next_action")
        ):
            errors.append(f"{location}.suitability must state best use, exclusions and next action")

    if len(candidate_ids) != len(candidates):
        errors.append("candidate IDs must be unique and every candidate must be an object")
    return errors


def main() -> int:
    try:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        indicators = json.loads(INDICATOR_CATALOG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot read dataset candidates or indicator catalog: {exc}", file=sys.stderr)
        return 1
    errors = validate_manifest(manifest, indicators)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"Dataset candidate manifest rejected: {len(errors)} violation(s).", file=sys.stderr)
        return 1
    print(f"PASS: dataset candidate manifest structure validated ({len(manifest['candidates'])} candidates).")
    print("NOTE: every ingestion gate is blocked; no data was downloaded or approved by this validator.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
