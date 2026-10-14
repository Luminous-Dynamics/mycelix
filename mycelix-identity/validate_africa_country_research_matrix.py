#!/usr/bin/env python3
"""Validate the research-only Africa country interoperability matrix.

This validates evidence metadata and fail-closed gates, not legal status,
identity credentials, government APIs, or production integration.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

MATRIX_PATH = Path(__file__).with_name("AFRICA_COUNTRY_RESEARCH_MATRIX_V1.json")
SNAPSHOT_DATE = "2026-10-14"
REQUIRED_TOP_LEVEL_FIELDS = {
    "schema_id", "schema_version", "snapshot_date", "maturity",
    "production_enabled", "qualified_integrations", "common_status_values",
    "common_gate_keys", "route_records", "notes",
}
REQUIRED_ROUTE_FIELDS = {
    "route_key", "country_code", "country_name", "authority", "route_name",
    "status", "production_enabled", "external_verifier_protocol_reviewed",
    "written_authorization_obtained", "official_sources", "established_facts",
    "unresolved_questions", "release_gates",
}
STATUS_VALUES = {
    "not_assessed", "research_only", "partner_review",
    "sandbox_authorized", "production_authorized",
}
GATE_KEYS = {
    "domestic_legal_basis_reviewed",
    "written_authority_or_partner_authorization",
    "official_external_verifier_protocol_reviewed",
    "issuer_trust_metadata_and_key_rotation_verified",
    "credential_validity_status_and_revocation_reviewed",
    "purpose_minimization_and_data_protection_review_approved",
    "non_digital_or_assisted_fallback_tested",
    "accessibility_and_local_language_review_complete",
    "independent_security_review_complete",
    "authorized_sandbox_conformance_passed",
    "incident_correction_and_appeal_process_agreed",
    "human_release_approval_for_exact_commit",
}
EXPECTED_SOURCE_URLS = {
    "ZA-DHA-OVS": {
        "https://www.gov.za/news/media-statements/home-affairs-corrects-unsustainable-under-pricing-national-population",
        "https://www.gov.za/news/media-statements/minister-leon-schreiber-welcomes-withdrawal-act-court-application-against",
    },
    "ZA-DHA-SMARTPHONE-DIGITAL-ID": {
        "https://www.gov.za/news/speeches/minister-leon-schreiber-home-affairs-quantum-leap-vision-future-government-01-oct",
        "https://www.gov.za/news/media-statements/minister-leon-schreiber-invites-public-comment-draft-digital-identity",
        "https://www.dha.gov.za/images/AnnualReports/DHA-APP-2026-27_260326.pdf",
    },
    "GH-NIA-GHANA-CARD": {
        "https://nia.gov.gh/service/verification-services/",
        "https://nia.gov.gh/government-makes-biometric-verification-mandatory-for-ghana-card-verification-under-new-l-i-2523/",
    },
    "RW-NIDA-EID": {
        "https://www.nida.gov.rw/news-details/nida-launched-the-digital-id",
        "https://support.new.irembo.gov.rw/en/support/solutions/articles/47001285585-faqs-about-new-digital-id-e-id-or-e-ndangamuntu-and-digital-id-pre-enrollment",
    },
    "NG-NIMC-NIN": {
        "https://nimc.gov.ng/press-releases/president-tinubu-signs-nimc-act-2026-into-law",
    },
}
EXPECTED_COUNTRY_AND_ROUTE = {
    "ZA-DHA-OVS": ("ZA", "South Africa", "Department of Home Affairs"),
    "ZA-DHA-SMARTPHONE-DIGITAL-ID": ("ZA", "South Africa", "Department of Home Affairs"),
    "GH-NIA-GHANA-CARD": ("GH", "Ghana", "National Identification Authority"),
    "RW-NIDA-EID": ("RW", "Rwanda", "National Identification Agency (NIDA)"),
    "NG-NIMC-NIN": ("NG", "Nigeria", "National Identity Management Commission (NIMC)"),
}


def validate_matrix(data: Any) -> list[str]:
    """Return errors if source claims are unpinned or research gates were opened."""
    if not isinstance(data, dict):
        return ["matrix must be a JSON object"]
    errors: list[str] = []
    if set(data) != REQUIRED_TOP_LEVEL_FIELDS:
        errors.append("top-level fields must match the closed v1 matrix schema")
    if data.get("schema_id") != "org.luminousdynamics.africa-country-interoperability-research-matrix":
        errors.append("unexpected schema_id")
    if type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
        errors.append("schema_version must be the integer 1")
    if data.get("snapshot_date") != SNAPSHOT_DATE:
        errors.append(f"snapshot_date must remain {SNAPSHOT_DATE} until reviewed")
    if data.get("maturity") != "research_only":
        errors.append("matrix maturity must remain research_only")
    if data.get("production_enabled") is not False:
        errors.append("matrix production_enabled must be false")
    if data.get("qualified_integrations") != []:
        errors.append("qualified_integrations must remain empty in the research matrix")

    statuses = data.get("common_status_values")
    if (
        not isinstance(statuses, list)
        or any(not isinstance(value, str) for value in statuses)
        or len(statuses) != len(STATUS_VALUES)
        or set(statuses) != STATUS_VALUES
    ):
        errors.append("common_status_values must match the closed v1 status enum")

    gate_keys = data.get("common_gate_keys")
    if (
        not isinstance(gate_keys, list)
        or any(not isinstance(value, str) for value in gate_keys)
        or len(gate_keys) != len(GATE_KEYS)
        or set(gate_keys) != GATE_KEYS
    ):
        errors.append("common_gate_keys must preserve the closed v1 release-gate set")

    records = data.get("route_records")
    if not isinstance(records, list):
        errors.append("route_records must be an array")
        records = []
    route_keys: set[str] = set()
    for index, route in enumerate(records):
        prefix = f"route_records[{index}]"
        if not isinstance(route, dict):
            errors.append(f"{prefix} must be an object")
            continue
        route_key = route.get("route_key")
        if set(route) != REQUIRED_ROUTE_FIELDS:
            errors.append(f"{prefix} fields must match the closed v1 route schema")
        if not isinstance(route_key, str) or not route_key:
            errors.append(f"{prefix}.route_key must be a non-empty string")
            continue
        if route_key in route_keys:
            errors.append(f"duplicate route_key: {route_key}")
        route_keys.add(route_key)
        expected_identity = EXPECTED_COUNTRY_AND_ROUTE.get(route_key)
        if expected_identity is None:
            errors.append(f"unknown route_key requires a reviewed matrix revision: {route_key}")
        else:
            expected_code, expected_country, expected_authority = expected_identity
            if route.get("country_code") != expected_code:
                errors.append(f"{prefix}.country_code mismatch for {route_key}")
            if route.get("country_name") != expected_country:
                errors.append(f"{prefix}.country_name mismatch for {route_key}")
            if route.get("authority") != expected_authority:
                errors.append(f"{prefix}.authority mismatch for {route_key}")

        for field in ("country_code", "country_name", "authority", "route_name"):
            if not isinstance(route.get(field), str) or not route[field].strip():
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if route.get("status") != "research_only":
            errors.append(f"{prefix}.status must remain research_only")
        for field in ("production_enabled", "external_verifier_protocol_reviewed", "written_authorization_obtained"):
            if route.get(field) is not False:
                errors.append(f"{prefix}.{field} must remain false")

        sources = route.get("official_sources")
        actual_urls: set[str] = set()
        if not isinstance(sources, list) or not sources:
            errors.append(f"{prefix}.official_sources must be a non-empty array")
            sources = []
        for source_index, source in enumerate(sources):
            source_prefix = f"{prefix}.official_sources[{source_index}]"
            if not isinstance(source, dict) or set(source) != {"url", "retrieved_on", "supports"}:
                errors.append(f"{source_prefix} must match the closed v1 source schema")
                continue
            url = source.get("url")
            if not isinstance(url, str) or not url.startswith("https://"):
                errors.append(f"{source_prefix}.url must use HTTPS")
            elif url in actual_urls:
                errors.append(f"{prefix} contains a duplicate source URL: {url}")
            else:
                actual_urls.add(url)
            if source.get("retrieved_on") != SNAPSHOT_DATE:
                errors.append(f"{source_prefix}.retrieved_on must match matrix snapshot date")
            if not isinstance(source.get("supports"), str) or not source["supports"].strip():
                errors.append(f"{source_prefix}.supports must state the exact claim supported")
        expected_urls = EXPECTED_SOURCE_URLS.get(route_key)
        if expected_urls is not None and actual_urls != expected_urls:
            errors.append(f"{prefix}.official_sources must match the pinned official sources for {route_key}")

        for field in ("established_facts", "unresolved_questions"):
            values = route.get(field)
            if not isinstance(values, list) or not values or any(
                not isinstance(value, str) or not value.strip() for value in values
            ):
                errors.append(f"{prefix}.{field} must be a non-empty array of strings")

        gates = route.get("release_gates")
        if not isinstance(gates, dict):
            errors.append(f"{prefix}.release_gates must be an object")
        else:
            if set(gates) != GATE_KEYS:
                errors.append(f"{prefix}.release_gates keys must match the common v1 gate set")
            for gate in GATE_KEYS:
                if gates.get(gate) is not False:
                    errors.append(f"{prefix} release gate must remain closed: {gate}")

    expected_route_keys = set(EXPECTED_COUNTRY_AND_ROUTE)
    if route_keys != expected_route_keys:
        missing = expected_route_keys - route_keys
        extra = route_keys - expected_route_keys
        if missing:
            errors.append("missing pinned research routes: " + ", ".join(sorted(missing)))
        if extra:
            errors.append("unexpected research routes: " + ", ".join(sorted(extra)))
    if len(records) != len(EXPECTED_COUNTRY_AND_ROUTE):
        errors.append(f"expected exactly {len(EXPECTED_COUNTRY_AND_ROUTE)} pinned research routes")

    notes = data.get("notes")
    if not isinstance(notes, list) or not notes or any(
        not isinstance(note, str) for note in notes
    ):
        errors.append("notes must be a non-empty array of strings")
    else:
        combined = " ".join(notes).lower()
        for phrase in (
            "not an integration registry",
            "mycelix has access or permission",
            "no real identity data was processed",
        ):
            if phrase not in combined:
                errors.append(f"notes must preserve the research nonclaim: {phrase}")
    return errors


def _reject_duplicate_object_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, default=MATRIX_PATH)
    args = parser.parse_args()
    try:
        matrix = json.loads(
            args.matrix.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_object_keys,
        )
    except (OSError, ValueError) as exc:
        print(f"ERROR: could not load Africa country research matrix: {exc}", file=sys.stderr)
        return 2
    errors = validate_matrix(matrix)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"AFRICA_COUNTRY_MATRIX_VALIDATION=FAIL errors={len(errors)}", file=sys.stderr)
        return 1
    gates_closed = sum(
        value is False
        for route in matrix["route_records"]
        for value in route["release_gates"].values()
    )
    print(
        "AFRICA_COUNTRY_MATRIX_VALIDATION=PASS "
        f"routes={len(matrix['route_records'])} "
        f"gates_closed={gates_closed} "
        "qualified_integrations=0 production_enabled=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
