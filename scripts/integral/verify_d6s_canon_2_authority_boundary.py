#!/usr/bin/env python3
"""Independent hash/census verifier for D6S-CANON-2 authority-boundary controls."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "docs/integral/d6s-canon-2-manifest.json"


def unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique_pairs,
                      parse_constant=lambda token: (_ for _ in ()).throw(ValueError(f"invalid constant: {token}")))


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def main() -> int:
    manifest = load(MANIFEST)
    if manifest.get("manifest_version") != "d6s-canon-2-manifest/1":
        raise ValueError("MANIFEST_VERSION_MISMATCH")
    if manifest.get("fixture_id") != "D6S-CANON-2":
        raise ValueError("FIXTURE_ID_MISMATCH")
    path_text = manifest.get("fixture_path")
    if path_text != "docs/integral/d6s-canon-2-authority-boundary-fixture.json":
        raise ValueError("FIXTURE_PATH_NOT_ALLOWLISTED")
    path = (ROOT / path_text).resolve()
    if ROOT.resolve() not in path.parents or not path.is_file():
        raise ValueError("FIXTURE_PATH_INVALID_OR_MISSING")
    actual_blob = git_blob_sha(path.read_bytes())
    if actual_blob != manifest.get("fixture_git_blob_sha"):
        raise ValueError(f"FIXTURE_BLOB_PIN_MISMATCH expected={manifest.get('fixture_git_blob_sha')} actual={actual_blob}")
    fixture = load(path)
    if fixture.get("schema_version") != "d6s-canon-2-authority-boundary-fixture/1":
        raise ValueError("FIXTURE_SCHEMA_VERSION_MISMATCH")
    if fixture.get("fixture_id") != manifest["fixture_id"] or fixture.get("status") != "ReferenceModelOnly":
        raise ValueError("FIXTURE_METADATA_MISMATCH")
    cases = fixture.get("cases")
    if not isinstance(cases, list) or len(cases) != 7 or manifest.get("case_count") != 7:
        raise ValueError("CASE_COUNT_MISMATCH")
    ids = [item.get("case_id") for item in cases]
    if ids != manifest.get("required_case_ids") or len(set(ids)) != len(ids):
        raise ValueError("CASE_ID_CENSUS_MISMATCH")
    required = set(manifest.get("formal_obligation_vocabulary", []))
    if required != {f"COS-FV-{n:03d}" for n in range(1, 11)}:
        raise ValueError("FORMAL_OBLIGATION_MANIFEST_INCOMPLETE")
    covered: set[str] = set()
    required_fields = {
        "production-observation-is-not-itc-mint": ("expected_without_projection", "expected_after_explicit_projection"),
        "frs-projection-is-not-source-observation": ("expected_without_source_observation", "expected_after_explicit_source_observation"),
        "recommendation-does-not-create-effect": ("expected_without_authority", "expected_with_authority_but_without_effect", "expected_after_authorized_effect_receipt"),
        "foreign-recognition-preserves-origin": ("expected_decision", "expected_origin_before", "expected_origin_after"),
        "qualified-output-is-not-physical-work": ("expected_without_physical_work_binding", "expected_after_physical_work_binding"),
        "stale-evidence-does-not-satisfy-current-requirement": ("expected_decision",),
        "unknown-transition-remains-unknown": ("expected_decision",),
    }
    for item in cases:
        case_id = item.get("case_id")
        obligations = item.get("formal_obligations")
        if not isinstance(obligations, list) or not obligations or not set(obligations) <= required:
            raise ValueError(f"INVALID_FORMAL_MAPPING {case_id}")
        if not item.get("claim_ceiling"):
            raise ValueError(f"CLAIM_CEILING_MISSING {case_id}")
        for field in required_fields.get(case_id, ()):
            if field not in item:
                raise ValueError(f"REQUIRED_EXPECTATION_MISSING {case_id}/{field}")
        covered.update(obligations)
    if set(ids) != set(required_fields):
        raise ValueError("UNEXPECTED_OR_MISSING_BOUNDARY_CASE")
    if not covered <= required:
        raise ValueError("UNKNOWN_FORMAL_OBLIGATION")
    print(f"PASS: D6S-CANON-2 manifest and {len(cases)} pinned authority-boundary cases verified.")
    print("Claim ceiling: fixture integrity and control census only; Rust tests separately exercise the reference transitions.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError, TypeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
