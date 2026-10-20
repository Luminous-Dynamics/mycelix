#!/usr/bin/env python3
"""Independent structural and digest verifier for D6S-CANON-2 boundary fixture.

This is a CI/test verifier only; it does not import or execute production COS code.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = ROOT / "docs/integral/d6s-canon-2-manifest.json"
REQUIRED = {
    "planned_input_is_not_consumption",
    "scheduled_operation_is_not_observed_work",
    "finding_is_not_source_observation",
    "recommendation_needs_authorization_and_effect",
    "unknown_delivery_is_not_failure",
    "foreign_origin_survives_recognition",
    "qualification_does_not_prove_physical_work",
    "later_success_preserves_failure_history",
    "availability_requires_temporal_binding",
    "single_success_is_not_general_capability",
}


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"{path.relative_to(ROOT)}: cannot parse fixture: {exc}") from exc


manifest = load_json(MANIFEST_PATH)
if manifest.get("schema_version") != "d6s-canon-2-manifest/1":
    raise SystemExit("D6S-CANON-2 manifest schema mismatch")
fixture_path = ROOT / manifest["fixture_path"]
raw = fixture_path.read_bytes()
actual_digest = hashlib.sha256(raw).hexdigest()
expected_digest = manifest.get("fixture_sha256")
if actual_digest != expected_digest:
    raise SystemExit(
        f"D6S-CANON-2 fixture SHA-256 mismatch: expected {expected_digest}, actual {actual_digest}"
    )
fixture = json.loads(raw)
if fixture.get("schema_version") != "d6s-canon-2-authority-boundary-fixture/1":
    raise SystemExit("D6S-CANON-2 fixture schema mismatch")
if fixture.get("status") != "ReferenceModelOnly":
    raise SystemExit("D6S-CANON-2 claim status must remain ReferenceModelOnly")
if fixture.get("corpus_id") != manifest.get("corpus_id") or fixture.get("corpus_id") != "COS-CONF-001":
    raise SystemExit("D6S-CANON-2 corpus identity mismatch")
mutations = fixture.get("mutations")
if not isinstance(mutations, list) or len(mutations) != manifest.get("expected_mutation_count"):
    raise SystemExit("D6S-CANON-2 mutation count mismatch")
ids = [item.get("id") for item in mutations]
if set(ids) != REQUIRED or len(ids) != len(set(ids)):
    raise SystemExit("D6S-CANON-2 missing, duplicate, or unexpected mutation IDs")
by_id = {item["id"]: item for item in mutations}
expected_outcomes = {
    "planned_input_is_not_consumption": ("Rejected", "Accepted"),
    "finding_is_not_source_observation": ("Rejected", "Accepted"),
    "recommendation_needs_authorization_and_effect": ("Unauthorized", "Accepted"),
    "foreign_origin_survives_recognition": ("Rejected", "Accepted"),
    "qualification_does_not_prove_physical_work": ("Rejected", "Accepted"),
    "availability_requires_temporal_binding": ("Rejected", "Accepted"),
    "single_success_is_not_general_capability": ("Rejected", "Accepted"),
}
for case_id, expected in expected_outcomes.items():
    item = by_id[case_id]
    if (item.get("expected_without_binding"), item.get("expected_with_binding")) != expected:
        raise SystemExit(f"{case_id}: frozen transition expectation mismatch")
recommendation = by_id["recommendation_needs_authorization_and_effect"]
if recommendation.get("expected_authorization_only") != "Unauthorized":
    raise SystemExit("authorization alone must not imply an operational effect")
if by_id["unknown_delivery_is_not_failure"].get("expected_without_binding") != "Unknown":
    raise SystemExit("unknown delivery must not be coerced to failure")
if by_id["foreign_origin_survives_recognition"].get("expected_origin") != "Foreign(node-b)":
    raise SystemExit("foreign origin must remain explicit")
if by_id["later_success_preserves_failure_history"].get("expected_after_later_success") != "Preserved":
    raise SystemExit("later success must not erase failure history")
if manifest.get("rust_test_target") != "d6s_canon_2_authority_boundary":
    raise SystemExit("D6S-CANON-2 Rust target mapping mismatch")
if "do not prove runtime authorization" not in manifest.get("claim_ceiling", ""):
    raise SystemExit("D6S-CANON-2 manifest claim ceiling was weakened")
print(f"D6S-CANON-2 structural verifier: PASS mutations={len(mutations)} sha256={actual_digest} claim_ceiling=ReferenceModelOnly")
