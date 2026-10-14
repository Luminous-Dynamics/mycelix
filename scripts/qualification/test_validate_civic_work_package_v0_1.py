#!/usr/bin/env python3
"""Adversarial contract tests for the civic work-package reference validator."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_civic_work_package_v0_1 import SCHEMA_PATH, load_json, validate_record, _unique_pairs  # noqa: E402

SCHEMA = load_json(SCHEMA_PATH)
FIXTURES = ROOT / "docs/architecture/civic-work-package-fixtures"


class CivicWorkPackageTests(unittest.TestCase):
    def test_positive_fixtures_pass_schema_and_semantics(self) -> None:
        for name in ("proposed.synthetic.v0.1.json", "authorized.synthetic.v0.1.json"):
            with self.subTest(fixture=name):
                errors = validate_record(load_json(FIXTURES / name), SCHEMA, name)
                self.assertEqual(errors, [])

    def test_missing_commitment_evidence_is_rejected(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["commitment_evidence_ref"] = None
        errors = validate_record(record, SCHEMA, "missing commitment evidence")
        self.assertTrue(any(error.startswith("SCHEMA ") for error in errors), errors)

    def test_unknown_paid_role_classification_cannot_advance(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["labor_plan"]["roles"][0]["classification"].update({
            "status": "unknown", "classification_code": None, "wage_basis_ref": None,
            "source_ref": None, "reviewer_ref": None, "reviewed_at": None,
        })
        errors = validate_record(record, SCHEMA, "unknown wage classification")
        self.assertTrue(any("classification" in error for error in errors), errors)

    def test_open_tender_cannot_be_active_work(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["status"] = "active"
        record["procurement"].update({"route": "open_tender", "lifecycle_stage": "tender_open"})
        record["lifecycle_events"].extend([
            {"event_id":"event:SYNTH-005","from_status":"authorized","to_status":"announced","actor_ref":"person:synthetic-authorizer","actor_kind":"human","happened_at":"2026-10-14T12:11:00Z","evidence_refs":["evidence:SYNTH-CONTRACT"]},
            {"event_id":"event:SYNTH-006","from_status":"announced","to_status":"active","actor_ref":"person:synthetic-site-lead","actor_kind":"human","happened_at":"2026-10-14T12:12:00Z","evidence_refs":["evidence:SYNTH-START"]},
        ])
        record["timestamps"]["updated_at"] = "2026-10-14T12:12:00Z"
        errors = validate_record(record, SCHEMA, "open tender active")
        self.assertTrue(any("lifecycle_stage" in error or "PROCUREMENT_NOT_AUTHORIZED" in error for error in errors), errors)

    def test_open_tender_route_cannot_hide_as_not_applicable(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["procurement"].update({"route": "open_tender", "lifecycle_stage": "not_applicable"})
        errors = validate_record(record, SCHEMA, "open tender route mismatch")
        self.assertTrue(any("route" in error for error in errors), errors)

    def test_automated_authorization_is_rejected(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["authorization"]["approved_by_human"] = False
        errors = validate_record(record, SCHEMA, "automated authorization")
        self.assertTrue(any("approved_by_human" in error for error in errors), errors)

    def test_lifecycle_from_status_must_match_previous_event(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["lifecycle_events"][-1]["from_status"] = "proposed"
        errors = validate_record(record, SCHEMA, "lifecycle corruption")
        self.assertIn("CWP_E_LIFECYCLE_FROM_STATUS_MISMATCH", errors)

    def test_commitment_must_cover_full_period_cost(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["committed_amount_minor_units"] = 10000
        errors = validate_record(record, SCHEMA, "underfunded")
        self.assertIn("CWP_E_COMMITMENT_DOES_NOT_COVER_FULL_PERIOD_COST", errors)

    def test_cost_breakdown_total_must_match_declared_estimate(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["full_period_cost_estimate_minor_units"] += 1
        errors = validate_record(record, SCHEMA, "cost total mismatch")
        self.assertIn("CWP_E_COST_TOTAL_MISMATCH", errors)

    def test_every_cost_category_must_be_assessed_once(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["cost_breakdown"]["category_coverage"].pop()
        errors = validate_record(record, SCHEMA, "incomplete category coverage")
        self.assertTrue(
            "CWP_E_COST_COVERAGE_INCOMPLETE" in errors
            or any(error.startswith("SCHEMA ") for error in errors),
            errors,
        )

    def test_duplicate_cost_line_ids_are_rejected(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        items = record["funding"]["cost_breakdown"]["cost_items"]
        items[1]["line_item_id"] = items[0]["line_item_id"]
        errors = validate_record(record, SCHEMA, "duplicate cost line")
        self.assertIn("CWP_E_DUPLICATE_COST_ITEM_ID", errors)

    def test_cost_period_cannot_exceed_work_scope(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["cost_breakdown"]["cost_items"][0]["period_end"] = "2027-02-01"
        errors = validate_record(record, SCHEMA, "out-of-scope cost")
        self.assertIn("CWP_E_COST_PERIOD_OUT_OF_SCOPE", errors)

    def test_not_applicable_category_cannot_have_a_cost_line(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        item = dict(record["funding"]["cost_breakdown"]["cost_items"][0])
        item.update({
            "line_item_id": "cost-other-invalid",
            "category": "other",
        })
        record["funding"]["cost_breakdown"]["cost_items"].append(item)
        errors = validate_record(record, SCHEMA, "N/A category with cost")
        self.assertIn("CWP_E_NOT_APPLICABLE_CATEGORY_HAS_COST_LINE", errors)

    def test_provider_cannot_verify_own_work(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["status"] = "independently_verified"
        record["procurement"]["lifecycle_stage"] = "implementation"
        record["labor_plan"]["execution_actor_refs"] = ["org:synthetic-repair-cooperative"]
        record["verification"].update({
            "status":"completed",
            "completion_criteria_ref":"criteria:SYNTH-001",
            "method_ref":"method:SYNTH-independent-inspection",
            "reviewer_ref":"org:synthetic-repair-cooperative",
            "reviewed_at":"2026-10-14T12:20:00Z",
            "reviewer_is_independent":True,
            "evidence":[{"record_ref":"evidence:SYNTH-INDEPENDENT-REPORT","sha256":"a"*64,"reviewed_by_ref":"org:synthetic-repair-cooperative","visibility":"restricted"}],
        })
        record["lifecycle_events"].extend([
            {"event_id":"event:SYNTH-005","from_status":"authorized","to_status":"announced","actor_ref":"person:synthetic-authorizer","actor_kind":"human","happened_at":"2026-10-14T12:11:00Z","evidence_refs":["evidence:SYNTH-CONTRACT"]},
            {"event_id":"event:SYNTH-006","from_status":"announced","to_status":"active","actor_ref":"person:synthetic-site-lead","actor_kind":"human","happened_at":"2026-10-14T12:12:00Z","evidence_refs":["evidence:SYNTH-START"]},
            {"event_id":"event:SYNTH-007","from_status":"active","to_status":"completion_claimed","actor_ref":"org:synthetic-repair-cooperative","actor_kind":"human","happened_at":"2026-10-14T12:18:00Z","evidence_refs":["evidence:SYNTH-COMPLETION-CLAIM"]},
            {"event_id":"event:SYNTH-008","from_status":"completion_claimed","to_status":"independently_verified","actor_ref":"org:synthetic-repair-cooperative","actor_kind":"human","happened_at":"2026-10-14T12:20:00Z","evidence_refs":["evidence:SYNTH-INDEPENDENT-REPORT"]},
        ])
        record["timestamps"]["updated_at"] = "2026-10-14T12:20:00Z"
        errors = validate_record(record, SCHEMA, "self verification")
        self.assertIn("CWP_E_PROVIDER_SELF_VERIFICATION", errors)
        self.assertIn("CWP_E_REVIEWER_WAS_EXECUTION_ACTOR", errors)

    def test_duplicate_json_keys_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _unique_pairs([("status", "proposed"), ("status", "authorized")])


if __name__ == "__main__":
    unittest.main()
