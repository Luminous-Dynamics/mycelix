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


# Keep the manifest's control inventory tied to real, named tests. This prevents
# the manifest from claiming a regression control that the workflow never runs.
_MANIFEST_CONTROL_TESTS = {
    "missing-funding-commitment-evidence": "test_missing_commitment_evidence_is_rejected",
    "unknown-wage-regime-after-funding": "test_unknown_paid_role_classification_cannot_advance",
    "open-tender-treated-as-active": "test_open_tender_cannot_be_active_work",
    "automated-authorization": "test_automated_authorization_is_rejected",
    "non-monotonic-lifecycle": "test_lifecycle_from_status_must_match_previous_event",
    "underfunded-commitment": "test_commitment_must_cover_full_period_cost",
    "funded-period-must-cover-full-scope": "test_funded_period_must_cover_full_scope",
    "funded-work-requires-complete-scope-period": "test_funded_work_requires_complete_scope_period",
    "provider-self-verification": "test_provider_cannot_verify_own_work",
    "cancellation-does-not-erase-historical-funding-and-authorization-gates":
        "test_cancellation_does_not_erase_historical_funding_and_authorization_gates",
    "cancellation-after-execution-retains-worker-protection-gates":
        "test_cancellation_after_execution_retains_worker_protection_gates",
    "closure-does-not-erase-independent-verification-checks":
        "test_closure_does_not_erase_independent_verification_checks",
    "duplicate-json-keys": "test_duplicate_json_keys_are_rejected",
    "open-tender-route-mismatch": "test_open_tender_route_cannot_hide_as_not_applicable",
    "cost-total-mismatch": "test_cost_breakdown_total_must_match_declared_estimate",
    "missing-cost-category-assessment": "test_every_cost_category_must_be_assessed_once",
    "duplicate-cost-item-id": "test_duplicate_cost_line_ids_are_rejected",
    "cost-period-out-of-scope": "test_cost_period_cannot_exceed_work_scope",
    "not-applicable-category-has-cost-line": "test_not_applicable_category_cannot_have_a_cost_line",
    "missing-environment-and-closeout-costs": "test_environment_and_closeout_categories_cannot_be_omitted",
    "paid-role-with-zero-wage-cost": "test_paid_role_requires_positive_wage_cost",
    "currency-minor-unit-scale-mismatch": "test_currency_minor_unit_scale_must_match_currency",
    "unsupported-currency-minor-unit-rule": "test_unlisted_currency_scale_is_rejected_instead_of_guessed",
    "supported-zero-decimal-currency-scale": "test_supported_zero_decimal_currency_scale_is_accepted",
}


class CivicWorkPackageTests(unittest.TestCase):
    def test_manifest_control_inventory_matches_executable_tests(self) -> None:
        manifest = load_json(ROOT / "docs/architecture/civic-work-package-fixtures/manifest.v0.1.json")
        negative = set(manifest.get("required_negative_controls", []))
        positive_boundaries = set(manifest.get("required_positive_boundary_controls", []))
        self.assertFalse(negative & positive_boundaries, "a control cannot be both negative and positive")
        self.assertEqual(
            positive_boundaries,
            {"supported-zero-decimal-currency-scale"},
            "positive boundary cases must not be labelled as negative controls",
        )
        self.assertEqual(
            negative | positive_boundaries,
            set(_MANIFEST_CONTROL_TESTS),
            "manifest control inventory must match the reviewed registry exactly",
        )
        missing_tests = {
            control: method
            for control, method in _MANIFEST_CONTROL_TESTS.items()
            if not callable(getattr(self, method, None))
        }
        self.assertEqual(missing_tests, {}, "every manifest control must have an executable test")

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
        # Attack 1: a structurally valid authorization that disclaims human
        # approval must fail the semantic authorization gate.
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["authorization"]["approved_by_human"] = False
        errors = validate_record(record, SCHEMA, "automated authorization without human approval")
        self.assertIn("CWP_E_HUMAN_AUTHORIZATION_MISSING", errors)

        # Attack 2: a human-approval field must not legitimize an automated
        # lifecycle actor advancing the package into an authorization-gated state.
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        authorization_event = next(
            event for event in record["lifecycle_events"]
            if event["to_status"] == "authorized"
        )
        authorization_event["actor_kind"] = "automated"
        errors = validate_record(record, SCHEMA, "automated lifecycle authorization")
        self.assertIn("CWP_E_AUTOMATED_GATED_TRANSITION", errors)

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

    def test_environment_and_closeout_categories_cannot_be_omitted(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        coverage = record["funding"]["cost_breakdown"]["category_coverage"]
        coverage[:] = [
            item for item in coverage
            if item["category"] not in {"environmental_mitigation", "end_of_life_disposal"}
        ]
        coverage.extend([
            {
                "category": "other",
                "disposition": "not_applicable",
                "rationale": "Duplicate control entry that cannot replace required categories.",
            },
            {
                "category": "wages",
                "disposition": "included",
                "rationale": "Duplicate control entry that cannot replace required categories.",
            },
        ])
        errors = validate_record(record, SCHEMA, "missing environmental and closeout coverage")
        self.assertIn("CWP_E_COST_COVERAGE_INCOMPLETE", errors)

    def test_funded_period_must_cover_full_scope(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["funded_period_start"] = "2026-12-01"
        errors = validate_record(record, SCHEMA, "funded period shorter than scope")
        self.assertIn("CWP_E_FUNDED_PERIOD_DOES_NOT_COVER_SCOPE", errors)

    def test_funded_work_requires_complete_scope_period(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["scope"]["period"]["start_date"] = None
        errors = validate_record(record, SCHEMA, "funded scope period missing")
        self.assertIn("CWP_E_FUNDED_SCOPE_PERIOD_MISSING", errors)

    def test_paid_role_requires_positive_wage_cost(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        wages = next(
            item for item in record["funding"]["cost_breakdown"]["cost_items"]
            if item["category"] == "wages"
        )
        wages["amount_minor_units"] = 0
        total = sum(item["amount_minor_units"] for item in record["funding"]["cost_breakdown"]["cost_items"])
        record["funding"]["full_period_cost_estimate_minor_units"] = total
        record["funding"]["committed_amount_minor_units"] = total
        errors = validate_record(record, SCHEMA, "paid role without wage budget")
        self.assertIn("CWP_E_PAID_ROLES_WITHOUT_POSITIVE_WAGES", errors)

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

    def test_currency_minor_unit_scale_must_match_currency(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["minor_unit_digits"] = 0
        errors = validate_record(record, SCHEMA, "currency scale mismatch")
        self.assertIn("CWP_E_CURRENCY_MINOR_UNIT_DIGITS_MISMATCH", errors)

    def test_unlisted_currency_scale_is_rejected_instead_of_guessed(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["currency"] = "XYZ"
        errors = validate_record(record, SCHEMA, "unsupported currency")
        self.assertIn("CWP_E_UNSUPPORTED_CURRENCY_MINOR_UNIT_RULE", errors)

    def test_supported_zero_decimal_currency_scale_is_accepted(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["funding"]["currency"] = "JPY"
        record["funding"]["minor_unit_digits"] = 0
        errors = validate_record(record, SCHEMA, "zero-decimal currency scale")
        self.assertEqual(errors, [], errors)

    def test_cancellation_does_not_erase_historical_funding_and_authorization_gates(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["status"] = "closed_or_cancelled"
        record["lifecycle_events"].append({
            "event_id": "event:SYNTH-005",
            "from_status": "authorized",
            "to_status": "closed_or_cancelled",
            "actor_ref": "person:synthetic-closer",
            "actor_kind": "human",
            "happened_at": "2026-10-14T12:11:00Z",
            "evidence_refs": ["evidence:SYNTH-CLOSEOUT"],
        })
        record["timestamps"]["updated_at"] = "2026-10-14T12:11:00Z"
        # A terminal status must not hide missing evidence or invalidate the
        # historical human-authorization decision.
        record["funding"]["commitment_evidence_ref"] = None
        record["authorization"]["approved_by_human"] = False
        errors = validate_record(record, SCHEMA, "closed after authorization")
        self.assertIn("CWP_E_FUNDING_NOT_COMMITTED", errors)
        self.assertIn("CWP_E_HUMAN_AUTHORIZATION_MISSING", errors)

    def test_cancellation_after_execution_retains_worker_protection_gates(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["status"] = "closed_or_cancelled"
        record["lifecycle_events"].extend([
            {
                "event_id": "event:SYNTH-005", "from_status": "authorized",
                "to_status": "announced", "actor_ref": "person:synthetic-authorizer",
                "actor_kind": "human", "happened_at": "2026-10-14T12:11:00Z",
                "evidence_refs": ["evidence:SYNTH-CONTRACT"],
            },
            {
                "event_id": "event:SYNTH-006", "from_status": "announced",
                "to_status": "active", "actor_ref": "person:synthetic-site-lead",
                "actor_kind": "human", "happened_at": "2026-10-14T12:12:00Z",
                "evidence_refs": ["evidence:SYNTH-START"],
            },
            {
                "event_id": "event:SYNTH-007", "from_status": "active",
                "to_status": "closed_or_cancelled", "actor_ref": "person:synthetic-closer",
                "actor_kind": "human", "happened_at": "2026-10-14T12:13:00Z",
                "evidence_refs": ["evidence:SYNTH-CLOSEOUT"],
            },
        ])
        record["timestamps"]["updated_at"] = "2026-10-14T12:13:00Z"
        record["protections"]["safety_plan_ref"] = None
        errors = validate_record(record, SCHEMA, "closed after execution")
        self.assertIn("CWP_E_WORKER_PROTECTION_MISSING", errors)

    def test_closure_does_not_erase_independent_verification_checks(self) -> None:
        record = load_json(FIXTURES / "authorized.synthetic.v0.1.json")
        record["status"] = "closed_or_cancelled"
        execution_actor = "org:synthetic-repair-cooperative"
        record["procurement"]["lifecycle_stage"] = "implementation"
        record["labor_plan"]["execution_actor_refs"] = [execution_actor]
        record["verification"].update({
            "status": "completed",
            "completion_criteria_ref": "criteria:SYNTH-001",
            "method_ref": "method:SYNTH-independent-inspection",
            "reviewer_ref": execution_actor,
            "reviewed_at": "2026-10-14T12:14:00Z",
            "reviewer_is_independent": True,
            "evidence": [{
                "record_ref": "evidence:SYNTH-INDEPENDENT-REPORT",
                "sha256": "a" * 64,
                "reviewed_by_ref": execution_actor,
                "visibility": "restricted",
            }],
        })
        record["lifecycle_events"].extend([
            {
                "event_id": "event:SYNTH-005", "from_status": "authorized",
                "to_status": "announced", "actor_ref": "person:synthetic-authorizer",
                "actor_kind": "human", "happened_at": "2026-10-14T12:11:00Z",
                "evidence_refs": ["evidence:SYNTH-CONTRACT"],
            },
            {
                "event_id": "event:SYNTH-006", "from_status": "announced",
                "to_status": "active", "actor_ref": "person:synthetic-site-lead",
                "actor_kind": "human", "happened_at": "2026-10-14T12:12:00Z",
                "evidence_refs": ["evidence:SYNTH-START"],
            },
            {
                "event_id": "event:SYNTH-007", "from_status": "active",
                "to_status": "completion_claimed", "actor_ref": execution_actor,
                "actor_kind": "human", "happened_at": "2026-10-14T12:13:00Z",
                "evidence_refs": ["evidence:SYNTH-COMPLETION-CLAIM"],
            },
            {
                "event_id": "event:SYNTH-008", "from_status": "completion_claimed",
                "to_status": "independently_verified", "actor_ref": execution_actor,
                "actor_kind": "human", "happened_at": "2026-10-14T12:14:00Z",
                "evidence_refs": ["evidence:SYNTH-INDEPENDENT-REPORT"],
            },
            {
                "event_id": "event:SYNTH-009", "from_status": "independently_verified",
                "to_status": "closed_or_cancelled", "actor_ref": "person:synthetic-closer",
                "actor_kind": "human", "happened_at": "2026-10-14T12:15:00Z",
                "evidence_refs": ["evidence:SYNTH-CLOSEOUT"],
            },
        ])
        record["timestamps"]["updated_at"] = "2026-10-14T12:15:00Z"
        errors = validate_record(record, SCHEMA, "closed after verification")
        self.assertIn("CWP_E_PROVIDER_SELF_VERIFICATION", errors)
        self.assertIn("CWP_E_REVIEWER_WAS_EXECUTION_ACTOR", errors)

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
