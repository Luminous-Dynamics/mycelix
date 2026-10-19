"""Adversarial tests for the synthetic, proof-only service journey."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import synthetic_service_journey as journey  # noqa: E402


class SyntheticServiceJourneyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.corpus = json.loads(
            (IDENTITY_DIR / "SYNTHETIC_SERVICE_JOURNEY_V1.json").read_text(encoding="utf-8")
        )
        cls.cases = {case["case_id"]: case for case in cls.corpus["cases"]}

    def base_request(self) -> dict:
        return copy.deepcopy(self.cases["SVC-001-MINIMAL-SUCCESS"]["request"])

    def test_checked_in_corpus_is_closed_and_synthetic_only(self) -> None:
        self.assertEqual(journey.validate_corpus(self.corpus), [])
        self.assertIs(self.corpus["production_enabled"], False)
        self.assertIs(self.corpus["no_external_calls"], True)
        self.assertIs(self.corpus["no_real_identity_data"], True)
        self.assertEqual(len(self.cases), 18)

    def test_all_frozen_cases_match_expected_disposition_reason_route_and_claims(self) -> None:
        report = journey.run_corpus(self.corpus, "a" * 40)
        self.assertTrue(report["all_expected_outcomes_match"])
        self.assertEqual(report["case_count"], 18)
        self.assertEqual(
            sum(row["actual"]["disposition"] == "MINIMAL_DISCLOSURE" for row in report["case_results"]),
            1,
        )

    def test_success_discloses_only_synthetic_reference_name(self) -> None:
        result = journey.evaluate_request(
            "SVC-TEST-001",
            self.base_request(),
        )
        self.assertEqual(result["disposition"], "MINIMAL_DISCLOSURE")
        self.assertEqual(result["disclosed_claim_names"], ["synthetic_subject_reference"])
        self.assertEqual(result["receipt"]["disclosed_claim_names"], ["synthetic_subject_reference"])

    def test_receipt_is_allow_listed_and_contains_no_claim_values(self) -> None:
        result = journey.evaluate_request("SVC-TEST-002", self.base_request())
        receipt = result["receipt"]
        self.assertEqual(
            set(receipt),
            {
                "receipt_id", "service_id", "purpose", "disposition", "reason_code",
                "disclosed_claim_names", "appeal_reference", "receipt_persistence",
                "contains_identity_material",
            },
        )
        self.assertIs(receipt["contains_identity_material"], False)
        self.assertEqual(receipt["receipt_persistence"], "ephemeral_output_only")
        rendered = json.dumps(receipt).lower()
        self.assertNotIn("national_identifier_value", rendered)
        self.assertNotIn("biometric_template", rendered)
        self.assertNotIn("replayable_presentation", rendered)

    def test_every_result_includes_a_synthetic_appeal_reference(self) -> None:
        for case in self.corpus["cases"]:
            with self.subTest(case_id=case["case_id"]):
                result = journey.evaluate_request(case["case_id"], case["request"])
                self.assertTrue(result["appeal_receipt_required"])
                self.assertIn(case["case_id"], result["receipt"]["appeal_reference"])

    def test_identity_cannot_authorize_benefit_eligibility(self) -> None:
        result = journey.evaluate_request(
            "SVC-TEST-003",
            self.cases["SVC-002-ENTITLEMENT-INFERENCE"]["request"],
        )
        self.assertEqual(result["disposition"], "DENIED")
        self.assertEqual(result["reason_code"], "IDENTITY_NOT_ENTITLEMENT")
        self.assertEqual(result["disclosed_claim_names"], [])

    def test_unknown_or_unavailable_verification_fails_closed_to_fallback(self) -> None:
        for case_id in ("SVC-005-VERIFIER-OUTAGE", "SVC-006-STALE-STATUS", "SVC-012-UNKNOWN-ISSUER-TRUST", "SVC-013-UNKNOWN-PROOF-STATE"):
            with self.subTest(case_id=case_id):
                case = self.cases[case_id]
                result = journey.evaluate_request(case_id, case["request"])
                self.assertEqual(result["disposition"], "INDETERMINATE")
                self.assertEqual(result["route"], "assisted_or_non_digital")
                self.assertEqual(result["disclosed_claim_names"], [])

    def test_missing_fallback_never_causes_identity_request_to_be_approved(self) -> None:
        case = self.cases["SVC-016-NO-FALLBACK-AVAILABLE"]
        result = journey.evaluate_request(case["case_id"], case["request"])
        self.assertEqual(result["disposition"], "INDETERMINATE")
        self.assertEqual(result["route"], "manual_review_required")
        self.assertEqual(result["disclosed_claim_names"], [])

    def test_excessive_claim_request_is_denied_even_if_evidence_has_that_claim(self) -> None:
        request = self.base_request()
        request["requested_claims"] = ["national_identifier"]
        result = journey.evaluate_request("SVC-TEST-004", request)
        self.assertEqual(result["disposition"], "DENIED")
        self.assertEqual(result["reason_code"], "CLAIM_OUTSIDE_PURPOSE_SCOPE")

    def test_purpose_grant_is_bound_to_exact_service_and_purpose(self) -> None:
        request = self.base_request()
        request["purpose"] = "different-service-purpose"
        result = journey.evaluate_request("SVC-TEST-005", request)
        self.assertEqual(result["disposition"], "DENIED")
        self.assertEqual(result["reason_code"], "PURPOSE_GRANT_INVALID")

    def test_replayed_nonce_is_rejected_and_not_fallback_approved(self) -> None:
        request = self.base_request()
        request["evidence"]["nonce_fresh"] = False
        result = journey.evaluate_request("SVC-TEST-006", request)
        self.assertEqual(result["disposition"], "REJECTED")
        self.assertEqual(result["reason_code"], "REPLAY_DETECTED")
        self.assertEqual(result["disclosed_claim_names"], [])

    def test_malformed_request_fails_closed_without_crashing(self) -> None:
        for malformed in (None, [], {"requested_outcome": []}):
            with self.subTest(malformed=malformed):
                result = journey.evaluate_request("SVC-TEST-MALFORMED", malformed)
                self.assertEqual(result["disposition"], "INDETERMINATE")
                self.assertEqual(result["reason_code"], "MALFORMED_SYNTHETIC_REQUEST")
                self.assertEqual(result["disclosed_claim_names"], [])

    def test_unhashable_enum_input_is_rejected_without_crashing(self) -> None:
        request = self.base_request()
        request["requested_outcome"] = []
        self.assertTrue(journey.validate_request(request))
        result = journey.evaluate_request("SVC-TEST-ENUM", request)
        self.assertEqual(result["disposition"], "INDETERMINATE")

    def test_corpus_cannot_be_promoted_to_production(self) -> None:
        changed = copy.deepcopy(self.corpus)
        changed["production_enabled"] = True
        self.assertTrue(any("production_enabled must be false" in e for e in journey.validate_corpus(changed)))

    def test_duplicate_case_ids_are_rejected(self) -> None:
        changed = copy.deepcopy(self.corpus)
        changed["cases"][1]["case_id"] = changed["cases"][0]["case_id"]
        self.assertTrue(any("duplicate case_id" in e for e in journey.validate_corpus(changed)))

    def test_forbidden_raw_material_fields_are_rejected(self) -> None:
        changed = copy.deepcopy(self.corpus)
        changed["cases"][0]["request"]["national_identifier_value"] = "SYNTHETIC-RAW-VALUE"
        self.assertTrue(any("request fields must match" in e or "must not contain raw identity" in e for e in journey.validate_corpus(changed)))

    def test_malformed_enum_values_fail_closed_without_crashing(self) -> None:
        changed = copy.deepcopy(self.corpus)
        changed["route_values"] = [[]]
        self.assertTrue(journey.validate_corpus(changed))
        changed = copy.deepcopy(self.corpus)
        changed["cases"][0]["expected"]["disposition"] = []
        self.assertTrue(journey.validate_corpus(changed))

    def test_expected_outcome_drift_is_rejected_by_frozen_contract(self) -> None:
        changed = copy.deepcopy(self.corpus)
        changed["cases"][0]["expected"]["reason_code"] = "UNJUSTIFIED_PASS"
        errors = journey.validate_corpus(changed)
        self.assertTrue(any("frozen scenario contract" in error for error in errors))
        with self.assertRaisesRegex(ValueError, "invalid synthetic service-journey corpus"):
            journey.run_corpus(changed, "b" * 40)

    def test_missing_golden_vector_is_rejected(self) -> None:
        changed = copy.deepcopy(self.corpus)
        changed["cases"].pop()
        errors = journey.validate_corpus(changed)
        self.assertTrue(any("exactly 18 frozen vectors" in error for error in errors))

    def test_duplicate_json_keys_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            json.loads(
                '{"production_enabled": false, "production_enabled": true}',
                object_pairs_hook=journey._reject_duplicate_object_keys,
            )


if __name__ == "__main__":
    unittest.main()
