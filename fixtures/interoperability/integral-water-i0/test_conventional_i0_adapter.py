#!/usr/bin/env python3
"""Candidate-side tests for MYC-INT-006F. No oracle values are imported."""

from __future__ import annotations

import unittest

import build_candidate_input as b
import conventional_i0_adapter as a


KINDS = {
    "water.system.alpha": "Resource",
    "water.issue.maintenance": "Issue",
    "water.observation.pressure": "Observation",
    "water.report.operator": "ActorReport",
    "water.evidence.bundle": "EvidenceBundle",
    "water.alternative.repair": "Alternative",
    "water.alternative.replace": "Alternative",
    "water.objection.access": "Objection",
    "water.prediction.pressure": "Prediction",
    "water.recommendation.repair": "Recommendation",
    "water.decision.repair": "Decision",
    "water.decision.foreign": "ForeignDecision",
    "water.authorization.repair": "Authorization",
    "water.attempt.repair": "ImplementationAttempt",
    "water.receipt.repair": "ImplementationReceipt",
    "water.outcome.pressure-low": "OutcomeObservation",
    "water.review.repair": "ReviewCandidate",
    "water.decision.supersede": "SupersedingDecisionCandidate",
    "water.credential.operator": "Credential",
    "water.certification.external": "Certification",
    "water.summary.derived": "DerivedSummary",
    "water.authority.foreign": "ForeignAuthority",
    "water.receipt.delivery": "DeliveryReceipt",
}


def subject_doc() -> dict:
    return {
        key: {
            "kind": kind,
            "semantic_ref": {"namespace": "test", "name": key.replace(".", "-"), "version": "1"},
        }
        for key, kind in KINDS.items()
    }


def candidate(cases: dict) -> dict:
    return {
        "candidate_input_id": b.INPUT_ID,
        "candidate_input_version": b.INPUT_VERSION,
        "profile": b.INPUT_PROFILE,
        "corpus_id": "myc-int-006c-water-i0",
        "corpus_version": "1.0.0",
        "stimulus_id": "myc-int-006e-water-i0-stimulus",
        "stimulus_version": "1.0.0",
        "subjects": subject_doc(),
        "cases": cases,
    }


class ConventionalAdapterTests(unittest.TestCase):
    def test_oracle_bearing_input_is_rejected(self) -> None:
        doc = candidate({"X": {"operation": "ExecuteRecommendation", "subjects": ["water.recommendation.repair"]}})
        doc["cases"]["X"]["expected_disposition"] = {"class": "Rejected"}
        with self.assertRaises(a.AdapterError):
            a.run_document(doc)

    def test_recommendation_cannot_execute(self) -> None:
        doc = candidate({"X": {"operation": "ExecuteRecommendation", "subjects": ["water.recommendation.repair", "water.system.alpha"]}})
        result = a.run_document(doc)["results"][0]
        self.assertEqual(result["disposition"], "Rejected")
        self.assertFalse(result["facts"]["effect_authority_granted"])

    def test_duplicate_delivery_has_one_effect(self) -> None:
        doc = candidate({"X": {
            "operation": "DeliverImplementationAttempt",
            "subjects": ["water.attempt.repair"],
            "transport": {"delivery_count": 2, "semantic_attempt_count": 1},
        }})
        result = a.run_document(doc)["results"][0]
        self.assertEqual(result["disposition"], "Accepted")
        self.assertEqual(result["effect_count"], 1)
        self.assertEqual(result["facts"]["logical_effect_count"], 1)
        self.assertTrue(result["facts"]["idempotent_replay"])

    def test_possible_persistence_timeout_is_indeterminate(self) -> None:
        doc = candidate({"X": {
            "operation": "ClassifyTimeoutOutcome",
            "subjects": ["water.attempt.repair", "water.receipt.delivery"],
            "transport": {"receiver_persistence": "possible", "acknowledgement": "missing", "sender_result": "timeout"},
        }})
        result = a.run_document(doc)["results"][0]
        self.assertEqual(result["disposition"], "Indeterminate")
        self.assertTrue(result["facts"]["unknown_state_preserved"])

    def test_expired_authorization_is_rejected(self) -> None:
        doc = candidate({"X": {
            "operation": "ExecuteWithExpiredAuthorization",
            "subjects": ["water.authorization.repair", "water.attempt.repair"],
            "authority": {"status": "expired"},
        }})
        result = a.run_document(doc)["results"][0]
        self.assertEqual(result["disposition"], "Rejected")
        self.assertTrue(result["facts"]["expired_authority_rejected"])

    def test_reorder_without_total_order_is_accepted(self) -> None:
        doc = candidate({"X": {
            "operation": "ProcessReorderedTransport",
            "subjects": ["water.report.operator", "water.observation.pressure", "water.evidence.bundle"],
            "transport": {"delivery_order": ["water.report.operator", "water.observation.pressure"], "total_order_promised": False},
        }})
        result = a.run_document(doc)["results"][0]
        self.assertEqual(result["disposition"], "Accepted")
        self.assertTrue(result["facts"]["provenance_preserved"])
        self.assertTrue(result["facts"]["distinct_semantic_subjects"])

    def test_prediction_is_not_observation(self) -> None:
        doc = candidate({"X": {
            "operation": "PromotePredictionToObservation",
            "subjects": ["water.prediction.pressure", "water.observation.pressure"],
        }})
        result = a.run_document(doc)["results"][0]
        self.assertEqual(result["disposition"], "Rejected")
        self.assertFalse(result["facts"]["observation_promotion"])

    def test_builder_strips_oracle_bearing_corpus_cases(self) -> None:
        corpus = {
            "corpus_id": "myc-int-006c-water-i0",
            "corpus_version": "1.0.0",
            "subjects": {
                "water.system.alpha": {
                    "kind": "Resource",
                    "semantic_ref": {"namespace": "test", "name": "water-system", "version": "1"},
                    "description": "not projected",
                }
            },
            "cases": {
                "I0-X": {
                    "expected_disposition": {"class": "Rejected", "reason_code": "SHOULD_NOT_LEAK"},
                    "assertions": [{"predicate": "SHOULD_NOT_LEAK"}],
                }
            },
        }
        stimulus = {
            "stimulus_id": "s",
            "stimulus_version": "1",
            "cases": {"I0-X": {"operation": "RecordLineage", "subjects": ["water.system.alpha"]}},
        }
        projected = b.build_candidate_input(corpus, stimulus)
        encoded = str(projected)
        self.assertNotIn("expected_disposition", encoded)
        self.assertNotIn("reason_code", encoded)
        self.assertNotIn("assertions", encoded)
        self.assertNotIn("description", encoded)

    def test_full_stimulus_has_no_unsupported(self) -> None:
        import json
        with open("myc-int-006e-water-i0-stimulus.v1.json", "r", encoding="utf-8") as handle:
            stimulus = json.load(handle)
        result = a.run_document(candidate(stimulus["cases"]))
        self.assertEqual(len(result["results"]), 19)
        self.assertFalse([row for row in result["results"] if row["disposition"] == "Unsupported"])
        for row in result["results"]:
            self.assertIn("facts", row)


if __name__ == "__main__":
    unittest.main()
