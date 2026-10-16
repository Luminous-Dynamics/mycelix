#!/usr/bin/env python3
"""Adversarial schema/semantic regression tests for prediction-market evidence v1."""
from __future__ import annotations

import copy
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_prediction_market_evidence_v1 import (  # noqa: E402
    CONTRACT_INVALID_PATH, CONTRACT_SCHEMA_PATH, CONTRACT_VALID_PATH,
    EVENT_INVALID_PATH, EVENT_SCHEMA_PATH, EVENT_VALID_PATH, DuplicateJsonKey,
    contract_content_digest, event_content_digest, load_json, make_validator,
    validate_contract_record, validate_resolution_event_chain, validate_resolution_event_record,
)

CONTRACT = load_json(CONTRACT_VALID_PATH)
EVENT = load_json(EVENT_VALID_PATH)
CV = make_validator(CONTRACT_SCHEMA_PATH)
EV = make_validator(EVENT_SCHEMA_PATH)


def _challenge_event(index: int, predecessor: dict, minute: int) -> dict:
    candidate = copy.deepcopy(EVENT)
    candidate["event_id"] = f"pm-resolution-event-example-{index}"
    candidate["event_index"] = index
    candidate["event_kind"] = "resolution_challenged"
    candidate.pop("outcome", None)
    candidate.pop("rationale", None)
    candidate["challenge_basis"] = "Synthetic challenge fixture."
    candidate["recorded_at"] = f"2030-01-02T12:{minute:02d}:00Z"
    candidate["predecessor_event_digest_sha256"] = predecessor["event_digest_sha256"]
    candidate["event_digest_sha256"] = event_content_digest(candidate)
    candidate["attestation"]["signed_event_digest_sha256"] = candidate["event_digest_sha256"]
    return candidate


class PredictionMarketEvidenceTests(unittest.TestCase):
    def test_both_schemas_are_valid_draft_2020_12(self):
        make_validator(CONTRACT_SCHEMA_PATH)
        make_validator(EVENT_SCHEMA_PATH)

    def test_positive_contract_and_event_fixtures_pass(self):
        self.assertEqual(validate_contract_record(CONTRACT, CV), [])
        self.assertEqual(validate_resolution_event_record(EVENT, CONTRACT, EV), [])

    def test_declared_negative_fixtures_fail(self):
        self.assertTrue(validate_contract_record(load_json(CONTRACT_INVALID_PATH), CV))
        self.assertTrue(validate_resolution_event_record(load_json(EVENT_INVALID_PATH), CONTRACT, EV))

    def test_every_required_contract_root_field_fails_when_missing(self):
        for field in CV.schema["required"]:
            with self.subTest(field=field):
                candidate = copy.deepcopy(CONTRACT)
                candidate.pop(field, None)
                self.assertTrue(validate_contract_record(candidate, CV))

    def test_every_required_event_root_field_fails_when_missing(self):
        for field in EV.schema["required"]:
            with self.subTest(field=field):
                candidate = copy.deepcopy(EVENT)
                candidate.pop(field, None)
                self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_unknown_top_level_properties_are_rejected(self):
        contract = copy.deepcopy(CONTRACT)
        contract["prediction"] = 0.99
        event = copy.deepcopy(EVENT)
        event["final_truth"] = True
        self.assertTrue(validate_contract_record(contract, CV))
        self.assertTrue(validate_resolution_event_record(event, CONTRACT, EV))

    def test_contract_digest_matches_canonical_jcs_bytes(self):
        self.assertEqual(contract_content_digest(CONTRACT), CONTRACT["contract_digest_sha256"])

    def test_contract_change_without_digest_update_is_rejected(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["event_statement"] += " altered"
        self.assertTrue(validate_contract_record(candidate, CV))

    def test_contract_change_with_recomputed_digest_is_admitted(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["event_statement"] += " revised under a synthetic new revision"
        candidate["revision"] = 2
        candidate["supersedes_contract_digest_sha256"] = CONTRACT["contract_digest_sha256"]
        candidate["contract_digest_sha256"] = contract_content_digest(candidate)
        self.assertEqual(validate_contract_record(candidate, CV), [])

    def test_event_digest_matches_canonical_jcs_bytes(self):
        self.assertEqual(event_content_digest(EVENT), EVENT["event_digest_sha256"])

    def test_event_change_without_digest_update_is_rejected(self):
        candidate = copy.deepcopy(EVENT)
        candidate["rationale"] += " altered after digest commitment"
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_event_digest_recomputation_does_not_hide_old_attestation(self):
        candidate = copy.deepcopy(EVENT)
        candidate["rationale"] += " changed"
        candidate["event_digest_sha256"] = event_content_digest(candidate)
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_contiguous_resolution_event_chain_is_admitted(self):
        second = _challenge_event(2, EVENT, 2)
        self.assertEqual(validate_resolution_event_chain([EVENT, second], CONTRACT, EV), [])

    def test_resolution_chain_rejects_gap_and_duplicate_index(self):
        gap = _challenge_event(3, EVENT, 3)
        errors = validate_resolution_event_chain([EVENT, gap], CONTRACT, EV)
        self.assertTrue(any("expected event_index 2, got 3" in error for error in errors))

        second = _challenge_event(2, EVENT, 2)
        fork = _challenge_event(2, EVENT, 3)
        errors = validate_resolution_event_chain([EVENT, second, fork], CONTRACT, EV)
        self.assertTrue(any("duplicate event_index 2" in error for error in errors))
        self.assertTrue(any("expected event_index 3, got 2" in error for error in errors))

    def test_resolution_chain_rejects_wrong_predecessor(self):
        second = _challenge_event(2, EVENT, 2)
        second["predecessor_event_digest_sha256"] = "e" * 64
        second["event_digest_sha256"] = event_content_digest(second)
        second["attestation"]["signed_event_digest_sha256"] = second["event_digest_sha256"]
        errors = validate_resolution_event_chain([EVENT, second], CONTRACT, EV)
        self.assertTrue(any("predecessor digest mismatch" in error for error in errors))

    def test_resolution_chain_rejects_duplicate_event_ids(self):
        second = _challenge_event(2, EVENT, 2)
        second["event_id"] = EVENT["event_id"]
        second["event_digest_sha256"] = event_content_digest(second)
        second["attestation"]["signed_event_digest_sha256"] = second["event_digest_sha256"]
        errors = validate_resolution_event_chain([EVENT, second], CONTRACT, EV)
        self.assertTrue(any("duplicate event_id" in error for error in errors))

    def test_resolution_chain_requires_increasing_record_times(self):
        second = _challenge_event(2, EVENT, 2)
        second["recorded_at"] = "2030-01-02T11:59:00Z"
        second["event_digest_sha256"] = event_content_digest(second)
        second["attestation"]["signed_event_digest_sha256"] = second["event_digest_sha256"]
        errors = validate_resolution_event_chain([EVENT, second], CONTRACT, EV)
        self.assertTrue(any("recorded_at is not increasing" in error for error in errors))

    def test_empty_resolution_chain_is_not_vacuously_valid(self):
        self.assertTrue(validate_resolution_event_chain([], CONTRACT, EV))

    def test_revision_two_requires_predecessor_contract_digest(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["revision"] = 2
        self.assertTrue(validate_contract_record(candidate, CV))

    def test_revision_one_rejects_supersedes_digest(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["supersedes_contract_digest_sha256"] = "e" * 64
        self.assertTrue(validate_contract_record(candidate, CV))

    def test_resolution_decision_requires_outcome(self):
        candidate = copy.deepcopy(EVENT)
        candidate["event_kind"] = "resolution_decided"
        candidate["decision_reason"] = "synthetic decision"
        candidate["decision_policy_digest_sha256"] = "f" * 64
        candidate.pop("outcome")
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_void_requires_reason_and_rule(self):
        candidate = copy.deepcopy(EVENT)
        candidate["event_kind"] = "market_voided"
        candidate["outcome"] = "void"
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_challenge_requires_basis(self):
        candidate = copy.deepcopy(EVENT)
        candidate["event_kind"] = "resolution_challenged"
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_cross_record_contract_identity_is_checked(self):
        for field, value in [
            ("contract_id", "other"),
            ("contract_revision", 2),
            ("contract_digest_sha256", "f" * 64),
        ]:
            with self.subTest(field=field):
                candidate = copy.deepcopy(EVENT)
                candidate[field] = value
                self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_attestation_must_name_the_declared_event_digest(self):
        candidate = copy.deepcopy(EVENT)
        candidate["attestation"]["signed_event_digest_sha256"] = "f" * 64
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_resolution_deadline_cannot_precede_event_deadline(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["resolution_policy"]["resolution_deadline_at"] = "2029-12-31T00:00:00Z"
        self.assertTrue(validate_contract_record(candidate, CV))

    def test_contract_creation_after_event_deadline_is_rejected(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["created_at"] = "2031-01-01T00:00:00Z"
        self.assertTrue(validate_contract_record(candidate, CV))

    def test_event_after_resolution_deadline_is_rejected(self):
        candidate = copy.deepcopy(EVENT)
        candidate["recorded_at"] = "2031-01-01T00:00:00Z"
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_artifact_retrieval_before_publication_is_rejected(self):
        candidate = copy.deepcopy(EVENT)
        candidate["evidence_artifacts"][0]["retrieved_at"] = "2030-01-01T09:00:00Z"
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_source_publication_timestamp_with_utc_offset_is_normalized(self):
        candidate = copy.deepcopy(EVENT)
        candidate["evidence_artifacts"][0]["published_at"] = "2030-01-02T12:00:00+02:00"
        self.assertEqual(validate_resolution_event_record(candidate, CONTRACT, EV), [])

    def test_event_cannot_claim_evidence_retrieved_after_the_event(self):
        candidate = copy.deepcopy(EVENT)
        candidate["evidence_artifacts"][0]["retrieved_at"] = "2030-01-02T13:00:00Z"
        candidate["event_digest_sha256"] = event_content_digest(candidate)
        candidate["attestation"]["signed_event_digest_sha256"] = candidate["event_digest_sha256"]
        self.assertTrue(validate_resolution_event_record(candidate, CONTRACT, EV))

    def test_non_utc_timestamp_is_rejected(self):
        candidate = copy.deepcopy(CONTRACT)
        candidate["created_at"] = "2026-10-16T12:00:00+02:00"
        self.assertTrue(validate_contract_record(candidate, CV))

    def test_duplicate_json_object_keys_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text('{"schema_version":"one","schema_version":"two"}', encoding="utf-8")
            with self.assertRaises(DuplicateJsonKey):
                load_json(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
