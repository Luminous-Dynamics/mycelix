"""Adversarial tests for the synthetic correction/appeal case contract."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import appeal_case_engine as engine  # noqa: E402


OWNER = "ACTOR-SYN-OWNER-001"
INTAKE = "ACTOR-SYN-INTAKE-001"
STEWARD_1 = "ACTOR-SYN-STEWARD-001"
STEWARD_2 = "ACTOR-SYN-STEWARD-002"
REVIEWER = "ACTOR-SYN-REVIEWER-001"
PRIVACY = "ACTOR-SYN-PRIVACY-001"
RETENTION_POLICY = {
    "status": "approved",
    "jurisdiction_code": "JURISDICTION_SYNTHETIC",
    "record_class": "service_correction_case",
    "lawful_basis_ref": "POLICY-LAWFUL-BASIS-001",
    "responsible_party_ref": "ACTOR-SYN-CONTROLLER-001",
    "max_retention_days": 1,
    "deletion_procedure_ref": "POLICY-DELETE-001",
    "backup_deletion_policy_ref": "POLICY-BACKUP-DELETE-001",
    "review_evidence_ref": "POLICY-REVIEW-001",
}


class AppealCaseEngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = engine.load_contract()

    def make_case(self) -> dict:
        return engine.new_case(
            "CASE-SYN-000001",
            OWNER,
            "SERVICE_APPOINTMENT_INTAKE",
            "ISSUE_RECORD_CORRECTION",
            self.contract,
            jurisdiction_code="JURISDICTION_SYNTHETIC",
        )

    def append(self, case: dict, events: list, **kwargs):
        return engine.append_event(case, events, contract=self.contract, **kwargs)

    def append_open(self, case: dict, events: list):
        return self.append(
            case, events, actor_ref=OWNER, actor_role="requester",
            action="open_case", timestamp_utc="2026-01-01T09:00:00Z",
        )

    def append_acknowledge(self, case: dict, events: list):
        return self.append(
            case, events, actor_ref=INTAKE, actor_role="case_steward",
            action="acknowledge_case", timestamp_utc="2026-01-01T09:01:00Z",
            target_actor_ref=STEWARD_1,
        )

    def build_appeal_flow(self, with_retention: bool = False):
        case = self.make_case()
        events: list[dict] = []
        case, events = self.append_open(case, events)
        if with_retention:
            case, events = self.append(
                case, events,
                actor_ref=PRIVACY, actor_role="privacy_officer",
                action="record_retention_policy", timestamp_utc="2026-01-01T09:00:30Z",
                policy_ref="POLICY-RETENTION-001",
                retention_policy=copy.deepcopy(RETENTION_POLICY),
            )
        case, events = self.append_acknowledge(case, events)
        case, events = self.append(
            case, events, actor_ref=STEWARD_1, actor_role="case_steward",
            action="assign_steward", timestamp_utc="2026-01-01T09:02:00Z",
            target_actor_ref=STEWARD_2,
        )
        case, events = self.append(
            case, events, actor_ref=OWNER, actor_role="requester",
            action="submit_correction", timestamp_utc="2026-01-01T09:03:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=STEWARD_2, actor_role="case_steward",
            action="resume_review", timestamp_utc="2026-01-01T09:04:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=STEWARD_2, actor_role="case_steward",
            action="issue_initial_decision", timestamp_utc="2026-01-01T09:05:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=OWNER, actor_role="requester",
            action="request_appeal", timestamp_utc="2026-01-01T09:06:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=REVIEWER, actor_role="independent_reviewer",
            action="start_appeal_review", timestamp_utc="2026-01-01T09:07:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=REVIEWER, actor_role="independent_reviewer",
            action="issue_appeal_decision", timestamp_utc="2026-01-01T09:08:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=STEWARD_2, actor_role="case_steward",
            action="close_case", timestamp_utc="2026-01-01T09:09:00Z",
        )
        return case, events

    def test_checked_in_contract_is_research_only_and_storage_disabled(self) -> None:
        self.assertEqual(engine.validate_contract(self.contract), [])
        self.assertIs(self.contract["production_enabled"], False)
        self.assertIs(self.contract["durable_case_storage_enabled"], False)
        self.assertIs(self.contract["public_dht_publication_enabled"], False)
        self.assertIs(self.contract["external_notifications_enabled"], False)

    def test_unimplemented_external_issue_types_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "issue_code must use the closed v1 issue enum"):
            engine.new_case(
                "CASE-SYN-UNSUPPORTED",
                OWNER,
                "SERVICE_APPOINTMENT_INTAKE",
                "SOURCE_RECORD_CORRECTION_REFERRAL",
                self.contract,
                jurisdiction_code="JURISDICTION_SYNTHETIC",
            )
        self.assertIn("SOURCE_RECORD_CORRECTION_REFERRAL", self.contract["not_implemented_issue_codes"])

    def test_case_envelope_has_only_opaque_refs_and_control_metadata(self) -> None:
        case = self.make_case()
        self.assertEqual(case["current_state"], "not_opened")
        self.assertEqual(case["case_owner_ref"], OWNER)
        self.assertIs(case["durable_case_storage_enabled"], False)
        self.assertIs(case["public_dht_publication_enabled"], False)
        self.assertEqual(case["retention_policy_status"], "not_assessed")

    def test_full_correction_appeal_flow_replays_successfully(self) -> None:
        case, events = self.build_appeal_flow()
        self.assertEqual(case["current_state"], "closed")
        self.assertEqual(engine.verify_event_chain(case, events, self.contract), [])
        self.assertEqual(len(events), 10)

    def test_original_decision_maker_cannot_review_the_appeal(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        case, events = self.append_acknowledge(case, events)
        case, events = self.append(
            case, events, actor_ref=STEWARD_1, actor_role="case_steward",
            action="assign_steward", timestamp_utc="2026-01-01T09:02:00Z",
            target_actor_ref=STEWARD_2,
        )
        case, events = self.append(
            case, events, actor_ref=STEWARD_2, actor_role="case_steward",
            action="issue_initial_decision", timestamp_utc="2026-01-01T09:03:00Z",
        )
        case, events = self.append(
            case, events, actor_ref=OWNER, actor_role="requester",
            action="request_appeal", timestamp_utc="2026-01-01T09:04:00Z",
        )
        with self.assertRaisesRegex(ValueError, "not authorized"):
            self.append(
                case, events, actor_ref=STEWARD_2, actor_role="independent_reviewer",
                action="start_appeal_review", timestamp_utc="2026-01-01T09:05:00Z",
            )

    def test_only_case_owner_can_request_correction(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        case, events = self.append_acknowledge(case, events)
        case, events = self.append(
            case, events, actor_ref=STEWARD_1, actor_role="case_steward",
            action="assign_steward", timestamp_utc="2026-01-01T09:02:00Z",
            target_actor_ref=STEWARD_2,
        )
        with self.assertRaisesRegex(ValueError, "not authorized"):
            self.append(
                case, events, actor_ref="ACTOR-SYN-OTHER-001", actor_role="requester",
                action="submit_correction", timestamp_utc="2026-01-01T09:03:00Z",
            )

    def test_requester_cannot_issue_or_close_decision(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        with self.assertRaisesRegex(ValueError, "not authorized"):
            self.append(
                case, events, actor_ref=OWNER, actor_role="requester",
                action="acknowledge_case", timestamp_utc="2026-01-01T09:01:00Z",
                target_actor_ref=OWNER,
            )

    def test_auditor_can_view_metadata_but_not_case_or_mutate(self) -> None:
        case = self.make_case()
        auditor = "ACTOR-SYN-AUDITOR-001"
        self.assertTrue(engine.authorize_action(case, auditor, "auditor", "view_audit_metadata"))
        self.assertFalse(engine.authorize_action(case, auditor, "auditor", "view_case"))
        self.assertFalse(engine.authorize_action(case, auditor, "auditor", "close_case"))

    def test_case_steward_scope_is_assignment_bound(self) -> None:
        case = self.make_case()
        case["assigned_steward_ref"] = STEWARD_1
        self.assertTrue(engine.authorize_action(case, STEWARD_1, "case_steward", "view_case"))
        self.assertFalse(engine.authorize_action(case, STEWARD_2, "case_steward", "view_case"))

    def test_retention_policy_must_be_complete_and_approved(self) -> None:
        self.assertEqual(engine.validate_retention_policy(RETENTION_POLICY), [])
        policy = copy.deepcopy(RETENTION_POLICY)
        policy["max_retention_days"] = 0
        self.assertTrue(engine.validate_retention_policy(policy))
        policy = copy.deepcopy(RETENTION_POLICY)
        policy["review_evidence_ref"] = None
        self.assertTrue(engine.validate_retention_policy(policy))

    def test_retention_policy_must_match_case_jurisdiction(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        changed = copy.deepcopy(RETENTION_POLICY)
        changed["jurisdiction_code"] = "JURISDICTION_OTHER"
        with self.assertRaisesRegex(ValueError, "jurisdiction must match"):
            self.append(
                case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
                action="record_retention_policy", timestamp_utc="2026-01-01T09:00:30Z",
                policy_ref="POLICY-RETENTION-001", retention_policy=changed,
            )

    def test_retention_policy_must_match_issue_to_record_class_mapping(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        changed = copy.deepcopy(RETENTION_POLICY)
        changed["record_class"] = "privacy_complaint_referral"
        with self.assertRaisesRegex(ValueError, "record class must match"):
            self.append(
                case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
                action="record_retention_policy", timestamp_utc="2026-01-01T09:00:30Z",
                policy_ref="POLICY-RETENTION-001", retention_policy=changed,
            )

    def test_retention_policy_is_not_approved_by_an_unprivileged_actor(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        with self.assertRaisesRegex(ValueError, "not authorized"):
            self.append(
                case, events, actor_ref=OWNER, actor_role="requester",
                action="record_retention_policy", timestamp_utc="2026-01-01T09:01:00Z",
                policy_ref="POLICY-RETENTION-001",
                retention_policy=copy.deepcopy(RETENTION_POLICY),
            )

    def test_retention_review_binds_policy_digest_in_event_chain(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        case, events = self.append(
            case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
            action="record_retention_policy", timestamp_utc="2026-01-01T09:00:30Z",
            policy_ref="POLICY-RETENTION-001",
            retention_policy=copy.deepcopy(RETENTION_POLICY),
        )
        self.assertTrue(case["retention_review_complete"])
        self.assertEqual(events[-1]["policy_digest"], engine._hash(RETENTION_POLICY))
        self.assertEqual(engine.verify_event_chain(case, events, self.contract), [])

    def test_expiry_is_rejected_before_retention_window(self) -> None:
        case, events = self.build_appeal_flow(with_retention=True)
        with self.assertRaisesRegex(ValueError, "retention window has not elapsed"):
            self.append(
                case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
                action="expire_case", timestamp_utc="2026-01-01T09:10:00Z",
            )

    def test_expiry_requires_approved_retention_and_elapsed_window(self) -> None:
        case, events = self.build_appeal_flow(with_retention=True)
        case, events = self.append(
            case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
            action="expire_case", timestamp_utc="2026-01-02T09:09:00Z",
        )
        self.assertEqual(case["current_state"], "expired")
        self.assertEqual(engine.verify_event_chain(case, events, self.contract), [])

    def test_case_envelope_is_bound_to_chain_genesis(self) -> None:
        case, events = self.build_appeal_flow()
        changed = copy.deepcopy(case)
        changed["service_code"] = "SERVICE_OTHER"
        self.assertTrue(engine.verify_event_chain(changed, events, self.contract))

    def test_modified_event_is_detected(self) -> None:
        case, events = self.build_appeal_flow()
        changed_events = copy.deepcopy(events)
        changed_events[2]["reason_code"] = "ACCESS_GRANTED"
        self.assertTrue(engine.verify_event_chain(case, changed_events, self.contract))

    def test_reordered_event_is_detected(self) -> None:
        case, events = self.build_appeal_flow()
        changed_events = copy.deepcopy(events)
        changed_events[1], changed_events[2] = changed_events[2], changed_events[1]
        self.assertTrue(engine.verify_event_chain(case, changed_events, self.contract))

    def test_timestamp_cannot_move_backwards(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        with self.assertRaisesRegex(ValueError, "cannot precede previous event"):
            self.append(
                case, events, actor_ref=INTAKE, actor_role="case_steward",
                action="acknowledge_case", timestamp_utc="2025-12-31T23:59:59Z",
                target_actor_ref=STEWARD_1,
            )

    def test_extra_free_text_or_identity_fields_are_rejected(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        bad = copy.deepcopy(events)
        bad[0]["person_name"] = "DO-NOT-STORE"
        self.assertTrue(engine.verify_event_chain(case, bad, self.contract))

    def test_event_chain_genesis_is_case_scoped(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        self.assertEqual(events[0]["previous_hash"], engine._genesis_hash(self.make_case(), self.contract))
        other = engine.new_case("CASE-SYN-000002", OWNER, "SERVICE_APPOINTMENT_INTAKE", "ISSUE_RECORD_CORRECTION", self.contract)
        self.assertNotEqual(events[0]["previous_hash"], engine._genesis_hash(other, self.contract))

    def test_bad_timestamps_and_handles_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "timestamp_utc"):
            self.append(
                self.make_case(), [], actor_ref=OWNER, actor_role="requester",
                action="open_case", timestamp_utc="2026-01-01T09:00:00+02:00",
            )
        with self.assertRaisesRegex(ValueError, "case-scoped opaque"):
            engine.new_case("NOT-A-CASE", OWNER, "SERVICE_APPOINTMENT_INTAKE", "ISSUE_RECORD_CORRECTION", self.contract)

    def test_synthetic_demo_runs_through_review_appeal_retention_and_expiry(self) -> None:
        report = engine.run_synthetic_demo("c" * 40, self.contract)
        self.assertEqual(report["event_count"], 12)
        self.assertEqual(report["terminal_state"], "expired")
        self.assertTrue(report["hash_chain_valid"])
        self.assertTrue(report["independent_appeal_reviewer"])
        self.assertTrue(report["retention_review_complete"])
        self.assertIs(report["durable_case_storage_enabled"], False)
        self.assertIs(report["public_dht_publication_enabled"], False)
        self.assertIs(report["durable_deletion_performed"], False)
        self.assertFalse(report["real_identity_data_processed"])
        self.assertFalse(report["external_endpoints_contacted"])
        self.assertNotIn("actor_ref", report["event_summaries"][0])

    def test_report_rejects_non_exact_commit_identifiers(self) -> None:
        with self.assertRaisesRegex(ValueError, "40-character lowercase commit SHA"):
            engine.run_synthetic_demo("not-a-commit", self.contract)

    def test_retention_policy_cannot_be_changed_after_approval(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        case, events = self.append(
            case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
            action="record_retention_policy", timestamp_utc="2026-01-01T09:00:30Z",
            policy_ref="POLICY-RETENTION-001",
            retention_policy=copy.deepcopy(RETENTION_POLICY),
        )
        changed_policy = copy.deepcopy(RETENTION_POLICY)
        changed_policy["max_retention_days"] = 2
        with self.assertRaisesRegex(ValueError, "immutable after approval"):
            self.append(
                case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
                action="record_retention_policy", timestamp_utc="2026-01-01T09:00:45Z",
                policy_ref="POLICY-RETENTION-002", retention_policy=changed_policy,
            )

    def test_modified_retention_policy_is_detected_by_chain_verifier(self) -> None:
        case = self.make_case()
        events = []
        case, events = self.append_open(case, events)
        case, events = self.append(
            case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
            action="record_retention_policy", timestamp_utc="2026-01-01T09:00:30Z",
            policy_ref="POLICY-RETENTION-001",
            retention_policy=copy.deepcopy(RETENTION_POLICY),
        )
        changed = copy.deepcopy(case)
        changed["retention_policy"]["max_retention_days"] = 9
        self.assertTrue(engine.verify_event_chain(changed, events, self.contract))

    def test_duplicate_contract_json_keys_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            json.loads(
                '{"production_enabled": false, "production_enabled": true}',
                object_pairs_hook=engine._reject_duplicate_object_keys,
            )

    def test_event_chain_does_not_claim_durable_deletion_was_performed(self) -> None:
        case, events = self.build_appeal_flow(with_retention=True)
        case, events = self.append(
            case, events, actor_ref=PRIVACY, actor_role="privacy_officer",
            action="expire_case", timestamp_utc="2026-01-02T09:09:00Z",
        )
        self.assertEqual(case["current_state"], "expired")
        self.assertIs(case["durable_case_storage_enabled"], False)
        self.assertEqual(case["public_dht_publication_enabled"], False)


if __name__ == "__main__":
    unittest.main()
