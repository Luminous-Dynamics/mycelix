"""Unit tests for the offline CrossDomainDisruptionV1 fixture preflight."""
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from validate_cross_domain_disruption_v1 import FIXTURE_NAMES, validate_fixture_dir


def valid_fixture_package() -> dict[str, dict]:
    source_refs = [
        "source:carrier-A",
        "source:supplier-A-portal",
        "source:warehouse-B-snapshot",
        "source:warehouse-inspection-B-refresh",
        "source:independent-arrival-audit",
    ]
    facility_refs = [f"subject:supplier-B-facility-{letter}" for letter in "ABCDE"]
    f0 = {
        "fixture_profile": "OPS-INTEL-TEST-001",
        "fixture_version": "CrossDomainDisruptionV1",
        "data_class": "synthetic",
        "visibility": "solver-visible-only",
        "authority_ceiling": "FixtureDescriptionOnly",
        "frontier": {"frontier_ref": "frontier:F0", "cutoff_utc": "2026-06-01T12:00:00Z"},
        "source_registry": [{"source_ref": ref} for ref in source_refs[:3]],
        "artifacts": [
            {"artifact_ref": "artifact:shipment-carrier", "source_ref": "source:carrier-A"},
            {"artifact_ref": "artifact:shipment-portal", "source_ref": "source:supplier-A-portal"},
            {"artifact_ref": "artifact:inventory-ABC", "source_ref": "source:warehouse-B-snapshot"},
        ],
        "subjects": [{"subject_ref": "subject:supplier-B"}] + [{"subject_ref": ref} for ref in facility_refs],
        "facility_registry": {"facility_refs_known": facility_refs},
        "observations": [
            {"observation_ref": "observation:shipment-carrier", "source_ref": "source:carrier-A", "artifact_ref": "artifact:shipment-carrier"},
            {"observation_ref": "observation:shipment-portal", "source_ref": "source:supplier-A-portal", "artifact_ref": "artifact:shipment-portal"},
            {
                "observation_ref": "observation:supplier-B-inventory-partial",
                "source_ref": "source:warehouse-B-snapshot",
                "artifact_ref": "artifact:inventory-ABC",
                "scope_ref": "scope:supplier-B-facilities-A-B-C",
                "aggregate_semantics": "SumWithinListedFacilitiesAtObservationTime",
                "coverage": {
                    "state": "PartialCoverage",
                    "facility_refs_observed": facility_refs[:3],
                    "facility_refs_known": facility_refs,
                },
            },
        ],
        "coverage_assertions": [{
            "coverage_ref": "coverage:supplier-B-unobserved-facilities",
            "facility_refs_observed": facility_refs[:3],
            "facility_refs_known": facility_refs,
        }],
        "candidate_interventions": [{"candidate_ref": "candidate:NoAction", "authority_ceiling": "CandidateOnly", "execution_material_present": False}],
        "protected_fields": [{"handling_state": "OmittedUnderPolicy", "payload_included": False}],
    }
    f1 = {
        "fixture_profile": "OPS-INTEL-TEST-001",
        "fixture_version": "CrossDomainDisruptionV1",
        "data_class": "synthetic",
        "parent_frontier_ref": "frontier:F0",
        "frontier_cutoff_utc": "2026-06-01T12:20:00Z",
        "added_sources": [{"source_ref": source_refs[3]}],
        "added_artifacts": [{"artifact_ref": "artifact:inventory-DE", "source_ref": source_refs[3]}],
        "added_observations": [{
            "observation_ref": "observation:supplier-B-inventory-refresh",
            "source_ref": source_refs[3],
            "artifact_ref": "artifact:inventory-DE",
            "scope_ref": "scope:supplier-B-facilities-D-E",
            "aggregate_semantics": "SumWithinListedFacilitiesAtObservationTime",
            "coverage": {"facility_refs_observed": facility_refs[3:]},
            "limitations": ["Do not sum F0 and F1 inventory snapshots."],
        }],
        "added_coverage_assessments": [{"aggregate_inference": "NotPermittedWithoutTemporalReconciliation"}],
    }
    f2 = {
        "fixture_profile": "OPS-INTEL-TEST-001",
        "fixture_version": "CrossDomainDisruptionV1",
        "data_class": "synthetic",
        "parent_frontier_ref": "frontier:F1",
        "frontier_cutoff_utc": "2026-06-01T12:32:00Z",
        "attempt": {"execution_mode": "SimulationOnly", "actual_execution_authorized": False, "effect_state": "Unobserved"},
        "authority_effect_cases": [{"expected_disposition": value} for value in (
            "BlockedNoCurrentPermit", "RejectStalePermit", "RejectWrongSubject", "RejectWrongCandidate", "RejectWrongPayload"
        )],
    }
    f3 = {
        "fixture_profile": "OPS-INTEL-TEST-001",
        "fixture_version": "CrossDomainDisruptionV1",
        "data_class": "synthetic",
        "parent_frontier_ref": "frontier:F2",
        "frontier_cutoff_utc": "2026-06-01T16:32:00Z",
        "added_sources": [{"source_ref": source_refs[4]}],
        "added_artifacts": [{"artifact_ref": "artifact:arrival-audit", "source_ref": source_refs[4]}],
        "outcome_observations": [{
            "source_ref": source_refs[4],
            "artifact_ref": "artifact:arrival-audit",
            "causal_attribution": "NotEstablished",
        }],
    }
    return {
        "f0": f0,
        "f1": f1,
        "f2": f2,
        "f3": f3,
        "predicates": {"predicates": [{"id": f"P{i:02d}"} for i in range(1, 21)]},
        "mutations": {"mutations": [{"id": f"M{i:02d}"} for i in range(1, 24)]},
    }


class FixturePreflightTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.fixture_dir = Path(self.temp.name)
        self.package = valid_fixture_package()
        for key, filename in FIXTURE_NAMES.items():
            (self.fixture_dir / filename).write_text(json.dumps(self.package[key]), encoding="utf-8")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def assert_invalid(self, key: str, mutate) -> None:
        value = self.package[key]
        mutate(value)
        (self.fixture_dir / FIXTURE_NAMES[key]).write_text(json.dumps(value), encoding="utf-8")
        self.assertTrue(validate_fixture_dir(self.fixture_dir), f"{key} mutation unexpectedly passed")

    def test_valid_fixture_package_passes(self) -> None:
        self.assertEqual(validate_fixture_dir(self.fixture_dir), [])

    def test_broken_f0_artifact_reference_fails(self) -> None:
        self.assert_invalid("f0", lambda value: value["observations"][0].update(artifact_ref="artifact:missing"))

    def test_supplier_facility_scope_overlap_or_gap_fails(self) -> None:
        self.assert_invalid("f1", lambda value: value["added_observations"][0]["coverage"].update(facility_refs_observed=["subject:supplier-B-facility-A"]))

    def test_future_cutoff_out_of_order_fails(self) -> None:
        self.assert_invalid("f2", lambda value: value.update(frontier_cutoff_utc="2026-06-01T11:59:00Z"))

    def test_policy_allow_without_mock_permit_negative_controls_fails(self) -> None:
        self.assert_invalid("f2", lambda value: value.update(authority_effect_cases=[]))

    def test_attempt_receipt_cannot_be_effect_success(self) -> None:
        self.assert_invalid("f2", lambda value: value["attempt"].update(effect_state="EffectSuccess"))

    def test_protected_payload_inclusion_fails(self) -> None:
        self.assert_invalid("f0", lambda value: value["protected_fields"][0].update(payload_included=True))

    def test_duplicate_predicate_ids_fail(self) -> None:
        self.assert_invalid("predicates", lambda value: value["predicates"][1].update(id=value["predicates"][0]["id"]))

    def test_hidden_evaluator_key_fails(self) -> None:
        self.assert_invalid("f0", lambda value: value.update(evaluator_only_world={"truth": True}))

    def test_duplicate_json_object_keys_fail(self) -> None:
        filename = FIXTURE_NAMES["f0"]
        (self.fixture_dir / filename).write_text('{"duplicate": 1, "duplicate": 2}', encoding="utf-8")
        self.assertTrue(validate_fixture_dir(self.fixture_dir))

    def test_non_object_f0_frontier_fails_without_crashing(self) -> None:
        self.assert_invalid("f0", lambda value: value.update(frontier=[]))

    def test_non_object_f0_coverage_fails_without_crashing(self) -> None:
        self.assert_invalid(
            "f0",
            lambda value: value["observations"][2].update(coverage=[]),
        )

    def test_unhashable_f0_facility_ref_fails_without_crashing(self) -> None:
        self.assert_invalid(
            "f0",
            lambda value: value["observations"][2]["coverage"].update(
                facility_refs_observed=[["subject:malformed"]]
            ),
        )

    def test_unhashable_f1_facility_ref_fails_without_crashing(self) -> None:
        self.assert_invalid(
            "f1",
            lambda value: value["added_observations"][0]["coverage"].update(
                facility_refs_observed=[["subject:malformed"]]
            ),
        )


if __name__ == "__main__":
    unittest.main()
