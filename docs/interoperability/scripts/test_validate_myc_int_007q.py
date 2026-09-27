#!/usr/bin/env python3
import copy
import json
import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from validate_myc_int_007q import validate  # noqa: E402

REPO_ROOT = SCRIPT_DIR.parents[2]
CHECKPOINT = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007Q_OBSERVED_MATURATION_CHECKPOINT.json"
TRANSITION = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007Q_MATURATION_TRANSITION_RECORD.json"
HOSTILES = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007Q_HOSTILE_CASES.json"


class ObservedMaturationFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.checkpoint = json.loads(CHECKPOINT.read_text(encoding="utf-8"))
        cls.transition = json.loads(TRANSITION.read_text(encoding="utf-8"))
        cls.hostiles = json.loads(HOSTILES.read_text(encoding="utf-8"))

    def assert_rejected(self, mutate):
        checkpoint = copy.deepcopy(self.checkpoint)
        transition = copy.deepcopy(self.transition)
        hostiles = copy.deepcopy(self.hostiles)
        mutate(checkpoint, transition, hostiles)
        self.assertTrue(validate(checkpoint, transition, hostiles))

    def test_00_baseline_passes(self):
        self.assertEqual(validate(self.checkpoint, self.transition, self.hostiles), [])

    def test_01_real_world_claim_rejected(self):
        self.assert_rejected(lambda c, t, h: c.__setitem__("real_world_observation_claimed", True))

    def test_02_observed_relabel_rejected(self):
        self.assert_rejected(lambda c, t, h: c.__setitem__("fixture_provenance", "ObservedPhysical"))

    def test_03_planned_copy_promotion_rejected(self):
        self.assert_rejected(
            lambda c, t, h: c["planned_reference"].__setitem__("copied_as_observation", True)
        )

    def test_04_removed_dimension_rejected(self):
        self.assert_rejected(lambda c, t, h: c["dimension_entries"].pop())

    def test_05_duplicate_dimension_rejected(self):
        def mutate(c, t, h):
            c["dimension_entries"][1]["dimension"] = c["dimension_entries"][0]["dimension"]
        self.assert_rejected(mutate)

    def test_06_invalid_dependency_state_rejected(self):
        self.assert_rejected(
            lambda c, t, h: c["dimension_entries"][0].__setitem__("dependency_state", "MostlyLocal")
        )

    def test_07_invalid_source_class_rejected(self):
        self.assert_rejected(
            lambda c, t, h: c["dimension_entries"][0].__setitem__("source_class", "AITruth")
        )

    def test_08_missing_currentness_rejected(self):
        self.assert_rejected(lambda c, t, h: c["dimension_entries"][0].pop("currentness"))

    def test_09_checkpoint_authority_rejected(self):
        self.assert_rejected(lambda c, t, h: c.__setitem__("authority", "Governance"))

    def test_10_checkpoint_maturity_claim_rejected(self):
        self.assert_rejected(
            lambda c, t, h: c["claim_ceiling"].__setitem__("lifecycle_generation_established", True)
        )

    def test_11_transition_generation_drift_rejected(self):
        self.assert_rejected(lambda c, t, h: t.__setitem__("candidate_to_generation", "N2"))

    def test_12_required_evidence_removed_rejected(self):
        self.assert_rejected(lambda c, t, h: t["required_evidence"].pop())

    def test_13_satisfied_evidence_omitted_rejected(self):
        self.assert_rejected(lambda c, t, h: t["satisfied_evidence"].pop())

    def test_14_transition_authority_rejected(self):
        self.assert_rejected(lambda c, t, h: t.__setitem__("authority", "FederationAuthority"))

    def test_15_transition_established_claim_rejected(self):
        self.assert_rejected(
            lambda c, t, h: t["claim_ceiling"].__setitem__(
                "technical_transition_established_under_profile", True
            )
        )

    def test_16_hostile_removed_rejected(self):
        self.assert_rejected(lambda c, t, h: h["cases"].pop())

    def test_17_hostile_expected_vocabulary_drift_rejected(self):
        self.assert_rejected(lambda c, t, h: h["cases"][0].__setitem__("expected", "PASS"))

    def test_18_unrecognized_analysis_as_observation_rejected(self):
        self.assert_rejected(
            lambda c, t, h: c["dimension_entries"][0].__setitem__(
                "source_class", "SymthaeaDirectObservation"
            )
        )

    def test_19_transition_federation_claim_rejected(self):
        self.assert_rejected(
            lambda c, t, h: t["claim_ceiling"].__setitem__(
                "federation_membership_granted", True
            )
        )


if __name__ == "__main__":
    unittest.main()
