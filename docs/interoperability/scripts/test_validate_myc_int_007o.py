#!/usr/bin/env python3
import copy
import json
import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from validate_myc_int_007o import validate  # noqa: E402

REPO_ROOT = SCRIPT_DIR.parents[2]
TRAJECTORY_PATH = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007O_NODE_MATURATION_TRAJECTORY.json"


class MaturationTrajectoryFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.trajectory = json.loads(TRAJECTORY_PATH.read_text(encoding="utf-8"))

    def assert_rejected(self, mutate):
        trajectory = copy.deepcopy(self.trajectory)
        mutate(trajectory)
        self.assertTrue(validate(trajectory))

    def test_00_baseline_passes(self):
        self.assertEqual(validate(self.trajectory), [])

    def test_01_observed_provenance_rejected(self):
        self.assert_rejected(lambda t: t.__setitem__("provenance_class", "ObservedPhysical"))

    def test_02_physical_observation_claim_rejected(self):
        self.assert_rejected(lambda t: t.__setitem__("physical_observation_claimed", True))

    def test_03_scalar_maturity_score_enabled_rejected(self):
        self.assert_rejected(lambda t: t.__setitem__("scalar_maturity_score_allowed", True))

    def test_04_scalar_self_sufficiency_enabled_rejected(self):
        self.assert_rejected(lambda t: t.__setitem__("scalar_self_sufficiency_score_allowed", True))

    def test_05_snapshot_removed_rejected(self):
        self.assert_rejected(lambda t: t["snapshots"].pop(2))

    def test_06_dimension_removed_rejected(self):
        self.assert_rejected(lambda t: t["snapshots"][2]["dependency_states"].pop())

    def test_07_duplicate_dimension_rejected(self):
        def mutate(t):
            t["snapshots"][2]["dependency_states"][1]["dimension"] = "food"
        self.assert_rejected(mutate)

    def test_08_invalid_dependency_state_rejected(self):
        self.assert_rejected(
            lambda t: t["snapshots"][1]["dependency_states"][0].__setitem__("state", "MostlyLocal")
        )

    def test_09_remaining_dependencies_hidden_rejected(self):
        self.assert_rejected(
            lambda t: t["snapshots"][4].__setitem__("remaining_external_dependencies_explicit", False)
        )

    def test_10_n5_seed_export_execution_claim_rejected(self):
        self.assert_rejected(
            lambda t: t["snapshots"][5].__setitem__("seed_export_state", "Executed")
        )

    def test_11_claim_ceiling_upgrade_rejected(self):
        self.assert_rejected(
            lambda t: t["claim_ceiling"].__setitem__("trajectory_observed", True)
        )

    def test_12_n4_critical_imports_hidden_rejected(self):
        def mutate(t):
            for item in t["snapshots"][4]["dependency_states"]:
                if item["dimension"] == "critical-imports":
                    item["state"] = "LocallyAvailable"
        self.assert_rejected(mutate)

    def test_13_n5_health_dependency_hidden_rejected(self):
        def mutate(t):
            for item in t["snapshots"][5]["dependency_states"]:
                if item["dimension"] == "health-safety-support":
                    item["state"] = "LocallyAvailable"
        self.assert_rejected(mutate)

    def test_14_n5_magic_dependency_improvement_rejected(self):
        def mutate(t):
            for item in t["snapshots"][5]["dependency_states"]:
                if item["dimension"] == "energy":
                    item["state"] = "LocallyAvailable"
        self.assert_rejected(mutate)

    def test_15_transition_evidence_removed_rejected(self):
        self.assert_rejected(lambda t: t["transition_evidence_requirements"].pop("N3->N4"))

    def test_16_root_overall_score_rejected(self):
        self.assert_rejected(lambda t: t.__setitem__("overall_score", 0.82))

    def test_17_snapshot_maturity_score_rejected(self):
        self.assert_rejected(lambda t: t["snapshots"][3].__setitem__("maturity_score", 4))

    def test_18_ui_single_overall_score_rejected(self):
        self.assert_rejected(
            lambda t: t["display_contract"].__setitem__("show_single_overall_score", True)
        )

    def test_19_snapshot_relabelled_observed_rejected(self):
        self.assert_rejected(
            lambda t: t["snapshots"][3].__setitem__("evidence_class", "ObservedPhysical")
        )


if __name__ == "__main__":
    unittest.main()
