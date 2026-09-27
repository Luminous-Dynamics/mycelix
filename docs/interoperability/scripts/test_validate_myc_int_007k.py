#!/usr/bin/env python3
import copy
import json
import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from validate_myc_int_007k import validate  # noqa: E402

REPO_ROOT = SCRIPT_DIR.parents[2]
PACKAGE_PATH = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007K_NODE_SEED_PACKAGE_A_TO_B.json"
RUN_PATH = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007K_BOOTSTRAP_RUN_B.json"


class SeedBootstrapFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = json.loads(PACKAGE_PATH.read_text(encoding="utf-8"))
        cls.bootstrap = json.loads(RUN_PATH.read_text(encoding="utf-8"))

    def assert_rejected(self, mutate):
        package = copy.deepcopy(self.package)
        run = copy.deepcopy(self.bootstrap)
        mutate(package, run)
        self.assertTrue(validate(package, run))

    def test_00_baseline_passes(self):
        self.assertEqual(validate(self.package, self.bootstrap), [])

    def test_01_package_authority_rejected(self):
        self.assert_rejected(lambda p, r: p.__setitem__("authority", "Governance"))

    def test_02_removed_seed_class_rejected(self):
        self.assert_rejected(lambda p, r: p["components"].pop())

    def test_03_duplicate_component_id_rejected(self):
        def mutate(p, r):
            p["components"][1]["component_id"] = p["components"][0]["component_id"]
        self.assert_rejected(mutate)

    def test_04_component_authority_rejected(self):
        self.assert_rejected(lambda p, r: p["components"][0].__setitem__("authority", "LocalAuthority"))

    def test_05_missing_private_key_prohibition_rejected(self):
        self.assert_rejected(lambda p, r: p["forbidden_inheritance"].remove("source-node-private-keys"))

    def test_06_package_claim_upgrade_rejected(self):
        self.assert_rejected(lambda p, r: p["claim_ceiling"].__setitem__("recipient_identity_precreated", True))

    def test_07_generation_mismatch_rejected(self):
        self.assert_rejected(lambda p, r: r.__setitem__("seed_package_generation", "other-generation"))

    def test_08_missing_admission_rejected(self):
        self.assert_rejected(lambda p, r: r["component_admission"].pop())

    def test_09_unknown_admission_decision_rejected(self):
        self.assert_rejected(lambda p, r: r["component_admission"][0].__setitem__("decision", "AutoAccept"))

    def test_10_accepted_without_local_generation_rejected(self):
        self.assert_rejected(lambda p, r: r["component_admission"][0].__setitem__("local_generation", None))

    def test_11_deferred_with_local_generation_rejected(self):
        self.assert_rejected(lambda p, r: r["component_admission"][-1].__setitem__("local_generation", "B-hidden-gen"))

    def test_12_copied_identity_rejected(self):
        self.assert_rejected(lambda p, r: r["identity_events_required"][0].__setitem__("copied_from_seeder", True))

    def test_13_inherited_governance_role_rejected(self):
        self.assert_rejected(lambda p, r: r["local_governance"].__setitem__("source_node_roles_inherited", True))

    def test_14_foreign_certification_autopromotion_rejected(self):
        self.assert_rejected(lambda p, r: r["foreign_artifact_rules"].__setitem__("source_certification_auto_local", True))

    def test_15_required_federation_rejected(self):
        self.assert_rejected(lambda p, r: r["federation"].__setitem__("required_for_bootstrap", True))

    def test_16_shortened_independence_interval_rejected(self):
        self.assert_rejected(lambda p, r: r["independence_test"].__setitem__("required_local_operation_interval_seconds", 3600))

    def test_17_a_required_for_b_to_c_rejected(self):
        self.assert_rejected(lambda p, r: r["recursive_seeding_requirement"].__setitem__("A_required_for_B_to_C", True))

    def test_18_missing_second_generation_evidence_rejected(self):
        self.assert_rejected(lambda p, r: r["evidence_required"].remove("second-generation-seed-package-export"))

    def test_19_unexecuted_independence_claim_rejected(self):
        self.assert_rejected(lambda p, r: r["claim_ceiling"].__setitem__("independence_proven", True))


if __name__ == "__main__":
    unittest.main()
