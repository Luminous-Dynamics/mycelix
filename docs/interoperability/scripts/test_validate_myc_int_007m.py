#!/usr/bin/env python3
import copy
import json
import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from validate_myc_int_007m import validate  # noqa: E402

REPO_ROOT = SCRIPT_DIR.parents[2]
A_PACKAGE = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007K_NODE_SEED_PACKAGE_A_TO_B.json"
B_RUN = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007K_BOOTSTRAP_RUN_B.json"
B_PACKAGE = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007M_NODE_SEED_PACKAGE_B_TO_C.json"
C_RUN = REPO_ROOT / "docs/interoperability/fixtures/MYC_INT_007M_BOOTSTRAP_RUN_C.json"


class RecursiveSeedingFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a_package = json.loads(A_PACKAGE.read_text(encoding="utf-8"))
        cls.b_run = json.loads(B_RUN.read_text(encoding="utf-8"))
        cls.b_package = json.loads(B_PACKAGE.read_text(encoding="utf-8"))
        cls.c_run = json.loads(C_RUN.read_text(encoding="utf-8"))

    def assert_rejected(self, mutate):
        a_package = copy.deepcopy(self.a_package)
        b_run = copy.deepcopy(self.b_run)
        b_package = copy.deepcopy(self.b_package)
        c_run = copy.deepcopy(self.c_run)
        mutate(a_package, b_run, b_package, c_run)
        self.assertTrue(validate(a_package, b_run, b_package, c_run))

    def test_00_baseline_passes(self):
        self.assertEqual(
            validate(self.a_package, self.b_run, self.b_package, self.c_run), []
        )

    def test_01_b_package_contract_drift_rejected(self):
        self.assert_rejected(lambda ap, br, bp, cr: bp.__setitem__("contract_profile", "other"))

    def test_02_c_run_contract_drift_rejected(self):
        self.assert_rejected(lambda ap, br, bp, cr: cr.__setitem__("contract_profile", "other"))

    def test_03_a_required_for_b_package_rejected(self):
        self.assert_rejected(lambda ap, br, bp, cr: bp.__setitem__("original_seeder_A_required", True))

    def test_04_a_available_for_b_package_rejected(self):
        self.assert_rejected(lambda ap, br, bp, cr: bp.__setitem__("original_seeder_A_available", True))

    def test_05_b_package_authority_rejected(self):
        self.assert_rejected(lambda ap, br, bp, cr: bp.__setitem__("authority", "Governance"))

    def test_06_removed_class_rejected(self):
        self.assert_rejected(lambda ap, br, bp, cr: bp["components"].pop())

    def test_07_wrong_derived_origin_kind_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: bp["components"][0].__setitem__("origin_kind", "LocallyCreatedAtB")
        )

    def test_08_wrong_source_admission_ref_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: bp["components"][0].__setitem__("source_admission_ref", "wrong")
        )

    def test_09_stripped_a_provenance_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: bp["components"][0].__setitem__("public_provenance_lineage", [])
        )

    def test_10_fabricated_a_provenance_on_b_local_artifact_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: bp["components"][5].__setitem__(
                "public_provenance_lineage", ["A-physical-gen-1"]
            )
        )

    def test_11_authority_lineage_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: bp["lineage_policy"].__setitem__("authority_lineage_inherited", True)
        )

    def test_12_b_package_claim_upgrade_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: bp["claim_ceiling"].__setitem__("recursive_seeding_proven", True)
        )

    def test_13_b_package_c_run_generation_mismatch_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr.__setitem__("seed_package_generation", "other")
        )

    def test_14_c_identity_copied_from_b_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["identity_events_required"][0].__setitem__(
                "copied_from_seeder_B", True
            )
        )

    def test_15_c_identity_copied_from_a_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["identity_events_required"][0].__setitem__(
                "copied_from_original_A", True
            )
        )

    def test_16_b_governance_role_inherited_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["local_governance"].__setitem__("B_roles_inherited", True)
        )

    def test_17_b_certification_auto_local_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["foreign_artifact_rules"].__setitem__(
                "B_certification_auto_local", True
            )
        )

    def test_18_c_federation_dependency_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["federation"].__setitem__("required_for_bootstrap", True)
        )

    def test_19_a_reintroduced_in_c_independence_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["independence_test"].__setitem__("original_A_required", True)
        )

    def test_20_b_required_for_c_to_d_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["recursive_seeding_requirement"].__setitem__(
                "B_required_for_C_to_D", True
            )
        )

    def test_21_a_required_for_c_to_d_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["recursive_seeding_requirement"].__setitem__(
                "A_required_for_C_to_D", True
            )
        )

    def test_22_premature_recursive_seeding_claim_rejected(self):
        self.assert_rejected(
            lambda ap, br, bp, cr: cr["claim_ceiling"].__setitem__("recursive_seeding_proven", True)
        )


if __name__ == "__main__":
    unittest.main()
