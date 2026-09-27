#!/usr/bin/env python3
import copy
import importlib.util
import json
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
VALIDATOR = HERE / "validate_myc_int_007i.py"
FIXTURE = HERE.parent / "fixtures" / "MYC_INT_007I_NODE_MATURATION_AND_SEEDING.json"

spec = importlib.util.spec_from_file_location("validator", VALIDATOR)
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)

with FIXTURE.open("r", encoding="utf-8") as f:
    PRISTINE = json.load(f)


class Test007IValidator(unittest.TestCase):
    def assertRejected(self, mutate):
        doc = copy.deepcopy(PRISTINE)
        mutate(doc)
        with self.assertRaises(validator.ValidationError):
            validator.validate_profile(doc)

    def test_pristine(self):
        self.assertTrue(validator.validate_profile(copy.deepcopy(PRISTINE)))

    def test_reject_binary_self_sustaining(self):
        self.assertRejected(lambda d: d.__setitem__("self_sustaining", True))

    def test_reject_scalar_capability_score(self):
        self.assertRejected(lambda d: d["capability_dimensions"][0].__setitem__("scalar_score", True))

    def test_reject_seed_authority(self):
        self.assertRejected(lambda d: d["seed_package_classes"][0].__setitem__("authority", "Governance"))

    def test_reject_missing_new_identity(self):
        self.assertRejected(lambda d: d["lifecycle_generations"][6]["required_properties"].remove("new-node-identity-and-keys"))

    def test_reject_required_federation(self):
        self.assertRejected(lambda d: d["runtime_optionalities"].__setitem__("federation_required_for_local_operation", True))

    def test_reject_auto_foreign_certification(self):
        self.assertRejected(lambda d: d["seed_import_rules"].remove("foreign-certification-requires-explicit-local-recognition"))

    def test_reject_donation_authority_boundary_loss(self):
        self.assertRejected(lambda d: d["seed_import_rules"].remove("donation-does-not-create-authority"))

    def test_reject_no_selective_admission(self):
        self.assertRejected(lambda d: d["lifecycle_generations"][5]["required_properties"].remove("receiving-node-can-selectively-admit-components"))

    def test_reject_no_local_governance(self):
        self.assertRejected(lambda d: d["lifecycle_generations"][6]["required_properties"].remove("local-governance-owned-by-receiving-community"))

    def test_reject_n7_original_seeder_dependency(self):
        self.assertRejected(lambda d: d["lifecycle_generations"][7]["required_properties"].remove("original-seeder-not-required"))

    def test_reject_claim_ceiling_upgrade(self):
        self.assertRejected(lambda d: d["claim_ceiling"].__setitem__("economic_self_sufficiency_established", True))

    def test_reject_parent_authority_relationship(self):
        self.assertRejected(lambda d: d["showcase_mapping"].__setitem__("node_A_to_B_relationship", "ParentAuthority"))

    def test_reject_missing_source_generation_rule(self):
        self.assertRejected(lambda d: d["seed_import_rules"].remove("every-imported-artifact-keeps-source-generation"))

    def test_reject_missing_adaptation_generation(self):
        self.assertRejected(lambda d: d["seed_import_rules"].remove("local-adaptation-creates-new-generation"))

    def test_reject_permanent_seeder_dependency(self):
        self.assertRejected(lambda d: d["runtime_optionalities"].__setitem__("seeder_online_required_after_complete_bootstrap", True))


if __name__ == "__main__":
    unittest.main(verbosity=2)
