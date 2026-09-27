import unittest

from validate_myc_int_018s import EXPECTED_H1_SUBJECTS, GENERIC_PROFILE, H2_PROFILE, validate


def bench_fixture():
    channels = [
        "flow-rate", "reservoir-level", "reservoir-volume", "solution-temperature", "leak-state",
        "ph", "electrical-conductivity", "dissolved-oxygen", "air-temperature", "relative-humidity",
        "subsystem-power", "subsystem-energy",
    ]
    return {"channels": [{"id": channel, "actuation_authority": False} for channel in channels]}


def generic_fixture():
    return {
        "profile_id": GENERIC_PROFILE,
        "profile_version": "1.0.0",
        "status": "design-contract",
        "whole_node_extrapolation_prohibited": True,
        "authority": "None",
        "allowed_source_classes": [
            "DirectObservation", "SourceOwnedOperationalFact", "ReconstructedState", "DerivedAssessment",
            "OperatorDeclaration", "ImportedForeignEvidence", "Unknown",
        ],
        "required_sections": [
            "loop_identity", "useful_output_profile", "production_window",
            "input_material_observations", "work_observations", "process_observation_refs",
            "useful_output_observations", "loss_waste_failure_observations",
            "outcome_feedback", "external_dependency_import_map", "currentness_coverage",
            "conflicts_unknowns", "correction_supersession_lineage",
        ],
        "input_material_observation_requirements": [
            "material_resource_subject", "quantity", "unit_profile", "source_class",
            "event_or_window_time", "origin_provenance", "local_or_external_source",
            "estimated_or_observed_status", "correction_lineage",
        ],
        "work_observation_requirements": [
            "work_event_subject", "actor_or_role_reference_under_privacy_profile",
            "bounded_time_or_duration_profile", "activity_class", "productive_loop_subject",
            "source_class", "evidence_refs", "correction_lineage",
        ],
        "work_nonclaims": {
            "compensation_owed_established": False,
            "itc_credit_issued": False,
            "governance_standing_granted": False,
        },
        "process_observation_contract": {
            "domain_owned": True,
            "referenced_not_duplicated": True,
            "analysis_can_replace_source_observation": False,
        },
        "useful_output_observation_requirements": [
            "output_subject", "output_profile", "quantity", "unit_profile", "event_or_window_time",
            "source_class", "disposition", "evidence_refs", "correction_lineage",
        ],
        "output_nonclaims": {
            "edible": False,
            "food_safe": False,
            "nutritionally_adequate": False,
            "marketable": False,
            "whole_node_dependency_reduced": False,
        },
        "loss_waste_failure_requirements": [
            "loss_subject_or_class", "quantity_or_explicit_unknown", "unit_profile_or_not_applicable",
            "cause_status_observed_or_unknown", "source_class", "evidence_refs",
        ],
        "outcome_feedback_requirements": [
            "intended_use_or_output", "useful_output_obtained_status", "defects_rejections_losses",
            "post_cycle_inspection", "deviations_unknowns", "recovery_or_adaptation_required",
        ],
        "external_dependency_categories": [
            "seeds-genetics", "nutrients-material-feedstocks", "water-input", "electricity-energy",
            "replacement-parts", "calibration-materials", "network-cloud-services",
            "external-specialist-labor-or-knowledge", "consumables-packaging",
            "legal-food-safety-or-other-external-services",
        ],
        "quantitative_metric_rule": {
            "denominator_required_when_share_or_yield_claimed": True,
            "coverage_window_required": True,
            "unknown_denominator_means_metric_unknown": True,
        },
        "maturation_binding": {
            "transition": "N1->N2",
            "requirements": [
                "at-least-one-real-productive-loop", "work-material-observations", "outcome-feedback"
            ],
            "evidence_complete_loop_automatically_establishes_transition": False,
            "transition_record_required": True,
        },
        "correction_policy": {"history_rewrite_allowed": False, "supersession_required": True},
        "supported_conformer_classes": [
            "hydroponic-food-production", "fabrication", "repair", "water-treatment", "energy-service",
            "other-explicit-profile",
        ],
        "claim_ceiling": {
            "physical_loop_executed": False,
            "useful_output_observed": False,
            "N2_established": False,
            "food_production_established": False,
            "food_safety_established": False,
            "commercial_viability_established": False,
            "economic_independence_established": False,
            "governance_standing_granted": False,
            "federation_membership_granted": False,
            "actuation_authority_granted": False,
        },
    }


def h2_fixture():
    return {
        "profile_id": H2_PROFILE,
        "profile_version": "1.0.0",
        "status": "synthetic-planned-showcase",
        "conforms_to": GENERIC_PROFILE,
        "parent_h1_subjects": dict(EXPECTED_H1_SUBJECTS),
        "whole_node_extrapolation_prohibited": True,
        "authority": "None",
        "synthetic_fixture": True,
        "physical_cycle_executed": False,
        "crop_profile": {
            "binding_status": "Unbound",
            "crop_or_cultivar_subject": None,
            "agronomic_profile": None,
            "reason": "Crop/cultivar must be researched and frozen separately before physical H2 execution",
        },
        "loop_identity": {
            "loop_id": "H2-hydroponic-loop-001",
            "generation": "planned-v1",
            "site_subject": "H1-node-A-site-placeholder",
            "productive_subject": "hydroponic-crop-lot-placeholder",
        },
        "useful_output_profile": {
            "output_class": "harvested-crop-output",
            "output_profile_binding_status": "UnboundUntilCropProfile",
            "edible_claim_allowed": False,
            "food_safety_claim_allowed": False,
            "marketability_claim_allowed": False,
        },
        "stages": [
            {"id": "H2a", "name": "crop-lot-input-work-preregistration", "planned_requirements": ["bind-crop-profile"]},
            {"id": "H2b", "name": "cultivation-process-observation-window", "planned_requirements": ["H1-process-observation-refs"]},
            {"id": "H2c", "name": "harvest-useful-output-observation", "planned_requirements": ["harvest-event"]},
            {"id": "H2d", "name": "losses-outcome-feedback-post-cycle-review", "planned_requirements": ["remaining-external-dependency-map"]},
        ],
        "h1_process_observation_refs": [
            "flow-rate", "reservoir-level", "reservoir-volume", "solution-temperature", "leak-state",
            "ph", "electrical-conductivity", "dissolved-oxygen", "air-temperature", "relative-humidity",
            "subsystem-power", "subsystem-energy",
        ],
        "h1_channel_identity_duplicated": False,
        "input_material_classes": ["seed-or-planting-material", "nutrient-or-feedstock", "water-input"],
        "work_activity_classes": ["setup", "maintenance", "harvest", "post-cycle-review"],
        "planned_output_observations": ["harvest-event", "gross-harvested-quantity"],
        "planned_outcome_feedback": ["useful-output-obtained-status", "defects-rejections-losses"],
        "external_dependency_categories_required": [
            "seeds-genetics", "nutrients-material-feedstocks", "water-input", "electricity-energy",
            "replacement-parts", "calibration-materials", "external-specialist-labor-or-knowledge",
            "legal-food-safety-or-other-external-services",
        ],
        "maturation_contribution": {
            "transition": "N1->N2",
            "can_contribute_if_physical_evidence_complete": [
                "at-least-one-real-productive-loop", "work-material-observations", "outcome-feedback"
            ],
            "automatically_establishes_N2": False,
            "transition_record_required": True,
        },
        "negative_semantic_guards": [
            "H1-process-telemetry-alone-does-not-establish-productive-loop",
            "planned-input-does-not-equal-consumed-material",
            "work-estimate-does-not-equal-observed-work",
            "work-does-not-issue-credits-or-standing",
            "harvested-output-does-not-imply-edible-or-food-safe",
            "pH-EC-nominal-does-not-imply-useful-output",
            "one-cycle-does-not-imply-node-food-independence",
            "external-inputs-must-remain-visible",
            "unknown-denominator-does-not-permit-yield-share",
            "Symthaea-prediction-does-not-become-output-observation",
            "correction-does-not-rewrite-history",
            "ProductiveLoop-does-not-grant-authority",
            "H2-references-H1-observations-without-duplicating-identity",
            "synthetic-fixture-does-not-become-physical-evidence",
            "completed-loop-does-not-auto-establish-N2",
        ],
        "claim_ceiling": {
            "crop_profile_bound": False,
            "physical_cycle_executed": False,
            "productive_loop_established": False,
            "useful_output_observed": False,
            "crop_performance_established": False,
            "food_safety_established": False,
            "food_independence_established": False,
            "N2_established": False,
            "commercial_viability_established": False,
            "governance_standing_granted": False,
            "federation_membership_granted": False,
            "actuation_authority_granted": False,
        },
    }


class ValidatorTests(unittest.TestCase):
    def assert_rejected(self, mutate):
        bench, generic, h2 = bench_fixture(), generic_fixture(), h2_fixture()
        mutate(bench, generic, h2)
        self.assertTrue(validate(bench, generic, h2))

    def test_pristine(self):
        self.assertEqual(validate(bench_fixture(), generic_fixture(), h2_fixture()), [])

    def test_generic_authority(self):
        self.assert_rejected(lambda b, g, h: g.__setitem__("authority", "Governance"))

    def test_generic_extrapolation(self):
        self.assert_rejected(lambda b, g, h: g.__setitem__("whole_node_extrapolation_prohibited", False))

    def test_work_issues_credit(self):
        self.assert_rejected(lambda b, g, h: g["work_nonclaims"].__setitem__("itc_credit_issued", True))

    def test_process_duplicated(self):
        self.assert_rejected(lambda b, g, h: g["process_observation_contract"].__setitem__("referenced_not_duplicated", False))

    def test_analysis_replaces_observation(self):
        self.assert_rejected(lambda b, g, h: g["process_observation_contract"].__setitem__("analysis_can_replace_source_observation", True))

    def test_output_food_safe(self):
        self.assert_rejected(lambda b, g, h: g["output_nonclaims"].__setitem__("food_safe", True))

    def test_unknown_denominator_metric(self):
        self.assert_rejected(lambda b, g, h: g["quantitative_metric_rule"].__setitem__("unknown_denominator_means_metric_unknown", False))

    def test_generic_auto_n2(self):
        self.assert_rejected(lambda b, g, h: g["maturation_binding"].__setitem__("evidence_complete_loop_automatically_establishes_transition", True))

    def test_history_rewrite(self):
        self.assert_rejected(lambda b, g, h: g["correction_policy"].__setitem__("history_rewrite_allowed", True))

    def test_conformer_narrowing(self):
        self.assert_rejected(lambda b, g, h: g.__setitem__("supported_conformer_classes", ["hydroponic-food-production"]))

    def test_generic_claim_upgrade(self):
        self.assert_rejected(lambda b, g, h: g["claim_ceiling"].__setitem__("N2_established", True))

    def test_h2_physical_executed(self):
        self.assert_rejected(lambda b, g, h: h.__setitem__("physical_cycle_executed", True))

    def test_h2_crop_silently_bound(self):
        def mutate(b, g, h):
            h["crop_profile"]["binding_status"] = "Bound"
            h["crop_profile"]["crop_or_cultivar_subject"] = "lettuce"
        self.assert_rejected(mutate)

    def test_h2_duplicates_channel_identity(self):
        self.assert_rejected(lambda b, g, h: h.__setitem__("h1_channel_identity_duplicated", True))

    def test_h2_unknown_channel(self):
        self.assert_rejected(lambda b, g, h: h["h1_process_observation_refs"].append("invented-channel"))

    def test_h2_stage_removed(self):
        self.assert_rejected(lambda b, g, h: h["stages"].pop())

    def test_h2_dependency_hidden(self):
        self.assert_rejected(lambda b, g, h: h["external_dependency_categories_required"].remove("electricity-energy"))

    def test_h2_output_food_safe(self):
        self.assert_rejected(lambda b, g, h: h["useful_output_profile"].__setitem__("food_safety_claim_allowed", True))

    def test_h2_auto_n2(self):
        self.assert_rejected(lambda b, g, h: h["maturation_contribution"].__setitem__("automatically_establishes_N2", True))

    def test_h2_claim_upgrade(self):
        self.assert_rejected(lambda b, g, h: h["claim_ceiling"].__setitem__("productive_loop_established", True))

    def test_symthaea_guard_removed(self):
        self.assert_rejected(lambda b, g, h: h["negative_semantic_guards"].remove("Symthaea-prediction-does-not-become-output-observation"))


if __name__ == "__main__":
    unittest.main()
