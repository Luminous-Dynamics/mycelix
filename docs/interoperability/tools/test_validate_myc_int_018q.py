import unittest

from validate_myc_int_018q import EXPECTED_SUBJECTS, validate


def bench_fixture():
    channels = [
        "pump-command-state", "pump-electrical-state", "subsystem-power", "subsystem-energy",
        "controller-power-state", "manual-override-state", "flow-rate", "reservoir-level",
        "reservoir-volume", "solution-temperature", "leak-state", "ph", "electrical-conductivity",
        "dissolved-oxygen", "air-temperature", "relative-humidity", "light", "co2",
    ]
    return {
        "stages": [
            {"id": "H1a", "name": "dry-instrumentation-power", "water_present": False, "nutrient_solution_present": False},
            {"id": "H1b", "name": "clean-water-closed-loop", "water_present": True, "nutrient_solution_present": False},
            {"id": "H1c", "name": "nutrient-solution-sensing", "water_present": True, "nutrient_solution_present": True},
        ],
        "channels": [{"id": channel, "actuation_authority": False} for channel in channels],
        "failure_injections": [{"id": f"FI-{i:02d}"} for i in range(1, 19)],
        "runtime_optionalities": {
            "mycelix_required_for_local_acquisition": False,
            "holochain_required_for_local_acquisition": False,
            "symthaea_required_for_local_acquisition": False,
            "itc_required_for_local_acquisition": False,
            "fleet_required_for_local_acquisition": False,
        },
    }


def package_fixture():
    return {
        "claim_ceiling": {
            "wiring_approved": False,
            "energization_approved": False,
            "electrical_safety_certified": False,
            "sensor_calibration_established": False,
            "agronomic_qualified": False,
            "autonomous_control_authorized": False,
        },
        "stage_gates": {"H1c": {"required_checks": ["automatic-dosing-disabled"]}},
        "stop_abort_conditions": ["manual-stop-unavailable"],
        "run_evidence_requirements": [
            "fault-action-log", "manual-interventions", "stop-abort-events", "post-run-inspection-result"
        ],
    }


def _binding(transition, requirement, relation, scope="H1 subsystem"):
    return {
        "transition": transition,
        "requirement": requirement,
        "relation": relation,
        "claim_scope": scope,
        "h1_sources": ["fixture"],
        "candidate_source_classes": ["DirectObservation"],
        "currentness_window_required": True,
        "completeness_ceiling": "Exact H1 scope only",
        "known_gaps": ["whole-node evidence outside H1"],
        "authority": "None",
        "whole_node_extrapolation_prohibited": True,
    }


def binding_fixture():
    return {
        "profile_id": "myc-int-018q-h1-maturation-binding-v1",
        "profile_version": "1.0.0",
        "status": "design-binding-fixture",
        "source_subjects": {key: {"exact_subject": value} for key, value in EXPECTED_SUBJECTS.items()},
        "relation_vocabulary": [
            "CanSatisfyUnderExactProfile", "ContributesButInsufficient", "CannotSatisfy", "NotApplicable"
        ],
        "source_class_vocabulary": [
            "DirectObservation", "SourceOwnedOperationalFact", "ReconstructedState", "DerivedAssessment",
            "OperatorDeclaration", "ImportedForeignEvidence", "Unknown",
        ],
        "global_invariants": {
            "whole_node_extrapolation_prohibited": True,
            "mapping_is_current_evidence": False,
            "mapping_establishes_transition": False,
            "mapping_grants_authority": False,
            "mapping_grants_federation_membership": False,
            "mapping_grants_governance_standing": False,
            "symthaea_analysis_is_direct_observation": False,
        },
        "transition_bindings": [
            _binding("N0->N1", "local-acquisition-or-evidence-path", "CanSatisfyUnderExactProfile"),
            _binding("N0->N1", "dependency-map", "ContributesButInsufficient"),
            _binding("N1->N2", "at-least-one-real-productive-loop", "CannotSatisfy"),
            _binding("N1->N2", "work-material-observations", "CannotSatisfy"),
            _binding("N1->N2", "outcome-feedback", "CannotSatisfy"),
            _binding("N2->N3", "bounded-outage-continuity-evidence", "ContributesButInsufficient"),
            _binding("N2->N3", "failure-recovery-evidence", "ContributesButInsufficient"),
            _binding("N2->N3", "local-safe-stop", "CanSatisfyUnderExactProfile", "H1 physical/process subsystem only"),
        ],
        "domain_scope_bindings": [
            {"dimension": "water", "relation": "ContributesButInsufficient", "h1_evidence": ["flow-rate", "reservoir-level", "reservoir-volume", "solution-temperature", "leak-state"], "prohibited_inference": "H1 recirculation != node water independence", "node_level_gap": "node water denominator"},
            {"dimension": "energy", "relation": "ContributesButInsufficient", "h1_evidence": ["subsystem-power", "subsystem-energy"], "prohibited_inference": "H1 energy != node energy independence", "node_level_gap": "node energy denominator"},
            {"dimension": "food", "relation": "CannotSatisfy", "h1_evidence": [], "prohibited_inference": "nutrient sensing != food production", "node_level_gap": "productive crop/output profile"},
            {"dimension": "repair-fabrication", "relation": "ContributesButInsufficient", "h1_evidence": ["fault-action-log", "manual-interventions", "restart-power-cycle"], "prohibited_inference": "restart != repair capability", "node_level_gap": "repair loop"},
            {"dimension": "compute-network", "relation": "ContributesButInsufficient", "h1_evidence": ["network-partition", "local-acquisition-can-continue"], "prohibited_inference": "H1 continuity != node continuity", "node_level_gap": "node service continuity"},
        ],
        "productive_loop_gap": {
            "current_h1_profile_must_not_be_widened_silently": True,
            "follow_on_profile_required": True,
            "candidate_profile_name": "ProductiveLoopV1",
            "required_semantics": [
                "useful-output-subject", "production-window", "material-input-observations", "work-observations",
                "process-observations", "useful-output-observations", "loss-waste-failure-observations",
                "outcome-feedback", "external-dependency-import-map", "correction-supersession-lineage",
                "whole-node-extrapolation-prohibited",
            ],
            "hydroponics_may_be_first_conformer": True,
            "food_is_only_valid_productive_loop": False,
        },
        "claim_ceiling": {
            "N1_established": False, "N2_established": False, "N3_established": False,
            "food_production_established": False, "whole_node_resilience_established": False,
            "water_independence_established": False, "energy_independence_established": False,
            "economic_independence_established": False, "governance_standing_granted": False,
            "federation_membership_granted": False,
        },
    }


class ValidatorTests(unittest.TestCase):
    def assert_rejected(self, mutate):
        bench, package, binding = bench_fixture(), package_fixture(), binding_fixture()
        mutate(bench, package, binding)
        self.assertTrue(validate(bench, package, binding))

    def test_pristine(self):
        self.assertEqual(validate(bench_fixture(), package_fixture(), binding_fixture()), [])

    def test_productive_loop_promoted(self):
        self.assert_rejected(lambda b, p, q: q["transition_bindings"][2].__setitem__("relation", "CanSatisfyUnderExactProfile"))

    def test_ph_ec_food_claim(self):
        self.assert_rejected(lambda b, p, q: q["domain_scope_bindings"][2].__setitem__("relation", "CanSatisfyUnderExactProfile"))

    def test_dependency_map_promoted(self):
        self.assert_rejected(lambda b, p, q: q["transition_bindings"][1].__setitem__("relation", "CanSatisfyUnderExactProfile"))

    def test_outage_promoted(self):
        self.assert_rejected(lambda b, p, q: q["transition_bindings"][5].__setitem__("relation", "CanSatisfyUnderExactProfile"))

    def test_safe_stop_scope_lost(self):
        self.assert_rejected(lambda b, p, q: q["transition_bindings"][7].__setitem__("whole_node_extrapolation_prohibited", False))

    def test_water_independence(self):
        self.assert_rejected(lambda b, p, q: q["domain_scope_bindings"][0].__setitem__("relation", "CanSatisfyUnderExactProfile"))

    def test_energy_independence(self):
        self.assert_rejected(lambda b, p, q: q["domain_scope_bindings"][1].__setitem__("relation", "CanSatisfyUnderExactProfile"))

    def test_food_mapping_widened(self):
        self.assert_rejected(lambda b, p, q: q["domain_scope_bindings"][2].__setitem__("relation", "ContributesButInsufficient"))

    def test_extrapolation_disabled(self):
        self.assert_rejected(lambda b, p, q: q["global_invariants"].__setitem__("whole_node_extrapolation_prohibited", False))

    def test_authority_granted(self):
        self.assert_rejected(lambda b, p, q: q["transition_bindings"][0].__setitem__("authority", "Governance"))

    def test_claim_upgrade(self):
        self.assert_rejected(lambda b, p, q: q["claim_ceiling"].__setitem__("N2_established", True))

    def test_channel_authority(self):
        self.assert_rejected(lambda b, p, q: b["channels"][0].__setitem__("actuation_authority", True))

    def test_fi13_removed(self):
        self.assert_rejected(lambda b, p, q: b.__setitem__("failure_injections", [x for x in b["failure_injections"] if x["id"] != "FI-13"]))

    def test_network_required_for_acquisition(self):
        self.assert_rejected(lambda b, p, q: b["runtime_optionalities"].__setitem__("mycelix_required_for_local_acquisition", True))

    def test_auto_dosing_enabled(self):
        self.assert_rejected(lambda b, p, q: p["stage_gates"]["H1c"].__setitem__("required_checks", []))

    def test_manual_stop_evidence_removed(self):
        self.assert_rejected(lambda b, p, q: p.__setitem__("stop_abort_conditions", []))

    def test_productive_followon_disabled(self):
        self.assert_rejected(lambda b, p, q: q["productive_loop_gap"].__setitem__("follow_on_profile_required", False))

    def test_productive_food_only(self):
        self.assert_rejected(lambda b, p, q: q["productive_loop_gap"].__setitem__("food_is_only_valid_productive_loop", True))

    def test_productive_semantic_removed(self):
        self.assert_rejected(lambda b, p, q: q["productive_loop_gap"]["required_semantics"].remove("outcome-feedback"))

    def test_symthaea_promoted(self):
        self.assert_rejected(lambda b, p, q: q["global_invariants"].__setitem__("symthaea_analysis_is_direct_observation", True))


if __name__ == "__main__":
    unittest.main()
