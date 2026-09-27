import unittest
from validate_myc_int_018z import validate, EXPECTED_PARENT


def fixture():
    return {
        "profile_id":"myc-int-018x-repair-productive-loop-v1","profile_version":"1.0.0","status":"synthetic-planned",
        "parent_productive_loop_subject":EXPECTED_PARENT,"generic_conformer":"ProductiveLoopV1","conformer_class":"repair-restoration",
        "whole_node_extrapolation_prohibited":True,"authority":"None",
        "existing_owners":{"work_orders":"mycelix-manufacturing/zomes/workorders"},
        "required_evidence_sections":["repair-demand","asset-capability-subject","pre-repair-observations","diagnosis-hypotheses","work-order-reference","planned-parts-materials","observed-consumed-replaced-parts","work-events","interventions","post-repair-verification-profile","restored-capability-evidence","residual-defects-limitations","recurrence-window","loss-failure-evidence","outcome-feedback","external-dependency-map","correction-supersession-lineage","unknowns-conflicts"],
        "semantic_separations":["repair-demand != diagnosis","work-order-created != work-performed","planned-part != consumed-or-replaced-part","work-order-completed != capability-restored","immediate-functional-test-pass != recurrence-free-repair","work-performed != compensation-or-itc-or-standing","diagnosis-hypothesis != observed-cause","synthetic-fault != physical-repair-event"],
        "first_synthetic_reference_case":{
            "physical_execution_claimed":False,
            "pre_repair_evidence_refs":["H1/FI-08:pump-command-without-flow","H1/FI-09:partial-flow-restriction"],
            "pre_repair_state":{"cause":"UnknownUntilEvidence"},
            "work_order":{"owner":"mycelix-manufacturing/zomes/workorders","status_is_not_restoration_proof":True},
            "post_repair_verification_profile":{"required":["exact-commanded-state","electrical-state-if-observed","measured-flow-under-exact-test-profile","leak-containment-check","manual-stop-verified","residual-limitations-recorded"],"single_nominal_sample_sufficient":False},
            "recurrence_window":{"required":True,"duration_profile":"Unbound","no_recurrence_claim_without_window":True},
            "outcome":{"restored_capability_claimed":False,"recurrence_free_claimed":False,"n2_established":False}},
        "productiveloop_mapping":{
            "inputs_materials":{"planned_parts_are_not_consumed_parts":True,"replacement_part_identity_provenance_required":True},
            "work":{"auto_compensation":False,"auto_itc_credit":False,"auto_governance_standing":False},
            "process":{"diagnosis_may_not_replace_observation":True},
            "useful_output":{"type":"RestoredCapabilityUnderExactVerificationProfile","work_order_status_is_not_output":True},
            "loss_failure":{"failed_attempts_retained":True,"damaged_or_rejected_parts_retained":True,"unresolved_defects_retained":True,"downtime_retained":True,"recurrence_retained":True}},
        "n1_to_n2":{"can_contribute_at_least_one_real_productive_loop_when_physical_and_evidence_complete":True,"can_contribute_work_material_observations_when_physical_and_evidence_complete":True,"can_contribute_outcome_feedback_when_physical_and_evidence_complete":True,"automatically_establishes_n2":False,"maturation_transition_record_required":True},
        "claim_ceiling":{"physical_repair_executed":False,"worker_qualified":False,"restored_capability_established":False,"recurrence_free_established":False,"compensation_entitlement_established":False,"safety_qualified":False,"n2_established":False,"whole_node_resilience_established":False}
    }


class Tests(unittest.TestCase):
    def reject(self, mutate):
        x=fixture(); mutate(x); self.assertTrue(validate(x))
    def test_00_pristine(self): self.assertEqual(validate(fixture()),[])
    def test_01_physical_promoted(self): self.reject(lambda x:x["first_synthetic_reference_case"].__setitem__("physical_execution_claimed",True))
    def test_02_workorder_as_proof(self): self.reject(lambda x:x["first_synthetic_reference_case"]["work_order"].__setitem__("status_is_not_restoration_proof",False))
    def test_03_planned_part_as_consumed(self): self.reject(lambda x:x["productiveloop_mapping"]["inputs_materials"].__setitem__("planned_parts_are_not_consumed_parts",False))
    def test_04_diagnosis_as_cause(self): self.reject(lambda x:x["first_synthetic_reference_case"]["pre_repair_state"].__setitem__("cause","BlockedFilter"))
    def test_05_synthetic_as_physical(self): self.reject(lambda x:x["semantic_separations"].remove("synthetic-fault != physical-repair-event"))
    def test_06_flow_inferred(self): self.reject(lambda x:x["first_synthetic_reference_case"]["post_repair_verification_profile"]["required"].remove("measured-flow-under-exact-test-profile"))
    def test_07_single_sample_sufficient(self): self.reject(lambda x:x["first_synthetic_reference_case"]["post_repair_verification_profile"].__setitem__("single_nominal_sample_sufficient",True))
    def test_08_recurrence_removed(self): self.reject(lambda x:x["first_synthetic_reference_case"]["recurrence_window"].__setitem__("required",False))
    def test_09_failed_attempt_hidden(self): self.reject(lambda x:x["productiveloop_mapping"]["loss_failure"].__setitem__("failed_attempts_retained",False))
    def test_10_part_provenance_removed(self): self.reject(lambda x:x["productiveloop_mapping"]["inputs_materials"].__setitem__("replacement_part_identity_provenance_required",False))
    def test_11_auto_compensation(self): self.reject(lambda x:x["productiveloop_mapping"]["work"].__setitem__("auto_compensation",True))
    def test_12_auto_itc(self): self.reject(lambda x:x["productiveloop_mapping"]["work"].__setitem__("auto_itc_credit",True))
    def test_13_auto_standing(self): self.reject(lambda x:x["productiveloop_mapping"]["work"].__setitem__("auto_governance_standing",True))
    def test_14_authority(self): self.reject(lambda x:x.__setitem__("authority","Governance"))
    def test_15_whole_node(self): self.reject(lambda x:x.__setitem__("whole_node_extrapolation_prohibited",False))
    def test_16_auto_n2(self): self.reject(lambda x:x["n1_to_n2"].__setitem__("automatically_establishes_n2",True))
    def test_17_completed_equals_restored(self): self.reject(lambda x:x["semantic_separations"].remove("work-order-completed != capability-restored"))
    def test_18_hide_limitation(self): self.reject(lambda x:x["productiveloop_mapping"]["loss_failure"].__setitem__("unresolved_defects_retained",False))
    def test_19_output_as_status(self): self.reject(lambda x:x["productiveloop_mapping"]["useful_output"].__setitem__("type","WorkOrderCompleted"))
    def test_20_claim_upgrade(self): self.reject(lambda x:x["claim_ceiling"].__setitem__("restored_capability_established",True))


if __name__=="__main__":
    unittest.main()
