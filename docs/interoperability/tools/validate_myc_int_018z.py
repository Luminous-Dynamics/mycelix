#!/usr/bin/env python3
import argparse, json
from pathlib import Path

EXPECTED_PARENT="7a6b4e8f26661969f686ce54afb5ad3da10ba754"
REQUIRED_SECTIONS={"repair-demand","asset-capability-subject","pre-repair-observations","diagnosis-hypotheses","work-order-reference","planned-parts-materials","observed-consumed-replaced-parts","work-events","interventions","post-repair-verification-profile","restored-capability-evidence","residual-defects-limitations","recurrence-window","loss-failure-evidence","outcome-feedback","external-dependency-map","correction-supersession-lineage","unknowns-conflicts"}
REQUIRED_SEPARATIONS={"repair-demand != diagnosis","work-order-created != work-performed","planned-part != consumed-or-replaced-part","work-order-completed != capability-restored","immediate-functional-test-pass != recurrence-free-repair","work-performed != compensation-or-itc-or-standing","diagnosis-hypothesis != observed-cause","synthetic-fault != physical-repair-event"}
REQUIRED_VERIFY={"exact-commanded-state","electrical-state-if-observed","measured-flow-under-exact-test-profile","leak-containment-check","manual-stop-verified","residual-limitations-recorded"}


def load_json(path):
    return json.loads(Path(path).read_text())


def all_false(d):
    return isinstance(d,dict) and all(v is False for v in d.values())


def validate(x):
    e=[]
    if (x.get("profile_id"),x.get("profile_version"),x.get("status"),x.get("parent_productive_loop_subject")) != ("myc-int-018x-repair-productive-loop-v1","1.0.0","synthetic-planned",EXPECTED_PARENT):
        e.append("identity drift")
    if x.get("generic_conformer")!="ProductiveLoopV1" or x.get("conformer_class")!="repair-restoration":
        e.append("conformer drift")
    if x.get("whole_node_extrapolation_prohibited") is not True or x.get("authority")!="None":
        e.append("scope/authority drift")
    owners=x.get("existing_owners",{})
    if owners.get("work_orders")!="mycelix-manufacturing/zomes/workorders":
        e.append("work-order owner drift")
    if set(x.get("required_evidence_sections",[])) != REQUIRED_SECTIONS:
        e.append("evidence section drift")
    if not REQUIRED_SEPARATIONS.issubset(set(x.get("semantic_separations",[]))):
        e.append("semantic separation drift")
    case=x.get("first_synthetic_reference_case",{})
    if case.get("physical_execution_claimed") is not False:
        e.append("synthetic case promoted physical")
    if not {"H1/FI-08:pump-command-without-flow","H1/FI-09:partial-flow-restriction"}.issubset(set(case.get("pre_repair_evidence_refs",[]))):
        e.append("H1 fault refs drift")
    if case.get("pre_repair_state",{}).get("cause")!="UnknownUntilEvidence":
        e.append("cause invented")
    wo=case.get("work_order",{})
    if wo.get("owner")!="mycelix-manufacturing/zomes/workorders" or wo.get("status_is_not_restoration_proof") is not True:
        e.append("work-order boundary drift")
    verify=case.get("post_repair_verification_profile",{})
    if not REQUIRED_VERIFY.issubset(set(verify.get("required",[]))) or verify.get("single_nominal_sample_sufficient") is not False:
        e.append("post-repair verification drift")
    rec=case.get("recurrence_window",{})
    if rec.get("required") is not True or rec.get("duration_profile")!="Unbound" or rec.get("no_recurrence_claim_without_window") is not True:
        e.append("recurrence boundary drift")
    out=case.get("outcome",{})
    for k in ["restored_capability_claimed","recurrence_free_claimed","n2_established"]:
        if out.get(k) is not False:
            e.append("outcome claim upgraded:"+k)
    m=x.get("productiveloop_mapping",{})
    inp=m.get("inputs_materials",{})
    if inp.get("planned_parts_are_not_consumed_parts") is not True or inp.get("replacement_part_identity_provenance_required") is not True:
        e.append("parts boundary drift")
    work=m.get("work",{})
    for k in ["auto_compensation","auto_itc_credit","auto_governance_standing"]:
        if work.get(k) is not False:
            e.append("work authority/economic promotion:"+k)
    if m.get("process",{}).get("diagnosis_may_not_replace_observation") is not True:
        e.append("diagnosis promotion")
    useful=m.get("useful_output",{})
    if useful.get("type")!="RestoredCapabilityUnderExactVerificationProfile" or useful.get("work_order_status_is_not_output") is not True:
        e.append("useful-output drift")
    lf=m.get("loss_failure",{})
    for k in ["failed_attempts_retained","damaged_or_rejected_parts_retained","unresolved_defects_retained","downtime_retained","recurrence_retained"]:
        if lf.get(k) is not True:
            e.append("loss/failure retention drift:"+k)
    n=x.get("n1_to_n2",{})
    for k in ["can_contribute_at_least_one_real_productive_loop_when_physical_and_evidence_complete","can_contribute_work_material_observations_when_physical_and_evidence_complete","can_contribute_outcome_feedback_when_physical_and_evidence_complete","maturation_transition_record_required"]:
        if n.get(k) is not True:
            e.append("N1-N2 conditional contribution drift:"+k)
    if n.get("automatically_establishes_n2") is not False:
        e.append("N2 auto establishment")
    if not all_false(x.get("claim_ceiling",{})):
        e.append("claim ceiling upgraded")
    return e


def main():
    p=argparse.ArgumentParser(); p.add_argument("fixture"); a=p.parse_args()
    errs=validate(load_json(a.fixture))
    if errs:
        for z in errs: print("ERROR:",z)
        raise SystemExit(1)
    print("MYC-INT-018Z: PASS")


if __name__=="__main__":
    main()
