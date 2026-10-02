#!/usr/bin/env python3
"""Qualify SYM-CIVIC-002 causal falsification benchmark v1."""
from __future__ import annotations
import json, pathlib, sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
MANIFEST=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_002_benchmark.json"
PARENT="424e9342fcea47f982d3c5b9f43ad9a4cce4f1a0"
IDS=[f"CF-{i:02d}" for i in range(1,11)]
FAMILIES=[
 "adversity_changes_option_stable","option_expands_no_outcome_improvement","connection_environment_harmful",
 "nominally_available_unreachable","shared_hidden_dependency","lethal_permeability_changes_mortality",
 "same_current_different_history","prediction_contradicted_by_observation","reporting_change_mimics_outcome",
 "alternative_specification_erases_effect"
]
DISPOSITIONS={"Supported","Contradicted","Inconclusive","Underpowered","Confounded","Invalidated"}
FORBIDDEN={"individual_risk_label","individual_danger_label","combined_violence_score","combined_resilience_score","combined_safety_score","authority_action","diagnosis"}
def fail(m): raise SystemExit("SYM-CIVIC-002 FAIL: "+m)
def main():
    d=json.loads(MANIFEST.read_text(encoding="utf-8"))
    if d.get("schema")!="mycelix.sym-civic.causal-falsification-benchmark.v1": fail("schema drift")
    if d.get("program")!="SYM-CIVIC-002" or d.get("parent_subject")!=PARENT: fail("program/parent drift")
    if d.get("analysis_role")!="research_only": fail("analysis role drift")
    uq=d.get("upstream_qualification",{})
    if uq.get("program")!="CIV-RES-004": fail("upstream program drift")
    if uq.get("qualifier_commit")!="424e9342fcea47f982d3c5b9f43ad9a4cce4f1a0": fail("upstream qualifier drift")
    if uq.get("workflow_run_id")!=37040844685: fail("upstream run drift")
    if uq.get("receipt_sha256")!="3e35ec4c9049ccfcabe164548b379ec45404509ae58162e1f765b886ca0bdce7": fail("upstream receipt drift")
    if uq.get("fixture_sha256")!="68c3266ee77f59afd5fb9e0d24f0caa13c19b56943102703fb783148239b09db": fail("upstream fixture drift")
    if uq.get("oracle_sha256")!="e5fb8b63bd2ca0cc5a2a55649fa9fe9ce31590dc0818bf5bc13e14a1cfcb8722": fail("upstream oracle drift")
    if uq.get("case_count")!=18 or uq.get("candidate_input_policy")!="oracle_excluded": fail("upstream corpus policy drift")
    threats={"positivity_overlap","time_varying_confounding_feedback","network_interference_spillover","measurement_error_misclassification"}
    if set(d.get("required_causal_threat_checks",[]))!=threats: fail("causal threat-check drift")
    sb=d.get("input_contract",{}).get("study_manifest",[])
    required={"study_id","input_snapshot","population_or_cohort","geographic_scope","temporal_scope","estimand","model_identity","execution_identity","seed_or_randomness","uncertainty","missingness_policy","identification_assumptions","diagnostics","sensitivity_analyses","spillover_displacement_checks","alternative_explanations","output_disposition"}
    if set(sb)!=required: fail("study binding drift")
    if set(d.get("input_contract",{}).get("output_dispositions",[]))!=DISPOSITIONS: fail("disposition vocabulary drift")
    if set(d.get("prohibited_surfaces",[]))!=FORBIDDEN: fail("prohibited surface drift")
    if d.get("counterexample_cases")!=IDS: fail("counterexample count/order drift")
    if [x["family"] for x in d.get("cases",[])]!=FAMILIES: fail("family coverage drift")
    cc=d.get("case_contract",{})
    if cc.get("required_fields")!=["id","family","seed","design","observation","threat_checks","identification_assumptions","notes"]: fail("case contract drift")
    if cc.get("threat_check_fields")!=["positivity_overlap","time_varying_confounding_feedback","network_interference_spillover","measurement_error_misclassification"]: fail("case threat contract drift")
    if cc.get("disposition_field_forbidden") is not True or cc.get("oracle_policy")!="case fixtures contain conditions and observations only; scientific output disposition is derived by the analysis layer": fail("case oracle policy drift")
    cases=d["cases"]
    for c in cases:
        if c["id"] not in IDS or not isinstance(c["seed"],int): fail(c["id"]+" identity/seed")
        if c["id"]=="CF-01" and not (c["design"]["community_a"]["option_space"]=="stable" and c["design"]["community_b"]["option_space"]=="stable"): fail("CF-01 option stability")
        if c["id"]=="CF-02" and c["observation"]["outcome_improvement"]!="none": fail("CF-02 outcome")
        if c["id"]=="CF-03" and c["observation"]["harmful_context"]!="present": fail("CF-03 context")
        if c["id"]=="CF-04" and c["observation"]["practical_reachability"]!="none": fail("CF-04 reachability")
        if c["id"]=="CF-05" and c["observation"]["independent_redundancy"]!=1: fail("CF-05 dependency")
        if c["id"]=="CF-06" and c["observation"]["upstream_crisis_change"]!="none": fail("CF-06 upstream")
        if c["id"]=="CF-07" and c["observation"]["current_state"]!="equal": fail("CF-07 state")
        if c["id"]=="CF-08" and c["observation"]["prediction_result"]!="contradicted_by_later_observation": fail("CF-08 prediction")
        if c["id"]=="CF-09" and c["observation"]["measured_true_rate"]!="stable": fail("CF-09 reporting")
        if c["id"]=="CF-10" and c["observation"]["effect_under_alternative"]!="absent": fail("CF-10 alternative")
        if set(c.get("threat_checks",{}))!=set(["positivity_overlap","time_varying_confounding_feedback","network_interference_spillover","measurement_error_misclassification"]): fail(c["id"]+" threat coverage")
        if any(k in c.get("observation",{}) for k in ("disposition","expected_disposition","oracle_verdict")): fail(c["id"]+" embedded oracle")
    if any("person" in x.lower() for x in json.dumps(d).split()):
        fail("unexpected person-scoped surface")
    print("SYM-CIVIC-002 PASS: 10 counterexample families, complete study binding, explicit scientific dispositions, research-only/non-authorizing boundary")
    return 0
if __name__=="__main__": sys.exit(main())
