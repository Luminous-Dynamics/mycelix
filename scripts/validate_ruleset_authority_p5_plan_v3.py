#!/usr/bin/env python3
"""Validate the preregistered CI-GOV-001J P5 execution matrix v3."""
from __future__ import annotations
import argparse, copy, json
from pathlib import Path
from typing import Any

VALIDATOR_ID="ci-gov-001j-p5-plan-validator-v3"
PLAN_ID="ci-gov-001j-p5-execution-plan-v3"
SOURCE_SHA="a248fe016e2f7fbe5a632595265887bffdf22df9"
QUAL_SHA="b1031615e22d93b4f64d52dd93272c7e468a2410"
PROMOTION_HEAD="63aa8b73bc0d8825eb3f8b4eb747fb8c0af8d12f"
CONFIG_HEAD="db82151323e5aba22a78bd32a5401e2e5922b217"
REQUIRED=["format","test-commons","test-civic","test-hearth","test-finance","test-governance","test-identity","test-personal","test-attribution","test-bridge","test-sdk","test-prism"]
INFO=["test-finance-integration"]
EXPECTED_NAMES=["KnownRelevantClosure","ProvenIrrelevantTypedNoCiRequired","UnknownPathFullRequiredFanout","RenamePreservesPreviousRelevantPath","InformationalOnlyKnownRelevantClosure","InformationalFinanceObservationNonGating","RequiredProductRedBlocks","StaleSubjectCannotQualifyNewerHead","AuthorityInputUncertaintyDeniesPass"]

class PlanError(ValueError): pass

def require(ok: bool,msg: str)->None:
    if not ok: raise PlanError(msg)

def by_name(p: dict[str,Any])->dict[str,dict[str,Any]]:
    cases=p.get("runtime_cases")
    require(isinstance(cases,list) and len(cases)==9,"runtime case census drift")
    out={}
    for case in cases:
        require(isinstance(case,dict),"runtime case must be object")
        name=case.get("name"); require(isinstance(name,str) and name,"case name missing")
        require(name not in out,f"duplicate case {name}"); out[name]=case
    require(list(out)==EXPECTED_NAMES,"runtime case order/name drift")
    require([c.get("id") for c in cases]==[f"P5-0{i}" for i in range(1,10)],"runtime case id drift")
    return out

def expect_jobs(case: dict[str,Any], required: Any, info: Any)->None:
    e=case.get("expect"); require(isinstance(e,dict),"case expectation missing")
    require(e.get("required_selected")==required,"required selection drift")
    require(e.get("informational_selected")==info,"informational selection drift")

def validate(p: Any)->dict[str,Any]:
    require(isinstance(p,dict),"plan root must be object")
    require(p.get("plan_id")==PLAN_ID,"plan identity drift")
    require(p.get("version")==3,"version drift")
    require(p.get("issue")==1705,"issue drift")
    require(p.get("promotion_refinement_head")==PROMOTION_HEAD,"promotion refinement drift")
    require(p.get("authority_source_sha")==SOURCE_SHA,"authority source drift")
    require(p.get("qualification_head_sha")==QUAL_SHA,"qualification head drift")
    require(p.get("configuration_head")==CONFIG_HEAD,"configuration head drift")
    require(p.get("required_jobs")==REQUIRED,"required job census/order drift")
    require(p.get("informational_jobs")==INFO,"informational job census drift")
    epoch=p.get("epoch_requirements")
    require(isinstance(epoch,list) and set(epoch)=={
      "same_authority_source_sha","same_workflow_sha","same_ruleset_id",
      "same_p4_structural_commitment","same_p4_configuration_commitment",
      "same_qualification_receipt_sha256","raw_run_and_job_ids_retained",
      "target_sha_bound_per_case","no_cross_epoch_evidence_mixing"},"epoch requirements drift")
    policy=p.get("probe_policy"); require(isinstance(policy,dict),"probe policy missing")
    require(policy.get("base_ref")=="refs/heads/main","probe base drift")
    for key in ("must_remain_unmerged","probe_only_changes","cleanup_after_evidence","no_probe_may_modify_authority_source_or_ruleset_configuration"):
        require(policy.get(key) is True,f"probe policy invariant missing {key}")
    c=by_name(p)

    e=c["KnownRelevantClosure"]["expect"]
    require(e.get("router_disposition")=="AdmitKnownRelevant" and e.get("authority_complete") is True,"known relevant disposition drift")
    expect_jobs(c["KnownRelevantClosure"],["format","test-finance"],INFO)
    require(e.get("merge_authority_passed") is True,"known relevant must pass when selected required jobs pass")

    e=c["ProvenIrrelevantTypedNoCiRequired"]["expect"]
    require(e.get("router_disposition")=="GenericCiNotRequired" and e.get("aggregate_disposition")=="TrustedGenericCiNotRequired","irrelevant typed result drift")
    expect_jobs(c["ProvenIrrelevantTypedNoCiRequired"],[],[])
    require(e.get("all_product_jobs_skipped") is True,"irrelevant case must prove skipped product graph")

    e=c["UnknownPathFullRequiredFanout"]["expect"]
    require(e.get("router_disposition")=="AdmitUnknown","unknown disposition drift")
    expect_jobs(c["UnknownPathFullRequiredFanout"],"ALL_REQUIRED",[])

    m=c["RenamePreservesPreviousRelevantPath"]["mutation"]
    require(m.get("kind")=="rename_unchanged" and m.get("from")=="mycelix-governance/README.md","rename source drift")
    require(m.get("to")=="docs/lex-net/CI_GOV_001J_P5_RENAMED_GOVERNANCE_README.md","rename target drift")
    require(m.get("require_api_status")=="renamed" and m.get("require_previous_filename") is True,"rename API evidence drift")
    expect_jobs(c["RenamePreservesPreviousRelevantPath"],["format","test-governance"],[])

    e=c["InformationalOnlyKnownRelevantClosure"]["expect"]
    require(c["InformationalOnlyKnownRelevantClosure"]["mutation"].get("path")=="nix/modules/holochain-base.nix","informational-only path drift")
    require(e.get("router_disposition")=="AdmitKnownRelevant" and e.get("authority_complete") is True,"informational-only disposition drift")
    expect_jobs(c["InformationalOnlyKnownRelevantClosure"],[],INFO)
    require(e.get("informational_observation")=="success_or_failure_terminal","informational-only observation contract drift")

    e=c["InformationalFinanceObservationNonGating"]["expect"]
    require(c["InformationalFinanceObservationNonGating"]["mutation"].get("kind")=="append_intentional_invalid_nix","informational RED mutation drift")
    expect_jobs(c["InformationalFinanceObservationNonGating"],[],INFO)
    require(e.get("informational_job_conclusion")=="success","informational wrapper job must remain scheduler-successful")
    require(e.get("informational_observation")=="failure" and e.get("informational_failure_phase")=="build-zomes","informational RED observation drift")
    require(e.get("merge_authority_passed") is True,"informational RED must remain non-gating")

    e=c["RequiredProductRedBlocks"]["expect"]
    require(c["RequiredProductRedBlocks"]["mutation"].get("path")=="mycelix-finance/Cargo.toml","required RED path drift")
    expect_jobs(c["RequiredProductRedBlocks"],["format","test-finance"],INFO)
    require(e.get("at_least_one_selected_required_not_success") is True,"required RED predicate missing")
    require(e.get("aggregate_disposition")=="RequiredJobNotSuccessful" and e.get("merge_authority_passed") is False,"required RED must block")

    e=c["StaleSubjectCannotQualifyNewerHead"]["expect"]
    require(c["StaleSubjectCannotQualifyNewerHead"]["mutation"].get("kind")=="two_head_sequence","stale-subject mutation drift")
    require(e.get("first_run_target")=="H1" and e.get("second_run_target")=="H2" and e.get("current_pr_head")=="H2","stale head sequence drift")
    require(e.get("first_run_must_not_satisfy_H2") is True and e.get("no_cancel_in_progress") is True,"stale authority invariant missing")

    e=c["AuthorityInputUncertaintyDeniesPass"]["expect"]
    m=c["AuthorityInputUncertaintyDeniesPass"]["mutation"]
    require(m.get("kind")=="add_many_files" and m.get("count")==3001,"large uncertainty fixture drift")
    require(e.get("router_disposition")=="AuthorityFilesIndeterminate" and e.get("authority_complete") is False,"uncertainty disposition drift")
    expect_jobs(c["AuthorityInputUncertaintyDeniesPass"],"ALL_REQUIRED",[])
    require(e.get("aggregate_disposition")=="AuthorityInputsIncomplete" and e.get("merge_authority_passed") is False,"uncertainty must deny PASS")

    static=p.get("static_readiness"); require(isinstance(static,dict),"static readiness missing")
    require(static.get("id")=="P5-S10" and static.get("name")=="MergeGroupFullRequiredFanoutReady","static case identity drift")
    req=static.get("requirements")
    require(isinstance(req,list) and set(req)=={"authority_workflow_has_merge_group_trigger","router_merge_group_selects_all_required","informational_jobs_not_selected_for_merge_group","no_pr_path_optimization_for_merge_group"},"merge-group readiness requirements drift")
    require(static.get("runtime_not_applicable_reason")=="current exact ruleset has no merge_queue rule","merge-group applicability rationale drift")
    future=p.get("conditional_future_case"); require(isinstance(future,dict),"future merge-group case missing")
    require(future.get("name")=="MergeGroupFullRequiredFanout" and future.get("required_if")=="future authority epoch enables merge queue" and future.get("must_execute_before_active") is True,"future merge-group theorem drift")

    q=p.get("qualification"); require(isinstance(q,dict),"qualification contract missing")
    require(q.get("claim")=="RulesetEvaluateExecutionBound","P5 claim drift")
    require(q.get("required_runtime_case_ids")==[f"P5-0{i}" for i in range(1,10)],"runtime qualification IDs drift")
    require(q.get("required_static_case_ids")==["P5-S10"],"static qualification IDs drift")
    require(q.get("case_results_must_be_retained") is True and q.get("ruleset_raw_payload_must_be_retained") is True and q.get("p4_receipt_must_be_retained") is True,"P5 evidence retention drift")
    require(q.get("no_product_qualification") is True,"P5 product nonclaim missing")
    return {"validator_id":VALIDATOR_ID,"valid":True,"plan_id":PLAN_ID,"runtime_case_count":9,"static_case_count":1,"authority_source_sha":SOURCE_SHA,"claim_ceiling":"RulesetEvaluateExecutionBound","executes_probes":False,"grants_product_qualification":False}

def must_fail(p: dict[str,Any])->None:
    try: validate(p)
    except PlanError: return
    raise AssertionError("unsafe P5 plan unexpectedly validated")

def self_test(p: dict[str,Any])->None:
    validate(p)
    x=copy.deepcopy(p); x["runtime_cases"][4]["expect"]["required_selected"]=["format"]; must_fail(x)
    x=copy.deepcopy(p); x["runtime_cases"][5]["expect"]["merge_authority_passed"]=False; must_fail(x)
    x=copy.deepcopy(p); x["runtime_cases"][8]["expect"]["authority_complete"]=True; must_fail(x)
    x=copy.deepcopy(p); x["static_readiness"]["requirements"].remove("router_merge_group_selects_all_required"); must_fail(x)
    x=copy.deepcopy(p); x["conditional_future_case"]["must_execute_before_active"]=False; must_fail(x)

def main()->int:
    a=argparse.ArgumentParser(); a.add_argument("--plan",default="docs/ci/ruleset_authority_p5_plan_v3.json"); a.add_argument("--self-test",action="store_true"); args=a.parse_args()
    try:
        p=json.loads(Path(args.plan).read_text(encoding="utf-8"));
        if args.self_test: self_test(p)
        r=validate(p); r.update({"self_test":"PASS"} if args.self_test else {}); print(json.dumps(r,sort_keys=True)); return 0
    except (OSError,json.JSONDecodeError,PlanError,AssertionError) as exc:
        print(json.dumps({"validator_id":VALIDATOR_ID,"valid":False,"reason":str(exc),"executes_probes":False,"grants_product_qualification":False},sort_keys=True)); return 2
if __name__=="__main__": raise SystemExit(main())
