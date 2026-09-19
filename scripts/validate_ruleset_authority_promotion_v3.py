#!/usr/bin/env python3
"""Validate CI-GOV-001J promotion refinement v3."""
from __future__ import annotations
import argparse, copy, json
from pathlib import Path
from typing import Any

VALIDATOR_ID="ci-gov-001j-promotion-refinement-validator-v3"
CONTRACT_ID="ci-gov-001j-authority-promotion-v3"
SOURCE_SHA="a248fe016e2f7fbe5a632595265887bffdf22df9"
QUAL_SHA="b1031615e22d93b4f64d52dd93272c7e468a2410"
REQUIRED_RUNTIME={
"KnownRelevantClosure","ProvenIrrelevantTypedNoCiRequired","UnknownPathFullRequiredFanout",
"RenamePreservesPreviousRelevantPath","InformationalOnlyKnownRelevantClosure",
"InformationalFinanceObservationNonGating","RequiredProductRedBlocks",
"StaleSubjectCannotQualifyNewerHead","AuthorityInputUncertaintyDeniesPass"}
STATIC={"MergeGroupFullRequiredFanoutReady"}
READINESS={"authority_workflow_has_merge_group_trigger","router_merge_group_selects_all_required",
"informational_jobs_not_selected_for_merge_group","no_pr_path_optimization_for_merge_group"}

class ContractError(ValueError): pass

def require(ok: bool,msg: str)->None:
    if not ok: raise ContractError(msg)

def validate(c: Any)->dict[str,Any]:
    require(isinstance(c,dict),"contract root must be object")
    require(c.get("contract_id")==CONTRACT_ID,"contract identity drift")
    require(c.get("version")==3,"version drift")
    require(c.get("issue")==1705,"issue drift")
    require(c.get("supersedes")=="ci-gov-001j-authority-promotion-v2","supersession drift")
    require(c.get("authority_source_sha")==SOURCE_SHA,"source SHA drift")
    require(c.get("qualification_head_sha")==QUAL_SHA,"qualification head drift")
    require(c.get("promotion_v2_head")=="d5df13d4ff9078b43f6846a634b7dc5f7d18e302","promotion-v2 head drift")
    require(c.get("admin_observation_v2_head")=="5328e0ce2edca544d76644184b3c094bc4095733","observation-v2 head drift")
    require(c.get("ruleset_configuration_v2_head")=="db82151323e5aba22a78bd32a5401e2e5922b217","config-v2 head drift")
    epoch=c.get("current_epoch")
    require(isinstance(epoch,dict),"current_epoch missing")
    require(epoch.get("workflow_source_sha")==SOURCE_SHA,"epoch source drift")
    require(epoch.get("merge_queue_rule_present") is False,"current epoch unexpectedly enables merge queue")
    require(epoch.get("merge_group_runtime_applicable") is False,"merge_group runtime applicability drift")
    p5=c.get("p5")
    require(isinstance(p5,dict),"P5 contract missing")
    rr=p5.get("required_runtime_cases")
    sr=p5.get("static_readiness_cases")
    require(isinstance(rr,list) and set(rr)==REQUIRED_RUNTIME and len(rr)==len(REQUIRED_RUNTIME),"runtime case census drift")
    require(isinstance(sr,list) and set(sr)==STATIC and len(sr)==len(STATIC),"static readiness census drift")
    cond=p5.get("conditional_runtime_cases")
    require(isinstance(cond,list) and len(cond)==1,"conditional runtime case census drift")
    require(cond[0].get("case")=="MergeGroupFullRequiredFanout","merge-group conditional case drift")
    require(cond[0].get("required_when")=="ruleset_or_branch_protection_enables_merge_queue","merge-group condition drift")
    require(cond[0].get("new_authority_epoch_required_if_enabled") is True,"merge-group new-epoch requirement missing")
    readiness=p5.get("merge_group_readiness_requirements")
    require(isinstance(readiness,list) and set(readiness)==READINESS and len(readiness)==len(READINESS),"merge-group readiness drift")
    p6=c.get("p6")
    require(isinstance(p6,dict),"P6 contract missing")
    require(p6.get("requires_p5") is True,"P6 must require P5")
    require(p6.get("requires_exact_p4_configuration_identity") is True,"P6 P4 identity requirement missing")
    require(p6.get("merge_group_runtime_required_for_current_epoch") is False,"current epoch merge-group runtime requirement drift")
    future=c.get("future_merge_queue_epoch")
    require(isinstance(future,dict),"future merge-queue epoch missing")
    for key in ("must_add_merge_queue_rule_to_structural_configuration","must_start_new_authority_epoch",
                "must_requalify_p4","must_execute_merge_group_runtime_case_before_active"):
        require(future.get(key) is True,f"future merge-queue invariant missing {key}")
    return {"validator_id":VALIDATOR_ID,"valid":True,"contract_id":CONTRACT_ID,
            "required_runtime_case_count":len(rr),"static_readiness_case_count":len(sr),
            "merge_group_runtime_applicable":False,"claim_ceiling":"AuthorityPromotionContractBound",
            "grants_promotion":False,"grants_ruleset_activation":False,"grants_product_qualification":False}

def must_fail(c: dict[str,Any])->None:
    try: validate(c)
    except ContractError: return
    raise AssertionError("unsafe refinement unexpectedly validated")

def self_test(c: dict[str,Any])->None:
    validate(c)
    x=copy.deepcopy(c); x["current_epoch"]["merge_queue_rule_present"]=True; must_fail(x)
    x=copy.deepcopy(c); x["p5"]["required_runtime_cases"].remove("InformationalOnlyKnownRelevantClosure"); must_fail(x)
    x=copy.deepcopy(c); x["p5"]["merge_group_readiness_requirements"].remove("router_merge_group_selects_all_required"); must_fail(x)
    x=copy.deepcopy(c); x["future_merge_queue_epoch"]["must_execute_merge_group_runtime_case_before_active"]=False; must_fail(x)

def main()->int:
    p=argparse.ArgumentParser(); p.add_argument("--contract",default="docs/ci/ruleset_authority_promotion_v3.json"); p.add_argument("--self-test",action="store_true"); a=p.parse_args()
    try:
        c=json.loads(Path(a.contract).read_text(encoding="utf-8"));
        if a.self_test: self_test(c)
        r=validate(c); r.update({"self_test":"PASS"} if a.self_test else {}); print(json.dumps(r,sort_keys=True)); return 0
    except (OSError,json.JSONDecodeError,ContractError,AssertionError) as exc:
        print(json.dumps({"validator_id":VALIDATOR_ID,"valid":False,"reason":str(exc),"grants_promotion":False,"grants_ruleset_activation":False},sort_keys=True)); return 2
if __name__=="__main__": raise SystemExit(main())
