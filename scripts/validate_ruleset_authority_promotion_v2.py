#!/usr/bin/env python3
"""Static validator for CI-GOV-001J promotion contract v2."""
from __future__ import annotations

import argparse
import copy
import json
import re
from pathlib import Path
from typing import Any

VALIDATOR_ID = "ci-gov-001j-promotion-contract-validator-v2"
CONTRACT_ID = "ci-gov-001j-authority-promotion-v2"
SOURCE_SHA = "a248fe016e2f7fbe5a632595265887bffdf22df9"
QUAL_SHA = "b1031615e22d93b4f64d52dd93272c7e468a2410"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")
EXPECTED_STATES = [
    ("P0","SourceQualified","RulesetAuthoritySourceCapsuleBound",[]),
    ("P1","AuthorityRefCreated","AuthorityRefExactSourceBound",["P0"]),
    ("P2","AuthorityRefProtected","AuthorityRefProtectionBound",["P1"]),
    ("P3","RequiredWorkflowCapabilityVerified","RequiredWorkflowCapabilityObserved",["P2"]),
    ("P4","RulesetEvaluateConfigured","RulesetEvaluateConfigurationBound",["P3"]),
    ("P5","RulesetEvaluateExecutionQualified","RulesetEvaluateExecutionBound",["P4"]),
    ("P6","RulesetActive","RulesetActiveConfigurationBound",["P5"]),
]
EXPECTED_CASES = {
    "KnownRelevantClosure",
    "ProvenIrrelevantTypedNoCiRequired",
    "UnknownPathFullRequiredFanout",
    "RenamePreservesPreviousRelevantPath",
    "InformationalOnlyKnownRelevantClosure",
    "InformationalFinanceObservationNonGating",
    "RequiredProductRedBlocks",
    "StaleSubjectCannotQualifyNewerHead",
    "MergeGroupFullRequiredFanout",
    "AuthorityInputUncertaintyDeniesPass",
}
EXPECTED_EPOCH_FIELDS = {
    "authority_source_sha","workflow_blob","manifest_blob",
    "qualification_receipt_sha256","ruleset_id",
    "ruleset_configuration_commitment","enforcement_mode",
}

class ContractError(ValueError):
    pass

def require(ok: bool, msg: str) -> None:
    if not ok:
        raise ContractError(msg)

def sha40(value: Any, label: str) -> str:
    require(isinstance(value,str) and SHA_RE.fullmatch(value) is not None,
            f"{label} must be lowercase 40-hex")
    return value


def validate(c: Any) -> dict[str, Any]:
    require(isinstance(c,dict), "contract root must be object")
    require(c.get("contract_id") == CONTRACT_ID, "contract identity drift")
    require(c.get("version") == 2, "version drift")
    require(c.get("issue") == 1705, "issue binding drift")
    require(c.get("authority") == "PromotionGovernanceOnly", "authority drift")
    require(c.get("supersedes") == "ci-gov-001j-authority-promotion-v1", "supersession drift")

    source=c.get("source")
    require(isinstance(source,dict), "source missing")
    for key in ("repaired_executable_authority_sha","authority_source_root_sha",
                "prior_authority_source_root_sha","qualification_head_sha","qualification_base_sha"):
        sha40(source.get(key), f"source.{key}")
    require(source.get("authority_source_root_sha") == SOURCE_SHA, "source root drift")
    require(source.get("qualification_pr") == 1747, "qualification PR drift")
    require(source.get("qualification_head_sha") == QUAL_SHA, "qualification head drift")
    require(source.get("qualification_claim") == "RulesetAuthoritySourceCapsuleBound", "qualification claim drift")
    require(source.get("informational_only_repair_required") is True, "repair binding missing")

    target=c.get("promotion_target")
    require(isinstance(target,dict), "promotion_target missing")
    require(target.get("repository") == "Luminous-Dynamics/mycelix", "repository drift")
    require(target.get("repository_id") == 1176351975, "repository id drift")
    require(target.get("ref") == "refs/heads/ci-authority/v1", "promotion ref drift")
    require(target.get("initial_source_sha") == SOURCE_SHA, "promotion source drift")
    require(target.get("required_workflow_path") == ".github/workflows/ruleset-generic-ci-authority.yml", "workflow path drift")

    states=c.get("states")
    require(isinstance(states,list) and len(states)==len(EXPECTED_STATES), "state census drift")
    for state, expected in zip(states, EXPECTED_STATES):
        require(isinstance(state,dict), "state must be object")
        sid,name,claim,reqs=expected
        require(state.get("id")==sid and state.get("name")==name, f"state identity drift {sid}")
        require(state.get("claim")==claim and state.get("requires")==reqs, f"state theorem drift {sid}")
        require(isinstance(state.get("mutation_allowed"),bool), f"mutation flag invalid {sid}")

    protection=c.get("protection_requirements")
    require(isinstance(protection,dict), "protection requirements missing")
    for key in ("block_deletion","block_force_push","block_unqualified_direct_update",
                "bypass_identities_must_be_enumerated","bypass_set_must_be_minimized",
                "qualified_successor_required_for_update"):
        require(protection.get(key) is True, f"protection invariant missing {key}")

    workflow=c.get("required_workflow_configuration")
    require(isinstance(workflow,dict), "required workflow config missing")
    require(workflow.get("source_repository") == "Luminous-Dynamics/mycelix", "workflow repo drift")
    require(workflow.get("source_repository_id") == 1176351975, "workflow repo id drift")
    require(workflow.get("source_branch") == "ci-authority/v1", "workflow branch drift")
    require(workflow.get("source_sha") == SOURCE_SHA, "workflow source SHA drift")
    require(workflow.get("workflow_path") == ".github/workflows/ruleset-generic-ci-authority.yml", "workflow path drift")
    require(workflow.get("evaluate_before_active") is True, "Evaluate-first requirement missing")
    require(workflow.get("merge_group_required") is True, "merge_group requirement missing")
    require(workflow.get("pull_request_required") is True, "pull_request requirement missing")
    require(workflow.get("pull_request_target_forbidden") is True, "pull_request_target must be forbidden")

    cases=c.get("evaluate_execution_cases")
    require(isinstance(cases,list) and set(cases)==EXPECTED_CASES and len(cases)==len(EXPECTED_CASES),
            "P5 case census drift")

    activation=c.get("activation")
    require(isinstance(activation,dict), "activation contract missing")
    require(activation.get("requires_exact_evaluate_configuration_identity") is True, "exact Evaluate identity missing")
    require(activation.get("requires_all_prior_states") == ["P0","P1","P2","P3","P4","P5"], "Active prerequisites drift")
    require(activation.get("configuration_drift_requires_requalification") is True, "config drift requalification missing")

    rollback=c.get("rollback")
    require(isinstance(rollback,dict), "rollback contract missing")
    for key in ("deactivate_before_source_change","force_move_while_active_forbidden",
                "rollback_target_must_be_previously_qualified","record_prior_active_sha",
                "record_replacement_sha","record_ruleset_transition","record_reason",
                "record_operator_identity","evaluate_before_reactivation"):
        require(rollback.get(key) is True, f"rollback invariant missing {key}")

    epoch=c.get("authority_epoch")
    require(isinstance(epoch,dict), "authority_epoch missing")
    fields=epoch.get("required_fields")
    require(isinstance(fields,list) and set(fields)==EXPECTED_EPOCH_FIELDS and len(fields)==len(EXPECTED_EPOCH_FIELDS),
            "authority epoch field drift")
    require(epoch.get("new_source_sha_starts_new_epoch") is True, "source change must start epoch")
    require(epoch.get("ruleset_configuration_change_starts_new_epoch") is True, "config change must start epoch")
    require(epoch.get("cross_epoch_evidence_mixing_forbidden") is True, "cross-epoch evidence mixing must be forbidden")

    return {"validator_id":VALIDATOR_ID,"valid":True,"contract_id":CONTRACT_ID,
            "source_sha":SOURCE_SHA,"qualification_head_sha":QUAL_SHA,
            "state_count":len(states),"evaluate_case_count":len(cases),
            "epoch_field_count":len(fields),"claim_ceiling":"AuthorityPromotionContractBound",
            "grants_promotion":False,"grants_ruleset_activation":False,
            "grants_product_qualification":False}


def must_fail(c: dict[str, Any]) -> None:
    try:
        validate(c)
    except ContractError:
        return
    raise AssertionError("unsafe mutation unexpectedly validated")


def self_test(c: dict[str, Any]) -> None:
    validate(c)
    x=copy.deepcopy(c); x["source"]["qualification_head_sha"]="0"*40; must_fail(x)
    x=copy.deepcopy(c); x["required_workflow_configuration"]["source_sha"]="0"*40; must_fail(x)
    x=copy.deepcopy(c); x["evaluate_execution_cases"].remove("InformationalOnlyKnownRelevantClosure"); must_fail(x)
    x=copy.deepcopy(c); x["activation"]["requires_all_prior_states"]=["P0","P1","P2","P3","P4"]; must_fail(x)
    x=copy.deepcopy(c); x["rollback"]["deactivate_before_source_change"]=False; must_fail(x)
    x=copy.deepcopy(c); x["authority_epoch"]["required_fields"].remove("qualification_receipt_sha256"); must_fail(x)


def main() -> int:
    p=argparse.ArgumentParser()
    p.add_argument("--contract", default="docs/ci/ruleset_authority_promotion_v2.json")
    p.add_argument("--self-test", action="store_true")
    args=p.parse_args()
    try:
        contract=json.loads(Path(args.contract).read_text(encoding="utf-8"))
        if args.self_test:
            self_test(contract)
        result=validate(contract)
        if args.self_test:
            result["self_test"]="PASS"
        print(json.dumps(result,sort_keys=True))
        return 0
    except (OSError,json.JSONDecodeError,ContractError,AssertionError) as exc:
        print(json.dumps({"validator_id":VALIDATOR_ID,"valid":False,"reason":str(exc),
                          "claim_ceiling":"AuthorityPromotionContractBound",
                          "grants_promotion":False,"grants_ruleset_activation":False,
                          "grants_product_qualification":False},sort_keys=True))
        return 2

if __name__ == "__main__":
    raise SystemExit(main())
