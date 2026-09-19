#!/usr/bin/env python3
"""Validate CI-GOV-001J repaired Evaluate/Active ruleset configuration v2."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

VALIDATOR_ID = "ci-gov-001j-ruleset-configuration-validator-v2"
CONTRACT_ID = "ci-gov-001j-ruleset-configuration-v2"
PROMOTION_HEAD = "d5df13d4ff9078b43f6846a634b7dc5f7d18e302"
OBSERVATION_HEAD = "5328e0ce2edca544d76644184b3c094bc4095733"
ORG = "Luminous-Dynamics"
REPO_ID = 1176351975
RULESET_NAME = "Mycelix Generic CI Authority v2"
WORKFLOW_PATH = ".github/workflows/ruleset-generic-ci-authority.yml"
WORKFLOW_REF = "refs/heads/ci-authority/v1"
WORKFLOW_SHA = "a248fe016e2f7fbe5a632595265887bffdf22df9"
TARGET_REF = "refs/heads/main"

class ConfigError(ValueError):
    pass

def require(ok: bool, msg: str) -> None:
    if not ok:
        raise ConfigError(msg)

def stable_json(value: Any) -> bytes:
    return (json.dumps(value,sort_keys=True,separators=(",",":"))+"\n").encode()

def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()

def load_raw(path: Path):
    raw=path.read_bytes()
    try:
        obj=json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ConfigError(f"invalid ruleset JSON: {exc}") from exc
    require(isinstance(obj,dict), "ruleset root must be object")
    return obj,sha256_bytes(raw)


def validate_contract(c: Any) -> None:
    require(isinstance(c,dict), "configuration contract root must be object")
    require(c.get("configuration_contract_id") == CONTRACT_ID, "contract identity drift")
    require(c.get("version") == 2, "contract version drift")
    require(c.get("issue") == 1705, "issue drift")
    require(c.get("promotion_contract_head") == PROMOTION_HEAD, "promotion head drift")
    require(c.get("admin_observation_head") == OBSERVATION_HEAD, "admin observation head drift")
    require(c.get("organization") == ORG, "organization drift")
    require(c.get("ruleset_name") == RULESET_NAME, "ruleset name drift")
    require(c.get("target") == "branch", "target drift")
    require(c.get("repository_id") == REPO_ID, "repository id drift")
    require(c.get("authority_source_sha") == WORKFLOW_SHA, "authority source drift")
    require(c.get("target_refs") == {"include":[TARGET_REF],"exclude":[]}, "target refs drift")
    require(c.get("bypass_actors") == [], "v2 requires empty bypass actor set")
    wr=c.get("workflow_rule")
    require(isinstance(wr,dict), "workflow rule missing")
    require(wr.get("type") == "workflows", "workflow rule type drift")
    require(wr.get("do_not_enforce_on_create") is False, "create enforcement drift")
    require(wr.get("workflows") == [{"path":WORKFLOW_PATH,"ref":WORKFLOW_REF,
                                      "repository_id":REPO_ID,"sha":WORKFLOW_SHA}],
            "workflow source binding drift")
    p4=c.get("p4",{}); p6=c.get("p6",{})
    require(p4.get("required_enforcement") == "evaluate", "P4 enforcement drift")
    require(p4.get("admin_evidence_receipt_required") is True, "P4 admin evidence binding missing")
    require(p4.get("qualification_receipt_sha256_carried_forward") is True, "P4 P0 receipt carry-forward missing")
    require(p6.get("required_enforcement") == "active", "P6 enforcement drift")
    require(p6.get("must_match_p4_structural_commitment") is True, "P6 structural identity missing")
    require(p6.get("must_match_p4_ruleset_id") is True, "P6 ruleset id identity missing")
    require(p6.get("must_match_p4_qualification_receipt_sha256") is True, "P6 P0 receipt identity missing")


def validate_admin_receipt(receipt: Any) -> str:
    require(isinstance(receipt,dict), "P3 admin evidence receipt missing")
    require(receipt.get("valid") is True, "P3 admin evidence receipt invalid")
    require(receipt.get("highest_state") == "P3", "P3 admin evidence highest state drift")
    require(receipt.get("claim_ceiling") == "RequiredWorkflowCapabilityObserved", "P3 claim ceiling drift")
    require(receipt.get("authority_source_sha") == WORKFLOW_SHA, "P3 authority source drift")
    value=receipt.get("qualification_receipt_sha256")
    require(isinstance(value,str) and len(value)==64 and all(c in "0123456789abcdef" for c in value),
            "P3 qualification receipt sha256 invalid")
    return value


def canonicalize(ruleset: dict[str,Any], expected_enforcement: str) -> dict[str,Any]:
    require(expected_enforcement in {"evaluate","active"}, "unsupported enforcement")
    require(isinstance(ruleset.get("id"),int) and ruleset["id"] > 0, "ruleset id invalid")
    require(ruleset.get("name") == RULESET_NAME, "ruleset name mismatch")
    require(ruleset.get("target") == "branch", "ruleset target mismatch")
    require(ruleset.get("source_type") == "Organization", "ruleset source_type mismatch")
    require(ruleset.get("source") == ORG, "ruleset source mismatch")
    require(ruleset.get("enforcement") == expected_enforcement, f"ruleset enforcement must be {expected_enforcement}")
    require(ruleset.get("bypass_actors") == [], "v2 ruleset must have empty bypass actors")
    conditions=ruleset.get("conditions")
    require(isinstance(conditions,dict), "conditions missing")
    require(conditions.get("repository_id") == {"repository_ids":[REPO_ID]}, "repository targeting mismatch")
    require(conditions.get("ref_name") == {"include":[TARGET_REF],"exclude":[]}, "ref targeting mismatch")
    require(set(conditions) == {"repository_id","ref_name"}, "unexpected conditions")
    rules=ruleset.get("rules")
    require(isinstance(rules,list) and len(rules)==1, "v2 requires exactly one rule")
    rule=rules[0]
    require(isinstance(rule,dict) and rule.get("type") == "workflows", "workflows rule missing")
    params=rule.get("parameters")
    require(isinstance(params,dict), "workflows parameters missing")
    require(params.get("do_not_enforce_on_create") is False, "do_not_enforce_on_create must be false")
    workflows=params.get("workflows")
    require(isinstance(workflows,list) and len(workflows)==1, "v2 requires exactly one workflow")
    wf=workflows[0]
    require(isinstance(wf,dict), "workflow binding invalid")
    require(wf.get("path") == WORKFLOW_PATH, "workflow path mismatch")
    require(wf.get("ref") == WORKFLOW_REF, "workflow ref mismatch")
    require(wf.get("repository_id") == REPO_ID, "workflow repository mismatch")
    require(wf.get("sha") == WORKFLOW_SHA, "workflow source SHA mismatch")
    require(set(wf) <= {"path","ref","repository_id","sha"}, "unexpected workflow binding fields")
    require(set(params) <= {"do_not_enforce_on_create","workflows"}, "unexpected workflow parameters")
    structural={"name":RULESET_NAME,"target":"branch","source_type":"Organization","source":ORG,
        "bypass_actors":[],"conditions":{"repository_id":{"repository_ids":[REPO_ID]},
        "ref_name":{"include":[TARGET_REF],"exclude":[]}},"rules":[{"type":"workflows",
        "parameters":{"do_not_enforce_on_create":False,"workflows":[{"path":WORKFLOW_PATH,
        "ref":WORKFLOW_REF,"repository_id":REPO_ID,"sha":WORKFLOW_SHA}]}}]}
    return {"ruleset_id":ruleset["id"],"enforcement":expected_enforcement,
            "structural_configuration":structural,
            "structural_commitment_sha256":sha256_bytes(stable_json(structural))}


def evaluate(contract: dict[str,Any], ruleset: dict[str,Any], raw_sha256: str,
             expected_enforcement: str, admin_receipt: dict[str,Any] | None = None,
             evaluate_reference: dict[str,Any] | None = None) -> dict[str,Any]:
    validate_contract(contract)
    canon=canonicalize(ruleset,expected_enforcement)
    if expected_enforcement == "evaluate":
        require(admin_receipt is not None, "Evaluate validation requires P3 admin evidence receipt")
        p0_hash=validate_admin_receipt(admin_receipt)
    else:
        require(evaluate_reference is not None, "Active validation requires Evaluate reference")
        require(evaluate_reference.get("valid") is True, "Evaluate reference invalid")
        require(evaluate_reference.get("enforcement") == "evaluate", "reference is not Evaluate")
        p0_hash=evaluate_reference.get("qualification_receipt_sha256")
        require(isinstance(p0_hash,str) and len(p0_hash)==64, "Evaluate reference P0 receipt hash invalid")
    config={"ruleset_id":canon["ruleset_id"],"enforcement":expected_enforcement,
            "structural_configuration":canon["structural_configuration"],
            "qualification_receipt_sha256":p0_hash}
    result={"validator_id":VALIDATOR_ID,"valid":True,"configuration_contract_id":CONTRACT_ID,
            "raw_payload_sha256":raw_sha256,**canon,
            "configuration_commitment_sha256":sha256_bytes(stable_json(config)),
            "workflow_source_sha":WORKFLOW_SHA,"qualification_receipt_sha256":p0_hash,
            "claim_ceiling":"RulesetEvaluateConfigurationBound" if expected_enforcement=="evaluate" else "RulesetActiveConfigurationBound",
            "grants_p5_execution_qualification":False,"grants_product_qualification":False}
    if expected_enforcement == "active":
        require(evaluate_reference.get("ruleset_id") == canon["ruleset_id"], "Active changed ruleset id")
        require(evaluate_reference.get("structural_commitment_sha256") == canon["structural_commitment_sha256"], "Active changed structural config")
        require(evaluate_reference.get("workflow_source_sha") == WORKFLOW_SHA, "Active changed workflow source")
        require(evaluate_reference.get("qualification_receipt_sha256") == p0_hash, "Active changed P0 receipt identity")
        result["evaluate_to_active_identity_preserved"]=True
    return result


def fixture(enforcement: str) -> dict[str,Any]:
    return {"id":42,"name":RULESET_NAME,"target":"branch","source_type":"Organization","source":ORG,
      "enforcement":enforcement,"bypass_actors":[],"conditions":{"repository_id":{"repository_ids":[REPO_ID]},
      "ref_name":{"include":[TARGET_REF],"exclude":[]}},"rules":[{"type":"workflows","parameters":{
      "do_not_enforce_on_create":False,"workflows":[{"path":WORKFLOW_PATH,"ref":WORKFLOW_REF,
      "repository_id":REPO_ID,"sha":WORKFLOW_SHA}]}}],"created_at":"2026-09-19T00:00:00Z",
      "updated_at":"2026-09-19T00:00:00Z","node_id":"fixture","_links":{}}


def admin_fixture() -> dict[str,Any]:
    return {"valid":True,"highest_state":"P3","claim_ceiling":"RequiredWorkflowCapabilityObserved",
            "authority_source_sha":WORKFLOW_SHA,"qualification_receipt_sha256":"1"*64}


def must_fail(contract: dict[str,Any], ruleset: dict[str,Any], mode: str,
              admin: dict[str,Any] | None = None, ref: dict[str,Any] | None = None) -> None:
    try:
        evaluate(contract,ruleset,"0"*64,mode,admin,ref)
    except ConfigError:
        return
    raise AssertionError("unsafe ruleset unexpectedly validated")


def self_test(contract: dict[str,Any]) -> None:
    ev=evaluate(contract,fixture("evaluate"),"1"*64,"evaluate",admin_fixture())
    active=evaluate(contract,fixture("active"),"2"*64,"active",evaluate_reference=ev)
    assert active["evaluate_to_active_identity_preserved"] is True
    assert ev["structural_commitment_sha256"] == active["structural_commitment_sha256"]
    assert ev["configuration_commitment_sha256"] != active["configuration_commitment_sha256"]
    assert ev["qualification_receipt_sha256"] == active["qualification_receipt_sha256"]
    x=fixture("evaluate"); x["rules"][0]["parameters"]["workflows"][0]["sha"]="0"*40; must_fail(contract,x,"evaluate",admin_fixture())
    x=fixture("evaluate"); x["bypass_actors"]=[{"actor_id":1}]; must_fail(contract,x,"evaluate",admin_fixture())
    bad=admin_fixture(); bad["authority_source_sha"]="0"*40; must_fail(contract,fixture("evaluate"),"evaluate",bad)
    bad=admin_fixture(); bad["qualification_receipt_sha256"]="0"*63; must_fail(contract,fixture("evaluate"),"evaluate",bad)
    changed=fixture("active"); changed["rules"][0]["parameters"]["workflows"][0]["ref"]="refs/heads/other"; must_fail(contract,changed,"active",ref=ev)


def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument("--contract",default="docs/ci/ruleset_authority_configuration_v2.json")
    p.add_argument("--ruleset"); p.add_argument("--mode",choices=["evaluate","active"])
    p.add_argument("--admin-evidence-receipt"); p.add_argument("--evaluate-receipt"); p.add_argument("--self-test",action="store_true")
    args=p.parse_args()
    try:
        contract=json.loads(Path(args.contract).read_text(encoding="utf-8"))
        if args.self_test:
            self_test(contract); print(json.dumps({"validator_id":VALIDATOR_ID,"self_test":"PASS",
                "workflow_source_sha":WORKFLOW_SHA,"grants_product_qualification":False},sort_keys=True)); return 0
        require(args.ruleset is not None and args.mode is not None,"--ruleset and --mode required")
        ruleset,raw_sha=load_raw(Path(args.ruleset)); admin=None; ev_ref=None
        if args.admin_evidence_receipt: admin=json.loads(Path(args.admin_evidence_receipt).read_text(encoding="utf-8"))
        if args.evaluate_receipt: ev_ref=json.loads(Path(args.evaluate_receipt).read_text(encoding="utf-8"))
        print(json.dumps(evaluate(contract,ruleset,raw_sha,args.mode,admin,ev_ref),sort_keys=True)); return 0
    except (OSError,json.JSONDecodeError,ConfigError,AssertionError) as exc:
        print(json.dumps({"validator_id":VALIDATOR_ID,"valid":False,"reason":str(exc),
            "workflow_source_sha":WORKFLOW_SHA,"grants_ruleset_activation":False,
            "grants_product_qualification":False},sort_keys=True)); return 2

if __name__ == "__main__":
    raise SystemExit(main())
