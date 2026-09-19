#!/usr/bin/env python3
"""Canonicalize and validate CI-GOV-001J organization-ruleset configuration."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

VALIDATOR_ID = "ci-gov-001j-ruleset-configuration-validator-v1"
CONTRACT_ID = "ci-gov-001j-ruleset-configuration-v1"
ORG = "Luminous-Dynamics"
REPO_ID = 1176351975
RULESET_NAME = "Mycelix Generic CI Authority v1"
WORKFLOW_PATH = ".github/workflows/ruleset-generic-ci-authority.yml"
WORKFLOW_REF = "refs/heads/ci-authority/v1"
WORKFLOW_SHA = "979160704924d992c4f1c1af32279615c83ae062"
TARGET_REF = "refs/heads/main"


class ConfigError(ValueError):
    pass


def require(ok: bool, msg: str) -> None:
    if not ok:
        raise ConfigError(msg)


def stable_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def load_raw(path: Path) -> tuple[dict[str, Any], str]:
    raw = path.read_bytes()
    try:
        obj = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ConfigError(f"invalid ruleset JSON: {exc}") from exc
    require(isinstance(obj, dict), "ruleset root must be object")
    return obj, sha256_bytes(raw)


def validate_contract(c: Any) -> None:
    require(isinstance(c, dict), "configuration contract root must be object")
    require(c.get("configuration_contract_id") == CONTRACT_ID, "contract identity drift")
    require(c.get("version") == 1, "contract version drift")
    require(c.get("issue") == 1705, "issue drift")
    require(c.get("organization") == ORG, "organization drift")
    require(c.get("ruleset_name") == RULESET_NAME, "ruleset name drift")
    require(c.get("target") == "branch", "target drift")
    require(c.get("repository_id") == REPO_ID, "repository id drift")
    require(c.get("target_refs") == {"include":[TARGET_REF], "exclude":[]}, "target ref drift")
    require(c.get("bypass_actors") == [], "v1 requires empty bypass actor set")
    wr = c.get("workflow_rule")
    require(isinstance(wr, dict), "workflow_rule missing")
    require(wr.get("type") == "workflows", "workflow rule type drift")
    require(wr.get("do_not_enforce_on_create") is False, "do_not_enforce_on_create drift")
    require(wr.get("workflows") == [{
        "path": WORKFLOW_PATH,
        "ref": WORKFLOW_REF,
        "repository_id": REPO_ID,
        "sha": WORKFLOW_SHA,
    }], "workflow source binding drift")
    require(c.get("p4", {}).get("required_enforcement") == "evaluate", "P4 enforcement drift")
    require(c.get("p6", {}).get("required_enforcement") == "active", "P6 enforcement drift")
    require(c.get("p6", {}).get("must_match_p4_structural_commitment") is True,
            "P6 structural identity requirement missing")
    require(c.get("p6", {}).get("must_match_p4_ruleset_id") is True,
            "P6 ruleset id identity requirement missing")


def canonicalize(ruleset: dict[str, Any], expected_enforcement: str) -> dict[str, Any]:
    require(expected_enforcement in {"evaluate", "active"}, "unsupported enforcement")
    require(isinstance(ruleset.get("id"), int) and ruleset["id"] > 0, "ruleset id invalid")
    require(ruleset.get("name") == RULESET_NAME, "ruleset name mismatch")
    require(ruleset.get("target") == "branch", "ruleset target mismatch")
    require(ruleset.get("source_type") == "Organization", "ruleset source_type mismatch")
    require(ruleset.get("source") == ORG, "ruleset source mismatch")
    require(ruleset.get("enforcement") == expected_enforcement,
            f"ruleset enforcement must be {expected_enforcement}")

    bypass = ruleset.get("bypass_actors")
    require(bypass == [], "v1 authority ruleset must have empty bypass_actors")

    conditions = ruleset.get("conditions")
    require(isinstance(conditions, dict), "conditions missing")
    repo_cond = conditions.get("repository_id")
    ref_cond = conditions.get("ref_name")
    require(repo_cond == {"repository_ids":[REPO_ID]}, "repository targeting mismatch")
    require(ref_cond == {"include":[TARGET_REF], "exclude":[]}, "ref targeting mismatch")
    require(set(conditions) == {"repository_id", "ref_name"}, "unexpected ruleset conditions")

    rules = ruleset.get("rules")
    require(isinstance(rules, list) and len(rules) == 1, "v1 requires exactly one ruleset rule")
    rule = rules[0]
    require(isinstance(rule, dict) and rule.get("type") == "workflows", "workflows rule missing")
    params = rule.get("parameters")
    require(isinstance(params, dict), "workflows parameters missing")
    require(params.get("do_not_enforce_on_create") is False,
            "do_not_enforce_on_create must be false")
    workflows = params.get("workflows")
    require(isinstance(workflows, list) and len(workflows) == 1,
            "v1 requires exactly one required workflow")
    wf = workflows[0]
    require(isinstance(wf, dict), "workflow binding invalid")
    require(wf.get("path") == WORKFLOW_PATH, "workflow path mismatch")
    require(wf.get("ref") == WORKFLOW_REF, "workflow ref mismatch")
    require(wf.get("repository_id") == REPO_ID, "workflow repository_id mismatch")
    require(wf.get("sha") == WORKFLOW_SHA, "workflow source SHA mismatch")
    require(set(wf) <= {"path", "ref", "repository_id", "sha"},
            "unexpected workflow binding fields")
    require(set(params) <= {"do_not_enforce_on_create", "workflows"},
            "unexpected workflow parameters")

    structural = {
        "name": RULESET_NAME,
        "target": "branch",
        "source_type": "Organization",
        "source": ORG,
        "bypass_actors": [],
        "conditions": {
            "repository_id": {"repository_ids":[REPO_ID]},
            "ref_name": {"include":[TARGET_REF], "exclude":[]},
        },
        "rules": [{
            "type":"workflows",
            "parameters": {
                "do_not_enforce_on_create": False,
                "workflows": [{
                    "path": WORKFLOW_PATH,
                    "ref": WORKFLOW_REF,
                    "repository_id": REPO_ID,
                    "sha": WORKFLOW_SHA,
                }],
            },
        }],
    }
    structural_commitment = sha256_bytes(stable_json(structural))
    config = {
        "ruleset_id": ruleset["id"],
        "enforcement": expected_enforcement,
        "structural_configuration": structural,
    }
    config_commitment = sha256_bytes(stable_json(config))
    return {
        "ruleset_id": ruleset["id"],
        "enforcement": expected_enforcement,
        "structural_configuration": structural,
        "structural_commitment_sha256": structural_commitment,
        "configuration_commitment_sha256": config_commitment,
    }


def evaluate(contract: dict[str, Any], ruleset: dict[str, Any], raw_sha256: str,
             expected_enforcement: str, evaluate_reference: dict[str, Any] | None = None) -> dict[str, Any]:
    validate_contract(contract)
    canon = canonicalize(ruleset, expected_enforcement)
    result = {
        "validator_id": VALIDATOR_ID,
        "valid": True,
        "configuration_contract_id": CONTRACT_ID,
        "raw_payload_sha256": raw_sha256,
        **canon,
        "workflow_source_sha": WORKFLOW_SHA,
        "claim_ceiling": "RulesetEvaluateConfigurationBound" if expected_enforcement == "evaluate"
                         else "RulesetActiveConfigurationBound",
        "grants_p5_execution_qualification": False,
        "grants_product_qualification": False,
    }
    if expected_enforcement == "active":
        require(evaluate_reference is not None, "active validation requires Evaluate reference")
        require(evaluate_reference.get("valid") is True, "Evaluate reference invalid")
        require(evaluate_reference.get("enforcement") == "evaluate", "reference is not Evaluate")
        require(evaluate_reference.get("ruleset_id") == canon["ruleset_id"],
                "Active transition changed ruleset id")
        require(evaluate_reference.get("structural_commitment_sha256") == canon["structural_commitment_sha256"],
                "Active transition changed structural configuration")
        require(evaluate_reference.get("workflow_source_sha") == WORKFLOW_SHA,
                "Active transition changed workflow source SHA")
        result["evaluate_to_active_identity_preserved"] = True
    return result


def fixture(enforcement: str) -> dict[str, Any]:
    return {
        "id": 42,
        "name": RULESET_NAME,
        "target": "branch",
        "source_type": "Organization",
        "source": ORG,
        "enforcement": enforcement,
        "bypass_actors": [],
        "conditions": {
            "repository_id": {"repository_ids":[REPO_ID]},
            "ref_name": {"include":[TARGET_REF], "exclude":[]},
        },
        "rules": [{"type":"workflows","parameters":{
            "do_not_enforce_on_create":False,
            "workflows":[{"path":WORKFLOW_PATH,"ref":WORKFLOW_REF,
                          "repository_id":REPO_ID,"sha":WORKFLOW_SHA}],
        }}],
        "created_at":"2026-09-19T00:00:00Z",
        "updated_at":"2026-09-19T00:00:00Z",
        "node_id":"fixture",
        "_links":{},
    }


def must_fail(contract: dict[str, Any], ruleset: dict[str, Any], mode: str) -> None:
    try:
        evaluate(contract, ruleset, "0"*64, mode)
    except ConfigError:
        return
    raise AssertionError("unsafe ruleset unexpectedly validated")


def self_test(contract: dict[str, Any]) -> None:
    ev = evaluate(contract, fixture("evaluate"), "1"*64, "evaluate")
    active = evaluate(contract, fixture("active"), "2"*64, "active", ev)
    assert active["evaluate_to_active_identity_preserved"] is True
    assert ev["structural_commitment_sha256"] == active["structural_commitment_sha256"]
    assert ev["configuration_commitment_sha256"] != active["configuration_commitment_sha256"]

    x = fixture("evaluate"); x["rules"][0]["parameters"]["workflows"][0]["sha"] = "0"*40; must_fail(contract, x, "evaluate")
    x = fixture("evaluate"); x["bypass_actors"] = [{"actor_id":1,"actor_type":"User","bypass_mode":"always"}]; must_fail(contract, x, "evaluate")
    x = fixture("evaluate"); x["conditions"]["ref_name"]["include"] = ["~ALL"]; must_fail(contract, x, "evaluate")
    x = fixture("evaluate"); x["rules"].append({"type":"deletion"}); must_fail(contract, x, "evaluate")

    active_changed = fixture("active")
    active_changed["rules"][0]["parameters"]["workflows"][0]["ref"] = "refs/heads/other"
    try:
        evaluate(contract, active_changed, "3"*64, "active", ev)
    except ConfigError:
        pass
    else:
        raise AssertionError("structurally changed Active config unexpectedly validated")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--contract", default="docs/ci/ruleset_authority_configuration_v1.json")
    p.add_argument("--ruleset")
    p.add_argument("--mode", choices=["evaluate", "active"])
    p.add_argument("--evaluate-receipt")
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    try:
        contract = json.loads(Path(args.contract).read_text(encoding="utf-8"))
        if args.self_test:
            self_test(contract)
            print(json.dumps({"validator_id":VALIDATOR_ID,"self_test":"PASS","workflow_source_sha":WORKFLOW_SHA,"grants_product_qualification":False}, sort_keys=True))
            return 0
        require(args.ruleset is not None and args.mode is not None,
                "--ruleset and --mode are required unless --self-test")
        ruleset, raw_sha = load_raw(Path(args.ruleset))
        ev_ref = None
        if args.evaluate_receipt:
            ev_ref = json.loads(Path(args.evaluate_receipt).read_text(encoding="utf-8"))
        result = evaluate(contract, ruleset, raw_sha, args.mode, ev_ref)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, json.JSONDecodeError, ConfigError, AssertionError) as exc:
        print(json.dumps({"validator_id":VALIDATOR_ID,"valid":False,"reason":str(exc),"workflow_source_sha":WORKFLOW_SHA,"grants_ruleset_activation":False,"grants_product_qualification":False}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
