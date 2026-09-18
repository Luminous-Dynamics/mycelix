#!/usr/bin/env python3
"""Static validator for CI-GOV-001J authority promotion contract v1."""
from __future__ import annotations

import argparse
import copy
import json
import re
from pathlib import Path
from typing import Any

VALIDATOR_ID = "ci-gov-001j-promotion-contract-validator-v1"
CONTRACT_ID = "ci-gov-001j-authority-promotion-v1"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")
EXPECTED_STATES = [
    ("P0", "SourceQualified", "RulesetAuthoritySourceCapsuleBound", []),
    ("P1", "AuthorityRefCreated", "AuthorityRefExactSourceBound", ["P0"]),
    ("P2", "AuthorityRefProtected", "AuthorityRefProtectionBound", ["P1"]),
    ("P3", "RequiredWorkflowCapabilityVerified", "RequiredWorkflowCapabilityObserved", ["P2"]),
    ("P4", "RulesetEvaluateConfigured", "RulesetEvaluateConfigurationBound", ["P3"]),
    ("P5", "RulesetEvaluateExecutionQualified", "RulesetEvaluateExecutionBound", ["P4"]),
    ("P6", "RulesetActive", "RulesetActiveConfigurationBound", ["P5"]),
]
EXPECTED_CASES = {
    "KnownRelevantClosure",
    "ProvenIrrelevantTypedNoCiRequired",
    "UnknownPathFullRequiredFanout",
    "RenamePreservesPreviousRelevantPath",
    "InformationalFinanceObservationNonGating",
    "RequiredProductRedBlocks",
    "StaleSubjectCannotQualifyNewerHead",
    "MergeGroupFullRequiredFanout",
    "AuthorityInputUncertaintyDeniesPass",
}
EXPECTED_EPOCH_FIELDS = {
    "authority_source_sha",
    "workflow_blob",
    "manifest_blob",
    "ruleset_id",
    "ruleset_configuration_commitment",
    "enforcement_mode",
}


class ContractError(ValueError):
    pass


def require(ok: bool, msg: str) -> None:
    if not ok:
        raise ContractError(msg)


def sha40(value: Any, label: str) -> str:
    require(isinstance(value, str) and SHA_RE.fullmatch(value) is not None,
            f"{label} must be lowercase 40-hex")
    return value


def validate(c: Any) -> dict[str, Any]:
    require(isinstance(c, dict), "contract root must be object")
    require(c.get("contract_id") == CONTRACT_ID, "contract identity drift")
    require(c.get("version") == 1, "version drift")
    require(c.get("issue") == 1705, "issue binding drift")
    require(c.get("authority") == "PromotionGovernanceOnly", "authority drift")

    source = c.get("source")
    require(isinstance(source, dict), "source missing")
    for key in ("executable_authority_sha", "static_validator_sha",
                "qualification_head_sha", "qualification_base_sha"):
        sha40(source.get(key), f"source.{key}")
    require(source.get("qualification_pr") == 1704, "qualification PR drift")
    require(source.get("qualification_claim") == "RulesetAuthoritySourceCapsuleBound",
            "qualification claim drift")

    target = c.get("promotion_target")
    require(isinstance(target, dict), "promotion_target missing")
    require(target.get("repository") == "Luminous-Dynamics/mycelix", "repository drift")
    require(target.get("ref") == "refs/heads/ci-authority/v1", "promotion ref drift")
    sha40(target.get("initial_source_sha"), "promotion_target.initial_source_sha")
    require(target.get("initial_source_sha") == source.get("static_validator_sha"),
            "promotion target must be exact qualified source root")
    require(target.get("required_workflow_path") == ".github/workflows/ruleset-generic-ci-authority.yml",
            "workflow path drift")

    obs = c.get("initial_observation")
    require(isinstance(obs, dict), "initial observation missing")
    require(obs.get("authority_ref_exists") is False, "initial authority ref must be absent")
    require(obs.get("repository_ruleset_count") == 0, "initial repository ruleset count drift")
    require("organization" in str(obs.get("observation_scope", "")),
            "initial observation must disclaim organization inference")

    states = c.get("states")
    require(isinstance(states, list) and len(states) == len(EXPECTED_STATES),
            "state census drift")
    ids = []
    claims = []
    for state, expected in zip(states, EXPECTED_STATES):
        require(isinstance(state, dict), "state must be object")
        sid, name, claim, reqs = expected
        require(state.get("id") == sid, f"state id drift {sid}")
        require(state.get("name") == name, f"state name drift {sid}")
        require(state.get("claim") == claim, f"state claim drift {sid}")
        require(state.get("requires") == reqs, f"state prerequisite drift {sid}")
        require(isinstance(state.get("mutation_allowed"), bool), f"mutation flag invalid {sid}")
        ids.append(sid); claims.append(claim)
    require(len(ids) == len(set(ids)), "duplicate state ids")
    require(len(claims) == len(set(claims)), "duplicate claim ceilings")

    protection = c.get("protection_requirements")
    require(isinstance(protection, dict), "protection requirements missing")
    for key in (
        "block_deletion", "block_force_push", "block_unqualified_direct_update",
        "bypass_identities_must_be_enumerated", "bypass_set_must_be_minimized",
        "qualified_successor_required_for_update",
    ):
        require(protection.get(key) is True, f"protection property not required: {key}")

    workflow = c.get("required_workflow_configuration")
    require(isinstance(workflow, dict), "required workflow config missing")
    require(workflow.get("source_repository") == "Luminous-Dynamics/mycelix", "workflow source repo drift")
    require(workflow.get("source_branch") == "ci-authority/v1", "workflow source branch drift")
    require(workflow.get("workflow_path") == ".github/workflows/ruleset-generic-ci-authority.yml",
            "workflow path drift")
    require(workflow.get("evaluate_before_active") is True, "Evaluate-first requirement missing")
    require(workflow.get("merge_group_required") is True, "merge_group requirement missing")
    require(workflow.get("pull_request_required") is True, "pull_request requirement missing")
    require(workflow.get("pull_request_target_forbidden") is True, "pull_request_target must be forbidden")

    cases = c.get("evaluate_execution_cases")
    require(isinstance(cases, list) and set(cases) == EXPECTED_CASES and len(cases) == len(EXPECTED_CASES),
            "Evaluate execution case census drift")

    activation = c.get("activation")
    require(isinstance(activation, dict), "activation contract missing")
    require(activation.get("requires_exact_evaluate_configuration_identity") is True,
            "exact Evaluate identity not required")
    require(activation.get("requires_all_prior_states") == ["P0","P1","P2","P3","P4","P5"],
            "Active prerequisite set drift")
    require(activation.get("configuration_drift_requires_requalification") is True,
            "configuration drift must requalify")

    rollback = c.get("rollback")
    require(isinstance(rollback, dict), "rollback contract missing")
    for key in (
        "deactivate_before_source_change", "force_move_while_active_forbidden",
        "rollback_target_must_be_previously_qualified", "record_prior_active_sha",
        "record_replacement_sha", "record_ruleset_transition", "record_reason",
        "record_operator_identity", "evaluate_before_reactivation",
    ):
        require(rollback.get(key) is True, f"rollback invariant missing: {key}")

    epoch = c.get("authority_epoch")
    require(isinstance(epoch, dict), "authority_epoch missing")
    fields = epoch.get("required_fields")
    require(isinstance(fields, list) and set(fields) == EXPECTED_EPOCH_FIELDS
            and len(fields) == len(EXPECTED_EPOCH_FIELDS), "authority epoch field drift")
    require(epoch.get("new_source_sha_starts_new_epoch") is True, "source change must start epoch")
    require(epoch.get("ruleset_configuration_change_starts_new_epoch") is True,
            "ruleset config change must start epoch")
    require(epoch.get("cross_epoch_evidence_mixing_forbidden") is True,
            "cross-epoch evidence mixing must be forbidden")

    return {
        "validator_id": VALIDATOR_ID,
        "valid": True,
        "contract_id": CONTRACT_ID,
        "state_count": len(states),
        "evaluate_case_count": len(cases),
        "epoch_field_count": len(fields),
        "claim_ceiling": "AuthorityPromotionContractBound",
        "grants_promotion": False,
        "grants_ruleset_activation": False,
        "grants_product_qualification": False,
    }


def must_fail(c: dict[str, Any]) -> None:
    try:
        validate(c)
    except ContractError:
        return
    raise AssertionError("mutated unsafe contract unexpectedly validated")


def self_test(c: dict[str, Any]) -> None:
    validate(c)

    x = copy.deepcopy(c)
    x["states"][1]["requires"] = []
    must_fail(x)

    x = copy.deepcopy(c)
    x["activation"]["requires_all_prior_states"] = ["P0","P1","P2","P3","P4"]
    must_fail(x)

    x = copy.deepcopy(c)
    x["rollback"]["deactivate_before_source_change"] = False
    must_fail(x)

    x = copy.deepcopy(c)
    x["required_workflow_configuration"]["source_branch"] = "main"
    must_fail(x)

    x = copy.deepcopy(c)
    x["authority_epoch"]["required_fields"].remove("ruleset_configuration_commitment")
    must_fail(x)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--contract", default="docs/ci/ruleset_authority_promotion_v1.json")
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    try:
        contract = json.loads(Path(args.contract).read_text(encoding="utf-8"))
        if args.self_test:
            self_test(contract)
        result = validate(contract)
        if args.self_test:
            result["self_test"] = "PASS"
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, json.JSONDecodeError, ContractError, AssertionError) as exc:
        print(json.dumps({
            "validator_id": VALIDATOR_ID,
            "valid": False,
            "reason": str(exc),
            "claim_ceiling": "AuthorityPromotionContractBound",
            "grants_promotion": False,
            "grants_ruleset_activation": False,
            "grants_product_qualification": False,
        }, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
