#!/usr/bin/env python3
"""Fail-closed evaluator for live GitHub repository-governance observations."""

from __future__ import annotations

import argparse
import copy
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

EVALUATOR_ID = "mycelix-repository-governance-evaluator-v1"
SCHEMA = "MYCELIX-REPOSITORY-GOVERNANCE-OBSERVATION-V1"
POLICY_SCHEMA = "MYCELIX-REPOSITORY-GOVERNANCE-POLICY-V1"
REPOSITORY = "Luminous-Dynamics/mycelix"
REPOSITORY_ID = 1176351975
TARGET_REF = "refs/heads/main"

SHA256 = re.compile(r"^[0-9a-f]{64}$")


class EvidenceError(ValueError):
    pass


def require(ok: bool, message: str) -> None:
    if not ok:
        raise EvidenceError(message)


def parse_ts(value: Any) -> datetime:
    require(isinstance(value, str), "observed_at_utc must be a string")
    try:
        dt = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise EvidenceError(f"invalid observed_at_utc: {exc}") from exc
    require(dt.tzinfo is not None, "observed_at_utc must be timezone-aware")
    return dt.astimezone(timezone.utc)


def sha256(value: Any, label: str) -> str:
    require(isinstance(value, str) and SHA256.fullmatch(value) is not None,
            f"{label} must be lowercase sha256")
    return value


def validate_policy(policy: Any) -> None:
    require(isinstance(policy, dict), "policy must be an object")
    require(policy.get("schema") == POLICY_SCHEMA, "policy schema drift")
    require(policy.get("version") == 1, "policy version drift")
    require(policy.get("repository") == REPOSITORY, "policy repository drift")
    require(policy.get("repository_id") == REPOSITORY_ID, "policy repository id drift")
    require(policy.get("target_ref") == TARGET_REF, "policy target ref drift")
    required = policy.get("required_controls")
    require(isinstance(required, dict), "required_controls missing")
    for key in (
        "pull_request_required",
        "dismiss_stale_reviews_on_push",
        "require_last_push_approval",
        "required_review_thread_resolution",
        "block_force_push",
        "block_deletion",
        "bypass_actors_must_be_enumerated",
        "bypass_set_must_be_minimized",
    ):
        require(required.get(key) is True, f"required control drift: {key}")
    require(required.get("required_approving_review_count") == 1,
            "required review count drift")


def validate_observation_shape(observation: Any) -> None:
    require(isinstance(observation, dict), "observation must be an object")
    require(observation.get("schema") == SCHEMA, "observation schema drift")
    require(observation.get("version") == 1, "observation version drift")
    require(observation.get("repository") == REPOSITORY, "observation repository drift")
    require(observation.get("repository_id") == REPOSITORY_ID, "observation repository id drift")
    require(observation.get("target_ref") == TARGET_REF, "observation target ref drift")
    parse_ts(observation.get("observed_at_utc"))
    sha256(observation.get("policy_sha256"), "policy_sha256")
    for name in ("branch_payload", "rulesets_payload"):
        sha256(observation.get(f"{name}_sha256"), f"{name}_sha256")


def evaluate(policy: Any, observation: Any) -> dict[str, Any]:
    validate_policy(policy)
    validate_observation_shape(observation)

    branch = observation.get("branch")
    require(isinstance(branch, dict), "branch observation missing")
    require(branch.get("name") == "main", "branch name drift")
    require(branch.get("target_ref") == TARGET_REF, "branch target drift")
    branch_protected = branch.get("protected")
    require(isinstance(branch_protected, bool), "branch protected must be boolean")

    rulesets = observation.get("rulesets")
    require(isinstance(rulesets, dict), "rulesets observation missing")
    ruleset_entries = rulesets.get("entries")
    require(isinstance(ruleset_entries, list), "rulesets entries must be list")

    protection_api = observation.get("branch_protection_api")
    require(isinstance(protection_api, dict), "branch protection API observation missing")
    protection_status = protection_api.get("http_status")
    require(isinstance(protection_status, int), "branch protection http_status must be integer")

    admin = observation.get("admin_observation")
    require(isinstance(admin, dict), "admin_observation missing")
    admin_visibility = admin.get("status")
    require(admin_visibility in {"verified", "unverified", "not_run"},
            "admin_observation status invalid")

    if protection_status in {401, 403}:
        admin_visibility = "unverified"

    if admin_visibility != "verified":
        state = "MISMATCH" if branch_protected is False and len(ruleset_entries) == 0 else "UNVERIFIED"
        reason = (
            "github_branch_protection_admin_observation_unavailable"
            if state == "UNVERIFIED"
            else "target_branch_is_unprotected_and_no_ruleset_is_observed"
        )
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": state != "MISMATCH",
            "governance_state": state,
            "reason": reason,
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": False,
            "grants_trusted_verifier_root": False,
        }

    if protection_status != 200:
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "UNVERIFIED",
            "reason": f"unexpected_branch_protection_http_status:{protection_status}",
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": False,
            "grants_trusted_verifier_root": False,
        }

    protection = admin.get("protection")
    require(isinstance(protection, dict), "verified admin protection missing")
    failures: list[str] = []

    expected = policy["required_controls"]
    if protection.get("pull_request_required") is not True:
        failures.append("pull_request_required")
    if protection.get("required_approving_review_count") != expected["required_approving_review_count"]:
        failures.append("required_approving_review_count")
    for key in (
        "dismiss_stale_reviews_on_push",
        "require_last_push_approval",
        "required_review_thread_resolution",
        "block_force_push",
        "block_deletion",
    ):
        if protection.get(key) is not True:
            failures.append(key)

    bypass = protection.get("bypass_actors")
    if not isinstance(bypass, list):
        failures.append("bypass_actors_not_enumerated")
    else:
        if expected["bypass_set_must_be_minimized"] and len(bypass) != 0:
            failures.append("bypass_set_not_minimized")
        if len(bypass) != len({json.dumps(x, sort_keys=True) for x in bypass}):
            failures.append("duplicate_bypass_identity")

    if failures:
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "MISMATCH",
            "reason": "required_governance_controls_mismatch",
            "mismatches": failures,
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": True,
            "grants_trusted_verifier_root": False,
        }

    return {
        "evaluator_id": EVALUATOR_ID,
        "valid": True,
        "governance_state": "VERIFIED",
        "reason": "all_required_repository_governance_controls_observed",
        "claim_ceiling": "RepositoryGovernanceVerified",
        "authoritative_admin_observation": True,
        "grants_trusted_verifier_root": True,
    }


def fixture_policy() -> dict[str, Any]:
    return {
        "schema": POLICY_SCHEMA,
        "version": 1,
        "repository": REPOSITORY,
        "repository_id": REPOSITORY_ID,
        "target_ref": TARGET_REF,
        "required_controls": {
            "pull_request_required": True,
            "required_approving_review_count": 1,
            "dismiss_stale_reviews_on_push": True,
            "require_last_push_approval": True,
            "required_review_thread_resolution": True,
            "block_force_push": True,
            "block_deletion": True,
            "bypass_actors_must_be_enumerated": True,
            "bypass_set_must_be_minimized": True,
        },
    }


def fixture_observation(protection_status: int = 200, admin_status: str = "verified") -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "version": 1,
        "repository": REPOSITORY,
        "repository_id": REPOSITORY_ID,
        "target_ref": TARGET_REF,
        "observed_at_utc": "2026-10-07T00:00:00Z",
        "policy_sha256": "1" * 64,
        "branch_payload_sha256": "2" * 64,
        "rulesets_payload_sha256": "3" * 64,
        "branch": {"name": "main", "target_ref": TARGET_REF, "protected": True},
        "rulesets": {"entries": [{"id": 1, "enforcement": "active"}]},
        "branch_protection_api": {"http_status": protection_status},
        "admin_observation": {
            "status": admin_status,
            "protection": {
                "pull_request_required": True,
                "required_approving_review_count": 1,
                "dismiss_stale_reviews_on_push": True,
                "require_last_push_approval": True,
                "required_review_thread_resolution": True,
                "block_force_push": True,
                "block_deletion": True,
                "bypass_actors": [],
            },
        },
    }


def self_test() -> None:
    policy = fixture_policy()
    evaluate(policy, fixture_observation())

    x = copy.deepcopy(fixture_observation())
    x["branch"]["protected"] = False
    x["rulesets"]["entries"] = []
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(protection_status=403))
    result = evaluate(policy, x)
    assert result["governance_state"] in {"UNVERIFIED", "MISMATCH"}
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation())
    x["admin_observation"]["protection"]["block_force_push"] = False
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"

    x = copy.deepcopy(fixture_observation())
    x["admin_observation"]["protection"]["bypass_actors"] = [{"actor_type": "RepositoryRole"}]
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", default="docs/ci/repository_governance_policy_v1.json")
    parser.add_argument("--observation")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        policy = json.loads(Path(args.policy).read_text(encoding="utf-8"))
        if args.self_test:
            self_test()
            print(json.dumps({
                "evaluator_id": EVALUATOR_ID,
                "self_test": "PASS",
                "governance_state": "VERIFIED",
                "grants_trusted_verifier_root": True,
            }, sort_keys=True))
            return 0

        require(args.observation is not None, "--observation is required unless --self-test")
        observation = json.loads(Path(args.observation).read_text(encoding="utf-8"))
        result = evaluate(policy, observation)
        print(json.dumps(result, sort_keys=True))
        return 0 if result["valid"] else 2
    except (OSError, json.JSONDecodeError, EvidenceError, AssertionError) as exc:
        print(json.dumps({
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "NOT_RUN",
            "reason": str(exc),
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "grants_trusted_verifier_root": False,
        }, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
