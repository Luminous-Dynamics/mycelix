#!/usr/bin/env python3
"""Fail-closed evaluator for live GitHub repository-governance observations."""

from __future__ import annotations

import argparse
import base64
import copy
import hashlib
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


def validate_bound_raw_payload(observation: Any, name: str) -> Any:
    raw_field = f"{name}_payload_base64"
    digest_field = f"{name}_payload_sha256"
    encoded = observation.get(raw_field)
    expected_digest = sha256(observation.get(digest_field), digest_field)
    require(isinstance(encoded, str) and encoded != "", f"{raw_field} missing")
    try:
        raw = base64.b64decode(encoded, validate=True)
        parsed = json.loads(raw.decode("utf-8"))
    except (ValueError, TypeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f"{raw_field} is not valid UTF-8 JSON") from exc
    actual_digest = hashlib.sha256(raw).hexdigest()
    require(actual_digest == expected_digest, f"{digest_field} does not match {raw_field}")
    return parsed


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
    branch_raw = validate_bound_raw_payload(observation, "branch")
    rulesets_raw = validate_bound_raw_payload(observation, "rulesets")
    require(isinstance(branch_raw, dict), "branch raw payload must be an object")
    require(isinstance(rulesets_raw, list), "rulesets raw payload must be a list")
    require(
        observation.get("branch") == {
            "name": branch_raw.get("name"),
            "protected": branch_raw.get("protected"),
        },
        "normalized branch observation does not match raw branch payload",
    )
    require(
        observation.get("rulesets", {}).get("entries") == rulesets_raw,
        "normalized ruleset observation does not match raw rulesets payload",
    )


def _ruleset_targets_main(entry: Any) -> bool:
    if not isinstance(entry, dict):
        return False
    if entry.get("target") != "branch" or entry.get("enforcement") != "active":
        return False
    conditions = entry.get("conditions")
    if not isinstance(conditions, dict):
        return False
    ref_name = conditions.get("ref_name")
    if not isinstance(ref_name, dict):
        return False
    includes = ref_name.get("include")
    if not isinstance(includes, list):
        return False
    return TARGET_REF in includes or "~DEFAULT_BRANCH" in includes or "~ALL" in includes


def _evaluate_rulesets(ruleset_entries: list[Any]) -> tuple[str, list[str]]:
    targeted: list[dict[str, Any]] = []
    for entry in ruleset_entries:
        if _ruleset_targets_main(entry):
            targeted.append(entry)

    if not targeted:
        return "MISMATCH", ["no_active_main_targeting_ruleset"]

    failures: list[str] = []
    for index, entry in enumerate(targeted):
        prefix = f"ruleset[{index}]"
        rules = entry.get("rules")
        if not isinstance(rules, list):
            return "UNVERIFIED", [f"{prefix}_rules_not_enumerated"]

        bypass = entry.get("bypass_actors")
        if not isinstance(bypass, list):
            return "UNVERIFIED", [f"{prefix}_bypass_actors_not_enumerated"]
        if len(bypass) != 0:
            failures.append(f"{prefix}_bypass_set_not_minimized")

        pull_rules = [
            rule for rule in rules
            if isinstance(rule, dict) and rule.get("type") == "pull_request"
        ]
        if not pull_rules:
            failures.append(f"{prefix}_pull_request_required")
            continue

        merged_parameters: dict[str, Any] = {}
        for rule in pull_rules:
            parameters = rule.get("parameters")
            if isinstance(parameters, dict):
                merged_parameters.update(parameters)
        if merged_parameters.get("required_approving_review_count") != 1:
            failures.append(f"{prefix}_required_approving_review_count")
        for key in (
            "dismiss_stale_reviews_on_push",
            "require_last_push_approval",
            "required_review_thread_resolution",
        ):
            if merged_parameters.get(key) is not True:
                failures.append(f"{prefix}_{key}")

        rule_types = {
            rule.get("type")
            for rule in rules
            if isinstance(rule, dict)
        }
        if "non_fast_forward" not in rule_types:
            failures.append(f"{prefix}_block_force_push")
        if "deletion" not in rule_types:
            failures.append(f"{prefix}_block_deletion")

    return ("MISMATCH" if failures else "VERIFIED"), failures


def evaluate(policy: Any, observation: Any) -> dict[str, Any]:
    validate_policy(policy)
    validate_observation_shape(observation)
    expected_policy_digest = hashlib.sha256(
        json.dumps(policy, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    require(
        observation.get("policy_sha256") == expected_policy_digest,
        "policy_sha256 does not match the evaluator policy",
    )

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
    admin_source = admin.get("source")
    require(
        admin_source in {"repository_administration_secret", "github_token"},
        "admin_observation source invalid",
    )
    if admin_visibility == "verified":
        require(
            admin_source == "repository_administration_secret",
            "verified administration observation requires an external administration credential",
        )

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

    ruleset_state, ruleset_mismatches = _evaluate_rulesets(ruleset_entries)

    branch_state = "UNAVAILABLE"
    branch_mismatches: list[str] = []
    if protection_status == 200:
        protection = admin.get("protection")
        if not isinstance(protection, dict):
            branch_state = "UNVERIFIED"
            branch_mismatches = ["verified_admin_protection_missing"]
        else:
            expected = policy["required_controls"]
            if protection.get("pull_request_required") is not True:
                branch_mismatches.append("pull_request_required")
            if protection.get("required_approving_review_count") != expected["required_approving_review_count"]:
                branch_mismatches.append("required_approving_review_count")
            for key in (
                "dismiss_stale_reviews_on_push",
                "require_last_push_approval",
                "required_conversation_resolution",
                "block_force_push",
                "block_deletion",
            ):
                if protection.get(key) is not True:
                    branch_mismatches.append(key)

            bypass = protection.get("bypass_actors")
            if not isinstance(bypass, list):
                branch_state = "UNVERIFIED"
                branch_mismatches.append("bypass_actors_not_enumerated")
            else:
                if expected["bypass_set_must_be_minimized"] and len(bypass) != 0:
                    branch_mismatches.append("bypass_set_not_minimized")
                if len(bypass) != len({json.dumps(x, sort_keys=True) for x in bypass}):
                    branch_mismatches.append("duplicate_bypass_identity")
            if branch_state != "UNVERIFIED":
                branch_state = "MISMATCH" if branch_mismatches else "VERIFIED"
    elif protection_status == 404:
        branch_state = "UNAVAILABLE"
    else:
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "UNVERIFIED",
            "reason": f"unexpected_branch_protection_http_status:{protection_status}",
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": False,
            "grants_trusted_verifier_root": False,
        }

    if branch_state == "VERIFIED" or ruleset_state == "VERIFIED":
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": True,
            "governance_state": "VERIFIED",
            "reason": "all_required_controls_observed_in_an_acceptable_control_plane",
            "mismatches": [],
            "claim_ceiling": "RepositoryGovernanceVerified",
            "authoritative_admin_observation": True,
            "grants_trusted_verifier_root": True,
        }

    if branch_state == "UNVERIFIED" or ruleset_state == "UNVERIFIED":
        mismatches = branch_mismatches + ruleset_mismatches
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "UNVERIFIED",
            "reason": "acceptable_control_plane_observation_incomplete",
            "mismatches": mismatches,
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": True,
            "grants_trusted_verifier_root": False,
        }

    mismatches = branch_mismatches + ruleset_mismatches
    return {
        "evaluator_id": EVALUATOR_ID,
        "valid": False,
        "governance_state": "MISMATCH",
        "reason": "required_governance_controls_mismatch",
        "mismatches": mismatches,
        "claim_ceiling": "RepositoryGovernanceObservationOnly",
        "authoritative_admin_observation": True,
        "grants_trusted_verifier_root": False,
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
    ruleset_entry = {
        "id": 1,
        "target": "branch",
        "enforcement": "active",
        "conditions": {"ref_name": {"include": [TARGET_REF], "exclude": []}},
        "bypass_actors": [],
        "rules": [
            {
                "type": "pull_request",
                "parameters": {
                    "dismiss_stale_reviews_on_push": True,
                    "require_last_push_approval": True,
                    "required_approving_review_count": 1,
                    "required_review_thread_resolution": True,
                },
            },
            {"type": "non_fast_forward"},
            {"type": "deletion"},
        ],
    }
    branch_payload = {"name": "main", "protected": True}
    rulesets_payload = [ruleset_entry]
    branch_raw = json.dumps(branch_payload, separators=(",", ":"), sort_keys=True).encode()
    rulesets_raw = json.dumps(rulesets_payload, separators=(",", ":"), sort_keys=True).encode()
    return {
        "schema": SCHEMA,
        "version": 1,
        "repository": REPOSITORY,
        "repository_id": REPOSITORY_ID,
        "target_ref": TARGET_REF,
        "observed_at_utc": "2026-10-07T00:00:00Z",
        "policy_sha256": hashlib.sha256(
            json.dumps(fixture_policy(), sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        "branch_payload_base64": base64.b64encode(branch_raw).decode(),
        "branch_payload_sha256": hashlib.sha256(branch_raw).hexdigest(),
        "rulesets_payload_base64": base64.b64encode(rulesets_raw).decode(),
        "rulesets_payload_sha256": hashlib.sha256(rulesets_raw).hexdigest(),
        "branch": branch_payload,
        "rulesets": {"entries": rulesets_payload},
        "branch_protection_api": {"http_status": protection_status},
        "admin_observation": {
            "status": admin_status,
            "source": (
                "repository_administration_secret"
                if admin_status == "verified"
                else "github_token"
            ),
            "protection": {
                "pull_request_required": True,
                "required_approving_review_count": 1,
                "dismiss_stale_reviews_on_push": True,
                "require_last_push_approval": True,
                "required_conversation_resolution": True,
                "block_force_push": True,
                "block_deletion": True,
                "bypass_actors": [],
            },
        },
    }


def self_test() -> None:
    policy = fixture_policy()
    positive = evaluate(policy, fixture_observation())
    assert positive["governance_state"] == "VERIFIED"
    assert positive["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation())
    x["branch"]["protected"] = False
    x["rulesets"]["entries"] = []
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(protection_status=403))
    result = evaluate(policy, x)
    assert result["governance_state"] == "UNVERIFIED"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(protection_status=404))
    result = evaluate(policy, x)
    assert result["governance_state"] == "VERIFIED"
    assert result["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation())
    x["policy_sha256"] = "f" * 64
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("policy digest substitution must be rejected")

    x = copy.deepcopy(fixture_observation())
    x["branch_payload_base64"] = base64.b64encode(
        b'{"name":"attacker","protected":true}'
    ).decode()
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("raw branch payload substitution must be rejected")

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
                "governance_state": "NOT_RUN",
                "synthetic_positive_case": "VERIFIED",
                "grants_trusted_verifier_root": False,
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
