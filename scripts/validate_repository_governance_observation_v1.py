#!/usr/bin/env python3
"""Fail-closed evaluator for live GitHub repository-governance observations."""

from __future__ import annotations

import argparse
import base64
import copy
import fnmatch
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
    repository_raw = validate_bound_raw_payload(observation, "repository")
    branch_raw = validate_bound_raw_payload(observation, "branch")
    rulesets_index_raw = validate_bound_raw_payload(observation, "rulesets_index")
    rulesets_raw = validate_bound_raw_payload(observation, "rulesets")
    effective_rules_raw = validate_bound_raw_payload(observation, "effective_rules")
    protection_raw = validate_bound_raw_payload(observation, "branch_protection")
    require(isinstance(repository_raw, dict), "repository raw payload must be an object")
    require(repository_raw.get("id") == REPOSITORY_ID, "repository raw id drift")
    require(repository_raw.get("full_name") == REPOSITORY, "repository raw full_name drift")
    require(isinstance(repository_raw.get("default_branch"), str), "repository raw default_branch missing")
    require(
        observation.get("default_branch") == repository_raw.get("default_branch"),
        "normalized default_branch observation does not match raw repository payload",
    )
    require(isinstance(branch_raw, dict), "branch raw payload must be an object")
    require(isinstance(rulesets_index_raw, list), "rulesets index raw payload must be a list")
    require(isinstance(rulesets_raw, list), "rulesets raw payload must be a list")
    require(isinstance(effective_rules_raw, list), "effective rules raw payload must be a list")
    index_ids = [entry.get("id") for entry in rulesets_index_raw if isinstance(entry, dict)]
    full_ids = [entry.get("id") for entry in rulesets_raw if isinstance(entry, dict)]
    require(len(index_ids) == len(rulesets_index_raw), "rulesets index entry is not an object")
    require(len(full_ids) == len(rulesets_raw), "full ruleset entry is not an object")
    require(all(isinstance(value, int) for value in index_ids), "rulesets index id missing")
    require(all(isinstance(value, int) for value in full_ids), "full ruleset id missing")
    require(len(index_ids) == len(set(index_ids)), "duplicate ruleset ids in index")
    require(len(full_ids) == len(set(full_ids)), "duplicate ruleset ids in full rulesets")
    require(set(index_ids) == set(full_ids), "ruleset index/full object id sets differ")
    index_by_id = {entry["id"]: entry for entry in rulesets_index_raw}
    for entry in rulesets_raw:
        summary = index_by_id[entry["id"]]
        for key in ("id", "name", "source_type", "source", "enforcement", "updated_at"):
            require(
                entry.get(key) == summary.get(key),
                f"ruleset full object diverges from index summary: {entry['id']}:{key}",
            )
    require(
        observation.get("branch", {}).get("name") == branch_raw.get("name")
        and observation.get("branch", {}).get("protected") == branch_raw.get("protected"),
        "normalized branch observation does not match raw branch payload",
    )
    require(
        observation.get("rulesets", {}).get("entries") == rulesets_raw,
        "normalized ruleset observation does not match raw rulesets payload",
    )
    require(
        observation.get("effective_rules", {}).get("entries") == effective_rules_raw,
        "normalized effective rules observation does not match raw effective rules payload",
    )
    protection_api = observation.get("branch_protection_api")
    require(isinstance(protection_api, dict), "branch protection API observation missing")
    protection_status = protection_api.get("http_status")
    require(isinstance(protection_status, int), "branch protection http_status must be integer")
    if protection_status == 200:
        normalized_protection = _normalize_branch_protection(protection_raw)
        require(
            observation.get("admin_observation", {}).get("protection") == normalized_protection,
            "normalized admin protection does not match raw branch protection payload",
        )


def _ref_pattern_matches_main(pattern: Any, default_branch: str) -> bool:
    if not isinstance(pattern, str) or pattern == "":
        return False
    if pattern == "~ALL":
        return True
    if pattern == "~DEFAULT_BRANCH":
        return default_branch == "main"
    return (
        fnmatch.fnmatchcase(TARGET_REF, pattern)
        or fnmatch.fnmatchcase("main", pattern)
    )


def _ruleset_targets_main(entry: Any, default_branch: str) -> bool:
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
    excludes = ref_name.get("exclude")
    if not isinstance(includes, list) or not isinstance(excludes, list):
        return False
    return (
        any(_ref_pattern_matches_main(pattern, default_branch) for pattern in includes)
        and not any(_ref_pattern_matches_main(pattern, default_branch) for pattern in excludes)
    )


def _validate_ruleset_bypass_actors(entry: dict[str, Any], prefix: str) -> str | None:
    bypass = entry.get("bypass_actors")
    if not isinstance(bypass, list):
        return f"{prefix}_bypass_actors_not_enumerated"
    allowed_types = {
        "Integration",
        "OrganizationAdmin",
        "RepositoryRole",
        "Team",
        "DeployKey",
        "EnterpriseOwner",
        "EnterpriseRole",
        "User",
    }
    allowed_modes = {"always", "pull_request", "exempt"}
    for actor in bypass:
        if not isinstance(actor, dict):
            return f"{prefix}_bypass_actor_not_enumerated"
        actor_type = actor.get("actor_type")
        if actor_type not in allowed_types:
            return f"{prefix}_bypass_actor_type_invalid"
        mode = actor.get("bypass_mode", "always")
        if mode not in allowed_modes:
            return f"{prefix}_bypass_mode_invalid"
        actor_id = actor.get("actor_id")
        if actor_type in {"Integration", "RepositoryRole", "Team", "User"}:
            if not isinstance(actor_id, int) or isinstance(actor_id, bool):
                return f"{prefix}_bypass_actor_id_invalid"
        elif actor_type == "DeployKey":
            if actor_id is not None:
                return f"{prefix}_deploy_key_actor_id_invalid"
        elif actor_id is not None and not isinstance(actor_id, int):
            return f"{prefix}_administrative_actor_id_invalid"
    return None


def _evaluate_rulesets(
    ruleset_entries: list[Any], default_branch: str
) -> tuple[str, list[str]]:
    for index, entry in enumerate(ruleset_entries):
        if not isinstance(entry, dict):
            return "UNVERIFIED", [f"ruleset[{index}]_entry_not_enumerated"]
        if entry.get("target") != "branch" or entry.get("enforcement") != "active":
            continue
        conditions = entry.get("conditions")
        if not isinstance(conditions, dict):
            return "UNVERIFIED", [f"ruleset[{index}]_conditions_not_enumerated"]
        ref_name = conditions.get("ref_name")
        if not isinstance(ref_name, dict):
            return "UNVERIFIED", [f"ruleset[{index}]_ref_name_conditions_not_enumerated"]
        includes = ref_name.get("include")
        excludes = ref_name.get("exclude")
        if not isinstance(includes, list) or not isinstance(excludes, list):
            return "UNVERIFIED", [f"ruleset[{index}]_target_patterns_not_enumerated"]
        if "~DEFAULT_BRANCH" in includes or "~DEFAULT_BRANCH" in excludes:
            return "UNVERIFIED", [f"ruleset[{index}]_default_branch_target_unbound"]

    targeted: list[dict[str, Any]] = []
    for entry in ruleset_entries:
        if _ruleset_targets_main(entry):
            targeted.append(entry)

    if not targeted:
        return "ABSENT", []

    failures: list[str] = []
    all_rule_types: set[Any] = set()
    approval_counts: list[int] = []
    merged_parameters: dict[str, bool] = {}

    for index, entry in enumerate(targeted):
        prefix = f"ruleset[{index}]"
        rules = entry.get("rules")
        if not isinstance(rules, list):
            return "UNVERIFIED", [f"{prefix}_rules_not_enumerated"]

        bypass_error = _validate_ruleset_bypass_actors(entry, prefix)
        if bypass_error is not None:
            return "UNVERIFIED", [bypass_error]
        bypass = entry["bypass_actors"]
        if bypass:
            failures.append(f"{prefix}_bypass_set_not_minimized")

        for rule in rules:
            if not isinstance(rule, dict):
                continue
            all_rule_types.add(rule.get("type"))
            if rule.get("type") != "pull_request":
                continue
            parameters = rule.get("parameters")
            if not isinstance(parameters, dict):
                return "UNVERIFIED", [f"{prefix}_pull_request_parameters_not_enumerated"]
            count = parameters.get("required_approving_review_count")
            if isinstance(count, int):
                approval_counts.append(count)
            for key in (
                "dismiss_stale_reviews_on_push",
                "require_last_push_approval",
                "required_review_thread_resolution",
            ):
                if parameters.get(key) is True:
                    merged_parameters[key] = True

    if not approval_counts:
        failures.append("required_approving_review_count_not_observed")
    elif max(approval_counts) != 1:
        failures.append("required_approving_review_count")

    for key in (
        "dismiss_stale_reviews_on_push",
        "require_last_push_approval",
        "required_review_thread_resolution",
    ):
        if merged_parameters.get(key) is not True:
            failures.append(key)

    if "pull_request" not in all_rule_types:
        failures.append("pull_request_required")
    if "non_fast_forward" not in all_rule_types:
        failures.append("block_force_push")
    if "deletion" not in all_rule_types:
        failures.append("block_deletion")

    return ("MISMATCH" if failures else "VERIFIED"), failures


def _evaluate_effective_rules(
    effective_rules: list[Any],
) -> tuple[str, list[str]]:
    required_types = {
        "pull_request",
        "non_fast_forward",
        "deletion",
    }
    failures: list[str] = []
    observed_types: set[Any] = set()
    approval_counts: list[int] = []
    merged_parameters: dict[str, bool] = {}

    for index, rule in enumerate(effective_rules):
        if not isinstance(rule, dict):
            return "UNVERIFIED", [f"effective_rule[{index}]_not_enumerated"]
        rule_type = rule.get("type")
        observed_types.add(rule_type)
        if rule_type != "pull_request":
            continue
        parameters = rule.get("parameters")
        if not isinstance(parameters, dict):
            return "UNVERIFIED", [f"effective_rule[{index}]_pull_request_parameters_not_enumerated"]
        count = parameters.get("required_approving_review_count")
        if isinstance(count, int):
            approval_counts.append(count)
        for key in (
            "dismiss_stale_reviews_on_push",
            "require_last_push_approval",
            "required_review_thread_resolution",
        ):
            if parameters.get(key) is True:
                merged_parameters[key] = True

    missing_types = sorted(required_types - observed_types)
    failures.extend(f"effective_rule_missing:{rule_type}" for rule_type in missing_types)
    if not approval_counts:
        failures.append("effective_rule_required_approving_review_count_not_observed")
    elif max(approval_counts) != 1:
        failures.append("effective_rule_required_approving_review_count")
    for key in (
        "dismiss_stale_reviews_on_push",
        "require_last_push_approval",
        "required_review_thread_resolution",
    ):
        if merged_parameters.get(key) is not True:
            failures.append(f"effective_rule_missing:{key}")

    return ("MISMATCH" if failures else "VERIFIED"), failures


def _normalize_branch_protection(source: Any) -> dict[str, Any]:
    require(isinstance(source, dict), "branch protection raw payload must be an object")
    review = source.get("required_pull_request_reviews")
    enforce_admins = source.get("enforce_admins") or {}
    allow_force_pushes = source.get("allow_force_pushes")
    allow_deletions = source.get("allow_deletions")
    bypass: list[dict[str, Any]] = []

    if isinstance(review, dict):
        allowances = review.get("bypass_pull_request_allowances")
        require(
            isinstance(allowances, dict),
            "branch protection bypass allowances are not enumerated",
        )
        for actor_kind, actor_type in (
            ("users", "User"),
            ("teams", "Team"),
            ("apps", "Integration"),
        ):
            actors = allowances.get(actor_kind)
            require(
                isinstance(actors, list),
                f"branch protection bypass {actor_kind} are not enumerated",
            )
            for actor in actors:
                require(
                    isinstance(actor, str) and bool(actor),
                    f"branch protection bypass {actor_kind} contain invalid actor identity",
                )
                bypass.append({"actor_type": actor_type, "actor_id": actor})
    if enforce_admins.get("enabled") is not True:
        bypass.append({"actor_type": "RepositoryAdministrator"})

    return {
        "pull_request_required": isinstance(review, dict),
        "required_approving_review_count": (
            review.get("required_approving_review_count")
            if isinstance(review, dict)
            else 0
        ),
        "dismiss_stale_reviews_on_push": (
            review.get("dismiss_stale_reviews") is True
            if isinstance(review, dict)
            else False
        ),
        "require_last_push_approval": (
            review.get("require_last_push_approval") is True
            if isinstance(review, dict)
            else False
        ),
        "required_conversation_resolution": (
            source.get("required_conversation_resolution") is True
        ),
        "block_force_push": (
            isinstance(allow_force_pushes, dict)
            and allow_force_pushes.get("enabled") is False
        ),
        "block_deletion": (
            isinstance(allow_deletions, dict)
            and allow_deletions.get("enabled") is False
        ),
        "bypass_actors": bypass,
        "administrator_bypass_prevented": (
            enforce_admins.get("enabled") is True
        ),
    }


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

    default_branch = observation.get("default_branch")
    require(
        default_branch == "main",
        f"repository default branch is not main: {default_branch!r}",
    )
    ruleset_state, ruleset_mismatches = _evaluate_rulesets(
        ruleset_entries, default_branch
    )
    effective_rules = observation.get("effective_rules", {}).get("entries")
    require(isinstance(effective_rules, list), "effective rules observation missing")
    effective_state, effective_mismatches = _evaluate_effective_rules(effective_rules)
    if ruleset_state == "VERIFIED" and effective_state == "UNVERIFIED":
        ruleset_mismatches.extend(effective_mismatches)
        ruleset_state = "UNVERIFIED"
    elif ruleset_state == "VERIFIED" and effective_state == "MISMATCH":
        ruleset_mismatches.extend(effective_mismatches)
        ruleset_state = "MISMATCH"

    if admin_visibility != "verified":
        if branch_protected is False and ruleset_state in {"ABSENT", "MISMATCH"}:
            reason = (
                "target_branch_is_unprotected_and_no_rule_set_is_observed"
                if ruleset_state == "ABSENT"
                else "publicly_observed_ruleset_controls_mismatch"
            )
            return {
                "evaluator_id": EVALUATOR_ID,
                "valid": False,
                "governance_state": "MISMATCH",
                "reason": reason,
                "mismatches": ruleset_mismatches,
                "claim_ceiling": "RepositoryGovernanceObservationOnly",
                "authoritative_admin_observation": False,
                "grants_trusted_verifier_root": False,
            }

        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "UNVERIFIED",
            "reason": "github_branch_protection_admin_observation_unavailable",
            "mismatches": ruleset_mismatches,
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": False,
            "grants_trusted_verifier_root": False,
        }

    branch_state = "ABSENT"
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
        branch_state = "ABSENT"
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

    mismatches = branch_mismatches + ruleset_mismatches

    # An observed contradiction is stronger than an unavailable secondary
    # control-plane view, but a complete verified control-plane witness can
    # qualify on its own.
    if "MISMATCH" in {branch_state, ruleset_state}:
        reason = (
            "observable_governance_control_mismatch"
            if mismatches
            else "observable_governance_control_mismatch_without_detail"
        )
        return {
            "evaluator_id": EVALUATOR_ID,
            "valid": False,
            "governance_state": "MISMATCH",
            "reason": reason,
            "mismatches": mismatches,
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": True,
            "grants_trusted_verifier_root": False,
        }

    if "VERIFIED" in {branch_state, ruleset_state}:
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

    return {
        "evaluator_id": EVALUATOR_ID,
        "valid": False,
        "governance_state": "MISMATCH",
        "reason": "no_acceptable_control_plane_observed",
        "mismatches": ["no_acceptable_control_plane_observed"],
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


def fixture_observation(
    policy: dict[str, Any],
    protection_status: int = 200,
    admin_status: str = "verified",
) -> dict[str, Any]:
    ruleset_entry = {
        "id": 1,
        "name": "fixture-main-protection",
        "source_type": "Repository",
        "source": REPOSITORY,
        "target": "branch",
        "updated_at": "2026-10-07T00:00:00Z",
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
    effective_rules_payload = [
        {
            "type": rule["type"],
            **({"parameters": rule["parameters"]} if "parameters" in rule else {}),
        }
        for rule in ruleset_entry["rules"]
    ]
    effective_rules_raw = json.dumps(
        effective_rules_payload, separators=(",", ":"), sort_keys=True
    ).encode()

    rulesets_index_payload = [
        {
            key: ruleset_entry[key]
            for key in ("id", "name", "source_type", "source", "enforcement", "updated_at")
        }
    ]
    protection_payload = {
        "required_pull_request_reviews": {
            "dismiss_stale_reviews": True,
            "require_last_push_approval": True,
            "required_approving_review_count": 1,
            "bypass_pull_request_allowances": {"users": [], "teams": [], "apps": []},
        },
        "enforce_admins": {"enabled": True},
        "required_conversation_resolution": True,
        "allow_force_pushes": {"enabled": False},
        "allow_deletions": {"enabled": False},
    }
    branch_raw = json.dumps(branch_payload, separators=(",", ":"), sort_keys=True).encode()
    rulesets_raw = json.dumps(rulesets_payload, separators=(",", ":"), sort_keys=True).encode()
    effective_rules_raw = json.dumps(
        effective_rules_payload, separators=(",", ":"), sort_keys=True
    ).encode()
    rulesets_index_raw = json.dumps(
        rulesets_index_payload, separators=(",", ":"), sort_keys=True
    ).encode()
    protection_raw = json.dumps(protection_payload, separators=(",", ":"), sort_keys=True).encode()
    repository_raw = json.dumps(
        {
            "id": REPOSITORY_ID,
            "full_name": REPOSITORY,
            "default_branch": "main",
        },
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    return {
        "schema": SCHEMA,
        "version": 1,
        "repository": REPOSITORY,
        "repository_id": REPOSITORY_ID,
        "target_ref": TARGET_REF,
        "observed_at_utc": "2026-10-07T00:00:00Z",
        "default_branch": "main",
        "repository_payload_base64": base64.b64encode(repository_raw).decode(),
        "repository_payload_sha256": hashlib.sha256(repository_raw).hexdigest(),
        "policy_sha256": hashlib.sha256(
            json.dumps(policy, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        "branch_payload_base64": base64.b64encode(branch_raw).decode(),
        "branch_payload_sha256": hashlib.sha256(branch_raw).hexdigest(),
        "rulesets_index_payload_base64": base64.b64encode(rulesets_index_raw).decode(),
        "rulesets_index_payload_sha256": hashlib.sha256(rulesets_index_raw).hexdigest(),
        "effective_rules_payload_base64": base64.b64encode(effective_rules_raw).decode(),
        "effective_rules_payload_sha256": hashlib.sha256(effective_rules_raw).hexdigest(),
        "rulesets_payload_base64": base64.b64encode(rulesets_raw).decode(),
        "rulesets_payload_sha256": hashlib.sha256(rulesets_raw).hexdigest(),
        "branch_protection_payload_base64": base64.b64encode(protection_raw).decode(),
        "branch_protection_payload_sha256": hashlib.sha256(protection_raw).hexdigest(),
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


def _refresh_bound_fixture_payloads(observation: dict[str, Any]) -> None:
    repository_raw = json.dumps(
        {
            "id": REPOSITORY_ID,
            "full_name": REPOSITORY,
            "default_branch": observation["default_branch"],
        },
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    observation["repository_payload_base64"] = base64.b64encode(repository_raw).decode()
    observation["repository_payload_sha256"] = hashlib.sha256(repository_raw).hexdigest()

    branch_raw = json.dumps(
        {
            "name": observation["branch"]["name"],
            "protected": observation["branch"]["protected"],
        },
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    rulesets_payload = observation["rulesets"]["entries"]
    effective_rules_payload = []
    for ruleset in rulesets_payload:
        for rule in ruleset.get("rules", []):
            effective_rules_payload.append(
                {
                    "type": rule.get("type"),
                    **({"parameters": rule["parameters"]} if "parameters" in rule else {}),
                }
            )
    rulesets_raw = json.dumps(
        rulesets_payload,
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    rulesets_index_payload = [
        {
            key: entry[key]
            for key in ("id", "name", "source_type", "source", "enforcement", "updated_at")
        }
        for entry in rulesets_payload
    ]
    rulesets_index_raw = json.dumps(
        rulesets_index_payload, separators=(",", ":"), sort_keys=True
    ).encode()
    effective_rules_raw = json.dumps(
        effective_rules_payload, separators=(",", ":"), sort_keys=True
    ).encode()
    observation["branch_payload_base64"] = base64.b64encode(branch_raw).decode()
    observation["branch_payload_sha256"] = hashlib.sha256(branch_raw).hexdigest()
    observation["rulesets_index_payload_base64"] = base64.b64encode(rulesets_index_raw).decode()
    observation["rulesets_index_payload_sha256"] = hashlib.sha256(rulesets_index_raw).hexdigest()
    observation["effective_rules_payload_base64"] = base64.b64encode(effective_rules_raw).decode()
    observation["effective_rules_payload_sha256"] = hashlib.sha256(effective_rules_raw).hexdigest()
    observation["rulesets_payload_base64"] = base64.b64encode(rulesets_raw).decode()
    observation["rulesets_payload_sha256"] = hashlib.sha256(rulesets_raw).hexdigest()

    normalized = observation["admin_observation"]["protection"]
    require(isinstance(normalized, dict), "fixture normalized protection missing")
    users: list[Any] = []
    teams: list[Any] = []
    apps: list[Any] = []
    for actor in normalized.get("bypass_actors", []):
        actor_type = actor.get("actor_type")
        if actor_type == "User":
            users.append(actor.get("actor_id"))
        elif actor_type == "Team":
            teams.append(actor.get("actor_id"))
        elif actor_type == "Integration":
            apps.append(actor.get("actor_id"))
        elif actor_type == "RepositoryAdministrator":
            pass
        else:
            raise EvidenceError(f"unsupported fixture bypass actor type: {actor_type}")

    protection_raw = {
        "required_pull_request_reviews": (
            {
                "dismiss_stale_reviews": normalized["dismiss_stale_reviews_on_push"],
                "require_last_push_approval": normalized["require_last_push_approval"],
                "required_approving_review_count": normalized["required_approving_review_count"],
                "bypass_pull_request_allowances": {
                    "users": users,
                    "teams": teams,
                    "apps": apps,
                },
            }
            if normalized["pull_request_required"]
            else None
        ),
        "enforce_admins": {
            "enabled": normalized["administrator_bypass_prevented"]
        },
        "required_conversation_resolution": normalized["required_conversation_resolution"],
        "allow_force_pushes": {
            "enabled": not normalized["block_force_push"]
        },
        "allow_deletions": {
            "enabled": not normalized["block_deletion"]
        },
    }
    protection_bytes = json.dumps(
        protection_raw, separators=(",", ":"), sort_keys=True
    ).encode()
    observation["branch_protection_payload_base64"] = base64.b64encode(protection_bytes).decode()
    observation["branch_protection_payload_sha256"] = hashlib.sha256(protection_bytes).hexdigest()


def self_test(policy: dict[str, Any]) -> None:
    fixture = fixture_policy()
    require(
        policy.get("repository") == fixture["repository"]
        and policy.get("repository_id") == fixture["repository_id"]
        and policy.get("target_ref") == fixture["target_ref"]
        and policy.get("required_controls") == fixture["required_controls"],
        "committed policy controls drift from evaluator self-test fixture",
    )
    positive = evaluate(policy, fixture_observation(policy))
    assert positive["governance_state"] == "VERIFIED"
    assert positive["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation(policy))
    x["branch"]["protected"] = False
    x["rulesets"]["entries"] = []
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(policy, protection_status=403))
    result = evaluate(policy, x)
    assert result["governance_state"] == "VERIFIED"
    assert result["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation(policy, protection_status=403))
    x["rulesets"]["entries"][0]["rules"] = [
        {"type": "non_fast_forward"},
        {"type": "deletion"},
    ]
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "UNVERIFIED"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(policy, protection_status=404))
    result = evaluate(policy, x)
    assert result["governance_state"] == "VERIFIED"
    assert result["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation(policy, protection_status=404))
    second = copy.deepcopy(x["rulesets"]["entries"][0])
    x["rulesets"]["entries"][0]["rules"] = [
        {
            "type": "pull_request",
            "parameters": {
                "dismiss_stale_reviews_on_push": True,
                "required_approving_review_count": 1,
            },
        },
    ]
    second["id"] = 2
    second["rules"] = [
        {
            "type": "pull_request",
            "parameters": {
                "require_last_push_approval": True,
                "required_review_thread_resolution": True,
            },
        },
        {"type": "non_fast_forward"},
        {"type": "deletion"},
    ]
    x["rulesets"]["entries"].append(second)
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "VERIFIED"
    assert result["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation(policy, protection_status=404))
    x["rulesets"]["entries"][0]["conditions"]["ref_name"]["include"] = ["refs/heads/*"]
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "VERIFIED"
    assert result["grants_trusted_verifier_root"] is True

    x = copy.deepcopy(fixture_observation(policy, protection_status=404))
    x["default_branch"] = "develop"
    repository_raw = json.loads(
        base64.b64decode(x["repository_payload_base64"]).decode("utf-8")
    )
    repository_raw["default_branch"] = "develop"
    repository_bytes = json.dumps(
        repository_raw, separators=(",", ":"), sort_keys=True
    ).encode()
    x["repository_payload_base64"] = base64.b64encode(repository_bytes).decode()
    x["repository_payload_sha256"] = hashlib.sha256(repository_bytes).hexdigest()
    result = evaluate(policy, x)
    assert result["governance_state"] == "UNVERIFIED"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(policy, protection_status=404))
    x["rulesets"]["entries"][0]["conditions"]["ref_name"]["include"] = ["~ALL"]
    x["rulesets"]["entries"][0]["conditions"]["ref_name"]["exclude"] = ["~DEFAULT_BRANCH"]
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(policy, protection_status=404))
    x["rulesets"]["entries"][0]["conditions"]["ref_name"]["include"] = ["refs/heads/*"]
    x["rulesets"]["entries"][0]["conditions"]["ref_name"]["exclude"] = ["refs/heads/main"]
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(policy))
    x["rulesets"]["entries"][0]["bypass_actors"] = [{"actor_type": "User", "actor_id": 7}]
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"
    assert result["grants_trusted_verifier_root"] is False

    x = copy.deepcopy(fixture_observation(policy))
    protection_raw = json.loads(
        base64.b64decode(x["branch_protection_payload_base64"]).decode("utf-8")
    )
    del protection_raw["required_pull_request_reviews"]["bypass_pull_request_allowances"]
    protection_bytes = json.dumps(
        protection_raw, separators=(",", ":"), sort_keys=True
    ).encode()
    x["branch_protection_payload_base64"] = base64.b64encode(protection_bytes).decode()
    x["branch_protection_payload_sha256"] = hashlib.sha256(protection_bytes).hexdigest()
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("missing bypass enumeration must be rejected")

    x = copy.deepcopy(fixture_observation(policy))
    x["policy_sha256"] = "f" * 64
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("policy digest substitution must be rejected")

    x = copy.deepcopy(fixture_observation(policy))
    x["branch_payload_base64"] = base64.b64encode(
        b'{"name":"attacker","protected":true}'
    ).decode()
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("raw branch payload substitution must be rejected")

    x = copy.deepcopy(fixture_observation(policy))
    raw_branch = json.dumps(
        {"name": "attacker", "protected": True},
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    x["branch_payload_base64"] = base64.b64encode(raw_branch).decode()
    x["branch_payload_sha256"] = hashlib.sha256(raw_branch).hexdigest()
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("rehashed raw branch substitution must be rejected at normalization binding")

    x = copy.deepcopy(fixture_observation(policy))
    x["admin_observation"]["source"] = "github_token"
    try:
        evaluate(policy, x)
    except EvidenceError:
        pass
    else:
        raise AssertionError("a non-administration observation source must not qualify for VERIFIED")

    x = copy.deepcopy(fixture_observation(policy))
    x["branch"]["protected"] = False
    x["rulesets"]["entries"][0]["rules"] = [
        {"type": "non_fast_forward"},
        {"type": "deletion"},
    ]
    _refresh_bound_fixture_payloads(x)
    x["admin_observation"]["status"] = "unverified"
    x["admin_observation"]["source"] = "github_token"
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"

    x = copy.deepcopy(fixture_observation(policy))
    x["rulesets"]["entries"][0]["rules"] = [
        {"type": "non_fast_forward"},
        {"type": "deletion"},
    ]
    _refresh_bound_fixture_payloads(x)
    x["admin_observation"]["status"] = "unverified"
    x["admin_observation"]["source"] = "github_token"
    result = evaluate(policy, x)
    assert result["governance_state"] == "UNVERIFIED"

    x = copy.deepcopy(fixture_observation(policy))
    x["admin_observation"]["protection"]["block_force_push"] = False
    _refresh_bound_fixture_payloads(x)
    result = evaluate(policy, x)
    assert result["governance_state"] == "MISMATCH"

    x = copy.deepcopy(fixture_observation(policy))
    x["admin_observation"]["protection"]["bypass_actors"] = [{"actor_type": "User", "actor_id": 123}]
    _refresh_bound_fixture_payloads(x)
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
            self_test(policy)
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
            "governance_state": "UNVERIFIED",
            "reason": f"evidence_evaluation_error:{exc}",
            "claim_ceiling": "RepositoryGovernanceObservationOnly",
            "authoritative_admin_observation": False,
            "grants_trusted_verifier_root": False,
        }, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
