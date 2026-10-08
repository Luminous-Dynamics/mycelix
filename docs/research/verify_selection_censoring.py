#!/usr/bin/env python3
"""Dependency-light research reference verifier for adaptive selection/censoring.

Research fixture only. It detects protocol/selection distortions; it does not
identify a universally valid causal censoring model and is not authoritative.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path

ATTEMPT_REQUIRED = {
    "id","eligible_at_time_zero","time_zero_epoch","action","terminal_state",
    "observation_end_epoch","observation_at_horizon","censoring_reason","censoring_basis",
    "action_induced_censoring","outcome_dependent_censoring",
    "horizon_completed","failure_is_outcome",
}
DECISION_REQUIRED = {"id","attempt_id","epoch","eligible","triggered"}


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def digest(value):
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def ids_unique(items):
    ids = [item.get("id") for item in items]
    return all(isinstance(i, str) for i in ids) and len(ids) == len(set(ids))


def semantic_normalize(case):
    out = copy.deepcopy(case)
    out["attempts"] = sorted(out["attempts"], key=lambda x: x["id"])
    out["decision_points"] = sorted(out["decision_points"], key=lambda x: x["id"])
    analysis = out["analysis"]
    analysis["included_attempt_ids"] = sorted(analysis["included_attempt_ids"])
    analysis["trigger_denominator_ids"] = sorted(analysis["trigger_denominator_ids"])
    return out


def structure_valid(case, protocol, policy):
    if not isinstance(case, dict):
        return False
    attempts = case.get("attempts")
    decisions = case.get("decision_points")
    analysis = case.get("analysis")
    if not isinstance(attempts, list) or not isinstance(decisions, list) or not isinstance(analysis, dict):
        return False
    if policy["attempt_completeness"].get("one_record_per_attempt", True) and not ids_unique(attempts):
        return False
    if policy["decision_completeness"].get("one_record_per_attempt", True) and not ids_unique(decisions):
        return False
    attempt_ids = {a["id"] for a in attempts}
    expected_attempts = protocol.get("expected_attempt_ids", [])
    expected_decisions = protocol.get("expected_decision_ids", [])
    if policy["attempt_completeness"].get("require_exact_id_set", True):
        if set(attempt_ids) != set(expected_attempts):
            return False
    else:
        if not set(attempt_ids).issubset(set(expected_attempts)):
            return False
    decision_ids = {d["id"] for d in decisions}
    if policy["decision_completeness"].get("require_exact_id_set", True):
        if set(decision_ids) != set(expected_decisions):
            return False
    else:
        if not set(decision_ids).issubset(set(expected_decisions)):
            return False
    for attempt in attempts:
        if not ATTEMPT_REQUIRED.issubset(attempt):
            return False
        if policy["time_zero"].get("must_be_frozen_before_action", True):
            if attempt["time_zero_epoch"] != protocol["time_zero_epoch"]:
                return False
        if attempt["horizon_completed"] and attempt["observation_at_horizon"] is not True:
            return False
        if attempt["observation_end_epoch"] == protocol["horizon_epoch"] and attempt["observation_at_horizon"] is not True:
            return False
        if attempt["terminal_state"] == "OutcomeFailure" and attempt["failure_is_outcome"] is not True:
            return False
        if attempt["terminal_state"] == "OutcomeFailure" and attempt["censoring_reason"] != "None":
            return False
        if attempt["censoring_reason"] == "None":
            if attempt["observation_at_horizon"] is not True and attempt["terminal_state"] == "OutcomeFailure":
                return False
    for decision in decisions:
        if not DECISION_REQUIRED.issubset(decision):
            return False
        if decision["attempt_id"] not in attempt_ids:
            return False
        if decision["eligible"] is True and not isinstance(decision["triggered"], bool):
            return False
    included = analysis.get("included_attempt_ids")
    if not isinstance(included, list) or len(included) != len(set(included)):
        return False
    if not set(included).issubset(attempt_ids):
        return False
    denominator = analysis.get("trigger_denominator_ids")
    if not isinstance(denominator, list) or len(denominator) != len(set(denominator)):
        return False
    if not set(denominator).issubset(decision_ids):
        return False
    return True


def verify(case, policy, protocol):
    if not structure_valid(case, protocol, policy):
        return "unresolved"

    attempts = {a["id"]: a for a in case["attempts"]}
    analysis = case["analysis"]
    estimand = analysis.get("estimand")
    selection_rule = analysis.get("selection_rule")
    included = set(analysis["included_attempt_ids"])
    eligible_attempts = {a["id"] for a in case["attempts"] if a["eligible_at_time_zero"] is True}
    eligible_decisions = {d["id"] for d in case["decision_points"] if d["eligible"] is True}
    denominator = set(analysis["trigger_denominator_ids"])

    if policy["analysis"].get("pre_outcome_classification_required", True):
        if analysis.get("classification_timing") != "pre_outcome":
            return "unqualified"

    if policy["decision_completeness"].get("eligible_decisions_must_be_counted", True):
        if denominator != eligible_decisions:
            return "unqualified"

    if analysis.get("complete_case_filter") is True and estimand != policy["analysis"]["complete_case_filter_allowed_only"]:
        return "unqualified"

    survivor_rule = policy["population"]["survivor_selection_rule"]
    full_rule = policy["population"]["full_episode_selection_rule"]
    full_estimands = set(policy["analysis"]["full_population_estimands"])

    if selection_rule == survivor_rule and estimand != policy["analysis"]["survivor_selection_allowed_only"]:
        return "unqualified"
    if selection_rule != full_rule and selection_rule != survivor_rule:
        return "unqualified"

    if estimand in full_estimands:
        if selection_rule != full_rule:
            return "unqualified"
        if included != eligible_attempts:
            return "unqualified"

    if estimand == "SurvivorConditionalValue":
        if selection_rule != survivor_rule:
            return "unqualified"
        if not included.issubset(eligible_attempts):
            return "unqualified"

    if estimand == "CensoringUnresolved":
        return "unresolved"

    informative_without_adjustment = False
    explicit_basis_missing = False
    for a in attempts.values():
        reason = a["censoring_reason"]
        if reason == "ActionInducedCensoring" and not a["action_induced_censoring"]:
            return "unresolved"
        if reason == "OutcomeDependentCensoring" and not a["outcome_dependent_censoring"]:
            return "unresolved"
        if reason in policy["censoring"]["requires_explicit_basis"] and not a.get("censoring_basis"):
            explicit_basis_missing = True
        if reason in policy["censoring"]["always_requires_adjustment_or_block"]:
            informative_without_adjustment = True

        if reason == "None" and a["action_induced_censoring"]:
            return "unresolved"

        if a["terminal_state"] == "OutcomeFailure" and not a["failure_is_outcome"]:
            return "unresolved"
        if a["terminal_state"] == "OutcomeFailure" and a["censoring_reason"] != "None":
            return "unresolved"

    adjustment = analysis.get("adjustment")
    adjusted_ok = False
    if isinstance(adjustment, dict):
        required = policy["censoring"]["adjustment_requirements"]
        adjusted_ok = (
            all(adjustment.get(k) is not None for k in required)
            and adjustment.get("pre_specified_before_outcomes") is True
            and adjustment.get("frozen") is True
        )

    if explicit_basis_missing:
        return "unresolved"

    if informative_without_adjustment and not adjusted_ok:
        return "unresolved"

    if estimand == "CensoringAdjustedValue" and not adjusted_ok:
        return "unresolved"

    for a in attempts.values():
        if a["censoring_reason"] in policy["censoring"]["requires_explicit_basis"]:
            if not a.get("censoring_basis"):
                return "unresolved"

    return "qualified"


def apply_mutations(base, mutations):
    out = copy.deepcopy(base)
    for op in mutations:
        kind = op[0]
        if kind == "set_attempt":
            _, attempt_id, field, value = op
            node = next((a for a in out["attempts"] if a["id"] == attempt_id), None)
            if node is None:
                raise ValueError("unknown attempt")
            node[field] = value
        elif kind == "set_analysis":
            _, field, value = op
            out["analysis"][field] = value
        elif kind == "set_decision":
            _, decision_id, field, value = op
            node = next((d for d in out["decision_points"] if d["id"] == decision_id), None)
            if node is None:
                raise ValueError("unknown decision")
            node[field] = value
        elif kind == "remove_attempt":
            _, attempt_id = op
            out["attempts"] = [a for a in out["attempts"] if a["id"] != attempt_id]
        elif kind == "remove_decision":
            _, decision_id = op
            out["decision_points"] = [d for d in out["decision_points"] if d["id"] != decision_id]
        elif kind == "reverse_collection":
            _, collection = op
            if collection not in ("attempts","decision_points"):
                raise ValueError("unsupported collection")
            out[collection].reverse()
        else:
            raise ValueError(f"unknown mutation: {kind}")
    return out


def main():
    if len(sys.argv) != 5:
        print("usage: verify_selection.py EXPECTED_POLICY_SHA POLICY.json FIXTURES.json REPORT.json", file=sys.stderr)
        return 2
    expected_sha, policy_path, fixtures_path, report_path = sys.argv[1:5]
    policy_file = Path(policy_path)
    policy_bytes = policy_file.read_bytes()
    actual_sha = hashlib.sha1(f"blob {len(policy_bytes)} ".encode() + policy_bytes).hexdigest()
    if actual_sha != expected_sha:
        print("policy binding mismatch", file=sys.stderr)
        return 1

    policy = json.loads(policy_bytes)
    corpus = json.loads(Path(fixtures_path).read_text(encoding="utf-8"))
    if corpus.get("policy_binding", {}).get("git_blob_sha") != actual_sha:
        print("fixture policy binding mismatch", file=sys.stderr)
        return 1

    failures = []
    rows = []
    for test_case in corpus["cases"]:
        mutated = apply_mutations(corpus["base_case"], test_case["mutation"])
        verdict = verify(mutated, policy, corpus["protocol"])
        semantic = digest(semantic_normalize(mutated))
        rows.append({
            "case_id":test_case["case_id"],
            "expected_verdict":test_case["expected_verdict"],
            "actual_verdict":verdict,
            "semantic_digest_sha256":semantic
        })
        if verdict != test_case["expected_verdict"]:
            failures.append([test_case["case_id"], test_case["expected_verdict"], verdict])

    report = {
        "schema":"mycelix.continual-adaptation.selection-censoring-report.v1",
        "status":"research-evidence-only",
        "policy_blob_sha":actual_sha,
        "cases":rows,
        "failures":failures,
    }
    Path(report_path).write_text(json.dumps(report, indent=2, sort_keys=True)+"
", encoding="utf-8")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
