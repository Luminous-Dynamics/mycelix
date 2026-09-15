#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-stewardship-decision-gate-v1"
RECEIPT_VERSION = "mycelix-stewardship-decision-receipt-v1"
PPM = 1_000_000
DECISION_CLASSES = {"ORDINARY", "CONSTITUTIONAL", "EMERGENCY"}
EMERGENCY_STATES = {"ACTIVE", "PENDING", "REVOKED"}
HEX64 = re.compile(r"^[0-9a-f]{64}$")

PROFILE_KEYS = {
    "profile_version", "project_id", "max_vote_count", "enforcer_ref",
    "emergency_authority_ref", "chambers", "conflict_codes", "actions",
}
ACTION_KEYS = {
    "decision_class", "prohibited", "required_chambers", "enforcer_required",
    "emergency_required", "required_recusal_conflicts",
}
THRESHOLD_KEYS = {"quorum_ppm", "approval_ppm"}
DECISION_KEYS = {
    "decision_id", "project_id", "profile_sha256", "action_code", "declared_class",
    "chamber_votes", "conflicts", "enforcer", "emergency", "evidence_ref",
}
VOTE_KEYS = {"eligible_count", "participating_count", "approval_count"}
CONFLICT_KEYS = {"actor_ref", "conflict_code", "disclosed", "recused", "counted_vote"}
ENFORCER_KEYS = {"authority_ref", "concurred", "evidence_ref"}
EMERGENCY_KEYS = {"authority_ref", "state", "scope_actions", "evidence_ref"}


class GovernanceError(ValueError):
    pass


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def sha256_hex(value: Any) -> str:
    raw = value if isinstance(value, (bytes, bytearray)) else canonical_bytes(value)
    return hashlib.sha256(raw).hexdigest()


def _exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    got = set(obj)
    if got != expected:
        raise GovernanceError(f"{label} keys mismatch missing={sorted(expected-got)} unknown={sorted(got-expected)}")


def _str(v: Any, label: str) -> str:
    if not isinstance(v, str) or not v or len(v) > 256:
        raise GovernanceError(f"invalid {label}")
    return v


def _bool(v: Any, label: str) -> bool:
    if type(v) is not bool:
        raise GovernanceError(f"invalid {label}")
    return v


def _int(v: Any, label: str, maximum: int) -> int:
    if type(v) is not int or v < 0 or v > maximum:
        raise GovernanceError(f"invalid {label}")
    return v


def _ppm(v: Any, label: str) -> int:
    return _int(v, label, PPM)


def validate_profile(profile: dict[str, Any]) -> None:
    _exact_keys(profile, PROFILE_KEYS, "profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise GovernanceError("unsupported profile version")
    _str(profile["project_id"], "project_id")
    max_vote_count = _int(profile["max_vote_count"], "max_vote_count", 10**9)
    if max_vote_count == 0:
        raise GovernanceError("max_vote_count must be positive")
    _str(profile["enforcer_ref"], "enforcer_ref")
    _str(profile["emergency_authority_ref"], "emergency_authority_ref")

    chambers = profile["chambers"]
    if not isinstance(chambers, list) or not chambers or len(chambers) > 32:
        raise GovernanceError("invalid chambers")
    if len(set(chambers)) != len(chambers):
        raise GovernanceError("duplicate chamber")
    for c in chambers:
        _str(c, "chamber")

    conflict_codes = profile["conflict_codes"]
    if not isinstance(conflict_codes, list) or len(conflict_codes) > 64:
        raise GovernanceError("invalid conflict_codes")
    if len(set(conflict_codes)) != len(conflict_codes):
        raise GovernanceError("duplicate conflict code")
    for code in conflict_codes:
        _str(code, "conflict_code")

    actions = profile["actions"]
    if not isinstance(actions, dict) or not actions or len(actions) > 128:
        raise GovernanceError("invalid actions")
    for action_code, rule in actions.items():
        _str(action_code, "action_code")
        if not isinstance(rule, dict):
            raise GovernanceError("invalid action rule")
        _exact_keys(rule, ACTION_KEYS, f"action:{action_code}")
        if rule["decision_class"] not in DECISION_CLASSES:
            raise GovernanceError("invalid decision class")
        _bool(rule["prohibited"], "prohibited")
        _bool(rule["enforcer_required"], "enforcer_required")
        _bool(rule["emergency_required"], "emergency_required")
        if rule["emergency_required"] != (rule["decision_class"] == "EMERGENCY"):
            raise GovernanceError("emergency_required/class mismatch")
        required = rule["required_chambers"]
        if not isinstance(required, dict) or not required:
            raise GovernanceError("invalid required_chambers")
        if not set(required).issubset(set(chambers)):
            raise GovernanceError("unknown required chamber")
        for chamber, thresholds in required.items():
            _exact_keys(thresholds, THRESHOLD_KEYS, f"threshold:{action_code}:{chamber}")
            _ppm(thresholds["quorum_ppm"], "quorum_ppm")
            _ppm(thresholds["approval_ppm"], "approval_ppm")
        recusal = rule["required_recusal_conflicts"]
        if not isinstance(recusal, list) or len(set(recusal)) != len(recusal):
            raise GovernanceError("invalid required_recusal_conflicts")
        if not set(recusal).issubset(set(conflict_codes)):
            raise GovernanceError("unknown recusal conflict")


def _meets(numerator: int, denominator: int, threshold_ppm: int) -> bool:
    if denominator <= 0:
        return False
    return numerator * PPM >= threshold_ppm * denominator


@dataclass(frozen=True)
class Result:
    value: dict[str, Any]

    def receipt(self) -> dict[str, Any]:
        return self.value


def qualify(profile: dict[str, Any], decision: dict[str, Any]) -> Result:
    validate_profile(profile)
    if not isinstance(decision, dict):
        raise GovernanceError("decision must be object")
    _exact_keys(decision, DECISION_KEYS, "decision")

    project_id = _str(decision["project_id"], "decision.project_id")
    if project_id != profile["project_id"]:
        raise GovernanceError("project substitution")
    profile_sha = sha256_hex(profile)
    if decision["profile_sha256"] != profile_sha or not HEX64.fullmatch(decision["profile_sha256"]):
        raise GovernanceError("profile substitution")
    decision_id = _str(decision["decision_id"], "decision_id")
    action_code = _str(decision["action_code"], "action_code")
    if action_code not in profile["actions"]:
        raise GovernanceError("unknown action code")
    rule = profile["actions"][action_code]
    if decision["declared_class"] != rule["decision_class"]:
        raise GovernanceError("decision class substitution")
    _str(decision["evidence_ref"], "evidence_ref")

    blockers: list[str] = []
    chamber_results: dict[str, Any] = {}
    votes = decision["chamber_votes"]
    if not isinstance(votes, dict):
        raise GovernanceError("invalid chamber_votes")
    required_names = set(rule["required_chambers"])
    supplied_names = set(votes)
    unknown = supplied_names - set(profile["chambers"])
    if unknown:
        raise GovernanceError(f"unknown chamber votes={sorted(unknown)}")
    for chamber in sorted(required_names):
        if chamber not in votes:
            blockers.append(f"MISSING_CHAMBER:{chamber}")
            continue
        vote = votes[chamber]
        if not isinstance(vote, dict):
            raise GovernanceError("invalid chamber vote")
        _exact_keys(vote, VOTE_KEYS, f"vote:{chamber}")
        max_count = profile["max_vote_count"]
        eligible = _int(vote["eligible_count"], f"{chamber}.eligible_count", max_count)
        participating = _int(vote["participating_count"], f"{chamber}.participating_count", max_count)
        approvals = _int(vote["approval_count"], f"{chamber}.approval_count", max_count)
        if participating > eligible or approvals > participating:
            raise GovernanceError("impossible chamber counts")
        thresholds = rule["required_chambers"][chamber]
        quorum_pass = _meets(participating, eligible, thresholds["quorum_ppm"])
        approval_pass = _meets(approvals, participating, thresholds["approval_ppm"])
        if not quorum_pass:
            blockers.append(f"QUORUM_FAIL:{chamber}")
        if not approval_pass:
            blockers.append(f"APPROVAL_FAIL:{chamber}")
        chamber_results[chamber] = {
            "eligible_count": eligible,
            "participating_count": participating,
            "approval_count": approvals,
            "quorum_pass": quorum_pass,
            "approval_pass": approval_pass,
        }

    extra_required_irrelevant = supplied_names - required_names
    if extra_required_irrelevant:
        raise GovernanceError(f"non-required chamber injected={sorted(extra_required_irrelevant)}")

    conflicts = decision["conflicts"]
    if not isinstance(conflicts, list) or len(conflicts) > 256:
        raise GovernanceError("invalid conflicts")
    seen_conflicts: set[tuple[str, str]] = set()
    conflict_results: list[dict[str, Any]] = []
    required_recusal = set(rule["required_recusal_conflicts"])
    for item in conflicts:
        if not isinstance(item, dict):
            raise GovernanceError("invalid conflict record")
        _exact_keys(item, CONFLICT_KEYS, "conflict")
        actor = _str(item["actor_ref"], "actor_ref")
        code = _str(item["conflict_code"], "conflict_code")
        if code not in profile["conflict_codes"]:
            raise GovernanceError("unknown conflict code")
        key = (actor, code)
        if key in seen_conflicts:
            raise GovernanceError("duplicate conflict identity")
        seen_conflicts.add(key)
        disclosed = _bool(item["disclosed"], "disclosed")
        recused = _bool(item["recused"], "recused")
        counted_vote = _bool(item["counted_vote"], "counted_vote")
        if recused and counted_vote:
            raise GovernanceError("recused vote counted")
        if code in required_recusal and not (disclosed and recused and not counted_vote):
            blockers.append(f"RECUSAL_FAIL:{actor}:{code}")
        conflict_results.append({
            "actor_ref": actor,
            "conflict_code": code,
            "disclosed": disclosed,
            "recused": recused,
            "counted_vote": counted_vote,
        })
    conflict_results.sort(key=lambda x: (x["actor_ref"], x["conflict_code"]))

    enforcer_result = "NOT_REQUIRED"
    enforcer = decision["enforcer"]
    if rule["enforcer_required"]:
        if not isinstance(enforcer, dict):
            blockers.append("ENFORCER_MISSING")
            enforcer_result = "MISSING"
        else:
            _exact_keys(enforcer, ENFORCER_KEYS, "enforcer")
            authority_ref = _str(enforcer["authority_ref"], "enforcer.authority_ref")
            concurred = _bool(enforcer["concurred"], "enforcer.concurred")
            _str(enforcer["evidence_ref"], "enforcer.evidence_ref")
            if authority_ref != profile["enforcer_ref"]:
                raise GovernanceError("enforcer substitution")
            if not concurred:
                blockers.append("ENFORCER_DID_NOT_CONCUR")
                enforcer_result = "NO_CONCURRENCE"
            else:
                enforcer_result = "CONCURRED"
    elif enforcer is not None:
        raise GovernanceError("unexpected enforcer evidence")

    emergency_result = "NOT_REQUIRED"
    emergency = decision["emergency"]
    if rule["emergency_required"]:
        if not isinstance(emergency, dict):
            blockers.append("EMERGENCY_MISSING")
            emergency_result = "MISSING"
        else:
            _exact_keys(emergency, EMERGENCY_KEYS, "emergency")
            authority_ref = _str(emergency["authority_ref"], "emergency.authority_ref")
            if authority_ref != profile["emergency_authority_ref"]:
                raise GovernanceError("emergency authority substitution")
            state = emergency["state"]
            if state not in EMERGENCY_STATES:
                raise GovernanceError("unknown emergency state")
            scope_actions = emergency["scope_actions"]
            if not isinstance(scope_actions, list) or not scope_actions or len(set(scope_actions)) != len(scope_actions):
                raise GovernanceError("invalid emergency scope")
            for scoped in scope_actions:
                _str(scoped, "emergency scope action")
            _str(emergency["evidence_ref"], "emergency.evidence_ref")
            if state != "ACTIVE":
                blockers.append(f"EMERGENCY_NOT_ACTIVE:{state}")
            if action_code not in scope_actions:
                blockers.append("EMERGENCY_SCOPE_MISS")
            emergency_result = state
    elif emergency is not None:
        raise GovernanceError("unexpected emergency evidence")

    if rule["prohibited"]:
        blockers.append(f"PROHIBITED_ACTION:{action_code}")

    blockers = sorted(set(blockers))
    authorization_state = "AUTHORIZED" if not blockers else "BLOCKED"
    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": profile["project_id"],
        "profile_sha256": profile_sha,
        "decision_id": decision_id,
        "decision_sha256": sha256_hex(decision),
        "action_code": action_code,
        "decision_class": rule["decision_class"],
        "chamber_results": chamber_results,
        "conflict_results": conflict_results,
        "enforcer_result": enforcer_result,
        "emergency_result": emergency_result,
        "blockers": blockers,
        "authorization_state": authorization_state,
        "asset_lock_removed": False,
        "legal_validity_established": False,
        "democratic_legitimacy_established": False,
        "nonclaims": [
            "governance authorization is not democratic legitimacy",
            "governance authorization is not legal or constitutional-law validity",
            "governance authorization does not establish custody or operational control",
            "conflict evidence validity does not reconstruct all voter identities or chamber counts",
            "authorization does not establish wisdom, fairness, or moral correctness of the decision",
        ],
    }
    return Result(receipt)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    args = ap.parse_args()
    case = json.loads(args.case.read_text(encoding="utf-8"))
    _exact_keys(case, {"profile", "decision"}, "case")
    receipt = qualify(case["profile"], case["decision"]).receipt()
    out = canonical_bytes(receipt)
    if args.receipt_out:
        args.receipt_out.write_bytes(out)
    else:
        print(out.decode("utf-8"), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
