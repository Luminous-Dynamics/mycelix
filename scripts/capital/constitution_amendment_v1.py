#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import governance_decision_currentness_v1 as g2a

PROFILE_VERSION = "mycelix-constitution-amendment-v1"
RECEIPT_VERSION = "mycelix-constitution-amendment-receipt-v1"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
PPM = 1_000_000
AMENDMENT_CLASSES = {
    "POLICY_PARAMETER",
    "REPRESENTATION",
    "CONSTITUTIONAL_STRUCTURE",
    "PROTECTED_INVARIANT",
}

PROFILE_KEYS = {
    "profile_version", "project_id", "registry_id", "max_epoch", "max_vote_count",
    "enforcer_ref", "protected_actions", "protected_chambers", "classes",
}
CLASS_KEYS = {
    "prohibited", "required_chambers", "enforcer_required", "allowed_changed_paths",
}
THRESHOLD_KEYS = {"quorum_ppm", "approval_ppm"}
AUTH_KEYS = {
    "decision_id", "project_id", "registry_id", "prior_epoch",
    "prior_profile_sha256", "successor_profile_sha256", "amendment_class",
    "chamber_votes", "enforcer", "notice_evidence_ref", "currentness_receipt_sha256",
}
VOTE_KEYS = {"eligible_count", "participating_count", "approval_count"}
ENFORCER_KEYS = {"authority_ref", "concurred", "evidence_ref"}
TRANSITION_KEYS = {
    "project_id", "registry_id", "prior_epoch", "successor_epoch",
    "prior_profile_sha256", "successor_profile_sha256", "amendment_class",
    "previous_transition_sha256", "evidence_ref",
}
CURRENTNESS_RECEIPT_KEYS = {
    "receipt_version", "profile_version", "project_id", "registry_id",
    "designation_epoch", "designation_state", "designation_sha256",
    "active_g1_profile_sha256", "g1_decision_receipt_sha256", "decision_id",
    "action_code", "decision_authorization_state", "revocation_set_sha256",
    "currentness_state", "execution_authority_established",
    "legal_validity_established", "democratic_legitimacy_established", "nonclaims",
}

class AmendmentError(ValueError):
    pass

def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")

def sha256_hex(value: Any) -> str:
    raw = value if isinstance(value, (bytes, bytearray)) else canonical_bytes(value)
    return hashlib.sha256(raw).hexdigest()

def _exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    got = set(obj)
    if got != expected:
        raise AmendmentError(f"{label} keys mismatch missing={sorted(expected-got)} unknown={sorted(got-expected)}")

def _str(v: Any, label: str) -> str:
    if not isinstance(v, str) or not v or len(v) > 512:
        raise AmendmentError(f"invalid {label}")
    return v

def _bool(v: Any, label: str) -> bool:
    if type(v) is not bool:
        raise AmendmentError(f"invalid {label}")
    return v

def _int(v: Any, label: str, maximum: int) -> int:
    if type(v) is not int or v < 0 or v > maximum:
        raise AmendmentError(f"invalid {label}")
    return v

def _hex(v: Any, label: str) -> str:
    if not isinstance(v, str) or not HEX64.fullmatch(v):
        raise AmendmentError(f"invalid {label}")
    return v

def _pointer_escape(token: str) -> str:
    return token.replace("~", "~0").replace("/", "~1")

def semantic_changed_paths(a: Any, b: Any, path: str = "") -> list[str]:
    if type(a) is not type(b):
        return [path or "/"]
    if isinstance(a, dict):
        changes: list[str] = []
        for key in sorted(set(a) | set(b)):
            child = f"{path}/{_pointer_escape(str(key))}"
            if key not in a or key not in b:
                changes.append(child)
            else:
                changes.extend(semantic_changed_paths(a[key], b[key], child))
        return changes
    if isinstance(a, list):
        return [] if canonical_bytes(a) == canonical_bytes(b) else [path or "/"]
    return [] if a == b else [path or "/"]

def validate_profile(profile: dict[str, Any]) -> None:
    if not isinstance(profile, dict):
        raise AmendmentError("amendment profile must be object")
    _exact_keys(profile, PROFILE_KEYS, "amendment profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise AmendmentError("unsupported amendment profile")
    _str(profile["project_id"], "profile.project_id")
    _str(profile["registry_id"], "profile.registry_id")
    if _int(profile["max_epoch"], "profile.max_epoch", 2**31 - 1) == 0:
        raise AmendmentError("max_epoch must be positive")
    if _int(profile["max_vote_count"], "profile.max_vote_count", 10**9) == 0:
        raise AmendmentError("max_vote_count must be positive")
    _str(profile["enforcer_ref"], "profile.enforcer_ref")
    if profile["protected_actions"] != ["ASSET_LOCK_REMOVAL", "STEWARD_SEAT_SALE"]:
        raise AmendmentError("protected_actions must be exact v1 set")
    chambers = profile["protected_chambers"]
    if not isinstance(chambers, list) or not chambers or len(set(chambers)) != len(chambers):
        raise AmendmentError("invalid protected_chambers")
    if "CAPITAL" in chambers:
        raise AmendmentError("capital chamber forbidden")
    for chamber in chambers:
        _str(chamber, "protected chamber")
    classes = profile["classes"]
    if not isinstance(classes, dict) or set(classes) != AMENDMENT_CLASSES:
        raise AmendmentError("amendment classes must be exact v1 set")
    for name, rule in classes.items():
        if not isinstance(rule, dict):
            raise AmendmentError("invalid amendment class rule")
        _exact_keys(rule, CLASS_KEYS, f"class:{name}")
        _bool(rule["prohibited"], f"{name}.prohibited")
        _bool(rule["enforcer_required"], f"{name}.enforcer_required")
        required = rule["required_chambers"]
        if not isinstance(required, dict) or not required:
            raise AmendmentError("invalid amendment required chambers")
        if not set(required).issubset(set(chambers)):
            raise AmendmentError("unknown amendment chamber")
        for chamber, thresholds in required.items():
            if not isinstance(thresholds, dict):
                raise AmendmentError("invalid amendment threshold")
            _exact_keys(thresholds, THRESHOLD_KEYS, f"threshold:{name}:{chamber}")
            _int(thresholds["quorum_ppm"], "quorum_ppm", PPM)
            _int(thresholds["approval_ppm"], "approval_ppm", PPM)
        allowed = rule["allowed_changed_paths"]
        if not isinstance(allowed, list) or len(set(allowed)) != len(allowed):
            raise AmendmentError("invalid allowed_changed_paths")
        for p in allowed:
            if not isinstance(p, str) or not p.startswith("/") or len(p) > 512:
                raise AmendmentError("invalid allowed changed path")
        if rule["prohibited"] and allowed:
            raise AmendmentError("prohibited class cannot allow changed paths")

def _g1_validation_context(profile: dict[str, Any]) -> dict[str, Any]:
    return {
        "expected_g1_profile_version": g2a.G1_PROFILE_VERSION,
        "project_id": profile["project_id"],
        "allowed_chambers": profile["protected_chambers"],
        "protected_actions": profile["protected_actions"],
    }

def validate_currentness_receipt(receipt: dict[str, Any], profile: dict[str, Any], prior_sha: str, prior_epoch: int) -> str:
    if not isinstance(receipt, dict):
        raise AmendmentError("currentness receipt must be object")
    _exact_keys(receipt, CURRENTNESS_RECEIPT_KEYS, "currentness receipt")
    if receipt["receipt_version"] != g2a.RECEIPT_VERSION or receipt["profile_version"] != g2a.PROFILE_VERSION:
        raise AmendmentError("currentness receipt version mismatch")
    if receipt["designation_state"] not in g2a.DESIGNATION_STATES:
        raise AmendmentError("invalid currentness designation state")
    if receipt["currentness_state"] not in g2a.CURRENTNESS_STATES:
        raise AmendmentError("invalid currentness state")
    if receipt["decision_authorization_state"] not in {"AUTHORIZED", "BLOCKED"}:
        raise AmendmentError("invalid prior decision authorization state")
    if receipt["project_id"] != profile["project_id"]:
        raise AmendmentError("currentness project substitution")
    if receipt["registry_id"] != profile["registry_id"]:
        raise AmendmentError("currentness registry substitution")
    if receipt["designation_epoch"] != prior_epoch:
        raise AmendmentError("currentness epoch substitution")
    if receipt["active_g1_profile_sha256"] != prior_sha:
        raise AmendmentError("currentness prior profile substitution")
    _hex(receipt["designation_sha256"], "currentness designation digest")
    _hex(receipt["g1_decision_receipt_sha256"], "currentness G1 receipt digest")
    _hex(receipt["revocation_set_sha256"], "currentness revocation digest")
    if receipt["execution_authority_established"] is not False:
        raise AmendmentError("currentness authority contamination: execution")
    if receipt["legal_validity_established"] is not False:
        raise AmendmentError("currentness authority contamination: legal")
    if receipt["democratic_legitimacy_established"] is not False:
        raise AmendmentError("currentness authority contamination: legitimacy")
    if not isinstance(receipt["nonclaims"], list):
        raise AmendmentError("invalid currentness nonclaims")
    return sha256_hex(receipt)

def _meets(numerator: int, denominator: int, threshold_ppm: int) -> bool:
    if denominator <= 0:
        return False
    return numerator * PPM >= threshold_ppm * denominator

def validate_authorization(
    auth: dict[str, Any],
    profile: dict[str, Any],
    prior_sha: str,
    successor_sha: str,
    prior_epoch: int,
    currentness_sha: str,
) -> list[str]:
    if not isinstance(auth, dict):
        raise AmendmentError("authorization must be object")
    _exact_keys(auth, AUTH_KEYS, "authorization")
    if auth["project_id"] != profile["project_id"]:
        raise AmendmentError("authorization project substitution")
    if auth["registry_id"] != profile["registry_id"]:
        raise AmendmentError("authorization registry substitution")
    if auth["prior_epoch"] != prior_epoch:
        raise AmendmentError("authorization prior epoch substitution")
    if auth["prior_profile_sha256"] != prior_sha:
        raise AmendmentError("authorization prior profile substitution")
    if auth["successor_profile_sha256"] != successor_sha:
        raise AmendmentError("authorization successor profile substitution")
    amendment_class = auth["amendment_class"]
    if amendment_class not in AMENDMENT_CLASSES:
        raise AmendmentError("unknown amendment class")
    if auth["currentness_receipt_sha256"] != currentness_sha:
        raise AmendmentError("authorization currentness substitution")
    _str(auth["decision_id"], "authorization.decision_id")
    _str(auth["notice_evidence_ref"], "authorization.notice_evidence_ref")
    rule = profile["classes"][amendment_class]
    blockers: list[str] = []
    if rule["prohibited"]:
        blockers.append(f"PROHIBITED_AMENDMENT_CLASS:{amendment_class}")

    votes = auth["chamber_votes"]
    if not isinstance(votes, dict):
        raise AmendmentError("invalid amendment chamber votes")
    required = set(rule["required_chambers"])
    if set(votes) != required:
        raise AmendmentError("amendment chamber set mismatch")
    for chamber in sorted(required):
        vote = votes[chamber]
        if not isinstance(vote, dict):
            raise AmendmentError("invalid amendment chamber vote")
        _exact_keys(vote, VOTE_KEYS, f"amendment vote:{chamber}")
        max_count = profile["max_vote_count"]
        eligible = _int(vote["eligible_count"], f"{chamber}.eligible_count", max_count)
        participating = _int(vote["participating_count"], f"{chamber}.participating_count", max_count)
        approvals = _int(vote["approval_count"], f"{chamber}.approval_count", max_count)
        if participating > eligible or approvals > participating:
            raise AmendmentError("impossible amendment chamber counts")
        thresholds = rule["required_chambers"][chamber]
        if not _meets(participating, eligible, thresholds["quorum_ppm"]):
            blockers.append(f"QUORUM_FAIL:{chamber}")
        if not _meets(approvals, participating, thresholds["approval_ppm"]):
            blockers.append(f"APPROVAL_FAIL:{chamber}")

    enforcer = auth["enforcer"]
    if rule["enforcer_required"]:
        if not isinstance(enforcer, dict):
            blockers.append("ENFORCER_MISSING")
        else:
            _exact_keys(enforcer, ENFORCER_KEYS, "authorization enforcer")
            if enforcer["authority_ref"] != profile["enforcer_ref"]:
                raise AmendmentError("amendment enforcer substitution")
            concurred = _bool(enforcer["concurred"], "authorization.enforcer.concurred")
            _str(enforcer["evidence_ref"], "authorization.enforcer.evidence_ref")
            if not concurred:
                blockers.append("ENFORCER_DID_NOT_CONCUR")
    elif enforcer is not None:
        raise AmendmentError("unexpected amendment enforcer")
    return sorted(set(blockers))

def validate_transition(
    transition: dict[str, Any],
    profile: dict[str, Any],
    prior_sha: str,
    successor_sha: str,
    amendment_class: str,
    prior_epoch: int,
) -> None:
    if not isinstance(transition, dict):
        raise AmendmentError("transition must be object")
    _exact_keys(transition, TRANSITION_KEYS, "transition")
    if transition["project_id"] != profile["project_id"]:
        raise AmendmentError("transition project substitution")
    if transition["registry_id"] != profile["registry_id"]:
        raise AmendmentError("transition registry substitution")
    if transition["prior_epoch"] != prior_epoch:
        raise AmendmentError("transition prior epoch substitution")
    if transition["successor_epoch"] != prior_epoch + 1:
        raise AmendmentError("successor epoch must equal prior epoch + 1")
    if transition["successor_epoch"] > profile["max_epoch"]:
        raise AmendmentError("successor epoch exceeds max")
    if transition["prior_profile_sha256"] != prior_sha:
        raise AmendmentError("transition prior profile substitution")
    if transition["successor_profile_sha256"] != successor_sha:
        raise AmendmentError("transition successor profile substitution")
    if transition["amendment_class"] != amendment_class:
        raise AmendmentError("transition amendment class substitution")
    _hex(transition["previous_transition_sha256"], "previous transition digest")
    _str(transition["evidence_ref"], "transition.evidence_ref")

def normalize_g1_profile_for_diff(profile: dict[str, Any]) -> dict[str, Any]:
    value = json.loads(json.dumps(profile))
    if isinstance(value.get("chambers"), list):
        value["chambers"] = sorted(value["chambers"])
    if isinstance(value.get("conflict_codes"), list):
        value["conflict_codes"] = sorted(value["conflict_codes"])
    actions = value.get("actions")
    if isinstance(actions, dict):
        for rule in actions.values():
            if isinstance(rule, dict) and isinstance(rule.get("required_recusal_conflicts"), list):
                rule["required_recusal_conflicts"] = sorted(rule["required_recusal_conflicts"])
    return value

@dataclass(frozen=True)
class Result:
    value: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return self.value

def qualify(case: dict[str, Any]) -> Result:
    if not isinstance(case, dict):
        raise AmendmentError("case must be object")
    _exact_keys(
        case,
        {
            "amendment_profile", "prior_g1_profile", "successor_g1_profile",
            "prior_currentness_receipt", "authorization", "transition",
        },
        "case",
    )
    profile = case["amendment_profile"]
    validate_profile(profile)
    context = _g1_validation_context(profile)

    try:
        prior_sha = g2a.validate_g1_profile(case["prior_g1_profile"], context)
        successor_sha = g2a.validate_g1_profile(case["successor_g1_profile"], context)
    except g2a.CurrentnessError as e:
        raise AmendmentError(str(e)) from e

    if prior_sha == successor_sha:
        raise AmendmentError("successor constitution must differ from prior")

    prior_epoch = case["transition"]["prior_epoch"] if isinstance(case["transition"], dict) and "prior_epoch" in case["transition"] else -1
    currentness_sha = validate_currentness_receipt(
        case["prior_currentness_receipt"], profile, prior_sha, prior_epoch
    )

    auth = case["authorization"]
    if not isinstance(auth, dict) or "amendment_class" not in auth:
        raise AmendmentError("authorization amendment class missing")
    amendment_class = auth["amendment_class"]

    blockers = validate_authorization(
        auth, profile, prior_sha, successor_sha, prior_epoch, currentness_sha
    )
    validate_transition(
        case["transition"], profile, prior_sha, successor_sha, amendment_class, prior_epoch
    )

    currentness_state = case["prior_currentness_receipt"]["currentness_state"]
    if currentness_state != "CURRENT":
        blockers.append(f"PRIOR_CURRENTNESS_NOT_CURRENT:{currentness_state}")

    changed_paths = semantic_changed_paths(
        normalize_g1_profile_for_diff(case["prior_g1_profile"]),
        normalize_g1_profile_for_diff(case["successor_g1_profile"]),
    )
    if not changed_paths:
        raise AmendmentError("empty constitutional change set")
    rule = profile["classes"][amendment_class]
    allowed = set(rule["allowed_changed_paths"])
    for path in changed_paths:
        if path not in allowed:
            blockers.append(f"UNAUTHORIZED_CHANGE:{path}")

    blockers = sorted(set(blockers))
    transition_state = "ACCEPTED" if not blockers else "BLOCKED"
    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": profile["project_id"],
        "registry_id": profile["registry_id"],
        "prior_epoch": case["transition"]["prior_epoch"],
        "successor_epoch": case["transition"]["successor_epoch"],
        "prior_profile_sha256": prior_sha,
        "successor_profile_sha256": successor_sha,
        "amendment_class": amendment_class,
        "computed_changed_paths": changed_paths,
        "currentness_receipt_sha256": currentness_sha,
        "prior_currentness_state": currentness_state,
        "authorization_sha256": sha256_hex(auth),
        "transition_sha256": sha256_hex(case["transition"]),
        "previous_transition_sha256": case["transition"]["previous_transition_sha256"],
        "blockers": blockers,
        "transition_state": transition_state,
        "successor_designation_established": False,
        "legal_validity_established": False,
        "democratic_legitimacy_established": False,
        "nonclaims": [
            "accepted amendment transition is not successor constitution designation",
            "amendment authorization does not establish legal or constitutional-law validity",
            "amendment authorization does not establish democratic legitimacy or social consensus",
            "aggregate amendment chamber counts are not independent ballot or identity reconstruction",
            "authority references are not external signature or identity authentication",
        ],
    }
    return Result(receipt)

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    args = ap.parse_args()
    case = json.loads(args.case.read_text(encoding="utf-8"))
    receipt = qualify(case).receipt()
    out = canonical_bytes(receipt)
    if args.receipt_out:
        args.receipt_out.write_bytes(out)
    else:
        print(out.decode("utf-8"), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
