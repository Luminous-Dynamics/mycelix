#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-governance-decision-currentness-v1"
RECEIPT_VERSION = "mycelix-governance-decision-currentness-receipt-v1"
G1_PROFILE_VERSION = "mycelix-stewardship-decision-gate-v1"
G1_RECEIPT_VERSION = "mycelix-stewardship-decision-receipt-v1"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
DESIGNATION_STATES = {"ACTIVE", "PENDING", "REVOKED"}
CURRENTNESS_STATES = {"CURRENT", "HISTORICAL", "REVOKED", "PENDING", "NOT_ASSESSED"}

PROFILE_KEYS = {
    "profile_version", "project_id", "registry_id", "designation_authority_ref",
    "revocation_authority_ref", "max_epoch", "expected_g1_profile_version",
    "expected_g1_receipt_version", "allowed_chambers", "protected_actions",
}
DESIGNATION_KEYS = {
    "project_id", "registry_id", "epoch", "active_profile_sha256",
    "state", "authority_ref", "evidence_ref",
}
REVOCATION_KEYS = {
    "project_id", "registry_id", "epoch", "decision_receipt_sha256",
    "authority_ref", "reason_code", "evidence_ref",
}
G1_PROFILE_KEYS = {
    "profile_version", "project_id", "max_vote_count", "enforcer_ref",
    "emergency_authority_ref", "chambers", "conflict_codes", "actions",
}
G1_ACTION_KEYS = {
    "decision_class", "prohibited", "required_chambers", "enforcer_required",
    "emergency_required", "required_recusal_conflicts",
}
G1_THRESHOLD_KEYS = {"quorum_ppm", "approval_ppm"}
G1_RECEIPT_KEYS = {
    "receipt_version", "profile_version", "project_id", "profile_sha256",
    "decision_id", "decision_sha256", "action_code", "decision_class",
    "chamber_results", "conflict_results", "enforcer_result",
    "emergency_result", "blockers", "authorization_state",
    "asset_lock_removed", "legal_validity_established",
    "democratic_legitimacy_established", "nonclaims",
}

class CurrentnessError(ValueError):
    pass

def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")

def sha256_hex(value: Any) -> str:
    raw = value if isinstance(value, (bytes, bytearray)) else canonical_bytes(value)
    return hashlib.sha256(raw).hexdigest()

def _exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    got = set(obj)
    if got != expected:
        raise CurrentnessError(f"{label} keys mismatch missing={sorted(expected-got)} unknown={sorted(got-expected)}")

def _str(v: Any, label: str) -> str:
    if not isinstance(v, str) or not v or len(v) > 256:
        raise CurrentnessError(f"invalid {label}")
    return v

def _bool(v: Any, label: str) -> bool:
    if type(v) is not bool:
        raise CurrentnessError(f"invalid {label}")
    return v

def _int(v: Any, label: str, maximum: int) -> int:
    if type(v) is not int or v < 0 or v > maximum:
        raise CurrentnessError(f"invalid {label}")
    return v

def _hex(v: Any, label: str) -> str:
    if not isinstance(v, str) or not HEX64.fullmatch(v):
        raise CurrentnessError(f"invalid {label}")
    return v

def validate_currentness_profile(profile: dict[str, Any]) -> None:
    if not isinstance(profile, dict):
        raise CurrentnessError("profile must be object")
    _exact_keys(profile, PROFILE_KEYS, "profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise CurrentnessError("unsupported currentness profile version")
    _str(profile["project_id"], "profile.project_id")
    _str(profile["registry_id"], "profile.registry_id")
    _str(profile["designation_authority_ref"], "profile.designation_authority_ref")
    _str(profile["revocation_authority_ref"], "profile.revocation_authority_ref")
    if _int(profile["max_epoch"], "profile.max_epoch", 2**31 - 1) == 0:
        raise CurrentnessError("max_epoch must be positive")
    if profile["expected_g1_profile_version"] != G1_PROFILE_VERSION:
        raise CurrentnessError("unexpected G1 profile version")
    if profile["expected_g1_receipt_version"] != G1_RECEIPT_VERSION:
        raise CurrentnessError("unexpected G1 receipt version")
    chambers = profile["allowed_chambers"]
    if not isinstance(chambers, list) or not chambers or len(chambers) > 32:
        raise CurrentnessError("invalid allowed_chambers")
    if len(set(chambers)) != len(chambers):
        raise CurrentnessError("duplicate allowed chamber")
    for chamber in chambers:
        _str(chamber, "allowed chamber")
        if chamber == "CAPITAL":
            raise CurrentnessError("CAPITAL chamber forbidden")
    protected = profile["protected_actions"]
    if protected != ["ASSET_LOCK_REMOVAL", "STEWARD_SEAT_SALE"]:
        raise CurrentnessError("protected_actions must be exact v1 set")

def validate_g1_profile(profile: dict[str, Any], currentness_profile: dict[str, Any]) -> str:
    if not isinstance(profile, dict):
        raise CurrentnessError("designated G1 profile must be object")
    _exact_keys(profile, G1_PROFILE_KEYS, "g1 profile")
    if profile["profile_version"] != currentness_profile["expected_g1_profile_version"]:
        raise CurrentnessError("G1 profile version mismatch")
    if profile["project_id"] != currentness_profile["project_id"]:
        raise CurrentnessError("G1 project substitution")
    _int(profile["max_vote_count"], "g1.max_vote_count", 10**9)
    _str(profile["enforcer_ref"], "g1.enforcer_ref")
    _str(profile["emergency_authority_ref"], "g1.emergency_authority_ref")
    chambers = profile["chambers"]
    if not isinstance(chambers, list) or len(set(chambers)) != len(chambers):
        raise CurrentnessError("invalid G1 chambers")
    for chamber in chambers:
        _str(chamber, "g1 chamber")
        if chamber == "CAPITAL":
            raise CurrentnessError("capital chamber forbidden")
    if set(chambers) != set(currentness_profile["allowed_chambers"]):
        raise CurrentnessError("protected chamber topology changed")
    conflicts = profile["conflict_codes"]
    if not isinstance(conflicts, list) or len(set(conflicts)) != len(conflicts):
        raise CurrentnessError("invalid G1 conflict codes")
    for code in conflicts:
        _str(code, "g1 conflict code")
    actions = profile["actions"]
    if not isinstance(actions, dict) or not actions:
        raise CurrentnessError("invalid G1 actions")
    for action_code, rule in actions.items():
        _str(action_code, "g1 action code")
        if not isinstance(rule, dict):
            raise CurrentnessError("invalid G1 action rule")
        _exact_keys(rule, G1_ACTION_KEYS, f"g1 action:{action_code}")
        if rule["decision_class"] not in {"ORDINARY", "CONSTITUTIONAL", "EMERGENCY"}:
            raise CurrentnessError("invalid G1 decision class")
        _bool(rule["prohibited"], "g1 prohibited")
        _bool(rule["enforcer_required"], "g1 enforcer_required")
        _bool(rule["emergency_required"], "g1 emergency_required")
        required = rule["required_chambers"]
        if not isinstance(required, dict) or not required:
            raise CurrentnessError("invalid G1 required chambers")
        if not set(required).issubset(set(chambers)):
            raise CurrentnessError("unknown G1 required chamber")
        for chamber, thresholds in required.items():
            if not isinstance(thresholds, dict):
                raise CurrentnessError("invalid G1 threshold")
            _exact_keys(thresholds, G1_THRESHOLD_KEYS, f"g1 threshold:{action_code}:{chamber}")
            for name in G1_THRESHOLD_KEYS:
                _int(thresholds[name], f"g1 {name}", 1_000_000)
        recusals = rule["required_recusal_conflicts"]
        if not isinstance(recusals, list) or len(set(recusals)) != len(recusals):
            raise CurrentnessError("invalid G1 recusal list")
        if not set(recusals).issubset(set(conflicts)):
            raise CurrentnessError("unknown G1 recusal conflict")
    for protected_action in currentness_profile["protected_actions"]:
        if protected_action not in actions:
            raise CurrentnessError(f"protected action missing:{protected_action}")
        rule = actions[protected_action]
        if rule["prohibited"] is not True:
            raise CurrentnessError(f"protected action unprohibited:{protected_action}")
        if rule["decision_class"] != "CONSTITUTIONAL":
            raise CurrentnessError(f"protected action class weakened:{protected_action}")
        if rule["enforcer_required"] is not True:
            raise CurrentnessError(f"protected action enforcer weakened:{protected_action}")
    return sha256_hex(profile)

def validate_designation(designation: dict[str, Any], profile: dict[str, Any], active_profile_sha: str) -> None:
    if not isinstance(designation, dict):
        raise CurrentnessError("designation must be object")
    _exact_keys(designation, DESIGNATION_KEYS, "designation")
    if designation["project_id"] != profile["project_id"]:
        raise CurrentnessError("designation project substitution")
    if designation["registry_id"] != profile["registry_id"]:
        raise CurrentnessError("designation registry substitution")
    _int(designation["epoch"], "designation.epoch", profile["max_epoch"])
    if designation["active_profile_sha256"] != active_profile_sha:
        raise CurrentnessError("designation profile digest mismatch")
    if designation["state"] not in DESIGNATION_STATES:
        raise CurrentnessError("invalid designation state")
    if designation["authority_ref"] != profile["designation_authority_ref"]:
        raise CurrentnessError("designation authority substitution")
    _str(designation["evidence_ref"], "designation.evidence_ref")

def validate_g1_receipt(receipt: dict[str, Any], profile: dict[str, Any]) -> str:
    if not isinstance(receipt, dict):
        raise CurrentnessError("G1 receipt must be object")
    _exact_keys(receipt, G1_RECEIPT_KEYS, "g1 receipt")
    if receipt["receipt_version"] != profile["expected_g1_receipt_version"]:
        raise CurrentnessError("G1 receipt version mismatch")
    if receipt["profile_version"] != profile["expected_g1_profile_version"]:
        raise CurrentnessError("G1 receipt profile version mismatch")
    if receipt["project_id"] != profile["project_id"]:
        raise CurrentnessError("G1 receipt project substitution")
    _hex(receipt["profile_sha256"], "g1 receipt profile_sha256")
    _str(receipt["decision_id"], "g1 receipt decision_id")
    _hex(receipt["decision_sha256"], "g1 receipt decision_sha256")
    _str(receipt["action_code"], "g1 receipt action_code")
    if receipt["decision_class"] not in {"ORDINARY", "CONSTITUTIONAL", "EMERGENCY"}:
        raise CurrentnessError("invalid G1 receipt decision class")
    if not isinstance(receipt["chamber_results"], dict):
        raise CurrentnessError("invalid G1 chamber_results")
    if not isinstance(receipt["conflict_results"], list):
        raise CurrentnessError("invalid G1 conflict_results")
    if not isinstance(receipt["blockers"], list):
        raise CurrentnessError("invalid G1 blockers")
    if receipt["authorization_state"] not in {"AUTHORIZED", "BLOCKED"}:
        raise CurrentnessError("invalid G1 authorization state")
    if receipt["asset_lock_removed"] is not False:
        raise CurrentnessError("G1 authority contamination: asset lock")
    if receipt["legal_validity_established"] is not False:
        raise CurrentnessError("G1 authority contamination: legal validity")
    if receipt["democratic_legitimacy_established"] is not False:
        raise CurrentnessError("G1 authority contamination: democratic legitimacy")
    if not isinstance(receipt["nonclaims"], list):
        raise CurrentnessError("invalid G1 nonclaims")
    return sha256_hex(receipt)

def normalize_revocations(revocations: Any, profile: dict[str, Any], designation_epoch: int) -> list[dict[str, Any]]:
    if not isinstance(revocations, list) or len(revocations) > 512:
        raise CurrentnessError("invalid revocations")
    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in revocations:
        if not isinstance(item, dict):
            raise CurrentnessError("invalid revocation")
        _exact_keys(item, REVOCATION_KEYS, "revocation")
        if item["project_id"] != profile["project_id"]:
            raise CurrentnessError("revocation project substitution")
        if item["registry_id"] != profile["registry_id"]:
            raise CurrentnessError("revocation registry substitution")
        if _int(item["epoch"], "revocation.epoch", profile["max_epoch"]) != designation_epoch:
            raise CurrentnessError("revocation epoch substitution")
        digest = _hex(item["decision_receipt_sha256"], "revocation decision digest")
        if digest in seen:
            raise CurrentnessError("duplicate decision revocation")
        seen.add(digest)
        if item["authority_ref"] != profile["revocation_authority_ref"]:
            raise CurrentnessError("revocation authority substitution")
        _str(item["reason_code"], "revocation.reason_code")
        _str(item["evidence_ref"], "revocation.evidence_ref")
        normalized.append(dict(item))
    normalized.sort(key=lambda x: x["decision_receipt_sha256"])
    return normalized

@dataclass(frozen=True)
class Result:
    value: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return self.value

def derive(case: dict[str, Any]) -> Result:
    if not isinstance(case, dict):
        raise CurrentnessError("case must be object")
    _exact_keys(
        case,
        {"currentness_profile", "designated_g1_profile", "designation", "g1_receipt", "revocations"},
        "case",
    )
    profile = case["currentness_profile"]
    validate_currentness_profile(profile)
    designated_profile_sha = validate_g1_profile(case["designated_g1_profile"], profile)
    designation = case["designation"]
    validate_designation(designation, profile, designated_profile_sha)
    g1_receipt_sha = validate_g1_receipt(case["g1_receipt"], profile)
    revocations = normalize_revocations(case["revocations"], profile, designation["epoch"])

    designation_state = designation["state"]
    receipt = case["g1_receipt"]
    if designation_state == "PENDING":
        state = "PENDING"
    elif designation_state == "REVOKED":
        state = "NOT_ASSESSED"
    elif receipt["authorization_state"] != "AUTHORIZED":
        state = "NOT_ASSESSED"
    elif any(r["decision_receipt_sha256"] == g1_receipt_sha for r in revocations):
        state = "REVOKED"
    elif receipt["profile_sha256"] != designated_profile_sha:
        state = "HISTORICAL"
    else:
        state = "CURRENT"

    if state not in CURRENTNESS_STATES:
        raise CurrentnessError("internal currentness state error")

    output = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": profile["project_id"],
        "registry_id": profile["registry_id"],
        "designation_epoch": designation["epoch"],
        "designation_state": designation_state,
        "designation_sha256": sha256_hex(designation),
        "active_g1_profile_sha256": designated_profile_sha,
        "g1_decision_receipt_sha256": g1_receipt_sha,
        "decision_id": receipt["decision_id"],
        "action_code": receipt["action_code"],
        "decision_authorization_state": receipt["authorization_state"],
        "revocation_set_sha256": sha256_hex(revocations),
        "currentness_state": state,
        "execution_authority_established": False,
        "legal_validity_established": False,
        "democratic_legitimacy_established": False,
        "nonclaims": [
            "CURRENT means only current relative to this exact designated constitution and revocation set",
            "currentness does not establish execution authority",
            "currentness does not establish legal or constitutional-law validity",
            "currentness does not establish democratic legitimacy or social consensus",
            "designation validity does not prove amendment authorization or authority authenticity",
        ],
    }
    return Result(output)

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    args = ap.parse_args()
    case = json.loads(args.case.read_text(encoding="utf-8"))
    receipt = derive(case).receipt()
    out = canonical_bytes(receipt)
    if args.receipt_out:
        args.receipt_out.write_bytes(out)
    else:
        print(out.decode("utf-8"), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
