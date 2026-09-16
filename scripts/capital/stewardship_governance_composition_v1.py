#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-stewardship-governance-composition-v1"
RECEIPT_VERSION = "mycelix-stewardship-governance-dimension-receipt-v1"
G2A_RECEIPT_VERSION = "mycelix-governance-decision-currentness-receipt-v1"
G2A_PROFILE_VERSION = "mycelix-governance-decision-currentness-v1"
G2C_RECEIPT_VERSION = "mycelix-constitution-designation-receipt-v1"
G2C_PROFILE_VERSION = "mycelix-constitution-designation-v1"
HEX64 = re.compile(r"^[0-9a-f]{64}$")

PROFILE_KEYS = {
    "profile_version", "composition_profile_id", "project_id", "registry_id",
    "allowed_action_codes",
}
G2A_KEYS = {
    "receipt_version", "profile_version", "project_id", "registry_id",
    "active_g1_profile_sha256", "currentness_state",
    "decision_authorization_state", "decision_id", "action_code",
    "designation_epoch", "designation_sha256", "designation_state",
    "g1_decision_receipt_sha256", "revocation_set_sha256",
    "execution_authority_established", "legal_validity_established",
    "democratic_legitimacy_established", "nonclaims",
}
G2C_KEYS = {
    "receipt_version", "profile_version", "project_id", "registry_id",
    "prior_checkpoint_sha256", "prior_chain_tip_sha256", "prior_epoch",
    "prior_profile_sha256", "amendment_receipt_sha256", "successor_epoch",
    "successor_profile_sha256", "designation_event_sha256",
    "designation_state", "blockers", "new_checkpoint",
    "prior_epoch_decisions_become_historical",
    "execution_authority_established", "legal_validity_established",
    "democratic_legitimacy_established", "nonclaims",
}
CHECKPOINT_KEYS = {
    "checkpoint_version", "project_id", "registry_id", "active_epoch",
    "active_profile_sha256", "chain_tip_sha256", "state",
}

class CompositionError(ValueError):
    pass

def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")

def sha256_hex(value: Any) -> str:
    raw = value if isinstance(value, (bytes, bytearray)) else canonical_bytes(value)
    return hashlib.sha256(raw).hexdigest()

def _exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    got = set(obj)
    if got != expected:
        raise CompositionError(f"{label} keys mismatch missing={sorted(expected-got)} unknown={sorted(got-expected)}")

def _text(v: Any, label: str) -> str:
    if not isinstance(v, str) or not v or len(v) > 512:
        raise CompositionError(f"invalid {label}")
    return v

def _hex(v: Any, label: str) -> str:
    if not isinstance(v, str) or not HEX64.fullmatch(v):
        raise CompositionError(f"invalid {label}")
    return v

def _false(v: Any, label: str) -> None:
    if v is not False:
        raise CompositionError(f"authority contamination:{label}")

def validate_profile(profile: dict[str, Any]) -> None:
    if not isinstance(profile, dict):
        raise CompositionError("composition profile must be object")
    _exact_keys(profile, PROFILE_KEYS, "composition profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise CompositionError("unsupported composition profile")
    _text(profile["composition_profile_id"], "composition_profile_id")
    _text(profile["project_id"], "project_id")
    _text(profile["registry_id"], "registry_id")
    actions = profile["allowed_action_codes"]
    if not isinstance(actions, list) or not actions or any(not isinstance(x, str) or not x for x in actions):
        raise CompositionError("invalid allowed_action_codes")
    if actions != sorted(set(actions)):
        raise CompositionError("allowed_action_codes must be sorted unique")

def validate_g2a(receipt: dict[str, Any], profile: dict[str, Any]) -> str:
    if not isinstance(receipt, dict):
        raise CompositionError("G2A receipt must be object")
    _exact_keys(receipt, G2A_KEYS, "G2A receipt")
    if receipt["receipt_version"] != G2A_RECEIPT_VERSION or receipt["profile_version"] != G2A_PROFILE_VERSION:
        raise CompositionError("G2A version mismatch")
    if receipt["project_id"] != profile["project_id"]:
        raise CompositionError("G2A project substitution")
    if receipt["registry_id"] != profile["registry_id"]:
        raise CompositionError("G2A registry substitution")
    for key in ["active_g1_profile_sha256", "designation_sha256", "g1_decision_receipt_sha256", "revocation_set_sha256"]:
        _hex(receipt[key], f"G2A {key}")
    if type(receipt["designation_epoch"]) is not int or receipt["designation_epoch"] < 0:
        raise CompositionError("invalid G2A designation_epoch")
    for key in ["decision_id", "action_code", "currentness_state", "decision_authorization_state", "designation_state"]:
        _text(receipt[key], f"G2A {key}")
    _false(receipt["execution_authority_established"], "G2A execution")
    _false(receipt["legal_validity_established"], "G2A legal")
    _false(receipt["democratic_legitimacy_established"], "G2A legitimacy")
    if not isinstance(receipt["nonclaims"], list):
        raise CompositionError("invalid G2A nonclaims")
    return sha256_hex(receipt)

def validate_g2c(receipt: dict[str, Any], profile: dict[str, Any]) -> tuple[str, dict[str, Any] | None]:
    if not isinstance(receipt, dict):
        raise CompositionError("G2C receipt must be object")
    _exact_keys(receipt, G2C_KEYS, "G2C receipt")
    if receipt["receipt_version"] != G2C_RECEIPT_VERSION or receipt["profile_version"] != G2C_PROFILE_VERSION:
        raise CompositionError("G2C version mismatch")
    if receipt["project_id"] != profile["project_id"]:
        raise CompositionError("G2C project substitution")
    if receipt["registry_id"] != profile["registry_id"]:
        raise CompositionError("G2C registry substitution")
    for key in [
        "prior_checkpoint_sha256", "prior_chain_tip_sha256", "prior_profile_sha256",
        "amendment_receipt_sha256", "successor_profile_sha256", "designation_event_sha256",
    ]:
        _hex(receipt[key], f"G2C {key}")
    for key in ["prior_epoch", "successor_epoch"]:
        if type(receipt[key]) is not int or receipt[key] < 0:
            raise CompositionError(f"invalid G2C {key}")
    _text(receipt["designation_state"], "G2C designation_state")
    if not isinstance(receipt["blockers"], list) or not isinstance(receipt["nonclaims"], list):
        raise CompositionError("invalid G2C lists")
    if type(receipt["prior_epoch_decisions_become_historical"]) is not bool:
        raise CompositionError("invalid G2C historical flag")
    _false(receipt["execution_authority_established"], "G2C execution")
    _false(receipt["legal_validity_established"], "G2C legal")
    _false(receipt["democratic_legitimacy_established"], "G2C legitimacy")
    cp = receipt["new_checkpoint"]
    if cp is not None:
        if not isinstance(cp, dict):
            raise CompositionError("invalid G2C checkpoint")
        _exact_keys(cp, CHECKPOINT_KEYS, "G2C checkpoint")
        if cp["checkpoint_version"] != "mycelix-constitution-checkpoint-v1":
            raise CompositionError("unsupported checkpoint version")
        if cp["project_id"] != profile["project_id"] or cp["registry_id"] != profile["registry_id"]:
            raise CompositionError("checkpoint substitution")
        if type(cp["active_epoch"]) is not int or cp["active_epoch"] < 0:
            raise CompositionError("invalid checkpoint epoch")
        _hex(cp["active_profile_sha256"], "checkpoint profile")
        _hex(cp["chain_tip_sha256"], "checkpoint chain tip")
        _text(cp["state"], "checkpoint state")
    return sha256_hex(receipt), cp

def qualify(case: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(case, dict):
        raise CompositionError("case must be object")
    _exact_keys(case, {"composition_profile", "g2a_receipt", "g2c_receipt"}, "case")
    profile = case["composition_profile"]
    validate_profile(profile)
    g2a = case["g2a_receipt"]
    g2c = case["g2c_receipt"]
    g2a_sha = validate_g2a(g2a, profile)
    g2c_sha, cp = validate_g2c(g2c, profile)

    blockers: list[str] = []
    state = "CURRENT"

    if g2c["designation_state"] == "STALE":
        state = "STALE"
        blockers.append("CONSTITUTION_CHECKPOINT_STALE")
    elif g2c["designation_state"] != "ACTIVE":
        state = "BLOCKED"
        blockers.append("CONSTITUTION_NOT_ACTIVE")
    elif cp is None or cp["state"] != "ACTIVE":
        raise CompositionError("ACTIVE G2C must carry ACTIVE checkpoint")

    if g2a["action_code"] not in profile["allowed_action_codes"]:
        state = "UNSUPPORTED"
        blockers.append("ACTION_NOT_SUPPORTED_BY_COMPOSITION_PROFILE")
    elif g2a["decision_authorization_state"] != "AUTHORIZED":
        state = "BLOCKED"
        blockers.append("GOVERNANCE_DECISION_NOT_AUTHORIZED")
    elif g2a["currentness_state"] == "HISTORICAL":
        state = "STALE"
        blockers.append("GOVERNANCE_DECISION_HISTORICAL")
    elif g2a["currentness_state"] == "REVOKED":
        state = "BLOCKED"
        blockers.append("GOVERNANCE_DECISION_REVOKED")
    elif g2a["currentness_state"] == "PENDING":
        state = "PENDING"
        blockers.append("GOVERNANCE_DECISION_PENDING")
    elif g2a["currentness_state"] != "CURRENT":
        state = "NOT_ASSESSED"
        blockers.append("GOVERNANCE_DECISION_NOT_ASSESSED")

    if state == "CURRENT":
        if g2a["designation_state"] != "ACTIVE":
            state = "BLOCKED"
            blockers.append("GOVERNANCE_DESIGNATION_NOT_ACTIVE")
        if cp is None:
            raise CompositionError("CURRENT composition requires checkpoint")
        if g2a["designation_epoch"] != cp["active_epoch"]:
            state = "STALE"
            blockers.append("GOVERNANCE_EPOCH_MISMATCH")
        if g2a["active_g1_profile_sha256"] != cp["active_profile_sha256"]:
            state = "STALE"
            blockers.append("GOVERNANCE_PROFILE_MISMATCH")

    blockers = sorted(set(blockers))
    checkpoint_sha = sha256_hex(cp) if cp is not None else None

    return {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "composition_profile_id": profile["composition_profile_id"],
        "project_id": profile["project_id"],
        "registry_id": profile["registry_id"],
        "g2c_receipt_sha256": g2c_sha,
        "checkpoint_sha256": checkpoint_sha,
        "active_epoch": cp["active_epoch"] if cp is not None else None,
        "active_profile_sha256": cp["active_profile_sha256"] if cp is not None else None,
        "g2a_receipt_sha256": g2a_sha,
        "decision_id": g2a["decision_id"],
        "action_code": g2a["action_code"],
        "governance_state": state,
        "blockers": blockers,
        "execution_authority_established": False,
        "legal_validity_established": False,
        "democratic_legitimacy_established": False,
        "nonclaims": [
            "CURRENT establishes only composition of one current authorized decision with one exact active constitution checkpoint",
            "composition does not establish execution authority",
            "composition does not establish legal or constitutional-law validity",
            "composition does not establish democratic legitimacy or social consensus",
            "composition does not establish service quality, solvency, handback readiness, or operational sovereignty",
        ],
    }

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    args = ap.parse_args()
    case = json.loads(args.case.read_text(encoding="utf-8"))
    out = canonical_bytes(qualify(case))
    if args.receipt_out:
        args.receipt_out.write_bytes(out)
    else:
        print(out.decode("utf-8"), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
