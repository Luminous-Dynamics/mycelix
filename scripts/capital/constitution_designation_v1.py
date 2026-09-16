#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-constitution-designation-v1"
RECEIPT_VERSION = "mycelix-constitution-designation-receipt-v1"
G2B_RECEIPT_VERSION = "mycelix-constitution-amendment-receipt-v1"
G2B_PROFILE_VERSION = "mycelix-constitution-amendment-v1"
G1_PROFILE_VERSION = "mycelix-stewardship-decision-gate-v1"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
STATES = {"ACTIVE", "BLOCKED", "STALE"}

PROFILE_KEYS = {
    "profile_version", "project_id", "registry_id", "designation_authority_ref",
    "max_epoch", "protected_chambers", "protected_actions",
}
CHECKPOINT_KEYS = {
    "checkpoint_version", "project_id", "registry_id", "active_epoch",
    "active_profile_sha256", "chain_tip_sha256", "state",
}
EVENT_KEYS = {
    "event_id", "project_id", "registry_id", "expected_current_checkpoint_sha256",
    "expected_current_chain_tip_sha256", "amendment_receipt_sha256",
    "successor_profile_sha256", "authority_ref", "evidence_ref",
}
G2B_KEYS = {
    "receipt_version", "profile_version", "project_id", "registry_id", "prior_epoch",
    "successor_epoch", "prior_profile_sha256", "successor_profile_sha256",
    "amendment_class", "computed_changed_paths", "currentness_receipt_sha256",
    "prior_currentness_state", "authorization_sha256", "transition_sha256",
    "previous_transition_sha256", "blockers", "transition_state",
    "successor_designation_established", "legal_validity_established",
    "democratic_legitimacy_established", "nonclaims",
}
G1_PROFILE_KEYS = {
    "profile_version", "project_id", "max_vote_count", "enforcer_ref",
    "emergency_authority_ref", "chambers", "conflict_codes", "actions",
}
ACTION_KEYS = {
    "decision_class", "prohibited", "required_chambers", "enforcer_required",
    "emergency_required", "required_recusal_conflicts",
}

class DesignationError(ValueError):
    pass

def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")

def sha256_hex(value: Any) -> str:
    raw = value if isinstance(value, (bytes, bytearray)) else canonical_bytes(value)
    return hashlib.sha256(raw).hexdigest()

def _exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    got = set(obj)
    if got != expected:
        raise DesignationError(f"{label} keys mismatch missing={sorted(expected-got)} unknown={sorted(got-expected)}")

def _str(v: Any, label: str) -> str:
    if not isinstance(v, str) or not v or len(v) > 512:
        raise DesignationError(f"invalid {label}")
    return v

def _int(v: Any, label: str, maximum: int) -> int:
    if type(v) is not int or v < 0 or v > maximum:
        raise DesignationError(f"invalid {label}")
    return v

def _hex(v: Any, label: str) -> str:
    if not isinstance(v, str) or not HEX64.fullmatch(v):
        raise DesignationError(f"invalid {label}")
    return v

def validate_profile(profile: dict[str, Any]) -> None:
    if not isinstance(profile, dict):
        raise DesignationError("designation profile must be object")
    _exact_keys(profile, PROFILE_KEYS, "designation profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise DesignationError("unsupported designation profile")
    _str(profile["project_id"], "profile.project_id")
    _str(profile["registry_id"], "profile.registry_id")
    _str(profile["designation_authority_ref"], "profile.designation_authority_ref")
    if _int(profile["max_epoch"], "profile.max_epoch", 2**31 - 1) == 0:
        raise DesignationError("max_epoch must be positive")
    chambers = profile["protected_chambers"]
    if chambers != ["USERS", "WORKERS", "PUBLIC", "GUARDIAN"]:
        raise DesignationError("protected_chambers must be exact v1 set")
    if profile["protected_actions"] != ["ASSET_LOCK_REMOVAL", "STEWARD_SEAT_SALE"]:
        raise DesignationError("protected_actions must be exact v1 set")

def validate_checkpoint(checkpoint: dict[str, Any], profile: dict[str, Any]) -> str:
    if not isinstance(checkpoint, dict):
        raise DesignationError("checkpoint must be object")
    _exact_keys(checkpoint, CHECKPOINT_KEYS, "checkpoint")
    if checkpoint["checkpoint_version"] != "mycelix-constitution-checkpoint-v1":
        raise DesignationError("unsupported checkpoint version")
    if checkpoint["project_id"] != profile["project_id"]:
        raise DesignationError("checkpoint project substitution")
    if checkpoint["registry_id"] != profile["registry_id"]:
        raise DesignationError("checkpoint registry substitution")
    _int(checkpoint["active_epoch"], "checkpoint.active_epoch", profile["max_epoch"])
    _hex(checkpoint["active_profile_sha256"], "checkpoint active profile digest")
    _hex(checkpoint["chain_tip_sha256"], "checkpoint chain tip")
    if checkpoint["state"] != "ACTIVE":
        raise DesignationError("current checkpoint must be ACTIVE")
    return sha256_hex(checkpoint)

def validate_g2b(receipt: dict[str, Any], profile: dict[str, Any]) -> str:
    if not isinstance(receipt, dict):
        raise DesignationError("amendment receipt must be object")
    _exact_keys(receipt, G2B_KEYS, "amendment receipt")
    if receipt["receipt_version"] != G2B_RECEIPT_VERSION or receipt["profile_version"] != G2B_PROFILE_VERSION:
        raise DesignationError("amendment receipt version mismatch")
    if receipt["project_id"] != profile["project_id"]:
        raise DesignationError("amendment project substitution")
    if receipt["registry_id"] != profile["registry_id"]:
        raise DesignationError("amendment registry substitution")
    _int(receipt["prior_epoch"], "amendment prior epoch", profile["max_epoch"])
    _int(receipt["successor_epoch"], "amendment successor epoch", profile["max_epoch"])
    for key in ["prior_profile_sha256", "successor_profile_sha256", "currentness_receipt_sha256", "authorization_sha256", "transition_sha256", "previous_transition_sha256"]:
        _hex(receipt[key], f"amendment {key}")
    if not isinstance(receipt["computed_changed_paths"], list):
        raise DesignationError("invalid amendment changed paths")
    if not isinstance(receipt["blockers"], list):
        raise DesignationError("invalid amendment blockers")
    if receipt["transition_state"] not in {"ACCEPTED", "BLOCKED"}:
        raise DesignationError("invalid amendment transition state")
    if type(receipt["successor_designation_established"]) is not bool:
        raise DesignationError("invalid amendment designation flag")
    if receipt["legal_validity_established"] is not False or receipt["democratic_legitimacy_established"] is not False:
        raise DesignationError("amendment authority contamination")
    if not isinstance(receipt["nonclaims"], list):
        raise DesignationError("invalid amendment nonclaims")
    return sha256_hex(receipt)

def validate_successor_profile(g1: dict[str, Any], profile: dict[str, Any]) -> str:
    if not isinstance(g1, dict):
        raise DesignationError("successor G1 profile must be object")
    _exact_keys(g1, G1_PROFILE_KEYS, "successor G1 profile")
    if g1["profile_version"] != G1_PROFILE_VERSION:
        raise DesignationError("successor G1 version mismatch")
    if g1["project_id"] != profile["project_id"]:
        raise DesignationError("successor G1 project substitution")
    chambers = g1["chambers"]
    if not isinstance(chambers, list) or len(set(chambers)) != len(chambers):
        raise DesignationError("invalid successor chambers")
    if chambers != profile["protected_chambers"] or "CAPITAL" in chambers:
        raise DesignationError("protected chamber topology changed")
    actions = g1["actions"]
    if not isinstance(actions, dict):
        raise DesignationError("invalid successor actions")
    for action in profile["protected_actions"]:
        rule = actions.get(action)
        if not isinstance(rule, dict):
            raise DesignationError(f"missing protected action:{action}")
        _exact_keys(rule, ACTION_KEYS, f"protected action:{action}")
        if rule["decision_class"] != "CONSTITUTIONAL" or rule["prohibited"] is not True or rule["enforcer_required"] is not True:
            raise DesignationError(f"protected action weakened:{action}")
        if rule["emergency_required"] is not False:
            raise DesignationError(f"protected action emergency contamination:{action}")
    return sha256_hex(g1)

def validate_event(event: dict[str, Any], profile: dict[str, Any], amendment_sha: str, successor_sha: str, receipt_successor_epoch: int) -> str:
    if not isinstance(event, dict):
        raise DesignationError("designation event must be object")
    _exact_keys(event, EVENT_KEYS, "designation event")
    event_id = _str(event["event_id"], "event_id")
    expected_event_id = f"designation:{receipt_successor_epoch}:{amendment_sha[:16]}"
    if event_id != expected_event_id:
        raise DesignationError("designation event id is not deterministic for amendment")
    if event["project_id"] != profile["project_id"]:
        raise DesignationError("event project substitution")
    if event["registry_id"] != profile["registry_id"]:
        raise DesignationError("event registry substitution")
    _hex(event["expected_current_checkpoint_sha256"], "expected checkpoint digest")
    _hex(event["expected_current_chain_tip_sha256"], "expected chain tip")
    if event["amendment_receipt_sha256"] != amendment_sha:
        raise DesignationError("amendment receipt substitution")
    if event["successor_profile_sha256"] != successor_sha:
        raise DesignationError("successor profile substitution")
    if event["authority_ref"] != profile["designation_authority_ref"]:
        raise DesignationError("designation authority substitution")
    _str(event["evidence_ref"], "event.evidence_ref")
    return sha256_hex(event)

@dataclass(frozen=True)
class Result:
    value: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return self.value

def qualify(case: dict[str, Any]) -> Result:
    if not isinstance(case, dict):
        raise DesignationError("case must be object")
    _exact_keys(case, {"designation_profile", "live_checkpoint", "amendment_receipt", "successor_g1_profile", "designation_event"}, "case")
    profile = case["designation_profile"]
    validate_profile(profile)
    live = case["live_checkpoint"]
    live_sha = validate_checkpoint(live, profile)
    amendment = case["amendment_receipt"]
    amendment_sha = validate_g2b(amendment, profile)
    successor_sha = validate_successor_profile(case["successor_g1_profile"], profile)
    event_sha = validate_event(case["designation_event"], profile, amendment_sha, successor_sha, amendment["successor_epoch"])
    event = case["designation_event"]

    stale = (
        event["expected_current_checkpoint_sha256"] != live_sha
        or event["expected_current_chain_tip_sha256"] != live["chain_tip_sha256"]
        or amendment["prior_epoch"] != live["active_epoch"]
        or amendment["prior_profile_sha256"] != live["active_profile_sha256"]
    )
    blockers: list[str] = []
    if amendment["transition_state"] != "ACCEPTED":
        blockers.append("AMENDMENT_NOT_ACCEPTED")
    if amendment["blockers"]:
        blockers.append("AMENDMENT_HAS_BLOCKERS")
    if amendment["successor_designation_established"] is not False:
        blockers.append("AMENDMENT_ALREADY_CLAIMS_DESIGNATION")
    if amendment["successor_epoch"] != amendment["prior_epoch"] + 1:
        blockers.append("AMENDMENT_EPOCH_NOT_CONTIGUOUS")
    if amendment["successor_profile_sha256"] != successor_sha:
        raise DesignationError("successor profile digest differs from amendment")
    if amendment["successor_epoch"] > profile["max_epoch"]:
        raise DesignationError("successor epoch exceeds max")

    if stale:
        designation_state = "STALE"
        blockers = []
    elif blockers:
        designation_state = "BLOCKED"
        blockers = sorted(set(blockers))
    else:
        designation_state = "ACTIVE"

    new_checkpoint = None
    if designation_state == "ACTIVE":
        checkpoint_material = {
            "checkpoint_version": "mycelix-constitution-checkpoint-v1",
            "project_id": profile["project_id"],
            "registry_id": profile["registry_id"],
            "active_epoch": amendment["successor_epoch"],
            "active_profile_sha256": successor_sha,
            "previous_checkpoint_sha256": live_sha,
            "previous_chain_tip_sha256": live["chain_tip_sha256"],
            "amendment_receipt_sha256": amendment_sha,
            "designation_event_sha256": event_sha,
        }
        new_tip = sha256_hex(checkpoint_material)
        new_checkpoint = {
            "checkpoint_version": "mycelix-constitution-checkpoint-v1",
            "project_id": profile["project_id"],
            "registry_id": profile["registry_id"],
            "active_epoch": amendment["successor_epoch"],
            "active_profile_sha256": successor_sha,
            "chain_tip_sha256": new_tip,
            "state": "ACTIVE",
        }

    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": profile["project_id"],
        "registry_id": profile["registry_id"],
        "prior_checkpoint_sha256": live_sha,
        "prior_chain_tip_sha256": live["chain_tip_sha256"],
        "prior_epoch": live["active_epoch"],
        "prior_profile_sha256": live["active_profile_sha256"],
        "amendment_receipt_sha256": amendment_sha,
        "successor_epoch": amendment["successor_epoch"],
        "successor_profile_sha256": successor_sha,
        "designation_event_sha256": event_sha,
        "designation_state": designation_state,
        "blockers": blockers,
        "new_checkpoint": new_checkpoint,
        "prior_epoch_decisions_become_historical": designation_state == "ACTIVE",
        "execution_authority_established": False,
        "legal_validity_established": False,
        "democratic_legitimacy_established": False,
        "nonclaims": [
            "designation ACTIVE is not execution authority",
            "designation does not establish legal or constitutional-law validity",
            "designation does not establish democratic legitimacy or social consensus",
            "STALE means compare-and-swap currentness conflict, not semantic invalidity of the amendment",
            "designation authority references are not external identity or signature authentication",
        ],
    }
    return Result(receipt)

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    args = ap.parse_args()
    case = json.loads(args.case.read_text(encoding="utf-8"))
    out = canonical_bytes(qualify(case).receipt())
    if args.receipt_out:
        args.receipt_out.write_bytes(out)
    else:
        print(out.decode("utf-8"), end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
