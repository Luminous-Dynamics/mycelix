#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-public-service-composition-v1"
RECEIPT_VERSION = "mycelix-public-service-composition-receipt-v1"
F_RECEIPT_VERSION = "mycelix-public-service-gate-receipt-v1"
F1_RECEIPT_VERSION = "mycelix-service-currentness-receipt-v1"

HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")

PROFILE_KEYS = {"profile_version","project_id","service_gate_subject_sha","service_currentness_subject_sha","service_gate_profile_sha256"}
F_KEYS = {"receipt_version","profile_version","project_id","unit","profile_sha256","parent_subject_sha","parent_transition_receipt_sha256","measurement_id","measurement_epoch","snapshot_sha256","distribution_eligibility","blockers","parent_remaining_claim_units","claim_modified","observed","nonclaims"}
F1_KEYS = {"receipt_version","profile_version","project_id","profile_sha256","service_gate_subject_sha","service_gate_profile_sha256","service_receipt_sha256","measurement_id","service_distribution_eligibility","designation_id","designation_registry_id","designation_registry_epoch","designation_sha256","event_history_sha256","event_chain_tip_sha256","event_count","currentness_state","blockers","claim_modified","uses_local_wall_clock","nonclaims"}
F_ELIGIBILITY = {"ELIGIBLE","BLOCKED","NO_ACTIVE_CLAIM"}
F1_CURRENTNESS = {"CURRENT","STALE","PENDING","REVOKED"}
NONCLAIMS = (
    "ELIGIBLE_CURRENT is not payment authorization",
    "ELIGIBLE_CURRENT is not legal distribution entitlement",
    "service composition does not mutate or increase the investor claim",
    "service composition does not establish truth or authenticity of external measurements",
    "service composition is not financial satisfaction, handback readiness, custody currentness, or governance legitimacy",
    "service composition does not establish execution authority",
)

class CompositionError(ValueError):
    pass

@dataclass(frozen=True)
class QualifiedComposition:
    _receipt: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v: Any) -> bytes:
    return json.dumps(v, sort_keys=True, separators=(",",":"), ensure_ascii=False, allow_nan=False).encode("utf-8")

def sha256_hex(v: Any) -> str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()

def _exact(obj: dict[str, Any], keys: set[str], ctx: str) -> None:
    if not isinstance(obj, dict): raise CompositionError(f"{ctx}: expected object")
    if set(obj) != keys: raise CompositionError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")

def _hex40(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not HEX40.fullmatch(v): raise CompositionError(f"{ctx}: invalid git sha")
    return v

def _hex64(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not HEX64.fullmatch(v): raise CompositionError(f"{ctx}: invalid sha256")
    return v

def _bounded_str(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not v or len(v.encode("utf-8")) > 512 or v != v.strip(): raise CompositionError(f"{ctx}: invalid string")
    return v

def validate_profile(p: dict[str, Any]) -> None:
    _exact(p, PROFILE_KEYS, "profile")
    if p["profile_version"] != PROFILE_VERSION: raise CompositionError("profile.profile_version: unsupported")
    _bounded_str(p["project_id"], "profile.project_id")
    _hex40(p["service_gate_subject_sha"], "profile.service_gate_subject_sha")
    _hex40(p["service_currentness_subject_sha"], "profile.service_currentness_subject_sha")
    _hex64(p["service_gate_profile_sha256"], "profile.service_gate_profile_sha256")

def validate_f(r: dict[str, Any], p: dict[str, Any]) -> None:
    _exact(r, F_KEYS, "service_gate_receipt")
    if r["receipt_version"] != F_RECEIPT_VERSION: raise CompositionError("service_gate_receipt.receipt_version: unsupported")
    if r["project_id"] != p["project_id"]: raise CompositionError("service_gate_receipt: project mismatch")
    if r["profile_sha256"] != p["service_gate_profile_sha256"]: raise CompositionError("service_gate_receipt: service profile mismatch")
    _bounded_str(r["measurement_id"], "service_gate_receipt.measurement_id")
    if r["distribution_eligibility"] not in F_ELIGIBILITY: raise CompositionError("service_gate_receipt.distribution_eligibility: unsupported")
    if not isinstance(r["blockers"], list): raise CompositionError("service_gate_receipt.blockers: expected array")
    if r["claim_modified"] is not False: raise CompositionError("service_gate_receipt: claim authority contamination")
    remaining = r["parent_remaining_claim_units"]
    if not isinstance(remaining, int) or isinstance(remaining, bool) or remaining < 0: raise CompositionError("service_gate_receipt.parent_remaining_claim_units: invalid")
    if r["distribution_eligibility"] == "NO_ACTIVE_CLAIM" and remaining != 0: raise CompositionError("service_gate_receipt: NO_ACTIVE_CLAIM requires zero remaining claim")
    if r["distribution_eligibility"] != "NO_ACTIVE_CLAIM" and remaining == 0: raise CompositionError("service_gate_receipt: active eligibility with zero remaining claim")

def validate_f1(r: dict[str, Any], p: dict[str, Any], f: dict[str, Any]) -> None:
    _exact(r, F1_KEYS, "service_currentness_receipt")
    if r["receipt_version"] != F1_RECEIPT_VERSION: raise CompositionError("service_currentness_receipt.receipt_version: unsupported")
    if r["project_id"] != p["project_id"]: raise CompositionError("service_currentness_receipt: project mismatch")
    if r["service_gate_subject_sha"] != p["service_gate_subject_sha"]: raise CompositionError("service_currentness_receipt: service-gate subject mismatch")
    if r["service_gate_profile_sha256"] != f["profile_sha256"]: raise CompositionError("service_currentness_receipt: service profile mismatch")
    if r["measurement_id"] != f["measurement_id"]: raise CompositionError("service_currentness_receipt: measurement mismatch")
    if r["service_receipt_sha256"] != sha256_hex(f): raise CompositionError("service_currentness_receipt: service receipt digest mismatch")
    if r["service_distribution_eligibility"] != f["distribution_eligibility"]: raise CompositionError("service_currentness_receipt: eligibility echo mismatch")
    if r["currentness_state"] not in F1_CURRENTNESS: raise CompositionError("service_currentness_receipt.currentness_state: unsupported")
    if r["claim_modified"] is not False: raise CompositionError("service_currentness_receipt: claim authority contamination")
    if r["uses_local_wall_clock"] is not False: raise CompositionError("service_currentness_receipt: local wall clock authority is forbidden")
    if not isinstance(r["blockers"], list): raise CompositionError("service_currentness_receipt.blockers: expected array")

def qualify(profile: dict[str, Any], service_gate_receipt: dict[str, Any], service_currentness_receipt: dict[str, Any]) -> QualifiedComposition:
    validate_profile(profile); validate_f(service_gate_receipt, profile); validate_f1(service_currentness_receipt, profile, service_gate_receipt)
    f_state = service_gate_receipt["distribution_eligibility"]
    c_state = service_currentness_receipt["currentness_state"]
    blockers: list[str] = []
    if f_state == "NO_ACTIVE_CLAIM": state = "NO_ACTIVE_CLAIM"
    elif c_state == "STALE": state = "STALE"; blockers.append("SERVICE_EVIDENCE_STALE")
    elif c_state == "PENDING": state = "PENDING"; blockers.append("SERVICE_EVIDENCE_PENDING")
    elif c_state == "REVOKED": state = "REVOKED"; blockers.append("SERVICE_EVIDENCE_REVOKED")
    elif c_state == "CURRENT" and f_state == "BLOCKED":
        state = "BLOCKED_SERVICE"; blockers.append("SERVICE_COVENANT_BLOCKED"); blockers.extend(str(x) for x in service_gate_receipt["blockers"])
    elif c_state == "CURRENT" and f_state == "ELIGIBLE": state = "ELIGIBLE_CURRENT"
    else: state = "UNSUPPORTED"; blockers.append("UNSUPPORTED_COMPOSITION")
    blockers = sorted(set(blockers))
    receipt = {
        "receipt_version": RECEIPT_VERSION, "profile_version": PROFILE_VERSION, "project_id": profile["project_id"], "profile_sha256": sha256_hex(profile),
        "service_gate_subject_sha": profile["service_gate_subject_sha"], "service_currentness_subject_sha": profile["service_currentness_subject_sha"],
        "service_gate_receipt_sha256": sha256_hex(service_gate_receipt), "service_currentness_receipt_sha256": sha256_hex(service_currentness_receipt),
        "service_gate_profile_sha256": service_gate_receipt["profile_sha256"], "measurement_id": service_gate_receipt["measurement_id"],
        "service_distribution_eligibility": f_state, "service_currentness_state": c_state, "public_service_distribution_state": state, "blockers": blockers,
        "claim_modified": False, "execution_authority_established": False, "payment_authority_established": False, "legal_distribution_authority_established": False,
        "nonclaims": list(NONCLAIMS),
    }
    return QualifiedComposition(receipt)

def load_case(path: Path):
    data = json.loads(path.read_text(encoding="utf-8")); keys = {"profile","service_gate_receipt","service_currentness_receipt"}
    if not isinstance(data, dict) or set(data) != keys: raise CompositionError("case: expected exact keys")
    return data["profile"], data["service_gate_receipt"], data["service_currentness_receipt"]

def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("case", type=Path); ap.add_argument("--receipt-out", type=Path); args = ap.parse_args()
    receipt = qualify(*load_case(args.case)).receipt(); text = json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    if args.receipt_out: args.receipt_out.write_text(text, encoding="utf-8")
    else: print(text, end="")
    return 0

if __name__ == "__main__": raise SystemExit(main())
