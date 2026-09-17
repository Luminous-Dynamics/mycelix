#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from pathlib import Path
from typing import Any

RECEIPT_VERSION = "mycelix-public-service-distribution-composition-receipt-v1"
F_RECEIPT_VERSION = "mycelix-public-service-gate-receipt-v1"
F1_RECEIPT_VERSION = "mycelix-service-currentness-receipt-v1"
F_SUBJECT = "92fe8408b52ce1b63808201e9f70ab5280a6f489"
F1_SUBJECT = "caf653560310c9d47b036ef88a2207bff6d4b06f"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
F_KEYS = {
    "blockers","claim_modified","distribution_eligibility","measurement_epoch","measurement_id",
    "nonclaims","observed","parent_remaining_claim_units","parent_subject_sha",
    "parent_transition_receipt_sha256","profile_sha256","profile_version","project_id",
    "receipt_version","snapshot_sha256","unit",
}
F1_KEYS = {
    "blockers","claim_modified","currentness_state","designation_id","designation_registry_epoch",
    "designation_registry_id","designation_sha256","event_chain_tip_sha256","event_count",
    "event_history_sha256","measurement_id","nonclaims","profile_sha256","profile_version",
    "project_id","receipt_version","service_distribution_eligibility",
    "service_gate_profile_sha256","service_gate_subject_sha","service_receipt_sha256",
    "uses_local_wall_clock",
}

class CompositionError(ValueError): pass

def canonical_bytes(v: Any) -> bytes:
    return json.dumps(v, sort_keys=True, separators=(",",":"), ensure_ascii=False, allow_nan=False).encode("utf-8")

def sha256_hex(v: Any) -> str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()

def _exact(v: dict[str,Any], keys:set[str], name:str):
    if not isinstance(v, dict): raise CompositionError(f"{name}: expected object")
    got=set(v)
    if got != keys: raise CompositionError(f"{name}: key mismatch missing={sorted(keys-got)} unknown={sorted(got-keys)}")

def _hex(v: Any, name:str):
    if not isinstance(v,str) or not HEX64.fullmatch(v): raise CompositionError(f"{name}: invalid sha256")

def validate_f(r: dict[str,Any]):
    _exact(r,F_KEYS,"F")
    if r["receipt_version"] != F_RECEIPT_VERSION: raise CompositionError("F: receipt version mismatch")
    if r["claim_modified"] is not False: raise CompositionError("F: claim_modified must be false")
    if r["distribution_eligibility"] not in {"ELIGIBLE","BLOCKED","NO_ACTIVE_CLAIM"}:
        raise CompositionError("F: unsupported distribution eligibility")
    if not isinstance(r["project_id"],str) or not r["project_id"]: raise CompositionError("F: invalid project")
    if not isinstance(r["measurement_id"],str) or not r["measurement_id"]: raise CompositionError("F: invalid measurement")
    _hex(r["profile_sha256"],"F.profile_sha256")

def validate_f1(r: dict[str,Any]):
    _exact(r,F1_KEYS,"F1")
    if r["receipt_version"] != F1_RECEIPT_VERSION: raise CompositionError("F1: receipt version mismatch")
    if r["claim_modified"] is not False: raise CompositionError("F1: claim_modified must be false")
    if r["uses_local_wall_clock"] is not False: raise CompositionError("F1: local wall clock authority unsupported")
    if r["currentness_state"] not in {"CURRENT","STALE","PENDING","REVOKED"}:
        raise CompositionError("F1: unsupported currentness state")
    if r["service_gate_subject_sha"] != F_SUBJECT:
        raise CompositionError("F1: wrong qualified F subject")
    for k in ("service_gate_profile_sha256","service_receipt_sha256"):
        _hex(r[k],f"F1.{k}")

def compose(f: dict[str,Any], f1: dict[str,Any]) -> dict[str,Any]:
    validate_f(f); validate_f1(f1)
    if f["project_id"] != f1["project_id"]: raise CompositionError("project mismatch")
    if f["measurement_id"] != f1["measurement_id"]: raise CompositionError("measurement mismatch")
    if f["profile_sha256"] != f1["service_gate_profile_sha256"]: raise CompositionError("service profile mismatch")
    fdigest=sha256_hex(f)
    if fdigest != f1["service_receipt_sha256"]: raise CompositionError("service receipt digest mismatch")
    if f["distribution_eligibility"] != f1["service_distribution_eligibility"]:
        raise CompositionError("eligibility echo mismatch")

    blockers=[]
    cur=f1["currentness_state"]
    elig=f["distribution_eligibility"]
    if cur == "STALE": state="STALE"; blockers.append("SERVICE_EVIDENCE_STALE")
    elif cur == "PENDING": state="PENDING"; blockers.append("SERVICE_EVIDENCE_PENDING")
    elif cur == "REVOKED": state="REVOKED"; blockers.append("SERVICE_EVIDENCE_REVOKED")
    elif elig == "BLOCKED": state="BLOCKED_SERVICE"; blockers.extend(f["blockers"] or ["SERVICE_COVENANT_BLOCKED"])
    elif elig == "NO_ACTIVE_CLAIM": state="NO_ACTIVE_CLAIM"
    elif elig == "ELIGIBLE": state="ELIGIBLE_CURRENT"
    else: state="UNSUPPORTED"; blockers.append("UNSUPPORTED_SERVICE_STATE")
    blockers=sorted(set(blockers))
    return {
        "receipt_version": RECEIPT_VERSION,
        "project_id": f["project_id"],
        "qualified_f_subject_sha": F_SUBJECT,
        "qualified_f1_subject_sha": F1_SUBJECT,
        "f_receipt_sha256": fdigest,
        "f1_receipt_sha256": sha256_hex(f1),
        "measurement_id": f["measurement_id"],
        "service_gate_profile_sha256": f["profile_sha256"],
        "service_currentness_profile_sha256": f1["profile_sha256"],
        "public_service_distribution_state": state,
        "blockers": blockers,
        "claim_modified": False,
        "execution_authority_established": False,
        "payment_authority_established": False,
        "legal_distribution_authority_established": False,
        "nonclaims": [
            "current service eligibility is not payment authorization",
            "current service eligibility does not establish legal distribution entitlement",
            "service composition does not establish measurement authenticity",
            "service composition does not establish solvency, handback readiness, or democratic legitimacy",
        ],
    }

def load_case(path:Path):
    d=json.loads(path.read_text())
    if not isinstance(d,dict) or set(d)!={"service_gate_receipt","service_currentness_receipt"}:
        raise CompositionError("case: exact inputs required")
    return d["service_gate_receipt"],d["service_currentness_receipt"]

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); f,f1=load_case(a.case); r=compose(f,f1)
    text=json.dumps(r,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text)
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
