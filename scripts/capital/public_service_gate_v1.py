#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-public-service-distribution-gate-v1"
RECEIPT_VERSION="mycelix-public-service-gate-receipt-v1"
PPM=1_000_000
_ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
HEX40=re.compile(r"^[0-9a-f]{40}$")
HEX64=re.compile(r"^[0-9a-f]{64}$")
PROFILE_KEYS={"profile_version","project_id","unit","parent_subject_sha","parent_transition_receipt_sha256","max_amount_units","min_coverage_ppm","min_uptime_ppm","max_tariff_units","required_reserve_units","max_deferred_maintenance_units","required_continuity_state"}
SNAPSHOT_KEYS={"measurement_id","project_id","profile_sha256","parent_transition_receipt_sha256","measurement_epoch","coverage_ppm","uptime_ppm","tariff_units","reserve_balance_units","deferred_maintenance_units","continuity_state","authority_ref","evidence_ref"}
CONTINUITY={"PASS","FAIL","NOT_ASSESSED"}
BLOCK_ORDER=("BLOCKED_SERVICE_COVERAGE","BLOCKED_RELIABILITY","BLOCKED_AFFORDABILITY","BLOCKED_RESERVE","BLOCKED_MAINTENANCE","BLOCKED_RESILIENCE")
NONCLAIMS=(
"distribution eligibility is not a determination of legally fair tariffs",
"distribution eligibility is not infrastructure engineering adequacy outside the frozen metrics",
"blocking distribution does not itself impair or increase the investor claim",
"service evidence validity is not proof of external measurement authenticity",
"service compliance is not solvency or handback readiness",
"service compliance is not democratic or regulatory legitimacy",
)

class GateError(ValueError): pass
@dataclass(frozen=True)
class QualifiedGate:
    _receipt:dict[str,Any]
    def receipt(self)->dict[str,Any]: return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode()
def sha256_hex(v:Any)->str: return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(o:dict[str,Any],keys:set[str],ctx:str):
    if set(o)!=keys: raise GateError(f"{ctx}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")
def _id(v:Any,ctx:str):
    if not isinstance(v,str) or not _ID.fullmatch(v): raise GateError(f"{ctx}: invalid identifier")
    return v
def _ref(v:Any,ctx:str):
    if not isinstance(v,str) or not v or len(v.encode())>512 or v!=v.strip(): raise GateError(f"{ctx}: invalid reference")
    return v
def _int(v:Any,lo:int,hi:int,ctx:str):
    if isinstance(v,bool) or not isinstance(v,int) or v<lo or v>hi: raise GateError(f"{ctx}: outside bounds")
    return v

def validate_profile(p:dict[str,Any]):
    if not isinstance(p,dict): raise GateError("profile: expected object")
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise GateError("profile.profile_version: unsupported")
    _id(p["project_id"],"profile.project_id"); _id(p["unit"],"profile.unit")
    if not isinstance(p["parent_subject_sha"],str) or not HEX40.fullmatch(p["parent_subject_sha"]): raise GateError("profile.parent_subject_sha: invalid")
    if not isinstance(p["parent_transition_receipt_sha256"],str) or not HEX64.fullmatch(p["parent_transition_receipt_sha256"]): raise GateError("profile.parent_transition_receipt_sha256: invalid")
    maxa=_int(p["max_amount_units"],1,10**24,"profile.max_amount_units")
    _int(p["min_coverage_ppm"],0,PPM,"profile.min_coverage_ppm")
    _int(p["min_uptime_ppm"],0,PPM,"profile.min_uptime_ppm")
    _int(p["max_tariff_units"],0,maxa,"profile.max_tariff_units")
    _int(p["required_reserve_units"],0,maxa,"profile.required_reserve_units")
    _int(p["max_deferred_maintenance_units"],0,maxa,"profile.max_deferred_maintenance_units")
    if p["required_continuity_state"]!="PASS": raise GateError("profile.required_continuity_state: v1 requires PASS")
    return p

def validate_parent(parent:dict[str,Any],p:dict[str,Any]):
    if not isinstance(parent,dict): raise GateError("parent_receipt: expected object")
    if sha256_hex(parent)!=p["parent_transition_receipt_sha256"]: raise GateError("parent_receipt: semantic digest mismatch")
    for k in ("project_id","unit","remaining_claim_units","profile_version"):
        if k not in parent: raise GateError(f"parent_receipt: missing {k}")
    if parent["project_id"]!=p["project_id"]: raise GateError("parent_receipt: project mismatch")
    if parent["unit"]!=p["unit"]: raise GateError("parent_receipt: unit mismatch")
    if parent["profile_version"]!="mycelix-capital-to-commons-fixed-preferred-v1": raise GateError("parent_receipt: unsupported financial profile")

def qualify(p:dict[str,Any],parent:dict[str,Any],s:dict[str,Any])->QualifiedGate:
    validate_profile(p); validate_parent(parent,p)
    if not isinstance(s,dict): raise GateError("snapshot: expected object")
    _exact(s,SNAPSHOT_KEYS,"snapshot")
    _id(s["measurement_id"],"snapshot.measurement_id")
    if s["project_id"]!=p["project_id"]: raise GateError("snapshot.project_id: project substitution")
    if s["profile_sha256"]!=sha256_hex(p): raise GateError("snapshot.profile_sha256: profile substitution")
    if s["parent_transition_receipt_sha256"]!=p["parent_transition_receipt_sha256"]: raise GateError("snapshot.parent_transition_receipt_sha256: parent substitution")
    _int(s["measurement_epoch"],1,10**18,"snapshot.measurement_epoch")
    coverage=_int(s["coverage_ppm"],0,PPM,"snapshot.coverage_ppm")
    uptime=_int(s["uptime_ppm"],0,PPM,"snapshot.uptime_ppm")
    maxa=p["max_amount_units"]
    tariff=_int(s["tariff_units"],0,maxa,"snapshot.tariff_units")
    reserve=_int(s["reserve_balance_units"],0,maxa,"snapshot.reserve_balance_units")
    maint=_int(s["deferred_maintenance_units"],0,maxa,"snapshot.deferred_maintenance_units")
    if s["continuity_state"] not in CONTINUITY: raise GateError("snapshot.continuity_state: invalid")
    _ref(s["authority_ref"],"snapshot.authority_ref"); _ref(s["evidence_ref"],"snapshot.evidence_ref")
    blockers=[]
    if coverage<p["min_coverage_ppm"]: blockers.append("BLOCKED_SERVICE_COVERAGE")
    if uptime<p["min_uptime_ppm"]: blockers.append("BLOCKED_RELIABILITY")
    if tariff>p["max_tariff_units"]: blockers.append("BLOCKED_AFFORDABILITY")
    if reserve<p["required_reserve_units"]: blockers.append("BLOCKED_RESERVE")
    if maint>p["max_deferred_maintenance_units"]: blockers.append("BLOCKED_MAINTENANCE")
    if s["continuity_state"]!=p["required_continuity_state"]: blockers.append("BLOCKED_RESILIENCE")
    blockers=[b for b in BLOCK_ORDER if b in blockers]
    remaining=parent["remaining_claim_units"]
    eligibility="NO_ACTIVE_CLAIM" if remaining==0 else ("ELIGIBLE" if not blockers else "BLOCKED")
    receipt={
      "receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,"project_id":p["project_id"],"unit":p["unit"],
      "profile_sha256":sha256_hex(p),"parent_subject_sha":p["parent_subject_sha"],
      "parent_transition_receipt_sha256":p["parent_transition_receipt_sha256"],
      "measurement_id":s["measurement_id"],"measurement_epoch":s["measurement_epoch"],
      "snapshot_sha256":sha256_hex(s),"distribution_eligibility":eligibility,"blockers":blockers,
      "parent_remaining_claim_units":remaining,"claim_modified":False,
      "observed":{"coverage_ppm":coverage,"uptime_ppm":uptime,"tariff_units":tariff,"reserve_balance_units":reserve,"deferred_maintenance_units":maint,"continuity_state":s["continuity_state"]},
      "nonclaims":list(NONCLAIMS)}
    return QualifiedGate(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text())
    if not isinstance(d,dict) or set(d)!={"profile","parent_receipt","snapshot"}: raise GateError("case: expected exact keys")
    return d["profile"],d["parent_receipt"],d["snapshot"]

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); p,parent,s=load_case(a.case); r=qualify(p,parent,s).receipt()
    text=json.dumps(r,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text)
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
