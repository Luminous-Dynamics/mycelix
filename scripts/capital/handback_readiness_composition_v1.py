#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-handback-readiness-composition-v1"
RECEIPT_VERSION="mycelix-handback-readiness-composition-receipt-v1"
D1_VERSION="mycelix-digital-operational-sovereignty-receipt-v1"
D4_VERSION="mycelix-physical-handback-readiness-receipt-v1"
HEX40=re.compile(r"^[0-9a-f]{40}$")
HEX64=re.compile(r"^[0-9a-f]{64}$")
PROFILE_KEYS={"profile_version","project_id","digital_subject_sha","physical_subject_sha"}
D1_KEYS={"assessment_id","assessment_sha256","assessor_ref","blockers","controls","handover_accepted","legal_transition_complete","nonclaims","outgoing_operator_ref","parent_financial_state","parent_remaining_claim_units","parent_subject_sha","parent_transition_receipt_sha256","profile_sha256","profile_version","project_id","readiness_state","receipt_version","required_dimensions"}
D4_KEYS={"assessment_id","assessment_sha256","assessor_ref","blockers","components","currency_unit","deferred_maintenance_units","handover_accepted","legal_transition_complete","max_deferred_maintenance_units","nonclaims","outgoing_operator_ref","parent_financial_state","parent_remaining_claim_units","parent_subject_sha","parent_transition_receipt_sha256","physical_state","profile_sha256","profile_version","project_id","receipt_version","required_handback_reserve_units","reserve_balance_units","residual_life_unit"}
D1_STATES={"OPERATIONAL_TRANSFER_READY","ASSESSMENT_INCOMPLETE","REMEDIATION_REQUIRED"}
D4_STATES={"PHYSICAL_CONDITION_ACCEPTABLE","RESERVE_DEFICIENT","ASSESSMENT_INCOMPLETE","REMEDIATION_REQUIRED"}
NONCLAIMS=["handback readiness is not handover acceptance","handback readiness is not operational custody transfer","handback readiness is not legal title transfer","handback readiness does not establish assessment currentness","handback readiness does not establish statutory inspection or engineering fitness outside the qualified input scopes","handback readiness does not establish execution authority"]

class CompositionError(ValueError): pass

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode()
def sha256_hex(v:Any)->str: return hashlib.sha256(canonical_bytes(v)).hexdigest()
def exact(v:Any,keys:set[str],ctx:str):
    if not isinstance(v,dict): raise CompositionError(f"{ctx}: expected object")
    if set(v)!=keys: raise CompositionError(f"{ctx}: key mismatch missing={sorted(keys-set(v))} unknown={sorted(set(v)-keys)}")
def text(v:Any,ctx:str):
    if not isinstance(v,str) or not v or v!=v.strip() or len(v.encode())>512: raise CompositionError(f"{ctx}: invalid string")
def hex40(v:Any,ctx:str):
    if not isinstance(v,str) or not HEX40.fullmatch(v): raise CompositionError(f"{ctx}: invalid git sha")
def hex64(v:Any,ctx:str):
    if not isinstance(v,str) or not HEX64.fullmatch(v): raise CompositionError(f"{ctx}: invalid sha256")
def nni(v:Any,ctx:str):
    if isinstance(v,bool) or not isinstance(v,int) or v<0: raise CompositionError(f"{ctx}: invalid non-negative integer")

def validate_profile(p):
    exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise CompositionError("profile: unsupported version")
    text(p["project_id"],"profile.project_id"); hex40(p["digital_subject_sha"],"profile.digital_subject_sha"); hex40(p["physical_subject_sha"],"profile.physical_subject_sha")
def validate_d1(r,p):
    exact(r,D1_KEYS,"digital_receipt")
    if r["receipt_version"]!=D1_VERSION: raise CompositionError("digital_receipt: unsupported version")
    if r["project_id"]!=p["project_id"]: raise CompositionError("digital_receipt: project mismatch")
    if r["readiness_state"] not in D1_STATES: raise CompositionError("digital_receipt: unsupported state")
    if r["handover_accepted"] is not False or r["legal_transition_complete"] is not False: raise CompositionError("digital_receipt: authority contamination")
    if not isinstance(r["blockers"],list): raise CompositionError("digital_receipt.blockers: expected array")
    hex64(r["profile_sha256"],"digital_receipt.profile_sha256"); hex40(r["parent_subject_sha"],"digital_receipt.parent_subject_sha"); hex64(r["parent_transition_receipt_sha256"],"digital_receipt.parent_transition_receipt_sha256")
    text(r["parent_financial_state"],"digital_receipt.parent_financial_state"); nni(r["parent_remaining_claim_units"],"digital_receipt.parent_remaining_claim_units")
def validate_d4(r,p):
    exact(r,D4_KEYS,"physical_receipt")
    if r["receipt_version"]!=D4_VERSION: raise CompositionError("physical_receipt: unsupported version")
    if r["project_id"]!=p["project_id"]: raise CompositionError("physical_receipt: project mismatch")
    if r["physical_state"] not in D4_STATES: raise CompositionError("physical_receipt: unsupported state")
    if r["handover_accepted"] is not False or r["legal_transition_complete"] is not False: raise CompositionError("physical_receipt: authority contamination")
    if not isinstance(r["blockers"],list): raise CompositionError("physical_receipt.blockers: expected array")
    hex64(r["profile_sha256"],"physical_receipt.profile_sha256"); hex40(r["parent_subject_sha"],"physical_receipt.parent_subject_sha"); hex64(r["parent_transition_receipt_sha256"],"physical_receipt.parent_transition_receipt_sha256")
    text(r["parent_financial_state"],"physical_receipt.parent_financial_state"); nni(r["parent_remaining_claim_units"],"physical_receipt.parent_remaining_claim_units")
    reserve=r["reserve_balance_units"]; required=r["required_handback_reserve_units"]; deferred=r["deferred_maintenance_units"]; maxd=r["max_deferred_maintenance_units"]
    for v,n in [(reserve,"reserve"),(required,"required reserve"),(deferred,"deferred maintenance"),(maxd,"max deferred maintenance")]: nni(v,f"physical_receipt.{n}")
    if r["physical_state"]=="PHYSICAL_CONDITION_ACCEPTABLE" and reserve<required: raise CompositionError("physical_receipt: acceptable with deficient reserve")
    if r["physical_state"]=="PHYSICAL_CONDITION_ACCEPTABLE" and deferred>maxd: raise CompositionError("physical_receipt: acceptable with excessive deferred maintenance")
def align(a,b):
    for k,label in [("project_id","project"),("parent_subject_sha","parent subject"),("parent_transition_receipt_sha256","parent receipt"),("parent_financial_state","financial state"),("parent_remaining_claim_units","remaining claim")]:
        if a[k]!=b[k]: raise CompositionError(f"input alignment: {label} mismatch")
def db(prefix,raw,fallback): return [f"{prefix}:{x}" for x in raw] if raw else [fallback]

def qualify(profile,digital,physical):
    validate_profile(profile); validate_d1(digital,profile); validate_d4(physical,profile); align(digital,physical)
    ds=digital["readiness_state"]; ps=physical["physical_state"]; blockers=[]
    if ds=="REMEDIATION_REQUIRED": blockers+=db("DIGITAL",digital["blockers"],"DIGITAL_REMEDIATION_REQUIRED")
    if ps=="REMEDIATION_REQUIRED": blockers+=db("PHYSICAL",physical["blockers"],"PHYSICAL_REMEDIATION_REQUIRED")
    if blockers: state="REMEDIATION_REQUIRED"
    elif ds=="ASSESSMENT_INCOMPLETE" or ps=="ASSESSMENT_INCOMPLETE":
        if ds=="ASSESSMENT_INCOMPLETE": blockers+=db("DIGITAL",digital["blockers"],"DIGITAL_ASSESSMENT_INCOMPLETE")
        if ps=="ASSESSMENT_INCOMPLETE": blockers+=db("PHYSICAL",physical["blockers"],"PHYSICAL_ASSESSMENT_INCOMPLETE")
        state="ASSESSMENT_INCOMPLETE"
    elif ps=="RESERVE_DEFICIENT": blockers+=db("PHYSICAL",physical["blockers"],"PHYSICAL_RESERVE_DEFICIENT"); state="RESERVE_DEFICIENT"
    elif ds=="OPERATIONAL_TRANSFER_READY" and ps=="PHYSICAL_CONDITION_ACCEPTABLE": state="HANDOVER_READY"
    else: state="UNSUPPORTED"; blockers.append("UNSUPPORTED_HANDOVER_COMPOSITION")
    blockers=sorted(set(blockers))
    return {"receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,"project_id":profile["project_id"],"profile_sha256":sha256_hex(profile),"digital_subject_sha":profile["digital_subject_sha"],"physical_subject_sha":profile["physical_subject_sha"],"digital_receipt_sha256":sha256_hex(digital),"physical_receipt_sha256":sha256_hex(physical),"parent_subject_sha":digital["parent_subject_sha"],"parent_transition_receipt_sha256":digital["parent_transition_receipt_sha256"],"parent_financial_state":digital["parent_financial_state"],"parent_remaining_claim_units":digital["parent_remaining_claim_units"],"digital_readiness_state":ds,"physical_readiness_state":ps,"handback_readiness_state":state,"blockers":blockers,"currentness_established":False,"handover_accepted":False,"operational_custody_accepted":False,"legal_transition_complete":False,"execution_authority_established":False,"nonclaims":NONCLAIMS}
def load_case(path:Path):
    d=json.loads(path.read_text())
    if not isinstance(d,dict) or set(d)!={"profile","digital_receipt","physical_receipt"}: raise CompositionError("case: exact inputs required")
    return d["profile"],d["digital_receipt"],d["physical_receipt"]
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path); a=ap.parse_args()
    r=qualify(*load_case(a.case)); s=json.dumps(r,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(s)
    else: print(s,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
