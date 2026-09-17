#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-handback-readiness-composition-v1"
RECEIPT_VERSION="mycelix-handback-readiness-composition-receipt-v1"
D1_VERSION="mycelix-digital-operational-sovereignty-receipt-v1"
D4_VERSION="mycelix-physical-handback-readiness-receipt-v1"
_HEX40=re.compile(r"^[0-9a-f]{40}$"); _HEX64=re.compile(r"^[0-9a-f]{64}$")
PROFILE_KEYS={"profile_version","project_id","qualified_digital_subject_sha","qualified_physical_subject_sha"}
D1_KEYS={"assessment_id","assessment_sha256","assessor_ref","blockers","controls","handover_accepted","legal_transition_complete","nonclaims","outgoing_operator_ref","parent_financial_state","parent_remaining_claim_units","parent_subject_sha","parent_transition_receipt_sha256","profile_sha256","profile_version","project_id","readiness_state","receipt_version","required_dimensions"}
D4_KEYS={"assessment_id","assessment_sha256","assessor_ref","blockers","components","currency_unit","deferred_maintenance_units","handover_accepted","legal_transition_complete","max_deferred_maintenance_units","nonclaims","outgoing_operator_ref","parent_financial_state","parent_remaining_claim_units","parent_subject_sha","parent_transition_receipt_sha256","physical_state","profile_sha256","profile_version","project_id","receipt_version","required_handback_reserve_units","reserve_balance_units","residual_life_unit"}
D1_STATES={"OPERATIONAL_TRANSFER_READY","REMEDIATION_REQUIRED","ASSESSMENT_INCOMPLETE"}
D4_STATES={"PHYSICAL_CONDITION_ACCEPTABLE","REMEDIATION_REQUIRED","ASSESSMENT_INCOMPLETE","RESERVE_DEFICIENT"}
NONCLAIMS=(
"handback readiness composition does not establish present-time currentness",
"handback readiness composition is not handover acceptance",
"handback readiness composition is not operational custody acceptance",
"handback readiness composition is not legal title transfer",
"handback readiness composition is not execution authority",
"handback readiness composition does not extend engineering assurance beyond the qualified D1/D4 scopes",
)
class HandbackError(ValueError): pass
@dataclass(frozen=True)
class QualifiedHandback:
    _receipt:dict[str,Any]
    def receipt(self): return json.loads(json.dumps(self._receipt))
def canonical_bytes(v): return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode()
def sha256_hex(v): return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(o,keys,label):
    if not isinstance(o,dict): raise HandbackError(f"{label}: expected object")
    if set(o)!=keys: raise HandbackError(f"{label}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")
def _hex(v,n,label):
    p=_HEX40 if n==40 else _HEX64
    if not isinstance(v,str) or not p.fullmatch(v): raise HandbackError(f"{label}: invalid digest")
def validate_profile(p):
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise HandbackError("profile: unsupported version")
    if not isinstance(p["project_id"],str) or not p["project_id"]: raise HandbackError("profile: invalid project")
    _hex(p["qualified_digital_subject_sha"],40,"profile.digital_subject")
    _hex(p["qualified_physical_subject_sha"],40,"profile.physical_subject")
def validate_d1(d,p):
    _exact(d,D1_KEYS,"digital_receipt")
    if d["receipt_version"]!=D1_VERSION: raise HandbackError("digital_receipt: unsupported version")
    if d["project_id"]!=p["project_id"]: raise HandbackError("digital_receipt: project mismatch")
    if d["readiness_state"] not in D1_STATES: raise HandbackError("digital_receipt: unsupported state")
    if d["handover_accepted"] is not False or d["legal_transition_complete"] is not False: raise HandbackError("digital_receipt: authority contamination")
    if not isinstance(d["blockers"],list): raise HandbackError("digital_receipt: blockers must be array")
    _hex(d["parent_subject_sha"],40,"digital_receipt.parent_subject")
    _hex(d["parent_transition_receipt_sha256"],64,"digital_receipt.parent_receipt")
def validate_d4(x,p):
    _exact(x,D4_KEYS,"physical_receipt")
    if x["receipt_version"]!=D4_VERSION: raise HandbackError("physical_receipt: unsupported version")
    if x["project_id"]!=p["project_id"]: raise HandbackError("physical_receipt: project mismatch")
    if x["physical_state"] not in D4_STATES: raise HandbackError("physical_receipt: unsupported state")
    if x["handover_accepted"] is not False or x["legal_transition_complete"] is not False: raise HandbackError("physical_receipt: authority contamination")
    if not isinstance(x["blockers"],list): raise HandbackError("physical_receipt: blockers must be array")
    _hex(x["parent_subject_sha"],40,"physical_receipt.parent_subject")
    _hex(x["parent_transition_receipt_sha256"],64,"physical_receipt.parent_receipt")
def qualify(p,d,x):
    validate_profile(p); validate_d1(d,p); validate_d4(x,p)
    for k in ("parent_subject_sha","parent_transition_receipt_sha256","parent_financial_state","parent_remaining_claim_units"):
        if d[k]!=x[k]: raise HandbackError(f"input lineage mismatch:{k}")
    blockers=[]
    ds=d["readiness_state"]; ps=x["physical_state"]
    if ds=="REMEDIATION_REQUIRED": blockers += ["DIGITAL_REMEDIATION_REQUIRED"]+[f"DIGITAL:{b}" for b in d["blockers"]]
    if ps=="REMEDIATION_REQUIRED": blockers += ["PHYSICAL_REMEDIATION_REQUIRED"]+[f"PHYSICAL:{b}" for b in x["blockers"]]
    if ds=="ASSESSMENT_INCOMPLETE": blockers += ["DIGITAL_ASSESSMENT_INCOMPLETE"]+[f"DIGITAL:{b}" for b in d["blockers"]]
    if ps=="ASSESSMENT_INCOMPLETE": blockers += ["PHYSICAL_ASSESSMENT_INCOMPLETE"]+[f"PHYSICAL:{b}" for b in x["blockers"]]
    if ps=="RESERVE_DEFICIENT": blockers += ["PHYSICAL_RESERVE_DEFICIENT"]+[f"PHYSICAL:{b}" for b in x["blockers"]]
    blockers=sorted(set(blockers))
    if ds=="REMEDIATION_REQUIRED" or ps=="REMEDIATION_REQUIRED": state="REMEDIATION_REQUIRED"
    elif ds=="ASSESSMENT_INCOMPLETE" or ps=="ASSESSMENT_INCOMPLETE": state="ASSESSMENT_INCOMPLETE"
    elif ps=="RESERVE_DEFICIENT": state="RESERVE_DEFICIENT"
    else: state="HANDOVER_READY"
    r={
      "receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,"project_id":p["project_id"],"profile_sha256":sha256_hex(p),
      "qualified_digital_subject_sha":p["qualified_digital_subject_sha"],"qualified_physical_subject_sha":p["qualified_physical_subject_sha"],
      "digital_receipt_sha256":sha256_hex(d),"physical_receipt_sha256":sha256_hex(x),
      "parent_subject_sha":d["parent_subject_sha"],"parent_transition_receipt_sha256":d["parent_transition_receipt_sha256"],
      "parent_financial_state":d["parent_financial_state"],"parent_remaining_claim_units":d["parent_remaining_claim_units"],
      "digital_readiness_state":ds,"physical_readiness_state":ps,"handback_readiness_state":state,"blockers":blockers,
      "currentness_established":False,"handover_accepted":False,"operational_custody_accepted":False,"legal_transition_complete":False,"execution_authority_established":False,
      "nonclaims":list(NONCLAIMS)}
    return QualifiedHandback(r)
def load_case(path):
    q=json.loads(Path(path).read_text())
    if not isinstance(q,dict) or set(q)!={"profile","digital_receipt","physical_receipt"}: raise HandbackError("case: expected exact keys")
    return q["profile"],q["digital_receipt"],q["physical_receipt"]
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path); a=ap.parse_args()
    text=json.dumps(qualify(*load_case(a.case)).receipt(),sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out:a.receipt_out.write_text(text)
    else:print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
