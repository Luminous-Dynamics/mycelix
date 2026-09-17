#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-handback-readiness-currentness-v1"
RECEIPT_VERSION="mycelix-handback-readiness-currentness-receipt-v1"
H3_RECEIPT_VERSION="mycelix-handback-readiness-composition-receipt-v1"
_ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX40=re.compile(r"^[0-9a-f]{40}$")
_HEX64=re.compile(r"^[0-9a-f]{64}$")
PROFILE_KEYS={"profile_version","project_id","handback_subject_sha","handback_receipt_sha256","designation_registry_id","max_events"}
H3_KEYS={
    "blockers","currentness_established","digital_readiness_state","digital_receipt_sha256",
    "digital_subject_sha","execution_authority_established","handback_readiness_state",
    "handover_accepted","legal_transition_complete","nonclaims","operational_custody_accepted",
    "parent_financial_state","parent_remaining_claim_units","parent_subject_sha",
    "parent_transition_receipt_sha256","physical_readiness_state","physical_receipt_sha256",
    "physical_subject_sha","profile_sha256","profile_version","project_id","receipt_version",
}
DESIGNATION_KEYS={
    "designation_id","registry_id","registry_epoch","project_id","profile_sha256",
    "designated_handback_receipt_sha256","designation_state","authority_ref","evidence_ref",
}
EVENT_KEYS={
    "seq","event_id","project_id","profile_sha256","designation_id","prev_event_sha256",
    "kind","authority_ref","evidence_ref",
}
DESIGNATION_STATES={"ACTIVE","PENDING","REVOKED"}
EVENT_KINDS={"MaterialInvalidation","PendingReassessment","RevokeEvidence"}
PRECEDENCE={"CURRENT":0,"STALE":1,"PENDING":2,"REVOKED":3}
H3_STATES={"HANDOVER_READY","REMEDIATION_REQUIRED","ASSESSMENT_INCOMPLETE","RESERVE_DEFICIENT"}
NONCLAIMS=(
    "handback currentness is not handover acceptance",
    "handback currentness is not operational custody transfer",
    "handback currentness is not legal title transfer",
    "CURRENT currentness does not imply HANDOVER_READY",
    "handback currentness does not authenticate external authority or evidence references",
    "handback currentness is relative to the supplied designation lineage and not local wall-clock time",
)

class CurrentnessError(ValueError): pass

@dataclass(frozen=True)
class QualifiedCurrentness:
    _receipt:dict[str,Any]
    def receipt(self)->dict[str,Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(o:Any,keys:set[str],ctx:str)->None:
    if not isinstance(o,dict): raise CurrentnessError(f"{ctx}: expected object")
    if set(o)!=keys: raise CurrentnessError(f"{ctx}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not _ID.fullmatch(v): raise CurrentnessError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or len(v.encode("utf-8"))>512 or v!=v.strip(): raise CurrentnessError(f"{ctx}: invalid reference")
    return v
def _hex(v:Any,ctx:str,n:int)->str:
    pat=_HEX40 if n==40 else _HEX64
    if not isinstance(v,str) or not pat.fullmatch(v): raise CurrentnessError(f"{ctx}: invalid digest")
    return v

def validate_profile(p:dict[str,Any])->None:
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise CurrentnessError("profile.profile_version: unsupported")
    _id(p["project_id"],"profile.project_id")
    _hex(p["handback_subject_sha"],"profile.handback_subject_sha",40)
    _hex(p["handback_receipt_sha256"],"profile.handback_receipt_sha256",64)
    _id(p["designation_registry_id"],"profile.designation_registry_id")
    m=p["max_events"]
    if isinstance(m,bool) or not isinstance(m,int) or m<0 or m>100000: raise CurrentnessError("profile.max_events: invalid")

def validate_h3(r:dict[str,Any],p:dict[str,Any])->None:
    _exact(r,H3_KEYS,"handback_receipt")
    if r["receipt_version"]!=H3_RECEIPT_VERSION: raise CurrentnessError("handback_receipt: unsupported version")
    if r["project_id"]!=p["project_id"]: raise CurrentnessError("handback_receipt: project mismatch")
    if sha256_hex(r)!=p["handback_receipt_sha256"]: raise CurrentnessError("handback_receipt: semantic digest mismatch")
    if r["handback_readiness_state"] not in H3_STATES: raise CurrentnessError("handback_receipt: unsupported readiness state")
    if r["currentness_established"] is not False: raise CurrentnessError("handback_receipt: currentness authority contamination")
    for k in ("handover_accepted","operational_custody_accepted","legal_transition_complete","execution_authority_established"):
        if r[k] is not False: raise CurrentnessError(f"handback_receipt: {k} authority contamination")

def validate_designation(d:dict[str,Any],p:dict[str,Any])->None:
    _exact(d,DESIGNATION_KEYS,"designation")
    _id(d["designation_id"],"designation.designation_id")
    if d["registry_id"]!=p["designation_registry_id"]: raise CurrentnessError("designation.registry_id: registry substitution")
    e=d["registry_epoch"]
    if isinstance(e,bool) or not isinstance(e,int) or e<0 or e>10**18: raise CurrentnessError("designation.registry_epoch: invalid")
    if d["project_id"]!=p["project_id"]: raise CurrentnessError("designation.project_id: project substitution")
    if d["profile_sha256"]!=sha256_hex(p): raise CurrentnessError("designation.profile_sha256: profile substitution")
    _hex(d["designated_handback_receipt_sha256"],"designation.designated_handback_receipt_sha256",64)
    if d["designation_state"] not in DESIGNATION_STATES: raise CurrentnessError("designation.designation_state: unsupported")
    _ref(d["authority_ref"],"designation.authority_ref")
    _ref(d["evidence_ref"],"designation.evidence_ref")

def validate_events(events:Any,p:dict[str,Any],d:dict[str,Any])->list[dict[str,Any]]:
    if not isinstance(events,list): raise CurrentnessError("events: expected array")
    if len(events)>p["max_events"]: raise CurrentnessError("events: exceeds max_events")
    psha=sha256_hex(p); out=[]; ids=set()
    for i,e in enumerate(events):
        _exact(e,EVENT_KEYS,f"events[{i}]")
        if e["seq"]!=i: raise CurrentnessError(f"events[{i}].seq: expected {i}")
        eid=_id(e["event_id"],f"events[{i}].event_id")
        if eid in ids: raise CurrentnessError(f"events[{i}].event_id: duplicate")
        ids.add(eid)
        if e["project_id"]!=p["project_id"]: raise CurrentnessError(f"events[{i}].project_id: project substitution")
        if e["profile_sha256"]!=psha: raise CurrentnessError(f"events[{i}].profile_sha256: profile substitution")
        if e["designation_id"]!=d["designation_id"]: raise CurrentnessError(f"events[{i}].designation_id: designation substitution")
        if i==0:
            if e["prev_event_sha256"] is not None: raise CurrentnessError("events[0].prev_event_sha256: must be null")
        elif e["prev_event_sha256"]!=sha256_hex(out[-1]):
            raise CurrentnessError(f"events[{i}].prev_event_sha256: broken event chain")
        if e["kind"] not in EVENT_KINDS: raise CurrentnessError(f"events[{i}].kind: unsupported")
        _ref(e["authority_ref"],f"events[{i}].authority_ref")
        _ref(e["evidence_ref"],f"events[{i}].evidence_ref")
        out.append(e)
    return out

def _raise_state(current:str,candidate:str)->str:
    return candidate if PRECEDENCE[candidate]>PRECEDENCE[current] else current

def qualify(p:dict[str,Any],h3:dict[str,Any],d:dict[str,Any],events:Any)->QualifiedCurrentness:
    validate_profile(p); validate_h3(h3,p); validate_designation(d,p)
    history=validate_events(events,p,d)
    digest=sha256_hex(h3)
    state="CURRENT"; blockers=[]
    if d["designated_handback_receipt_sha256"]!=digest:
        state=_raise_state(state,"STALE"); blockers.append("NOT_DESIGNATED_CURRENT")
    if d["designation_state"]=="PENDING":
        state=_raise_state(state,"PENDING"); blockers.append("DESIGNATION_PENDING")
    elif d["designation_state"]=="REVOKED":
        state=_raise_state(state,"REVOKED"); blockers.append("DESIGNATION_REVOKED")
    for e in history:
        if e["kind"]=="MaterialInvalidation":
            state=_raise_state(state,"STALE"); blockers.append("MATERIAL_INVALIDATION")
        elif e["kind"]=="PendingReassessment":
            state=_raise_state(state,"PENDING"); blockers.append("PENDING_REASSESSMENT")
        elif e["kind"]=="RevokeEvidence":
            state=_raise_state(state,"REVOKED"); blockers.append("EVIDENCE_REVOKED")
    blockers=sorted(set(blockers))
    receipt={
        "receipt_version":RECEIPT_VERSION,
        "profile_version":PROFILE_VERSION,
        "project_id":p["project_id"],
        "profile_sha256":sha256_hex(p),
        "handback_subject_sha":p["handback_subject_sha"],
        "handback_receipt_sha256":digest,
        "handback_readiness_state":h3["handback_readiness_state"],
        "designation_id":d["designation_id"],
        "designation_registry_id":d["registry_id"],
        "designation_registry_epoch":d["registry_epoch"],
        "designation_sha256":sha256_hex(d),
        "event_count":len(history),
        "event_history_sha256":sha256_hex(history),
        "event_chain_tip_sha256":None if not history else sha256_hex(history[-1]),
        "handback_readiness_currentness_state":state,
        "blockers":blockers,
        "handover_accepted":False,
        "operational_custody_accepted":False,
        "legal_transition_complete":False,
        "execution_authority_established":False,
        "uses_local_wall_clock":False,
        "nonclaims":list(NONCLAIMS),
    }
    return QualifiedCurrentness(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(d,dict) or set(d)!={"profile","handback_receipt","designation","events"}:
        raise CurrentnessError("case: expected exact keys")
    return d["profile"],d["handback_receipt"],d["designation"],d["events"]

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); out=qualify(*load_case(a.case)).receipt()
    text=json.dumps(out,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text,encoding="utf-8")
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
