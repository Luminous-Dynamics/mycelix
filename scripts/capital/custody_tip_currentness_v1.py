#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-custody-tip-currentness-v1"
RECEIPT_VERSION="mycelix-custody-tip-currentness-receipt-v1"
D3_RECEIPT_VERSION="mycelix-custody-state-receipt-v1"
QUALIFIED_D3_SUBJECT="1fefe7a313d1cc83f3559d1d9f3223e12b2df019"
QUALIFIED_D3_RECEIPT_SHA256="26f9f23e24d6a120c3cf210e920e6111fdefbd1355fb0069052bdfb5a68f01e1"
QUALIFIED_D3_PROFILE_SHA256="99478af7e37e6d878c42051bb88657feaf083fe9d55afa6726aa8fa78e8fca3a"
QUALIFIED_D3_EVENT_HISTORY_SHA256="5ec741fe98ae16872cd1a6a954b22fa7ea679ff48075cca96a622ed8ee864807"
QUALIFIED_D3_EVENT_TIP_SHA256="82b13fad92801eee7038b237b0a5e47aebb198ef4ec0e63d3f4ed139dfe28744"
_ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX64=re.compile(r"^[0-9a-f]{64}$")
PROFILE_KEYS={"profile_version","project_id","d3_subject_sha","d3_receipt_sha256","d3_profile_sha256","d3_event_history_sha256","d3_event_tip_sha256","designation_registry_id","max_events"}
D3_KEYS={"acceptance_receipt_sha256","acceptance_subject_sha","active_blockers","constitutional_stewardship_transition_established","current_custodian_ref","custody_state","event_chain_tip_sha256","event_count","event_history_sha256","event_tip_currentness_established","historical_handover_accepted","legal_title_transition_established","lineage_custodian_ref","nonclaims","profile_sha256","profile_version","project_id","receipt_version","successor_acceptance_receipt_sha256","successor_custodian_ref","suspended","uses_local_wall_clock"}
DESIGNATION_KEYS={"designation_id","registry_id","registry_epoch","project_id","profile_sha256","designated_d3_subject_sha","designated_d3_receipt_sha256","designated_d3_profile_sha256","designated_event_history_sha256","designated_event_tip_sha256","designation_state","authority_ref","evidence_ref"}
EVENT_KEYS={"seq","event_id","project_id","profile_sha256","designation_id","prev_event_sha256","kind","authority_ref","evidence_ref"}
DESIGNATION_STATES={"ACTIVE","PENDING","REVOKED"}
EVENT_KINDS={"SupersedeTip","PendingReconciliation","MaterialInvalidation","RevokeEvidence"}
PRECEDENCE={"CURRENT":0,"STALE":1,"PENDING":2,"REVOKED":3}
D3_STATES={"CURRENT","DEGRADED","SUSPENDED","SUPERSEDED","TERMINATED"}
NONCLAIMS=(
    "custody-tip currentness does not establish custody health",
    "custody-tip currentness does not alter the D3 custody state",
    "custody-tip currentness is not legal title transfer",
    "custody-tip currentness is not constitutional stewardship transition",
    "custody-tip currentness does not authenticate external authority or evidence references",
    "custody-tip currentness is relative to the supplied designation lineage and not local wall-clock time",
)

class TipCurrentnessError(ValueError): pass

@dataclass(frozen=True)
class QualifiedTipCurrentness:
    _receipt:dict[str,Any]
    def receipt(self)->dict[str,Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(o:Any,keys:set[str],ctx:str)->None:
    if not isinstance(o,dict): raise TipCurrentnessError(f"{ctx}: expected object")
    if set(o)!=keys: raise TipCurrentnessError(f"{ctx}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not _ID.fullmatch(v): raise TipCurrentnessError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or len(v.encode("utf-8"))>512 or v!=v.strip(): raise TipCurrentnessError(f"{ctx}: invalid reference")
    return v
def _hex(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not _HEX64.fullmatch(v): raise TipCurrentnessError(f"{ctx}: invalid sha256")
    return v

def validate_profile(p:dict[str,Any])->None:
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise TipCurrentnessError("profile.profile_version: unsupported")
    _id(p["project_id"],"profile.project_id")
    if p["d3_subject_sha"]!=QUALIFIED_D3_SUBJECT: raise TipCurrentnessError("profile.d3_subject_sha: unqualified subject")
    for key,expected in (
        ("d3_receipt_sha256",QUALIFIED_D3_RECEIPT_SHA256),
        ("d3_profile_sha256",QUALIFIED_D3_PROFILE_SHA256),
        ("d3_event_history_sha256",QUALIFIED_D3_EVENT_HISTORY_SHA256),
        ("d3_event_tip_sha256",QUALIFIED_D3_EVENT_TIP_SHA256),
    ):
        _hex(p[key],f"profile.{key}")
        if p[key]!=expected: raise TipCurrentnessError(f"profile.{key}: unqualified commitment")
    _id(p["designation_registry_id"],"profile.designation_registry_id")
    m=p["max_events"]
    if isinstance(m,bool) or not isinstance(m,int) or m<0 or m>100000: raise TipCurrentnessError("profile.max_events: invalid")

def validate_d3(r:dict[str,Any],p:dict[str,Any])->None:
    _exact(r,D3_KEYS,"d3_receipt")
    if r["receipt_version"]!=D3_RECEIPT_VERSION: raise TipCurrentnessError("d3_receipt: unsupported version")
    if r["project_id"]!=p["project_id"]: raise TipCurrentnessError("d3_receipt: project mismatch")
    if sha256_hex(r)!=QUALIFIED_D3_RECEIPT_SHA256 or sha256_hex(r)!=p["d3_receipt_sha256"]:
        raise TipCurrentnessError("d3_receipt: qualified semantic digest mismatch")
    if r["profile_sha256"]!=p["d3_profile_sha256"]: raise TipCurrentnessError("d3_receipt: profile commitment mismatch")
    if r["event_history_sha256"]!=p["d3_event_history_sha256"]: raise TipCurrentnessError("d3_receipt: event history mismatch")
    if r["event_chain_tip_sha256"]!=p["d3_event_tip_sha256"]: raise TipCurrentnessError("d3_receipt: event tip mismatch")
    if r["custody_state"] not in D3_STATES: raise TipCurrentnessError("d3_receipt: unsupported custody state")
    if r["event_tip_currentness_established"] is not False: raise TipCurrentnessError("d3_receipt: currentness authority contamination")
    if r["uses_local_wall_clock"] is not False: raise TipCurrentnessError("d3_receipt: wall-clock authority contamination")
    if r["historical_handover_accepted"] is not True: raise TipCurrentnessError("d3_receipt: historical acceptance missing")
    if r["legal_title_transition_established"] is not False or r["constitutional_stewardship_transition_established"] is not False:
        raise TipCurrentnessError("d3_receipt: legal/constitutional authority contamination")

def validate_designation(d:dict[str,Any],p:dict[str,Any])->None:
    _exact(d,DESIGNATION_KEYS,"designation")
    _id(d["designation_id"],"designation.designation_id")
    if d["registry_id"]!=p["designation_registry_id"]: raise TipCurrentnessError("designation.registry_id: registry substitution")
    e=d["registry_epoch"]
    if isinstance(e,bool) or not isinstance(e,int) or e<0 or e>10**18: raise TipCurrentnessError("designation.registry_epoch: invalid")
    if d["project_id"]!=p["project_id"]: raise TipCurrentnessError("designation.project_id: project substitution")
    if d["profile_sha256"]!=sha256_hex(p): raise TipCurrentnessError("designation.profile_sha256: profile substitution")
    for k in ("designated_d3_receipt_sha256","designated_d3_profile_sha256","designated_event_history_sha256","designated_event_tip_sha256"):
        _hex(d[k],f"designation.{k}")
    if not isinstance(d["designated_d3_subject_sha"],str) or not re.fullmatch(r"[0-9a-f]{40}",d["designated_d3_subject_sha"]):
        raise TipCurrentnessError("designation.designated_d3_subject_sha: invalid")
    if d["designation_state"] not in DESIGNATION_STATES: raise TipCurrentnessError("designation.designation_state: unsupported")
    _ref(d["authority_ref"],"designation.authority_ref")
    _ref(d["evidence_ref"],"designation.evidence_ref")

def validate_events(events:Any,p:dict[str,Any],d:dict[str,Any])->list[dict[str,Any]]:
    if not isinstance(events,list): raise TipCurrentnessError("events: expected array")
    if len(events)>p["max_events"]: raise TipCurrentnessError("events: exceeds max_events")
    psha=sha256_hex(p); out=[]; ids=set()
    for i,e in enumerate(events):
        _exact(e,EVENT_KEYS,f"events[{i}]")
        if e["seq"]!=i: raise TipCurrentnessError(f"events[{i}].seq: expected {i}")
        eid=_id(e["event_id"],f"events[{i}].event_id")
        if eid in ids: raise TipCurrentnessError(f"events[{i}].event_id: duplicate")
        ids.add(eid)
        if e["project_id"]!=p["project_id"]: raise TipCurrentnessError(f"events[{i}].project_id: project substitution")
        if e["profile_sha256"]!=psha: raise TipCurrentnessError(f"events[{i}].profile_sha256: profile substitution")
        if e["designation_id"]!=d["designation_id"]: raise TipCurrentnessError(f"events[{i}].designation_id: designation substitution")
        if i==0:
            if e["prev_event_sha256"] is not None: raise TipCurrentnessError("events[0].prev_event_sha256: must be null")
        elif e["prev_event_sha256"]!=sha256_hex(out[-1]):
            raise TipCurrentnessError(f"events[{i}].prev_event_sha256: broken event chain")
        if e["kind"] not in EVENT_KINDS: raise TipCurrentnessError(f"events[{i}].kind: unsupported")
        _ref(e["authority_ref"],f"events[{i}].authority_ref")
        _ref(e["evidence_ref"],f"events[{i}].evidence_ref")
        out.append(e)
    return out

def _raise_state(current:str,candidate:str)->str:
    return candidate if PRECEDENCE[candidate]>PRECEDENCE[current] else current

def qualify(p:dict[str,Any],d3:dict[str,Any],d:dict[str,Any],events:Any)->QualifiedTipCurrentness:
    validate_profile(p); validate_d3(d3,p); validate_designation(d,p)
    history=validate_events(events,p,d)
    state="CURRENT"; blockers=[]
    if d["designated_d3_subject_sha"]!=p["d3_subject_sha"]:
        state=_raise_state(state,"STALE"); blockers.append("SUBJECT_NOT_DESIGNATED_CURRENT")
    if d["designated_d3_receipt_sha256"]!=p["d3_receipt_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("RECEIPT_NOT_DESIGNATED_CURRENT")
    if d["designated_d3_profile_sha256"]!=p["d3_profile_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("D3_PROFILE_NOT_DESIGNATED_CURRENT")
    if d["designated_event_history_sha256"]!=p["d3_event_history_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("EVENT_HISTORY_NOT_DESIGNATED_CURRENT")
    if d["designated_event_tip_sha256"]!=p["d3_event_tip_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("EVENT_TIP_NOT_DESIGNATED_CURRENT")
    if d["designation_state"]=="PENDING":
        state=_raise_state(state,"PENDING"); blockers.append("DESIGNATION_PENDING")
    elif d["designation_state"]=="REVOKED":
        state=_raise_state(state,"REVOKED"); blockers.append("DESIGNATION_REVOKED")
    for e in history:
        if e["kind"]=="SupersedeTip":
            state=_raise_state(state,"STALE"); blockers.append("TIP_SUPERSEDED")
        elif e["kind"]=="MaterialInvalidation":
            state=_raise_state(state,"STALE"); blockers.append("MATERIAL_INVALIDATION")
        elif e["kind"]=="PendingReconciliation":
            state=_raise_state(state,"PENDING"); blockers.append("PENDING_RECONCILIATION")
        elif e["kind"]=="RevokeEvidence":
            state=_raise_state(state,"REVOKED"); blockers.append("EVIDENCE_REVOKED")
    receipt={
        "receipt_version":RECEIPT_VERSION,
        "profile_version":PROFILE_VERSION,
        "project_id":p["project_id"],
        "profile_sha256":sha256_hex(p),
        "d3_subject_sha":p["d3_subject_sha"],
        "d3_receipt_sha256":p["d3_receipt_sha256"],
        "d3_profile_sha256":p["d3_profile_sha256"],
        "d3_event_history_sha256":p["d3_event_history_sha256"],
        "d3_event_chain_tip_sha256":p["d3_event_tip_sha256"],
        "d3_custody_state":d3["custody_state"],
        "designation_id":d["designation_id"],
        "designation_registry_id":d["registry_id"],
        "designation_registry_epoch":d["registry_epoch"],
        "designation_sha256":sha256_hex(d),
        "event_count":len(history),
        "event_history_sha256":sha256_hex(history),
        "event_chain_tip_sha256":None if not history else sha256_hex(history[-1]),
        "custody_tip_currentness_state":state,
        "blockers":sorted(set(blockers)),
        "custody_health_established":False,
        "legal_title_transition_established":False,
        "constitutional_stewardship_transition_established":False,
        "execution_authority_established":False,
        "uses_local_wall_clock":False,
        "nonclaims":list(NONCLAIMS),
    }
    return QualifiedTipCurrentness(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(d,dict) or set(d)!={"profile","d3_receipt","designation","events"}:
        raise TipCurrentnessError("case: expected exact keys")
    return d["profile"],d["d3_receipt"],d["designation"],d["events"]

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); out=qualify(*load_case(a.case)).receipt()
    text=json.dumps(out,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text,encoding="utf-8")
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
