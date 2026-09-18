#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-claim-registry-currentness-v1"
RECEIPT_VERSION="mycelix-claim-registry-currentness-receipt-v1"
CLAIM_RECEIPT_VERSION="mycelix-claim-registry-receipt-v1"
EXPECTED_SUBJECT="9f940f915e6aa2bf253f859045d104e80406dd4c"
EXPECTED_RECEIPT_SHA="ec4431b5e4ee5ab8c91a4b2a24285d617c9c37d81d65473c997a2633a0b8b628"
EXPECTED_PROFILE_SHA="6011726fcb6af452a3c8d8d8027c000e9d324a2776a7739e4f22eb76de3ceb40"
EXPECTED_HISTORY_SHA="db65331686b023f8683cff201a608354c3a492d8baf483f5fa8c2b7099b92a92"
EXPECTED_TIP_SHA="9a9a2c7a462f9a1b449891a63b7516f7e13460caaf1fda69d707a00938ebaa05"
HEX40=re.compile(r"^[0-9a-f]{40}$")
HEX64=re.compile(r"^[0-9a-f]{64}$")
IDENT=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")

PROFILE_KEYS={
 "profile_version","project_id","claim_registry_subject_sha","claim_registry_receipt_sha256",
 "claim_registry_profile_sha256","operation_history_sha256","operation_chain_tip_sha256",
 "designation_registry_id","max_events",
}
CLAIM_KEYS={
 "active_claim_total_units","active_slices","claim_class_id","commons_asset_transferable",
 "conservation_passed","holder_totals","nonclaims","operation_chain_tip_sha256",
 "operation_count","operation_history_sha256","parent_remaining_claim_units",
 "parent_subject_sha","parent_transition_receipt_sha256","profile_sha256","project_id",
 "receipt_version","registry_version","unit",
}
DESIGNATION_KEYS={
 "designation_id","registry_id","registry_epoch","project_id","profile_sha256",
 "designated_claim_registry_receipt_sha256","designated_operation_history_sha256",
 "designated_operation_chain_tip_sha256","designation_state","authority_ref","evidence_ref",
}
EVENT_KEYS={
 "seq","event_id","project_id","profile_sha256","designation_id","prev_event_sha256",
 "kind","authority_ref","evidence_ref",
}
DESIGNATION_STATES={"ACTIVE","PENDING","REVOKED"}
EVENT_KINDS={"SupersedeTip","PendingReconciliation","MaterialInvalidation","RevokeEvidence"}
PRECEDENCE={"CURRENT":0,"STALE":1,"PENDING":2,"REVOKED":3}
NONCLAIMS=(
 "claim-registry currentness is not project payment authority",
 "claim-registry currentness is not ownership of the commons asset",
 "claim-registry currentness is not constitutional stewardship or operator authority",
 "claim-registry currentness does not authenticate external holder identities or signatures",
 "CURRENT means the exact qualified holder snapshot is currently designated under this registry lineage",
 "claim-registry currentness is relative to the supplied designation lineage and not local wall-clock time",
)

class ClaimCurrentnessError(ValueError): pass

@dataclass(frozen=True)
class QualifiedClaimCurrentness:
    _receipt: dict[str,Any]
    def receipt(self)->dict[str,Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(obj:Any,keys:set[str],ctx:str)->None:
    if not isinstance(obj,dict): raise ClaimCurrentnessError(f"{ctx}: expected object")
    if set(obj)!=keys:
        raise ClaimCurrentnessError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not IDENT.fullmatch(v): raise ClaimCurrentnessError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or v!=v.strip() or len(v.encode("utf-8"))>512:
        raise ClaimCurrentnessError(f"{ctx}: invalid reference")
    return v
def _hex(v:Any,ctx:str,n:int=64)->str:
    pat=HEX40 if n==40 else HEX64
    if not isinstance(v,str) or not pat.fullmatch(v): raise ClaimCurrentnessError(f"{ctx}: invalid digest")
    return v
def _nonneg_int(v:Any,ctx:str,maxv:int=10**18)->int:
    if isinstance(v,bool) or not isinstance(v,int) or v<0 or v>maxv: raise ClaimCurrentnessError(f"{ctx}: invalid integer")
    return v

def validate_profile(p:dict[str,Any])->None:
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise ClaimCurrentnessError("profile: unsupported version")
    _id(p["project_id"],"profile.project_id")
    if p["claim_registry_subject_sha"]!=EXPECTED_SUBJECT: raise ClaimCurrentnessError("profile: wrong qualified claim-registry subject")
    if p["claim_registry_receipt_sha256"]!=EXPECTED_RECEIPT_SHA: raise ClaimCurrentnessError("profile: wrong qualified claim-registry receipt")
    if p["claim_registry_profile_sha256"]!=EXPECTED_PROFILE_SHA: raise ClaimCurrentnessError("profile: wrong claim-registry profile")
    if p["operation_history_sha256"]!=EXPECTED_HISTORY_SHA: raise ClaimCurrentnessError("profile: wrong operation history")
    if p["operation_chain_tip_sha256"]!=EXPECTED_TIP_SHA: raise ClaimCurrentnessError("profile: wrong operation chain tip")
    _id(p["designation_registry_id"],"profile.designation_registry_id")
    _nonneg_int(p["max_events"],"profile.max_events",100000)

def validate_claim_receipt(r:dict[str,Any],p:dict[str,Any])->None:
    _exact(r,CLAIM_KEYS,"claim_registry_receipt")
    if r["receipt_version"]!=CLAIM_RECEIPT_VERSION: raise ClaimCurrentnessError("claim_registry_receipt: unsupported version")
    if r["project_id"]!=p["project_id"]: raise ClaimCurrentnessError("claim_registry_receipt: project mismatch")
    if sha256_hex(r)!=p["claim_registry_receipt_sha256"]: raise ClaimCurrentnessError("claim_registry_receipt: semantic digest mismatch")
    if r["profile_sha256"]!=p["claim_registry_profile_sha256"]: raise ClaimCurrentnessError("claim_registry_receipt: profile mismatch")
    if r["operation_history_sha256"]!=p["operation_history_sha256"]: raise ClaimCurrentnessError("claim_registry_receipt: history mismatch")
    if r["operation_chain_tip_sha256"]!=p["operation_chain_tip_sha256"]: raise ClaimCurrentnessError("claim_registry_receipt: tip mismatch")
    if r["conservation_passed"] is not True: raise ClaimCurrentnessError("claim_registry_receipt: conservation not established")
    if r["commons_asset_transferable"] is not False: raise ClaimCurrentnessError("claim_registry_receipt: asset-boundary contamination")
    active=_nonneg_int(r["active_claim_total_units"],"claim_registry_receipt.active_claim_total_units")
    parent=_nonneg_int(r["parent_remaining_claim_units"],"claim_registry_receipt.parent_remaining_claim_units")
    if active!=parent: raise ClaimCurrentnessError("claim_registry_receipt: conserved total mismatch")
    if not isinstance(r["active_slices"],list) or not isinstance(r["holder_totals"],list) or not isinstance(r["nonclaims"],list):
        raise ClaimCurrentnessError("claim_registry_receipt: invalid array field")
    _nonneg_int(r["operation_count"],"claim_registry_receipt.operation_count",1000000)
    _hex(r["parent_subject_sha"],"claim_registry_receipt.parent_subject_sha",40)
    _hex(r["parent_transition_receipt_sha256"],"claim_registry_receipt.parent_transition_receipt_sha256",64)

def validate_designation(d:dict[str,Any],p:dict[str,Any])->None:
    _exact(d,DESIGNATION_KEYS,"designation")
    _id(d["designation_id"],"designation.designation_id")
    if d["registry_id"]!=p["designation_registry_id"]: raise ClaimCurrentnessError("designation: registry substitution")
    _nonneg_int(d["registry_epoch"],"designation.registry_epoch")
    if d["project_id"]!=p["project_id"]: raise ClaimCurrentnessError("designation: project substitution")
    if d["profile_sha256"]!=sha256_hex(p): raise ClaimCurrentnessError("designation: profile substitution")
    for k in ("designated_claim_registry_receipt_sha256","designated_operation_history_sha256","designated_operation_chain_tip_sha256"):
        _hex(d[k],f"designation.{k}",64)
    if d["designation_state"] not in DESIGNATION_STATES: raise ClaimCurrentnessError("designation: unsupported state")
    _ref(d["authority_ref"],"designation.authority_ref")
    _ref(d["evidence_ref"],"designation.evidence_ref")

def validate_events(events:Any,p:dict[str,Any],d:dict[str,Any])->list[dict[str,Any]]:
    if not isinstance(events,list): raise ClaimCurrentnessError("events: expected array")
    if len(events)>p["max_events"]: raise ClaimCurrentnessError("events: exceeds max_events")
    out=[]; ids=set(); psha=sha256_hex(p)
    for i,e in enumerate(events):
        _exact(e,EVENT_KEYS,f"events[{i}]")
        if e["seq"]!=i: raise ClaimCurrentnessError(f"events[{i}].seq: expected {i}")
        eid=_id(e["event_id"],f"events[{i}].event_id")
        if eid in ids: raise ClaimCurrentnessError(f"events[{i}].event_id: duplicate")
        ids.add(eid)
        if e["project_id"]!=p["project_id"]: raise ClaimCurrentnessError(f"events[{i}]: project substitution")
        if e["profile_sha256"]!=psha: raise ClaimCurrentnessError(f"events[{i}]: profile substitution")
        if e["designation_id"]!=d["designation_id"]: raise ClaimCurrentnessError(f"events[{i}]: designation substitution")
        if i==0:
            if e["prev_event_sha256"] is not None: raise ClaimCurrentnessError("events[0]: prev must be null")
        elif e["prev_event_sha256"]!=sha256_hex(out[-1]):
            raise ClaimCurrentnessError(f"events[{i}]: broken event chain")
        if e["kind"] not in EVENT_KINDS: raise ClaimCurrentnessError(f"events[{i}]: unsupported kind")
        _ref(e["authority_ref"],f"events[{i}].authority_ref")
        _ref(e["evidence_ref"],f"events[{i}].evidence_ref")
        out.append(e)
    return out

def _raise_state(cur:str,new:str)->str:
    return new if PRECEDENCE[new]>PRECEDENCE[cur] else cur

def qualify(p:dict[str,Any],claim:dict[str,Any],d:dict[str,Any],events:Any)->QualifiedClaimCurrentness:
    validate_profile(p); validate_claim_receipt(claim,p); validate_designation(d,p)
    history=validate_events(events,p,d)
    state="CURRENT"; blockers=[]
    if d["designated_claim_registry_receipt_sha256"]!=p["claim_registry_receipt_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("CLAIM_REGISTRY_RECEIPT_NOT_DESIGNATED")
    if d["designated_operation_history_sha256"]!=p["operation_history_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("OPERATION_HISTORY_NOT_DESIGNATED")
    if d["designated_operation_chain_tip_sha256"]!=p["operation_chain_tip_sha256"]:
        state=_raise_state(state,"STALE"); blockers.append("OPERATION_TIP_NOT_DESIGNATED")
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
    holder_snapshot={
        "active_claim_total_units":claim["active_claim_total_units"],
        "active_slices":claim["active_slices"],
        "holder_totals":claim["holder_totals"],
    }
    receipt={
        "receipt_version":RECEIPT_VERSION,
        "profile_version":PROFILE_VERSION,
        "project_id":p["project_id"],
        "profile_sha256":sha256_hex(p),
        "claim_registry_subject_sha":p["claim_registry_subject_sha"],
        "claim_registry_receipt_sha256":p["claim_registry_receipt_sha256"],
        "claim_registry_profile_sha256":p["claim_registry_profile_sha256"],
        "operation_history_sha256":p["operation_history_sha256"],
        "operation_chain_tip_sha256":p["operation_chain_tip_sha256"],
        "holder_snapshot_sha256":sha256_hex(holder_snapshot),
        "active_claim_total_units":claim["active_claim_total_units"],
        "parent_remaining_claim_units":claim["parent_remaining_claim_units"],
        "designation_id":d["designation_id"],
        "designation_registry_id":d["registry_id"],
        "designation_registry_epoch":d["registry_epoch"],
        "designation_sha256":sha256_hex(d),
        "event_count":len(history),
        "event_history_sha256":sha256_hex(history),
        "event_chain_tip_sha256":None if not history else sha256_hex(history[-1]),
        "claim_registry_currentness_state":state,
        "blockers":sorted(set(blockers)),
        "payment_authority_established":False,
        "asset_title_authority_established":False,
        "constitutional_authority_established":False,
        "operator_authority_established":False,
        "uses_local_wall_clock":False,
        "nonclaims":list(NONCLAIMS),
    }
    return QualifiedClaimCurrentness(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(d,dict) or set(d)!={"profile","claim_registry_receipt","designation","events"}:
        raise ClaimCurrentnessError("case: exact keys required")
    return d["profile"],d["claim_registry_receipt"],d["designation"],d["events"]

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); out=qualify(*load_case(a.case)).receipt()
    text=json.dumps(out,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text,encoding="utf-8")
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
