#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-return-envelope-currentness-v1"
RECEIPT_VERSION="mycelix-return-envelope-currentness-receipt-v1"
FINANCIAL_RECEIPT_VERSION="mycelix-commons-transition-receipt-v1"
EXPECTED_SUBJECT="270b852e0ac744dfca3a2cb966bf53fce78f2ab9"
EXPECTED_RECEIPT_SHA="60b7217980c615eab453eae25b74b27e75fc67cb336ea60d140f089341d50565"
EXPECTED_PROFILE_SHA="6929b9e089e3f7b272713f6ff8587bf429594a51a9d67d67d0d4468ba72b52e7"
EXPECTED_HISTORY_SHA="b1e75101137bcecc60daba64aa0d232398b78d65e4e946723bd25e5ab23184b7"
EXPECTED_TIP_SHA="f766650cbc8233f016ae3482f46708f395264cbc0baeb869ad7c09dd662d240d"
HEX40=re.compile(r"^[0-9a-f]{40}$")
HEX64=re.compile(r"^[0-9a-f]{64}$")
IDENT=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")

PROFILE_KEYS={"profile_version","project_id","financial_subject_sha","financial_receipt_sha256","financial_profile_sha256","event_history_sha256","event_chain_tip_sha256","unit","designation_registry_id","max_events"}
FINANCIAL_KEYS={"counted_investor_distributions_units","created_entitlement_units","event_chain_tip_sha256","event_count","event_history_sha256","financial_state","grant_or_subsidy_units","handback_accepted","impairments_units","initial_principal_units","legal_transition_complete","nonclaims","preferred_return_cap_units","profile_sha256","profile_version","project_id","qualified_new_capital_units","receipt_version","recoverable_lifecycle_units","remaining_claim_units","required_reserve_units","reserve_balance_units","reserve_compliant","retired_claim_ppm","retired_claim_units","unit"}
DESIGNATION_KEYS={"designation_id","registry_id","registry_epoch","project_id","profile_sha256","designated_financial_receipt_sha256","designated_event_history_sha256","designated_event_chain_tip_sha256","designation_state","authority_ref","evidence_ref"}
EVENT_KEYS={"seq","event_id","project_id","profile_sha256","designation_id","prev_event_sha256","kind","authority_ref","evidence_ref"}
DESIGNATION_STATES={"ACTIVE","PENDING","REVOKED"}
EVENT_KINDS={"SupersedeCheckpoint","PendingReconciliation","MaterialInvalidation","RevokeEvidence"}
PRECEDENCE={"CURRENT":0,"STALE":1,"PENDING":2,"REVOKED":3}
FINANCIAL_STATES={"CLAIM_ACTIVE","RETURN_ENVELOPE_SATISFIED"}
NONCLAIMS=(
 "financial checkpoint currentness is not payment authority",
 "financial checkpoint currentness is not accounting, tax, or securities-law compliance",
 "financial checkpoint currentness is not legal title transfer or handback acceptance",
 "CURRENT means only that the exact qualified Return Envelope checkpoint is currently designated under this registry lineage",
 "financial checkpoint currentness does not authenticate external authority or evidence references",
 "financial checkpoint currentness is relative to the supplied designation lineage and not local wall-clock time",
)

class FinancialCurrentnessError(ValueError): pass

@dataclass(frozen=True)
class QualifiedFinancialCurrentness:
    _receipt:dict[str,Any]
    def receipt(self)->dict[str,Any]: return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str: return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(obj:Any,keys:set[str],ctx:str)->None:
    if not isinstance(obj,dict): raise FinancialCurrentnessError(f"{ctx}: expected object")
    if set(obj)!=keys: raise FinancialCurrentnessError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not IDENT.fullmatch(v): raise FinancialCurrentnessError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or v!=v.strip() or len(v.encode("utf-8"))>512: raise FinancialCurrentnessError(f"{ctx}: invalid reference")
    return v
def _hex(v:Any,ctx:str,n:int=64)->str:
    pat=HEX40 if n==40 else HEX64
    if not isinstance(v,str) or not pat.fullmatch(v): raise FinancialCurrentnessError(f"{ctx}: invalid digest")
    return v
def _nonneg_int(v:Any,ctx:str,maxv:int=10**18)->int:
    if isinstance(v,bool) or not isinstance(v,int) or v<0 or v>maxv: raise FinancialCurrentnessError(f"{ctx}: invalid integer")
    return v

def validate_profile(p:dict[str,Any])->None:
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise FinancialCurrentnessError("profile: unsupported version")
    _id(p["project_id"],"profile.project_id")
    if p["financial_subject_sha"]!=EXPECTED_SUBJECT: raise FinancialCurrentnessError("profile: wrong qualified financial subject")
    if p["financial_receipt_sha256"]!=EXPECTED_RECEIPT_SHA: raise FinancialCurrentnessError("profile: wrong financial receipt")
    if p["financial_profile_sha256"]!=EXPECTED_PROFILE_SHA: raise FinancialCurrentnessError("profile: wrong financial profile")
    if p["event_history_sha256"]!=EXPECTED_HISTORY_SHA: raise FinancialCurrentnessError("profile: wrong financial event history")
    if p["event_chain_tip_sha256"]!=EXPECTED_TIP_SHA: raise FinancialCurrentnessError("profile: wrong financial event tip")
    if p["unit"]!="ZAR-cent": raise FinancialCurrentnessError("profile: unit substitution")
    _id(p["designation_registry_id"],"profile.designation_registry_id")
    _nonneg_int(p["max_events"],"profile.max_events",100000)

def validate_financial_receipt(r:dict[str,Any],p:dict[str,Any])->None:
    _exact(r,FINANCIAL_KEYS,"financial_receipt")
    if r["receipt_version"]!=FINANCIAL_RECEIPT_VERSION: raise FinancialCurrentnessError("financial_receipt: unsupported version")
    if r["project_id"]!=p["project_id"]: raise FinancialCurrentnessError("financial_receipt: project mismatch")
    if r["unit"]!=p["unit"]: raise FinancialCurrentnessError("financial_receipt: unit mismatch")
    if sha256_hex(r)!=p["financial_receipt_sha256"]: raise FinancialCurrentnessError("financial_receipt: semantic digest mismatch")
    if r["profile_sha256"]!=p["financial_profile_sha256"]: raise FinancialCurrentnessError("financial_receipt: profile mismatch")
    if r["event_history_sha256"]!=p["event_history_sha256"]: raise FinancialCurrentnessError("financial_receipt: history mismatch")
    if r["event_chain_tip_sha256"]!=p["event_chain_tip_sha256"]: raise FinancialCurrentnessError("financial_receipt: tip mismatch")
    if r["financial_state"] not in FINANCIAL_STATES: raise FinancialCurrentnessError("financial_receipt: unsupported financial state")
    for k in ("counted_investor_distributions_units","created_entitlement_units","event_count","grant_or_subsidy_units","impairments_units","initial_principal_units","preferred_return_cap_units","qualified_new_capital_units","recoverable_lifecycle_units","remaining_claim_units","required_reserve_units","reserve_balance_units","retired_claim_ppm","retired_claim_units"):
        _nonneg_int(r[k],f"financial_receipt.{k}")
    if not isinstance(r["reserve_compliant"],bool): raise FinancialCurrentnessError("financial_receipt: invalid reserve flag")
    if r["handback_accepted"] is not False: raise FinancialCurrentnessError("financial_receipt: handback authority contamination")
    if r["legal_transition_complete"] is not False: raise FinancialCurrentnessError("financial_receipt: legal-transition authority contamination")
    if not isinstance(r["nonclaims"],list): raise FinancialCurrentnessError("financial_receipt: invalid nonclaims")

def validate_designation(d:dict[str,Any],p:dict[str,Any])->None:
    _exact(d,DESIGNATION_KEYS,"designation")
    _id(d["designation_id"],"designation.designation_id")
    if d["registry_id"]!=p["designation_registry_id"]: raise FinancialCurrentnessError("designation: registry substitution")
    _nonneg_int(d["registry_epoch"],"designation.registry_epoch")
    if d["project_id"]!=p["project_id"]: raise FinancialCurrentnessError("designation: project substitution")
    if d["profile_sha256"]!=sha256_hex(p): raise FinancialCurrentnessError("designation: profile substitution")
    for k in ("designated_financial_receipt_sha256","designated_event_history_sha256","designated_event_chain_tip_sha256"): _hex(d[k],f"designation.{k}")
    if d["designation_state"] not in DESIGNATION_STATES: raise FinancialCurrentnessError("designation: unsupported state")
    _ref(d["authority_ref"],"designation.authority_ref"); _ref(d["evidence_ref"],"designation.evidence_ref")

def validate_events(events:Any,p:dict[str,Any],d:dict[str,Any])->list[dict[str,Any]]:
    if not isinstance(events,list): raise FinancialCurrentnessError("events: expected array")
    if len(events)>p["max_events"]: raise FinancialCurrentnessError("events: exceeds max_events")
    out=[]; ids=set(); psha=sha256_hex(p)
    for i,e in enumerate(events):
        _exact(e,EVENT_KEYS,f"events[{i}]")
        if e["seq"]!=i: raise FinancialCurrentnessError(f"events[{i}].seq: expected {i}")
        eid=_id(e["event_id"],f"events[{i}].event_id")
        if eid in ids: raise FinancialCurrentnessError(f"events[{i}].event_id: duplicate")
        ids.add(eid)
        if e["project_id"]!=p["project_id"]: raise FinancialCurrentnessError(f"events[{i}]: project substitution")
        if e["profile_sha256"]!=psha: raise FinancialCurrentnessError(f"events[{i}]: profile substitution")
        if e["designation_id"]!=d["designation_id"]: raise FinancialCurrentnessError(f"events[{i}]: designation substitution")
        if i==0:
            if e["prev_event_sha256"] is not None: raise FinancialCurrentnessError("events[0]: prev must be null")
        elif e["prev_event_sha256"]!=sha256_hex(out[-1]): raise FinancialCurrentnessError(f"events[{i}]: broken event chain")
        if e["kind"] not in EVENT_KINDS: raise FinancialCurrentnessError(f"events[{i}]: unsupported kind")
        _ref(e["authority_ref"],f"events[{i}].authority_ref"); _ref(e["evidence_ref"],f"events[{i}].evidence_ref")
        out.append(e)
    return out

def _raise_state(cur:str,new:str)->str: return new if PRECEDENCE[new]>PRECEDENCE[cur] else cur

def qualify(p:dict[str,Any],financial:dict[str,Any],d:dict[str,Any],events:Any)->QualifiedFinancialCurrentness:
    validate_profile(p); validate_financial_receipt(financial,p); validate_designation(d,p); history=validate_events(events,p,d)
    state="CURRENT"; blockers=[]
    if d["designated_financial_receipt_sha256"]!=p["financial_receipt_sha256"]: state=_raise_state(state,"STALE"); blockers.append("FINANCIAL_RECEIPT_NOT_DESIGNATED")
    if d["designated_event_history_sha256"]!=p["event_history_sha256"]: state=_raise_state(state,"STALE"); blockers.append("FINANCIAL_HISTORY_NOT_DESIGNATED")
    if d["designated_event_chain_tip_sha256"]!=p["event_chain_tip_sha256"]: state=_raise_state(state,"STALE"); blockers.append("FINANCIAL_TIP_NOT_DESIGNATED")
    if d["designation_state"]=="PENDING": state=_raise_state(state,"PENDING"); blockers.append("DESIGNATION_PENDING")
    elif d["designation_state"]=="REVOKED": state=_raise_state(state,"REVOKED"); blockers.append("DESIGNATION_REVOKED")
    for e in history:
        if e["kind"]=="SupersedeCheckpoint": state=_raise_state(state,"STALE"); blockers.append("CHECKPOINT_SUPERSEDED")
        elif e["kind"]=="MaterialInvalidation": state=_raise_state(state,"STALE"); blockers.append("MATERIAL_INVALIDATION")
        elif e["kind"]=="PendingReconciliation": state=_raise_state(state,"PENDING"); blockers.append("PENDING_RECONCILIATION")
        elif e["kind"]=="RevokeEvidence": state=_raise_state(state,"REVOKED"); blockers.append("EVIDENCE_REVOKED")
    receipt={
        "receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,"project_id":p["project_id"],"profile_sha256":sha256_hex(p),
        "financial_subject_sha":p["financial_subject_sha"],"financial_receipt_sha256":p["financial_receipt_sha256"],"financial_profile_sha256":p["financial_profile_sha256"],
        "event_history_sha256":p["event_history_sha256"],"event_chain_tip_sha256":p["event_chain_tip_sha256"],"financial_state":financial["financial_state"],
        "remaining_claim_units":financial["remaining_claim_units"],"reserve_balance_units":financial["reserve_balance_units"],"reserve_compliant":financial["reserve_compliant"],"unit":financial["unit"],
        "designation_id":d["designation_id"],"designation_registry_id":d["registry_id"],"designation_registry_epoch":d["registry_epoch"],"designation_sha256":sha256_hex(d),
        "currentness_event_count":len(history),"currentness_event_history_sha256":sha256_hex(history),"currentness_event_chain_tip_sha256":None if not history else sha256_hex(history[-1]),
        "financial_checkpoint_currentness_state":state,"blockers":sorted(set(blockers)),"payment_authority_established":False,"accounting_compliance_established":False,
        "tax_compliance_established":False,"securities_law_compliance_established":False,"legal_title_transition_established":False,"handover_accepted":False,"uses_local_wall_clock":False,
        "nonclaims":list(NONCLAIMS),
    }
    return QualifiedFinancialCurrentness(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(d,dict) or set(d)!={"profile","financial_receipt","designation","events"}: raise FinancialCurrentnessError("case: exact keys required")
    return d["profile"],d["financial_receipt"],d["designation"],d["events"]

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path); a=ap.parse_args()
    out=qualify(*load_case(a.case)).receipt(); text=json.dumps(out,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text,encoding="utf-8")
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
