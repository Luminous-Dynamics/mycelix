#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-physical-handback-readiness-v1"
RECEIPT_VERSION="mycelix-physical-handback-readiness-receipt-v1"
_ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX40=re.compile(r"^[0-9a-f]{40}$")
_HEX64=re.compile(r"^[0-9a-f]{64}$")
STATES={"PASS","FAIL","NOT_ASSESSED","NOT_APPLICABLE"}
PROFILE_KEYS={"profile_version","project_id","parent_subject_sha","parent_transition_receipt_sha256","currency_unit","residual_life_unit","max_amount_units","required_handback_reserve_units","max_deferred_maintenance_units","components"}
COMPONENT_PROFILE_KEYS={"component_id","required","min_residual_life_units"}
ASSESSMENT_KEYS={"assessment_id","project_id","profile_sha256","parent_transition_receipt_sha256","outgoing_operator_ref","assessor_ref","components","reserve_balance_units","deferred_maintenance_units","deferred_maintenance_evidence_ref","evidence_ref"}
COMPONENT_OBS_KEYS={"condition_state","residual_life_units","evidence_ref"}
NONCLAIMS=(
"physical handback readiness is not handover acceptance",
"physical handback readiness is not legal title transfer",
"physical handback readiness is not statutory inspection compliance",
"physical handback readiness is not engineering fitness outside the frozen component profile",
"assessor reference separation is not proof of external identity or licensure",
"reserve sufficiency is not itself proof of physical condition",
)

class PhysicalError(ValueError): pass

@dataclass(frozen=True)
class QualifiedPhysical:
    _receipt:dict[str,Any]
    def receipt(self)->dict[str,Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(o:dict[str,Any], keys:set[str], ctx:str)->None:
    if set(o)!=keys: raise PhysicalError(f"{ctx}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not _ID.fullmatch(v): raise PhysicalError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or len(v.encode())>512 or v!=v.strip(): raise PhysicalError(f"{ctx}: invalid reference")
    return v
def _amount(v:Any,maxv:int,ctx:str)->int:
    if isinstance(v,bool) or not isinstance(v,int) or v<0 or v>maxv: raise PhysicalError(f"{ctx}: invalid integer amount")
    return v
def _residual(v:Any,ctx:str)->int:
    if isinstance(v,bool) or not isinstance(v,int) or v<0 or v>10**9: raise PhysicalError(f"{ctx}: invalid residual-life value")
    return v

def validate_profile(p:dict[str,Any])->None:
    if not isinstance(p,dict): raise PhysicalError("profile: expected object")
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise PhysicalError("profile.profile_version: unsupported")
    _id(p["project_id"],"profile.project_id")
    if not isinstance(p["parent_subject_sha"],str) or not _HEX40.fullmatch(p["parent_subject_sha"]): raise PhysicalError("profile.parent_subject_sha: invalid")
    if not isinstance(p["parent_transition_receipt_sha256"],str) or not _HEX64.fullmatch(p["parent_transition_receipt_sha256"]): raise PhysicalError("profile.parent_transition_receipt_sha256: invalid")
    _id(p["currency_unit"],"profile.currency_unit"); _id(p["residual_life_unit"],"profile.residual_life_unit")
    maxa=p["max_amount_units"]
    if isinstance(maxa,bool) or not isinstance(maxa,int) or maxa<1 or maxa>10**24: raise PhysicalError("profile.max_amount_units: invalid")
    _amount(p["required_handback_reserve_units"],maxa,"profile.required_handback_reserve_units")
    _amount(p["max_deferred_maintenance_units"],maxa,"profile.max_deferred_maintenance_units")
    comps=p["components"]
    if not isinstance(comps,list) or not comps: raise PhysicalError("profile.components: expected non-empty array")
    ids=set()
    for i,c in enumerate(comps):
        if not isinstance(c,dict): raise PhysicalError(f"profile.components[{i}]: expected object")
        _exact(c,COMPONENT_PROFILE_KEYS,f"profile.components[{i}]")
        cid=_id(c["component_id"],f"profile.components[{i}].component_id")
        if cid in ids: raise PhysicalError("profile.components: duplicate component_id")
        ids.add(cid)
        if not isinstance(c["required"],bool): raise PhysicalError(f"profile.components[{i}].required: expected bool")
        _residual(c["min_residual_life_units"],f"profile.components[{i}].min_residual_life_units")

def validate_parent(parent:dict[str,Any],p:dict[str,Any])->None:
    if not isinstance(parent,dict): raise PhysicalError("parent_receipt: expected object")
    if sha256_hex(parent)!=p["parent_transition_receipt_sha256"]: raise PhysicalError("parent_receipt: semantic digest mismatch")
    for k in ("project_id","financial_state","remaining_claim_units"):
        if k not in parent: raise PhysicalError(f"parent_receipt: missing {k}")
    if parent["project_id"]!=p["project_id"]: raise PhysicalError("parent_receipt: project mismatch")
    if isinstance(parent["remaining_claim_units"],bool) or not isinstance(parent["remaining_claim_units"],int) or parent["remaining_claim_units"]<0: raise PhysicalError("parent_receipt.remaining_claim_units: invalid")

def validate_assessment(a:dict[str,Any],p:dict[str,Any])->None:
    if not isinstance(a,dict): raise PhysicalError("assessment: expected object")
    _exact(a,ASSESSMENT_KEYS,"assessment")
    _id(a["assessment_id"],"assessment.assessment_id")
    if a["project_id"]!=p["project_id"]: raise PhysicalError("assessment: project substitution")
    if a["profile_sha256"]!=sha256_hex(p): raise PhysicalError("assessment: profile substitution")
    if a["parent_transition_receipt_sha256"]!=p["parent_transition_receipt_sha256"]: raise PhysicalError("assessment: parent substitution")
    op=_ref(a["outgoing_operator_ref"],"assessment.outgoing_operator_ref")
    assessor=_ref(a["assessor_ref"],"assessment.assessor_ref")
    if op==assessor: raise PhysicalError("assessment: outgoing operator cannot be sole assessor")
    _ref(a["deferred_maintenance_evidence_ref"],"assessment.deferred_maintenance_evidence_ref")
    _ref(a["evidence_ref"],"assessment.evidence_ref")
    _amount(a["reserve_balance_units"],p["max_amount_units"],"assessment.reserve_balance_units")
    _amount(a["deferred_maintenance_units"],p["max_amount_units"],"assessment.deferred_maintenance_units")
    observed=a["components"]
    if not isinstance(observed,dict): raise PhysicalError("assessment.components: expected object")
    expected={c["component_id"] for c in p["components"]}
    _exact(observed,expected,"assessment.components")
    byid={c["component_id"]:c for c in p["components"]}
    for cid,o in observed.items():
        if not isinstance(o,dict): raise PhysicalError(f"assessment.components.{cid}: expected object")
        _exact(o,COMPONENT_OBS_KEYS,f"assessment.components.{cid}")
        state=o["condition_state"]
        if state not in STATES: raise PhysicalError(f"assessment.components.{cid}.condition_state: unsupported")
        residual=_residual(o["residual_life_units"],f"assessment.components.{cid}.residual_life_units")
        _ref(o["evidence_ref"],f"assessment.components.{cid}.evidence_ref")
        if byid[cid]["required"] and state=="NOT_APPLICABLE": raise PhysicalError(f"assessment.components.{cid}: required component cannot be NOT_APPLICABLE")
        if not byid[cid]["required"] and state!="NOT_APPLICABLE": raise PhysicalError(f"assessment.components.{cid}: non-required component must be NOT_APPLICABLE")
        if state in {"NOT_ASSESSED","NOT_APPLICABLE"} and residual!=0: raise PhysicalError(f"assessment.components.{cid}: unassessed/N/A residual must be zero")

def qualify(p:dict[str,Any],parent:dict[str,Any],a:dict[str,Any])->QualifiedPhysical:
    validate_profile(p); validate_parent(parent,p); validate_assessment(a,p)
    byid={c["component_id"]:c for c in p["components"]}
    fail=[]; incomplete=[]; residual_deficits=[]
    for cid in sorted(byid):
        req=byid[cid]; obs=a["components"][cid]; state=obs["condition_state"]
        if not req["required"]: continue
        if state=="FAIL": fail.append(cid)
        elif state=="NOT_ASSESSED": incomplete.append(cid)
        elif state=="PASS" and obs["residual_life_units"]<req["min_residual_life_units"]:
            residual_deficits.append(cid)
    blockers=[f"CONDITION_FAIL:{x}" for x in fail]
    blockers += [f"RESIDUAL_LIFE_DEFICIT:{x}" for x in residual_deficits]
    deferred_excess=a["deferred_maintenance_units"]>p["max_deferred_maintenance_units"]
    if deferred_excess: blockers.append("DEFERRED_MAINTENANCE_EXCESS")
    blockers += [f"NOT_ASSESSED:{x}" for x in incomplete]
    reserve_deficit=a["reserve_balance_units"]<p["required_handback_reserve_units"]
    if reserve_deficit: blockers.append("HANDBACK_RESERVE_DEFICIENT")
    if fail or residual_deficits or deferred_excess: state="REMEDIATION_REQUIRED"
    elif incomplete: state="ASSESSMENT_INCOMPLETE"
    elif reserve_deficit: state="RESERVE_DEFICIENT"
    else: state="PHYSICAL_CONDITION_ACCEPTABLE"
    receipt={
      "receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,
      "project_id":p["project_id"],"profile_sha256":sha256_hex(p),
      "parent_subject_sha":p["parent_subject_sha"],"parent_transition_receipt_sha256":p["parent_transition_receipt_sha256"],
      "parent_financial_state":parent["financial_state"],"parent_remaining_claim_units":parent["remaining_claim_units"],
      "assessment_id":a["assessment_id"],"assessment_sha256":sha256_hex(a),
      "outgoing_operator_ref":a["outgoing_operator_ref"],"assessor_ref":a["assessor_ref"],
      "currency_unit":p["currency_unit"],"residual_life_unit":p["residual_life_unit"],
      "required_handback_reserve_units":p["required_handback_reserve_units"],
      "reserve_balance_units":a["reserve_balance_units"],
      "max_deferred_maintenance_units":p["max_deferred_maintenance_units"],
      "deferred_maintenance_units":a["deferred_maintenance_units"],
      "components":{cid:{
          "required":byid[cid]["required"],
          "min_residual_life_units":byid[cid]["min_residual_life_units"],
          "condition_state":a["components"][cid]["condition_state"],
          "residual_life_units":a["components"][cid]["residual_life_units"],
      } for cid in sorted(byid)},
      "blockers":blockers,"physical_state":state,
      "handover_accepted":False,"legal_transition_complete":False,
      "nonclaims":list(NONCLAIMS)}
    return QualifiedPhysical(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(d,dict) or set(d)!={"profile","parent_receipt","assessment"}: raise PhysicalError("case: expected exact keys")
    return d["profile"],d["parent_receipt"],d["assessment"]

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    x=ap.parse_args(); text=json.dumps(qualify(*load_case(x.case)).receipt(),sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if x.receipt_out: x.receipt_out.write_text(text,encoding="utf-8")
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
