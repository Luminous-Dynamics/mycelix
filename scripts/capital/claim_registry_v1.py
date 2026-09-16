#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REGISTRY_VERSION="mycelix-capital-claim-registry-single-class-v1"
RECEIPT_VERSION="mycelix-claim-registry-receipt-v1"
CLAIM_CLASS="single-priority-bounded-economic-claim-v1"
_ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
HEX40=re.compile(r"^[0-9a-f]{40}$")
HEX64=re.compile(r"^[0-9a-f]{64}$")
PROFILE_KEYS={"registry_version","project_id","unit","claim_class_id","parent_subject_sha","parent_transition_receipt_sha256","parent_financial_profile_sha256","parent_remaining_claim_units","max_face_units","max_transaction_price_units","max_operations"}
OP_KEYS={"seq","operation_id","project_id","profile_sha256","prev_operation_sha256","kind","inputs","outputs","transaction_price_units","authority_ref","evidence_ref"}
OUT_KEYS={"slice_id","holder_ref","face_units"}
KINDS={"GenesisIssue","Transfer","Split","Merge"}
NONCLAIMS=(
"claim ownership is not ownership of the commons asset",
"claim ownership is not constitutional stewardship authority",
"claim ownership is not operator authority",
"secondary transaction price does not change project liability",
"registry validity is not securities-law compliance or market valuation",
"registry validity is not proof of external holder identity or signature authenticity",
)

class RegistryError(ValueError): pass

@dataclass(frozen=True)
class QualifiedRegistry:
    _receipt: dict[str,Any]
    def receipt(self)->dict[str,Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(obj:dict[str,Any], keys:set[str], ctx:str)->None:
    if set(obj)!=keys:
        raise RegistryError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not _ID.fullmatch(v): raise RegistryError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or len(v.encode())>512 or v!=v.strip(): raise RegistryError(f"{ctx}: invalid reference")
    return v
def _amount(v:Any,maxv:int,ctx:str)->int:
    if isinstance(v,bool) or not isinstance(v,int) or v<0 or v>maxv: raise RegistryError(f"{ctx}: invalid amount")
    return v

def validate_profile(p:dict[str,Any])->dict[str,Any]:
    if not isinstance(p,dict): raise RegistryError("profile: expected object")
    _exact(p,PROFILE_KEYS,"profile")
    if p["registry_version"]!=REGISTRY_VERSION: raise RegistryError("profile.registry_version: unsupported")
    if p["claim_class_id"]!=CLAIM_CLASS: raise RegistryError("profile.claim_class_id: unsupported")
    _id(p["project_id"],"profile.project_id"); _id(p["unit"],"profile.unit")
    if not isinstance(p["parent_subject_sha"],str) or not HEX40.fullmatch(p["parent_subject_sha"]): raise RegistryError("profile.parent_subject_sha: invalid")
    for k in ("parent_transition_receipt_sha256","parent_financial_profile_sha256"):
        if not isinstance(p[k],str) or not HEX64.fullmatch(p[k]): raise RegistryError(f"profile.{k}: invalid")
    max_face=p["max_face_units"]
    if isinstance(max_face,bool) or not isinstance(max_face,int) or max_face<1 or max_face>10**24: raise RegistryError("profile.max_face_units: invalid")
    _amount(p["parent_remaining_claim_units"],max_face,"profile.parent_remaining_claim_units")
    max_price=p["max_transaction_price_units"]
    if isinstance(max_price,bool) or not isinstance(max_price,int) or max_price<0 or max_price>10**24: raise RegistryError("profile.max_transaction_price_units: invalid")
    mo=p["max_operations"]
    if isinstance(mo,bool) or not isinstance(mo,int) or mo<1 or mo>100000: raise RegistryError("profile.max_operations: invalid")
    return p

def validate_parent(parent:dict[str,Any],p:dict[str,Any])->None:
    if not isinstance(parent,dict): raise RegistryError("parent_receipt: expected object")
    if sha256_hex(parent)!=p["parent_transition_receipt_sha256"]: raise RegistryError("parent_receipt: semantic digest mismatch")
    for key in ("project_id","unit","profile_sha256","remaining_claim_units"):
        if key not in parent: raise RegistryError(f"parent_receipt: missing {key}")
    if parent["project_id"]!=p["project_id"]: raise RegistryError("parent_receipt: project mismatch")
    if parent["unit"]!=p["unit"]: raise RegistryError("parent_receipt: unit mismatch")
    if parent["profile_sha256"]!=p["parent_financial_profile_sha256"]: raise RegistryError("parent_receipt: financial profile mismatch")
    if parent["remaining_claim_units"]!=p["parent_remaining_claim_units"]: raise RegistryError("parent_receipt: remaining claim mismatch")

def qualify(p:dict[str,Any], parent:dict[str,Any], ops:Any)->QualifiedRegistry:
    validate_profile(p); validate_parent(parent,p)
    if not isinstance(ops,list) or not ops: raise RegistryError("operations: expected non-empty array")
    if len(ops)>p["max_operations"]: raise RegistryError("operations: exceeds max_operations")
    psha=sha256_hex(p)
    active:dict[str,dict[str,Any]]={}
    ever:set[str]=set()
    seen_operations:set[str]=set()
    normalized=[]
    for i,op in enumerate(ops):
        if not isinstance(op,dict): raise RegistryError(f"operations[{i}]: expected object")
        _exact(op,OP_KEYS,f"operations[{i}]")
        if op["seq"]!=i: raise RegistryError(f"operations[{i}].seq: expected {i}")
        operation_id=_id(op["operation_id"],f"operations[{i}].operation_id")
        if operation_id in seen_operations: raise RegistryError(f"operations[{i}].operation_id: duplicate")
        seen_operations.add(operation_id)
        if op["project_id"]!=p["project_id"]: raise RegistryError(f"operations[{i}].project_id: project substitution")
        if op["profile_sha256"]!=psha: raise RegistryError(f"operations[{i}].profile_sha256: profile substitution")
        prev=op["prev_operation_sha256"]
        if i==0:
            if prev is not None: raise RegistryError("operations[0].prev_operation_sha256: must be null")
        elif prev!=sha256_hex(normalized[-1]):
            raise RegistryError(f"operations[{i}].prev_operation_sha256: broken operation chain")
        kind=op["kind"]
        if kind not in KINDS: raise RegistryError(f"operations[{i}].kind: unsupported kind")
        _ref(op["authority_ref"],f"operations[{i}].authority_ref"); _ref(op["evidence_ref"],f"operations[{i}].evidence_ref")
        price=_amount(op["transaction_price_units"],p["max_transaction_price_units"],f"operations[{i}].transaction_price_units")
        inputs=op["inputs"]; outputs=op["outputs"]
        if not isinstance(inputs,list) or not all(isinstance(x,str) for x in inputs): raise RegistryError(f"operations[{i}].inputs: invalid")
        if not isinstance(outputs,list) or not outputs: raise RegistryError(f"operations[{i}].outputs: invalid")
        if len(set(inputs))!=len(inputs): raise RegistryError(f"operations[{i}].inputs: duplicate slice")
        consumed=[]
        for sid in inputs:
            if sid not in active: raise RegistryError(f"operations[{i}].inputs: stale or unknown slice")
            consumed.append(active[sid])
        created=[]
        new_ids:set[str]=set()
        for j,out in enumerate(outputs):
            if not isinstance(out,dict): raise RegistryError(f"operations[{i}].outputs[{j}]: invalid")
            _exact(out,OUT_KEYS,f"operations[{i}].outputs[{j}]")
            sid=_id(out["slice_id"],f"operations[{i}].outputs[{j}].slice_id")
            if sid in ever or sid in new_ids: raise RegistryError(f"operations[{i}].outputs[{j}].slice_id: reused")
            new_ids.add(sid)
            holder=_ref(out["holder_ref"],f"operations[{i}].outputs[{j}].holder_ref")
            face=_amount(out["face_units"],p["max_face_units"],f"operations[{i}].outputs[{j}].face_units")
            if face==0: raise RegistryError(f"operations[{i}].outputs[{j}].face_units: zero forbidden")
            created.append({"slice_id":sid,"holder_ref":holder,"face_units":face})
        if kind=="GenesisIssue":
            if i!=0 or inputs or len(created)!=1 or price!=0: raise RegistryError("GenesisIssue: invalid shape")
            if created[0]["face_units"]!=p["parent_remaining_claim_units"]: raise RegistryError("GenesisIssue: must equal parent remaining claim")
        else:
            if i==0: raise RegistryError("operations[0]: must be GenesisIssue")
            in_face=sum(x["face_units"] for x in consumed); out_face=sum(x["face_units"] for x in created)
            if in_face!=out_face: raise RegistryError(f"operations[{i}]: face conservation violated")
            if kind=="Transfer" and not (len(consumed)==1 and len(created)==1): raise RegistryError(f"operations[{i}]: Transfer shape invalid")
            if kind=="Split":
                if not (len(consumed)==1 and len(created)>=2): raise RegistryError(f"operations[{i}]: Split shape invalid")
                if any(x["holder_ref"]!=consumed[0]["holder_ref"] for x in created):
                    raise RegistryError(f"operations[{i}]: Split must preserve holder")
            if kind=="Merge":
                if not (len(consumed)>=2 and len(created)==1): raise RegistryError(f"operations[{i}]: Merge shape invalid")
                holders={x["holder_ref"] for x in consumed}
                if len(holders)!=1 or created[0]["holder_ref"] not in holders: raise RegistryError(f"operations[{i}]: Merge requires one current holder")
        for sid in inputs: del active[sid]
        for out in created:
            active[out["slice_id"]]=out; ever.add(out["slice_id"])
        if sum(x["face_units"] for x in active.values())!=p["parent_remaining_claim_units"]:
            raise RegistryError(f"operations[{i}]: aggregate claim no longer matches parent remaining claim")
        normalized.append(op)
    if ops[0]["kind"]!="GenesisIssue": raise RegistryError("operations[0]: must be GenesisIssue")
    slices=sorted(active.values(),key=lambda x:x["slice_id"])
    holders={}
    for s in slices: holders[s["holder_ref"]]=holders.get(s["holder_ref"],0)+s["face_units"]
    holder_totals=[{"holder_ref":h,"face_units":holders[h]} for h in sorted(holders)]
    receipt={
      "receipt_version":RECEIPT_VERSION,"registry_version":REGISTRY_VERSION,
      "project_id":p["project_id"],"unit":p["unit"],"claim_class_id":CLAIM_CLASS,
      "profile_sha256":psha,"parent_subject_sha":p["parent_subject_sha"],
      "parent_transition_receipt_sha256":p["parent_transition_receipt_sha256"],
      "parent_remaining_claim_units":p["parent_remaining_claim_units"],
      "operation_history_sha256":sha256_hex(normalized),
      "operation_chain_tip_sha256":sha256_hex(normalized[-1]),
      "operation_count":len(normalized),"active_claim_total_units":sum(x["face_units"] for x in slices),
      "active_slices":slices,"holder_totals":holder_totals,"conservation_passed":True,
      "commons_asset_transferable":False,"nonclaims":list(NONCLAIMS)}
    return QualifiedRegistry(receipt)

def load_case(path:Path):
    data=json.loads(path.read_text())
    if not isinstance(data,dict) or set(data)!={"profile","parent_receipt","operations"}: raise RegistryError("case: expected exact keys")
    return data["profile"],data["parent_receipt"],data["operations"]

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); p,parent,ops=load_case(a.case); receipt=qualify(p,parent,ops).receipt()
    text=json.dumps(receipt,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text)
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
