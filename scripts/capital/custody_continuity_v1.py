#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-custody-continuity-v1"
RECEIPT_VERSION="mycelix-custody-state-receipt-v1"
ACCEPTANCE_VERSION="mycelix-handover-acceptance-receipt-v1"
_ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX64=re.compile(r"^[0-9a-f]{64}$")
KINDS={"GenesisCurrent","MaterialRegression","VerifiedCure","SuspendCustody","ResumeCustody","SuccessorAcceptance","TerminateCustody"}
PROFILE_KEYS={"profile_version","project_id","acceptance_subject_sha","acceptance_receipt_sha256","initial_custodian_ref","regression_authority_ref","cure_authority_ref","custody_authority_ref","successor_authority_ref","allowed_blocker_codes","max_events"}
EVENT_KEYS={"seq","event_id","project_id","profile_sha256","prev_event_sha256","kind","blocker_code","successor_custodian_ref","successor_acceptance_receipt_sha256","authority_ref","evidence_ref"}
NONCLAIMS=(
"current custody state does not alter the historical handover acceptance receipt",
"current custody state is not legal title",
"current custody state is not constitutional stewardship legitimacy",
"successor acceptance reference is not independently authenticated by this theorem",
"custody currentness is not service-quality or engineering fitness outside this profile",
)

class CustodyError(ValueError): pass

@dataclass(frozen=True)
class QualifiedCustody:
    _receipt:dict[str,Any]
    def receipt(self)->dict[str,Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")
def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()
def _exact(o:dict[str,Any],keys:set[str],ctx:str)->None:
    if set(o)!=keys: raise CustodyError(f"{ctx}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")
def _id(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not _ID.fullmatch(v): raise CustodyError(f"{ctx}: invalid bounded identifier")
    return v
def _ref(v:Any,ctx:str)->str:
    if not isinstance(v,str) or not v or len(v.encode())>512 or v!=v.strip(): raise CustodyError(f"{ctx}: invalid reference")
    return v
def _hex64_or_none(v:Any,ctx:str):
    if v is None: return None
    if not isinstance(v,str) or not _HEX64.fullmatch(v): raise CustodyError(f"{ctx}: invalid sha256")
    return v

def validate_profile(p:dict[str,Any])->None:
    if not isinstance(p,dict): raise CustodyError("profile: expected object")
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise CustodyError("profile.profile_version: unsupported")
    _id(p["project_id"],"profile.project_id")
    if not isinstance(p["acceptance_subject_sha"],str) or not re.fullmatch(r"[0-9a-f]{40}",p["acceptance_subject_sha"]): raise CustodyError("profile.acceptance_subject_sha: invalid")
    if not isinstance(p["acceptance_receipt_sha256"],str) or not _HEX64.fullmatch(p["acceptance_receipt_sha256"]): raise CustodyError("profile.acceptance_receipt_sha256: invalid")
    for k in ("initial_custodian_ref","regression_authority_ref","cure_authority_ref","custody_authority_ref","successor_authority_ref"):
        _ref(p[k],f"profile.{k}")
    blockers=p["allowed_blocker_codes"]
    if not isinstance(blockers,list) or not blockers or len(set(blockers))!=len(blockers): raise CustodyError("profile.allowed_blocker_codes: invalid")
    for i,b in enumerate(blockers): _id(b,f"profile.allowed_blocker_codes[{i}]")
    me=p["max_events"]
    if isinstance(me,bool) or not isinstance(me,int) or me<1 or me>100000: raise CustodyError("profile.max_events: invalid")

def validate_acceptance(a:dict[str,Any],p:dict[str,Any])->None:
    if not isinstance(a,dict): raise CustodyError("acceptance_receipt: expected object")
    if sha256_hex(a)!=p["acceptance_receipt_sha256"]: raise CustodyError("acceptance_receipt: semantic digest mismatch")
    for k in ("receipt_version","project_id","incoming_custodian_ref","operational_custody_accepted","legal_title_transition_established","constitutional_stewardship_transition_established"):
        if k not in a: raise CustodyError(f"acceptance_receipt: missing {k}")
    if a["receipt_version"]!=ACCEPTANCE_VERSION: raise CustodyError("acceptance_receipt: unsupported version")
    if a["project_id"]!=p["project_id"]: raise CustodyError("acceptance_receipt: project mismatch")
    if a["incoming_custodian_ref"]!=p["initial_custodian_ref"]: raise CustodyError("acceptance_receipt: custodian mismatch")
    if a["operational_custody_accepted"] is not True: raise CustodyError("acceptance_receipt: custody not accepted")
    if a["legal_title_transition_established"] is not False or a["constitutional_stewardship_transition_established"] is not False:
        raise CustodyError("acceptance_receipt: authority contamination")

def qualify(p:dict[str,Any],acceptance:dict[str,Any],events:Any)->QualifiedCustody:
    validate_profile(p); validate_acceptance(acceptance,p)
    if not isinstance(events,list) or not events: raise CustodyError("events: expected non-empty array")
    if len(events)>p["max_events"]: raise CustodyError("events: exceeds max_events")
    psha=sha256_hex(p)
    active=set(); seen_ids=set(); normalized=[]
    suspended=False; terminal=None; successor_ref=None; successor_receipt=None
    for i,e in enumerate(events):
        if not isinstance(e,dict): raise CustodyError(f"events[{i}]: expected object")
        _exact(e,EVENT_KEYS,f"events[{i}]")
        if e["seq"]!=i: raise CustodyError(f"events[{i}].seq: expected {i}")
        eid=_id(e["event_id"],f"events[{i}].event_id")
        if eid in seen_ids: raise CustodyError(f"events[{i}].event_id: duplicate")
        seen_ids.add(eid)
        if e["project_id"]!=p["project_id"]: raise CustodyError(f"events[{i}]: project substitution")
        if e["profile_sha256"]!=psha: raise CustodyError(f"events[{i}]: profile substitution")
        if i==0:
            if e["prev_event_sha256"] is not None: raise CustodyError("events[0].prev_event_sha256: must be null")
        elif e["prev_event_sha256"]!=sha256_hex(normalized[-1]):
            raise CustodyError(f"events[{i}]: broken event chain")
        kind=e["kind"]
        if kind not in KINDS: raise CustodyError(f"events[{i}]: unsupported kind")
        _ref(e["authority_ref"],f"events[{i}].authority_ref"); _ref(e["evidence_ref"],f"events[{i}].evidence_ref")
        blocker=e["blocker_code"]
        succ=e["successor_custodian_ref"]
        succ_digest=_hex64_or_none(e["successor_acceptance_receipt_sha256"],f"events[{i}].successor_acceptance_receipt_sha256")
        if terminal is not None: raise CustodyError(f"events[{i}]: terminal lineage cannot continue")
        if kind=="GenesisCurrent":
            if i!=0 or blocker is not None or succ is not None or succ_digest is not None or e["authority_ref"]!=p["custody_authority_ref"]:
                raise CustodyError("GenesisCurrent: invalid shape/authority")
        elif kind=="MaterialRegression":
            if blocker not in p["allowed_blocker_codes"] or blocker in active: raise CustodyError("MaterialRegression: invalid/duplicate blocker")
            if succ is not None or succ_digest is not None or e["authority_ref"]!=p["regression_authority_ref"]: raise CustodyError("MaterialRegression: invalid shape/authority")
            active.add(blocker)
        elif kind=="VerifiedCure":
            if blocker not in active: raise CustodyError("VerifiedCure: blocker not active")
            if succ is not None or succ_digest is not None or e["authority_ref"]!=p["cure_authority_ref"]: raise CustodyError("VerifiedCure: invalid shape/authority")
            active.remove(blocker)
        elif kind=="SuspendCustody":
            if blocker is not None or succ is not None or succ_digest is not None or suspended or e["authority_ref"]!=p["custody_authority_ref"]: raise CustodyError("SuspendCustody: invalid")
            suspended=True
        elif kind=="ResumeCustody":
            if blocker is not None or succ is not None or succ_digest is not None or not suspended or active or e["authority_ref"]!=p["custody_authority_ref"]: raise CustodyError("ResumeCustody: invalid")
            suspended=False
        elif kind=="SuccessorAcceptance":
            if blocker is not None or suspended or active or e["authority_ref"]!=p["successor_authority_ref"]: raise CustodyError("SuccessorAcceptance: blockers/suspension/authority invalid")
            successor_ref=_ref(succ,"SuccessorAcceptance.successor_custodian_ref")
            if successor_ref==p["initial_custodian_ref"]: raise CustodyError("SuccessorAcceptance: successor must differ")
            if succ_digest is None: raise CustodyError("SuccessorAcceptance: receipt digest required")
            successor_receipt=succ_digest; terminal="SUPERSEDED"
        elif kind=="TerminateCustody":
            if blocker is not None or succ is not None or succ_digest is not None or e["authority_ref"]!=p["custody_authority_ref"]: raise CustodyError("TerminateCustody: invalid")
            terminal="TERMINATED"
        normalized.append(e)
    if events[0]["kind"]!="GenesisCurrent": raise CustodyError("events[0]: must be GenesisCurrent")
    if terminal: state=terminal
    elif suspended: state="SUSPENDED"
    elif active: state="DEGRADED"
    else: state="CURRENT"
    receipt={
      "receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,
      "project_id":p["project_id"],"profile_sha256":psha,
      "acceptance_subject_sha":p["acceptance_subject_sha"],
      "acceptance_receipt_sha256":p["acceptance_receipt_sha256"],
      "lineage_custodian_ref":p["initial_custodian_ref"],
      "current_custodian_ref": successor_ref if terminal=="SUPERSEDED" else (None if terminal=="TERMINATED" else p["initial_custodian_ref"]),
      "event_count":len(normalized),"event_history_sha256":sha256_hex(normalized),
      "event_chain_tip_sha256":sha256_hex(normalized[-1]),
      "active_blockers":sorted(active),"custody_state":state,
      "suspended":suspended,"successor_custodian_ref":successor_ref,
      "successor_acceptance_receipt_sha256":successor_receipt,
      "historical_handover_accepted":True,
      "legal_title_transition_established":False,
      "constitutional_stewardship_transition_established":False,
      "nonclaims":list(NONCLAIMS)}
    return QualifiedCustody(receipt)

def load_case(path:Path):
    d=json.loads(path.read_text())
    if not isinstance(d,dict) or set(d)!={"profile","acceptance_receipt","events"}: raise CustodyError("case: expected exact keys")
    return d["profile"],d["acceptance_receipt"],d["events"]
def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    x=ap.parse_args(); text=json.dumps(qualify(*load_case(x.case)).receipt(),sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if x.receipt_out: x.receipt_out.write_text(text)
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
