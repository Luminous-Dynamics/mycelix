#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION="mycelix-composition-dimension-projection-v1"
BUNDLE_VERSION="mycelix-composition-dimension-projection-bundle-v1"
PROJECTION_VERSION="mycelix-composition-dimension-projection-record-v1"

H1_SUBJECT="cb6f2e88d81a7ff0fb0adcf36888f44c4fe8ed69"
H2_SUBJECT="7ba66c70be2df27b1d5deaa77a88ffdda96baa35"
H3_SUBJECT="f9d88b41a750d2bddfa8fa2bf26727f04a4998a6"
H1_RECEIPT_SHA="ef3e4b8772d4476fe24568cd8c503db7dec192418cd4719a052d6b941d523544"
H2_RECEIPT_SHA="bd78cf22a9899d0a816de80600d8f375e1d089f07b9ba64f3146b0bd32318e83"
H3_RECEIPT_SHA="be1c02a199688e8aef4c6054abfef216cfffe830730f090fdb1be1db406e3045"

PROFILE_KEYS={"profile_version","project_id","sources"}
SOURCE_BINDING_KEYS={"subject_sha","receipt_sha256"}
H1_KEYS={"action_code","active_epoch","active_profile_sha256","blockers","checkpoint_sha256","composition_profile_id","decision_id","democratic_legitimacy_established","execution_authority_established","g2a_receipt_sha256","g2c_receipt_sha256","governance_state","legal_validity_established","nonclaims","profile_version","project_id","receipt_version","registry_id"}
H2_KEYS={"blockers","claim_modified","execution_authority_established","f1_receipt_sha256","f_receipt_sha256","legal_distribution_authority_established","measurement_id","nonclaims","payment_authority_established","project_id","public_service_distribution_state","qualified_f1_subject_sha","qualified_f_subject_sha","receipt_version","service_currentness_profile_sha256","service_gate_profile_sha256"}
H3_KEYS={"blockers","currentness_established","digital_readiness_state","digital_receipt_sha256","digital_subject_sha","execution_authority_established","handback_readiness_state","handover_accepted","legal_transition_complete","nonclaims","operational_custody_accepted","parent_financial_state","parent_remaining_claim_units","parent_subject_sha","parent_transition_receipt_sha256","physical_readiness_state","physical_receipt_sha256","physical_subject_sha","profile_sha256","profile_version","project_id","receipt_version"}

H2_STATES={"ELIGIBLE_CURRENT","BLOCKED_SERVICE","STALE","PENDING","REVOKED","NO_ACTIVE_CLAIM"}
H3_STATES={"HANDOVER_READY","REMEDIATION_REQUIRED","ASSESSMENT_INCOMPLETE","RESERVE_DEFICIENT"}

ADAPTER_NONCLAIMS=[
    "projection validity is not source qualification",
    "normalization does not add currentness or authority",
    "source subject and receipt identity remain authoritative provenance",
    "projection does not establish execution, payment-transfer, legal-title, or constitutional authority",
]

class ProjectionError(ValueError): pass


def canonical_bytes(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode("utf-8")

def sha256_hex(v:Any)->str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()

def _exact(o:Any,keys:set[str],ctx:str):
    if not isinstance(o,dict): raise ProjectionError(f"{ctx}: expected object")
    if set(o)!=keys: raise ProjectionError(f"{ctx}: key mismatch missing={sorted(keys-set(o))} unknown={sorted(set(o)-keys)}")

def _false(v:Any,ctx:str):
    if v is not False: raise ProjectionError(f"{ctx}: authority contamination")

def _list(v:Any,ctx:str):
    if not isinstance(v,list): raise ProjectionError(f"{ctx}: expected list")
    return list(v)


def validate_profile(p:dict[str,Any]):
    _exact(p,PROFILE_KEYS,"profile")
    if p["profile_version"]!=PROFILE_VERSION: raise ProjectionError("profile: unsupported version")
    if not isinstance(p["project_id"],str) or not p["project_id"]: raise ProjectionError("profile: invalid project")
    if set(p["sources"])!={"STEWARDSHIP_GOVERNANCE","PUBLIC_SERVICE_DISTRIBUTION","HANDBACK_READINESS"}:
        raise ProjectionError("profile: exact source set required")
    expected={
        "STEWARDSHIP_GOVERNANCE":(H1_SUBJECT,H1_RECEIPT_SHA),
        "PUBLIC_SERVICE_DISTRIBUTION":(H2_SUBJECT,H2_RECEIPT_SHA),
        "HANDBACK_READINESS":(H3_SUBJECT,H3_RECEIPT_SHA),
    }
    for k,(sub,dig) in expected.items():
        b=p["sources"][k]; _exact(b,SOURCE_BINDING_KEYS,f"profile.sources.{k}")
        if b["subject_sha"]!=sub or b["receipt_sha256"]!=dig:
            raise ProjectionError(f"profile.sources.{k}: frozen source binding mismatch")


def _base_projection(dimension_id:str,project_id:str,subject:str,receipt:dict[str,Any],semantic:str,currentness:str,owner:str,ceiling:str,blockers:list[str]):
    return {
        "projection_version":PROJECTION_VERSION,
        "dimension_id":dimension_id,
        "project_id":project_id,
        "source_subject_sha":subject,
        "source_receipt_sha256":sha256_hex(receipt),
        "source_receipt_version":receipt["receipt_version"],
        "semantic_state":semantic,
        "currentness_state":currentness,
        "currentness_owner":owner,
        "blockers":sorted(set(blockers)),
        "authority_ceiling":ceiling,
        "execution_authority_established":False,
        "payment_authority_established":False,
        "legal_title_authority_established":False,
        "constitutional_authority_established":False,
        "source_nonclaims":list(receipt["nonclaims"]),
        "adapter_nonclaims":list(ADAPTER_NONCLAIMS),
    }


def project_h1(r:dict[str,Any],p:dict[str,Any]):
    _exact(r,H1_KEYS,"h1")
    if r["receipt_version"]!="mycelix-stewardship-governance-dimension-receipt-v1": raise ProjectionError("h1: receipt version")
    if r["project_id"]!=p["project_id"]: raise ProjectionError("h1: project mismatch")
    if sha256_hex(r)!=H1_RECEIPT_SHA: raise ProjectionError("h1: exact canonical receipt required")
    _false(r["execution_authority_established"],"h1.execution_authority_established")
    _false(r["legal_validity_established"],"h1.legal_validity_established")
    _false(r["democratic_legitimacy_established"],"h1.democratic_legitimacy_established")
    _list(r["blockers"],"h1.blockers"); _list(r["nonclaims"],"h1.nonclaims")
    if r["governance_state"]!="CURRENT": raise ProjectionError("h1: v1 adapter supports exact qualified CURRENT receipt only")
    return _base_projection("STEWARDSHIP_GOVERNANCE",r["project_id"],H1_SUBJECT,r,"CURRENT","CURRENT","INTEGRATED_SOURCE_THEOREM","GOVERNANCE_STATE_ONLY",r["blockers"])


def project_h2(r:dict[str,Any],p:dict[str,Any]):
    _exact(r,H2_KEYS,"h2")
    if r["receipt_version"]!="mycelix-public-service-distribution-composition-receipt-v1": raise ProjectionError("h2: receipt version")
    if r["project_id"]!=p["project_id"]: raise ProjectionError("h2: project mismatch")
    _false(r["execution_authority_established"],"h2.execution_authority_established")
    _false(r["payment_authority_established"],"h2.payment_authority_established")
    _false(r["legal_distribution_authority_established"],"h2.legal_distribution_authority_established")
    _false(r["claim_modified"],"h2.claim_modified")
    _list(r["blockers"],"h2.blockers"); _list(r["nonclaims"],"h2.nonclaims")
    s=r["public_service_distribution_state"]
    if s not in H2_STATES: raise ProjectionError("h2: unsupported state")
    mapping={
        "ELIGIBLE_CURRENT":("ELIGIBLE","CURRENT"),
        "BLOCKED_SERVICE":("BLOCKED_SERVICE","CURRENT"),
        "NO_ACTIVE_CLAIM":("NO_ACTIVE_CLAIM","CURRENT"),
        "STALE":("NOT_SEPARATELY_EXPOSED","STALE"),
        "PENDING":("NOT_SEPARATELY_EXPOSED","PENDING"),
        "REVOKED":("NOT_SEPARATELY_EXPOSED","REVOKED"),
    }
    semantic,current=mapping[s]
    return _base_projection("PUBLIC_SERVICE_DISTRIBUTION",r["project_id"],H2_SUBJECT,r,semantic,current,"INTEGRATED_SOURCE_THEOREM","SERVICE_DISTRIBUTION_STATE_ONLY",r["blockers"])


def project_h3(r:dict[str,Any],p:dict[str,Any]):
    _exact(r,H3_KEYS,"h3")
    if r["receipt_version"]!="mycelix-handback-readiness-composition-receipt-v1": raise ProjectionError("h3: receipt version")
    if r["project_id"]!=p["project_id"]: raise ProjectionError("h3: project mismatch")
    _false(r["currentness_established"],"h3.currentness_established")
    _false(r["execution_authority_established"],"h3.execution_authority_established")
    _false(r["handover_accepted"],"h3.handover_accepted")
    _false(r["operational_custody_accepted"],"h3.operational_custody_accepted")
    _false(r["legal_transition_complete"],"h3.legal_transition_complete")
    _list(r["blockers"],"h3.blockers"); _list(r["nonclaims"],"h3.nonclaims")
    if r["handback_readiness_state"] not in H3_STATES: raise ProjectionError("h3: unsupported readiness state")
    return _base_projection("HANDBACK_READINESS",r["project_id"],H3_SUBJECT,r,r["handback_readiness_state"],"NOT_ESTABLISHED","H3A_REQUIRED","READINESS_ONLY",r["blockers"])


@dataclass(frozen=True)
class Bundle:
    value:dict[str,Any]
    def receipt(self): return json.loads(json.dumps(self.value))


def qualify(case:dict[str,Any])->Bundle:
    _exact(case,{"profile","h1_receipt","h2_receipt","h3_receipt"},"case")
    p=case["profile"]; validate_profile(p)
    dims=[project_h1(case["h1_receipt"],p),project_h2(case["h2_receipt"],p),project_h3(case["h3_receipt"],p)]
    if len({d["project_id"] for d in dims})!=1: raise ProjectionError("bundle: cross-project composition")
    dims=sorted(dims,key=lambda d:d["dimension_id"])
    source_identity=[{"dimension_id":d["dimension_id"],"source_subject_sha":d["source_subject_sha"],"source_receipt_sha256":d["source_receipt_sha256"]} for d in dims]
    return Bundle({
        "receipt_version":BUNDLE_VERSION,
        "profile_version":PROFILE_VERSION,
        "project_id":p["project_id"],
        "profile_sha256":sha256_hex(p),
        "dimensions":dims,
        "source_identity_sha256":sha256_hex(source_identity),
        "execution_authority_established":False,
        "payment_authority_established":False,
        "legal_title_authority_established":False,
        "constitutional_authority_established":False,
        "nonclaims":[
            "dimension projection does not qualify any new source subject",
            "dimension projection does not establish project transition readiness",
            "H3 projection does not establish handback-readiness currentness",
            "bundle projection does not establish custody acceptance or current custody",
            "bundle projection does not establish payment, execution, legal-title, or constitutional authority",
        ],
    })


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path)
    a=ap.parse_args(); case=json.loads(a.case.read_text(encoding="utf-8")); out=qualify(case).receipt()
    text=json.dumps(out,sort_keys=True,indent=2,ensure_ascii=False)+"\n"
    if a.receipt_out: a.receipt_out.write_text(text,encoding="utf-8")
    else: print(text,end="")
    return 0
if __name__=="__main__": raise SystemExit(main())
