#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, subprocess
from pathlib import Path

MANIFEST_PATH=Path("docs/lex-net/lex_net_025_manifest.json")
SRC="LEX-NET/AUTHORITY/v3"
ENV="LEX-NET/AUTHORITY/LEASE-ENVELOPE/v1"
EVID="LEX-NET/EVIDENCE/v3"

def canon(x): return json.dumps(x,sort_keys=True,separators=(",",":"),ensure_ascii=False)
def commit(domain, fields): return hashlib.sha256((domain+"\0"+canon(fields)).encode()).hexdigest()
def body(x): return {k:v for k,v in x.items() if k!="commitment"}
def valid(x, domain): return isinstance(x,dict) and x.get("commitment")==commit(domain,body(x))

def source(**kw):
    f={"product_universe":"authority","kind":"LocalCapabilityLease","local_domain":"domain:a",
       "local_grant_commitment":"grant:1","subject":"org:alpha","purpose":"customs-submit",
       "resource":"shipment:123","action":"submit","mint_time":100,"current_until":160,
       "use_budget":2,"support_current_until":170,
       "satisfaction_evaluation_commitment":"eval:1","mint_authorization_commitment":"auth:1"}
    f.update(kw); f["commitment"]=commit(SRC,f); return f

def bind(lease,**kw):
    p={"presenter_binding_type":"key","presenter_binding_commitment":"presenter:a",
       "valid_from":110,"current_until":150,"requested_use_budget":1,
       "replay_domain":"replay:v1","revocation_handle_commitment":"rev:1",
       "policy_epoch_commitment":"policy:1","assurance_profile_commitment":"assurance:v1",
       "transferable":False,"redelegation_allowed":False,"delegation_depth":0}
    p.update(kw)
    if not (isinstance(lease,dict) and lease.get("product_universe")=="authority"
            and lease.get("kind")=="LocalCapabilityLease" and valid(lease,SRC)):
        return {"outcome":"SourceLeaseInvalid"}
    if p["presenter_binding_type"] not in {"key","session","principal"} or not p["presenter_binding_commitment"]:
        return {"outcome":"PresenterBindingRequired"}
    if p["transferable"]: return {"outcome":"TransferDenied"}
    if p["redelegation_allowed"] or p["delegation_depth"]!=0: return {"outcome":"RedelegationDenied"}
    if not p["replay_domain"]: return {"outcome":"ReplayDomainRequired"}
    if not p["revocation_handle_commitment"]: return {"outcome":"RevocationBindingRequired"}
    if not p["policy_epoch_commitment"]: return {"outcome":"PolicyEpochRequired"}
    if (p["valid_from"]<lease["mint_time"] or p["current_until"]<p["valid_from"]
        or p["current_until"]>lease["current_until"] or p["requested_use_budget"]<=0
        or p["requested_use_budget"]>lease["use_budget"]):
        return {"outcome":"EnvelopeAttenuationViolation"}
    f={"product_universe":"authority","kind":"LocalCapabilityLeaseEnvelope",
       "source_lease_commitment":lease["commitment"],"local_domain":lease["local_domain"],
       "local_grant_commitment":lease["local_grant_commitment"],
       "subject":lease["subject"],"purpose":lease["purpose"],"resource":lease["resource"],"action":lease["action"],
       "presenter_binding_type":p["presenter_binding_type"],"presenter_binding_commitment":p["presenter_binding_commitment"],
       "valid_from":p["valid_from"],"current_until":p["current_until"],"use_budget":p["requested_use_budget"],
       "replay_domain":p["replay_domain"],"revocation_handle_commitment":p["revocation_handle_commitment"],
       "policy_epoch_commitment":p["policy_epoch_commitment"],"assurance_profile_commitment":p["assurance_profile_commitment"],
       "transferable":False,"redelegation_allowed":False,"delegation_depth":0}
    f["commitment"]=commit(ENV,f); return {"outcome":"IssuedCurrent","product":f}

def evaluate(x,**kw):
    c={"now":120,"local_domain":"domain:a","presenter_binding_commitment":"presenter:a",
       "revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1",
       "assurance_profile_commitment":"assurance:v1"}
    c.update(kw)
    if not (isinstance(x,dict) and x.get("product_universe")=="authority"
            and x.get("kind")=="LocalCapabilityLeaseEnvelope" and valid(x,ENV)):
        return "EnvelopeInvalid"
    if x["local_domain"]!=c["local_domain"]: return "DomainMismatch"
    if not x.get("presenter_binding_commitment"): return "PresenterBindingRequired"
    if x["presenter_binding_commitment"]!=c["presenter_binding_commitment"]: return "PresenterBindingFailed"
    if c["now"]<x["valid_from"]: return "NotYetValid"
    if c["now"]>x["current_until"]: return "Expired"
    if x["revocation_handle_commitment"]!=c["revocation_handle_commitment"]: return "RevocationStateMismatch"
    if x["policy_epoch_commitment"]!=c["policy_epoch_commitment"]: return "PolicyEpochMismatch"
    if x.get("assurance_profile_commitment") is not None and x["assurance_profile_commitment"]!=c["assurance_profile_commitment"]:
        return "AssuranceProfileMismatch"
    if x.get("transferable") is not False: return "TransferDenied"
    if x.get("redelegation_allowed") is not False or x.get("delegation_depth")!=0: return "RedelegationDenied"
    return "IssuedCurrent"

def export(x,destination="domain:b"):
    f={"product_universe":"evidence","kind":"EvidenceAboutForeignAuthority",
       "source_authority_commitment":x["commitment"],"source_domain":x["local_domain"],
       "destination_domain":destination,"subject":x["subject"],"purpose":x["purpose"],
       "resource":x["resource"],"action":x["action"],"current_until":x["current_until"],
       "grants_local_authority":False,"grants_external_effect_authority":False}
    f["commitment"]=commit(EVID,f); return f

def env(**kw): return bind(source(),**kw)["product"]

def run(fid):
    if fid=="exact_current": return evaluate(env())
    if fid=="forged_source_commitment":
        x=source(); x["use_budget"]=99; return bind(x)["outcome"]
    if fid=="wrong_source_kind": return bind(source(kind="LocalAuthorization"))["outcome"]
    if fid=="source_scope_tamper":
        x=source(); x["resource"]="shipment:999"; return bind(x)["outcome"]
    if fid=="domain_mismatch": return evaluate(env(),local_domain="domain:b")
    if fid=="missing_presenter": return bind(source(),presenter_binding_commitment="")["outcome"]
    if fid=="presenter_mismatch": return evaluate(env(),presenter_binding_commitment="presenter:b")
    if fid=="valid_from_before_mint": return bind(source(),valid_from=99)["outcome"]
    if fid=="expiry_after_source": return bind(source(),current_until=161)["outcome"]
    if fid=="expiry_before_valid_from": return bind(source(),valid_from=130,current_until=129)["outcome"]
    if fid=="use_budget_exceeds_source": return bind(source(),requested_use_budget=3)["outcome"]
    if fid=="zero_use_budget": return bind(source(),requested_use_budget=0)["outcome"]
    if fid=="transfer_true": return bind(source(),transferable=True)["outcome"]
    if fid=="redelegation_true": return bind(source(),redelegation_allowed=True)["outcome"]
    if fid=="delegation_depth_nonzero": return bind(source(),delegation_depth=1)["outcome"]
    if fid=="missing_replay_domain": return bind(source(),replay_domain="")["outcome"]
    if fid=="missing_revocation_handle": return bind(source(),revocation_handle_commitment="")["outcome"]
    if fid=="missing_policy_epoch": return bind(source(),policy_epoch_commitment="")["outcome"]
    if fid=="not_yet_valid": return evaluate(env(),now=109)
    if fid=="expired": return evaluate(env(),now=151)
    if fid=="revocation_mismatch": return evaluate(env(),revocation_handle_commitment="rev:2")
    if fid=="policy_epoch_mismatch": return evaluate(env(),policy_epoch_commitment="policy:2")
    if fid=="assurance_mismatch": return evaluate(env(),assurance_profile_commitment="assurance:high")
    if fid=="export_degrades_to_evidence":
        x=env(); y=export(x); return "ExportedAsEvidence" if y["product_universe"]=="evidence" and not y["grants_local_authority"] and y["commitment"]!=x["commitment"] else "EnvelopeInvalid"
    if fid=="evidence_relabel_cannot_be_envelope":
        y=export(env()); y["kind"]="LocalCapabilityLeaseEnvelope"; y["commitment"]=commit(EVID,body(y)); return evaluate(y)
    if fid=="envelope_commitment_tamper":
        x=env(); x["current_until"]=159; return evaluate(x)
    if fid=="same_fields_domain_separated":
        f={"x":1}; return "DomainSeparated" if commit(ENV,f)!=commit(EVID,f)!=commit(SRC,f) else "EnvelopeInvalid"
    if fid=="cross_domain_export_not_authority":
        y=export(env(),"domain:b"); return "ExportedAsEvidence" if y["destination_domain"]=="domain:b" and y["product_universe"]=="evidence" else "EnvelopeInvalid"
    raise KeyError(fid)

def load(): return json.loads(MANIFEST_PATH.read_text())
def self_test():
    m=load()
    assert m["source_authority_domain"]==SRC and m["envelope_authority_domain"]==ENV and m["evidence_domain"]==EVID
    assert commit(SRC,{"x":1})!=commit(ENV,{"x":1})!=commit(EVID,{"x":1})
    print("LEX-NET-025 R1 self-test PASS")
def semantic():
    m=load(); seen=set()
    for f in m["fixtures"]:
        got=run(f["id"])
        if got!=f["expected"]: raise SystemExit(f"{f['id']}: expected {f['expected']} got {got}")
        seen.add(got)
    print(json.dumps({"tranche":"LEX-NET-025","candidate_revision":"R1","fixture_count":len(m["fixtures"]),
        "observed_outcomes":sorted(seen),"atomic_consumption_claim":False,"portable_authority_claim":False,"semantic_result":"PASS"},sort_keys=True))
def scope():
    m=load(); h=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip()
    p=subprocess.check_output(["git","rev-parse","HEAD^"],text=True).strip()
    n=int(subprocess.check_output(["git","rev-list","--count",f"{p}..{h}"],text=True).strip())
    paths=sorted(subprocess.check_output(["git","diff","--name-only",p,h],text=True).splitlines())
    if p!=m["qualified_parent"]: raise SystemExit("parent mismatch")
    if n!=1: raise SystemExit("commit count mismatch")
    if paths!=sorted(m["expected_paths"]): raise SystemExit(f"path set mismatch {paths}")
    print(json.dumps({"tranche":"LEX-NET-025","candidate_revision":"R1","head":h,"parent":p,"commit_count":n,"paths":paths,"scope_result":"PASS"},sort_keys=True))
def main():
    a=argparse.ArgumentParser(); a.add_argument("--self-test",action="store_true"); a.add_argument("--semantic",action="store_true"); a.add_argument("--scope",action="store_true"); x=a.parse_args()
    if x.self_test:self_test()
    elif x.semantic:semantic()
    elif x.scope:scope()
    else:a.error("select mode")
if __name__=="__main__":main()
