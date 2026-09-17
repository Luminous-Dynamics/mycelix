#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, subprocess
from pathlib import Path

MANIFEST_PATH=Path("docs/lex-net/lex_net_028_manifest.json")
EVIDENCE_DOMAIN="LEX-NET/EVIDENCE/v2"
AUTHORITY_DOMAIN="LEX-NET/AUTHORITY/v2"
REQUIREMENT_DOMAIN="LEX-NET/REQUIREMENT/v1"

def canon(x): return json.dumps(x,sort_keys=True,separators=(",",":"),ensure_ascii=False)
def commit(domain,fields): return hashlib.sha256((domain+"\0"+canon(fields)).encode()).hexdigest()
def signed_fields(p): return {k:v for k,v in p.items() if k!="commitment"}
def verify_commitment(p,domain): return isinstance(p,dict) and p.get("commitment")==commit(domain,signed_fields(p))

def evidence(kind,evidence_id,*,subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",profile_commitment="profile:v1",disposition="positive",current_until=200,independence_group="issuer:alpha",predecessors=None,extra=None):
    f={"product_universe":"evidence","kind":kind,"evidence_id":evidence_id,"subject":subject,"purpose":purpose,"resource":resource,"action":action,"profile_commitment":profile_commitment,"disposition":disposition,"current_until":current_until,"independence_group":independence_group,"predecessors":list(predecessors or []),"grants_local_authority":False,"grants_external_effect_authority":False}
    if extra: f.update(extra)
    f["commitment"]=commit(EVIDENCE_DOMAIN,f)
    return f

def atom(kind="RecognitionEvidence",*,subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",profile_commitment="profile:v1",accepted_disposition="positive",requires_current=True,independent_count=1):
    return {"kind":kind,"subject":subject,"purpose":purpose,"resource":resource,"action":action,"profile_commitment":profile_commitment,"accepted_disposition":accepted_disposition,"requires_current":requires_current,"independent_count":independent_count}

def requirement(atoms=None,*,alternatives=None,alternative_policy_id=None):
    r={"atoms":list(atoms or []),"alternatives":alternatives or [],"alternative_policy_id":alternative_policy_id}
    r["commitment"]=commit(REQUIREMENT_DOMAIN,{k:v for k,v in r.items() if k!="commitment"})
    return r

def _prepare(products):
    if any(isinstance(p,dict) and p.get("product_universe")=="authority" for p in products): return "AuthorityProductRejected",[],{}
    seen_id={}; dedup={}
    for p in products:
        if not isinstance(p,dict) or p.get("product_universe")!="evidence" or not verify_commitment(p,EVIDENCE_DOMAIN): continue
        eid=p["evidence_id"]; old=seen_id.get(eid)
        if old is not None and old!=p["commitment"]: return "EvidenceIdentityConflict",[],{}
        seen_id[eid]=p["commitment"]; dedup[p["commitment"]]=p
    vals=list(dedup.values())
    return None,vals,{p["evidence_id"]:p for p in vals}

def _current(e,by_id,t,seen=None):
    seen=set(seen or ()); eid=e["evidence_id"]
    if eid in seen: return "MissingDependency"
    seen.add(eid)
    if e.get("current_until") is not None and t>e["current_until"]: return "StaleDependency"
    for pid in e.get("predecessors",[]):
        p=by_id.get(pid)
        if p is None: return "MissingDependency"
        r=_current(p,by_id,t,seen)
        if r!="Satisfied": return r
    return "Satisfied"

def _satisfy_atoms(atoms,products,t):
    if not atoms: return "EmptyRequirementRejected"
    prep,products,by_id=_prepare(products)
    if prep: return prep
    for req in atoms:
        kinds=[p for p in products if p["kind"]==req["kind"]]
        if not kinds: return "MissingRequiredEvidence"
        scoped=[p for p in kinds if all(p[k]==req[k] for k in ("subject","purpose","resource","action"))]
        if not scoped: return "ScopeMismatch"
        prof=[p for p in scoped if p["profile_commitment"]==req["profile_commitment"]]
        if not prof: return "ProfileMismatch"
        if len({p["disposition"] for p in prof})>1: return "ConflictIndeterminate"
        accepted=[p for p in prof if p["disposition"]==req["accepted_disposition"]]
        if not accepted: return "DispositionMismatch"
        usable=[]; current_errors=[]
        for p in accepted:
            if req.get("requires_current",True):
                c=_current(p,by_id,t)
                if c!="Satisfied": current_errors.append(c); continue
            usable.append(p)
        if not usable:
            if "MissingDependency" in current_errors: return "MissingDependency"
            if "StaleDependency" in current_errors: return "StaleDependency"
            return "MissingRequiredEvidence"
        if len({p["independence_group"] for p in usable})<int(req.get("independent_count",1)): return "IndependenceInsufficient"
    return "Satisfied"

def satisfies(req,products,t):
    if not isinstance(req,dict) or not verify_commitment(req,REQUIREMENT_DOMAIN): return "EmptyRequirementRejected"
    if req.get("alternatives"):
        if not req.get("alternative_policy_id"): return "AlternativePolicyRequired"
        if any(not opt for opt in req["alternatives"]): return "EmptyRequirementRejected"
        results=[_satisfy_atoms(opt,products,t) for opt in req["alternatives"]]
        return "Satisfied" if "Satisfied" in results else results[0]
    return _satisfy_atoms(req.get("atoms",[]),products,t)

def evaluate(req,products,t,evaluation_id="eval:1"):
    outcome=satisfies(req,products,t); prepared,vals,_=_prepare(products)
    evidence_commitments=sorted(p["commitment"] for p in vals) if prepared is None else []
    source_atoms=req.get("atoms") or (req.get("alternatives") or [[]])[0]
    scope={k:source_atoms[0][k] for k in ("subject","purpose","resource","action")} if source_atoms else {}
    return evidence("SatisfactionEvaluation",evaluation_id,subject=scope.get("subject",""),purpose=scope.get("purpose",""),resource=scope.get("resource",""),action=scope.get("action",""),profile_commitment=req.get("commitment",""),disposition=outcome,current_until=t,independence_group="local-evaluator",extra={"requirement":req,"requirement_commitment":req.get("commitment"),"evidence_commitments":evidence_commitments,"evaluation_time":t,"evaluation_outcome":outcome})

def local_authorization(auth_id,*,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,authorized_operation="mint-capability"):
    f={"product_universe":"authority","kind":"LocalAuthorization","authorization_id":auth_id,"local_domain":local_domain,"local_grant_commitment":local_grant_commitment,"authorized_operation":authorized_operation,"subject":subject,"purpose":purpose,"resource":resource,"action":action,"current_until":current_until}
    f["commitment"]=commit(AUTHORITY_DOMAIN,f); return f

def _eval_ok(e):
    return isinstance(e,dict) and e.get("product_universe")=="evidence" and e.get("kind")=="SatisfactionEvaluation" and verify_commitment(e,EVIDENCE_DOMAIN) and isinstance(e.get("requirement"),dict) and e.get("requirement_commitment")==e["requirement"].get("commitment") and verify_commitment(e["requirement"],REQUIREMENT_DOMAIN)

def _auth_ok(a): return isinstance(a,dict) and a.get("product_universe")=="authority" and a.get("kind")=="LocalAuthorization" and verify_commitment(a,AUTHORITY_DOMAIN)

def mint_local_capability(evaluation,mint_authorization,*,current_until=200,use_budget=1):
    if not _eval_ok(evaluation): return {"outcome":"EvaluationBindingMismatch"}
    if evaluation.get("evaluation_outcome")!="Satisfied": return {"outcome":"MintPreconditionUnsatisfied"}
    if not _auth_ok(mint_authorization): return {"outcome":"MintAuthorizationMismatch"}
    if mint_authorization.get("authorized_operation")!="mint-capability": return {"outcome":"MintAuthorizationMismatch"}
    if mint_authorization.get("current_until") is not None and evaluation["evaluation_time"]>mint_authorization["current_until"]: return {"outcome":"MintAuthorizationMismatch"}
    scope={k:evaluation[k] for k in ("subject","purpose","resource","action")}
    if any(mint_authorization.get(k)!=v for k,v in scope.items()): return {"outcome":"MintAuthorizationMismatch"}
    if not mint_authorization.get("local_domain") or not mint_authorization.get("local_grant_commitment") or use_budget<=0: return {"outcome":"MintAuthorizationMismatch"}
    f={"product_universe":"authority","kind":"LocalCapabilityLease","local_domain":mint_authorization["local_domain"],"local_grant_commitment":mint_authorization["local_grant_commitment"],**scope,"current_until":current_until,"use_budget":use_budget,"satisfaction_evaluation_commitment":evaluation["commitment"],"mint_authorization_commitment":mint_authorization["commitment"]}
    f["commitment"]=commit(AUTHORITY_DOMAIN,f); return {"outcome":"Satisfied","product":f}

def export_authority_as_evidence(a,eid):
    return evidence("EvidenceAboutForeignAuthority",eid,subject=a["subject"],purpose=a["purpose"],resource=a["resource"],action=a["action"],profile_commitment="foreign-authority-evidence:v2",disposition="observed",current_until=a["current_until"],independence_group="source-domain:"+a["local_domain"],extra={"source_authority_commitment":a["commitment"]})

def consume_authority(a,prior_execution_evidence,*,execution_id):
    if not isinstance(a,dict) or a.get("kind")!="LocalCapabilityLease" or not verify_commitment(a,AUTHORITY_DOMAIN): return {"outcome":"AuthorityConsumed"}
    unique={p["commitment"]:p for p in prior_execution_evidence if isinstance(p,dict) and p.get("kind")=="ExecutionEvidence" and p.get("authority_commitment")==a["commitment"] and verify_commitment(p,EVIDENCE_DOMAIN)}
    if len(unique)>=a.get("use_budget",0): return {"outcome":"AuthorityConsumed"}
    x=evidence("ExecutionEvidence",execution_id,subject=a["subject"],purpose=a["purpose"],resource=a["resource"],action=a["action"],profile_commitment="execution:v2",disposition="attempted",current_until=a["current_until"],independence_group="executor:"+a["local_domain"],extra={"authority_commitment":a["commitment"],"consumption_index":len(unique)+1})
    return {"outcome":"Satisfied","evidence":x,"authority":a}

def base_products():
    i=evidence("InterpretationEvidence","i1",profile_commitment="interp:v1",independence_group="parser:a")
    r=evidence("RecognitionEvidence","r1",profile_commitment="profile:v1",independence_group="recognizer:a",predecessors=["i1"])
    return [i,r]

def mint_ready():
    req=requirement([atom()]); products=base_products(); ev=evaluate(req,products,100); auth=local_authorization("auth:1"); return req,products,ev,auth

def run_fixture(fid):
    req=requirement([atom()]); products=base_products(); t=100
    if fid=="exact_match": return satisfies(req,products,t)
    if fid=="wrong_kind": return satisfies(requirement([atom(kind="TranslationEvidence")]),products,t)
    if fid=="scope_mismatch": return satisfies(requirement([atom(subject="org:beta")]),products,t)
    if fid=="profile_mismatch": return satisfies(requirement([atom(profile_commitment="profile:v2")]),products,t)
    if fid=="disposition_mismatch": return satisfies(requirement([atom(accepted_disposition="negative")]),products,t)
    if fid=="stale_direct": return satisfies(req,[evidence("RecognitionEvidence","r2",current_until=50)],t)
    if fid=="missing_predecessor": return satisfies(req,[evidence("RecognitionEvidence","r2",predecessors=["missing"])],t)
    if fid=="stale_predecessor":
        dep=evidence("InterpretationEvidence","i2",current_until=50,profile_commitment="interp:v1"); p=evidence("RecognitionEvidence","r2",predecessors=["i2"]); return satisfies(req,[dep,p],t)
    if fid=="fresh_candidate_survives_stale_extra":
        good=evidence("RecognitionEvidence","good",current_until=200,independence_group="g"); old=evidence("RecognitionEvidence","old",current_until=50,independence_group="g"); return satisfies(req,[good,old],t)
    if fid=="duplicate_not_independent":
        p=evidence("RecognitionEvidence","r2",independence_group="same"); return satisfies(requirement([atom(independent_count=2)]),[p,dict(p)],t)
    if fid=="correlated_not_independent":
        a=evidence("RecognitionEvidence","ra",independence_group="same"); b=evidence("RecognitionEvidence","rb",independence_group="same"); return satisfies(requirement([atom(independent_count=2)]),[a,b],t)
    if fid=="independent_two":
        a=evidence("RecognitionEvidence","ra",independence_group="a"); b=evidence("RecognitionEvidence","rb",independence_group="b"); return satisfies(requirement([atom(independent_count=2)]),[a,b],t)
    if fid=="conflict": return satisfies(req,[evidence("RecognitionEvidence","ra",disposition="positive"),evidence("RecognitionEvidence","rb",disposition="negative")],t)
    if fid=="majority_does_not_override_conflict": return satisfies(req,[evidence("RecognitionEvidence","ra",disposition="positive",independence_group="a"),evidence("RecognitionEvidence","rb",disposition="positive",independence_group="b"),evidence("RecognitionEvidence","rc",disposition="negative",independence_group="c")],t)
    if fid=="alternatives_require_policy": return satisfies(requirement(alternatives=[[atom()],[atom(kind="ExternalProofEvidence")]]),products,t)
    if fid=="explicit_alternative": return satisfies(requirement(alternatives=[[atom(kind="ExternalProofEvidence")],[atom()]],alternative_policy_id="local:v1"),products,t)
    if fid=="empty_requirement": return satisfies(requirement([]),products,t)
    if fid=="authority_rejected_as_evidence":
        _,_,ev,auth=mint_ready(); return satisfies(req,[mint_local_capability(ev,auth)["product"]],t)
    if fid=="evidence_id_conflict": return satisfies(req,[evidence("RecognitionEvidence","same"),evidence("RecognitionEvidence","same",resource="shipment:999")],t)
    if fid=="evaluation_is_bound":
        ev=evaluate(req,products,t); return "Satisfied" if ev["kind"]=="SatisfactionEvaluation" and ev["evaluation_outcome"]=="Satisfied" and verify_commitment(ev,EVIDENCE_DOMAIN) else "EvaluationBindingMismatch"
    if fid=="bare_string_cannot_mint": _,_,_,auth=mint_ready(); return mint_local_capability("Satisfied",auth)["outcome"]
    if fid=="forged_evaluation_commitment":
        _,_,ev,auth=mint_ready(); ev=dict(ev); ev["evaluation_time"]=101; return mint_local_capability(ev,auth)["outcome"]
    if fid=="unsatisfied_evaluation_cannot_mint":
        bad=requirement([atom(subject="org:beta")]); ev=evaluate(bad,base_products(),t); return mint_local_capability(ev,local_authorization("a",subject="org:beta"))["outcome"]
    if fid=="evidence_cannot_be_mint_authorization": _,_,ev,_=mint_ready(); return mint_local_capability(ev,evidence("RecognitionEvidence","fake"))["outcome"]
    if fid=="mint_authorization_wrong_scope": _,_,ev,_=mint_ready(); return mint_local_capability(ev,local_authorization("a",resource="shipment:999"))["outcome"]
    if fid=="expired_mint_authorization": _,_,ev,_=mint_ready(); return mint_local_capability(ev,local_authorization("a",current_until=50))["outcome"]
    if fid=="authorized_mint_binds_both":
        _,_,ev,auth=mint_ready(); m=mint_local_capability(ev,auth); p=m.get("product",{}); return "Satisfied" if m["outcome"]=="Satisfied" and p.get("satisfaction_evaluation_commitment")==ev["commitment"] and p.get("mint_authorization_commitment")==auth["commitment"] else "EvaluationBindingMismatch"
    if fid=="export_authority_degrades_to_evidence":
        _,_,ev,auth=mint_ready(); p=mint_local_capability(ev,auth)["product"]; x=export_authority_as_evidence(p,"fa"); return "Satisfied" if x["product_universe"]=="evidence" and x["commitment"]!=p["commitment"] and not x["grants_local_authority"] else "AuthorityProductRejected"
    if fid=="consume_once_append_only":
        _,_,ev,auth=mint_ready(); p=mint_local_capability(ev,auth)["product"]; before=p["commitment"]; c=consume_authority(p,[],execution_id="x1"); return "Satisfied" if c["outcome"]=="Satisfied" and c["authority"]["commitment"]==before and c["evidence"]["authority_commitment"]==before else "AuthorityConsumed"
    if fid=="consume_twice_rejected":
        _,_,ev,auth=mint_ready(); p=mint_local_capability(ev,auth)["product"]; c1=consume_authority(p,[],execution_id="x1"); return consume_authority(p,[c1["evidence"]],execution_id="x2")["outcome"]
    if fid=="duplicate_execution_does_not_double_count":
        _,_,ev,auth=mint_ready(); p=mint_local_capability(ev,auth,use_budget=2)["product"]; c1=consume_authority(p,[],execution_id="x1"); return consume_authority(p,[c1["evidence"],dict(c1["evidence"])],execution_id="x2")["outcome"]
    if fid=="execution_evidence_not_recognition": return satisfies(req,[evidence("ExecutionEvidence","x")],t)
    if fid=="order_invariant":
        a=evidence("RecognitionEvidence","a",independence_group="a"); b=evidence("RecognitionEvidence","b",independence_group="b"); r=requirement([atom(independent_count=2)]); return "Satisfied" if satisfies(r,[a,b],t)==satisfies(r,[b,a],t)=="Satisfied" else "ConflictIndeterminate"
    if fid=="external_proof_not_recognition": return satisfies(req,[evidence("ExternalProofEvidence","x",profile_commitment="profile:v1")],t)
    raise KeyError(fid)

def load_manifest(): return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
def self_test():
    assert commit(EVIDENCE_DOMAIN,{"x":1})!=commit(AUTHORITY_DOMAIN,{"x":1})
    assert set(load_manifest()["evidence_kinds"]).isdisjoint(load_manifest()["authority_kinds"])
    req=requirement([atom()]); assert verify_commitment(req,REQUIREMENT_DOMAIN)
    print("LEX-NET-028 R2 self-test PASS")
def semantic():
    m=load_manifest(); observed=set()
    for f in m["fixtures"]:
        got=run_fixture(f["id"])
        if got!=f["expected"]: raise SystemExit(f"fixture {f['id']} expected {f['expected']} got {got}")
        observed.add(got)
    print(json.dumps({"tranche":"LEX-NET-028","candidate_revision":"R2","fixture_count":len(m["fixtures"]),"observed_outcomes":sorted(observed),"grants_local_authority":False,"grants_external_effect_authority":False,"semantic_result":"PASS"},sort_keys=True))
def scope():
    m=load_manifest(); head=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(); parent=subprocess.check_output(["git","rev-parse","HEAD^"],text=True).strip(); count=int(subprocess.check_output(["git","rev-list","--count",f"{parent}..{head}"],text=True).strip()); paths=sorted(subprocess.check_output(["git","diff","--name-only",parent,head],text=True).splitlines())
    if parent!=m["qualified_parent"]: raise SystemExit(f"parent mismatch {parent}")
    if count!=1: raise SystemExit(f"commit count mismatch {count}")
    if paths!=sorted(m["expected_paths"]): raise SystemExit(f"path set mismatch {paths}")
    print(json.dumps({"tranche":"LEX-NET-028","candidate_revision":"R2","head":head,"parent":parent,"commit_count":count,"paths":paths,"scope_result":"PASS"},sort_keys=True))
def main():
    a=argparse.ArgumentParser(); a.add_argument("--self-test",action="store_true"); a.add_argument("--semantic",action="store_true"); a.add_argument("--scope",action="store_true"); x=a.parse_args()
    if x.self_test:self_test()
    elif x.semantic:semantic()
    elif x.scope:scope()
    else:a.error("select mode")
if __name__=="__main__":main()
