#!/usr/bin/env python3
from __future__ import annotations
import argparse, copy, hashlib, json, subprocess
from pathlib import Path

MANIFEST_PATH = Path("docs/lex-net/lex_net_028_manifest.json")
DOC_PATH = Path("docs/lex-net/LEX_NET_TYPED_TRANSITION_EVIDENCE_V1.md")
EVIDENCE_DOMAIN = "LEX-NET/EVIDENCE/v1"
AUTHORITY_DOMAIN = "LEX-NET/AUTHORITY/v1"

def canon(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)

def commitment(domain, fields):
    return hashlib.sha256((domain + "\0" + canon(fields)).encode()).hexdigest()

def evidence(kind, evidence_id, *, subject="org:alpha", purpose="customs-submit",
             resource="shipment:123", action="submit", profile_commitment="profile:v1",
             disposition="positive", current_until=200, independence_group="issuer:alpha",
             predecessors=None):
    fields = {
        "product_universe":"evidence","kind":kind,"evidence_id":evidence_id,
        "subject":subject,"purpose":purpose,"resource":resource,"action":action,
        "profile_commitment":profile_commitment,"disposition":disposition,
        "current_until":current_until,"independence_group":independence_group,
        "predecessors":list(predecessors or []),
        "grants_local_authority":False,"grants_external_effect_authority":False,
    }
    fields["commitment"] = commitment(EVIDENCE_DOMAIN, fields)
    return fields

def requirement(atom=None, *, alternatives=None, alternative_policy_id=None):
    return {"atoms": list(atom or []), "alternatives": alternatives or [],
            "alternative_policy_id": alternative_policy_id}

def atom(kind="RecognitionEvidence", *, subject="org:alpha", purpose="customs-submit",
         resource="shipment:123", action="submit", profile_commitment="profile:v1",
         accepted_disposition="positive", requires_current=True, independent_count=1):
    return {
        "kind":kind,"subject":subject,"purpose":purpose,"resource":resource,"action":action,
        "profile_commitment":profile_commitment,"accepted_disposition":accepted_disposition,
        "requires_current":requires_current,"independent_count":independent_count,
    }

def _current(e, by_id, evaluation_time, seen=None):
    seen=set(seen or ())
    if e["evidence_id"] in seen:
        return "MissingDependency"
    seen.add(e["evidence_id"])
    if e.get("current_until") is not None and evaluation_time > e["current_until"]:
        return "StaleDependency"
    for pred_id in e.get("predecessors", []):
        pred = by_id.get(pred_id)
        if pred is None:
            return "MissingDependency"
        result = _current(pred, by_id, evaluation_time, seen)
        if result != "Satisfied":
            return result
    return "Satisfied"

def _satisfy_atoms(atoms, products, evaluation_time):
    if any(p.get("product_universe") == "authority" for p in products):
        return "AuthorityProductRejected"
    dedup = {p["commitment"]:p for p in products if p.get("product_universe")=="evidence"}
    products=list(dedup.values())
    by_id={p["evidence_id"]:p for p in products}
    for req in atoms:
        kind_candidates=[p for p in products if p["kind"]==req["kind"]]
        if not kind_candidates:
            return "MissingRequiredEvidence"
        scoped=[p for p in kind_candidates if
                p["subject"]==req["subject"] and p["purpose"]==req["purpose"] and
                p["resource"]==req["resource"] and p["action"]==req["action"]]
        if not scoped:
            return "ScopeMismatch"
        profiled=[p for p in scoped if p["profile_commitment"]==req["profile_commitment"]]
        if not profiled:
            return "ProfileMismatch"
        dispositions={p["disposition"] for p in profiled}
        if len(dispositions)>1:
            return "ConflictIndeterminate"
        accepted=[p for p in profiled if p["disposition"]==req["accepted_disposition"]]
        if not accepted:
            return "DispositionMismatch"
        if req.get("requires_current", True):
            for p in accepted:
                cur=_current(p, by_id, evaluation_time)
                if cur != "Satisfied":
                    return cur
        need=int(req.get("independent_count",1))
        independent={p["independence_group"] for p in accepted}
        if len(independent) < need:
            return "IndependenceInsufficient"
    return "Satisfied"

def satisfies(req, products, evaluation_time):
    if req.get("alternatives"):
        if not req.get("alternative_policy_id"):
            return "AlternativePolicyRequired"
        results=[_satisfy_atoms(option, products, evaluation_time) for option in req["alternatives"]]
        return "Satisfied" if "Satisfied" in results else results[0]
    return _satisfy_atoms(req.get("atoms",[]), products, evaluation_time)

def mint_local_capability(*, satisfaction, local_mint_authorized, local_domain,
                          local_grant_commitment, subject, purpose, resource, action,
                          current_until, use_budget):
    if satisfaction != "Satisfied":
        return {"outcome":"MintPreconditionUnsatisfied"}
    if not local_mint_authorized:
        return {"outcome":"MintNotAuthorized"}
    if not local_domain or not local_grant_commitment or use_budget <= 0:
        return {"outcome":"MintNotAuthorized"}
    fields={
        "product_universe":"authority","kind":"LocalCapabilityLease",
        "local_domain":local_domain,"local_grant_commitment":local_grant_commitment,
        "subject":subject,"purpose":purpose,"resource":resource,"action":action,
        "current_until":current_until,"remaining_uses":use_budget,
    }
    fields["commitment"]=commitment(AUTHORITY_DOMAIN, fields)
    return {"outcome":"Satisfied","product":fields}

def export_authority_as_evidence(authority, evidence_id):
    return evidence("EvidenceAboutForeignAuthority", evidence_id,
                    subject=authority["subject"], purpose=authority["purpose"],
                    resource=authority["resource"], action=authority["action"],
                    profile_commitment="foreign-authority-evidence:v1",
                    disposition="observed", current_until=authority["current_until"],
                    independence_group="source-domain:"+authority["local_domain"])

def consume_authority(authority, *, execution_id):
    if authority.get("remaining_uses",0) <= 0:
        return {"outcome":"AuthorityConsumed"}
    out=copy.deepcopy(authority)
    out["remaining_uses"] -= 1
    out["commitment"]=commitment(AUTHORITY_DOMAIN, {k:v for k,v in out.items() if k!="commitment"})
    execution=evidence("ExecutionEvidence", execution_id,
                       subject=authority["subject"],purpose=authority["purpose"],
                       resource=authority["resource"],action=authority["action"],
                       profile_commitment="execution:v1",disposition="attempted",
                       current_until=authority["current_until"],
                       independence_group="executor:"+authority["local_domain"])
    return {"outcome":"Satisfied","authority":out,"evidence":execution}

def base_products():
    interp=evidence("InterpretationEvidence","i1",profile_commitment="interp:v1",independence_group="parser:a")
    recognition=evidence("RecognitionEvidence","r1",profile_commitment="profile:v1",independence_group="recognizer:a",
                         predecessors=["i1"])
    return [interp,recognition]

def run_fixture(fid):
    req=requirement([atom()])
    products=base_products()
    t=100
    if fid=="exact_match": return satisfies(req,products,t)
    if fid=="wrong_kind": return satisfies(requirement([atom(kind="TranslationEvidence")]),products,t)
    if fid=="scope_mismatch": return satisfies(requirement([atom(subject="org:beta")]),products,t)
    if fid=="profile_mismatch": return satisfies(requirement([atom(profile_commitment="profile:v2")]),products,t)
    if fid=="disposition_mismatch": return satisfies(requirement([atom(accepted_disposition="negative")]),products,t)
    if fid=="stale_direct":
        return satisfies(req,[evidence("RecognitionEvidence","r2",current_until=50,independence_group="recognizer:a")],t)
    if fid=="missing_predecessor":
        return satisfies(req,[evidence("RecognitionEvidence","r2",predecessors=["missing"],independence_group="recognizer:a")],t)
    if fid=="stale_predecessor":
        dep=evidence("InterpretationEvidence","i2",current_until=50,profile_commitment="interp:v1")
        p=evidence("RecognitionEvidence","r2",predecessors=["i2"],independence_group="recognizer:a")
        return satisfies(req,[dep,p],t)
    if fid=="duplicate_not_independent":
        p=evidence("RecognitionEvidence","r2",independence_group="same")
        return satisfies(requirement([atom(independent_count=2)]),[p,copy.deepcopy(p)],t)
    if fid=="correlated_not_independent":
        a=evidence("RecognitionEvidence","ra",independence_group="same")
        b=evidence("RecognitionEvidence","rb",independence_group="same")
        return satisfies(requirement([atom(independent_count=2)]),[a,b],t)
    if fid=="independent_two":
        a=evidence("RecognitionEvidence","ra",independence_group="a")
        b=evidence("RecognitionEvidence","rb",independence_group="b")
        return satisfies(requirement([atom(independent_count=2)]),[a,b],t)
    if fid=="conflict":
        a=evidence("RecognitionEvidence","ra",disposition="positive",independence_group="a")
        b=evidence("RecognitionEvidence","rb",disposition="negative",independence_group="b")
        return satisfies(req,[a,b],t)
    if fid=="majority_does_not_override_conflict":
        a=evidence("RecognitionEvidence","ra",disposition="positive",independence_group="a")
        b=evidence("RecognitionEvidence","rb",disposition="positive",independence_group="b")
        c=evidence("RecognitionEvidence","rc",disposition="negative",independence_group="c")
        return satisfies(req,[a,b,c],t)
    if fid=="alternatives_require_policy":
        return satisfies(requirement(alternatives=[[atom()],[atom(kind="ExternalProofEvidence")]]),products,t)
    if fid=="explicit_alternative":
        return satisfies(requirement(alternatives=[[atom(kind="ExternalProofEvidence")],[atom()]],alternative_policy_id="local-policy:v1"),products,t)
    if fid=="authority_rejected_as_evidence":
        m=mint_local_capability(satisfaction="Satisfied",local_mint_authorized=True,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)
        return satisfies(req,[m["product"]],t)
    if fid=="unauthorized_mint":
        return mint_local_capability(satisfaction="Satisfied",local_mint_authorized=False,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)["outcome"]
    if fid=="unsatisfied_mint":
        return mint_local_capability(satisfaction="MissingRequiredEvidence",local_mint_authorized=True,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)["outcome"]
    if fid=="authorized_mint_domain_separated":
        m=mint_local_capability(satisfaction="Satisfied",local_mint_authorized=True,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)
        ev=evidence("EvidenceAboutForeignAuthority","fa1",profile_commitment="foreign-authority-evidence:v1",disposition="observed")
        return "Satisfied" if m["outcome"]=="Satisfied" and m["product"]["product_universe"]=="authority" and m["product"]["commitment"] != ev["commitment"] else "MintNotAuthorized"
    if fid=="export_authority_degrades_to_evidence":
        m=mint_local_capability(satisfaction="Satisfied",local_mint_authorized=True,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)
        ev=export_authority_as_evidence(m["product"],"fa1")
        return "Satisfied" if ev["product_universe"]=="evidence" and ev["kind"]=="EvidenceAboutForeignAuthority" and ev["grants_local_authority"] is False and ev["commitment"] != m["product"]["commitment"] else "AuthorityProductRejected"
    if fid=="consume_once":
        m=mint_local_capability(satisfaction="Satisfied",local_mint_authorized=True,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)
        c=consume_authority(m["product"],execution_id="x1")
        return c["outcome"] if c["evidence"]["product_universe"]=="evidence" and c["authority"]["remaining_uses"]==0 else "AuthorityConsumed"
    if fid=="consume_twice_rejected":
        m=mint_local_capability(satisfaction="Satisfied",local_mint_authorized=True,local_domain="domain:a",local_grant_commitment="grant:1",subject="org:alpha",purpose="customs-submit",resource="shipment:123",action="submit",current_until=200,use_budget=1)
        c1=consume_authority(m["product"],execution_id="x1")
        return consume_authority(c1["authority"],execution_id="x2")["outcome"]
    if fid=="execution_evidence_not_recognition":
        return satisfies(req,[evidence("ExecutionEvidence","x1",profile_commitment="execution:v1",disposition="attempted")],t)
    if fid=="order_invariant":
        a=evidence("RecognitionEvidence","ra",independence_group="a"); b=evidence("RecognitionEvidence","rb",independence_group="b")
        r=requirement([atom(independent_count=2)])
        return "Satisfied" if satisfies(r,[a,b],t)==satisfies(r,[b,a],t)=="Satisfied" else "ConflictIndeterminate"
    if fid=="unrelated_extra_does_not_help":
        return satisfies(req,[evidence("ExternalProofEvidence","x1",profile_commitment="ext:v1")],t)
    if fid=="external_proof_not_recognition":
        return satisfies(req,[evidence("ExternalProofEvidence","x1",profile_commitment="profile:v1")],t)
    raise KeyError(fid)

def load_manifest():
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

def self_test():
    a=evidence("RecognitionEvidence","a")
    assert a["product_universe"]=="evidence" and not a["grants_local_authority"]
    assert commitment(EVIDENCE_DOMAIN,{"x":1}) != commitment(AUTHORITY_DOMAIN,{"x":1})
    assert set(load_manifest()["evidence_kinds"]).isdisjoint(load_manifest()["authority_kinds"])
    print("LEX-NET-028 self-test PASS")

def semantic():
    manifest=load_manifest(); observed=set()
    for f in manifest["fixtures"]:
        got=run_fixture(f["id"])
        if got != f["expected"]:
            raise SystemExit(f"fixture {f['id']} expected {f['expected']} got {got}")
        observed.add(got)
    print(json.dumps({"tranche":"LEX-NET-028","fixture_count":len(manifest["fixtures"]),"observed_outcomes":sorted(observed),"grants_local_authority":False,"grants_external_effect_authority":False,"semantic_result":"PASS"},sort_keys=True))

def scope():
    manifest=load_manifest()
    head=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip()
    parent=subprocess.check_output(["git","rev-parse","HEAD^"],text=True).strip()
    count=int(subprocess.check_output(["git","rev-list","--count",f"{parent}..{head}"],text=True).strip())
    paths=sorted(subprocess.check_output(["git","diff","--name-only",parent,head],text=True).splitlines())
    if parent != manifest["qualified_parent"]: raise SystemExit(f"parent mismatch {parent}")
    if count != 1: raise SystemExit(f"commit count mismatch {count}")
    if paths != sorted(manifest["expected_paths"]): raise SystemExit(f"path set mismatch {paths}")
    print(json.dumps({"tranche":"LEX-NET-028","head":head,"parent":parent,"commit_count":count,"paths":paths,"scope_result":"PASS"},sort_keys=True))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--self-test",action="store_true"); ap.add_argument("--semantic",action="store_true"); ap.add_argument("--scope",action="store_true"); args=ap.parse_args()
    if args.self_test: self_test()
    elif args.semantic: semantic()
    elif args.scope: scope()
    else: ap.error("select a mode")
if __name__=="__main__": main()
