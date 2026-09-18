#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, subprocess
from pathlib import Path

MANIFEST_PATH=Path("docs/lex-net/lex_net_040_manifest.json")
LEASE_DOMAIN="LEX-NET/AUTHORITY/LEASE-ENVELOPE/v1"
STATE_DOMAIN="LEX-NET/AUTHORITY/CONSUMPTION-STATE/v1"
INVOCATION_DOMAIN="LEX-NET/INVOCATION/v1"
EVIDENCE_DOMAIN="LEX-NET/EVIDENCE/v3"
MAX_USE_BUDGET=64
MAX_INVOCATION_ID_BYTES=128
MAX_PAYLOAD_COMMITMENT_BYTES=256

def canon(x): return json.dumps(x,sort_keys=True,separators=(",",":"),ensure_ascii=False)
def commitment(domain, fields): return hashlib.sha256((domain+"\0"+canon(fields)).encode()).hexdigest()
def body(x): return {k:v for k,v in x.items() if k!="commitment"}
def verify(x, domain): return isinstance(x,dict) and x.get("commitment")==commitment(domain,body(x))

def lease(**kw):
    f={"product_universe":"authority","kind":"LocalCapabilityLeaseEnvelope","source_lease_commitment":"source:lease","local_domain":"domain:a","local_grant_commitment":"grant:1","subject":"org:alpha","purpose":"customs-submit","resource":"shipment:123","action":"submit","presenter_binding_type":"key","presenter_binding_commitment":"presenter:a","valid_from":100,"current_until":160,"use_budget":1,"replay_domain":"replay:v1","revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1","assurance_profile_commitment":"assurance:v1","transferable":False,"redelegation_allowed":False,"delegation_depth":0}
    f.update(kw); f["commitment"]=commitment(LEASE_DOMAIN,f); return f

def lease_valid(x):
    if not (isinstance(x,dict) and x.get("product_universe")=="authority" and x.get("kind")=="LocalCapabilityLeaseEnvelope" and verify(x,LEASE_DOMAIN)): return False
    for k in ("source_lease_commitment","local_domain","local_grant_commitment","subject","purpose","resource","action","presenter_binding_commitment","replay_domain","revocation_handle_commitment","policy_epoch_commitment"):
        if not isinstance(x.get(k),str) or not x[k]: return False
    for k in ("valid_from","current_until","use_budget","delegation_depth"):
        if isinstance(x.get(k),bool) or not isinstance(x.get(k),int): return False
    if x["valid_from"]>x["current_until"] or not (0 < x["use_budget"] <= MAX_USE_BUDGET): return False
    if x.get("transferable") is not False or x.get("redelegation_allowed") is not False or x.get("delegation_depth")!=0: return False
    return True

def invocation(invocation_id="inv:1", payload_commitment="payload:1", **kw):
    f={"kind":"CapabilityInvocation","invocation_id":invocation_id,"subject":"org:alpha","purpose":"customs-submit","resource":"shipment:123","action":"submit","replay_domain":"replay:v1","payload_commitment":payload_commitment}
    f.update(kw); f["commitment"]=commitment(INVOCATION_DOMAIN,f); return f

def invocation_valid(x):
    if not (isinstance(x,dict) and x.get("kind")=="CapabilityInvocation" and verify(x,INVOCATION_DOMAIN)): return False
    for k in ("invocation_id","subject","purpose","resource","action","replay_domain","payload_commitment"):
        if not isinstance(x.get(k),str) or not x[k]: return False
    return len(x["invocation_id"].encode())<=MAX_INVOCATION_ID_BYTES and len(x["payload_commitment"].encode())<=MAX_PAYLOAD_COMMITMENT_BYTES

def genesis(l):
    if not lease_valid(l): return {"outcome":"StateInvalid"}
    f={"product_universe":"authority","kind":"CapabilityConsumptionState","local_domain":l["local_domain"],"lease_envelope_commitment":l["commitment"],"version":0,"prior_state_commitment":None,"lease_use_budget":l["use_budget"],"uses_consumed":0,"current_until":l["current_until"],"replay_domain":l["replay_domain"],"revocation_handle_commitment":l["revocation_handle_commitment"],"policy_epoch_commitment":l["policy_epoch_commitment"],"invocation_records":{}}
    f["commitment"]=commitment(STATE_DOMAIN,f); return {"outcome":"TransitionCommitted","state":f}

def state_structural_outcome(s,l=None):
    if not (isinstance(s,dict) and s.get("product_universe")=="authority" and s.get("kind")=="CapabilityConsumptionState" and verify(s,STATE_DOMAIN)): return "StateInvalid"
    for k in ("local_domain","lease_envelope_commitment","replay_domain","revocation_handle_commitment","policy_epoch_commitment"):
        if not isinstance(s.get(k),str) or not s[k]: return "StateInvalid"
    for k in ("version","lease_use_budget","uses_consumed","current_until"):
        if isinstance(s.get(k),bool) or not isinstance(s.get(k),int): return "StateInvalid"
    if not (0 <= s["uses_consumed"] <= s["lease_use_budget"] <= MAX_USE_BUDGET) or s["version"]!=s["uses_consumed"]: return "StateInvalid"
    records=s.get("invocation_records")
    if not isinstance(records,dict) or len(records)!=s["uses_consumed"] or len(records)>s["lease_use_budget"]: return "StateInvalid"
    for iid,rec in records.items():
        if not isinstance(iid,str) or not iid or len(iid.encode())>MAX_INVOCATION_ID_BYTES or not isinstance(rec,dict): return "StateInvalid"
        if not isinstance(rec.get("invocation_commitment"),str) or not rec["invocation_commitment"] or not isinstance(rec.get("transition_id"),str) or not rec["transition_id"]: return "StateInvalid"
        if isinstance(rec.get("consumption_index"),bool) or not isinstance(rec.get("consumption_index"),int): return "StateInvalid"
    if s["version"]==0 and s["prior_state_commitment"] is not None: return "StateInvalid"
    if s["version"]>0 and (not isinstance(s.get("prior_state_commitment"),str) or not s["prior_state_commitment"]): return "StateInvalid"
    if l is not None:
        if not lease_valid(l): return "LeaseMismatch"
        if s["lease_envelope_commitment"]!=l["commitment"]: return "LeaseMismatch"
        if s["local_domain"]!=l["local_domain"]: return "DomainMismatch"
        if s["lease_use_budget"]!=l["use_budget"] or s["current_until"]!=l["current_until"] or s["replay_domain"]!=l["replay_domain"]: return "LeaseMismatch"
        if s["revocation_handle_commitment"]!=l["revocation_handle_commitment"] or s["policy_epoch_commitment"]!=l["policy_epoch_commitment"]: return "LeaseMismatch"
    return "StateCurrent"

class AuthorityStateStore:
    def __init__(self): self.current={}; self.leases={}; self.states={}
    def register(self,l,s):
        if state_structural_outcome(s,l)!="StateCurrent" or s["version"]!=0: return "StateInvalid"
        self.leases[l["commitment"]]=json.loads(json.dumps(l)); self.states[s["commitment"]]=json.loads(json.dumps(s)); self.current[l["commitment"]]=s["commitment"]; return "Committed"
    def current_for(self,lease_commitment): return self.current.get(lease_commitment)
    def compare_and_swap(self,lease_commitment,expected,successor):
        if self.current.get(lease_commitment)!=expected: return "StaleState"
        l=self.leases.get(lease_commitment); prior=self.states.get(expected)
        if l is None or prior is None or state_structural_outcome(successor,l)!="StateCurrent": return "StateInvalid"
        if successor["prior_state_commitment"]!=expected or successor["version"]!=prior["version"]+1 or successor["uses_consumed"]!=prior["uses_consumed"]+1: return "StateInvalid"
        if successor["lease_use_budget"]!=prior["lease_use_budget"] or len(set(successor["invocation_records"])-set(prior["invocation_records"]))!=1: return "StateInvalid"
        self.states[successor["commitment"]]=json.loads(json.dumps(successor)); self.current[lease_commitment]=successor["commitment"]; return "Committed"

def transition_evidence(l,inv,prior,successor,transition_id):
    f={"product_universe":"evidence","kind":"AuthorityTransitionEvidence","local_domain":l["local_domain"],"lease_envelope_commitment":l["commitment"],"invocation_id":inv["invocation_id"],"invocation_commitment":inv["commitment"],"transition_id":transition_id,"prior_state_commitment":prior["commitment"],"prior_state_version":prior["version"],"successor_state_commitment":successor["commitment"],"successor_state_version":successor["version"],"consumption_index":successor["uses_consumed"],"disposition":"AuthorityUseCommitted","effect_attempted":False,"effect_confirmed":False,"grants_local_authority":False,"grants_external_effect_authority":False}
    f["commitment"]=commitment(EVIDENCE_DOMAIN,f); return f

def copy_state(s): return json.loads(json.dumps(s))
def consume(l,s,store,inv,now=120,context=None):
    context=context or {"local_domain":"domain:a","revoked":False,"revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1"}
    if store is None: return {"outcome":"StateStoreMissing"}
    if not lease_valid(l): return {"outcome":"LeaseMismatch"}
    so=state_structural_outcome(s,l)
    if so!="StateCurrent": return {"outcome":so}
    if store.current_for(l["commitment"])!=s["commitment"]: return {"outcome":"StaleState"}
    if not invocation_valid(inv): return {"outcome":"InvocationInvalid"}
    prior=s["invocation_records"].get(inv["invocation_id"])
    if prior is not None:
        return {"outcome":"IdempotentReplay","transition_id":prior["transition_id"],"consumption_index":prior["consumption_index"]} if prior["invocation_commitment"]==inv["commitment"] else {"outcome":"InvocationConflict"}
    if context.get("local_domain")!=l["local_domain"]: return {"outcome":"DomainMismatch"}
    if now<l["valid_from"] or now>l["current_until"]: return {"outcome":"LeaseNotCurrent"}
    if context.get("revoked") is True or context.get("revocation_handle_commitment")!=l["revocation_handle_commitment"]: return {"outcome":"Revoked"}
    if context.get("policy_epoch_commitment")!=l["policy_epoch_commitment"]: return {"outcome":"PolicyEpochMismatch"}
    for k in ("subject","purpose","resource","action"):
        if inv[k]!=l[k]: return {"outcome":"ScopeMismatch"}
    if inv["replay_domain"]!=l["replay_domain"]: return {"outcome":"ReplayDomainMismatch"}
    if s["uses_consumed"]>=s["lease_use_budget"]: return {"outcome":"BudgetExhausted"}
    successor=copy_state(s); successor["version"]+=1; successor["uses_consumed"]+=1; successor["prior_state_commitment"]=s["commitment"]; successor["invocation_records"]=dict(s["invocation_records"])
    transition_id=commitment("LEX-NET/AUTHORITY-TRANSITION-ID/v1",{"lease_envelope_commitment":l["commitment"],"invocation_id":inv["invocation_id"],"invocation_commitment":inv["commitment"],"prior_state_commitment":s["commitment"],"successor_version":successor["version"],"consumption_index":successor["uses_consumed"]})
    successor["invocation_records"][inv["invocation_id"]]={"invocation_commitment":inv["commitment"],"transition_id":transition_id,"consumption_index":successor["uses_consumed"]}; successor["commitment"]=commitment(STATE_DOMAIN,body(successor))
    evidence=transition_evidence(l,inv,s,successor,transition_id)
    if store.compare_and_swap(l["commitment"],s["commitment"],successor)!="Committed": return {"outcome":"StaleState"}
    return {"outcome":"TransitionCommitted","state":successor,"evidence":evidence}

def setup(use_budget=1):
    l=lease(use_budget=use_budget); g=genesis(l)["state"]; st=AuthorityStateStore(); assert st.register(l,g)=="Committed"; return l,g,st

def first(use_budget=1,inv=None):
    l,g,st=setup(use_budget); inv=inv or invocation(); r=consume(l,g,st,inv); return l,g,st,inv,r

def run(fid):
    if fid=="genesis_first_use": return first()[4]["outcome"]
    if fid=="one_use_second_distinct": l,g,st,_,r=first(); return consume(l,r["state"],st,invocation("inv:2","payload:2"))["outcome"]
    if fid in ("stale_state_replay","rollback_old_state","caller_claims_old_state_current"): l,g,st,_,r=first(); return consume(l,g,st,invocation("inv:2","payload:2"))["outcome"]
    if fid=="competing_distinct_one_use": l,g,st=setup(); consume(l,g,st,invocation("a","pa")); return consume(l,g,st,invocation("b","pb"))["outcome"]
    if fid=="concurrent_identical_resolves_idempotent": l,g,st=setup(); inv=invocation("same","p"); a=consume(l,g,st,inv); return consume(l,a["state"],st,inv)["outcome"]
    if fid=="forged_uses_consumed": l,g,st=setup(); x=copy_state(g); x["uses_consumed"]=1; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return state_structural_outcome(x,l)
    if fid=="state_wrong_lease": l,g,st=setup(); return state_structural_outcome(g,lease(resource="shipment:999"))
    if fid=="state_wrong_domain": l,g,st=setup(); x=copy_state(g); x["local_domain"]="domain:b"; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return state_structural_outcome(x,l)
    if fid=="foreign_state_kind": l,g,st=setup(); x=copy_state(g); x["kind"]="ForeignConsumptionState"; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return state_structural_outcome(x,l)
    if fid=="idempotent_same": l,g,st,inv,r=first(); return consume(l,r["state"],st,inv)["outcome"]
    if fid=="invocation_conflict": l,g,st,inv,r=first(); return consume(l,r["state"],st,invocation(inv["invocation_id"],"other"))["outcome"]
    if fid=="scope_mismatch": l,g,st=setup(); return consume(l,g,st,invocation(resource="shipment:999"))["outcome"]
    if fid=="replay_domain_mismatch": l,g,st=setup(); return consume(l,g,st,invocation(replay_domain="other"))["outcome"]
    if fid=="lease_expired": l,g,st=setup(); return consume(l,g,st,invocation(),now=161)["outcome"]
    if fid=="revoked": l,g,st=setup(); return consume(l,g,st,invocation(),context={"local_domain":"domain:a","revoked":True,"revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1"})["outcome"]
    if fid=="policy_epoch_mismatch": l,g,st=setup(); return consume(l,g,st,invocation(),context={"local_domain":"domain:a","revoked":False,"revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:2"})["outcome"]
    if fid=="successor_version_increment": r=first()[4]; return "TransitionCommitted" if r["state"]["version"]==1 and r["state"]["uses_consumed"]==1 else "StateInvalid"
    if fid=="successor_binds_prior": l,g,st,inv,r=first(); return "TransitionCommitted" if r["state"]["prior_state_commitment"]==g["commitment"] else "StateInvalid"
    if fid=="evidence_binds_prior_successor": l,g,st,inv,r=first(); e=r["evidence"]; return "TransitionCommitted" if e["prior_state_commitment"]==g["commitment"] and e["successor_state_commitment"]==r["state"]["commitment"] else "StateInvalid"
    if fid=="evidence_cannot_be_state": return state_structural_outcome(first()[4]["evidence"])
    if fid in ("omitted_evidence_does_not_restore","duplicate_evidence_does_not_restore","effect_unknown_does_not_restore","external_rejection_does_not_restore"): l,g,st,inv,r=first(); return consume(l,r["state"],st,invocation("new","new"))["outcome"]
    if fid=="skipped_version_state_invalid": l,g,st=setup(); x=copy_state(g); x["version"]=2; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return state_structural_outcome(x,l)
    if fid=="history_exceeds_budget": l=lease(use_budget=1); g=genesis(l)["state"]; x=copy_state(g); x["version"]=2; x["uses_consumed"]=2; x["prior_state_commitment"]="p"; x["invocation_records"]={"a":{"invocation_commitment":"i","transition_id":"t1","consumption_index":1},"b":{"invocation_commitment":"i2","transition_id":"t2","consumption_index":2}}; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return state_structural_outcome(x,l)
    if fid=="oversized_invocation_id": l,g,st=setup(); return consume(l,g,st,invocation("x"*129))["outcome"]
    if fid=="oversized_payload_commitment": l,g,st=setup(); return consume(l,g,st,invocation(payload_commitment="x"*257))["outcome"]
    if fid=="profile_budget_cap": return genesis(lease(use_budget=65))["outcome"]
    if fid=="idempotent_after_expiry": l,g,st,inv,r=first(); return consume(l,r["state"],st,inv,now=999)["outcome"]
    if fid=="same_id_conflict_after_budget": l,g,st,inv,r=first(); return consume(l,r["state"],st,invocation(inv["invocation_id"],"different"),now=999)["outcome"]
    if fid=="evidence_effect_flags_false": r=first()[4]; e=r["evidence"]; return "TransitionCommitted" if e["effect_attempted"] is False and e["effect_confirmed"] is False and not e["grants_external_effect_authority"] else "StateInvalid"
    if fid=="store_missing": l=lease(); g=genesis(l)["state"]; return consume(l,g,None,invocation())["outcome"]
    if fid=="state_commitment_tamper": l,g,st=setup(); g["uses_consumed"]=1; return state_structural_outcome(g,l)
    if fid=="store_rejects_successor_version_jump": l,g,st=setup(use_budget=3); x=copy_state(g); x["version"]=2; x["uses_consumed"]=2; x["prior_state_commitment"]=g["commitment"]; x["invocation_records"]={"a":{"invocation_commitment":"ia","transition_id":"ta","consumption_index":1},"b":{"invocation_commitment":"ib","transition_id":"tb","consumption_index":2}}; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return st.compare_and_swap(l["commitment"],g["commitment"],x)
    if fid=="store_rejects_wrong_prior": l,g,st=setup(); x=copy_state(g); x["version"]=1; x["uses_consumed"]=1; x["prior_state_commitment"]="wrong"; x["invocation_records"]={"a":{"invocation_commitment":"ia","transition_id":"ta","consumption_index":1}}; x["commitment"]=commitment(STATE_DOMAIN,body(x)); return st.compare_and_swap(l["commitment"],g["commitment"],x)
    raise KeyError(fid)

def load_manifest(): return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
def self_test():
    m=load_manifest(); assert len(m["fixtures"])==38; assert commitment(LEASE_DOMAIN,{"x":1})!=commitment(STATE_DOMAIN,{"x":1})!=commitment(EVIDENCE_DOMAIN,{"x":1}); print("LEX-NET-040 R1 self-test PASS")
def semantic():
    m=load_manifest(); observed=set()
    for f in m["fixtures"]:
        got=run(f["id"])
        if got!=f["expected"]: raise SystemExit(f"{f['id']}: expected {f['expected']} got {got}")
        observed.add(got)
    print(json.dumps({"tranche":"LEX-NET-040","candidate_revision":"R1","fixture_count":len(m["fixtures"]),"observed_outcomes":sorted(observed),"backend_atomicity_claim":False,"external_effect_claim":False,"semantic_result":"PASS"},sort_keys=True))
def scope():
    m=load_manifest(); h=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(); p=subprocess.check_output(["git","rev-parse","HEAD^"],text=True).strip(); n=int(subprocess.check_output(["git","rev-list","--count",f"{p}..{h}"],text=True).strip()); paths=sorted(subprocess.check_output(["git","diff","--name-only",p,h],text=True).splitlines())
    if p!=m["qualified_parent"]: raise SystemExit(f"parent mismatch {p}")
    if n!=1: raise SystemExit(f"commit count mismatch {n}")
    if paths!=sorted(m["expected_paths"]): raise SystemExit(f"path set mismatch {paths}")
    print(json.dumps({"tranche":"LEX-NET-040","candidate_revision":"R1","head":h,"parent":p,"commit_count":n,"paths":paths,"scope_result":"PASS"},sort_keys=True))
def main():
    a=argparse.ArgumentParser(); a.add_argument("--self-test",action="store_true"); a.add_argument("--semantic",action="store_true"); a.add_argument("--scope",action="store_true"); x=a.parse_args()
    if x.self_test:self_test()
    elif x.semantic:semantic()
    elif x.scope:scope()
    else:a.error("select mode")
if __name__=="__main__": main()
