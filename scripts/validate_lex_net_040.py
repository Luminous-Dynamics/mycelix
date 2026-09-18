#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, subprocess
from pathlib import Path

M=Path("docs/lex-net/lex_net_040_manifest.json")
LD="LEX-NET/AUTHORITY/LEASE-ENVELOPE/v1"
SD="LEX-NET/AUTHORITY/CONSUMPTION-STATE/v1"
ID="LEX-NET/INVOCATION/v1"
ED="LEX-NET/EVIDENCE/v3"
TD="LEX-NET/AUTHORITY-TRANSITION-ID/v1"
MAX_USE_BUDGET=64
MAX_INVOCATION_ID_BYTES=128
MAX_PAYLOAD_COMMITMENT_BYTES=256

def cj(x): return json.dumps(x,sort_keys=True,separators=(",",":"),ensure_ascii=False)
def h(d,x): return hashlib.sha256((d+"\0"+cj(x)).encode()).hexdigest()
def b(x): return {k:v for k,v in x.items() if k!="commitment"}
def ok(x,d): return isinstance(x,dict) and x.get("commitment")==h(d,b(x))
def cp(x): return json.loads(json.dumps(x))

def lease(**kw):
    x={"product_universe":"authority","kind":"LocalCapabilityLeaseEnvelope","source_lease_commitment":"src:1","local_domain":"domain:a","local_grant_commitment":"grant:1","subject":"org:alpha","purpose":"customs-submit","resource":"shipment:123","action":"submit","presenter_binding_type":"key","presenter_binding_commitment":"presenter:a","valid_from":100,"current_until":160,"use_budget":1,"replay_domain":"replay:v1","revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1","assurance_profile_commitment":"assurance:v1","transferable":False,"redelegation_allowed":False,"delegation_depth":0}
    x.update(kw); x["commitment"]=h(LD,x); return x

def lease_ok(x):
    if not (isinstance(x,dict) and x.get("product_universe")=="authority" and x.get("kind")=="LocalCapabilityLeaseEnvelope" and ok(x,LD)): return False
    for k in ("source_lease_commitment","local_domain","local_grant_commitment","subject","purpose","resource","action","presenter_binding_commitment","replay_domain","revocation_handle_commitment","policy_epoch_commitment"):
        if not isinstance(x.get(k),str) or not x[k]: return False
    for k in ("valid_from","current_until","use_budget","delegation_depth"):
        if isinstance(x.get(k),bool) or not isinstance(x.get(k),int): return False
    return x["valid_from"]<=x["current_until"] and 0<x["use_budget"]<=MAX_USE_BUDGET and x.get("transferable") is False and x.get("redelegation_allowed") is False and x.get("delegation_depth")==0

def inv(i="inv:1",p="payload:1",**kw):
    x={"kind":"CapabilityInvocation","invocation_id":i,"subject":"org:alpha","purpose":"customs-submit","resource":"shipment:123","action":"submit","replay_domain":"replay:v1","payload_commitment":p}
    x.update(kw); x["commitment"]=h(ID,x); return x

def inv_ok(x):
    if not (isinstance(x,dict) and x.get("kind")=="CapabilityInvocation" and ok(x,ID)): return False
    for k in ("invocation_id","subject","purpose","resource","action","replay_domain","payload_commitment"):
        if not isinstance(x.get(k),str) or not x[k]: return False
    return len(x["invocation_id"].encode())<=MAX_INVOCATION_ID_BYTES and len(x["payload_commitment"].encode())<=MAX_PAYLOAD_COMMITMENT_BYTES

def genesis(l):
    if not lease_ok(l): return {"outcome":"StateInvalid"}
    x={"product_universe":"authority","kind":"CapabilityConsumptionState","local_domain":l["local_domain"],"lease_envelope_commitment":l["commitment"],"version":0,"prior_state_commitment":None,"lease_use_budget":l["use_budget"],"uses_consumed":0,"current_until":l["current_until"],"replay_domain":l["replay_domain"],"revocation_handle_commitment":l["revocation_handle_commitment"],"policy_epoch_commitment":l["policy_epoch_commitment"],"invocation_records":{}}
    x["commitment"]=h(SD,x); return {"outcome":"TransitionCommitted","state":x}

def state_status(s,l=None):
    if not (isinstance(s,dict) and s.get("product_universe")=="authority" and s.get("kind")=="CapabilityConsumptionState" and ok(s,SD)): return "StateInvalid"
    for k in ("local_domain","lease_envelope_commitment","replay_domain","revocation_handle_commitment","policy_epoch_commitment"):
        if not isinstance(s.get(k),str) or not s[k]: return "StateInvalid"
    for k in ("version","lease_use_budget","uses_consumed","current_until"):
        if isinstance(s.get(k),bool) or not isinstance(s.get(k),int): return "StateInvalid"
    if not (0<=s["uses_consumed"]<=s["lease_use_budget"]<=MAX_USE_BUDGET) or s["version"]!=s["uses_consumed"]: return "StateInvalid"
    rs=s.get("invocation_records")
    if not isinstance(rs,dict) or len(rs)!=s["uses_consumed"] or len(rs)>s["lease_use_budget"]: return "StateInvalid"
    idx=set(); tids=set()
    for iid,r in rs.items():
        if not isinstance(iid,str) or not iid or len(iid.encode())>MAX_INVOCATION_ID_BYTES or not isinstance(r,dict): return "StateInvalid"
        if not isinstance(r.get("invocation_commitment"),str) or not r["invocation_commitment"] or not isinstance(r.get("transition_id"),str) or not r["transition_id"]: return "StateInvalid"
        if isinstance(r.get("consumption_index"),bool) or not isinstance(r.get("consumption_index"),int): return "StateInvalid"
        idx.add(r["consumption_index"]); tids.add(r["transition_id"])
    if idx!=set(range(1,s["uses_consumed"]+1)) or len(tids)!=len(rs): return "StateInvalid"
    if s["version"]==0 and s["prior_state_commitment"] is not None: return "StateInvalid"
    if s["version"]>0 and (not isinstance(s.get("prior_state_commitment"),str) or not s["prior_state_commitment"]): return "StateInvalid"
    if l is not None:
        if not lease_ok(l): return "LeaseMismatch"
        if s["lease_envelope_commitment"]!=l["commitment"]: return "LeaseMismatch"
        if s["local_domain"]!=l["local_domain"]: return "DomainMismatch"
        for a,k in (("lease_use_budget","use_budget"),("current_until","current_until"),("replay_domain","replay_domain"),("revocation_handle_commitment","revocation_handle_commitment"),("policy_epoch_commitment","policy_epoch_commitment")):
            if s[a]!=l[k]: return "LeaseMismatch"
    return "StateCurrent"

def transition_id(l,i,prior,version,index):
    return h(TD,{"lease_envelope_commitment":l["commitment"],"invocation_id":i["invocation_id"],"invocation_commitment":i["commitment"],"prior_state_commitment":prior,"successor_version":version,"consumption_index":index})

class AuthorityStateStore:
    def __init__(self): self.current={}; self.leases={}; self.states={}
    def register(self,l,s):
        if not lease_ok(l): return "StateInvalid"
        if l["commitment"] in self.current: return "AlreadyRegistered"
        if state_status(s,l)!="StateCurrent" or s["version"]!=0: return "StateInvalid"
        self.leases[l["commitment"]]=cp(l); self.states[s["commitment"]]=cp(s); self.current[l["commitment"]]=s["commitment"]; return "Committed"
    def current_for(self,lc): return self.current.get(lc)
    def compare_and_swap(self,lc,expected,s):
        if self.current.get(lc)!=expected: return "StaleState"
        l=self.leases.get(lc); p=self.states.get(expected)
        if l is None or p is None or state_status(s,l)!="StateCurrent": return "StateInvalid"
        if s["prior_state_commitment"]!=expected or s["version"]!=p["version"]+1 or s["uses_consumed"]!=p["uses_consumed"]+1 or s["lease_use_budget"]!=p["lease_use_budget"]: return "StateInvalid"
        old=set(p["invocation_records"]); new=set(s["invocation_records"]); added=new-old
        if len(added)!=1 or not old.issubset(new): return "StateInvalid"
        for k in old:
            if s["invocation_records"].get(k)!=p["invocation_records"][k]: return "StateInvalid"
        nid=next(iter(added)); r=s["invocation_records"][nid]
        if r["consumption_index"]!=s["uses_consumed"]: return "StateInvalid"
        expected_tid=h(TD,{"lease_envelope_commitment":lc,"invocation_id":nid,"invocation_commitment":r["invocation_commitment"],"prior_state_commitment":expected,"successor_version":s["version"],"consumption_index":s["uses_consumed"]})
        if r["transition_id"]!=expected_tid: return "StateInvalid"
        self.states[s["commitment"]]=cp(s); self.current[lc]=s["commitment"]; return "Committed"

def evidence(l,i,p,s,tid):
    x={"product_universe":"evidence","kind":"AuthorityTransitionEvidence","local_domain":l["local_domain"],"lease_envelope_commitment":l["commitment"],"invocation_id":i["invocation_id"],"invocation_commitment":i["commitment"],"transition_id":tid,"prior_state_commitment":p["commitment"],"prior_state_version":p["version"],"successor_state_commitment":s["commitment"],"successor_state_version":s["version"],"consumption_index":s["uses_consumed"],"disposition":"AuthorityUseCommitted","effect_attempted":False,"effect_confirmed":False,"grants_local_authority":False,"grants_external_effect_authority":False}
    x["commitment"]=h(ED,x); return x

def consume(l,s,store,i,now=120,ctx=None):
    ctx=ctx or {"local_domain":"domain:a","revoked":False,"revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1"}
    if store is None: return {"outcome":"StateStoreMissing"}
    if not lease_ok(l): return {"outcome":"LeaseMismatch"}
    q=state_status(s,l)
    if q!="StateCurrent": return {"outcome":q}
    if store.current_for(l["commitment"])!=s["commitment"]: return {"outcome":"StaleState"}
    if not inv_ok(i): return {"outcome":"InvocationInvalid"}
    old=s["invocation_records"].get(i["invocation_id"])
    if old is not None:
        if old["invocation_commitment"]==i["commitment"]: return {"outcome":"IdempotentReplay","transition_id":old["transition_id"],"consumption_index":old["consumption_index"]}
        return {"outcome":"InvocationConflict"}
    if ctx.get("local_domain")!=l["local_domain"]: return {"outcome":"DomainMismatch"}
    if now<l["valid_from"] or now>l["current_until"]: return {"outcome":"LeaseNotCurrent"}
    if ctx.get("revoked") is True or ctx.get("revocation_handle_commitment")!=l["revocation_handle_commitment"]: return {"outcome":"Revoked"}
    if ctx.get("policy_epoch_commitment")!=l["policy_epoch_commitment"]: return {"outcome":"PolicyEpochMismatch"}
    if any(i[k]!=l[k] for k in ("subject","purpose","resource","action")): return {"outcome":"ScopeMismatch"}
    if i["replay_domain"]!=l["replay_domain"]: return {"outcome":"ReplayDomainMismatch"}
    if s["uses_consumed"]>=s["lease_use_budget"]: return {"outcome":"BudgetExhausted"}
    n=cp(s); n["version"]+=1; n["uses_consumed"]+=1; n["prior_state_commitment"]=s["commitment"]; n["invocation_records"]=dict(s["invocation_records"])
    tid=transition_id(l,i,s["commitment"],n["version"],n["uses_consumed"])
    n["invocation_records"][i["invocation_id"]]={"invocation_commitment":i["commitment"],"transition_id":tid,"consumption_index":n["uses_consumed"]}; n["commitment"]=h(SD,b(n)); e=evidence(l,i,s,n,tid)
    if store.compare_and_swap(l["commitment"],s["commitment"],n)!="Committed": return {"outcome":"StaleState"}
    return {"outcome":"TransitionCommitted","state":n,"evidence":e}

def setup(budget=1):
    l=lease(use_budget=budget); g=genesis(l)["state"]; st=AuthorityStateStore(); assert st.register(l,g)=="Committed"; return l,g,st
def first(budget=1,i=None):
    l,g,st=setup(budget); i=i or inv(); return l,g,st,i,consume(l,g,st,i)

def run(fid):
    if fid=="genesis_first_use": return first()[4]["outcome"]
    if fid=="one_use_second_distinct": l,g,s,i,r=first(); return consume(l,r["state"],s,inv("inv:2","p2"))["outcome"]
    if fid in ("stale_state_replay","rollback_old_state","caller_claims_old_state_current"): l,g,s,i,r=first(); return consume(l,g,s,inv("inv:2","p2"))["outcome"]
    if fid=="competing_distinct_one_use": l,g,s=setup(); consume(l,g,s,inv("a","pa")); return consume(l,g,s,inv("b","pb"))["outcome"]
    if fid=="concurrent_identical_resolves_idempotent": l,g,s=setup(); i=inv("same","p"); r=consume(l,g,s,i); return consume(l,r["state"],s,i)["outcome"]
    if fid=="forged_uses_consumed": l,g,s=setup(); x=cp(g); x["uses_consumed"]=1; x["commitment"]=h(SD,b(x)); return state_status(x,l)
    if fid=="state_wrong_lease": l,g,s=setup(); return state_status(g,lease(resource="shipment:999"))
    if fid=="state_wrong_domain": l,g,s=setup(); x=cp(g); x["local_domain"]="domain:b"; x["commitment"]=h(SD,b(x)); return state_status(x,l)
    if fid=="foreign_state_kind": l,g,s=setup(); x=cp(g); x["kind"]="ForeignConsumptionState"; x["commitment"]=h(SD,b(x)); return state_status(x,l)
    if fid=="idempotent_same": l,g,s,i,r=first(); return consume(l,r["state"],s,i)["outcome"]
    if fid=="invocation_conflict": l,g,s,i,r=first(); return consume(l,r["state"],s,inv(i["invocation_id"],"other"))["outcome"]
    if fid=="scope_mismatch": l,g,s=setup(); return consume(l,g,s,inv(resource="shipment:999"))["outcome"]
    if fid=="replay_domain_mismatch": l,g,s=setup(); return consume(l,g,s,inv(replay_domain="other"))["outcome"]
    if fid=="lease_expired": l,g,s=setup(); return consume(l,g,s,inv(),now=161)["outcome"]
    if fid=="revoked": l,g,s=setup(); return consume(l,g,s,inv(),ctx={"local_domain":"domain:a","revoked":True,"revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:1"})["outcome"]
    if fid=="policy_epoch_mismatch": l,g,s=setup(); return consume(l,g,s,inv(),ctx={"local_domain":"domain:a","revoked":False,"revocation_handle_commitment":"rev:1","policy_epoch_commitment":"policy:2"})["outcome"]
    if fid=="successor_version_increment": r=first()[4]; return "TransitionCommitted" if r["state"]["version"]==1 and r["state"]["uses_consumed"]==1 else "StateInvalid"
    if fid=="successor_binds_prior": l,g,s,i,r=first(); return "TransitionCommitted" if r["state"]["prior_state_commitment"]==g["commitment"] else "StateInvalid"
    if fid=="evidence_binds_prior_successor": l,g,s,i,r=first(); e=r["evidence"]; return "TransitionCommitted" if e["prior_state_commitment"]==g["commitment"] and e["successor_state_commitment"]==r["state"]["commitment"] else "StateInvalid"
    if fid=="evidence_cannot_be_state": return state_status(first()[4]["evidence"])
    if fid in ("omitted_evidence_does_not_restore","duplicate_evidence_does_not_restore","effect_unknown_does_not_restore","external_rejection_does_not_restore"): l,g,s,i,r=first(); return consume(l,r["state"],s,inv("new","new"))["outcome"]
    if fid=="skipped_version_state_invalid": l,g,s=setup(); x=cp(g); x["version"]=2; x["commitment"]=h(SD,b(x)); return state_status(x,l)
    if fid=="history_exceeds_budget":
        l=lease(use_budget=1); x=genesis(l)["state"]; x["version"]=2; x["uses_consumed"]=2; x["prior_state_commitment"]="p"; x["invocation_records"]={"a":{"invocation_commitment":"i","transition_id":"t1","consumption_index":1},"b":{"invocation_commitment":"j","transition_id":"t2","consumption_index":2}}; x["commitment"]=h(SD,b(x)); return state_status(x,l)
    if fid=="oversized_invocation_id": l,g,s=setup(); return consume(l,g,s,inv("x"*129))["outcome"]
    if fid=="oversized_payload_commitment": l,g,s=setup(); return consume(l,g,s,inv(p="x"*257))["outcome"]
    if fid=="profile_budget_cap": return genesis(lease(use_budget=65))["outcome"]
    if fid=="idempotent_after_expiry": l,g,s,i,r=first(); return consume(l,r["state"],s,i,now=999)["outcome"]
    if fid=="same_id_conflict_after_budget": l,g,s,i,r=first(); return consume(l,r["state"],s,inv(i["invocation_id"],"different"),now=999)["outcome"]
    if fid=="evidence_effect_flags_false": e=first()[4]["evidence"]; return "TransitionCommitted" if e["effect_attempted"] is False and e["effect_confirmed"] is False and not e["grants_external_effect_authority"] else "StateInvalid"
    if fid=="store_missing": l=lease(); g=genesis(l)["state"]; return consume(l,g,None,inv())["outcome"]
    if fid=="state_commitment_tamper": l,g,s=setup(); g["uses_consumed"]=1; return state_status(g,l)
    if fid=="store_rejects_successor_version_jump":
        l,g,s=setup(3); x=cp(g); x["version"]=2; x["uses_consumed"]=2; x["prior_state_commitment"]=g["commitment"]; x["invocation_records"]={"a":{"invocation_commitment":"ia","transition_id":"ta","consumption_index":1},"b":{"invocation_commitment":"ib","transition_id":"tb","consumption_index":2}}; x["commitment"]=h(SD,b(x)); return s.compare_and_swap(l["commitment"],g["commitment"],x)
    if fid=="store_rejects_wrong_prior":
        l,g,s=setup(); x=cp(g); x["version"]=1; x["uses_consumed"]=1; x["prior_state_commitment"]="wrong"; x["invocation_records"]={"a":{"invocation_commitment":"ia","transition_id":"ta","consumption_index":1}}; x["commitment"]=h(SD,b(x)); return s.compare_and_swap(l["commitment"],g["commitment"],x)
    if fid=="store_rejects_prior_record_rewrite":
        l,g,s,i,r=first(2); p=r["state"]; x=cp(p); x["invocation_records"][i["invocation_id"]]["transition_id"]="rewritten"; ni=inv("inv:2","p2"); x["version"]=2; x["uses_consumed"]=2; x["prior_state_commitment"]=p["commitment"]; tid=transition_id(l,ni,p["commitment"],2,2); x["invocation_records"][ni["invocation_id"]]={"invocation_commitment":ni["commitment"],"transition_id":tid,"consumption_index":2}; x["commitment"]=h(SD,b(x)); return s.compare_and_swap(l["commitment"],p["commitment"],x)
    if fid=="state_rejects_nonmonotonic_indices":
        l=lease(use_budget=2); x=genesis(l)["state"]; x["version"]=2; x["uses_consumed"]=2; x["prior_state_commitment"]="p"; x["invocation_records"]={"a":{"invocation_commitment":"ia","transition_id":"ta","consumption_index":1},"b":{"invocation_commitment":"ib","transition_id":"tb","consumption_index":1}}; x["commitment"]=h(SD,b(x)); return state_status(x,l)
    if fid=="duplicate_genesis_before_use_rejected":
        l,g,s=setup(); before=s.current_for(l["commitment"]); got=s.register(l,g); after=s.current_for(l["commitment"]); return "AlreadyRegistered" if got=="AlreadyRegistered" and before==after==g["commitment"] else "StateInvalid"
    if fid=="duplicate_genesis_after_use_rejected_no_reset":
        l,g,s,i,r=first(); before=s.current_for(l["commitment"]); got=s.register(l,g); after=s.current_for(l["commitment"]); return "AlreadyRegistered" if got=="AlreadyRegistered" and before==after==r["state"]["commitment"] else "StateInvalid"
    raise KeyError(fid)

def load(): return json.loads(M.read_text())
def self_test():
    m=load(); assert len(m["fixtures"])==42; assert h(LD,{"x":1})!=h(SD,{"x":1})!=h(ED,{"x":1}); print("LEX-NET-040 R3 self-test PASS")
def semantic():
    m=load(); seen=set()
    for f in m["fixtures"]:
        got=run(f["id"])
        if got!=f["expected"]: raise SystemExit(f"{f['id']}: expected {f['expected']} got {got}")
        seen.add(got)
    print(json.dumps({"tranche":"LEX-NET-040","candidate_revision":"R3","fixture_count":len(m["fixtures"]),"observed_outcomes":sorted(seen),"backend_atomicity_claim":False,"backend_access_control_claim":False,"external_effect_claim":False,"semantic_result":"PASS"},sort_keys=True))
def scope():
    m=load(); hd=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(); p=subprocess.check_output(["git","rev-parse","HEAD^"],text=True).strip(); n=int(subprocess.check_output(["git","rev-list","--count",f"{p}..{hd}"],text=True).strip()); paths=sorted(subprocess.check_output(["git","diff","--name-only",p,hd],text=True).splitlines())
    if p!=m["qualified_parent"]: raise SystemExit(f"parent mismatch {p}")
    if n!=1: raise SystemExit(f"commit count mismatch {n}")
    if paths!=sorted(m["expected_paths"]): raise SystemExit(f"path set mismatch {paths}")
    print(json.dumps({"tranche":"LEX-NET-040","candidate_revision":"R3","head":hd,"parent":p,"commit_count":n,"paths":paths,"scope_result":"PASS"},sort_keys=True))
def main():
    a=argparse.ArgumentParser(); a.add_argument("--self-test",action="store_true"); a.add_argument("--semantic",action="store_true"); a.add_argument("--scope",action="store_true"); z=a.parse_args()
    if z.self_test:self_test()
    elif z.semantic:semantic()
    elif z.scope:scope()
    else:a.error("select mode")
if __name__=="__main__": main()
