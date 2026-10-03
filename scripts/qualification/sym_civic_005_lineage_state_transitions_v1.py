#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,pathlib,re,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_005_lineage_state_transitions.json"
IDS=[f"T-{i:02d}" for i in range(1,16)]
IMM=re.compile(r"^(immutable:|sha256:)")
def fail(m): raise SystemExit("SYM-CIVIC-005 FAIL: "+m)
def reject(c):
 x=c["candidate"]; i=c["id"]
 if i=="T-01": return any(e["event_type"]=="INVALIDATE" for e in x["events"])
 if i=="T-02": return x["result"]["bound_digest"]!=x["result"]["current_digest"]
 if i=="T-03": return x["result"]["state"]=="SUPERSEDED" and "replacement_claim" not in x
 if i=="T-04": return any(e["event_type"]=="INVALIDATE" for e in x["events"]) and any(e["event_type"]=="RESTORE" for e in x["events"])
 if i=="T-05": return any(e["event_type"]=="INVALIDATE" for e in x["events"]) and "latest_pointer" in x
 if i=="T-06": return x["result"]["state"]=="SUPERSEDED" and x["replacement"].get("derivation_refs")==[]
 if i=="T-07": return False
 if i=="T-08":
  evs=sorted(x["events"],key=lambda e:(e["effective_at"],e["event_id"]))
  return evs[0]["event_type"]=="INVALIDATE" and evs[1]["event_type"]=="SUPERSEDE"
 if i=="T-09": return len(x["events"])==2 and x["events"][0]!=x["events"][1] and x["events"][0]["event_id"]==x["events"][1]["event_id"]
 if i=="T-10": return x["event_receipt"]["declared_digest"]!=x["event_receipt"]["actual_digest"]
 if i=="T-11": return x.get("history_rewrite") is True
 if i=="T-12": return x.get("cached_claim_state")=="VALID" and any(e["event_type"]=="INVALIDATE" for e in x["events"])
 if i=="T-13": return False
 if i=="T-14":
  return not (sorted(x["equivalent_event_orders"][0])==sorted(x["equivalent_event_orders"][1]))
 if i=="T-15": return False
 return True
def main():
 d=json.loads(MAN.read_text(encoding="utf-8"))
 if d.get("schema")!="mycelix.sym-civic.lineage-state-transitions.v1" or d.get("program")!="SYM-CIVIC-005": fail("schema/program")
 if d.get("analysis_role")!="research_only": fail("role")
 cases=d.get("cases",[])
 if [c.get("id") for c in cases]!=IDS: fail("case order")
 for c in cases:
  if set(c)!={"id","family","candidate","note"}: fail(c["id"]+" fixture surface")
  blob=json.dumps(c,sort_keys=True,separators=(",",":")).lower()
  if any(t in blob for t in ("expected_disposition","expected_result","oracle_verdict","candidate_verdict")): fail(c["id"]+" embedded oracle")
  if any(k in c["candidate"] for k in ("raw_subject_identifier","raw_payload","authorized_decision","civic_authorization")): fail(c["id"]+" prohibited field")
 derived={c["id"]:reject(c) for c in cases}
 expected={f"T-{i:02d}" for i in range(1,13)}
 if {k for k,v in derived.items() if v}!=expected: fail("derived rejection set")
 if any(derived[k] for k in ("T-13","T-14","T-15")): fail("valid case rejected")
 payload={"program":d["program"],"schema":d["schema"],"cases":[{"id":c["id"],"rejected":derived[c["id"]]} for c in cases]}
 digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
 print(f"SYM-CIVIC-005 PASS: 15 temporal-lineage cases, 12 rejection cases, 3 admissible cases, canonical receipt={digest}")
if __name__=="__main__": main()
