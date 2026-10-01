#!/usr/bin/env python3
import json,sys
from pathlib import Path
SCHEMA="mobility-reconciliation-historical-projection-state-executable-v1"; VERSION="mobility-reconciliation-historical-projection-state-v1"; IDS=[f"HPS-{i:03}" for i in range(1,9)]
def identity_ok(x):
 return isinstance(x,dict) and set(x)=={"kind","namespace","id"} and x["kind"]=="reconciliation_witness" and x["namespace"]!="holochain" and bool(x["id"]) and not x["id"].startswith(("uhC0","uhCE"))
def witness_ok(w):
 if not isinstance(w,dict) or not identity_ok(w.get("witness_identity")): return False
 if w.get("left_claim")==w.get("right_claim"): return False
 a,b=w.get("left_applicability"),w.get("right_applicability")
 if not a or not b or a["end"]<a["start"] or b["end"]<b["start"]: return False
 if w["explicitly_superseded"]: cls="superseded"
 elif w["compatibility"]=="compatible": cls="coexistent"
 else:
  overlap=not(b["end"]<a["start"] or a["end"]<b["start"]); cls="conflicting" if overlap else "sequential"
 return w["result"]=={"classification":cls,"disputed":bool(w["disputed"] and cls=="conflicting")}
def project(c):
 w,p,b=c["witness"],c["projection"],c["base_state"]
 if not witness_ok(w) or not isinstance(p,dict) or not isinstance(b,dict): return None
 if set(b)!={"epistemic_disposition","lifecycle_disposition","conflict_disposition","authority_provenance","evidence_modality","contradiction_reference","conflict_reference","unresolved_dependency_reference","external_authority_reference"}: return None
 kind=next(iter(p),None); body=p.get(kind,{})
 kind=kind.removesuffix("Only") if False else kind
 if body.get("witness_ref")!=w["witness_identity"]: return None
 out=dict(b)
 if kind=="ConflictReferenceOnly" and w["result"]["classification"]=="conflicting":
  out["conflict_reference"]=w["witness_identity"]["id"]; out["conflict_disposition"]="disputed" if w["result"]["disputed"] else "uncontested"
 elif kind=="LifecycleSupersession" and w["result"]["classification"]=="superseded": out["lifecycle_disposition"]="superseded"
 else: return None
 return out
def evaluate(c):
 if c.get("required_historical_ref") and c["projection"].get(next(iter(c["projection"]),""),{}).get("witness_ref")!=c["required_historical_ref"]: return "rejected"
 return "accepted" if project(c)==c["expected_state"] else "rejected"
def main():
 d=json.loads(Path(sys.argv[1] if len(sys.argv)>1 else "docs/mobility/MOBILITY_RECONCILIATION_HISTORICAL_PROJECTION_STATE_V1_EXECUTABLE.json").read_text())
 if d.get("schema")!=SCHEMA or d.get("schema_version")!=VERSION or d.get("status")!="semantic-provenance-only" or len(d.get("cases",[]))!=8: raise SystemExit("invalid corpus")
 if [c["id"] for c in d["cases"]]!=IDS: raise SystemExit("unexpected IDs")
 out=[]
 for c in d["cases"]:
  actual=evaluate(c)
  if actual!=c["expected"]: raise SystemExit(f'{c["id"]}: {actual} != {c["expected"]}')
  out.append({"id":c["id"],"operation":c["operation"],"expected":c["expected"],"actual":actual})
 print(json.dumps({"schema":"mobility-reconciliation-historical-projection-state-normalized-v1","schema_version":VERSION,"status":"semantic-provenance-only","cases":out},indent=2))
if __name__=="__main__": main()
