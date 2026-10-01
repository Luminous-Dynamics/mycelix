#!/usr/bin/env python3
"""Independent executable reference evaluator for MOBILITY-COMMONS-019."""
import json, sys
from pathlib import Path

SCHEMA="mobility-reconciliation-witness-history-executable-v1"
VERSION="mobility-reconciliation-witness-history-v1"
IDS=[f"WHP-{i:03}" for i in range(1,13)]

def identity_ok(x):
    return isinstance(x,dict) and set(x)=={"kind","namespace","id"} and bool(str(x["namespace"]).strip()) and bool(str(x["id"]).strip()) and x["namespace"]!="holochain" and not str(x["id"]).startswith(("uhC0","uhCE"))

def witness_ok(w):
    req={"witness_identity","left_claim","right_claim","left_applicability","right_applicability","comparability","compatibility","explicitly_superseded","disputed","result"}
    if not isinstance(w,dict) or set(w)!=req: return False
    if not identity_ok(w["witness_identity"]) or w["witness_identity"]["kind"]!="reconciliation_witness": return False
    if not identity_ok(w["left_claim"]) or not identity_ok(w["right_claim"]) or w["left_claim"]==w["right_claim"]: return False
    for iv in (w["left_applicability"],w["right_applicability"]):
        if set(iv)!={"start","end"}: return False
        if iv["start"] is not None and iv["end"] is not None and iv["end"]<iv["start"]: return False
    if w["comparability"]=="incomparable": c="incomparable"
    elif w["explicitly_superseded"]: c="superseded"
    elif w["comparability"]=="unknown" or w["compatibility"]=="unknown": c="indeterminate"
    elif w["compatibility"]=="compatible": c="coexistent"
    else:
        a,b=w["left_applicability"],w["right_applicability"]
        overlap=None if None in (a["start"],a["end"],b["start"],b["end"]) else not(a["end"]<b["start"] or b["end"]<a["start"])
        c="indeterminate" if overlap is None else ("conflicting" if overlap else "sequential")
    return w["result"]=={"classification":c,"disputed":bool(w["disputed"] and c=="conflicting")}

def supersession_ok(edge,current,previous):
    return bool(edge and set(edge)=={"relation","source","target"} and edge["relation"]=="supersedes"
        and identity_ok(edge["source"]) and identity_ok(edge["target"])
        and edge["source"]==current["witness_identity"] and edge["target"]==previous["witness_identity"]
        and edge["source"]["kind"]=="reconciliation_witness" and edge["target"]["kind"]=="reconciliation_witness"
        and edge["source"]!=edge["target"])

def projection_ok(ref,witness):
    return identity_ok(ref) and ref["kind"]=="reconciliation_witness" and witness_ok(witness) and ref==witness["witness_identity"]

def transition_ok(previous,current,edge):
    if not witness_ok(previous) or not witness_ok(current): return False
    if previous["witness_identity"]==current["witness_identity"]: return previous==current
    return supersession_ok(edge,current,previous)

def evaluate(c):
    if c["operation"]!="history": return "rejected"
    return "accepted" if transition_ok(c["previous"],c["current"],c.get("supersession")) and projection_ok(c.get("predecessor_projection_ref"),c["previous"]) and projection_ok(c.get("successor_projection_ref"),c["current"]) else "rejected"

def validate_chain(d):
    chain=d.get("chain",[]); refs=d.get("chain_projection_refs",[])
    if len(chain)!=3 or len(refs)!=3: return False
    if not all(witness_ok(w) for w in chain): return False
    if not all(projection_ok(r,w) for r,w in zip(refs,chain)): return False
    return all(transition_ok(chain[i],chain[i+1],{"relation":"supersedes","source":chain[i+1]["witness_identity"],"target":chain[i]["witness_identity"]}) for i in range(2))

def main():
    path=Path(sys.argv[1] if len(sys.argv)>1 else "docs/mobility/MOBILITY_RECONCILIATION_WITNESS_HISTORY_V1_EXECUTABLE.json")
    d=json.loads(path.read_text())
    if d.get("schema")!=SCHEMA or d.get("schema_version")!=VERSION or d.get("status")!="semantic-provenance-only": raise SystemExit("invalid corpus envelope")
    if len(d.get("cases",[]))!=12 or [c["id"] for c in d["cases"]]!=IDS: raise SystemExit("unexpected case set")
    if not validate_chain(d): raise SystemExit("three-generation chain failed")
    out=[]
    for c in d["cases"]:
        actual=evaluate(c)
        if actual!=c["expected"]: raise SystemExit(f'{c["id"]}: expected {c["expected"]}, got {actual}')
        out.append({"id":c["id"],"operation":c["operation"],"expected":c["expected"],"actual":actual})
    print(json.dumps({"schema":"mobility-reconciliation-witness-history-normalized-v1","schema_version":VERSION,"status":"semantic-provenance-only","cases":out},indent=2))

if __name__=="__main__": main()
