#!/usr/bin/env python3
import json,sys
from pathlib import Path
KEYS={"schema","schema_version","status","cases"}
CASE_KEYS={"id","operation","expected","actual"}
VERSION="mobility-reconciliation-witness-history-v1"
IDS=[f"WHP-{i:03}" for i in range(1,13)]
def load(p):
    d=json.loads(Path(p).read_text())
    if set(d)!=KEYS or d.get("schema")!="mobility-reconciliation-witness-history-normalized-v1" or d.get("schema_version")!=VERSION or d.get("status")!="semantic-provenance-only": raise ValueError(f"{p}: invalid envelope")
    if not isinstance(d["cases"],list) or len(d["cases"])!=12: raise ValueError(f"{p}: expected 12 cases")
    for c in d["cases"]:
        if set(c)!=CASE_KEYS or any(not isinstance(c[k],str) for k in CASE_KEYS): raise ValueError(f"{p}: malformed case")
    if [c["id"] for c in d["cases"]]!=IDS: raise ValueError(f"{p}: noncanonical IDs")
    return d
if __name__=="__main__":
    if len(sys.argv)!=3: raise SystemExit("usage: compare... RUST.json PYTHON.json")
    a,b=load(sys.argv[1]),load(sys.argv[2])
    if a!=b: raise SystemExit("normalized evaluator outputs differ")
    print(json.dumps({"result":"matched","cases":12,"scope":"semantic-provenance-only"}))
