#!/usr/bin/env python3
import json, sys
from pathlib import Path

KEYS={"schema","schema_version","status","cases"}
CASE_KEYS={"id","operation","expected","actual"}
VERSION="mobility-reconciliation-witness-revision-v1"
IDS=[f"WRV-{i:03}" for i in range(1,17)]

def load(p):
    d=json.loads(Path(p).read_text())
    if set(d)!=KEYS or d["schema"]!="mobility-reconciliation-witness-revision-normalized-v1" or d["schema_version"]!=VERSION or d["status"]!="semantic-provenance-only":
        raise ValueError(f"{p}: invalid normalized envelope")
    cases=d["cases"]
    if not isinstance(cases,list) or len(cases)!=16: raise ValueError(f"{p}: expected 10 cases")
    for c in cases:
        if set(c)!=CASE_KEYS or not all(isinstance(c[k],str) for k in CASE_KEYS): raise ValueError(f"{p}: malformed case")
    if [c["id"] for c in cases]!=IDS: raise ValueError(f"{p}: noncanonical IDs")
    return d

if __name__=="__main__":
    if len(sys.argv)!=3: raise SystemExit("usage: compare... RUST.json PYTHON.json")
    a,b=load(sys.argv[1]),load(sys.argv[2])
    if a!=b:
        raise SystemExit("normalized evaluator outputs differ")
    print(json.dumps({"result":"matched","cases":16,"scope":"semantic-provenance-only"}))
