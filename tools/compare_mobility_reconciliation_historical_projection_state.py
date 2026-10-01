#!/usr/bin/env python3
import json,sys
from pathlib import Path
V="mobility-reconciliation-historical-projection-state-v1"; IDS=[f"HPS-{i:03}" for i in range(1,9)]
def load(p):
 d=json.loads(Path(p).read_text()); assert set(d)=={"schema","schema_version","status","cases"} and d["schema"]=="mobility-reconciliation-historical-projection-state-normalized-v1" and d["schema_version"]==V and d["status"]=="semantic-provenance-only" and [x["id"] for x in d["cases"]]==IDS
 assert all(set(x)=={"id","operation","expected","actual"} for x in d["cases"]); return d
a,b=load(sys.argv[1]),load(sys.argv[2]); assert a==b,"normalized outputs differ"; print(json.dumps({"result":"matched","cases":8,"scope":"semantic-provenance-only"}))
