#!/usr/bin/env python3
import json, sys
from pathlib import Path

VERSION = "mobility-reconciliation-target-bound-projection-v1"
IDS = [f"TBP-{i:03}" for i in range(1, 13)]

def load(path):
    d = json.loads(Path(path).read_text())
    assert set(d) == {"schema", "schema_version", "status", "cases"}
    assert d["schema"] == "mobility-reconciliation-target-bound-projection-normalized-v1"
    assert d["schema_version"] == VERSION
    assert d["status"] == "semantic-provenance-only"
    assert [c["id"] for c in d["cases"]] == IDS
    assert all(set(c) == {"id", "operation", "expected", "actual"} for c in d["cases"])
    return d

a, b = load(sys.argv[1]), load(sys.argv[2])
assert a == b, "normalized outputs differ"
print(json.dumps({"result": "matched", "cases": 12, "scope": "semantic-provenance-only"}))
