#!/usr/bin/env python3
"""Independent executable reference evaluator for MOBILITY-COMMONS-018."""
import json
import sys
from pathlib import Path

CORPUS_SCHEMA = "mobility-reconciliation-witness-revision-executable-v1"
VERSION = "mobility-reconciliation-witness-revision-v1"

def identity_ok(x):
    if not isinstance(x, dict) or set(x) != {"kind","namespace","id"}:
        return False
    if not str(x["namespace"]).strip() or not str(x["id"]).strip():
        return False
    if x["namespace"] == "holochain" or str(x["id"]).startswith(("uhC0","uhCE")):
        return False
    return True

def witness_ok(w):
    required={"witness_identity","left_claim","right_claim","left_applicability","right_applicability",
              "comparability","compatibility","explicitly_superseded","disputed","result"}
    if set(w) != required:
        return False
    if not identity_ok(w["witness_identity"]) or w["witness_identity"]["kind"] != "reconciliation_witness":
        return False
    if not identity_ok(w["left_claim"]) or not identity_ok(w["right_claim"]):
        return False
    if w["left_claim"] == w["right_claim"]:
        return False
    for interval in (w["left_applicability"], w["right_applicability"]):
        if set(interval) != {"start","end"}:
            return False
        if interval["start"] is not None and interval["end"] is not None and interval["end"] < interval["start"]:
            return False
    expected = classify(w)
    return w["result"] == expected

def overlap(a,b):
    if a["start"] is None or a["end"] is None or b["start"] is None or b["end"] is None:
        return None
    return not (a["end"] < b["start"] or b["end"] < a["start"])

def classify(w):
    if w["explicitly_superseded"]:
        c="superseded"
    elif w["comparability"] == "incomparable":
        c="incomparable"
    elif w["comparability"] == "unknown" or w["compatibility"] == "unknown":
        c="indeterminate"
    elif w["compatibility"] == "compatible":
        c="coexistent"
    else:
        o=overlap(w["left_applicability"],w["right_applicability"])
        c="indeterminate" if o is None else ("conflicting" if o else "sequential")
    return {"classification":c,"disputed":bool(w["disputed"] and c=="conflicting")}

def supersession_ok(edge, current, previous):
    if not edge or set(edge) != {"relation","source","target"}:
        return False
    return (edge["relation"]=="supersedes" and identity_ok(edge["source"]) and identity_ok(edge["target"])
            and edge["source"] == current["witness_identity"]
            and edge["target"] == previous["witness_identity"]
            and edge["source"]["kind"]=="reconciliation_witness"
            and edge["target"]["kind"]=="reconciliation_witness"
            and edge["source"] != edge["target"])

def evaluate(c):
    p,curr=c["previous"],c["current"]
    op=c["operation"]
    if op in {"transition","supersession_preserves_predecessor"}:
        if not witness_ok(p) or not witness_ok(curr):
            return "rejected"
        if p["witness_identity"] == curr["witness_identity"]:
            return "stable" if p == curr else "rejected"
        if not supersession_ok(c.get("supersession"),curr,p):
            return "rejected"
        return "accepted"
    if op=="projection":
        return "accepted" if witness_ok(curr) and identity_ok(c.get("projection_ref",{})) and c["projection_ref"] == curr["witness_identity"] else "rejected"
    if op=="identity":
        return "accepted" if witness_ok(curr) else "rejected"
    return "rejected"

def main():
    path=Path(sys.argv[1] if len(sys.argv)>1 else "docs/mobility/MOBILITY_RECONCILIATION_WITNESS_REVISION_V1_EXECUTABLE.json")
    d=json.loads(path.read_text())
    if d["schema"] != CORPUS_SCHEMA or d["schema_version"] != VERSION or d["status"] != "semantic-provenance-only":
        raise SystemExit("invalid executable corpus envelope")
    if len(d["cases"]) != 16:
        raise SystemExit("expected 10 executable cases")
    ids=[c["id"] for c in d["cases"]]
    if ids != [f"WRV-{i:03}" for i in range(1,17)]:
        raise SystemExit("unexpected case identifiers")
    out=[]
    for c in d["cases"]:
        actual=evaluate(c)
        if actual != c["expected"]:
            raise SystemExit(f"{c['id']}: expected {c['expected']}, got {actual}")
        out.append({"id":c["id"],"operation":c["operation"],"expected":c["expected"],"actual":actual})
    print(json.dumps({"schema":"mobility-reconciliation-witness-revision-normalized-v1",
                      "schema_version":VERSION,"status":"semantic-provenance-only","cases":out},
                     indent=2, sort_keys=False))

if __name__=="__main__":
    main()
