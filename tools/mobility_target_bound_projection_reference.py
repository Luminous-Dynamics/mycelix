#!/usr/bin/env python3
import json, sys
from pathlib import Path

SCHEMA = "mobility-reconciliation-target-bound-projection-executable-v1"
VERSION = "mobility-reconciliation-target-bound-projection-v1"
IDS = [f"TBP-{i:03}" for i in range(1, 13)]
STATE_FIELDS = {
    "epistemic_disposition", "lifecycle_disposition", "conflict_disposition",
    "authority_provenance", "evidence_modality", "contradiction_reference",
    "conflict_reference", "unresolved_dependency_reference",
    "external_authority_reference",
}

def identity_ok(x):
    return (
        isinstance(x, dict)
        and set(x) == {"kind", "namespace", "id"}
        and bool(str(x["namespace"]).strip())
        and bool(str(x["id"]).strip())
        and x["namespace"] != "holochain"
        and not str(x["id"]).startswith(("uhC0", "uhCE"))
    )

def witness_ok(w):
    if not isinstance(w, dict) or not identity_ok(w.get("witness_identity")):
        return False
    if w["witness_identity"]["kind"] != "reconciliation_witness":
        return False
    if not identity_ok(w.get("left_claim")) or not identity_ok(w.get("right_claim")):
        return False
    if w["left_claim"] == w["right_claim"]:
        return False
    if w["left_claim"]["kind"] != "evidence_record" or w["right_claim"]["kind"] != "evidence_record":
        return False
    a, b = w.get("left_applicability"), w.get("right_applicability")
    if not isinstance(a, dict) or not isinstance(b, dict):
        return False
    if a["end"] < a["start"] or b["end"] < b["start"]:
        return False
    if w["explicitly_superseded"]:
        cls = "superseded"
    elif w["compatibility"] == "compatible":
        cls = "coexistent"
    else:
        overlap = not (b["end"] < a["start"] or a["end"] < b["start"])
        cls = "conflicting" if overlap else "sequential"
    return w["result"] == {
        "classification": cls,
        "disputed": bool(w["disputed"] and cls == "conflicting"),
    }

def project(c):
    target, w, wrapper, base = c["target"], c["witness"], c["projection"], c["base_state"]
    if not identity_ok(target) or target["kind"] != "evidence_record":
        return None
    if not witness_ok(w):
        return None
    if not isinstance(wrapper, dict) or set(wrapper) != {"target", "projection"}:
        return None
    if wrapper["target"] != target:
        return None
    if target in (w["witness_identity"], w["left_claim"], w["right_claim"]):
        return None
    p = wrapper["projection"]
    if not isinstance(p, dict) or set(p) != {"ConflictReferenceOnly"}:
        return None
    body = p["ConflictReferenceOnly"]
    if not isinstance(body, dict) or set(body) != {"witness_ref"}:
        return None
    if body["witness_ref"] != w["witness_identity"]:
        return None
    if w["result"]["classification"] != "conflicting":
        return None
    if not isinstance(base, dict) or set(base) != STATE_FIELDS:
        return None
    out = dict(base)
    out["conflict_reference"] = w["witness_identity"]["id"]
    out["conflict_disposition"] = "disputed" if w["result"]["disputed"] else "uncontested"
    return out

def evaluate(c):
    return "accepted" if project(c) == c["expected_state"] else "rejected"

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "docs/mobility/MOBILITY_RECONCILIATION_TARGET_BOUND_PROJECTION_V1_EXECUTABLE.json"
    d = json.loads(Path(path).read_text())
    if d.get("schema") != SCHEMA or d.get("schema_version") != VERSION or d.get("status") != "semantic-provenance-only":
        raise SystemExit("invalid corpus envelope")
    if len(d.get("cases", [])) != 12 or [c["id"] for c in d["cases"]] != IDS:
        raise SystemExit("unexpected corpus IDs/count")
    out = []
    for c in d["cases"]:
        actual = evaluate(c)
        if actual != c["expected"]:
            raise SystemExit(f"{c['id']}: {actual} != {c['expected']}")
        out.append({"id": c["id"], "operation": c["operation"], "expected": c["expected"], "actual": actual})
    print(json.dumps({
        "schema": "mobility-reconciliation-target-bound-projection-normalized-v1",
        "schema_version": VERSION,
        "status": "semantic-provenance-only",
        "cases": out,
    }, indent=2))

if __name__ == "__main__":
    main()
