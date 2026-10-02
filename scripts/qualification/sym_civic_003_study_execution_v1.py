#!/usr/bin/env python3
"""Qualify SYM-CIVIC-003 immutable study-execution evidence v1."""
from __future__ import annotations
import hashlib, json, pathlib, sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_003_study_execution.json"
REQUIRED={"study_id","input_snapshot","upstream_evidence_cut","estimand","model_identity","analysis_commit","environment_identity","execution_identity","configuration_digest","randomness","temporal_evidence","uncertainty","missingness_policy","identification_assumptions","diagnostics","sensitivity_analyses","spillover_displacement_checks","alternative_explanations","output_disposition","result_artifact"}
INVARIANTS={"mutable_reference_rejected","provenance_axes_distinct","canonical_serialization_required","missing_provenance_explicit","determinism_mode_explicit","replication_semantics_explicit","result_digest_binding","protected_data_not_copied","output_disposition_not_authority"}
CASE_IDS=[f"SE-{i:02d}" for i in range(1,15)]
def fail(m): raise SystemExit("SYM-CIVIC-003 FAIL: "+m)
def rejects(c):
    x=c["candidate"]
    cid=c["id"]
    if cid=="SE-01": return x.get("input_snapshot")=="latest"
    if cid=="SE-02": return x.get("analysis_commit")!="commit-a"
    if cid=="SE-03": return x.get("model_identity")!="model-a@sha256:m-a"
    if cid=="SE-04": return x.get("environment_identity")!="sha256:env-a"
    if cid=="SE-05": return not {"mode","seed"} <= set(x.get("randomness",{}))
    if cid=="SE-06": return "configuration_digest" not in x
    if cid=="SE-07": return x.get("result_artifact",{}).get("digest")!="sha256:r-a"
    if cid=="SE-08": return x.get("randomness",{}).get("claim")=="deterministic_replication"
    if cid=="SE-09": return any(k in json.dumps(x).lower() for k in ("analysis_claim","oracle_verdict","candidate_verdict"))
    if cid=="SE-10": return str(x.get("upstream_evidence_cut","")).startswith(("refs/","branch:"))
    if cid=="SE-11": return "protected_record" in x and any(k in x["protected_record"] for k in ("raw_subject_identifier","raw_payload"))
    if cid=="SE-12": return not (x.get("randomness",{}).get("mode")=="stochastic" and x.get("randomness",{}).get("claim")=="distributional_replication")
    if cid=="SE-13":
        r=x.get("equivalent_records",[])
        return len(r)!=2 or json.dumps(r[0],sort_keys=True,separators=(",",":"))!=json.dumps(r[1],sort_keys=True,separators=(",",":"))
    if cid=="SE-14": return not REQUIRED <= set(x)
    return True
def main():
    d=json.loads(MAN.read_text(encoding="utf-8"))
    if d.get("schema")!="mycelix.sym-civic.study-execution-evidence.v1": fail("schema")
    if d.get("program")!="SYM-CIVIC-003" or d.get("analysis_role")!="research_only" or d.get("receipt_role")!="execution_provenance_only": fail("program/role")
    if set(d.get("required_fields",[]))!=REQUIRED: fail("required fields")
    if set(d.get("invariants",[]))!=INVARIANTS: fail("invariants")
    cases=d.get("cases",[])
    if [c.get("id") for c in cases]!=CASE_IDS: fail("case order")
    for c in cases:
        if set(c)!= {"id","family","reference","candidate","note"}: fail(c["id"]+" fixture surface")
        blob=json.dumps(c,sort_keys=True,separators=(",",":")).lower()
        if "analysis_claim" in blob or "oracle_verdict" in blob or "candidate_verdict" in blob: fail(c["id"]+" embedded oracle")
    derived={c["id"]:rejects(c) for c in cases}
    expected_reject={f"SE-{i:02d}" for i in range(1,12)}
    if {k for k,v in derived.items() if v} != expected_reject: fail("derived rejection set")
    if derived["SE-12"] or derived["SE-13"] or derived["SE-14"]: fail("valid cases rejected")
    payload={"program":"SYM-CIVIC-003","schema":d["schema"],"cases":[{"id":c["id"],"rejected":derived[c["id"]]} for c in cases]}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode("utf-8")).hexdigest()
    print(f"SYM-CIVIC-003 PASS: 14 execution-evidence cases, 11 rejection cases, 3 admissible cases, canonical receipt={digest}")
    return 0
if __name__=="__main__": sys.exit(main())
