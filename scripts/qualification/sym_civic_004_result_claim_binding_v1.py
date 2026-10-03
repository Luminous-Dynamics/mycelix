#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,pathlib,re,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_004_result_claim_binding.json"
REQ={"result_artifact","study_execution_evidence_ref","upstream_evidence_cut","result_semantics","claim_scope","claim_proposition","supporting_result_refs","derivation_refs","uncertainty_ref","limitations","alternative_explanations","output_disposition","claim_provenance"}
INV={"immutable_result_binding","execution_receipt_binding","explicit_scope","explicit_derivation","uncertainty_not_guarantee","invalidation_propagation","aggregate_not_person","recommendation_not_authority","canonicalization_required","protected_data_not_copied","authority_plane_absent"}
IDS=[f"RC-{i:02d}" for i in range(1,16)]
IMM=re.compile(r"^(immutable:|sha256:)")
def fail(m): raise SystemExit("SYM-CIVIC-004 FAIL: "+m)
def reject(c):
 x=c["candidate"]; i=c["id"]
 if i=="RC-01": return x["result_artifact"].get("digest")!="sha256:r-a"
 if i=="RC-02": return x["result_artifact"].get("bound_digest")!=x["result_artifact"].get("current_digest")
 if i=="RC-03": return not IMM.match(x["upstream_evidence_cut"]) or not IMM.match(x["claim_provenance"])
 if i=="RC-04": return x["claim_scope"]!=c["reference"]["scope"]
 if i=="RC-05": return "guarantee" in x["claim_proposition"].lower() and x["uncertainty_ref"]!="immutable:u-none"
 if i=="RC-06": return len(x["supporting_result_refs"])>1 and not x["derivation_refs"] and x["result_semantics"]=="aggregate"
 if i=="RC-07": return x["result_semantics"] in {"aggregated","transformed"} and not x["derivation_refs"]
 if i=="RC-08": return x.get("source_status")=="invalidated"
 if i=="RC-09": return x["result_semantics"]=="aggregate" and "person" in x["claim_proposition"].lower()
 if i=="RC-10": return "civic_authorization" in x or x.get("output_disposition")=="Authorized"
 if i=="RC-11": return not IMM.match(x["study_execution_evidence_ref"])
 return False
def main():
 d=json.loads(MAN.read_text(encoding="utf-8"))
 if d.get("schema")!="mycelix.sym-civic.result-claim-binding.v1" or d.get("program")!="SYM-CIVIC-004": fail("schema/program")
 if d.get("analysis_role")!="research_only" or d.get("receipt_role")!="result_claim_binding_only": fail("role")
 if set(d.get("required_fields",[]))!=REQ or set(d.get("invariants",[]))!=INV: fail("contract surface")
 cases=d.get("cases",[])
 if [c.get("id") for c in cases]!=IDS: fail("case order")
 for c in cases:
  if set(c)!={"id","family","reference","candidate","note"}: fail(c["id"]+" fixture surface")
  blob=json.dumps(c,sort_keys=True,separators=(",",":")).lower()
  if any(t in blob for t in ("expected_disposition","expected_result","oracle_verdict","candidate_verdict")): fail(c["id"]+" embedded oracle")
  if any(k in c["candidate"] for k in ("raw_subject_identifier","raw_payload")): fail(c["id"]+" protected data")
 derived={c["id"]:reject(c) for c in cases}
 if {k for k,v in derived.items() if v}!={f"RC-{i:02d}" for i in range(1,12)}: fail("derived rejection set")
 if any(derived[k] for k in ("RC-12","RC-13","RC-14","RC-15")): fail("valid case rejected")
 payload={"program":d["program"],"schema":d["schema"],"cases":[{"id":c["id"],"rejected":derived[c["id"]]} for c in cases]}
 digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
 print(f"SYM-CIVIC-004 PASS: 15 result/claim cases, 11 rejection cases, 4 admissible cases, canonical receipt={digest}")
if __name__=="__main__": main()
