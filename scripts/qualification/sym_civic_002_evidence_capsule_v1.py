#!/usr/bin/env python3
"""Verify the archived SYM-CIVIC-002 evidence capsule."""
from __future__ import annotations
import hashlib, json, pathlib, sys

ROOT=pathlib.Path(__file__).resolve().parents[2]
CAP=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_002_evidence_capsule.json"
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_002_benchmark.json"
SCRIPT=ROOT/"scripts/qualification/sym_civic_002_benchmark_v1.py"
WF=ROOT/".github/workflows/sym-civic-002-falsification.yml"

def fail(msg: str) -> None:
    raise SystemExit("SYM-CIVIC-002 CAPSULE FAIL: "+msg)

def main() -> int:
    c=json.loads(CAP.read_text(encoding="utf-8"))
    if c["schema"]!="mycelix.sym-civic.causal-falsification-evidence-capsule.v1": fail("schema")
    if c["program"]!="SYM-CIVIC-002": fail("program")
    if c["qualifier_commit"]!="49943c6e2f70fd7603c0af4f8a079981fde32c0c": fail("qualifier commit")
    if c["qualifier_parent"]!="424e9342fcea47f982d3c5b9f43ad9a4cce4f1a0": fail("qualifier parent")
    if c["workflow"]["run_id"]!=37048493418 or c["workflow"]["job_id"]!=110975732621: fail("workflow binding")
    if c["workflow"]["conclusion"]!="success": fail("workflow conclusion")
    if c["qualification"]["case_count"]!=10: fail("case count")
    if c["qualification"]["candidate_input_policy"]!="oracle_excluded": fail("candidate input policy")
    expected_cases=[f"CF-{i:02d}" for i in range(1,11)]
    if c["qualification"]["counterexample_cases"]!=expected_cases: fail("case ids")
    expected_threats={"positivity_overlap","time_varying_confounding_feedback","network_interference_spillover","measurement_error_misclassification"}
    if set(c["qualification"]["required_causal_threat_checks"])!=expected_threats: fail("threat contract")
    if not c["provenance"]["immutable_commit_binding"]: fail("immutable binding disabled")
    if c["provenance"]["protected_data_in_receipt"]: fail("protected-data flag")
    objs=c["git_objects"]
    if objs["manifest_blob_sha"]!="a1c9641f32f7ebade9c06a1ae1cb2f3c1827443e": fail("manifest blob")
    if objs["qualifier_script_blob_sha"]!="33c60ad4713ef6a30473b8f1ca1a5ed399aeafd9": fail("qualifier blob")
    if objs["workflow_blob_sha"]!="2e527e40d2cebb241b7dc4a4b4250dca7ee1fbb9": fail("workflow blob")
    def git_blob_sha(path: pathlib.Path) -> str:
        data=path.read_bytes()
        return hashlib.sha1(b"blob "+str(len(data)).encode("ascii")+b"\0"+data).hexdigest()
    if git_blob_sha(MAN)!=objs["manifest_blob_sha"]: fail("manifest content/blob mismatch")
    if git_blob_sha(SCRIPT)!=objs["qualifier_script_blob_sha"]: fail("qualifier content/blob mismatch")
    if git_blob_sha(WF)!=objs["workflow_blob_sha"]: fail("workflow content/blob mismatch")
    payload=c["receipt"]["payload"]
    canonical=json.dumps(payload,sort_keys=True,separators=(",",":")).encode("utf-8")
    if hashlib.sha256(canonical).hexdigest()!=c["receipt"]["sha256"]: fail("receipt digest")
    if c["receipt"]["sha256"]!="475cc62b69f6b391c6b716096265fc7d423f35e0677063b2f8b4f8497200dea0": fail("receipt identity")
    print("SYM-CIVIC-002 CAPSULE PASS: qualification binding, Git object identities, causal-threat contract, and canonical receipt are internally consistent")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
