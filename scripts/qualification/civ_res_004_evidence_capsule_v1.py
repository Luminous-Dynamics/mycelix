#!/usr/bin/env python3
"""Verify the archived CIV-RES-004 evidence capsule binding."""
from __future__ import annotations
import hashlib,json,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
CAP=ROOT/"mycelix-workspace/docs/civic-resilience/civ_res_004_evidence_capsule.json"
FIX=ROOT/"mycelix-workspace/docs/civic-resilience/civ_res_004_option_space_cases.json"
ORC=ROOT/"mycelix-workspace/docs/civic-resilience/civ_res_004_option_space_oracle.json"
def fail(m): raise SystemExit("CIV-RES-004 CAPSULE FAIL: "+m)
def main():
    c=json.loads(CAP.read_text(encoding="utf-8"))
    if c["schema"]!="mycelix.civic-resilience.option-space-evidence-capsule.v1": fail("schema")
    if c["program"]!="CIV-RES-004": fail("program")
    if c["workflow"]["conclusion"]!="success": fail("workflow conclusion")
    if c["qualification"]["case_count"]!=18: fail("case count")
    if c["inputs"]["fixture_sha256"]!=hashlib.sha256(FIX.read_bytes()).hexdigest(): fail("fixture hash mismatch")
    if c["inputs"]["oracle_sha256"]!=hashlib.sha256(ORC.read_bytes()).hexdigest(): fail("oracle hash mismatch")
    if c["evaluator"]["receipt_schema"]!="mycelix.civic-resilience.option-space-qualification-receipt.v1": fail("receipt schema")
    if len(c["evaluator"]["receipt_sha256"])!=64: fail("receipt hash shape")
    if not c["provenance"]["immutable_commit_binding"]: fail("immutable binding disabled")
    if len(c["qualification"]["nonclaims"])!=6: fail("nonclaim coverage")
    print("CIV-RES-004 CAPSULE PASS: archived hashes and qualification binding are internally consistent")
    return 0
if __name__=="__main__": sys.exit(main())
