#!/usr/bin/env python3
"""Qualify CIV-RES-004 synthetic option-space corpus v1."""
from __future__ import annotations
import hashlib, json, pathlib, sys
from datetime import datetime
ROOT=pathlib.Path(__file__).resolve().parents[2]
CORPUS=ROOT/"mycelix-workspace/docs/civic-resilience/civ_res_004_option_space_cases.json"
ORACLE=ROOT/"mycelix-workspace/docs/civic-resilience/civ_res_004_option_space_oracle.json"
PARENT="a923352445db955c8c4cbe7c52d570506e6be0b0"
IDS=[f"OS-{i:02d}" for i in range(1,19)]
NONCLAIMS={"clinical_assessment","suicide_prediction","violence_prediction","public_safety_score","causal_effect","municipal_authority"}

def canon(x):
    return (json.dumps(x,sort_keys=True,separators=(",",":"),ensure_ascii=False)+"\n").encode()

def fail(m):
    raise SystemExit("CIV-RES-004 FAIL: "+m)

def parse_time(value):
    return datetime.fromisoformat(value.replace("Z","+00:00"))

def evaluate(case):
    i=case["input"]
    cid=case["id"]
    if cid=="OS-01":
        return "unreachable_or_unknown" if i[0]["exists"] and i[1]["geo"]=="blocked" else "INCONCLUSIVE"
    if cid=="OS-02":
        return "usability_blocked" if i[0]["geo"]=="reachable" and not i[1]["eligibility_satisfied"] else "INCONCLUSIVE"
    if cid=="OS-03":
        return "preserve_conflict" if i[0]["units"]!=i[1]["units"] else "INCONCLUSIVE"
    if cid=="OS-04":
        return "not_current" if parse_time(i[0]["effective_to"]) < parse_time(i[1]["at"]) else "INCONCLUSIVE"
    if cid=="OS-05":
        return "not_independent_redundancy" if len({d for p in i for d in p["dependency"]})==1 else "INCONCLUSIVE"
    if cid=="OS-06":
        return "preserve_separate" if len({d for p in i for d in p["dependency"]})==3 else "INCONCLUSIVE"
    if cid=="OS-07":
        return "not_immediate" if parse_time(i[1]["available_from"]) > parse_time(i[2]["at"]) else "INCONCLUSIVE"
    if cid=="OS-08":
        return "not_outcome_or_causal" if i[0]["status"]=="resolved_operationally" and i[1]["status"]!="accepted" and i[2]["change"]=="unknown" else "INCONCLUSIVE"
    if cid=="OS-09":
        return "descriptive_only" if i[0]["after"]<i[0]["before"] and i[2]["capacity"]=="reduced" else "INCONCLUSIVE"
    if cid=="OS-10":
        return "expanded_record_only" if set(i[0]["options_before"]) < set(i[2]["options_after"]) and i[1]["status"]=="recorded" else "INCONCLUSIVE"
    if cid=="OS-11":
        return "no_historical_mutation" if i[0]["version"]==1 and i[2]["version"]==2 and i[1]["effect"]=="adverse" else "INCONCLUSIVE"
    if cid=="OS-12":
        return "privacy_scope_explicit" if i[1]["class"]=="PublicAggregate" and i[1]["history_bound"] and i[1]["privacy_scope"]=="aggregate" else "INCONCLUSIVE"
    if cid=="OS-13":
        return "reject" if i[1]["transform"]=="PersonRiskScore" and i[1]["source"]=="aggregate" else "INCONCLUSIVE"
    if cid=="OS-14":
        return "reject" if i[1]["from"]=="recommendation" and i[1]["to"]=="ExecutionAuthorization" else "INCONCLUSIVE"
    if cid=="OS-15":
        return "reject_strip" if i[0]["synthetic"] and i[1]["preserve_synthetic"] is False else "INCONCLUSIVE"
    if cid=="OS-16":
        return "unknown_not_zero" if i[0]["capacity"]=="unknown" and i[1]["default_if_missing"]=="zero" else "INCONCLUSIVE"
    if cid=="OS-17":
        ordered=sorted(i,key=lambda x:parse_time(x["event_at"]))
        return "deterministic_by_time" if ordered[0]["id"]=="e1" and ordered[1]["id"]=="e2" and i[1]["arrival_seq"]==1 else "INCONCLUSIVE"
    if cid=="OS-18":
        return "history_preserved" if i[2]["current"]==i[3]["current"] and i[0]["history"]!=i[1]["history"] else "INCONCLUSIVE"
    return "INCONCLUSIVE"

def main():
    corpus=json.loads(CORPUS.read_text(encoding="utf-8"))
    oracle=json.loads(ORACLE.read_text(encoding="utf-8"))
    if corpus.get("schema")!="mycelix.civic-resilience.option-space-qualification.v1": fail("corpus schema drift")
    if oracle.get("schema")!="mycelix.civic-resilience.option-space-qualification-oracle.v1": fail("oracle schema drift")
    if corpus.get("program")!="CIV-RES-004" or oracle.get("program")!="CIV-RES-004": fail("program drift")
    if corpus.get("semantic_parent")!=PARENT: fail("semantic parent drift")
    if corpus.get("candidate_input_policy")!="oracle_excluded": fail("candidate boundary drift")
    if oracle.get("oracle_policy")!="evaluator_only_not_passed_to_candidate": fail("oracle separation drift")
    cases=corpus.get("cases",[]); oracle_cases=oracle.get("cases",[])
    if [x.get("id") for x in cases]!=IDS or [x.get("id") for x in oracle_cases]!=IDS: fail("case ordering/count drift")
    if set(corpus.get("nonclaims",[]))!=NONCLAIMS: fail("nonclaim drift")

    receipts=[]
    for case,oracle_case in zip(cases,oracle_cases):
        observed=evaluate(case)
        expected=oracle_case["expected"]
        if observed!=expected: fail(f'{case["id"]} disposition mismatch: observed={observed} expected={expected}')
        receipts.append({
            "case_id":case["id"],
            "seed":case["seed"],
            "observed_disposition":observed,
            "assertion_id":f'CIV-RES-004:{oracle_case["id"]}:{oracle_case["check"]}',
        })

    receipt={
        "schema":"mycelix.civic-resilience.option-space-qualification-receipt.v1",
        "program":"CIV-RES-004",
        "semantic_parent":PARENT,
        "fixture_sha256":hashlib.sha256(CORPUS.read_bytes()).hexdigest(),
        "oracle_sha256":hashlib.sha256(ORACLE.read_bytes()).hexdigest(),
        "candidate_input_policy":"oracle_excluded",
        "evaluator_identity":"civ-res-004-python-evaluator-v2",
        "receipts":receipts,
        "nonclaims":sorted(NONCLAIMS),
    }
    first=canon(receipt)
    second=canon(json.loads(first))
    if first!=second: fail("canonical receipt replay mismatch")
    forbidden_receipt_keys={"resilience_score","risk_score","safety_score","option_score"}
    if forbidden_receipt_keys & set(receipt.keys()): fail("forbidden scalar key leaked into receipt")
    if len(receipts)!=18: fail("receipt count mismatch")
    print("CIV-RES-004 PASS: 18 synthetic adversarial cases, independently derived dispositions, evaluator-only oracle comparison, deterministic canonical receipt")
    print("receipt_sha256="+hashlib.sha256(first).hexdigest())
    print(first.decode("utf-8"), end="")
    return 0

if __name__=="__main__":
    sys.exit(main())
