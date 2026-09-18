#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,sys
from pathlib import Path

CAPSULE_DOMAIN=b"MYCELIX_QUALIFICATION_CAPSULE_V1\0"
RECEIPT_DOMAIN=b"MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V2\0"
EMPTY_SHA256=hashlib.sha256(b"").hexdigest()

def canonical_json(v):
    def e(x):
        if x is None:return "null"
        if x is True:return "true"
        if x is False:return "false"
        if isinstance(x,int) and not isinstance(x,bool):return str(x)
        if isinstance(x,float):raise ValueError("floats forbidden")
        if isinstance(x,str):return json.dumps(x,ensure_ascii=False,separators=(",",":"))
        if isinstance(x,list):return "["+",".join(e(i) for i in x)+"]"
        if isinstance(x,dict):
            if not all(isinstance(k,str) for k in x):raise ValueError("non-string key")
            ks=sorted(x,key=lambda k:k.encode("utf-8"))
            return "{"+",".join(json.dumps(k,ensure_ascii=False,separators=(",",":"))+":"+e(x[k]) for k in ks)+"}"
        raise ValueError("unsupported canonical type")
    return e(v).encode("utf-8")

def commitment(domain,v):
    b=canonical_json(v)
    return hashlib.sha256(domain+len(b).to_bytes(8,"big")+b).hexdigest()

def validate_result(r):
    if set(r)!={"id","status","output_sha256","exit_code"}:raise ValueError("result keys")
    if not isinstance(r["id"],str) or not r["id"]:raise ValueError("result id")
    if not isinstance(r["output_sha256"],str) or len(r["output_sha256"])!=64:raise ValueError("output hash")
    s=r["status"]; c=r["exit_code"]
    if s=="GateNotRun":
        if c is not None or r["output_sha256"]!=EMPTY_SHA256:raise ValueError("bad GateNotRun")
        return
    if not isinstance(c,int) or isinstance(c,bool):raise ValueError("bad exit")
    expected="GatePass" if c==0 else ("GateFail" if c==10 else "RunnerInfrastructureFailure")
    if s!=expected:raise ValueError("status/exit mismatch")

def attempt_verdict(results):
    infra=False; fail=False
    for r in results:
        validate_result(r)
        s=r["status"]
        if infra:
            if s!="GateNotRun":raise ValueError("only GateNotRun may follow runner failure")
            continue
        if s=="GateNotRun":raise ValueError("GateNotRun before runner failure")
        if s=="RunnerInfrastructureFailure":infra=True
        elif s=="GateFail":fail=True
    return "RunnerInfrastructureFailure" if infra else ("CompletedConjunctiveFail" if fail else "CompletedConjunctivePass")

def compose_receipt(m,ctx,attempt_id,results):
    verdict=attempt_verdict(results)
    body={
        "receipt_format_revision":2,
        "capsule_commitment":commitment(CAPSULE_DOMAIN,m),
        "theorem_id":m["theorem_id"],
        "theorem_revision":m["theorem_revision"],
        "repository_identity":m["repository_identity"],
        "product_subject_sha":m["product_subject_sha"],
        "attempt_id":attempt_id,
        "execution_context":ctx,
        "gate_results":results,
        "verdict":verdict,
        "claim":m["claim"],
        "nonclaims":m["nonclaims"],
    }
    body["receipt_commitment"]=commitment(RECEIPT_DOMAIN,body)
    return body

def main():
    p=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).with_name("qcap-v2-vector-001.json")
    v=json.loads(p.read_text(encoding="utf-8"))
    if v.get("vector_format_revision")!=2:raise SystemExit("vector_revision=FAIL")
    m=v["capsule_manifest"]; ctx=v["execution_context"]
    cap=commitment(CAPSULE_DOMAIN,m)
    if cap!=v["expected_capsule_commitment"]:raise SystemExit("capsule_commitment=FAIL")
    if hashlib.sha256(canonical_json(m)).hexdigest()!=v["expected_capsule_canonical_sha256"]:raise SystemExit("capsule_canonical_sha256=FAIL")

    # Object insertion order must not affect canonical commitment.
    if commitment(CAPSULE_DOMAIN,dict(reversed(list(m.items()))))!=cap:raise SystemExit("object_order_independence=FAIL")

    # Floats are outside the canonical domain.
    try:canonical_json({"x":1.5})
    except ValueError:pass
    else:raise SystemExit("float_rejection=FAIL")

    ids=[g["id"] for g in m["gates"]]
    for s in v["scenarios"]:
        rs=s["gate_results"]
        if [r["id"] for r in rs]!=ids:raise SystemExit("scenario_gate_order=FAIL")
        r=compose_receipt(m,ctx,s["attempt_id"],rs)
        if r["verdict"]!=s["expected_verdict"]:raise SystemExit(f"{s['name']}_verdict=FAIL")
        if r["receipt_commitment"]!=s["expected_receipt_commitment"]:raise SystemExit(f"{s['name']}_receipt_commitment=FAIL")

    for bad in v["invalid_gate_result_sequences"]:
        try:attempt_verdict(bad["gate_results"])
        except ValueError:pass
        else:raise SystemExit(f"{bad['name']}_rejection=FAIL")

    print("capsule_commitment="+cap)
    for s in v["scenarios"]:print(s["name"]+"_receipt_commitment="+s["expected_receipt_commitment"])
    print("qcap_v2_independent_vectors=PASS")
    return 0

if __name__=="__main__":raise SystemExit(main())
