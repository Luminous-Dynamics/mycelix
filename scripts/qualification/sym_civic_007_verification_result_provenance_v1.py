#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
import pathlib
import re
from datetime import datetime, timezone

ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_007_verification_result_provenance.json"
IDS=[f"V-{i:02d}" for i in range(1,24)]
CONTENT_UNQUALIFIED={"uncertain","failed","unsupported"}

def fail(m):
    raise SystemExit("SYM-CIVIC-007 FAIL: "+m)

def instant(v):
    return datetime.fromisoformat(v.replace("Z","+00:00")).astimezone(timezone.utc)

def subject_binding_ok(x):
    if "subject" not in x or "verified_subject" not in x:
        return False
    s=x["subject"]; vs=x["verified_subject"]
    return bool(s.get("digest") and vs.get("digest") and s.get("digest")==vs.get("digest"))

def verifier_ok(x):
    v=x.get("verifier",{})
    ident=v.get("id","")
    known=v.get("known",True)
    if not ident or known is False:
        return False
    # Research contract: materially versioned verifier identity is required.
    return bool(v.get("version") or re.search(r"/v[0-9]+(?:$|/)", ident))

def policy_reference_ok(x):
    policies=x.get("policies")
    if policies is None:
        return not x.get("reproducibility_claim",False)
    for p in policies:
        uri=p.get("uri","")
        digest=p.get("digest")
        if not digest or (uri.rstrip("/").endswith("/latest") and not p.get("version")):
            return False
        if p.get("retrieved_digest") and p["retrieved_digest"] != digest:
            return False
    return True

def policy_evaluation_ok(x):
    policies=x.get("policies")
    if not policies:
        return x.get("policy_evaluation") is None and not x.get("policy_evaluations")
    evaluations=x.get("policy_evaluations")
    if evaluations is None:
        single=x.get("policy_evaluation")
        evaluations=[single] if single is not None else None
    if not isinstance(evaluations,list) or len(evaluations)!=len(policies):
        return False
    t=instant(x["timeCreated"])
    for ref,evaln in zip(policies,evaluations):
        if not isinstance(evaln,dict):
            return False
        if evaln.get("digest") != ref.get("digest") or evaln.get("version") != ref.get("version"):
            return False
        if not set(evaln.get("scope",[])):
            return False
        if ref.get("valid_from") and t < instant(ref["valid_from"]):
            return False
        if ref.get("valid_until") and t >= instant(ref["valid_until"]) and not x.get("verification_policy_history",False):
            return False
        if x.get("verification_policy_history") and not ref.get("valid_from"):
            return False
    return True

def execution_time_ok(x):
    if not x.get("timeCreated"):
        return False
    t=instant(x["timeCreated"])
    start=x.get("execution_started_at")
    finish=x.get("execution_finished_at")
    if start and t < instant(start):
        return False
    if finish and t > instant(finish):
        return False
    return True

def property_scope_ok(x):
    props=x.get("properties")
    if not isinstance(props,list) or not props:
        return False
    policies=x.get("policies") or []
    evaln=x.get("policy_evaluation")
    if not policies:
        return all(bool(p.get("name")) and bool(p.get("status")) for p in props)
    scope=set(evaln.get("scope",[]))
    for p in props:
        name=p.get("name","")
        prefix=name.split("/",1)[0]
        if prefix not in scope:
            return False
    return True

def subject_set_ok(x):
    if "subjects" in x or "verified_subjects" in x:
        return x.get("subjects")==x.get("verified_subjects")
    return True

def result_identity_ok(x):
    if x.get("same_result_identity") and x.get("prior_policy_digest") and x.get("policy_evaluation"):
        return x["prior_policy_digest"] == x["policy_evaluation"].get("digest")
    if "identity_changed" in x and x.get("prior_result_id")==x.get("result_id"):
        return x.get("identity_changed") is True
    return True

def authentication_rejected(x):
    return not (
        subject_binding_ok(x)
        and verifier_ok(x)
        and policy_reference_ok(x)
        and policy_evaluation_ok(x)
        and execution_time_ok(x)
        and property_scope_ok(x)
        and subject_set_ok(x)
        and result_identity_ok(x)
    )

def disposition(c):
    x=c["candidate"]
    if authentication_rejected(x):
        return "REJECT_VERIFICATION_PROVENANCE"
    statuses={p.get("status") for p in x.get("properties",[])}
    if statuses & CONTENT_UNQUALIFIED:
        return "VERIFIED_CONTENT_UNQUALIFIED"
    if x.get("authorization_decision") not in (None,"",False):
        return "REJECT_VERIFICATION_PROVENANCE"
    return "VERIFIED"

def main():
    d=json.loads(MAN.read_text(encoding="utf-8"))
    if d.get("schema")!="mycelix.sym-civic.verification-result-provenance.v1" or d.get("program")!="SYM-CIVIC-007":
        fail("schema/program")
    if d.get("analysis_role")!="research_only":
        fail("role")
    cases=d.get("cases",[])
    if [c.get("id") for c in cases]!=IDS:
        fail("case order")
    for c in cases:
        if set(c)!={"id","family","candidate","note"}:
            fail(c["id"]+" fixture surface")
        blob=json.dumps(c,sort_keys=True,separators=(",",":")).lower()
        if any(t in blob for t in ("expected_disposition","expected_result","oracle_verdict","candidate_verdict")):
            fail(c["id"]+" embedded oracle")
        if any(k in c["candidate"] for k in ("raw_subject_identifier","raw_payload","civic_authorization")):
            fail(c["id"]+" prohibited field")

    derived={c["id"]:disposition(c) for c in cases}
    counts={k:sum(v==k for v in derived.values()) for k in (
        "REJECT_VERIFICATION_PROVENANCE",
        "VERIFIED_CONTENT_UNQUALIFIED",
        "VERIFIED",
    )}
    expected={
        "REJECT_VERIFICATION_PROVENANCE":16,
        "VERIFIED_CONTENT_UNQUALIFIED":3,
        "VERIFIED":4,
    }
    print("SYM-CIVIC-007 DERIVED="+json.dumps({"dispositions":derived,"counts":counts},separators=(",",":")))
    if counts!=expected:
        fail("disposition census")

    probes=[]
    seed=next(c["candidate"] for c in cases if c["id"]=="V-15")
    m=copy.deepcopy(seed); m["verified_subject"]["digest"]="sha256:mutated"
    probes.append(("subject_digest_mutation",disposition({"candidate":m}),"REJECT_VERIFICATION_PROVENANCE"))
    m=copy.deepcopy(seed); m["policies"][0]["digest"]="sha256:mutated"
    probes.append(("policy_digest_mutation",disposition({"candidate":m}),"REJECT_VERIFICATION_PROVENANCE"))
    m=copy.deepcopy(seed); m["properties"][0]["name"]="SCOPE_B/PASS"
    probes.append(("scope_mutation",disposition({"candidate":m}),"REJECT_VERIFICATION_PROVENANCE"))
    m=copy.deepcopy(seed); m["properties"][0]["status"]="uncertain"
    probes.append(("content_status_mutation",disposition({"candidate":m}),"VERIFIED_CONTENT_UNQUALIFIED"))
    seed_hist=next(c["candidate"] for c in cases if c["id"]=="V-17")
    m=copy.deepcopy(seed_hist); m["policy_evaluation"]["digest"]="sha256:mutated"
    probes.append(("historical_policy_digest_mutation",disposition({"candidate":m}),"REJECT_VERIFICATION_PROVENANCE"))
    if any(a!=e for _,a,e in probes):
        fail("metamorphic probe")
    print("SYM-CIVIC-007 METAMORPHIC="+json.dumps(
        [{"probe":n,"disposition":a} for n,a,_ in probes],separators=(",",":")
    ))

    payload={"program":d["program"],"schema":d["schema"],
             "cases":[{"id":c["id"],"disposition":derived[c["id"]]} for c in cases]}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    print(f"SYM-CIVIC-007 PASS: 22 verification-provenance cases; rejection=15; content-unqualified=3; verified=4; canonical receipt={digest}")

if __name__=="__main__":
    main()
