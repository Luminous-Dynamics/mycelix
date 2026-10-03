#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,pathlib
from datetime import datetime,timezone

ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_006_attestation_authenticity.json"
IDS=[f"A-{i:02d}" for i in range(1,22)]
CONTENT_UNQUALIFIED={"false","uncertain","unsupported"}

def fail(m): raise SystemExit("SYM-CIVIC-006 FAIL: "+m)

def instant(v):
    return datetime.fromisoformat(v.replace("Z","+00:00")).astimezone(timezone.utc)

def subject_binding_ok(x):
    if "subject" in x or "signed_subject" in x:
        s=x.get("subject",{}); ss=x.get("signed_subject",{})
        return bool(s.get("digest") and ss.get("digest") and s.get("digest")==ss.get("digest"))
    return x.get("subject_set")==x.get("signed_subject_set")

def attester_ok(x):
    return x.get("attester",{}).get("status")=="active"

def time_policy_ok(x):
    a=x.get("attester",{}); p=x.get("policy",{}); at=instant(x["attestation_time"])
    verification=instant(x["verification_time"]) if x.get("verification_time") else None
    if a.get("key_not_before") and at < instant(a["key_not_before"]):
        return False
    if a.get("key_not_after") and at > instant(a["key_not_after"]) and not p.get("historical_grace",False):
        return False
    if a.get("key_not_after") and verification and verification > instant(a["key_not_after"]) and not (
        p.get("historical_grace",False) or p.get("historical_key_lookup",False)
    ):
        return False
    if a.get("rotation"):
        key_id=a.get("key_id")
        if not key_id:
            return False
        matches=[
            k for k in a["rotation"]
            if k.get("key_id")==key_id
            and (not k.get("valid_from") or at >= instant(k["valid_from"]))
            and (not k.get("valid_until") or at < instant(k["valid_until"]))
        ]
        if len(matches)!=1:
            return False
    if a.get("key_history"):
        key_id=a.get("key_id")
        if not key_id:
            return False
        matches=[k for k in a["key_history"] if k.get("key_id")==key_id]
        if len(matches)!=1:
            return False
        epoch=matches[0]
        if epoch.get("valid_from") and at < instant(epoch["valid_from"]):
            return False
        if epoch.get("valid_until") and at >= instant(epoch["valid_until"]):
            return False
        if verification and verification > at and not p.get("historical_key_lookup"):
            return False
    return True

def predicate_ok(x):
    s=x.get("statement",{})
    if "predicate_type_signed" in s or "predicate_type_required" in s:
        return s.get("predicate_type_signed")==s.get("predicate_type_required")
    return bool(s.get("predicate_type"))

def delegation_ok(x):
    a=x.get("attester",{})
    scope=a.get("delegation_scope")
    if scope is None:
        return True
    return a.get("requested_scope") in scope

def signature_ok(x):
    sig=x.get("signature",{})
    return bool(sig.get("valid") and sig.get("canonical_bytes_match"))

def authentication_rejected(x):
    if not subject_binding_ok(x):
        return True
    if not attester_ok(x):
        return True
    if not time_policy_ok(x):
        return True
    if not delegation_ok(x):
        return True
    if not predicate_ok(x):
        return True
    if not signature_ok(x):
        return True
    stmt=x.get("statement",{})
    if stmt.get("signed_predicate_digest") and stmt.get("current_predicate_digest"):
        if stmt["signed_predicate_digest"] != stmt["current_predicate_digest"]:
            return True
    if "subject_set" in x and "signed_subject_set" in x and x["subject_set"] != x["signed_subject_set"]:
        return True
    return False

def disposition(c):
    x=c["candidate"]
    if authentication_rejected(x):
        return "REJECT_AUTHENTICATION"
    if x.get("statement",{}).get("scientific_proposition_status") in CONTENT_UNQUALIFIED:
        return "AUTHENTICATED_CONTENT_UNQUALIFIED"
    return "AUTHENTICATED"

def main():
    d=json.loads(MAN.read_text(encoding="utf-8"))
    if d.get("schema")!="mycelix.sym-civic.attestation-authenticity.v1" or d.get("program")!="SYM-CIVIC-006":
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
        if any(k in c["candidate"] for k in ("raw_subject_identifier","raw_payload","authorized_decision","civic_authorization")):
            fail(c["id"]+" prohibited field")
    derived={c["id"]:disposition(c) for c in cases}
    counts={k:sum(v==k for v in derived.values()) for k in (
        "REJECT_AUTHENTICATION","AUTHENTICATED_CONTENT_UNQUALIFIED","AUTHENTICATED"
    )}
    print("SYM-CIVIC-006 DERIVED="+json.dumps({"dispositions":derived,"counts":counts},separators=(",",":")))
    if counts != {
        "REJECT_AUTHENTICATION":14,
        "AUTHENTICATED_CONTENT_UNQUALIFIED":3,
        "AUTHENTICATED":4,
    }:
        fail("disposition census")
    payload={
        "program":d["program"],
        "schema":d["schema"],
        "cases":[{"id":c["id"],"disposition":derived[c["id"]]} for c in cases]
    }
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    print(f"SYM-CIVIC-006 PASS: 21 attestation-authenticity cases; rejection=14; authenticated-but-content-unqualified=3; authenticated=4; canonical receipt={digest}")

if __name__=="__main__":
    main()
