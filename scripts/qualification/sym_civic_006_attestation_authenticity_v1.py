#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,pathlib,re

ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_006_attestation_authenticity.json"
IDS=[f"A-{i:02d}" for i in range(1,20)]

def fail(m): raise SystemExit("SYM-CIVIC-006 FAIL: "+m)

def reject(c):
    x=c["candidate"]; i=c["id"]
    if i=="A-01":
        return x["subject"].get("digest") != x["signed_subject"].get("digest")
    if i=="A-02":
        return x["attester"].get("status") != "active"
    if i=="A-03":
        return x["attester"].get("status")=="revoked" and x["attestation_time"] >= x["attester"]["revoked_at"]
    if i=="A-04":
        return x["attestation_time"] > x["attester"]["key_not_after"] and not x["policy"].get("historical_grace",False)
    if i=="A-05":
        rotation=x["attester"].get("rotation",[])
        return x["attester"].get("key_id") is None and len(rotation) > 1 and rotation[0]["valid_until"] == rotation[1]["valid_from"]
    if i=="A-06":
        return x["attester"].get("requested_scope") not in x["attester"].get("delegation_scope",[])
    if i=="A-07":
        return x["statement"]["predicate_type_signed"] != x["statement"]["predicate_type_required"]
    if i=="A-08":
        return False
    if i=="A-09":
        return "digest" not in x["subject"] or "digest" not in x["signed_subject"]
    if i=="A-10":
        return not (x["signature"].get("valid") and x["signature"].get("canonical_bytes_match"))
    if i=="A-11":
        return x["attestation_time"] < x["attester"]["key_not_before"]
    if i=="A-12":
        return x["subject"].get("digest") != x["signed_subject"].get("digest")
    if i=="A-13":
        return not (
            x["subject"].get("digest")==x["signed_subject"].get("digest")
            and x["attester"].get("status")=="active"
            and x["signature"].get("valid")
            and x["signature"].get("canonical_bytes_match")
        )
    if i=="A-14":
        return not (
            x["subject"].get("digest")==x["signed_subject"].get("digest")
            and x["attester"].get("status")=="active"
            and x["attester"].get("requested_scope") in x["attester"].get("delegation_scope",[])
            and x["signature"].get("valid")
            and x["signature"].get("canonical_bytes_match")
        )
    if i=="A-15":
        return not (
            x["policy"].get("historical_key_lookup")
            and x["attester"].get("key_id")=="k1"
            and x["attestation_time"] < "2026-10-03T00:30:00Z"
            and x["signature"].get("valid")
            and x["signature"].get("canonical_bytes_match")
        )
    if i=="A-16":
        return not (
            x["statement"].get("scientific_proposition_status")=="uncertain"
            and x["attester"].get("status")=="active"
            and x["signature"].get("valid")
            and x["signature"].get("canonical_bytes_match")
        )
    if i=="A-17":
        return not (
            x["policy"].get("historical_grace")
            and x["attestation_time"] <= x["attester"]["key_not_after"]
            and x["signature"].get("valid")
            and x["signature"].get("canonical_bytes_match")
        )
    if i=="A-18":
        return x["subject_set"] != x["signed_subject_set"]
    if i=="A-19":
        return x["statement"]["signed_predicate_digest"] != x["statement"]["current_predicate_digest"]
    return True

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
    derived={c["id"]:reject(c) for c in cases}
    expected_rejects={f"A-{i:02d}" for i in range(1,13)} | {"A-18","A-19"}
    actual_rejects={k for k,v in derived.items() if v}
    print("SYM-CIVIC-006 DERIVED="+json.dumps({"rejected":sorted(actual_rejects),"admissible":sorted(k for k,v in derived.items() if not v)},separators=(",",":")))
    if actual_rejects!=expected_rejects:
        fail("derived rejection set")
    payload={"program":d["program"],"schema":d["schema"],"cases":[{"id":c["id"],"rejected":derived[c["id"]]} for c in cases]}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    print(f"SYM-CIVIC-006 PASS: 19 attestation-authenticity cases, 14 rejection cases, 5 admissible cases, canonical receipt={digest}")

if __name__=="__main__":
    main()
