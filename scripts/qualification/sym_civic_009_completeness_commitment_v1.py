#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
import pathlib
from datetime import datetime, timezone

ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_009_completeness_commitment.json"
IDS=[f"C-{i:02d}" for i in range(1,18)]

def fail(m):
    raise SystemExit("SYM-CIVIC-009 FAIL: "+m)

def instant(v):
    return datetime.fromisoformat(v.replace("Z","+00:00")).astimezone(timezone.utc)

def attester_ok(c):
    k=c.get("commitment",{})
    a=k.get("attester",{})
    sig=k.get("signature",{})
    return (
        a.get("status")=="active"
        and sig.get("valid") is True
        and sig.get("canonical_bytes_match") is True
    )

def bundle_binding_ok(c):
    b=c.get("bundle",{}); k=c.get("commitment",{})
    return bool(b.get("digest") and k.get("bundle_digest") and b["digest"]==k["bundle_digest"])

def manifest_ok(c):
    m=c.get("manifest",{}); k=c.get("commitment",{})
    if not m.get("digest") or m.get("actual_digest")!=m.get("digest"):
        return False
    if k.get("manifest_digest")!=m.get("digest"):
        return False
    if "uri" in m and m.get("uri","").rstrip("/").endswith("/latest") and not m.get("version"):
        return False
    if k.get("signed_manifest_digest") and k["signed_manifest_digest"]!=k.get("evaluated_manifest_digest"):
        return False
    return True

def scope_ok(c):
    b=set(c.get("bundle",{}).get("scope",[]))
    s=set(c.get("commitment",{}).get("scope",[]))
    if not b:
        return bool(s)
    return b==s

def time_ok(c):
    k=c.get("commitment",{})
    if not k.get("time"):
        return False
    t=instant(k["time"])
    w=c.get("bundle_window")
    if w:
        if w.get("created") and t < instant(w["created"]):
            return False
        if w.get("updated") and t > instant(w["updated"]):
            return False
    return True

def identity_history_ok(c):
    k=c.get("commitment",{})
    if c.get("membership_changed") and c.get("prior_commitment_id")==k.get("id"):
        return False
    if c.get("metadata_changed") and c.get("prior_commitment_id")==k.get("id"):
        return False
    if c.get("historical"):
        return bool(
            c.get("historical_manifest_exact")
            and c.get("historical_manifest_digest")==k.get("manifest_digest")
        )
    if c.get("historical_manifest_digest"):
        return c.get("historical_manifest_digest")==k.get("manifest_digest")
    return True

def conflict_ok(c):
    cs=c.get("commitments")
    if not cs:
        return True
    if c.get("commitments_collapsed"):
        return False
    return len({x.get("manifest_digest") for x in cs})==1

def interpretation_ok(c):
    if (
        c.get("external_exhaustiveness_evidence") is False
        and c.get("consumer_interpretation")=="authenticated_commitment_proves_exhaustive"
    ):
        return False
    if c.get("declares_exhaustive") and not c.get("external_exhaustiveness_evidence"):
        return False
    return True

def provenance_rejected(c):
    return not (
        bundle_binding_ok(c)
        and manifest_ok(c)
        and attester_ok(c)
        and scope_ok(c)
        and time_ok(c)
        and identity_history_ok(c)
        and conflict_ok(c)
        and interpretation_ok(c)
    )

def disposition(case):
    c=case["candidate"]
    if provenance_rejected(c):
        return "REJECT_COMPLETENESS_COMMITMENT"
    if c.get("completeness_status") in {"uncertain","unsupported"}:
        return "COMMITMENT_CONTENT_UNQUALIFIED"
    return "COMMITMENT_ACCEPTED"

def main():
    d=json.loads(MAN.read_text(encoding="utf-8"))
    if d.get("schema")!="mycelix.sym-civic.completeness-commitment.v1" or d.get("program")!="SYM-CIVIC-009":
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
        if any(k in c["candidate"] for k in ("authorized_decision","civic_authorization","raw_subject_identifier","raw_payload")):
            fail(c["id"]+" prohibited field")

    derived={c["id"]:disposition(c) for c in cases}
    counts={k:sum(v==k for v in derived.values()) for k in (
        "REJECT_COMPLETENESS_COMMITMENT","COMMITMENT_CONTENT_UNQUALIFIED","COMMITMENT_ACCEPTED"
    )}
    expected={
        "REJECT_COMPLETENESS_COMMITMENT":13,
        "COMMITMENT_CONTENT_UNQUALIFIED":1,
        "COMMITMENT_ACCEPTED":4
    }
    print("SYM-CIVIC-009 DERIVED="+json.dumps({"dispositions":derived,"counts":counts},separators=(",",":")))
    if counts!=expected:
        fail("disposition census")

    probes=[]
    seed=next(c["candidate"] for c in cases if c["id"]=="C-14")
    m=copy.deepcopy(seed); m["commitment"]["bundle_digest"]="sha256:mutated"
    probes.append(("bundle_digest_mutation",disposition({"candidate":m}),"REJECT_COMPLETENESS_COMMITMENT"))
    m=copy.deepcopy(seed); m["manifest"]["actual_digest"]="sha256:mutated"
    probes.append(("manifest_digest_mutation",disposition({"candidate":m}),"REJECT_COMPLETENESS_COMMITMENT"))
    m=copy.deepcopy(seed); m["commitment"]["attester"]["status"]="unknown"
    probes.append(("attester_mutation",disposition({"candidate":m}),"REJECT_COMPLETENESS_COMMITMENT"))
    m=copy.deepcopy(seed); m["commitment"]["scope"]=["bundle:other"]
    probes.append(("scope_mutation",disposition({"candidate":m}),"REJECT_COMPLETENESS_COMMITMENT"))
    seed_sig=next(c["candidate"] for c in cases if c["id"]=="C-14")
    m=copy.deepcopy(seed_sig); m["commitment"]["signature"]["valid"]=False
    probes.append(("signature_mutation",disposition({"candidate":m}),"REJECT_COMPLETENESS_COMMITMENT"))
    seed_unc=next(c["candidate"] for c in cases if c["id"]=="C-13")
    m=copy.deepcopy(seed_unc); m["completeness_status"]="unsupported"
    probes.append(("content_status_mutation",disposition({"candidate":m}),"COMMITMENT_CONTENT_UNQUALIFIED"))
    if any(a!=e for _,a,e in probes):
        fail("metamorphic probe")
    print("SYM-CIVIC-009 METAMORPHIC="+json.dumps(
        [{"probe":n,"disposition":a} for n,a,_ in probes],separators=(",",":")
    ))

    payload={"program":d["program"],"schema":d["schema"],
             "cases":[{"id":c["id"],"disposition":derived[c["id"]]} for c in cases]}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    print(f"SYM-CIVIC-009 PASS: 18 completeness-commitment cases; rejection=13; content-unqualified=1; accepted=4; canonical receipt={digest}")

if __name__=="__main__":
    main()
