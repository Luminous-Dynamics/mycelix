#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
import pathlib
from collections import defaultdict

ROOT=pathlib.Path(__file__).resolve().parents[2]
MAN=ROOT/"mycelix-workspace/docs/civic-resilience/sym_civic_008_bundle_completeness.json"
IDS=[f"B-{i:02d}" for i in range(1,22)]
CONTENT_UNQUALIFIED={"uncertain","failed","unsupported"}

def fail(m):
    raise SystemExit("SYM-CIVIC-008 FAIL: "+m)

def entry_key(e):
    return (e.get("attestation_id"),e.get("subject_digest"))

def recognized_selected(x):
    return [
        m for m in x.get("members",[])
        if m.get("recognized",False) and m.get("selected_as_evidence",False)
    ]

def membership_ok(x):
    required=x.get("required_attestations",[])
    members=x.get("members",[])
    by_id=defaultdict(list)
    for m in members:
        if m.get("attestation_id"):
            by_id[m["attestation_id"]].append(m)

    # Conflicting duplicate identities must stay explicit and never collapse.
    for ident,items in by_id.items():
        digests={m.get("subject_digest") for m in items}
        if len(digests)>1:
            return False
        statuses={m.get("status") for m in items}
        if len(items)>1 and (statuses-{"active"} or x.get("conflicts_collapsed")):
            if x.get("conflicts_collapsed"):
                return False

    for req in required:
        matches=[
            m for m in members
            if m.get("attestation_id")==req.get("attestation_id")
            and m.get("subject_digest")==req.get("subject_digest")
            and m.get("recognized",False)
            and m.get("selected_as_evidence",False)
        ]
        if len(matches)!=1 or matches[0].get("obsolete",False):
            return False

    for m in recognized_selected(x):
        if m.get("relevant") is False:
            return False
        if m.get("predicate_supported") is False:
            return False

    if x.get("unrecognized_lines"):
        for line in x["unrecognized_lines"]:
            if line.get("selected_as_evidence") or not line.get("ignored",False):
                return False

    if x.get("subjects") is not None and x.get("verified_subjects") is not None:
        if x["subjects"] != x["verified_subjects"]:
            return False

    return True

def completeness_ok(x):
    if not x.get("completeness_claim",False):
        return True
    manifest=x.get("manifest")
    if manifest is None:
        return False
    if manifest.get("declared_digest") != manifest.get("actual_digest"):
        return False
    selected=recognized_selected(x)
    declared=manifest.get("declared_entries",[])
    if declared != selected:
        return False
    required=x.get("required_attestations",[])
    selected_keys={entry_key(m) for m in selected}
    required_keys={entry_key(m) for m in required}
    return required_keys.issubset(selected_keys)

def identity_ok(x):
    if x.get("metadata_changed") and x.get("bundle_id")==x.get("prior_bundle_id"):
        return False
    if x.get("stale_verification_state")=="VALID":
        required_keys={entry_key(m) for m in x.get("required_attestations",[])}
        selected_keys={entry_key(m) for m in recognized_selected(x)}
        if not required_keys.issubset(selected_keys):
            return False
    if x.get("historical"):
        return bool(x.get("historical_manifest_exact"))
    return True

def monotonic_ok(x):
    if x.get("monotonic_property") and x.get("deletion_changes_decision"):
        return False
    if x.get("order_invariant") is False:
        return False
    return True

def provenance_rejected(x):
    return not (
        membership_ok(x)
        and completeness_ok(x)
        and identity_ok(x)
        and monotonic_ok(x)
    )

def disposition(c):
    x=c["candidate"]
    if provenance_rejected(x):
        return "REJECT_BUNDLE_PROVENANCE"
    statuses={p.get("status") for p in x.get("properties",[])}
    if statuses & CONTENT_UNQUALIFIED:
        return "BUNDLE_CONTENT_UNQUALIFIED"
    if x.get("unrecognized_lines") and all(
        (not line.get("selected_as_evidence")) and line.get("ignored",False)
        for line in x["unrecognized_lines"]
    ):
        return "UNRECOGNIZED_IGNORED"
    return "BUNDLE_ACCEPTED"

def main():
    d=json.loads(MAN.read_text(encoding="utf-8"))
    if d.get("schema")!="mycelix.sym-civic.bundle-completeness.v1" or d.get("program")!="SYM-CIVIC-008":
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
        "REJECT_BUNDLE_PROVENANCE",
        "BUNDLE_CONTENT_UNQUALIFIED",
        "BUNDLE_ACCEPTED",
        "UNRECOGNIZED_IGNORED",
    )}
    expected={
        "REJECT_BUNDLE_PROVENANCE":15,
        "BUNDLE_CONTENT_UNQUALIFIED":1,
        "BUNDLE_ACCEPTED":4,
        "UNRECOGNIZED_IGNORED":1,
    }
    print("SYM-CIVIC-008 DERIVED="+json.dumps({"dispositions":derived,"counts":counts},separators=(",",":")))
    if counts!=expected:
        fail("disposition census")

    probes=[]
    seed=next(c["candidate"] for c in cases if c["id"]=="B-16")
    m=copy.deepcopy(seed); m["members"]=m["members"][:1]
    probes.append(("required_member_removal",disposition({"candidate":m}),"REJECT_BUNDLE_PROVENANCE"))
    m=copy.deepcopy(seed); m["order_invariant"]=False
    probes.append(("order_dependence_mutation",disposition({"candidate":m}),"REJECT_BUNDLE_PROVENANCE"))
    m=copy.deepcopy(seed); m["members"].append({
        "attestation_id":"att-x","subject_digest":"sha256:x","recognized":True,
        "selected_as_evidence":True,"relevant":False
    }); m["manifest"]["declared_entries"]=m["members"]
    probes.append(("irrelevant_selection_mutation",disposition({"candidate":m}),"REJECT_BUNDLE_PROVENANCE"))
    m=copy.deepcopy(seed); m["members"].append({
        "attestation_id":"att-x","subject_digest":"sha256:x","recognized":False,
        "selected_as_evidence":False
    }); m["manifest"]["declared_entries"]=m["members"][:2]
    m["unrecognized_lines"]=[{"recognized":False,"ignored":True}]
    probes.append(("unrecognized_ignored_addition",disposition({"candidate":m}),"UNRECOGNIZED_IGNORED"))
    seed_content=next(c["candidate"] for c in cases if c["id"]=="B-21")
    m=copy.deepcopy(seed_content); m["properties"][0]["status"]="failed"
    probes.append(("content_status_mutation",disposition({"candidate":m}),"BUNDLE_CONTENT_UNQUALIFIED"))
    if any(a!=e for _,a,e in probes):
        fail("metamorphic probe")
    print("SYM-CIVIC-008 METAMORPHIC="+json.dumps(
        [{"probe":n,"disposition":a} for n,a,_ in probes],separators=(",",":")
    ))

    payload={"program":d["program"],"schema":d["schema"],
             "cases":[{"id":c["id"],"disposition":derived[c["id"]]} for c in cases]}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    print(f"SYM-CIVIC-008 PASS: 21 bundle-provenance cases; rejection=15; content-unqualified=1; accepted=4; unrecognized-ignored=1; canonical receipt={digest}")

if __name__=="__main__":
    main()
