#!/usr/bin/env python3
from __future__ import annotations
import copy, hashlib, json, pathlib
from datetime import datetime, timezone

ROOT = pathlib.Path(__file__).resolve().parents[2]
MAN = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_012_decision_finality.json"
IDS = [f"F-{i:02d}" for i in range(1, 22)]
STATUSES = {"FINAL", "REVOKED", "SUPERSEDED", "EXPIRED"}

def fail(msg): raise SystemExit("SYM-CIVIC-012 FAIL: " + msg)
def canon(v): return json.dumps(v, sort_keys=True, separators=(",", ":"))
def sha(v): return "sha256:" + hashlib.sha256(canon(v).encode()).hexdigest()
def when(v):
    if not isinstance(v, str) or not v: return None
    try: return datetime.fromisoformat(v.replace("Z", "+00:00")).astimezone(timezone.utc)
    except (TypeError, ValueError): return None

def finmat(a):
    return {k:a.get(k) for k in ("id","decision_id","decision_identity_digest","issuer","authority_profile_ref","authority_profile_digest","asserted_at","valid_from","valid_until","status","successor_id","predecessor_id")}

def set_fin_identity(a):
    a["identity_material"] = finmat(a)
    a["identity_digest"] = sha(a["identity_material"])

def decision_ok(c):
    d = c.get("decision", {})
    req = ("id","subject_digest","identity_material","identity_digest","valid_until","outcome","policy_id","policy_version","policy_digest","composition_digest","evaluated_at")
    if not isinstance(d, dict) or any(not d.get(k) for k in req): return False
    if d["identity_digest"] != sha(d["identity_material"]): return False
    if any(d["identity_material"].get(k) != d[k] for k in ("subject_digest","policy_id","policy_version","policy_digest","composition_digest","evaluated_at")): return False
    if d["identity_material"].get("decision_valid_until") != d["valid_until"] or d["identity_material"].get("outcome") != d["outcome"]: return False
    e, u = when(d["evaluated_at"]), when(d["valid_until"])
    return e is not None and u is not None and e <= u and d["outcome"] in {"allow","deny","unresolved"}

def authority_ok(p):
    req = ("id","version","digest","actual_digest","valid_from","valid_until","trusted_issuers")
    if not isinstance(p, dict) or any(not p.get(k) for k in req): return False
    if p["digest"] != p["actual_digest"] or str(p["id"]).rstrip("/").endswith("/latest"): return False
    a, b = when(p["valid_from"]), when(p["valid_until"])
    return a is not None and b is not None and a <= b and isinstance(p["trusted_issuers"],list) and len(p["trusted_issuers"]) == len(set(p["trusted_issuers"]))

def fin_ok(a,c):
    if not isinstance(a,dict): return False
    req = ("id","decision_id","decision_identity_digest","issuer","authority_profile_ref","authority_profile_digest","asserted_at","valid_from","valid_until","status","mutable_status","identity_material","identity_digest")
    if any(k not in a for k in req) or not a["id"] or a["status"] not in STATUSES or a["mutable_status"] is not False: return False
    if a["identity_material"] != finmat(a) or a["identity_digest"] != sha(a["identity_material"]): return False
    d, p = c["decision"], c["authority_profile"]
    if a["decision_id"] != d["id"] or a["decision_identity_digest"] != d["identity_digest"]: return False
    if a["authority_profile_ref"] != p["id"] or a["authority_profile_digest"] != p["digest"] or a["issuer"] not in p["trusted_issuers"]: return False
    at, vf, vu = when(a["asserted_at"]), when(a["valid_from"]), when(a["valid_until"])
    de, du = when(d["evaluated_at"]), when(d["valid_until"])
    pf, pu = when(p["valid_from"]), when(p["valid_until"])
    if None in (at,vf,vu,de,du,pf,pu): return False
    if vf > vu or vf < de or vu > du or at > vf or at < de or vf < pf or vu > pu: return False
    if a["status"] in {"REVOKED","SUPERSEDED"} and not a.get("predecessor_id"): return False
    if a.get("successor_id") and a["status"] != "SUPERSEDED": return False
    return True

def rejected(c):
    if not decision_ok(c) or not authority_ok(c.get("authority_profile")): return True
    aa = c.get("finality_assertions")
    if not isinstance(aa,list) or len(aa) != len({a.get("id") for a in aa if isinstance(a,dict)}): return True
    if any(not fin_ok(a,c) for a in aa): return True
    prior_id, prior_mat = c.get("previous_finality_id"), c.get("previous_finality_identity_material")
    if prior_id is not None or prior_mat is not None:
        matches = [a for a in aa if isinstance(a,dict) and a.get("id") == prior_id]
        if len(matches) != 1 or not isinstance(prior_mat,dict) or prior_mat != finmat(matches[0]): return True
    src = c.get("currentness_source")
    if isinstance(src,dict) and src.get("mutable") is True: return True
    if c.get("finality_authorization_mapping") in {"authorize","grant_capability"}: return True
    if c.get("finality_authority_claim") in {"civic_authority","legal_finality"}: return True
    if c.get("historical_truth_promotion") is True: return True
    return False

def independent(c):
    if rejected(c): return "REJECT_FINALITY_PROVENANCE"
    aa = c["finality_assertions"]
    if not aa: return "FINALITY_NOT_CURRENT"
    t = when(c.get("currentness_check_time"))
    if t is None: return "REJECT_FINALITY_PROVENANCE"
    active = [a for a in aa if when(a["valid_from"]) <= t <= when(a["valid_until"])]
    if not active: return "FINALITY_NOT_CURRENT"
    statuses = {a["status"] for a in active}
    if len(statuses) > 1: return "FINALITY_UNRESOLVED"
    return "FINALITY_CURRENT" if next(iter(statuses)) == "FINAL" else "FINALITY_NOT_CURRENT"

def qualify(c):
    if rejected(c): return "REJECT_FINALITY_PROVENANCE"
    aa = c["finality_assertions"]
    if not aa: return "FINALITY_NOT_CURRENT"
    t = when(c.get("currentness_check_time"))
    if t is None: return "REJECT_FINALITY_PROVENANCE"
    active = [a for a in aa if when(a["valid_from"]) <= t <= when(a["valid_until"])]
    if not active: return "FINALITY_NOT_CURRENT"
    statuses = {a["status"] for a in active}
    if len(statuses) > 1:
        return "FINALITY_UNRESOLVED" if c.get("finality_conflict_handling") == "preserve" else "REJECT_FINALITY_PROVENANCE"
    return "FINALITY_CURRENT" if next(iter(statuses)) == "FINAL" else "FINALITY_NOT_CURRENT"

def no_oracle(case):
    if set(case) != {"id","family","candidate","note"}: fail(case.get("id","?")+" fixture surface")
    if any(x in canon(case).lower() for x in ("expected_disposition","expected_result","oracle_verdict","candidate_verdict")): fail(case["id"]+" embedded oracle")

def main():
    doc=json.loads(MAN.read_text(encoding="utf-8"))
    if doc.get("schema")!="mycelix.sym-civic.decision-finality.v1" or doc.get("program")!="SYM-CIVIC-012" or doc.get("analysis_role")!="research_only": fail("header")
    cases=doc.get("cases",[])
    if [c.get("id") for c in cases] != IDS: fail("case order")
    for c in cases: no_oracle(c)
    ref={c["id"]:independent(c["candidate"]) for c in cases}
    derived={c["id"]:qualify(c["candidate"]) for c in cases}
    for i in IDS:
        if ref[i] != derived[i]: fail(i+" independent reference disagreement")
    counts={k:sum(v==k for v in derived.values()) for k in ("REJECT_FINALITY_PROVENANCE","FINALITY_CURRENT","FINALITY_NOT_CURRENT","FINALITY_UNRESOLVED")}
    expected={"REJECT_FINALITY_PROVENANCE":12,"FINALITY_CURRENT":3,"FINALITY_NOT_CURRENT":5,"FINALITY_UNRESOLVED":1}
    print("SYM-CIVIC-012 DERIVED="+canon({"counts":counts,"dispositions":derived}))
    if counts != expected: fail("disposition census")
    seed=copy.deepcopy(next(c["candidate"] for c in cases if c["id"]=="F-01"))
    base=copy.deepcopy(seed["finality_assertions"][0]); probes=[]
    def p(name,c,want): probes.append((name,qualify(c),want))
    m=copy.deepcopy(seed); m["decision"]["identity_digest"]="sha256:mutated"; p("decision_identity_mutation",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["finality_assertions"][0]["decision_id"]="decision-other"; p("finality_decision_binding_mutation",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["finality_assertions"][0]["issuer"]="issuer-x"; p("issuer_mutation",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["finality_assertions"][0]["authority_profile_ref"]="https://example.org/authority/latest"; p("authority_profile_mutation",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["finality_assertions"][0]["valid_until"]="2026-10-05T00:00:00Z"; p("horizon_widening",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["currentness_check_time"]="2026-10-04T01:00:00Z"; p("currentness_after_expiry",m,"FINALITY_NOT_CURRENT")
    m=copy.deepcopy(seed); a=m["finality_assertions"][0]; a.update(id="finality-revocation",status="REVOKED",valid_from="2026-10-03T04:00:00Z",asserted_at="2026-10-03T04:00:00Z",predecessor_id=base["id"]); set_fin_identity(a); p("revocation_transition",m,"FINALITY_NOT_CURRENT")
    m=copy.deepcopy(seed); a=m["finality_assertions"][0]; a.update(id="finality-supersession",status="SUPERSEDED",valid_from="2026-10-03T04:00:00Z",asserted_at="2026-10-03T04:00:00Z",predecessor_id=base["id"],successor_id="decision-b"); set_fin_identity(a); p("supersession_transition",m,"FINALITY_NOT_CURRENT")
    m=copy.deepcopy(seed); a=copy.deepcopy(base); b=copy.deepcopy(base); b.update(id="finality-revocation",status="REVOKED",valid_from="2026-10-03T02:30:00Z",asserted_at="2026-10-03T02:30:00Z",predecessor_id=base["id"]); set_fin_identity(b); m["finality_assertions"]=[a,b]; m["finality_conflict_handling"]="preserve"; p("conflict_preservation",m,"FINALITY_UNRESOLVED")
    m=copy.deepcopy(seed); m["finality_assertions"]=list(reversed(m["finality_assertions"])); p("assertion_order_permutation",m,"FINALITY_CURRENT")
    m=copy.deepcopy(seed); extra=copy.deepcopy(base); extra.update(id="finality-b"); set_fin_identity(extra); m["finality_assertions"]=[base,extra]; m["previous_finality_id"]="finality-b"; m["previous_finality_identity_material"]=finmat(extra); p("lineage_reference_order_forward",m,"FINALITY_CURRENT")
    m=copy.deepcopy(m); m["finality_assertions"]=list(reversed(m["finality_assertions"])); p("lineage_reference_order_reversed",m,"FINALITY_CURRENT")
    m=copy.deepcopy(seed); m["finality_assertions"][0]["identity_digest"]="sha256:0"; p("finality_identity_digest_tamper",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["exact_replay"]=True; m["replay_of"]=base["id"]; p("exact_replay",m,"FINALITY_CURRENT")
    m=copy.deepcopy(seed); a=m["finality_assertions"][0]; a["status"]="REVOKED"; m["previous_finality_id"]=base["id"]; m["previous_finality_identity_material"]=finmat(base); p("reused_finality_id",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["finality_authorization_mapping"]="authorize"; p("authorization_promotion",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["finality_authority_claim"]="civic_authority"; p("civic_authority_promotion",m,"REJECT_FINALITY_PROVENANCE")
    m=copy.deepcopy(seed); m["currentness_source"]={"ref":"https://example.org/finality-registry/latest","mutable":True}; p("latest_registry_substitution",m,"REJECT_FINALITY_PROVENANCE")
    if any(a!=e for _,a,e in probes): fail("metamorphic probe")
    print("SYM-CIVIC-012 METAMORPHIC="+canon([{"probe":n,"disposition":a} for n,a,_ in probes]))
    receipt=[{"id":c["id"],"disposition":derived[c["id"]],"decision_id":c["candidate"]["decision"]["id"],"decision_identity_digest":c["candidate"]["decision"]["identity_digest"],"finality_assertion_ids":sorted(a["id"] for a in c["candidate"]["finality_assertions"]),"finality_identity_digests":sorted(a["identity_digest"] for a in c["candidate"]["finality_assertions"]),"currentness_check_time":c["candidate"]["currentness_check_time"]} for c in cases]
    rd=hashlib.sha256(canon({"program":doc["program"],"schema":doc["schema"],"cases":receipt}).encode()).hexdigest()
    print(f"SYM-CIVIC-012 PASS: 21 decision-finality cases; rejection={counts['REJECT_FINALITY_PROVENANCE']}; current={counts['FINALITY_CURRENT']}; not_current={counts['FINALITY_NOT_CURRENT']}; unresolved={counts['FINALITY_UNRESOLVED']}; canonical receipt={rd}")
if __name__=="__main__": main()
