#!/usr/bin/env python3
"""Deterministic event simulation for delayed/partitioned observer gossip."""
from __future__ import annotations
import base64, copy, json, sys
from pathlib import Path

from verify_censoring_classification_anchor_observer_gossip import (
    GOSSIP_REGISTRY_ID, GOSSIP_REGISTRY_SCHEMA, GOSSIP_DOMAIN,
    verify_gossip_registry, verify_gossip, verify_subject_head, lookup_head, digest,
)
from verify_censoring_classification_anchor_cose_receipt import (
    mth, validate_head_quorum, consistency_valid,
)

FIXTURE_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation.v1"
CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation-campaign.v1"
EXPECTED_GOSSIP_REGISTRY_SHA="sha256:df41b280e6250f28425f2944f20e417bb69fa522eed9c52ca6b45d6b7931dd8e"
WREG_ID="mycelix.research.anchor-witness-registry.v2"
VDS_ID="mycelix.research.anchor-statement-sequence.v1"
HEAD_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1"

def b64u(raw):
    return base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")

def flip_signature(obs):
    out=copy.deepcopy(obs)
    raw=bytearray(base64.urlsafe_b64decode(out["signature"]+"="*((4-len(out["signature"])%4)%4)))
    raw[-1]^=1
    out["signature"]=b64u(bytes(raw))
    return out

def validate_observation(obs, gossip_reg, witness_reg, gossip_fixture, vds, head_fixture, q4):
    _, error = verify_gossip(obs, gossip_reg)
    if error:
        return None, error
    head=lookup_head(gossip_fixture,obs)
    if head is None:
        return None,"unknown-subject-head"
    if head.get("observer_id")!=obs.get("subject_observer_id") or head.get("tree_size")!=obs.get("observed_tree_size") or head.get("root_hash")!=obs.get("observed_root_hash"):
        return None,"subject-claim-binding"
    ok, head_error=verify_subject_head(head,witness_reg)
    if not ok:
        return None,head_error
    head_id=next((name for name,value in gossip_fixture.get("subject_heads",{}).items() if digest(value)==obs["subject_head_sha256"]),None)
    if head_id is None:
        return None,"unknown-subject-head"
    if head_id=="w02-fork4":
        # A validly signed conflicting root is evidence, not a malformed signature.
        if head["tree_size"]!=4 or bytes.fromhex(head["root_hash"][7:])==q4["root"]:
            return None,"fork-head-not-conflicting"
    else:
        try: root=mth(vds["entries"][:head["tree_size"]])
        except Exception: return None,"head-tree-invalid"
        if root.hex()!=head["root_hash"][7:]:
            return None,"subject-head-root-mismatch"
    return {"observation":obs,"head":head,"head_id":head_id},None

def view_conflict(view):
    values=list(view.values())
    for i,left in enumerate(values):
        for right in values[i+1:]:
            a,b=left["observation"],right["observation"]
            if a["observed_tree_size"]==b["observed_tree_size"] and a["observed_root_hash"]!=b["observed_root_hash"]:
                if a["monitor_id"]==b["monitor_id"]:
                    return "monitor-equivocation","same-monitor-signed-conflicting-roots"
                return "split-view-detected","authenticated-same-size-root-conflict"
    return None,None

def compatible_set(view, gossip_fixture):
    values=list(view.values())
    if len(values)<2 or len({x["observation"]["monitor_id"] for x in values})<2:
        return False,"insufficient-independent-observations"
    sizes_roots={(x["observation"]["observed_tree_size"],x["observation"]["observed_root_hash"]) for x in values}
    sizes=sorted({s for s,_ in sizes_roots})
    if len(sizes)>1:
        path=[bytes.fromhex(x[7:]) for x in gossip_fixture["consistency_proof_4_to_7"]]
        for low,high in zip(sizes,sizes[1:]):
            roots_low={bytes.fromhex(r[7:]) for s,r in sizes_roots if s==low}
            roots_high={bytes.fromhex(r[7:]) for s,r in sizes_roots if s==high}
            if len(roots_low)!=1 or len(roots_high)!=1:
                return False,"ambiguous-tree-heads"
            if low!=4 or high!=7 or not consistency_valid(low,high,next(iter(roots_low)),next(iter(roots_high)),path):
                return False,"consistency-proof-invalid"
    return True,"all-observed-heads-compatible"

def simulate_scenario(name, scenario, mutation, gossip_reg, witness_reg, gossip_fixture, vds, head_fixture, q4):
    views={"A":{},"B":{}}
    pending=[]
    sent_ids=set()
    partitioned=False
    duplicate_ingestions=0
    rejected_messages=0
    rejected_reasons=[]
    event_log=[]
    tick=-1

    def ingest(agent,obs_id,obs_payload):
        nonlocal duplicate_ingestions,rejected_messages
        checked,err=validate_observation(obs_payload,gossip_reg,witness_reg,gossip_fixture,vds,head_fixture,q4)
        if err:
            rejected_messages+=1
            rejected_reasons.append(err)
            event_log.append({"event":"rejected","agent":agent,"observation_id":obs_id,"reason":err})
            return
        if obs_id in views[agent]:
            duplicate_ingestions+=1
            event_log.append({"event":"duplicate-suppressed","agent":agent,"observation_id":obs_id})
            return
        views[agent][obs_id]=checked
        event_log.append({"event":"accepted","agent":agent,"observation_id":obs_id})

    for agent,ids in scenario.get("initial_views",{}).items():
        if agent not in views: raise ValueError("unknown-agent")
        for obs_id in ids:
            if obs_id not in gossip_fixture["observations"]: raise ValueError("unknown-observation")
            ingest(agent,obs_id,copy.deepcopy(gossip_fixture["observations"][obs_id]))

    for event in scenario.get("events",[]):
        if not isinstance(event.get("tick"),int) or event["tick"]<tick: raise ValueError("nonmonotonic-event-time")
        tick=event["tick"]
        action=event.get("action")
        if action=="partition":
            partitioned=True
            event_log.append({"event":"partition","tick":tick})
        elif action=="heal":
            partitioned=False
            order=event.get("delivery_order",[])
            by_id={m["message_id"]:m for m in pending}
            delivery=[by_id[x] for x in order if x in by_id]
            delivery.extend(m for m in pending if m["message_id"] not in set(order))
            pending=[]
            for msg in delivery:
                ingest(msg["to"],msg["observation_id"],msg["payload"])
                event_log.append({"event":"delivered","tick":tick,"message_id":msg["message_id"]})
        elif action=="send":
            src,dst,obs_id,msg_id=event.get("from"),event.get("to"),event.get("observation_id"),event.get("message_id")
            if src not in views or dst not in views or src==dst or obs_id not in views[src] or not isinstance(msg_id,str) or not msg_id:
                raise ValueError("invalid-send")
            if msg_id in sent_ids: raise ValueError("duplicate-message-id")
            sent_ids.add(msg_id)
            source_obs=copy.deepcopy(views[src][obs_id]["observation"])
            tamper=event.get("tamper_signature") or (mutation=="tamper-message-signature" and obs_id=="F4" and name=="tampered_gossip_message")
            if tamper: source_obs=flip_signature(source_obs)
            msg={"message_id":msg_id,"from":src,"to":dst,"observation_id":obs_id,"payload":source_obs,"sent_tick":tick}
            if partitioned:
                pending.append(msg)
                event_log.append({"event":"queued","tick":tick,"message_id":msg_id})
            else:
                ingest(dst,obs_id,source_obs)
                event_log.append({"event":"delivered","tick":tick,"message_id":msg_id})
        else:
            raise ValueError("unknown-event-action")

    for agent,view in views.items():
        result,reason=view_conflict(view)
        if result:
            verdict=result
            break
    else:
        converged=set(views["A"])==set(views["B"]) and bool(views["A"])
        if not converged:
            verdict,reason="unresolved-local-only","partition-or-no-cross-view-exchange"
        else:
            good,why=compatible_set(views["A"],gossip_fixture)
            if good:
                verdict,reason="converged-consistent","views-converged; all-heads-and-consistency-valid"
            else:
                verdict,reason="unresolved-local-only",why

    high_water={}
    for agent,view in views.items():
        by_subject={}
        for evidence in view.values():
            o=evidence["observation"]
            s=o["subject_observer_id"]
            by_subject[s]=max(by_subject.get(s,0),o["observed_tree_size"])
        high_water[agent]={k:by_subject[k] for k in sorted(by_subject)}
    return {
        "scenario":name,"verdict":verdict,"reason":reason,
        "view_observation_ids":{a:sorted(views[a]) for a in sorted(views)},
        "unique_observation_counts":{a:len(views[a]) for a in sorted(views)},
        "high_water_tree_sizes":high_water,
        "duplicates_suppressed":duplicate_ingestions,
        "rejected_messages":rejected_messages,
        "rejected_reasons":sorted(rejected_reasons),
        "pending_messages":len(pending),
        "partitioned_at_end":partitioned,
        "baseline_size_4_quorum":len(head_fixture["heads"]["size_4"]["attestations"]),
        "signed_fork_head_count":1,
    }

def main():
    if len(sys.argv)!=7:
        print("usage: verifier GOSSIP_REGISTRY SIM_FIXTURE CAMPAIGN GOSSIP_FIXTURE WITNESS_REGISTRY VDS_FIXTURE TREE_HEAD_FIXTURE REPORT",file=sys.stderr)
        return 2
    # There are eight runtime paths after the script name.
    gp,sp,cp,gfp,wrp,vdsp,hp,outp=sys.argv[1:]
    greg=json.loads(Path(gp).read_text())
    sim_fixture=json.loads(Path(sp).read_text())
    campaign=json.loads(Path(cp).read_text())
    gossip_fixture=json.loads(Path(gfp).read_text())
    wreg=json.loads(Path(wrp).read_text())
    vds=json.loads(Path(vdsp).read_text())
    head_fixture=json.loads(Path(hp).read_text())
    if sim_fixture.get("schema")!=FIXTURE_SCHEMA or campaign.get("schema")!=CAMPAIGN_SCHEMA or campaign.get("case_count")!=8 or len(campaign.get("cases",[]))!=8:
        return 1
    if verify_gossip_registry(greg) or digest(greg)!=EXPECTED_GOSSIP_REGISTRY_SHA: return 1
    q4,e4=validate_head_quorum(head_fixture,wreg,vds,"size_4")
    q7,e7=validate_head_quorum(head_fixture,wreg,vds,"size_7")
    if e4 or e7:
        print("tree-head-quorum-preflight="+(e4 or e7),file=sys.stderr);return 1
    fork_head=gossip_fixture["subject_heads"]["w02-fork4"]
    ok,err=verify_subject_head(fork_head,wreg)
    if not ok or fork_head["root_hash"]==q4["root"].hex():
        print("signed-fork-head-preflight="+(err or "not-conflicting"),file=sys.stderr);return 1
    for obs_id,obs in gossip_fixture["observations"].items():
        _,err=validate_observation(obs,greg,wreg,gossip_fixture,vds,head_fixture,q4)
        if err:
            print("observation-preflight:"+obs_id+"="+err,file=sys.stderr);return 1
    rows=[];failures=[]
    for case in campaign["cases"]:
        scenario=sim_fixture.get("scenarios",{}).get(case.get("scenario"))
        if scenario is None:return 1
        report=simulate_scenario(case["scenario"],scenario,case.get("mutation"),greg,wreg,gossip_fixture,vds,head_fixture,q4)
        row={"case_id":case["case_id"],"expected_verdict":case["expected_verdict"],**report}
        rows.append(row)
        if row["verdict"]!=case["expected_verdict"]:failures.append([case["case_id"],case["expected_verdict"],row["verdict"],row["reason"]])
    out={"schema":"mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation-report.v1","status":"research-evidence-only","case_count":len(rows),"cases":rows,"failures":failures}
    Path(outp).write_bytes(json.dumps(out,ensure_ascii=False,sort_keys=True,separators=(",",":")).encode()+b"\n")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0

if __name__=="__main__":raise SystemExit(main())
