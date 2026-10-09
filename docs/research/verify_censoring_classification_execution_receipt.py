#!/usr/bin/env python3
"""Verify a downloaded research execution receipt against GitHub workflow_run metadata."""
from __future__ import annotations
import argparse, copy, hashlib, json, sys
from pathlib import Path

RECEIPT_SCHEMA="mycelix.continual-adaptation.censoring-classification-execution-receipt.v1"
PREDICATE_SCHEMA="https://luminousdynamics.io/attestations/mycelix-anchor-research-execution/v1"
REPOSITORY="Luminous-Dynamics/mycelix"
WORKFLOW_NAME="continual-adaptation-censoring-classification-provenance"
WORKFLOW_PATH=".github/workflows/continual-adaptation-censoring-classification-provenance.yml"
REPORT_PAIRS=[
 ("fixed-classification","python-fixed.json","node-fixed.json",52),
 ("generated-classification","python-generated.json","node-generated.json",168),
 ("anchor-governance","anchor-governance-python.json","anchor-governance-node.json",24),
 ("witness-non-equivocation","witness-python.json","witness-node.json",24),
 ("witness-cryptographic-authentication","witness-crypto-python.json","witness-crypto-node.json",22),
 ("append-only-vds","witness-vds-python.json","witness-vds-node.json",15),
 ("governed-key-rotation","witness-rotation-python.json","witness-rotation-node.json",15),
 ("authenticated-tree-heads","witness-tree-head-python.json","witness-tree-head-node.json",21),
 ("legacy-inclusion-receipts","receipt-python.json","receipt-node.json",16),
 ("static-observer-gossip","observer-gossip-python.json","observer-gossip-node.json",16),
 ("audit-bundle-v1","audit-bundle-python.json","audit-bundle-node.json",18),
 ("cose-receipts","cose-receipt-python.json","cose-receipt-node.json",22),
 ("partitioned-gossip-simulation","gossip-simulation-python.json","gossip-simulation-node.json",8),
 ("audit-bundle-v2","audit-bundle-v2-python.json","audit-bundle-v2-node.json",34),
]
SUPPORTING=[("generated-corpus-a","supporting/generated-a.json",168),("generated-corpus-b","supporting/generated-b.json",168)]

def canonical(value):
    return json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(",",":")).encode("utf-8")
def sha(data:bytes)->str:
    return hashlib.sha256(data).hexdigest()
def read_json(path:Path):
    try:return json.loads(path.read_text(encoding="utf-8")),None
    except Exception as exc:return None,"json:"+str(exc)
def validate_report(path:Path,expected:int|None):
    raw=path.read_bytes()
    obj,err=read_json(path)
    if err:return None,err
    cases=obj.get("cases");failures=obj.get("failures")
    explicit_count="case_count" in obj
    count=obj.get("case_count",len(cases) if isinstance(cases,list) else None)
    if not isinstance(cases,list) or not cases:return None,"empty-or-missing-cases"
    if explicit_count and (not isinstance(count,int) or count!=len(cases)):return None,"case-count-mismatch"
    if expected is not None and count!=expected:return None,"unexpected-case-count"
    if not isinstance(failures,list) or failures:return None,"reported-failures"
    ids=[x.get("case_id",x.get("id")) if isinstance(x,dict) else None for x in cases]
    if any(not isinstance(x,str) or not x for x in ids):return None,"case-identity-missing"
    if len(ids)!=len(set(ids)):return None,"duplicate-case-id"
    for row in cases:
        if isinstance(row,dict) and "expected_verdict" in row:
            actual=row.get("actual_verdict",row.get("verdict"))
            if actual is None:return None,"actual-verdict-missing"
            if actual!=row["expected_verdict"]:return None,"verdict-mismatch"
    if not isinstance(obj.get("schema"),str) or not obj["schema"]:return None,"report-schema-missing"
    return {"sha256":sha(raw),"case_count":count,"schema":obj["schema"],"case_ids_sha256":sha(canonical(ids))},None

def validate_receipt(receipt_path:Path,artifact_root:Path,event_path:Path):
    receipt,err=read_json(receipt_path)
    if err:return None,err
    event,err=read_json(event_path)
    if err:return None,"workflow-run-"+err
    wr=event.get("workflow_run") if isinstance(event,dict) else None
    if not isinstance(wr,dict):return None,"workflow-run-event-missing"
    if not isinstance(receipt,dict) or set(receipt)!={"schema","status","source","evidence","claim_ceiling"}:return None,"receipt-envelope"
    if receipt.get("schema")!=RECEIPT_SCHEMA:return None,"receipt-schema"
    if receipt.get("status")!="research-evidence-only":return None,"receipt-status"
    src=receipt.get("source");evidence=receipt.get("evidence");ceiling=receipt.get("claim_ceiling")
    if not isinstance(src,dict) or set(src)!={"repository","workflow","workflow_ref","event_name","ref","checked_out_commit_sha","event_sha","pull_request_head_sha","pull_request_number","run_id","run_number","run_attempt"}:return None,"source-schema"
    if not isinstance(evidence,dict) or set(evidence)!={"report_pair_count","report_file_count","report_pairs","supporting_inputs","generated_corpus_a_equals_b"}:return None,"evidence-schema"
    if not isinstance(ceiling,dict) or set(ceiling)!={"prior_verifier_steps_succeeded_at_receipt_creation","overall_workflow_conclusion","hosted_qualification_pass_claimed","qualification_authority","scitt_interoperability_claimed","live_network_convergence_claimed"}:return None,"claim-ceiling-schema"
    event_repo=event.get("repository") if isinstance(event,dict) else None
    wr_repo=wr.get("repository");head_repo=wr.get("head_repository")
    if not isinstance(event_repo,dict) or event_repo.get("full_name")!=REPOSITORY or src.get("repository")!=REPOSITORY:return None,"repository-binding"
    if not isinstance(wr_repo,dict) or wr_repo.get("full_name")!=REPOSITORY:return None,"source-run-repository"
    if not isinstance(head_repo,dict) or head_repo.get("full_name")!=REPOSITORY:return None,"source-run-head-repository"
    if wr.get("name")!=WORKFLOW_NAME or str(wr.get("path","")).split("@")[0]!=WORKFLOW_PATH:return None,"source-workflow-binding"
    if wr.get("event")!="push" or src.get("event_name")!="push":return None,"source-event-not-push"
    if wr.get("head_branch")!="main" or src.get("ref")!="refs/heads/main":return None,"source-branch-not-main"
    if wr.get("conclusion")!="success":return None,"source-workflow-not-success"
    pairs=evidence.get("report_pairs")
    if not isinstance(pairs,list) or len(pairs)!=len(REPORT_PAIRS) or evidence.get("report_pair_count")!=len(REPORT_PAIRS):return None,"report-pair-inventory"
    if evidence.get("report_file_count")!=2*len(REPORT_PAIRS):return None,"report-file-count"
    if evidence.get("generated_corpus_a_equals_b") is not True:return None,"generated-corpus-identity-claim"
    support=evidence.get("supporting_inputs")
    if not isinstance(support,list) or len(support)!=len(SUPPORTING) or {x.get("name") for x in support if isinstance(x,dict)}!={x[0] for x in SUPPORTING}:return None,"supporting-input-inventory"
    expected_names={name for _,pyfile,nodefile,_ in REPORT_PAIRS for name in (pyfile,nodefile)}
    seen_names=set()
    expected_by_layer={name:(pyfile,nodefile,count) for name,pyfile,nodefile,count in REPORT_PAIRS}
    summary=[]
    expected_item_keys={"name","python_file","node_file","sha256","schema","case_count","case_ids_sha256","failure_count","python_node_byte_identical"}
    for item in pairs:
        if not isinstance(item,dict) or set(item)!=expected_item_keys:return None,"report-item-schema"
        layer=item.get("name")
        if layer not in expected_by_layer:return None,"unknown-report-layer"
        pyname,nodename,expected_count=expected_by_layer[layer]
        if item.get("python_file")!="reports/"+pyname or item.get("node_file")!="reports/"+nodename:return None,"report-path-substitution:"+layer
        if item.get("python_node_byte_identical") is not True or item.get("failure_count")!=0:return None,"report-parity-claim:"+layer
        for field,filename in (("python_file",pyname),("node_file",nodename)):
            rel=item[field]
            if rel in seen_names:return None,"duplicate-report-path"
            seen_names.add(rel)
            path=artifact_root/rel
            if not path.is_file():return None,"report-missing:"+filename
            meta,err=validate_report(path,expected_count)
            if err:return None,filename+":"+err
            if meta["sha256"]!=item.get("sha256"):return None,"report-sha:"+filename
            if meta["schema"]!=item.get("schema") or meta["case_count"]!=item.get("case_count") or meta["case_ids_sha256"]!=item.get("case_ids_sha256"):return None,"report-metadata:"+filename
        if (artifact_root/"reports"/pyname).read_bytes()!=(artifact_root/"reports"/nodename).read_bytes():return None,"python-node-report-mismatch:"+layer
        summary.append({"name":layer,"sha256":item["sha256"],"case_count":item["case_count"],"report_schema":item["schema"]})
    if seen_names!=expected_names:return None,"report-inventory-mismatch"
    for name,rel,count in SUPPORTING:
        path=artifact_root/rel
        if not path.is_file():return None,"supporting-input-missing:"+name
        raw=path.read_bytes();obj,err=read_json(path)
        if err:return None,"supporting-input-"+err
        cases=obj if isinstance(obj,list) else (obj.get("cases") if isinstance(obj,dict) else None)
        if not isinstance(cases,list) or len(cases)!=count:return None,"supporting-input-case-count:"+name
        entries=support
        pin=next((x for x in entries if x.get("name")==name),None)
        if not pin or pin.get("file")!=rel or pin.get("sha256")!=sha(raw) or pin.get("case_count")!=count:return None,"supporting-input-pin:"+name
    if (artifact_root/"supporting/generated-a.json").read_bytes()!=(artifact_root/"supporting/generated-b.json").read_bytes():return None,"generated-corpus-not-deterministic"
    source=receipt["source"]
    expected_src={
      "repository":REPOSITORY,"workflow":WORKFLOW_NAME,"event_name":"push","ref":"refs/heads/main",
      "checked_out_commit_sha":wr.get("head_sha"),"event_sha":wr.get("head_sha"),
      "run_id":wr.get("id"),"run_number":wr.get("run_number"),"run_attempt":wr.get("run_attempt",1)
    }
    for k,v in expected_src.items():
        if source.get(k)!=v:return None,"source-metadata-binding:"+k
    if source.get("workflow_ref")!=REPOSITORY+"/"+WORKFLOW_PATH+"@refs/heads/main":return None,"source-workflow-ref-binding"
    if source.get("pull_request_head_sha") is not None or source.get("pull_request_number") is not None:return None,"unexpected-pr-head"
    if not str(source.get("workflow_ref","")).endswith(WORKFLOW_PATH+"@refs/heads/main"):return None,"source-workflow-ref-binding"
    if ceiling.get("prior_verifier_steps_succeeded_at_receipt_creation") is not True or ceiling.get("overall_workflow_conclusion")!="pending-downstream-observation":return None,"claim-ceiling-context"
    for key in ("hosted_qualification_pass_claimed","qualification_authority","scitt_interoperability_claimed","live_network_convergence_claimed"):
        if ceiling.get(key) is not False:return None,"qualification-claim-injection"
    predicate={
      "schema":PREDICATE_SCHEMA,
      "evidence_type":"mycelix-anchor-transparency-execution-receipt",
      "source_workflow_run":{
        "repository":REPOSITORY,"workflow_name":wr["name"],"workflow_path":wr["path"],
        "workflow_id":wr.get("workflow_id"),"run_id":wr["id"],"run_number":wr["run_number"],
        "run_attempt":wr.get("run_attempt",1),"event":"push","branch":"main","head_sha":wr["head_sha"],
        "conclusion":"success"
      },
      "receipt_sha256":sha(receipt_path.read_bytes()),
      "report_pairs":summary,
      "report_pair_count":len(summary),
      "report_file_count":len(seen_names),
      "claim_ceiling":{
        "qualification_decision":"not-claimed",
        "hosted_qualification_pass":False,
        "complete_scitt_interoperability":False,
        "live_network_convergence":False,
        "organizational_independence":False,
        "private_key_custody_proven":False
      }
    }
    return predicate,None

def self_test():
    import tempfile
    sample={"schema":"test.v1","case_count":2,"cases":[{"case_id":"a"},{"case_id":"b"}],"failures":[]}
    with tempfile.TemporaryDirectory() as td:
        root=Path(td);report_path=root/"report.json";report_path.write_bytes(canonical(sample))
        _,err=validate_report(report_path,2);assert err is None
        legacy={"schema":"test.v1","cases":[{"case_id":"a"},{"case_id":"b"}],"failures":[]}
        report_path.write_bytes(canonical(legacy));meta,err=validate_report(report_path,2);assert err is None and meta["case_count"]==2
        legacy_bad=dict(legacy);legacy_bad["case_count"]=3;report_path.write_bytes(canonical(legacy_bad));_,err=validate_report(report_path,2);assert err=="case-count-mismatch"
        invalid=dict(sample);invalid["failures"]=[{"case_id":"a"}];report_path.write_bytes(canonical(invalid));_,err=validate_report(report_path,2);assert err=="reported-failures"
        invalid=dict(sample);invalid["cases"]=[{"case_id":"a"},{"case_id":"a"}];report_path.write_bytes(canonical(invalid));_,err=validate_report(report_path,2);assert err=="duplicate-case-id"
        invalid=dict(sample);invalid["cases"]=[{"case_id":"a","expected_verdict":"qualified","actual_verdict":"unresolved"},{"case_id":"b"}];report_path.write_bytes(canonical(invalid));_,err=validate_report(report_path,2);assert err=="verdict-mismatch"
        invalid=dict(sample);invalid["cases"]=[{"case_id":"a","expected_verdict":"qualified"},{"case_id":"b"}];report_path.write_bytes(canonical(invalid));_,err=validate_report(report_path,2);assert err=="actual-verdict-missing"

    with tempfile.TemporaryDirectory() as td:
        base=Path(td);artifact=base/"artifact";reports=artifact/"reports";support=artifact/"supporting"
        reports.mkdir(parents=True);support.mkdir()
        report_pairs=[]
        for layer,pyname,nodename,count in REPORT_PAIRS:
            assert count is not None, "self-test needs fixed expected report count: "+layer
            ids=[f"{layer}-{i:03d}" for i in range(count)]
            value={"schema":"self-test."+layer,"status":"research-evidence-only","case_count":count,
                   "cases":[{"case_id":x,"expected_verdict":"qualified","actual_verdict":"qualified"} for x in ids],"failures":[]}
            raw=canonical(value)+b"\n"
            for filename in (pyname,nodename):(reports/filename).write_bytes(raw)
            meta,_=validate_report(reports/pyname,count)
            report_pairs.append({"name":layer,"python_file":"reports/"+pyname,"node_file":"reports/"+nodename,
              "sha256":meta["sha256"],"schema":meta["schema"],"case_count":meta["case_count"],
              "case_ids_sha256":meta["case_ids_sha256"],"failure_count":0,"python_node_byte_identical":True})
        support_pins=[]
        for name,rel,count in SUPPORTING:
            value={"schema":"self-test.generated-corpus.v1","cases":[{"case_id":f"generated-{i:03d}"} for i in range(count)]}
            raw=canonical(value)+b"\n";(artifact/rel).write_bytes(raw)
            support_pins.append({"name":name,"file":rel,"sha256":sha(raw),"case_count":count})
        head="a"*40
        wr={"repository":{"full_name":REPOSITORY},"head_repository":{"full_name":REPOSITORY},
            "name":WORKFLOW_NAME,"path":WORKFLOW_PATH,"event":"push","head_branch":"main","conclusion":"success",
            "head_sha":head,"id":12345,"run_number":77,"run_attempt":2,"workflow_id":888}
        event={"repository":{"full_name":REPOSITORY},"workflow_run":wr}
        event_path=base/"event.json";event_path.write_bytes(canonical(event)+b"\n")
        receipt={
          "schema":RECEIPT_SCHEMA,"status":"research-evidence-only",
          "source":{"repository":REPOSITORY,"workflow":WORKFLOW_NAME,
            "workflow_ref":REPOSITORY+"/"+WORKFLOW_PATH+"@refs/heads/main","event_name":"push","ref":"refs/heads/main",
            "checked_out_commit_sha":head,"event_sha":head,"pull_request_head_sha":None,"pull_request_number":None,
            "run_id":12345,"run_number":77,"run_attempt":2},
          "evidence":{"report_pair_count":len(report_pairs),"report_file_count":2*len(report_pairs),
            "report_pairs":report_pairs,"supporting_inputs":support_pins,"generated_corpus_a_equals_b":True},
          "claim_ceiling":{"prior_verifier_steps_succeeded_at_receipt_creation":True,
            "overall_workflow_conclusion":"pending-downstream-observation","hosted_qualification_pass_claimed":False,
            "qualification_authority":False,"scitt_interoperability_claimed":False,"live_network_convergence_claimed":False}
        }
        receipt_path=base/"receipt.json"
        def write_receipt(value):receipt_path.write_bytes(canonical(value)+b"\n")
        write_receipt(receipt)
        _,err=validate_receipt(receipt_path,artifact,event_path);assert err is None, "valid synthetic receipt rejected: "+str(err)
        bad_event=dict(event);bad_event["workflow_run"]=dict(wr,conclusion="failure")
        event_path.write_bytes(canonical(bad_event)+b"\n")
        _,err=validate_receipt(receipt_path,artifact,event_path);assert err=="source-workflow-not-success"
        event_path.write_bytes(canonical(event)+b"\n")
        fork_event=dict(event);fork_event["workflow_run"]=dict(wr,head_repository={"full_name":"attacker/fork"},head_branch="main")
        event_path.write_bytes(canonical(fork_event)+b"\n")
        _,err=validate_receipt(receipt_path,artifact,event_path);assert err=="source-run-head-repository"
        event_path.write_bytes(canonical(event)+b"\n")
        tampered=reports/"python-fixed.json";tampered.write_bytes(tampered.read_bytes()+b" ")
        _,err=validate_receipt(receipt_path,artifact,event_path);assert err=="report-sha:python-fixed.json"
        raw=canonical({"schema":"self-test.fixed-classification","status":"research-evidence-only","case_count":52,
          "cases":[{"case_id":f"fixed-classification-{i:03d}","expected_verdict":"qualified","actual_verdict":"qualified"} for i in range(52)],"failures":[]})+b"\n"
        tampered.write_bytes(raw)
        changed=copy.deepcopy(receipt);changed["claim_ceiling"]["hosted_qualification_pass_claimed"]=True;write_receipt(changed)
        _,err=validate_receipt(receipt_path,artifact,event_path);assert err=="qualification-claim-injection"
        extra=copy.deepcopy(receipt);extra["untrusted_extra_field"]="present";write_receipt(extra)
        _,err=validate_receipt(receipt_path,artifact,event_path);assert err=="receipt-envelope"
    print("execution-evidence-verifier-self-test=pass")
    return 0

def main():
    ap=argparse.ArgumentParser()
    sub=ap.add_subparsers(dest="mode",required=True)
    sub.add_parser("self-test")
    v=sub.add_parser("verify")
    v.add_argument("--artifact-root",required=True);v.add_argument("--receipt",required=True);v.add_argument("--event",required=True);v.add_argument("--predicate",required=True)
    ns=ap.parse_args()
    if ns.mode=="self-test":return self_test()
    predicate,err=validate_receipt(Path(ns.receipt),Path(ns.artifact_root),Path(ns.event))
    if err:
        print("execution-evidence-rejected:"+err,file=sys.stderr);return 1
    Path(ns.predicate).write_bytes(canonical(predicate)+b"\n")
    print("execution-evidence-predicate=validated")
    return 0
if __name__=="__main__":raise SystemExit(main())
