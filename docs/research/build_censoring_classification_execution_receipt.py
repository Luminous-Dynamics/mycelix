#!/usr/bin/env python3
"""Build a deterministic, non-authoritative receipt over successful verifier reports."""
from __future__ import annotations
import argparse, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

SCHEMA="mycelix.continual-adaptation.censoring-classification-execution-receipt.v1"
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
 ("audit-bundle-v2","audit-bundle-v2-python.json","audit-bundle-v2-node.json",32),
]
SUPPORTING=[("generated-corpus-a","generated-a.json",168),("generated-corpus-b","generated-b.json",168)]

def canonical(value):
    return json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(",",":")).encode("utf-8")
def sha(data:bytes)->str:
    return hashlib.sha256(data).hexdigest()
def case_items(obj):
    if isinstance(obj,list): return obj
    if isinstance(obj,dict): return obj.get("cases")
    return None

def load_report(path:Path,expected:int|None):
    raw=path.read_bytes()
    obj=json.loads(raw)
    cases=obj.get("cases")
    failures=obj.get("failures")
    explicit_count="case_count" in obj
    count=obj.get("case_count",len(cases) if isinstance(cases,list) else None)
    if not isinstance(cases,list) or not cases:return None,"empty-or-missing-cases"
    if explicit_count and (not isinstance(count,int) or count!=len(cases)):return None,"case-count-mismatch"
    if expected is not None and count!=expected:return None,"unexpected-case-count"
    if not isinstance(failures,list) or failures:return None,"reported-failures"
    ids=[x.get("case_id",x.get("id")) if isinstance(x,dict) else None for x in cases]
    if any(not isinstance(x,str) or not x for x in ids):return None,"case-identity-missing"
    if len(ids)!=len(set(ids)):return None,"duplicate-case-id"
    schema=obj.get("schema")
    if not isinstance(schema,str) or not schema:return None,"report-schema-missing"
    return {"sha256":sha(raw),"schema":schema,"case_count":count,"case_ids_sha256":sha(canonical(ids)),"failure_count":0},None

def build(root:Path,out:Path):
    out.mkdir(parents=True,exist_ok=True)
    report_dir=out/"reports"; report_dir.mkdir(exist_ok=True)
    supporting_dir=out/"supporting";supporting_dir.mkdir(exist_ok=True)
    inventory=[];expected_total=0
    for name,pyfile,nodefile,count in REPORT_PAIRS:
        py_path=root/pyfile;node_path=root/nodefile
        if not py_path.is_file() or not node_path.is_file():raise ValueError("missing-report-pair:"+name)
        if py_path.read_bytes()!=node_path.read_bytes():raise ValueError("python-node-report-mismatch:"+name)
        py_meta,e=load_report(py_path,count)
        if e:raise ValueError(name+":"+pyfile+":"+e)
        node_meta,e=load_report(node_path,count)
        if e:raise ValueError(name+":"+nodefile+":"+e)
        # Byte identity is checked above; preserve the exact outputs under the artifact root.
        shutil.copyfile(py_path,report_dir/pyfile);shutil.copyfile(node_path,report_dir/nodefile)
        inventory.append({"name":name,"python_file":"reports/"+pyfile,"node_file":"reports/"+nodefile,
          "sha256":py_meta["sha256"],"schema":py_meta["schema"],"case_count":py_meta["case_count"],
          "case_ids_sha256":py_meta["case_ids_sha256"],"failure_count":0,"python_node_byte_identical":True})
        expected_total+=2
    corpus_hashes=[]
    for name,filename,count in SUPPORTING:
        source=root/filename
        if not source.is_file():raise ValueError("missing-supporting-input:"+filename)
        raw=source.read_bytes();obj=json.loads(raw);cases=case_items(obj)
        if not isinstance(cases,list) or len(cases)!=count:raise ValueError("supporting-corpus-count:"+filename)
        shutil.copyfile(source,supporting_dir/filename)
        corpus_hashes.append({"name":name,"file":"supporting/"+filename,"sha256":sha(raw),"case_count":count})
    a=(supporting_dir/"generated-corpus-a.json").read_bytes()
    b=(supporting_dir/"generated-corpus-b.json").read_bytes()
    if a!=b:raise ValueError("generated-corpus-not-deterministic")
    source_commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=root,text=True).strip()
    event_name=os.environ.get("GITHUB_EVENT_NAME","")
    event_sha=os.environ.get("GITHUB_SHA","")
    expected=os.environ.get("PR_HEAD_SHA","") if event_name=="pull_request" else event_sha
    if not expected or source_commit!=expected:raise ValueError("exact-head-mismatch")
    if event_name=="pull_request" and not os.environ.get("PR_NUMBER"):raise ValueError("pull-request-number-missing")
    receipt={
      "schema":SCHEMA,
      "status":"research-evidence-only",
      "source":{
        "repository":os.environ.get("GITHUB_REPOSITORY",""),
        "workflow":os.environ.get("GITHUB_WORKFLOW",""),
        "workflow_ref":os.environ.get("GITHUB_WORKFLOW_REF",""),
        "event_name":event_name,
        "ref":os.environ.get("GITHUB_REF",""),
        "checked_out_commit_sha":source_commit,
        "event_sha":event_sha,
        "pull_request_head_sha":os.environ.get("PR_HEAD_SHA","") or None,
        "pull_request_number":int(os.environ["PR_NUMBER"]) if os.environ.get("PR_NUMBER","").isdigit() else None,
        "run_id":int(os.environ.get("GITHUB_RUN_ID","0")),
        "run_number":int(os.environ.get("GITHUB_RUN_NUMBER","0")),
        "run_attempt":int(os.environ.get("GITHUB_RUN_ATTEMPT","0"))
      },
      "evidence":{
        "report_pair_count":len(inventory),
        "report_file_count":expected_total,
        "report_pairs":inventory,
        "supporting_inputs":corpus_hashes,
        "generated_corpus_a_equals_b":True
      },
      "claim_ceiling":{
        "prior_verifier_steps_succeeded_at_receipt_creation":True,
        "overall_workflow_conclusion":"pending-downstream-observation",
        "hosted_qualification_pass_claimed":False,
        "qualification_authority":False,
        "scitt_interoperability_claimed":False,
        "live_network_convergence_claimed":False
      }
    }
    (out/"qualification-execution-receipt.json").write_bytes(canonical(receipt)+b"\n")
    print("report_pairs="+str(len(inventory))+" report_files="+str(expected_total)+" receipt=created")
    return 0

def self_test():
    valid={"schema":"test.v1","case_count":2,"cases":[{"case_id":"a"},{"case_id":"b"}],"failures":[]}
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p=Path(td)/"r.json";p.write_bytes(canonical(valid))
        _,e=load_report(p,2)
        assert e is None
        legacy={"schema":"test.v1","cases":[{"case_id":"a"},{"case_id":"b"}],"failures":[]}
        p.write_bytes(canonical(legacy));meta,e=load_report(p,2);assert e is None and meta["case_count"]==2
        legacy_bad=dict(legacy);legacy_bad["case_count"]=3;p.write_bytes(canonical(legacy_bad));_,e=load_report(p,2);assert e=="case-count-mismatch"
        invalid=dict(valid);invalid["failures"]=[{"case_id":"a"}];p.write_bytes(canonical(invalid))
        _,e=load_report(p,2);assert e=="reported-failures"
        invalid=dict(valid);invalid["cases"]=[{"case_id":"a"},{"case_id":"a"}];p.write_bytes(canonical(invalid))
        _,e=load_report(p,2);assert e=="duplicate-case-id"
        invalid=dict(valid);invalid["case_count"]=3;p.write_bytes(canonical(invalid))
        _,e=load_report(p,2);assert e=="case-count-mismatch"
    print("execution-receipt-builder-self-test=pass")
    return 0

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("mode",choices=["create","self-test"])
    ap.add_argument("--root",default=".")
    ap.add_argument("--out",default="execution-evidence")
    ns=ap.parse_args()
    if ns.mode=="self-test":return self_test()
    try:return build(Path(ns.root).resolve(),Path(ns.out).resolve())
    except Exception as exc:
        print("execution-receipt-build-failed:"+str(exc),file=sys.stderr);return 1
if __name__=="__main__":raise SystemExit(main())
