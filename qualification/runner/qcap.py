#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from qcap_canon import *
from qcap_manifest import *
from qcap_receipt import *
import qcap_exec
RUNNER_FILES=("qcap.py","qcap_canon.py","qcap_manifest.py","qcap_receipt.py","qcap_isolation.py","qcap_exec.py")
def runner_commitment():
 h=hashlib.sha256();h.update(b"MYCELIX_QCAP_RUNNER_V2\0")
 for name in RUNNER_FILES:
  b=(HERE/name).read_bytes();n=name.encode();h.update(len(n).to_bytes(4,"big"));h.update(n);h.update(len(b).to_bytes(8,"big"));h.update(b)
 return h.hexdigest()
def run_capsule(m,root,repo,rid,a,c):return qcap_exec.run_capsule(m,root,repo,rid,a,c,runner_commitment())
preflight=qcap_exec.preflight
def load(p):return json.loads(Path(p).read_text())
def main(v=None):
 p=argparse.ArgumentParser();s=p.add_subparsers(dest="cmd",required=True);x=s.add_parser("commit");x.add_argument("manifest");x=s.add_parser("verify");x.add_argument("manifest");x.add_argument("--root",required=True);x=s.add_parser("verify-receipt");x.add_argument("manifest");x.add_argument("receipt");x=s.add_parser("run");x.add_argument("manifest");x.add_argument("--root",required=True);x.add_argument("--subject-repo",required=True);x.add_argument("--repository-identity",required=True);x.add_argument("--attempt-id",required=True);x.add_argument("--execution-context",required=True);x.add_argument("--receipt-out");a=p.parse_args(v)
 try:
  m=load(a.manifest)
  if a.cmd=="commit":validate_manifest(m);print("capsule_commitment="+capsule_commitment(m));return 0
  if a.cmd=="verify":validate_manifest(m,a.root);print("capsule_commitment="+capsule_commitment(m));print("capsule_integrity=PASS");return 0
  if a.cmd=="verify-receipt":validate_manifest(m);verify_receipt(load(a.receipt),m);print("receipt_integrity=PASS");return 0
  r=run_capsule(m,a.root,a.subject_repo,a.repository_identity,a.attempt_id,load(a.execution_context));z=json.dumps(r,sort_keys=True,indent=2)+"\n";(Path(a.receipt_out).write_text(z)if a.receipt_out else sys.stdout.write(z));return 0 if r["verdict"]=="CompletedConjunctivePass"else(10 if r["verdict"]=="CompletedConjunctiveFail"else 20)
 except(CapsuleError,ValueError,json.JSONDecodeError)as e:print("qcap_error="+str(e),file=sys.stderr);return 20
if __name__=="__main__":raise SystemExit(main())
