#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
RUNNER_FILES=("qcap3.py","qcap_canon.py","qcap_manifest.py","qcap3_limits.py","qcap3_receipt.py","qcap3_isolation.py","qcap3_exec.py")

def _bootstrap_fail(message):
 print("qcap3_bootstrap_error="+message,file=sys.stderr);raise SystemExit(20)
def _reference_bootstrap():
 if not sys.flags.isolated or not sys.dont_write_bytecode or not getattr(sys.flags,"safe_path",False):_bootstrap_fail("reference adapter requires python -I -B")
 for name in RUNNER_FILES:
  p=HERE/name
  if p.is_symlink()or not p.is_file():_bootstrap_fail("runner source closure invalid: "+name)
 for p in HERE.rglob("*.pyc"):_bootstrap_fail("bytecode cache forbidden: "+str(p.relative_to(HERE)))
 stdlib=getattr(sys,"stdlib_module_names",set())
 for p in HERE.iterdir():
  stem=p.stem if p.is_file() else p.name
  if stem in stdlib and ((p.is_file()and p.suffix==".py")or p.is_dir()):_bootstrap_fail("stdlib shadow forbidden: "+p.name)
if __name__=="__main__":_reference_bootstrap()
sys.path.insert(0,str(HERE))
from qcap_canon import CapsuleError
from qcap_manifest import validate_manifest
from qcap3_limits import limits_digest,limits_ref,validate_limits
from qcap3_receipt import verify_receipt_v3
import qcap3_exec

def runner_commitment():
 h=hashlib.sha256();h.update(b"MYCELIX_QCAP_RUNNER_V3\0")
 for name in RUNNER_FILES:
  b=(HERE/name).read_bytes();n=name.encode();h.update(len(n).to_bytes(4,"big"));h.update(n);h.update(len(b).to_bytes(8,"big"));h.update(b)
 return h.hexdigest()
def load(p):return json.loads(Path(p).read_text())
def run_capsule(m,root,repo,rid,a,c,l):
 before=runner_commitment();r=qcap3_exec.run_capsule_v3(m,root,repo,rid,a,c,l,before)
 if runner_commitment()!=before:raise CapsuleError("runner self-integrity changed during attempt")
 return r
def main(v=None):
 p=argparse.ArgumentParser();s=p.add_subparsers(dest="cmd",required=True);x=s.add_parser("limits");x.add_argument("limits");x=s.add_parser("verify-receipt");x.add_argument("manifest");x.add_argument("limits");x.add_argument("receipt");x=s.add_parser("run");x.add_argument("manifest");x.add_argument("--root",required=True);x.add_argument("--subject-repo",required=True);x.add_argument("--repository-identity",required=True);x.add_argument("--attempt-id",required=True);x.add_argument("--execution-context",required=True);x.add_argument("--execution-limits",required=True);x.add_argument("--receipt-out");a=p.parse_args(v)
 try:
  if a.cmd=="limits":
   l=load(a.limits);validate_limits(l);print("execution_limits_digest="+limits_digest(l));print("execution_limits_ref="+json.dumps(limits_ref(l),sort_keys=True));return 0
  m=load(a.manifest);l=load(a.limits)
  if a.cmd=="verify-receipt":validate_manifest(m);verify_receipt_v3(load(a.receipt),m,l);print("receipt_integrity=PASS");return 0
  r=run_capsule(m,a.root,a.subject_repo,a.repository_identity,a.attempt_id,load(a.execution_context),l);z=json.dumps(r,sort_keys=True,indent=2)+"\n";(Path(a.receipt_out).write_text(z)if a.receipt_out else sys.stdout.write(z));return 0 if r["verdict"]=="CompletedConjunctivePass"else(10 if r["verdict"]=="CompletedConjunctiveFail"else 20)
 except(CapsuleError,ValueError,json.JSONDecodeError)as e:print("qcap3_error="+str(e),file=sys.stderr);return 20
if __name__=="__main__":raise SystemExit(main())
