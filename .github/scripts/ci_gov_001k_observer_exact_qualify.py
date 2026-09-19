#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, os, pathlib, subprocess, sys, tempfile
from typing import Any

SUBJECT="d3de3a4d24c7459b80f939fee5c3c2bb4843ad71"
SUBJECT_TREE="3ef12424861acf5a1ee63d216b71805561bed977"
PARENT="bf07e3fa9701e947cb7ac5c03e43052661e8a155"
PARENT_TREE="e486d2d74d696b92577c52ab5d4e534ed2c36890"
SCHEMA="mycelix.ci-gov.001k.capacity-observer-qualification.receipt.v0.1"
PATHS={
 ".github/scripts/CI_GOV_001K_OBSERVER.lock.json":"04323683ae394748836ec75e2692c73063259085",
 ".github/scripts/CI_GOV_001K_OBSERVER.md":"ee47917ebcd00f8af0d5a0d149102428139f637d",
 ".github/scripts/ci_qualification_capacity_observer.py":"7af073d89f1b9db5be87b3c84fa07341a5adb722",
 ".github/scripts/test_ci_qualification_capacity_observer.py":"0cafbccbe1a147c6d81029cd9c78928d5f75d0eb",
}
EXEC_PATHS=(".github/scripts/ci_qualification_capacity_observer.py",".github/scripts/test_ci_qualification_capacity_observer.py")
QUALIFIER_PATHS={".github/scripts/CI_GOV_001K_OBSERVER_QUALIFICATION.md",".github/scripts/ci_gov_001k_observer_exact_qualify.py",".github/scripts/test_ci_gov_001k_observer_exact_qualify.py"}
EXPECTED_LOCK={
 "schema":"mycelix.ci-gov.001k.capacity-observer-lock.v0.1",
 "parent_source_commit":PARENT,"parent_source_tree":PARENT_TREE,
 "review_payload":{k:v for k,v in PATHS.items() if not k.endswith("lock.json")},
 "endpoint":{"method":"GET","url":"https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/concurrency_groups/mycelix-heavy-qualification-v1","api_version":"2026-03-10","group_name":"mycelix-heavy-qualification-v1","max_body_bytes":1048576,"max_timeout_seconds":10},
 "platform_bounds":{"max_active":1,"max_pending":100,"allowed_member_statuses":["in_progress","pending"]},
 "authority_ceiling":{"actions_read":True,"actions_mutation":False,"queue_admission_authority":False,"cancellation_authority":False,"merge_authority":False,"product_pass_authority":False,"scientific_pass_authority":False}
}
class QualificationError(RuntimeError): pass
def canonical(v:Any)->bytes: return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=True).encode()
def sha256(b:bytes)->str: return hashlib.sha256(b).hexdigest()
def git_env():
    for k in ("GIT_DIR","GIT_WORK_TREE","GIT_OBJECT_DIRECTORY","GIT_ALTERNATE_OBJECT_DIRECTORIES","GIT_COMMON_DIR","GIT_INDEX_FILE"):
        if os.environ.get(k): raise QualificationError(f"forbidden git redirect env: {k}")
    e=os.environ.copy(); e["GIT_NO_REPLACE_OBJECTS"]="1"; return e
def git(*args:str,text:bool=True):
    cp=subprocess.run(["git",*args],stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=git_env(),check=False)
    if cp.returncode: raise QualificationError(cp.stderr.decode(errors="replace"))
    return cp.stdout.decode().strip() if text else cp.stdout
def reject_rewrite():
    root=pathlib.Path(git("rev-parse","--show-toplevel")); gd=pathlib.Path(git("rev-parse","--git-dir"))
    if not gd.is_absolute(): gd=(root/gd).resolve()
    if (gd/"info"/"grafts").exists(): raise QualificationError("grafts file present")
    if git("for-each-ref","--format=%(refname)","refs/replace").strip(): raise QualificationError("replace refs present")
def verify_identity():
    reject_rewrite()
    if git("rev-parse",f"{SUBJECT}^{{tree}}")!=SUBJECT_TREE: raise QualificationError("subject tree mismatch")
    if git("rev-parse",f"{SUBJECT}^")!=PARENT: raise QualificationError("subject parent mismatch")
    if git("rev-parse",f"{PARENT}^{{tree}}")!=PARENT_TREE: raise QualificationError("parent tree mismatch")
    changed=[x for x in git("diff-tree","--no-commit-id","--name-only","-r",SUBJECT).splitlines() if x]
    if len(changed)!=4 or set(changed)!=set(PATHS): raise QualificationError("subject path set mismatch")
    for p,oid in PATHS.items():
        if git("rev-parse",f"{SUBJECT}:{p}")!=oid: raise QualificationError(f"blob mismatch: {p}")
def verify_lock():
    raw=git("show",f"{SUBJECT}:.github/scripts/CI_GOV_001K_OBSERVER.lock.json",text=False)
    try: value=json.loads(raw)
    except json.JSONDecodeError as e: raise QualificationError("invalid lock JSON") from e
    if value!=EXPECTED_LOCK: raise QualificationError("lock mismatch")
    return sha256(raw)
def verify_checkout():
    head=git("rev-parse","HEAD")
    if git("rev-parse","HEAD^")!=SUBJECT: raise QualificationError("qualifier parent mismatch")
    changed=[x for x in git("diff-tree","--no-commit-id","--name-only","-r","HEAD").splitlines() if x]
    if len(changed)!=3 or set(changed)!=QUALIFIER_PATHS: raise QualificationError("qualifier path set mismatch")
    if git("status","--porcelain").strip(): raise QualificationError("dirty qualifier")
    root=pathlib.Path(git("rev-parse","--show-toplevel"))
    for p in QUALIFIER_PATHS:
        if (root/p).read_bytes()!=git("show",f"HEAD:{p}",text=False): raise QualificationError(f"working byte mismatch: {p}")
    return head,git("rev-parse","HEAD^{tree}")
def run_tests():
    with tempfile.TemporaryDirectory() as td:
        root=pathlib.Path(td)
        for p in EXEC_PATHS: (root/pathlib.Path(p).name).write_bytes(git("show",f"{SUBJECT}:{p}",text=False))
        cp=subprocess.run([sys.executable,"-E","-s","-S","-B","test_ci_qualification_capacity_observer.py"],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env={"PATH":os.environ.get("PATH",""),"PYTHONHASHSEED":"0"},check=False)
        if cp.returncode: raise QualificationError("observer tests failed: "+cp.stderr.decode(errors="replace"))
        return {"returncode":0,"stdout_sha256":sha256(cp.stdout),"stderr_sha256":sha256(cp.stderr)}
def outside(path:pathlib.Path|None):
    if path is None:return
    root=pathlib.Path(git("rev-parse","--show-toplevel")).resolve(); target=path.resolve()
    if target==root or root in target.parents: raise QualificationError("receipt must be outside checkout")
def qualify(path=None):
    qcommit,qtree=verify_checkout(); verify_identity(); lock=verify_lock(); tests=run_tests(); verify_checkout()
    body={"schema":SCHEMA,"subject":SUBJECT,"subject_tree":SUBJECT_TREE,"parent":PARENT,"qualifier_commit":qcommit,"qualifier_tree":qtree,"source_lock_sha256":lock,"tests":tests,
          "proposition":"Exact CI-GOV-001K-B v0.1 read-only capacity observer identity and committed 26-case corpus passed.",
          "grants_live_observation":False,"grants_queue_admission":False,"grants_cancellation_authority":False,"grants_product_pass":False,"grants_scientific_pass":False,
          "nonclaims":["No live API observation was qualified by this source test.","No queue admission authority is granted.","No scheduler behavior is established."]}
    body["receipt_commitment"]=sha256(canonical(body))
    if path is not None: outside(path); path.write_text(json.dumps(body,indent=2,sort_keys=True)+"\n")
    return body
def main():
    p=argparse.ArgumentParser(); p.add_argument("--receipt-output",type=pathlib.Path); a=p.parse_args()
    try:r=qualify(a.receipt_output)
    except QualificationError as e: print(f"QUALIFICATION ERROR: {e}",file=sys.stderr); return 2
    print(json.dumps(r,indent=2,sort_keys=True)); return 0
if __name__=="__main__": raise SystemExit(main())
