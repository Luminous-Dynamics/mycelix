#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, os, pathlib, subprocess, sys, tempfile
from typing import Any

SUBJECT="bf07e3fa9701e947cb7ac5c03e43052661e8a155"
SUBJECT_TREE="e486d2d74d696b92577c52ab5d4e534ed2c36890"
BASE="884a14e14758a91d3c1d370d49648946dc8b89ef"
BASE_TREE="1a3190fef7ce9fc4ea9fba05aa20f3c277ec030f"
SCHEMA="mycelix.ci-gov.001k.source-qualification.receipt.v0.2"

PATHS={
 ".github/scripts/CI_GOV_001K.lock.json":"44caaaa35ed882c5f6c4d411e6ffc7a3f65777a6",
 ".github/scripts/CI_GOV_001K.md":"b478fa78e6b6085fedb28dd52d45d8943258a07e",
 ".github/scripts/ci_qualification_capacity_oracle.py":"1b2b2f4862c910a2c013432172d74b5213c42c41",
 ".github/scripts/test_ci_qualification_capacity_oracle.py":"0ef296066505bc88d65e2e354edb59856cad5127",
}
EXEC_PATHS=(".github/scripts/ci_qualification_capacity_oracle.py",".github/scripts/test_ci_qualification_capacity_oracle.py")
QUALIFIER_PATHS={".github/scripts/CI_GOV_001K_QUALIFICATION.md",".github/scripts/ci_gov_001k_exact_qualify.py",".github/scripts/test_ci_gov_001k_exact_qualify.py"}
EXPECTED_LOCK={
 "schema":"mycelix.ci-gov.001k.source-lock.v0.2",
 "qualified_base_commit":BASE,
 "qualified_base_tree":BASE_TREE,
 "review_payload":{k:v for k,v in PATHS.items() if not k.endswith("lock.json")},
 "policy":{"shared_group":"mycelix-heavy-qualification-v1","queue":"max","cancel_in_progress":False,"max_active":1,"max_pending":8,"platform_pending_cap":100,"explicit_label":"ci:qualify","ready_admits":False,"max_snapshot_age_seconds":30},
 "authority_ceiling":{"network_client":False,"actions_mutation":False,"cancellation_authority":False,"merge_authority":False,"product_pass_authority":False,"scientific_pass_authority":False},
}

class QualificationError(RuntimeError): pass

def canonical(v:Any)->bytes: return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=True).encode()
def sha256(b:bytes)->str: return hashlib.sha256(b).hexdigest()

def git_env()->dict[str,str]:
    forbidden=("GIT_DIR","GIT_WORK_TREE","GIT_OBJECT_DIRECTORY","GIT_ALTERNATE_OBJECT_DIRECTORIES","GIT_COMMON_DIR","GIT_INDEX_FILE")
    for k in forbidden:
        if os.environ.get(k): raise QualificationError(f"forbidden git redirect env: {k}")
    env=os.environ.copy(); env["GIT_NO_REPLACE_OBJECTS"]="1"; return env

def git(*args:str,text:bool=True):
    cp=subprocess.run(["git",*args],stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=git_env(),check=False)
    if cp.returncode: raise QualificationError(f"git {' '.join(args)} failed: {cp.stderr.decode(errors='replace').strip()}")
    return cp.stdout.decode().strip() if text else cp.stdout

def reject_git_rewrite_state()->None:
    root=pathlib.Path(git("rev-parse","--show-toplevel")); gd=pathlib.Path(git("rev-parse","--git-dir"))
    if not gd.is_absolute(): gd=(root/gd).resolve()
    if (gd/"info"/"grafts").exists(): raise QualificationError("grafts file present")
    if git("for-each-ref","--format=%(refname)","refs/replace").strip(): raise QualificationError("replace refs present")

def verify_identity()->dict[str,str]:
    reject_git_rewrite_state()
    if git("rev-parse",f"{SUBJECT}^{{tree}}")!=SUBJECT_TREE: raise QualificationError("subject tree mismatch")
    if git("rev-parse",f"{SUBJECT}^")!=BASE: raise QualificationError("subject parent mismatch")
    if git("rev-parse",f"{BASE}^{{tree}}")!=BASE_TREE: raise QualificationError("base tree mismatch")
    changed=[x for x in git("diff-tree","--no-commit-id","--name-only","-r",SUBJECT).splitlines() if x]
    if len(changed)!=len(PATHS) or set(changed)!=set(PATHS): raise QualificationError(f"path set mismatch: {changed}")
    for p,oid in PATHS.items():
        if git("rev-parse",f"{SUBJECT}:{p}")!=oid: raise QualificationError(f"blob mismatch: {p}")
    return {"subject":SUBJECT,"subject_tree":SUBJECT_TREE,"base":BASE,"base_tree":BASE_TREE}

def verify_lock()->str:
    raw=git("show",f"{SUBJECT}:.github/scripts/CI_GOV_001K.lock.json",text=False)
    try: val=json.loads(raw)
    except json.JSONDecodeError as e: raise QualificationError("lock JSON invalid") from e
    if val!=EXPECTED_LOCK: raise QualificationError("lock contract mismatch")
    return sha256(raw)

def verify_qualifier_checkout()->dict[str,str]:
    head=git("rev-parse","HEAD")
    if git("rev-parse","HEAD^")!=SUBJECT: raise QualificationError("qualifier must directly parent source subject")
    changed=[x for x in git("diff-tree","--no-commit-id","--name-only","-r","HEAD").splitlines() if x]
    if len(changed)!=len(QUALIFIER_PATHS) or set(changed)!=QUALIFIER_PATHS: raise QualificationError("qualifier path set mismatch")
    if git("status","--porcelain").strip(): raise QualificationError("qualifier checkout dirty")
    root=pathlib.Path(git("rev-parse","--show-toplevel"))
    for p in QUALIFIER_PATHS:
        if (root/p).read_bytes()!=git("show",f"HEAD:{p}",text=False): raise QualificationError(f"qualifier working bytes mismatch: {p}")
    return {"qualifier_commit":head,"qualifier_tree":git("rev-parse","HEAD^{tree}")}

def run_subject_tests()->dict[str,Any]:
    with tempfile.TemporaryDirectory() as td:
        root=pathlib.Path(td)
        for p in EXEC_PATHS: (root/pathlib.Path(p).name).write_bytes(git("show",f"{SUBJECT}:{p}",text=False))
        env={"PATH":os.environ.get("PATH",""),"PYTHONHASHSEED":"0"}
        cp=subprocess.run([sys.executable,"-E","-s","-S","-B","test_ci_qualification_capacity_oracle.py"],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=env,check=False)
        if cp.returncode: raise QualificationError("subject tests failed: "+cp.stderr.decode(errors="replace"))
        return {"command":["python","-E","-s","-S","-B","test_ci_qualification_capacity_oracle.py"],"returncode":0,"stdout_sha256":sha256(cp.stdout),"stderr_sha256":sha256(cp.stderr)}

def ensure_receipt_outside_checkout(path:pathlib.Path|None)->None:
    if path is None: return
    root=pathlib.Path(git("rev-parse","--show-toplevel")).resolve(); target=path.resolve()
    if target==root or root in target.parents: raise QualificationError("receipt output must be outside checkout")

def qualify(receipt:pathlib.Path|None)->dict[str,Any]:
    own=verify_qualifier_checkout(); ident=verify_identity(); lock_sha=verify_lock(); tests=run_subject_tests(); verify_qualifier_checkout()
    body={"schema":SCHEMA,**ident,**own,"source_lock_sha256":lock_sha,"tests":tests,"proposition":"Exact CI-GOV-001K v0.2 source capsule identity and committed 28-case offline admission/budget oracle suite passed.","nonclaims":["No GitHub concurrency behavior was exercised.","No capacity-observer correctness was established.","No Actions run was mutated.","No cancellation authority is granted.","No product or scientific PASS is granted.","No fairness/FIFO property is established."],"grants_live_scheduler_qualification":False,"grants_observer_qualification":False,"grants_cancellation_authority":False,"grants_product_pass":False,"grants_scientific_pass":False}
    body["receipt_commitment"]=sha256(canonical(body))
    if receipt is not None:
        ensure_receipt_outside_checkout(receipt); receipt.write_text(json.dumps(body,indent=2,sort_keys=True)+"\n")
    return body

def main()->int:
    p=argparse.ArgumentParser(); p.add_argument("--receipt-output",type=pathlib.Path); a=p.parse_args()
    try: out=qualify(a.receipt_output)
    except QualificationError as e: print(f"QUALIFICATION ERROR: {e}",file=sys.stderr); return 2
    print(json.dumps(out,indent=2,sort_keys=True)); return 0
if __name__=="__main__": raise SystemExit(main())
