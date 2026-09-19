#!/usr/bin/env python3
"""Read-only Git workflow identity verifier for EVIDENCE-CI-004D."""
from __future__ import annotations
import argparse, hashlib, json, os, re, shutil, subprocess, tempfile
from pathlib import Path
from typing import Any

SCHEMA="mycelix-qualification-workflow-identity-v1"
POLICY_SCHEMA="mycelix-qualification-registration-admission-policy-v1"
POLICY_IMPL="f0f7b60d02f98202433757222319afe7e9802b6be4d46ffd58856f8d75225845"
POLICY_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_ADMISSION_POLICY_V1\0"
INTENT_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_INTENT_V1\0"
VERIFY_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_IDENTITY_V1\0"
IMPL_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_IDENTITY_IMPLEMENTATION_V1\0"
MAX_JSON=8*1024*1024; MAX_OUT=1024*1024
OID=re.compile(r"^[0-9a-f]{40}$"); HEX64=re.compile(r"^[0-9a-f]{64}$")
REPO=re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
WF=re.compile(r"^\.github/workflows/[A-Za-z0-9._/-]+\.(?:yml|yaml)$")
INTENT_FIELDS={"schema","intent_id","repository","subject_sha","predecessor_sha","preflight_profile_id","preflight_profile_commitment","preflight_implementation_commitment","environment_adapter_implementation_commitment","preflight_environment_commitment","qualification_workflow_path","qualification_workflow_commit_sha","qualification_workflow_blob_sha1","registration_mode"}
POLICY_FIELDS={"schema","classification","intent_commitment","preflight_receipt_commitment","repository","subject_sha","predecessor_sha","qualification_workflow_path","qualification_workflow_commit_sha","qualification_workflow_blob_sha1","registration_mode","registration_authority","workflow_dispatched","workflow_identity_verified","qualification_result","qualification_authority","registration_admission_policy_implementation_commitment","registration_admission_core_implementation_commitment","supported_preflight_implementation_commitment","supported_environment_adapter_implementation_commitment","producer_revision_supported","receipt_authenticity_verified","registration_admission_core_commitment","registration_admission_policy_commitment"}
_B=Path(__file__).read_bytes(); IMPLEMENTATION_COMMITMENT=hashlib.sha256(IMPL_DOMAIN+_B).hexdigest(); del _B
class Invalid(RuntimeError): pass
class Refused(RuntimeError): pass
class Unavailable(RuntimeError): pass

def canon(v:Any)->bytes:
    if isinstance(v,float): raise Invalid("float in commitment input")
    if isinstance(v,int) and not isinstance(v,bool) and abs(v)>9007199254740991: raise Invalid("unsafe integer")
    if isinstance(v,dict):
        for k,x in v.items():
            if not isinstance(k,str): raise Invalid("non-string key")
            canon(x)
    elif isinstance(v,list):
        for x in v: canon(x)
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode()
def commit(domain:bytes,v:Any)->str: return hashlib.sha256(domain+canon(v)).hexdigest()
def reject_const(x:str): raise Invalid(f"non-I-JSON constant: {x}")
def load(path:Path,label:str)->dict[str,Any]:
    try: raw=path.read_bytes()
    except OSError as e: raise Invalid(f"cannot read {label}") from e
    if len(raw)>MAX_JSON: raise Invalid(f"{label} exceeds size bound")
    def hook(pairs):
        d={}
        for k,v in pairs:
            if k in d: raise Invalid(f"duplicate key in {label}: {k}")
            d[k]=v
        return d
    try: v=json.loads(raw.decode("utf-8","strict"),object_pairs_hook=hook,parse_constant=reject_const)
    except (UnicodeDecodeError,json.JSONDecodeError) as e: raise Invalid(f"invalid {label}") from e
    if not isinstance(v,dict): raise Invalid(f"{label} must be object")
    return v
def oid(v,label):
    if not isinstance(v,str) or not OID.fullmatch(v): raise Invalid(f"invalid {label}")
    return v
def h64(v,label):
    if not isinstance(v,str) or not HEX64.fullmatch(v): raise Invalid(f"invalid {label}")
    return v
def validate_intent(i:dict[str,Any])->str:
    if set(i)!=INTENT_FIELDS or i.get("schema")!="mycelix-qualification-registration-intent-v1": raise Invalid("intent shape/schema mismatch")
    if not isinstance(i.get("repository"),str) or not REPO.fullmatch(i["repository"]): raise Invalid("invalid repository")
    oid(i.get("subject_sha"),"subject"); oid(i.get("predecessor_sha"),"predecessor"); oid(i.get("qualification_workflow_commit_sha"),"workflow commit"); oid(i.get("qualification_workflow_blob_sha1"),"workflow blob")
    for f in ("preflight_profile_commitment","preflight_implementation_commitment","environment_adapter_implementation_commitment","preflight_environment_commitment"): h64(i.get(f),f)
    p=i.get("qualification_workflow_path")
    if not isinstance(p,str) or not WF.fullmatch(p) or "//" in p or "/./" in p or "/../" in p: raise Invalid("invalid workflow path")
    if i.get("registration_mode")!="manual-request-v1" or not i.get("intent_id") or not i.get("preflight_profile_id"): raise Invalid("invalid intent metadata")
    return commit(INTENT_DOMAIN,i)
def validate_policy(p:dict[str,Any],ic:str)->str:
    if set(p)!=POLICY_FIELDS or p.get("schema")!=POLICY_SCHEMA: raise Invalid("004C policy shape/schema mismatch")
    if p.get("registration_admission_policy_implementation_commitment")!=POLICY_IMPL: raise Invalid("unsupported 004C policy implementation")
    pc=h64(p.get("registration_admission_policy_commitment"),"004C policy commitment"); material=dict(p); material.pop("registration_admission_policy_commitment")
    if commit(POLICY_DOMAIN,material)!=pc: raise Invalid("004C policy commitment mismatch")
    for f in ("registration_authority","workflow_dispatched","workflow_identity_verified","qualification_authority"):
        if p.get(f) is not False: raise Invalid(f"004C broadened authority: {f}")
    if p.get("qualification_result") is not None or p.get("producer_revision_supported") is not True or p.get("receipt_authenticity_verified") is not False: raise Invalid("004C authority/provenance boundary mismatch")
    if p.get("classification")!="ADMISSIBLE_TO_REQUEST": raise Refused("004C result is not admissible")
    if p.get("intent_commitment")!=ic: raise Refused("intent commitment mismatch")
    return pc
def sha256_file(p:Path)->str:
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""): h.update(b)
    return h.hexdigest()
def git_identity()->dict[str,str]:
    f=shutil.which("git")
    if not f: raise Unavailable("git unavailable")
    try: p=Path(f).resolve(strict=True)
    except (OSError,RuntimeError) as e: raise Unavailable("cannot resolve git") from e
    if not p.is_file() or not os.access(p,os.X_OK): raise Unavailable("git is not executable regular file")
    return {"path":str(p),"sha256":sha256_file(p)}
def git_same(g:dict[str,str])->bool:
    try: return Path(g["path"]).resolve(strict=True)==Path(g["path"]) and sha256_file(Path(g["path"]))==g["sha256"]
    except (OSError,RuntimeError,KeyError): return False
def env(home:Path,git:Path)->dict[str,str]:
    home.mkdir(parents=True,exist_ok=True)
    return {"PATH":str(git.parent),"HOME":str(home),"LC_ALL":"C","LANG":"C","TZ":"UTC","GIT_CONFIG_NOSYSTEM":"1","GIT_CONFIG_GLOBAL":os.devnull,"GIT_TERMINAL_PROMPT":"0","GIT_OPTIONAL_LOCKS":"0","GIT_NO_LAZY_FETCH":"1"}
def run(git:str,repo:Path,e:dict[str,str],args:list[str],allow_fail=False)->bytes:
    try: cp=subprocess.run([git,"-C",str(repo),*args],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=e,timeout=30)
    except (OSError,subprocess.TimeoutExpired) as x: raise Unavailable(f"git command unavailable: {args[0]}") from x
    if len(cp.stdout)>MAX_OUT or len(cp.stderr)>MAX_OUT: raise Unavailable("git output exceeded bound")
    if cp.returncode:
        if allow_fail: return b""
        raise Unavailable(f"git command failed: {args[0]}")
    return cp.stdout
def normalize_origin(s:str)->str|None:
    for prefix in ("https://github.com/","http://github.com/","git://github.com/","ssh://git@github.com/"):
        if s.startswith(prefix): s=s[len(prefix):]; break
    else:
        if s.startswith("git@github.com:"): s=s[len("git@github.com:"):]
        else: return None
    s=s.removesuffix(".git").strip("/"); return s if REPO.fullmatch(s) else None
def head(g,r,e): return oid(run(g,r,e,["rev-parse","--verify","HEAD"]).decode().strip(),"HEAD")
def status(g,r,e): return run(g,r,e,["status","--porcelain=v1","-z","--untracked-files=all","--ignored=matching"])
def tree_entry(g,r,e,c,p,b):
    kind=run(g,r,e,["cat-file","-t",c],allow_fail=True)
    if not kind: raise Unavailable("workflow commit unavailable locally")
    if kind.decode().strip()!="commit": raise Refused("workflow SHA is not commit")
    raw=run(g,r,e,["ls-tree","-z","--full-tree",c,"--",f":(literal){p}"]); es=[x for x in raw.split(b"\0") if x]
    if len(es)!=1: raise Refused("workflow path does not resolve exactly once")
    try: hdr,rp=es[0].split(b"\t",1); mode,typ,o=hdr.decode("ascii").split(" ",2); op=rp.decode("utf-8")
    except (ValueError,UnicodeDecodeError) as x: raise Invalid("unexpected ls-tree output") from x
    if op!=p or mode!="100644" or typ!="blob": raise Refused("workflow path is not canonical regular blob")
    if o!=b: raise Refused("workflow blob identity mismatch")
    return {"mode":mode,"type":typ,"oid":o,"path":op}
def verify(repo:Path,p:dict[str,Any],i:dict[str,Any])->dict[str,Any]:
    ic=validate_intent(i); pc=validate_policy(p,ic)
    for f in ("repository","subject_sha","predecessor_sha","qualification_workflow_path","qualification_workflow_commit_sha","qualification_workflow_blob_sha1","registration_mode"):
        if p.get(f)!=i.get(f): raise Refused(f"004C field mismatch: {f}")
    gi=git_identity(); g=gi["path"]
    with tempfile.TemporaryDirectory(prefix="mycelix-wfid-") as td:
        e=env(Path(td)/"home",Path(g))
        if not repo.is_dir(): raise Unavailable("repository unavailable")
        ver=run(g,repo,e,["--version"]).decode().strip(); fmt=run(g,repo,e,["rev-parse","--show-object-format"]).decode().strip()
        if fmt!="sha1": raise Unavailable("unsupported Git object format")
        observed=normalize_origin(run(g,repo,e,["remote","get-url","origin"]).decode().strip())
        if observed is None or observed.lower()!=i["repository"].lower(): raise Refused("repository origin mismatch")
        hb=head(g,repo,e); sb=status(g,repo,e)
        if not git_same(gi): raise Invalid("Git executable drift before verification")
        te=tree_entry(g,repo,e,i["qualification_workflow_commit_sha"],i["qualification_workflow_path"],i["qualification_workflow_blob_sha1"])
        if not git_same(gi): raise Invalid("Git executable drift after verification")
        if head(g,repo,e)!=hb or status(g,repo,e)!=sb: raise Invalid("caller repository state changed")
    out={"schema":SCHEMA,"workflow_identity_verifier_implementation_commitment":IMPLEMENTATION_COMMITMENT,"classification":"WORKFLOW_IDENTITY_VERIFIED","registration_admission_policy_commitment":pc,"registration_intent_commitment":ic,"repository":i["repository"],"subject_sha":i["subject_sha"],"predecessor_sha":i["predecessor_sha"],"qualification_workflow_path":i["qualification_workflow_path"],"qualification_workflow_commit_sha":i["qualification_workflow_commit_sha"],"qualification_workflow_blob_sha1":i["qualification_workflow_blob_sha1"],"git":{**gi,"version":ver,"object_format":fmt},"tree_entry":te,"caller_head":hb,"caller_status_sha256":hashlib.sha256(sb).hexdigest(),"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"workflow_identity_verified":True,"qualification_result":None,"qualification_authority":False}
    out["workflow_identity_verification_commitment"]=commit(VERIFY_DOMAIN,out); return out
def failure(c,r): return {"schema":SCHEMA,"workflow_identity_verifier_implementation_commitment":IMPLEMENTATION_COMMITMENT,"classification":c,"reason":r,"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"workflow_identity_verified":False,"qualification_result":None,"qualification_authority":False}
def main()->int:
    a=argparse.ArgumentParser(description=__doc__); a.add_argument("--repo",default="."); a.add_argument("--policy-result",required=True); a.add_argument("--intent",required=True); x=a.parse_args()
    try: out=verify(Path(x.repo),load(Path(x.policy_result),"004C policy result"),load(Path(x.intent),"registration intent"))
    except Refused as e: out=failure("REFUSED",str(e))
    except Unavailable as e: out=failure("UNAVAILABLE",str(e))
    except (OSError,Invalid) as e: out=failure("INVALID",str(e))
    print(json.dumps(out,sort_keys=True,separators=(",",":"),ensure_ascii=False)); return {"WORKFLOW_IDENTITY_VERIFIED":0,"REFUSED":2,"UNAVAILABLE":3,"INVALID":4}.get(out["classification"],4)
if __name__=="__main__": raise SystemExit(main())
