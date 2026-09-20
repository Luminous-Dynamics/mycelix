#!/usr/bin/env python3
"""Independent SHA-1 Git object-chain oracle for EVIDENCE-CI-004D1."""
from __future__ import annotations
import argparse, hashlib, json, os, re, subprocess, tempfile
from pathlib import Path
from typing import Any

SCHEMA="mycelix-qualification-workflow-object-oracle-v1"
WFID_SCHEMA="mycelix-qualification-workflow-identity-v1"
WFID_IMPL="f932a24a11bb8d98c55b039fb0a1ddc951eacd64a159c03826d4af0675480761"
WFID_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_IDENTITY_V1\0"
INTENT_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_INTENT_V1\0"
ORACLE_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_V1\0"
IMPL_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_IMPLEMENTATION_V1\0"
MAX_JSON=MAX_OBJECT=MAX_OUTPUT=8*1024*1024; SAFE=9007199254740991
OID=re.compile(r"^[0-9a-f]{40}$"); H64=re.compile(r"^[0-9a-f]{64}$")
REPO=re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
WFP=re.compile(r"^\.github/workflows/[A-Za-z0-9._/-]+\.(?:yml|yaml)$")
INTENT_FIELDS=set("schema intent_id repository subject_sha predecessor_sha preflight_profile_id preflight_profile_commitment preflight_implementation_commitment environment_adapter_implementation_commitment preflight_environment_commitment qualification_workflow_path qualification_workflow_commit_sha qualification_workflow_blob_sha1 registration_mode".split())
WFID_FIELDS=set("schema workflow_identity_verifier_implementation_commitment classification registration_admission_policy_commitment registration_intent_commitment repository subject_sha predecessor_sha qualification_workflow_path qualification_workflow_commit_sha qualification_workflow_blob_sha1 git tree_entry caller_head caller_status_sha256 receipt_authenticity_verified registration_authority workflow_dispatched workflow_identity_verified qualification_result qualification_authority workflow_identity_verification_commitment".split())
GIT_FIELDS=set("path sha256 version object_format".split()); ENTRY_FIELDS=set("mode type oid path".split())
MODES={b"40000",b"100644",b"100755",b"120000",b"160000"}
_B=Path(__file__).read_bytes(); IMPLEMENTATION_COMMITMENT=hashlib.sha256(IMPL_DOMAIN+_B).hexdigest(); del _B
class Invalid(RuntimeError): pass
class Refused(RuntimeError): pass
class Unavailable(RuntimeError): pass

def _walk(v:Any)->None:
    if isinstance(v,float): raise Invalid("float in commitment input")
    if isinstance(v,int) and not isinstance(v,bool) and abs(v)>SAFE: raise Invalid("unsafe integer in commitment input")
    if isinstance(v,dict):
        for k,x in v.items():
            if not isinstance(k,str): raise Invalid("non-string key in commitment input")
            _walk(x)
    elif isinstance(v,list):
        for x in v:_walk(x)
def canonical(v:Any)->bytes:
    _walk(v); return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode()
def commitment(domain:bytes,v:Any)->str:return hashlib.sha256(domain+canonical(v)).hexdigest()
def _reject(x:str):raise Invalid(f"non-I-JSON constant: {x}")
def load(path:Path,label:str)->dict[str,Any]:
    try:b=path.read_bytes()
    except OSError as e:raise Invalid(f"cannot read {label}") from e
    if len(b)>MAX_JSON:raise Invalid(f"{label} exceeds size bound")
    def hook(ps):
        d={}
        for k,v in ps:
            if k in d:raise Invalid(f"duplicate JSON key in {label}: {k}")
            d[k]=v
        return d
    try:v=json.loads(b.decode("utf-8","strict"),object_pairs_hook=hook,parse_constant=_reject)
    except (UnicodeDecodeError,json.JSONDecodeError) as e:raise Invalid(f"invalid {label}") from e
    if not isinstance(v,dict):raise Invalid(f"{label} must be object")
    return v
def oid(v:object,label:str)->str:
    if not isinstance(v,str) or not OID.fullmatch(v):raise Invalid(f"invalid {label}")
    return v
def h64(v:object,label:str)->str:
    if not isinstance(v,str) or not H64.fullmatch(v):raise Invalid(f"invalid {label}")
    return v

def validate_intent(i:dict[str,Any])->str:
    if set(i)!=INTENT_FIELDS or i.get("schema")!="mycelix-qualification-registration-intent-v1":raise Invalid("registration intent shape/schema mismatch")
    if not isinstance(i.get("repository"),str) or not REPO.fullmatch(i["repository"]):raise Invalid("invalid repository")
    for f in ("subject_sha","predecessor_sha","qualification_workflow_commit_sha","qualification_workflow_blob_sha1"):oid(i.get(f),f)
    for f in ("preflight_profile_commitment","preflight_implementation_commitment","environment_adapter_implementation_commitment","preflight_environment_commitment"):h64(i.get(f),f)
    p=i.get("qualification_workflow_path")
    if not isinstance(p,str) or not WFP.fullmatch(p) or "//" in p or "/./" in p or "/../" in p:raise Invalid("invalid workflow path")
    if i.get("registration_mode")!="manual-request-v1" or not i.get("intent_id") or not i.get("preflight_profile_id"):raise Invalid("invalid intent metadata")
    return commitment(INTENT_DOMAIN,i)

def validate_wfid(r:dict[str,Any],i:dict[str,Any],ic:str)->str:
    if set(r)!=WFID_FIELDS or r.get("schema")!=WFID_SCHEMA:raise Invalid("004D result shape/schema mismatch")
    if r.get("workflow_identity_verifier_implementation_commitment")!=WFID_IMPL:raise Invalid("unsupported 004D implementation")
    if r.get("classification")!="WORKFLOW_IDENTITY_VERIFIED":raise Refused("004D result is not verified")
    for f in ("registration_authority","workflow_dispatched","qualification_authority"):
        if r.get(f) is not False:raise Invalid(f"004D broadened authority: {f}")
    if r.get("workflow_identity_verified") is not True:raise Invalid("004D did not assert workflow identity")
    if r.get("qualification_result") is not None:raise Invalid("004D attempted qualification result")
    if r.get("receipt_authenticity_verified") is not False:raise Invalid("004D provenance boundary mismatch")
    if r.get("registration_intent_commitment")!=ic:raise Refused("004D intent commitment mismatch")
    wc=h64(r.get("workflow_identity_verification_commitment"),"004D verification commitment"); x=dict(r);x.pop("workflow_identity_verification_commitment")
    if commitment(WFID_DOMAIN,x)!=wc:raise Invalid("004D verification commitment mismatch")
    for f in ("repository","subject_sha","predecessor_sha","qualification_workflow_path","qualification_workflow_commit_sha","qualification_workflow_blob_sha1"):
        if r.get(f)!=i.get(f):raise Refused(f"004D field mismatch: {f}")
    g=r.get("git")
    if not isinstance(g,dict) or set(g)!=GIT_FIELDS or not isinstance(g.get("path"),str) or not Path(g["path"]).is_absolute():raise Invalid("004D Git identity shape mismatch")
    h64(g.get("sha256"),"004D Git SHA-256")
    if not isinstance(g.get("version"),str) or not g["version"].startswith("git version "):raise Invalid("invalid 004D Git version")
    if g.get("object_format")!="sha1":raise Unavailable("004D used unsupported Git object format")
    e=r.get("tree_entry")
    expected={"mode":"100644","type":"blob","oid":i["qualification_workflow_blob_sha1"],"path":i["qualification_workflow_path"]}
    if not isinstance(e,dict) or set(e)!=ENTRY_FIELDS:raise Invalid("004D tree-entry shape mismatch")
    if e!=expected:raise Refused("004D tree entry does not match intent")
    oid(r.get("caller_head"),"004D caller HEAD");h64(r.get("caller_status_sha256"),"004D caller status")
    return wc

def sha256_file(p:Path)->str:
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()
def bind_git(w:dict[str,Any])->dict[str,str]:
    g=w["git"];p=Path(g["path"])
    try:r=p.resolve(strict=True)
    except (OSError,RuntimeError) as e:raise Unavailable("004D Git executable unavailable") from e
    if str(r)!=g["path"] or not r.is_file() or not os.access(r,os.X_OK):raise Unavailable("004D Git executable identity no longer resolves")
    if sha256_file(r)!=g["sha256"]:raise Invalid("004D Git executable content drift")
    return dict(g)
def git_same(g:dict[str,str])->bool:
    try:p=Path(g["path"]);return p.resolve(strict=True)==p and p.is_file() and os.access(p,os.X_OK) and sha256_file(p)==g["sha256"]
    except (OSError,RuntimeError,KeyError):return False
def env(home:Path,gp:Path)->dict[str,str]:
    home.mkdir(parents=True,exist_ok=True);return {"PATH":str(gp.parent),"HOME":str(home),"LC_ALL":"C","LANG":"C","TZ":"UTC","GIT_CONFIG_NOSYSTEM":"1","GIT_CONFIG_GLOBAL":os.devnull,"GIT_TERMINAL_PROMPT":"0","GIT_OPTIONAL_LOCKS":"0","GIT_NO_LAZY_FETCH":"1","GIT_NO_REPLACE_OBJECTS":"1"}
def run(g:str,repo:Path,e:dict[str,str],args:list[str],fail=False)->bytes|None:
    a=[g,"-c","core.fsmonitor=false","-c",f"core.hooksPath={os.devnull}","-c","diff.external=","-C",str(repo),*args]
    try:c=subprocess.run(a,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=e,timeout=30)
    except (OSError,subprocess.TimeoutExpired) as x:raise Unavailable(f"Git transport unavailable: {args[0]}") from x
    if len(c.stdout)>MAX_OUTPUT or len(c.stderr)>MAX_OUTPUT:raise Unavailable("Git transport output exceeded bound")
    if c.returncode:
        if fail:return None
        raise Unavailable(f"Git transport failed: {args[0]}")
    return c.stdout
def obj_sha1(t:str,b:bytes)->str:return hashlib.sha1(f"{t} {len(b)}\0".encode()+b).hexdigest()
def obj(g:str,repo:Path,e:dict[str,str],t:str,o:str)->bytes:
    oid(o,f"{t} object id");b=run(g,repo,e,["cat-file",t,o],True)
    if b is None:raise Unavailable(f"{t} object unavailable locally")
    if len(b)>MAX_OBJECT:raise Unavailable(f"{t} object exceeds size bound")
    if obj_sha1(t,b)!=o:raise Invalid(f"{t} transport bytes do not hash to requested object")
    return b
def root_tree(b:bytes)->str:
    x=b.split(b"\n",1)[0]
    if not x.startswith(b"tree "):raise Invalid("commit object does not begin with tree header")
    try:s=x[5:].decode("ascii")
    except UnicodeDecodeError as e:raise Invalid("commit tree id is not ASCII") from e
    return oid(s,"commit root tree id")
def parse_tree(b:bytes)->dict[bytes,tuple[bytes,str]]:
    d={};i=0
    while i<len(b):
        sp=b.find(b" ",i);nul=b.find(b"\0",sp+1)
        if sp<=i:raise Invalid("malformed tree mode")
        if nul<=sp+1:raise Invalid("malformed tree name")
        if nul+21>len(b):raise Invalid("truncated tree object id")
        mode,name,ro=b[i:sp],b[sp+1:nul],b[nul+1:nul+21]
        if mode not in MODES:raise Invalid("unsupported tree mode")
        if b"/" in name or name in {b".",b".."}:raise Invalid("invalid tree entry name")
        if name in d:raise Invalid("duplicate tree entry name")
        d[name]=(mode,ro.hex());i=nul+21
    return d
def traverse(g:str,repo:Path,e:dict[str,str],root:str,path:str)->tuple[str,list[dict[str,str]]]:
    cur=root;obs=[];prefix=[];parts=path.split("/")
    for n,comp in enumerate(parts):
        b=obj(g,repo,e,"tree",cur);obs.append({"tree_sha1":cur,"tree_sha256":hashlib.sha256(b).hexdigest(),"path_prefix":"/".join(prefix)})
        ent=parse_tree(b);name=comp.encode()
        if name not in ent:raise Refused(f"workflow path component missing: {comp}")
        mode,o=ent[name];final=n==len(parts)-1
        if not final:
            if mode!=b"40000":raise Refused(f"workflow intermediate component is not tree: {comp}")
            cur=o;prefix.append(comp);continue
        if mode!=b"100644":raise Refused("workflow final component is not canonical 100644 blob")
        return o,obs
    raise Invalid("workflow path traversal produced no final component")
def head(g:str,r:Path,e:dict[str,str])->str:
    b=run(g,r,e,["rev-parse","--verify","HEAD"]);assert b is not None;return oid(b.decode("ascii").strip(),"caller HEAD")
def status(g:str,r:Path,e:dict[str,str])->bytes:
    b=run(g,r,e,["status","--porcelain=v1","-z","--untracked-files=all","--ignored=matching"]);assert b is not None;return b

def confirm(repo:Path,w:dict[str,Any],i:dict[str,Any])->dict[str,Any]:
    ic=validate_intent(i);wc=validate_wfid(w,i,ic)
    if not repo.is_dir():raise Unavailable("repository unavailable")
    g=bind_git(w)
    with tempfile.TemporaryDirectory(prefix="mycelix-wfobj-") as td:
        e=env(Path(td)/"home",Path(g["path"]));vr=run(g["path"],repo,e,["--version"]);fr=run(g["path"],repo,e,["rev-parse","--show-object-format"]);assert vr and fr
        if vr.decode("utf-8","strict").strip()!=g["version"]:raise Invalid("Git version drift from 004D observation")
        if fr.decode("ascii").strip()!="sha1":raise Unavailable("unsupported Git object format")
        bh,bs=head(g["path"],repo,e),status(g["path"],repo,e)
        if not git_same(g):raise Invalid("Git executable drift before object oracle")
        cid=i["qualification_workflow_commit_sha"];cb=obj(g["path"],repo,e,"commit",cid);rt=root_tree(cb);ob,trees=traverse(g["path"],repo,e,rt,i["qualification_workflow_path"])
        if ob!=i["qualification_workflow_blob_sha1"]:raise Refused("independent tree traversal blob differs from intent")
        if ob!=w["tree_entry"]["oid"]:raise Refused("independent tree traversal blob differs from 004D")
        bb=obj(g["path"],repo,e,"blob",ob)
        if not git_same(g):raise Invalid("Git executable drift after object oracle")
        if head(g["path"],repo,e)!=bh:raise Invalid("caller HEAD changed during object oracle")
        if status(g["path"],repo,e)!=bs:raise Invalid("caller status changed during object oracle")
    out={"schema":SCHEMA,"workflow_object_oracle_implementation_commitment":IMPLEMENTATION_COMMITMENT,"classification":"WORKFLOW_OBJECT_CHAIN_CONFIRMED","workflow_identity_verifier_implementation_commitment":WFID_IMPL,"workflow_identity_verification_commitment":wc,"registration_intent_commitment":ic,"repository":i["repository"],"subject_sha":i["subject_sha"],"predecessor_sha":i["predecessor_sha"],"qualification_workflow_path":i["qualification_workflow_path"],"qualification_workflow_commit_sha":cid,"commit_bytes_sha256":hashlib.sha256(cb).hexdigest(),"root_tree_sha1":rt,"traversed_trees":trees,"qualification_workflow_blob_sha1":ob,"workflow_blob_bytes_sha256":hashlib.sha256(bb).hexdigest(),"object_verification_method":"python-git-object-rehash-and-tree-traversal-v1","git":g,"caller_head":bh,"caller_status_sha256":hashlib.sha256(bs).hexdigest(),"sha1_collision_resistance_claimed":False,"git_implementation_trust_verified":False,"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"workflow_identity_verified":True,"workflow_object_chain_confirmed":True,"qualification_result":None,"qualification_authority":False}
    out["workflow_object_oracle_commitment"]=commitment(ORACLE_DOMAIN,out);return out
def failure(c:str,r:str)->dict[str,Any]:return {"schema":SCHEMA,"workflow_object_oracle_implementation_commitment":IMPLEMENTATION_COMMITMENT,"classification":c,"reason":r,"sha1_collision_resistance_claimed":False,"git_implementation_trust_verified":False,"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"workflow_identity_verified":False,"workflow_object_chain_confirmed":False,"qualification_result":None,"qualification_authority":False}
def main()->int:
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--repo",default=".");p.add_argument("--workflow-identity-result",required=True);p.add_argument("--intent",required=True);a=p.parse_args()
    try:o=confirm(Path(a.repo),load(Path(a.workflow_identity_result),"004D workflow identity result"),load(Path(a.intent),"registration intent"))
    except Refused as e:o=failure("REFUSED",str(e))
    except Unavailable as e:o=failure("UNAVAILABLE",str(e))
    except (Invalid,OSError) as e:o=failure("INVALID",str(e))
    print(json.dumps(o,sort_keys=True,separators=(",",":"),ensure_ascii=False));return {"WORKFLOW_OBJECT_CHAIN_CONFIRMED":0,"REFUSED":2,"UNAVAILABLE":3,"INVALID":4}.get(o["classification"],4)
if __name__=="__main__":raise SystemExit(main())