#!/usr/bin/env python3
"""Repository-binding policy for the EVIDENCE-CI-004D1 object oracle."""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, tempfile
from pathlib import Path
from types import ModuleType
from typing import Any

SCHEMA="mycelix-qualification-workflow-object-oracle-policy-v1"
POLICY_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_POLICY_V1\0"
IMPL_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_POLICY_IMPLEMENTATION_V1\0"
CORE_IMPL_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_IMPLEMENTATION_V1\0"
CORE_BLOB="13ac8bf1ce33e686a09b54ee97e2c1a6b9c4e736"
CORE_IMPL="bea3334619f757905e072182ded806f507faa6b1cee68080104fb6e0087b2390"
_IMPL_BYTES=Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT=hashlib.sha256(IMPL_DOMAIN+_IMPL_BYTES).hexdigest()
del _IMPL_BYTES

class Invalid(RuntimeError):pass
class Refused(RuntimeError):pass
class Unavailable(RuntimeError):pass

def git_blob(data:bytes)->str:
    return hashlib.sha1(b"blob "+str(len(data)).encode()+b"\0"+data).hexdigest()

def verify_core(path:Path,expected_blob:str=CORE_BLOB,expected_impl:str=CORE_IMPL)->bytes:
    try:data=path.read_bytes()
    except OSError as e:raise Unavailable("object-oracle core unavailable") from e
    if git_blob(data)!=expected_blob:raise Invalid("object-oracle core Git blob mismatch")
    if hashlib.sha256(CORE_IMPL_DOMAIN+data).hexdigest()!=expected_impl:raise Invalid("object-oracle core implementation mismatch")
    return data

def load_core()->tuple[ModuleType,Path]:
    path=Path(__file__).with_name("workflow_object_oracle.py")
    verify_core(path)
    spec=importlib.util.spec_from_file_location("workflow_object_oracle_core",path)
    if spec is None or spec.loader is None:raise Invalid("cannot construct object-oracle core loader")
    mod=importlib.util.module_from_spec(spec)
    try:spec.loader.exec_module(mod)
    except Exception as e:raise Invalid("cannot import pinned object-oracle core") from e
    if getattr(mod,"IMPLEMENTATION_COMMITMENT",None)!=CORE_IMPL:raise Invalid("loaded object-oracle core identity mismatch")
    return mod,path

def normalize_origin(raw:str)->str|None:
    s=raw.strip()
    for prefix in ("https://github.com/","http://github.com/","git://github.com/","ssh://git@github.com/"):
        if s.startswith(prefix):s=s[len(prefix):];break
    else:
        if s.startswith("git@github.com:"):s=s[len("git@github.com:"):]
        else:return None
    s=s.removesuffix(".git").strip("/")
    import re
    return s if re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+",s) else None

def map_core_error(core:ModuleType,e:Exception)->None:
    for name,kind in (("Refused",Refused),("Unavailable",Unavailable),("Invalid",Invalid)):
        cls=getattr(core,name,None)
        if cls is not None and isinstance(e,cls):raise kind(str(e)) from e
    raise e

def core_load(core:ModuleType,path:Path,label:str)->dict[str,Any]:
    try:return core.load(path,label)
    except Exception as e:map_core_error(core,e)

def observed_origin(core:ModuleType,repo:Path,wfid:dict[str,Any])->str:
    try:
        g=core.bind_git(wfid)
        with tempfile.TemporaryDirectory(prefix="mycelix-wfobj-policy-") as td:
            env=core.env(Path(td)/"home",Path(g["path"]))
            raw=core.run(g["path"],repo,env,["remote","get-url","origin"],True)
    except Exception as e:map_core_error(core,e)
    if raw is None:raise Refused("repository origin unavailable")
    try:value=raw.decode("utf-8","strict").strip()
    except UnicodeDecodeError as e:raise Invalid("repository origin is not UTF-8") from e
    normalized=normalize_origin(value)
    if normalized is None:raise Refused("repository origin is not supported GitHub identity")
    return normalized

def authority_ceiling(result:dict[str,Any])->None:
    if result.get("classification")!="WORKFLOW_OBJECT_CHAIN_CONFIRMED":raise Refused("object-oracle core did not confirm chain")
    for field in ("registration_authority","workflow_dispatched","qualification_authority"):
        if result.get(field) is not False:raise Invalid(f"object-oracle core broadened authority: {field}")
    if result.get("qualification_result") is not None:raise Invalid("object-oracle core attempted qualification result")
    for field in ("git_implementation_trust_verified","receipt_authenticity_verified","sha1_collision_resistance_claimed"):
        if result.get(field) is not False:raise Invalid(f"object-oracle core broadened trust claim: {field}")
    if result.get("workflow_identity_verified") is not True or result.get("workflow_object_chain_confirmed") is not True:
        raise Invalid("object-oracle core success markers missing")

def policy_commitment(value:dict[str,Any])->str:
    core,_=load_core()
    return hashlib.sha256(POLICY_DOMAIN+core.canonical(value)).hexdigest()

def evaluate_with_core(core:ModuleType,core_path:Path|None,repo:Path,wfid:dict[str,Any],intent:dict[str,Any])->dict[str,Any]:
    expected=intent.get("repository")
    if not isinstance(expected,str):raise Invalid("intent repository unavailable")
    before=observed_origin(core,repo,wfid)
    if before.lower()!=expected.lower():raise Refused("repository origin mismatch")
    try:inner=core.confirm(repo,wfid,intent)
    except Exception as e:map_core_error(core,e)
    if core_path is not None:verify_core(core_path)
    after=observed_origin(core,repo,wfid)
    if after.lower()!=expected.lower():raise Refused("repository origin changed during object oracle")
    authority_ceiling(inner)
    result=dict(inner)
    core_impl=result.get("workflow_object_oracle_implementation_commitment")
    if core_impl!=CORE_IMPL:raise Invalid("object-oracle result implementation identity mismatch")
    result["schema"]=SCHEMA
    result["workflow_object_oracle_policy_implementation_commitment"]=IMPLEMENTATION_COMMITMENT
    result["workflow_object_oracle_core_git_blob_sha1"]=CORE_BLOB
    result["workflow_object_oracle_core_implementation_commitment"]=CORE_IMPL
    result["repository_identity_verified"]=True
    result["repository_origin"]=after
    result["workflow_object_oracle_policy_commitment"]=policy_commitment(result)
    return result

def evaluate(repo:Path,wfid:dict[str,Any],intent:dict[str,Any])->dict[str,Any]:
    core,path=load_core()
    return evaluate_with_core(core,path,repo,wfid,intent)

def failure(classification:str,reason:str)->dict[str,Any]:
    return {"schema":SCHEMA,"workflow_object_oracle_policy_implementation_commitment":IMPLEMENTATION_COMMITMENT,
        "classification":classification,"reason":reason,"repository_identity_verified":False,
        "sha1_collision_resistance_claimed":False,"git_implementation_trust_verified":False,
        "receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,
        "workflow_identity_verified":False,"workflow_object_chain_confirmed":False,
        "qualification_result":None,"qualification_authority":False}

def main()->int:
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--repo",default=".");p.add_argument("--workflow-identity-result",required=True);p.add_argument("--intent",required=True);a=p.parse_args()
    try:
        core,path=load_core()
        wfid=core_load(core,Path(a.workflow_identity_result),"004D workflow identity result")
        intent=core_load(core,Path(a.intent),"registration intent")
        out=evaluate_with_core(core,path,Path(a.repo),wfid,intent)
    except Refused as e:out=failure("REFUSED",str(e))
    except Unavailable as e:out=failure("UNAVAILABLE",str(e))
    except (Invalid,OSError) as e:out=failure("INVALID",str(e))
    print(json.dumps(out,sort_keys=True,separators=(",",":"),ensure_ascii=False))
    return {"WORKFLOW_OBJECT_CHAIN_CONFIRMED":0,"REFUSED":2,"UNAVAILABLE":3,"INVALID":4}.get(out["classification"],4)
if __name__=="__main__":raise SystemExit(main())