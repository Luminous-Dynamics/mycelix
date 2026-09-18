#!/usr/bin/env python3
"""Fail-closed generic Mycelix CI admission/job selection v2."""
from __future__ import annotations
import argparse, copy, fnmatch, json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_ID="generic-mycelix-ci-admission-v2"; AUTHORITY="SchedulingOnly"
REQUIRED=("format","test-commons","test-civic","test-hearth","test-finance","test-governance","test-identity","test-personal","test-attribution","test-bridge","test-sdk","test-prism")
INFO=("test-finance-integration",)
ALL=("format","test-commons","test-civic","test-hearth","test-finance","test-finance-integration","test-governance","test-identity","test-personal","test-attribution","test-bridge","test-sdk","test-prism")
DISPOSITIONS={"AdmitMainPush","AdmitKnownRelevant","AdmitUnknown","AdmitUnknownLarge","AdmissionSkipped","AdmissionErrorFailClosed","AdmissionProfileInvalid"}
REQ_FANOUT={"AdmitUnknown","AdmitUnknownLarge","AdmissionErrorFailClosed","AdmissionProfileInvalid"}
class ProfileError(ValueError): pass
def req(ok:bool,msg:str)->None:
    if not ok: raise ProfileError(msg)
def h40(v:Any)->bool: return isinstance(v,str) and len(v)==40 and all(c in "0123456789abcdef" for c in v)
def path(v:Any)->str:
    req(isinstance(v,str) and v,"path must be non-empty string")
    req(not v.startswith("/") and "\\" not in v and "\x00" not in v,"path must be repo-relative POSIX")
    req(all(p not in {"",".",".."} for p in v.split("/")),f"unsafe path {v!r}")
    return v
def pats(entries:Any,name:str)->list[str]:
    req(isinstance(entries,list) and entries,f"{name} must be non-empty list"); out=[]
    for i,e in enumerate(entries):
        req(isinstance(e,dict),f"{name}[{i}] must be object"); p=e.get("pattern"); r=e.get("reason")
        req(isinstance(p,str) and p and not p.startswith("/") and ".." not in p.split("/"),f"invalid {name}[{i}].pattern")
        req(isinstance(r,str) and r.strip(),f"invalid {name}[{i}].reason"); out.append(p)
    req(len(out)==len(set(out)),f"duplicate {name} patterns"); return out
def match(p:str,patterns:list[str])->bool: return any(fnmatch.fnmatchcase(p,x) for x in patterns)

@dataclass(frozen=True)
class Admission:
    disposition:str; source:str; declared:int|None; observed:int|None; paths:tuple[str,...]; classes:dict[str,str]; jobs:tuple[str,...]; reason:str
    def receipt(self,profile:dict[str,Any])->dict[str,Any]:
        return {"profile_id":PROFILE_ID,"baseline_commit":profile.get("baseline_commit"),"authority":AUTHORITY,"source":self.source,
        "declared_changed_files":self.declared,"observed_file_records":self.observed,"classified_paths":list(self.paths),"path_classes":self.classes,
        "disposition":self.disposition,"selected_jobs":list(self.jobs),"generic_ci_required":self.disposition!="AdmissionSkipped",
        "grants_ci_pass":False,"grants_product_qualification":False}

def validate(p:Any,fixtures:bool=True)->None:
    req(isinstance(p,dict),"profile root must be object")
    req(p.get("profile_id")==PROFILE_ID and p.get("profile_version")==2,"profile identity/version drift")
    req(p.get("issue")==1619 and p.get("authority")==AUTHORITY and p.get("matcher")=="PythonFnmatchCaseV1","authority/matcher drift")
    req(h40(p.get("baseline_commit")),"invalid baseline_commit")
    w=p.get("workflow"); req(isinstance(w,dict) and w.get("path")==".github/workflows/ci.yml" and h40(w.get("blob_sha")),"workflow binding invalid")
    api=p.get("pull_files_api"); req(isinstance(api,dict) and api.get("max_files")==3000 and api.get("page_size")==100,"API bounds drift")
    req(api.get("require_declared_observed_count_match") is True and api.get("classify_previous_filename_when_present") is True,"API safety theorem missing")
    req(p.get("selection_precedence")==["AdmissionErrorFailClosed","AdmitUnknownLarge","AdmitUnknown","AdmitKnownRelevant","AdmissionSkipped"],"selection precedence drift")
    req(p.get("unknown_policy")=="SelectAllRequiredJobs","unknown policy drift")
    req(tuple(p.get("required_jobs") or ())==REQUIRED and tuple(p.get("informational_jobs") or ())==INFO and tuple(p.get("all_jobs") or ())==ALL,"job partition drift")
    rel=pats(p.get("known_relevant"),"known_relevant"); irr=pats(p.get("proven_irrelevant"),"proven_irrelevant")
    req("docs/lex-net/**" in irr and "docs/**" not in irr and "**" not in irr and not set(rel)&set(irr),"skip surface drift")
    fan=p.get("full_fanout_patterns"); req(isinstance(fan,list) and fan and len(fan)==len(set(fan)) and all(x in rel for x in fan),"fanout invalid")
    closures=p.get("job_closures"); req(isinstance(closures,dict) and tuple(closures)==ALL,"job_closures mismatch"); consumed=set(fan)
    for j,ps in closures.items():
        req(isinstance(ps,list) and ps and len(ps)==len(set(ps)) and all(x in rel for x in ps),f"closure {j} invalid"); consumed.update(ps)
    req(set(rel)<=consumed,"known relevant pattern has no consumer")
    nc="\n".join(map(str,p.get("nonclaims",[]))).lower()
    req(all(x in nc for x in ("scheduling","not ci pass","all required jobs","informational","dependency graph")),"nonclaims weak")
    if fixtures:
        seen=set()
        for i,f in enumerate(p.get("fixtures",[])):
            req(isinstance(f,dict) and f.get("expect") in DISPOSITIONS,f"fixture {i} invalid")
            r=evaluate(p,f.get("event_name"),f.get("declared_changed_files"),f.get("files"))
            req(r.disposition==f["expect"],f"fixture {i}: {r.disposition} != {f['expect']}: {r.reason}")
            if "expect_jobs" in f: req(sorted(r.jobs)==sorted(f["expect_jobs"]),f"fixture {i} job drift")
            if r.disposition in REQ_FANOUT: req(r.jobs==REQUIRED,f"fixture {i} did not select all required jobs")
            if r.disposition=="AdmitMainPush": req(r.jobs==ALL,f"fixture {i} did not select all jobs")
            seen.add(r.disposition)
        req({"AdmitMainPush","AdmitKnownRelevant","AdmitUnknown","AdmitUnknownLarge","AdmissionSkipped","AdmissionErrorFailClosed"}<=seen,"fixture coverage incomplete")

def fallback(decl:int|None,obs:int|None,reason:str)->Admission:
    return Admission("AdmissionErrorFailClosed","FailClosedFallback",decl,obs,(),{},REQUIRED,reason)

def evaluate(p:dict[str,Any],event:Any,decl:Any,records:Any)->Admission:
    try: validate(p,False)
    except ProfileError as e: return Admission("AdmissionProfileInvalid","FailClosedFallback",None,None,(),{},REQUIRED,str(e))
    if event=="push": return Admission("AdmitMainPush","MainPush",None,None,(),{},ALL,"main push preserves required plus informational backstop")
    if event!="pull_request": return fallback(None,None,f"unsupported event {event!r}")
    if not isinstance(decl,int) or isinstance(decl,bool) or decl<=0: return fallback(None,None,"declared_changed_files must be positive integer")
    if decl>p["pull_files_api"]["max_files"]: return Admission("AdmitUnknownLarge","FailClosedFallback",decl,None,(),{},REQUIRED,"declared file count exceeds bounded completeness limit")
    if not isinstance(records,list): return fallback(decl,None,"file_records must be list")
    obs=len(records)
    if obs!=decl: return fallback(decl,obs,"declared/observed record count mismatch")
    current=[]; expanded=[]
    try:
        for i,rec in enumerate(records):
            req(isinstance(rec,dict),f"file_records[{i}] must be object"); name=path(rec.get("filename")); current.append(name); expanded.append(name)
            prev=rec.get("previous_filename")
            if prev is not None: prev=path(prev); req(prev!=name,f"file_records[{i}] previous_filename equals filename"); expanded.append(prev)
        req(len(current)==len(set(current)),"duplicate current filename records")
    except ProfileError as e: return fallback(decl,obs,str(e))
    paths=tuple(dict.fromkeys(expanded)); rel=pats(p["known_relevant"],"known_relevant"); irr=pats(p["proven_irrelevant"],"proven_irrelevant")
    classes={x:("KnownRelevant" if match(x,rel) else "ProvenIrrelevant" if match(x,irr) else "Unknown") for x in paths}
    if "Unknown" in classes.values(): return Admission("AdmitUnknown","PullRequestFilesApi",decl,obs,paths,classes,REQUIRED,"unknown path selects all required jobs; informational observers remain unselected")
    known=[x for x,c in classes.items() if c=="KnownRelevant"]
    if known:
        if any(match(x,p["full_fanout_patterns"]) for x in known): jobs=ALL; reason="CI control surface requires full required + informational fanout"
        else:
            chosen=set()
            for x in known:
                hit=False
                for j,ps in p["job_closures"].items():
                    if match(x,ps): chosen.add(j); hit=True
                if not hit: return fallback(decl,obs,f"known relevant path has no closure: {x!r}")
            jobs=tuple(j for j in ALL if j in chosen); reason="known paths mapped to frozen closures"
        return Admission("AdmitKnownRelevant","PullRequestFilesApi",decl,obs,paths,classes,jobs,reason)
    return Admission("AdmissionSkipped","PullRequestFilesApi",decl,obs,paths,classes,(),"all current/previous paths proven irrelevant to generic CI")

def self_test(p:dict[str,Any])->None:
    validate(p)
    r=evaluate(p,"pull_request",2,[{"filename":"mycelix-finance/src/lib.rs"},{"filename":"future-shared/runtime/new.rs"}])
    req(r.disposition=="AdmitUnknown" and r.jobs==REQUIRED and "test-finance-integration" not in r.jobs,"unknown sibling fanout wrong")
    r=evaluate(p,"pull_request",1,[{"filename":"docs/lex-net/moved.md","previous_filename":"mycelix-governance/src/lib.rs"}])
    req(r.disposition=="AdmitKnownRelevant" and "test-governance" in r.jobs,"rename source relevance lost")
    o=copy.deepcopy(p); o["known_relevant"].append({"pattern":"docs/lex-net/runtime/**","reason":"synthetic overlap"}); o["full_fanout_patterns"].append("docs/lex-net/runtime/**"); validate(o,False)
    req(evaluate(o,"pull_request",1,[{"filename":"docs/lex-net/runtime/x.rs"}]).disposition=="AdmitKnownRelevant","relevant must win overlap")
    b=copy.deepcopy(p); b["unknown_policy"]="SkipUnknown"; r=evaluate(b,"pull_request",1,[{"filename":"docs/lex-net/a.md"}])
    req(r.disposition=="AdmissionProfileInvalid" and r.jobs==REQUIRED,"corrupt profile not fail-closed")

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("profile",nargs="?",default="docs/ci/generic_ci_admission_v2.json"); ap.add_argument("--event-name"); ap.add_argument("--declared-changed-files",type=int); ap.add_argument("--files-json"); ap.add_argument("--self-test",action="store_true"); a=ap.parse_args()
    try:
        p=json.loads(Path(a.profile).read_text()); validate(p)
        if a.self_test: self_test(p); print(json.dumps({"profile_id":PROFILE_ID,"self_test":"PASS","authority":AUTHORITY,"grants_ci_pass":False,"grants_product_qualification":False},sort_keys=True)); return 0
        records=None if a.files_json is None else json.loads(a.files_json); r=evaluate(p,a.event_name,a.declared_changed_files,records); print(json.dumps(r.receipt(p),sort_keys=True)); return 0
    except (ProfileError,json.JSONDecodeError,OSError) as e:
        print(json.dumps({"profile_id":PROFILE_ID,"disposition":"AdmissionProfileInvalid","reason":str(e),"selected_jobs":list(REQUIRED),"generic_ci_required":True,"grants_ci_pass":False,"grants_product_qualification":False},sort_keys=True)); return 2
if __name__=="__main__": raise SystemExit(main())
