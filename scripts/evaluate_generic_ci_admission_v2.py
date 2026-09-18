#!/usr/bin/env python3
"""Fail-closed generic Mycelix CI admission/job selection v2.

Pure policy logic: no GitHub calls. Unknown, oversized, malformed, partial, or
inconsistent observations select all jobs. Renames classify both filename and
previous_filename. AdmissionSkipped is scheduling evidence, never CI PASS.
"""
from __future__ import annotations
import argparse, copy, fnmatch, json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_ID="generic-mycelix-ci-admission-v2"
AUTHORITY="SchedulingOnly"
ALL_DISPOSITIONS={"AdmitMainPush","AdmitKnownRelevant","AdmitUnknown","AdmitUnknownLarge","AdmissionSkipped","AdmissionErrorFailClosed","AdmissionProfileInvalid"}
ALL_JOB_DISPOSITIONS={"AdmitMainPush","AdmitUnknown","AdmitUnknownLarge","AdmissionErrorFailClosed","AdmissionProfileInvalid"}
class ProfileError(ValueError): pass

def require(ok: bool, msg: str)->None:
    if not ok: raise ProfileError(msg)

def hex40(v:Any)->bool:
    return isinstance(v,str) and len(v)==40 and all(c in "0123456789abcdef" for c in v)

def valid_path(v:Any)->str:
    require(isinstance(v,str) and bool(v),"path must be non-empty string")
    require(not v.startswith("/") and "\\" not in v and "\x00" not in v,"path must be repo-relative POSIX")
    require(all(p not in {"",".",".."} for p in v.split("/")),f"unsafe path {v!r}")
    return v

def patterns(entries:Any, field:str)->list[str]:
    require(isinstance(entries,list) and entries,f"{field} must be non-empty list")
    out=[]
    for i,e in enumerate(entries):
        require(isinstance(e,dict),f"{field}[{i}] must be object")
        p=e.get("pattern"); r=e.get("reason")
        require(isinstance(p,str) and p and not p.startswith("/") and ".." not in p.split("/"),f"invalid {field}[{i}].pattern")
        require(isinstance(r,str) and r.strip(),f"invalid {field}[{i}].reason")
        out.append(p)
    require(len(out)==len(set(out)),f"duplicate {field} patterns")
    return out

def matches(path:str, pats:list[str])->bool: return any(fnmatch.fnmatchcase(path,p) for p in pats)
def all_jobs(p:dict[str,Any])->tuple[str,...]: return tuple(p["all_jobs"])

@dataclass(frozen=True)
class Admission:
    disposition:str; source:str; declared:int|None; observed:int|None
    paths:tuple[str,...]; classes:dict[str,str]; jobs:tuple[str,...]; reason:str
    def receipt(self,p:dict[str,Any])->dict[str,Any]:
        return {"profile_id":PROFILE_ID,"subject_commit":p.get("subject_commit"),"authority":AUTHORITY,
        "source":self.source,"declared_changed_files":self.declared,"observed_file_records":self.observed,
        "classified_paths":list(self.paths),"path_classes":self.classes,"disposition":self.disposition,
        "selected_jobs":list(self.jobs),"generic_ci_required":self.disposition!="AdmissionSkipped",
        "grants_ci_pass":False,"grants_product_qualification":False}

def validate_profile(p:Any,fixtures:bool=True)->None:
    require(isinstance(p,dict),"profile root must be object")
    require(p.get("profile_id")==PROFILE_ID and p.get("profile_version")==2,"profile identity/version drift")
    require(p.get("issue")==1619 and p.get("authority")==AUTHORITY,"issue/authority drift")
    require(p.get("matcher")=="PythonFnmatchCaseV1","matcher drift")
    require(hex40(p.get("subject_commit")),"invalid subject_commit")
    w=p.get("workflow"); require(isinstance(w,dict) and w.get("path")==".github/workflows/ci.yml" and hex40(w.get("blob_sha")),"workflow binding invalid")
    api=p.get("pull_files_api"); require(isinstance(api,dict),"pull_files_api missing")
    require(api.get("max_files")==3000 and api.get("page_size")==100,"API bounds drift")
    require(api.get("require_declared_observed_count_match") is True and api.get("classify_previous_filename_when_present") is True,"API safety theorem missing")
    require(p.get("selection_precedence")==["AdmissionErrorFailClosed","AdmitUnknownLarge","AdmitUnknown","AdmitKnownRelevant","AdmissionSkipped"],"selection precedence drift")
    require(p.get("unknown_policy")=="SelectAllJobs","unknown policy drift")
    jobs=p.get("all_jobs"); require(isinstance(jobs,list) and jobs and len(jobs)==len(set(jobs)),"all_jobs invalid")
    rel=patterns(p.get("known_relevant"),"known_relevant"); irr=patterns(p.get("proven_irrelevant"),"proven_irrelevant")
    require("docs/lex-net/**" in irr and "docs/**" not in irr and "**" not in irr,"skip surface drift")
    require(not set(rel)&set(irr),"exact relevant/irrelevant overlap")
    fan=p.get("full_fanout_patterns"); require(isinstance(fan,list) and fan and len(fan)==len(set(fan)) and all(x in rel for x in fan),"fanout invalid")
    closures=p.get("job_closures"); require(isinstance(closures,dict) and set(closures)==set(jobs),"job_closures mismatch")
    consumed=set(fan)
    for j,ps in closures.items():
        require(isinstance(ps,list) and ps and len(ps)==len(set(ps)),f"closure {j} invalid")
        require(all(x in rel for x in ps),f"closure {j} references non-relevant pattern"); consumed.update(ps)
    require(set(rel)<=consumed,"known relevant pattern has no consumer")
    nc="\n".join(map(str,p.get("nonclaims",[]))).lower()
    require(all(x in nc for x in ("scheduling","not ci pass","select all jobs","dependency graph")),"nonclaims weak")
    if fixtures:
        seen=set()
        for i,f in enumerate(p.get("fixtures",[])):
            require(isinstance(f,dict) and f.get("expect") in ALL_DISPOSITIONS,f"fixture {i} invalid")
            r=evaluate(p,f.get("event_name"),f.get("declared_changed_files"),f.get("files"))
            require(r.disposition==f["expect"],f"fixture {i}: {r.disposition} != {f['expect']}: {r.reason}")
            if "expect_jobs" in f: require(sorted(r.jobs)==sorted(f["expect_jobs"]),f"fixture {i} job drift")
            if r.disposition in ALL_JOB_DISPOSITIONS: require(r.jobs==all_jobs(p),f"fixture {i} did not select all jobs")
            seen.add(r.disposition)
        require({"AdmitMainPush","AdmitKnownRelevant","AdmitUnknown","AdmitUnknownLarge","AdmissionSkipped","AdmissionErrorFailClosed"}<=seen,"fixture coverage incomplete")

def err(p:dict[str,Any],decl:int|None,obs:int|None,reason:str)->Admission:
    return Admission("AdmissionErrorFailClosed","FailClosedFallback",decl,obs,(),{},all_jobs(p),reason)

def evaluate(p:dict[str,Any],event:Any,decl:Any,records:Any)->Admission:
    try: validate_profile(p,False)
    except ProfileError as e:
        jobs=tuple(p.get("all_jobs") or ()) if isinstance(p,dict) else ()
        return Admission("AdmissionProfileInvalid","FailClosedFallback",None,None,(),{},jobs,str(e))
    if event=="push": return Admission("AdmitMainPush","MainPush",None,None,(),{},all_jobs(p),"main push selects all jobs")
    if event!="pull_request": return err(p,None,None,f"unsupported event {event!r}")
    if not isinstance(decl,int) or isinstance(decl,bool) or decl<=0: return err(p,None,None,"declared_changed_files must be positive integer")
    if decl>p["pull_files_api"]["max_files"]: return Admission("AdmitUnknownLarge","FailClosedFallback",decl,None,(),{},all_jobs(p),"declared file count exceeds bounded completeness limit")
    if not isinstance(records,list): return err(p,decl,None,"file_records must be list")
    obs=len(records)
    if obs!=decl: return err(p,decl,obs,"declared/observed record count mismatch")
    current=[]; expanded=[]
    try:
        for i,rec in enumerate(records):
            require(isinstance(rec,dict),f"file_records[{i}] must be object")
            name=valid_path(rec.get("filename")); current.append(name); expanded.append(name)
            prev=rec.get("previous_filename")
            if prev is not None:
                prev=valid_path(prev); require(prev!=name,f"file_records[{i}] previous_filename equals filename"); expanded.append(prev)
        require(len(current)==len(set(current)),"duplicate current filename records")
    except ProfileError as e: return err(p,decl,obs,str(e))
    paths=tuple(dict.fromkeys(expanded)); rel=patterns(p["known_relevant"],"known_relevant"); irr=patterns(p["proven_irrelevant"],"proven_irrelevant")
    classes={x:("KnownRelevant" if matches(x,rel) else "ProvenIrrelevant" if matches(x,irr) else "Unknown") for x in paths}
    if "Unknown" in classes.values(): return Admission("AdmitUnknown","PullRequestFilesApi",decl,obs,paths,classes,all_jobs(p),"unknown path selects all jobs")
    known=[x for x,c in classes.items() if c=="KnownRelevant"]
    if known:
        if any(matches(x,p["full_fanout_patterns"]) for x in known): jobs=all_jobs(p); reason="control surface requires full fanout"
        else:
            chosen=set()
            for x in known:
                hit=False
                for j,ps in p["job_closures"].items():
                    if matches(x,ps): chosen.add(j); hit=True
                if not hit: return err(p,decl,obs,f"known relevant path has no closure: {x!r}")
            jobs=tuple(j for j in p["all_jobs"] if j in chosen); reason="known paths mapped to frozen closures"
        return Admission("AdmitKnownRelevant","PullRequestFilesApi",decl,obs,paths,classes,jobs,reason)
    return Admission("AdmissionSkipped","PullRequestFilesApi",decl,obs,paths,classes,(),"all current/previous paths proven irrelevant")

def self_test(p:dict[str,Any])->None:
    validate_profile(p)
    r=evaluate(p,"pull_request",2,[{"filename":"mycelix-finance/src/lib.rs"},{"filename":"future-shared/runtime/new.rs"}])
    require(r.disposition=="AdmitUnknown" and r.jobs==all_jobs(p),"unknown sibling must dominate selection")
    r=evaluate(p,"pull_request",1,[{"filename":"docs/lex-net/moved.md","previous_filename":"mycelix-governance/src/lib.rs"}])
    require(r.disposition=="AdmitKnownRelevant" and "test-governance" in r.jobs,"rename source relevance lost")
    o=copy.deepcopy(p); o["known_relevant"].append({"pattern":"docs/lex-net/runtime/**","reason":"synthetic overlap"}); o["full_fanout_patterns"].append("docs/lex-net/runtime/**"); validate_profile(o,False)
    require(evaluate(o,"pull_request",1,[{"filename":"docs/lex-net/runtime/x.rs"}]).disposition=="AdmitKnownRelevant","relevant must win per-path overlap")
    b=copy.deepcopy(p); b["unknown_policy"]="SkipUnknown"; require(evaluate(b,"pull_request",1,[{"filename":"docs/lex-net/a.md"}]).disposition=="AdmissionProfileInvalid","corrupt profile not fail-closed")

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("profile",nargs="?",default="docs/ci/generic_ci_admission_v2.json"); ap.add_argument("--event-name"); ap.add_argument("--declared-changed-files",type=int); ap.add_argument("--files-json"); ap.add_argument("--self-test",action="store_true"); a=ap.parse_args()
    try:
        p=json.loads(Path(a.profile).read_text()); validate_profile(p)
        if a.self_test:
            self_test(p); print(json.dumps({"profile_id":PROFILE_ID,"self_test":"PASS","authority":AUTHORITY,"grants_ci_pass":False,"grants_product_qualification":False},sort_keys=True)); return 0
        records=None if a.files_json is None else json.loads(a.files_json); r=evaluate(p,a.event_name,a.declared_changed_files,records); print(json.dumps(r.receipt(p),sort_keys=True)); return 0
    except (ProfileError,json.JSONDecodeError,OSError) as e:
        print(json.dumps({"profile_id":PROFILE_ID,"disposition":"AdmissionProfileInvalid","reason":str(e),"generic_ci_required":True,"grants_ci_pass":False,"grants_product_qualification":False},sort_keys=True)); return 2
if __name__=="__main__": raise SystemExit(main())
