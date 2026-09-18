#!/usr/bin/env python3
"""Aggregate evaluator for generic Mycelix CI after admission-v2 selection.

Required product failures are product failures. Admission/root ambiguity is not.
AdmissionSkipped must bypass ci-pass; if this evaluator is invoked for it, the
wiring fails closed instead of emitting a misleading green aggregate result.
"""
from __future__ import annotations
import argparse, copy, json, os
from dataclasses import dataclass
from typing import Any

PROFILE_ID="generic-mycelix-ci-required-checks-v2"
PRODUCTS={"commons":"test-commons","civic":"test-civic","hearth":"test-hearth","finance":"test-finance","governance":"test-governance","identity":"test-identity","personal":"test-personal","attribution":"test-attribution","bridge":"test-bridge","sdk":"test-sdk","prism":"test-prism"}
INFO={"finance_integration":"test-finance-integration"}
SELECTORS=("format",*PRODUCTS.keys(),*INFO.keys())
KNOWN_RESULTS={"success","failure","cancelled","skipped"}
FULL_FANOUT={"AdmitMainPush","AdmitUnknown","AdmitUnknownLarge","AdmissionErrorFailClosed","AdmissionProfileInvalid"}
KNOWN_ADMISSION=FULL_FANOUT|{"AdmitKnownRelevant","AdmissionSkipped"}

@dataclass(frozen=True)
class Evaluation:
    disposition:str; reason:str; exit_code:int; required:dict[str,str]; informational:dict[str,str]
    def receipt(self)->dict[str,Any]:
        return {"profile_id":PROFILE_ID,"disposition":self.disposition,"reason":self.reason,
                "required_results":self.required,"informational_results":self.informational,
                "required_checks_passed":self.disposition=="RequiredChecksPassed"}

def job(needs:dict[str,Any],name:str)->dict[str,Any]|None:
    v=needs.get(name); return v if isinstance(v,dict) else None

def result(needs:dict[str,Any],name:str)->str|None:
    j=job(needs,name); v=j.get("result") if j else None; return v if isinstance(v,str) else None

def fail(d:str,r:str,req:dict[str,str],info:dict[str,str],code:int=2)->Evaluation: return Evaluation(d,r,code,req,info)

def selector(outputs:dict[str,Any],key:str)->bool|None:
    v=outputs.get(key)
    return True if v=="true" else False if v=="false" else None

def check_selected(needs:dict[str,Any],name:str,selected:bool,req:dict[str,str],info:dict[str,str])->Evaluation|None:
    r=result(needs,name); req[name]=r or "missing"
    if r is None: return fail("RequiredPrerequisiteMissing",f"{name!r} has no result",req,info)
    if r not in KNOWN_RESULTS: return fail("RequiredPrerequisiteIndeterminate",f"{name!r} unknown result {r!r}",req,info)
    if selected:
        if r=="success": return None
        if r=="failure": return fail("RequiredCheckFailed",f"selected required job {name!r} failed",req,info,1)
        if r=="cancelled": return fail("RunCancelledOrSuperseded",f"selected required job {name!r} cancelled",req,info)
        return fail("RequiredPrerequisiteIndeterminate",f"selected required job {name!r} unexpectedly skipped",req,info)
    if r!="skipped": return fail("RequiredPrerequisiteIndeterminate",f"unselected required job {name!r} should be skipped, got {r!r}",req,info)
    return None

def evaluate(event:str,needs:dict[str,Any])->Evaluation:
    req={}; info={}
    if event not in {"pull_request","push"}: return fail("RequiredPrerequisiteIndeterminate",f"unsupported event {event!r}",req,info)
    cr=result(needs,"changes"); req["changes"]=cr or "missing"
    if cr is None: return fail("RequiredPrerequisiteMissing","changes has no result",req,info)
    if cr=="cancelled": return fail("RunCancelledOrSuperseded","admission root cancelled",req,info)
    if cr=="failure": return fail("AdmissionRootFailed","admission root failed; not a product failure",req,info)
    if cr!="success": return fail("RequiredPrerequisiteIndeterminate",f"changes unexpected result {cr!r}",req,info)
    cj=job(needs,"changes") or {}; outputs=cj.get("outputs")
    if not isinstance(outputs,dict): return fail("RequiredPrerequisiteMissing","changes has no outputs object",req,info)
    admission=outputs.get("disposition"); generic=outputs.get("generic_ci_required")
    if admission not in KNOWN_ADMISSION: return fail("RequiredPrerequisiteIndeterminate",f"unknown admission disposition {admission!r}",req,info)
    if generic not in {"true","false"}: return fail("RequiredPrerequisiteIndeterminate",f"generic_ci_required invalid {generic!r}",req,info)
    if admission=="AdmissionSkipped":
        if generic!="false": return fail("RequiredPrerequisiteIndeterminate","AdmissionSkipped must set generic_ci_required=false",req,info)
        if any(selector(outputs,k) is not False for k in SELECTORS): return fail("RequiredPrerequisiteIndeterminate","AdmissionSkipped must select zero jobs",req,info)
        return fail("UnexpectedAggregateExecutionForSkippedCi","ci-pass executed even though generic CI was not required",req,info)
    if generic!="true": return fail("RequiredPrerequisiteIndeterminate",f"{admission} must set generic_ci_required=true",req,info)
    selected={k:selector(outputs,k) for k in SELECTORS}
    bad=[k for k,v in selected.items() if v is None]
    if bad: return fail("RequiredPrerequisiteIndeterminate",f"invalid/missing selectors: {bad}",req,info)
    if admission in FULL_FANOUT and not all(selected.values()):
        return fail("RequiredPrerequisiteIndeterminate",f"fail-closed admission {admission} did not select all jobs",req,info)
    if event=="push" and (admission!="AdmitMainPush" or not all(selected.values())):
        return fail("RequiredPrerequisiteIndeterminate","push must be AdmitMainPush with full fanout",req,info)
    e=check_selected(needs,"format",bool(selected["format"]),req,info)
    if e: return e
    for key,name in PRODUCTS.items():
        e=check_selected(needs,name,bool(selected[key]),req,info)
        if e: return e
    for key,name in INFO.items():
        r=result(needs,name); info[key]=r or "missing"
        if r is None or r not in KNOWN_RESULTS: return fail("RequiredPrerequisiteIndeterminate",f"informational {name!r} has invalid result {r!r}",req,info)
        if not selected[key] and r!="skipped": return fail("InformationalSelectionDrift",f"unselected informational job {name!r} executed as {r!r}",req,info)
    return Evaluation("RequiredChecksPassed","all selected required jobs passed for admission-v2 graph",0,req,info)

def fixture(selected:set[str],admission="AdmitKnownRelevant",event="pull_request",info_result="success")->dict[str,Any]:
    full=admission in FULL_FANOUT or event=="push"
    outs={"disposition":"AdmitMainPush" if event=="push" else admission,"generic_ci_required":"true"}
    for k in SELECTORS: outs[k]="true" if full or k in selected else "false"
    n={"changes":{"result":"success","outputs":outs},"format":{"result":"success" if outs["format"]=="true" else "skipped","outputs":{}}}
    for k,name in PRODUCTS.items(): n[name]={"result":"success" if outs[k]=="true" else "skipped","outputs":{}}
    for k,name in INFO.items(): n[name]={"result":info_result if outs[k]=="true" else "skipped","outputs":{}}
    return n

def self_test()->None:
    n=fixture({"format","finance","finance_integration"}); assert evaluate("pull_request",n).disposition=="RequiredChecksPassed"
    n=fixture({"finance","finance_integration"}); assert result(n,"format")=="skipped" and evaluate("pull_request",n).disposition=="RequiredChecksPassed"
    n=fixture(set(),"AdmitUnknown"); assert evaluate("pull_request",n).disposition=="RequiredChecksPassed"
    n=fixture({"format","finance"}); n["format"]["result"]="failure"; assert evaluate("pull_request",n).disposition=="RequiredCheckFailed"
    n=fixture({"finance"}); n["test-finance"]["result"]="cancelled"; assert evaluate("pull_request",n).disposition=="RunCancelledOrSuperseded"
    n=fixture({"finance"}); n["test-finance"]["result"]="skipped"; assert evaluate("pull_request",n).disposition=="RequiredPrerequisiteIndeterminate"
    n=fixture(set()); n["test-sdk"]["result"]="success"; assert evaluate("pull_request",n).disposition=="RequiredPrerequisiteIndeterminate"
    n=fixture(set(),"AdmitUnknown"); n["changes"]["outputs"]["sdk"]="false"; assert evaluate("pull_request",n).disposition=="RequiredPrerequisiteIndeterminate"
    n=fixture({"finance_integration"},info_result="failure"); assert evaluate("pull_request",n).disposition=="RequiredChecksPassed"
    n=fixture(set()); n["test-finance-integration"]["result"]="success"; assert evaluate("pull_request",n).disposition=="InformationalSelectionDrift"
    n=fixture(set()); n["changes"]["result"]="failure"; assert evaluate("pull_request",n).disposition=="AdmissionRootFailed"
    n=fixture(set()); n["changes"]["outputs"]={"disposition":"AdmissionSkipped","generic_ci_required":"false",**{k:"false" for k in SELECTORS}}; assert evaluate("pull_request",n).disposition=="UnexpectedAggregateExecutionForSkippedCi"
    n=fixture(set(),event="push"); assert evaluate("push",n).disposition=="RequiredChecksPassed"
    n=fixture(set(),event="push"); n["changes"]["outputs"]["sdk"]="false"; n["test-sdk"]["result"]="skipped"; assert evaluate("push",n).disposition=="RequiredPrerequisiteIndeterminate"

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("--self-test",action="store_true"); ap.add_argument("--event-name",default=os.getenv("GITHUB_EVENT_NAME","")); ap.add_argument("--needs-json",default=os.getenv("MYCELIX_CI_NEEDS_JSON","")); a=ap.parse_args()
    if a.self_test: self_test(); print(json.dumps({"profile_id":PROFILE_ID,"self_test":"PASS"},sort_keys=True)); return 0
    if not a.needs_json: print(json.dumps({"profile_id":PROFILE_ID,"disposition":"RequiredPrerequisiteMissing","required_checks_passed":False,"reason":"needs JSON empty"},sort_keys=True)); return 2
    try: n=json.loads(a.needs_json)
    except json.JSONDecodeError as e: print(json.dumps({"profile_id":PROFILE_ID,"disposition":"RequiredPrerequisiteIndeterminate","required_checks_passed":False,"reason":str(e)},sort_keys=True)); return 2
    if not isinstance(n,dict): print(json.dumps({"profile_id":PROFILE_ID,"disposition":"RequiredPrerequisiteIndeterminate","required_checks_passed":False,"reason":"needs root must be object"},sort_keys=True)); return 2
    r=evaluate(a.event_name,n); print(json.dumps(r.receipt(),sort_keys=True)); return r.exit_code
if __name__=="__main__": raise SystemExit(main())
