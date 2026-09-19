#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, os, sys, time, urllib.error, urllib.request
from datetime import datetime, timezone
from typing import Any, Callable

OWNER="Luminous-Dynamics"
REPO="mycelix"
GROUP="mycelix-heavy-qualification-v1"
API_VERSION="2026-03-10"
API_URL=f"https://api.github.com/repos/{OWNER}/{REPO}/actions/concurrency_groups/{GROUP}"
GROUP_URL=API_URL
MAX_BODY_BYTES=1_048_576
SCHEMA="mycelix.ci-gov.001k.capacity-observer.v0.1"
ALLOWED_STATUSES={"in_progress","pending"}
PLATFORM_MAX_ACTIVE=1
PLATFORM_MAX_PENDING=100

class ObserverError(RuntimeError): pass

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None

def canonical(v:Any)->bytes:
    return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=True).encode()
def digest(v:Any)->str:
    return hashlib.sha256(canonical(v)).hexdigest()
def _is_int(v:Any)->bool:
    return type(v) is int
def _read_bounded(resp)->bytes:
    data=resp.read(MAX_BODY_BYTES+1)
    if len(data)>MAX_BODY_BYTES: raise ObserverError("response_too_large")
    return data
def _headers(token:str|None)->dict[str,str]:
    h={"Accept":"application/vnd.github+json","X-GitHub-Api-Version":API_VERSION,"User-Agent":"mycelix-ci-gov-001k-observer"}
    if token: h["Authorization"]=f"Bearer {token}"
    return h

def http_get(token:str|None=None, timeout_seconds:float=5.0)->tuple[int,bytes]:
    req=urllib.request.Request(API_URL,headers=_headers(token),method="GET")
    opener=urllib.request.build_opener(NoRedirect())
    try:
        with opener.open(req,timeout=timeout_seconds) as resp:
            return int(resp.status),_read_bounded(resp)
    except urllib.error.HTTPError as exc:
        return int(exc.code),_read_bounded(exc)
    except (urllib.error.URLError,TimeoutError,OSError):
        raise ObserverError("transport_error")

def _valid_run_urls(m:dict[str,Any],run_id:int)->bool:
    api=f"https://api.github.com/repos/{OWNER}/{REPO}/actions/runs/{run_id}"
    html=f"https://github.com/{OWNER}/{REPO}/actions/runs/{run_id}"
    if m.get("run_url")!=api or m.get("run_html_url")!=html: return False
    job_id=m.get("job_id")
    if job_id is None:
        return not any(k in m for k in ("job_name","job_url","job_html_url"))
    if not _is_int(job_id) or job_id<=0: return False
    if not isinstance(m.get("job_name"),str) or not m["job_name"]: return False
    if m.get("job_url")!=f"https://api.github.com/repos/{OWNER}/{REPO}/actions/jobs/{job_id}": return False
    if m.get("job_html_url")!=f"{html}/job/{job_id}": return False
    return True

def parse_live_group(status:int,body:bytes)->dict[str,Any]:
    if status==404:
        return {"complete":True,"group_present":False,"configuration_established":False,
                "active_count":0,"pending_count":0,"total_count":0,"members":[],"source_status":404,
                "reason":"inactive_or_nonexistent_group"}
    if status!=200:
        return {"complete":False,"group_present":None,"configuration_established":False,
                "active_count":None,"pending_count":None,"total_count":None,"members":[],"source_status":status,
                "reason":"api_status_non_authoritative"}
    try: payload=json.loads(body)
    except (UnicodeDecodeError,json.JSONDecodeError):
        raise ObserverError("invalid_json")
    if not isinstance(payload,dict): raise ObserverError("payload_not_object")
    if payload.get("group_name")!=GROUP or payload.get("group_url")!=GROUP_URL:
        raise ObserverError("group_identity_mismatch")
    total=payload.get("total_count"); members=payload.get("group_members")
    if not _is_int(total) or total<1 or not isinstance(members,list) or total!=len(members):
        raise ObserverError("group_count_mismatch")
    seen=set(); normalized=[]; active=0; pending=0
    for m in members:
        if not isinstance(m,dict): raise ObserverError("member_not_object")
        run_id=m.get("run_id")
        if not _is_int(run_id) or run_id<=0: raise ObserverError("invalid_run_id")
        if not isinstance(m.get("run_name"),str) or not m["run_name"]: raise ObserverError("invalid_run_name")
        if not _valid_run_urls(m,run_id): raise ObserverError("member_url_mismatch")
        st=m.get("status")
        if st not in ALLOWED_STATUSES: raise ObserverError("unknown_member_status")
        job_id=m.get("job_id")
        identity=(run_id,job_id)
        if identity in seen: raise ObserverError("duplicate_member_identity")
        seen.add(identity)
        if st=="in_progress": active+=1
        else: pending+=1
        normalized.append({"run_id":run_id,"job_id":job_id,"status":st})
    if active>PLATFORM_MAX_ACTIVE or pending>PLATFORM_MAX_PENDING:
        raise ObserverError("platform_bound_violation")
    return {"complete":True,"group_present":True,"configuration_established":True,
            "active_count":active,"pending_count":pending,"total_count":total,
            "members":sorted(normalized,key=lambda x:(x["run_id"],-1 if x["job_id"] is None else x["job_id"])),
            "source_status":200,"reason":"live_group_observed"}

def _finalize(live:dict[str,Any], observed_at:int|None)->dict[str,Any]:
    body={"schema":SCHEMA,"repository":f"{OWNER}/{REPO}","group_name":GROUP,
          "api_url":API_URL,"api_version":API_VERSION,
          "observed_at_epoch_seconds":observed_at,
          "observed_at_utc":None if observed_at is None else datetime.fromtimestamp(observed_at,timezone.utc).isoformat().replace("+00:00","Z"),
          **live,
          "grants_queue_admission":False,"grants_cancellation_authority":False,
          "grants_product_pass":False,"grants_scientific_pass":False,
          "nonclaims":["404 establishes only an empty live group, not workflow configuration.",
                       "Observer completeness does not grant queue admission by itself.",
                       "No Actions mutation authority is granted."]}
    body["receipt_commitment"]=digest(body)
    return body

def observe(getter:Callable[[],tuple[int,bytes]], now_fn:Callable[[],float]=time.time)->dict[str,Any]:
    try:
        raw_now=now_fn()
        if isinstance(raw_now,bool) or not isinstance(raw_now,(int,float)):
            raise ObserverError("invalid_clock")
        observed_at=int(raw_now)
        if observed_at<0:
            raise ObserverError("invalid_clock")
    except (ValueError,OverflowError,ObserverError):
        return _finalize({"complete":False,"group_present":None,"configuration_established":False,
                          "active_count":None,"pending_count":None,"total_count":None,"members":[],
                          "source_status":None,"reason":"invalid_clock"},None)
    try:
        status,body=getter()
        live=parse_live_group(status,body)
    except ObserverError as exc:
        live={"complete":False,"group_present":None,"configuration_established":False,
              "active_count":None,"pending_count":None,"total_count":None,"members":[],
              "source_status":None,"reason":str(exc)}
    return _finalize(live,observed_at)

def main()->int:
    p=argparse.ArgumentParser(); p.add_argument("--timeout-seconds",type=float,default=5.0); a=p.parse_args()
    if not (0<a.timeout_seconds<=10): print("OBSERVER ERROR: timeout out of bounds",file=sys.stderr); return 2
    token=os.environ.get("GITHUB_TOKEN")
    result=observe(lambda:http_get(token,a.timeout_seconds))
    print(json.dumps(result,indent=2,sort_keys=True))
    return 0 if result["complete"] else 3

if __name__=="__main__": raise SystemExit(main())
