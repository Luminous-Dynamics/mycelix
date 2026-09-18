#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, re, socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import HTTPRedirectHandler, Request, build_opener

ADAPTER_ID="generic-mycelix-ci-pr-files-adapter-v1"
AUTHORITY="SchedulingObservationOnly"
API_HOST="api.github.com"
API_VERSION="2022-11-28"
PAGE_SIZE=100
MAX_FILES=3000
MAX_PAGES=30
MAX_PAGE_BYTES=8*1024*1024
KNOWN_STATUSES={"added","modified","removed","renamed","copied","changed","unchanged"}
REPO_RE=re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")

class AdapterError(RuntimeError): pass
class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None

@dataclass(frozen=True)
class Observation:
    observable: bool
    source: str
    declared_changed_files: int|None
    observed_file_records: int|None
    pages_requested: int
    records: list[dict[str,str]]|None
    reason: str
    def receipt(self)->dict[str,Any]:
        return {"adapter_id":ADAPTER_ID,"authority":AUTHORITY,"observable":self.observable,
                "source":self.source,"declared_changed_files":self.declared_changed_files,
                "observed_file_records":self.observed_file_records,
                "pages_requested":self.pages_requested,"page_size":PAGE_SIZE,"max_files":MAX_FILES,
                "reason":self.reason,"grants_ci_pass":False,"grants_product_qualification":False}

def _validate_repo(repo:Any)->str:
    if not isinstance(repo,str) or not REPO_RE.fullmatch(repo):
        raise AdapterError("repository must be owner/name using safe GitHub path characters")
    return repo

def _positive_int(value:Any,label:str)->int:
    if isinstance(value,bool): raise AdapterError(f"{label} must be a positive integer")
    try: number=int(value)
    except (TypeError,ValueError) as exc: raise AdapterError(f"{label} must be a positive integer") from exc
    if number<=0: raise AdapterError(f"{label} must be a positive integer")
    return number

def _safe_path(value:Any,field:str)->str:
    if not isinstance(value,str) or not value: raise AdapterError(f"{field} must be a non-empty string")
    if value.startswith("/") or "\\" in value or "\x00" in value: raise AdapterError(f"{field} must be a repo-relative POSIX path")
    if any(p in {"",".",".."} for p in value.split("/")): raise AdapterError(f"{field} contains unsafe path segment")
    return value

def _normalize_record(raw:Any,index:int)->dict[str,str]:
    if not isinstance(raw,dict): raise AdapterError(f"file record {index} must be an object")
    filename=_safe_path(raw.get("filename"),f"file record {index}.filename")
    status=raw.get("status")
    if status not in KNOWN_STATUSES: raise AdapterError(f"file record {index} has unknown status {status!r}")
    previous=raw.get("previous_filename")
    out={"filename":filename,"status":status}
    if status=="renamed":
        previous=_safe_path(previous,f"file record {index}.previous_filename")
        if previous==filename: raise AdapterError(f"file record {index} rename has identical old/new path")
        out["previous_filename"]=previous
    elif previous is not None:
        raise AdapterError(f"file record {index} has previous_filename but status is not renamed")
    return out

def _has_next(link_header:str|None)->bool:
    return bool(link_header and any('rel="next"' in p for p in link_header.split(",")))

def _default_fetch(url:str,token:str)->tuple[int,str|None,bytes]:
    request=Request(url,method="GET",headers={"Accept":"application/vnd.github+json","Authorization":f"Bearer {token}","X-GitHub-Api-Version":API_VERSION,"User-Agent":ADAPTER_ID})
    opener=build_opener(NoRedirect)
    try:
        with opener.open(request,timeout=15) as response:
            status=getattr(response,"status",response.getcode())
            if response.geturl()!=url: raise AdapterError("GitHub API redirect/final URL drift refused")
            body=response.read(MAX_PAGE_BYTES+1)
            if len(body)>MAX_PAGE_BYTES: raise AdapterError("GitHub API page exceeded byte bound")
            return status,response.headers.get("Link"),body
    except HTTPError as exc: raise AdapterError(f"GitHub API HTTP error {exc.code}") from exc
    except (URLError,TimeoutError,socket.timeout) as exc: raise AdapterError(f"GitHub API transport error: {type(exc).__name__}") from exc

FetchPage=Callable[[str,str],tuple[int,str|None,bytes]]

def observe(*,repository:Any,pr_number:Any,declared_changed_files:Any,token:str,fetch_page:FetchPage=_default_fetch)->Observation:
    try:
        repo=_validate_repo(repository); pr=_positive_int(pr_number,"pull request number"); declared=_positive_int(declared_changed_files,"declared changed_files")
    except AdapterError as exc:
        return Observation(False,"FailClosedFallback",None,None,0,None,str(exc))
    if declared>MAX_FILES:
        return Observation(False,"FailClosedFallback",declared,None,0,None,"declared changed_files exceeds bounded Pull Request Files API limit")
    if not isinstance(token,str) or not token:
        return Observation(False,"FailClosedFallback",declared,None,0,None,"GitHub token missing")
    expected_pages=(declared+PAGE_SIZE-1)//PAGE_SIZE
    records=[]; pages=0
    try:
        for page in range(1,expected_pages+1):
            url=f"https://{API_HOST}/repos/{quote(repo,safe='/')}/pulls/{pr}/files?per_page={PAGE_SIZE}&page={page}"
            status,link,body=fetch_page(url,token); pages+=1
            if status!=200: raise AdapterError(f"GitHub API returned unexpected status {status}")
            try: payload=json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError,json.JSONDecodeError) as exc: raise AdapterError("GitHub API page is not valid UTF-8 JSON") from exc
            if not isinstance(payload,list): raise AdapterError("GitHub API page root must be a list")
            expected_len=PAGE_SIZE if page<expected_pages else declared-PAGE_SIZE*(expected_pages-1)
            if len(payload)!=expected_len: raise AdapterError(f"page {page} length {len(payload)} != expected {expected_len}")
            if page<expected_pages and not _has_next(link): raise AdapterError(f"page {page} missing rel=next despite expected continuation")
            if page==expected_pages and _has_next(link): raise AdapterError("final expected page still advertises rel=next")
            for raw in payload: records.append(_normalize_record(raw,len(records)))
        if len(records)!=declared: raise AdapterError("declared/observed file record count mismatch")
        current=[r["filename"] for r in records]
        if len(current)!=len(set(current)): raise AdapterError("duplicate current filenames in PR-files observation")
    except AdapterError as exc:
        return Observation(False,"FailClosedFallback",declared,len(records),pages,None,str(exc))
    except Exception as exc:
        return Observation(False,"FailClosedFallback",declared,len(records),pages,None,f"unexpected adapter error: {type(exc).__name__}")
    return Observation(True,"PullRequestFilesApi",declared,len(records),pages,records,"bounded PR-files observation is complete and internally consistent")

def _fake_pages(pages):
    idx={"i":0}
    def fetch(url,token):
        assert url.startswith("https://api.github.com/repos/") and token=="token"
        payload,link=pages[idx["i"]]; idx["i"]+=1
        return 200,link,json.dumps(payload).encode()
    return fetch

def self_test():
    one=[{"filename":"docs/lex-net/a.md","status":"modified"}]
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=1,declared_changed_files=1,token="token",fetch_page=_fake_pages([(one,None)])); assert r.observable
    renamed=[{"filename":"docs/lex-net/moved.md","previous_filename":"mycelix-governance/src/lib.rs","status":"renamed"}]
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=2,declared_changed_files=1,token="token",fetch_page=_fake_pages([(renamed,None)])); assert r.observable and r.records[0]["previous_filename"]
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=3,declared_changed_files=3001,token="token",fetch_page=_fake_pages([])); assert not r.observable and r.pages_requested==0
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=4,declared_changed_files=2,token="token",fetch_page=_fake_pages([(one,None)])); assert not r.observable
    duplicate=[{"filename":"x","status":"modified"},{"filename":"x","status":"modified"}]
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=5,declared_changed_files=2,token="token",fetch_page=_fake_pages([(duplicate,None)])); assert not r.observable
    unknown=[{"filename":"x","status":"future"}]
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=6,declared_changed_files=1,token="token",fetch_page=_fake_pages([(unknown,None)])); assert not r.observable
    hundred=[{"filename":f"f/{i}","status":"modified"} for i in range(100)]
    last=[{"filename":"last","status":"added"}]
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=7,declared_changed_files=101,token="token",fetch_page=_fake_pages([(hundred,'<x>; rel="next"'),(last,None)])); assert r.observable and len(r.records)==101
    r=observe(repository="Luminous-Dynamics/mycelix",pr_number=8,declared_changed_files=100,token="token",fetch_page=_fake_pages([(hundred,'<x>; rel="next"')])); assert not r.observable

def _write_json(path,value):
    if path: Path(path).write_text(json.dumps(value,sort_keys=True)+"\n",encoding="utf-8")

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("--repository",default=os.getenv("GITHUB_REPOSITORY","")); ap.add_argument("--pr-number",default=os.getenv("PR_NUMBER","")); ap.add_argument("--declared-changed-files",default=os.getenv("PR_CHANGED_FILES","")); ap.add_argument("--token",default=os.getenv("GITHUB_TOKEN","")); ap.add_argument("--records-output"); ap.add_argument("--receipt-output"); ap.add_argument("--self-test",action="store_true"); a=ap.parse_args()
    if a.self_test:
        self_test(); print(json.dumps({"adapter_id":ADAPTER_ID,"self_test":"PASS","authority":AUTHORITY,"grants_ci_pass":False,"grants_product_qualification":False},sort_keys=True)); return 0
    o=observe(repository=a.repository,pr_number=a.pr_number,declared_changed_files=a.declared_changed_files,token=a.token)
    _write_json(a.records_output,o.records); _write_json(a.receipt_output,o.receipt()); print(json.dumps(o.receipt(),sort_keys=True)); return 0

if __name__=="__main__": raise SystemExit(main())
