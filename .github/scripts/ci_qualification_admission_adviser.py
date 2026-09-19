#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, importlib.util, json, math, os, pathlib, sys, time, urllib.error, urllib.request
from typing import Any, Callable
HERE=pathlib.Path(__file__).parent

def _load(name,path):
    s=importlib.util.spec_from_file_location(name,str(path)); m=importlib.util.module_from_spec(s); sys.modules[name]=m; s.loader.exec_module(m); return m
oracle=_load('oracle',HERE/'ci_qualification_capacity_oracle.py')
observer=_load('observer',HERE/'ci_qualification_capacity_observer.py')
OWNER='Luminous-Dynamics';REPO='mycelix';LABEL='ci:qualify';API_VERSION='2026-03-10';SCHEMA='mycelix.ci-gov.001k.admission-adviser.v0.1';MAX_BODY_BYTES=1_048_576;MAX_TIMEOUT_SECONDS=10
class AdviserError(RuntimeError): pass
class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,req,fp,code,msg,headers,newurl): return None

def canonical(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode()
def digest(v:Any)->str:return hashlib.sha256(canonical(v)).hexdigest()
def valid_sha(v):return isinstance(v,str) and len(v)==40 and all(c in '0123456789abcdef' for c in v)
def is_int(v):return type(v) is int

def headers(token):
    h={'Accept':'application/vnd.github+json','X-GitHub-Api-Version':API_VERSION,'User-Agent':'mycelix-ci-gov-001k-adviser'}
    if token:h['Authorization']=f'Bearer {token}'
    return h
def read_bounded(resp):
    data=resp.read(MAX_BODY_BYTES+1)
    if len(data)>MAX_BODY_BYTES:raise AdviserError('response_too_large')
    return data
def pr_api_url(pr_number):
    if not is_int(pr_number) or pr_number<=0:raise AdviserError('invalid_pr_number')
    return f'https://api.github.com/repos/{OWNER}/{REPO}/pulls/{pr_number}'
def http_get_pr(pr_number,token=None,timeout_seconds=5.0):
    if isinstance(timeout_seconds,bool) or not isinstance(timeout_seconds,(int,float)) or not math.isfinite(float(timeout_seconds)) or not (0<float(timeout_seconds)<=MAX_TIMEOUT_SECONDS):raise AdviserError('timeout_out_of_bounds')
    req=urllib.request.Request(pr_api_url(pr_number),headers=headers(token),method='GET');opener=urllib.request.build_opener(NoRedirect())
    try:
        with opener.open(req,timeout=float(timeout_seconds)) as resp:return int(resp.status),read_bounded(resp)
    except urllib.error.HTTPError as exc:return int(exc.code),read_bounded(exc)
    except (urllib.error.URLError,TimeoutError,OSError):raise AdviserError('transport_error')

def parse_pr_snapshot(status:int,body:bytes,pr_number:int,subject_head:str):
    if not is_int(pr_number) or pr_number<=0 or not valid_sha(subject_head):raise AdviserError('invalid_subject_identity')
    if status!=200:return {'complete':False,'reason':'pr_api_status_non_authoritative'}
    try:p=json.loads(body)
    except Exception as e:raise AdviserError('invalid_pr_json') from e
    if p.get('number')!=pr_number:raise AdviserError('pr_number_mismatch')
    if p.get('state') not in ('open','closed') or type(p.get('draft')) is not bool:raise AdviserError('invalid_pr_state')
    head=p.get('head');base=p.get('base')
    if not isinstance(head,dict) or not isinstance(base,dict) or not valid_sha(head.get('sha')):raise AdviserError('invalid_pr_refs')
    if head.get('repo',{}).get('full_name')!=f'{OWNER}/{REPO}' or base.get('repo',{}).get('full_name')!=f'{OWNER}/{REPO}':raise AdviserError('repository_identity_mismatch')
    raw=p.get('labels')
    if not isinstance(raw,list):raise AdviserError('invalid_labels')
    labels=[]
    for x in raw:
        if not isinstance(x,dict) or not isinstance(x.get('name'),str) or not x['name']:raise AdviserError('invalid_label_entry')
        labels.append(x['name'])
    if len(labels)!=len(set(labels)):raise AdviserError('duplicate_labels')
    labels=sorted(labels); proposed=labels if LABEL in labels else sorted(labels+[LABEL])
    relevant={'pr_open':p['state']=='open','draft':p['draft'],'current_head':head['sha'],'actual_labels':labels}
    subject={'event':'pull_request','pr_open':relevant['pr_open'],'draft':relevant['draft'],'subject_head':subject_head,'current_head':head['sha'],'labels':proposed}
    return {'complete':True,'reason':'pr_snapshot_observed','subject':subject,'actual_labels':labels,'snapshot_commitment':digest(relevant)}

def validate_observer(r):
    req={'schema','repository','group_name','api_url','api_version','observed_at_epoch_seconds','observed_at_utc','complete','group_present','configuration_established','active_count','pending_count','total_count','members','source_status','reason','grants_queue_admission','grants_cancellation_authority','grants_product_pass','grants_scientific_pass','nonclaims','receipt_commitment'}
    if not isinstance(r,dict) or set(r)!=req:raise AdviserError('observer_receipt_fields_mismatch')
    if r['schema']!='mycelix.ci-gov.001k.capacity-observer.v0.1' or r['repository']!=f'{OWNER}/{REPO}' or r['group_name']!=oracle.EXPECTED_GROUP:raise AdviserError('observer_identity_mismatch')
    if r['api_url']!=observer.API_URL or r['api_version']!=API_VERSION:raise AdviserError('observer_api_identity_mismatch')
    if any(r[k] is not False for k in ('grants_queue_admission','grants_cancellation_authority','grants_product_pass','grants_scientific_pass')):raise AdviserError('observer_authority_ceiling_mismatch')
    body=dict(r);commit=body.pop('receipt_commitment')
    if not isinstance(commit,str) or commit!=digest(body):raise AdviserError('observer_commitment_mismatch')
    if type(r['complete']) is not bool:raise AdviserError('observer_complete_invalid')
    if r['complete']:
        for k in ('observed_at_epoch_seconds','active_count','pending_count','total_count'):
            if not is_int(r[k]) or r[k]<0:raise AdviserError('observer_count_or_clock_invalid')
        if r['total_count']!=r['active_count']+r['pending_count']:raise AdviserError('observer_count_mismatch')
        if r['active_count']>1 or r['pending_count']>100:raise AdviserError('observer_platform_bound_violation')

def clock(now_fn):
    raw=now_fn()
    if isinstance(raw,bool) or not isinstance(raw,(int,float)) or not math.isfinite(float(raw)) or raw<0:raise AdviserError('decision_clock_invalid')
    return int(raw)

def incomplete(reason,pr_number,subject_head):
    b={'schema':SCHEMA,'complete':False,'reason':reason,'repository':f'{OWNER}/{REPO}','pr_number':pr_number,'subject_head':subject_head,'adviser_state':'IncompleteFailClosed','policy_label_admission_eligible':False,'grants_operational_label_mutation':False,'grants_actions_mutation':False,'grants_cancellation_authority':False,'grants_product_pass':False,'grants_scientific_pass':False,'requires_live_scheduler_qualification':True};b['receipt_commitment']=digest(b);return b

def evaluate(pr_number:int,subject_head:str,pr_getter:Callable[[],tuple[int,bytes]],group_getter:Callable[[],tuple[int,bytes]],now_fn:Callable[[],float]):
    try:
        pre=parse_pr_snapshot(*pr_getter(),pr_number,subject_head)
        if not pre['complete']:return incomplete(pre['reason'],pr_number,subject_head)
        obs=observer.observe(group_getter,now_fn);validate_observer(obs)
        post=parse_pr_snapshot(*pr_getter(),pr_number,subject_head)
        if not post['complete']:return incomplete(post['reason'],pr_number,subject_head)
        if pre['snapshot_commitment']!=post['snapshot_commitment']:return incomplete('pr_changed_during_observation',pr_number,subject_head)
        now=clock(now_fn);t=obs.get('observed_at_epoch_seconds')
        if not is_int(t) or t<0 or now<t:return incomplete('observation_clock_invalid',pr_number,subject_head)
        age=now-t
        cap={'complete':bool(obs['complete']),'age_seconds':age,'active_count':obs['active_count'] if is_int(obs['active_count']) else 0,'pending_count':obs['pending_count'] if is_int(obs['pending_count']) else 0}
        orec=oracle.build_receipt(pre['subject'],cap);already=LABEL in pre['actual_labels']
        eligible=orec['capacity_state'] in ('ExecutionEligible','QueueAdmissionEligible')
        if already:state='TokenAlreadyPresent';pe=False
        elif orec['admission_state']!='QualificationAdmitted':state='PolicyAdmissionDenied';pe=False
        elif eligible:state='PolicyLabelAdmissionEligible';pe=True
        else:state='PolicyLabelAdmissionDeferred';pe=False
        b={'schema':SCHEMA,'complete':True,'reason':'decision_complete','repository':f'{OWNER}/{REPO}','pr_number':pr_number,'subject_head':subject_head,'pr_snapshot_commitment':pre['snapshot_commitment'],'observer_receipt_commitment':obs['receipt_commitment'],'observed_at_epoch_seconds':t,'decision_at_epoch_seconds':now,'derived_observation_age_seconds':age,'actual_labels':pre['actual_labels'],'proposed_labels':pre['subject']['labels'],'observer_configuration_established':obs['configuration_established'],'oracle_receipt':orec,'adviser_state':state,'policy_label_admission_eligible':pe,'grants_operational_label_mutation':False,'grants_actions_mutation':False,'grants_cancellation_authority':False,'grants_product_pass':False,'grants_scientific_pass':False,'requires_live_scheduler_qualification':True}
        b['receipt_commitment']=digest(b);return b
    except AdviserError as e:return incomplete(str(e),pr_number,subject_head)


def main():
    p=argparse.ArgumentParser();p.add_argument('pr_number',type=int);p.add_argument('subject_head');p.add_argument('--timeout-seconds',type=float,default=5.0);a=p.parse_args();token=os.environ.get('GITHUB_TOKEN')
    try:r=evaluate(a.pr_number,a.subject_head,lambda:http_get_pr(a.pr_number,token,a.timeout_seconds),lambda:observer.http_get(token,a.timeout_seconds),time.time)
    except AdviserError as e:r=incomplete(str(e),a.pr_number,a.subject_head)
    print(json.dumps(r,indent=2,sort_keys=True));return 0 if r.get('complete') else 3
if __name__=='__main__':raise SystemExit(main())
