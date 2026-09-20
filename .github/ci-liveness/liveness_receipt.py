#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

MANIFEST_SCHEMA='mycelix-required-job-manifest-v1'
RECEIPT_SCHEMA='mycelix-workflow-run-liveness-receipt-v1'
MD=b'MYCELIX_REQUIRED_JOB_MANIFEST_V1\0'; RD=b'MYCELIX_WORKFLOW_RUN_LIVENESS_RECEIPT_V1\0'; ID=b'MYCELIX_CI_LIVENESS_IMPLEMENTATION_V1\0'
SHA1=re.compile(r'^[0-9a-f]{40}$'); UTC=re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z$')
SUCCESS={'success'}; INTERRUPT={'cancelled','timed_out','startup_failure','stale'}
class LivenessError(ValueError): pass
_IMPL=Path(__file__).read_bytes(); IMPLEMENTATION_COMMITMENT_SHA256=hashlib.sha256(ID+_IMPL).hexdigest(); del _IMPL

def pairs(p):
    d={}
    for k,v in p:
        if k in d: raise LivenessError(f'duplicate JSON key: {k}')
        d[k]=v
    return d

def loads(s:str)->Any:
    try:return json.loads(s,object_pairs_hook=pairs)
    except json.JSONDecodeError as e: raise LivenessError(f'invalid JSON: {e}') from e

def canon(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()
def exact(o:dict,ks:set[str],ctx:str):
    if set(o)!=ks: raise LivenessError(f'{ctx} field mismatch')
def s(v,ctx):
    if not isinstance(v,str) or not v: raise LivenessError(f'{ctx} must be non-empty string')
    return v

def manifest(m:dict)->tuple[list[dict],str]:
    if not isinstance(m,dict): raise LivenessError('manifest must be object')
    exact(m,{'schema','repository','theorem_id','qualification_head','workflow_path','required_jobs'},'manifest')
    if m['schema']!=MANIFEST_SCHEMA: raise LivenessError('unsupported manifest schema')
    for k in ('repository','theorem_id','workflow_path'): s(m[k],f'manifest.{k}')
    if not SHA1.fullmatch(s(m['qualification_head'],'manifest.qualification_head')): raise LivenessError('qualification_head must be SHA-1')
    if m['workflow_path'].startswith('/') or '..' in m['workflow_path'].split('/'): raise LivenessError('noncanonical workflow_path')
    if not isinstance(m['required_jobs'],list) or not m['required_jobs']: raise LivenessError('required_jobs must be non-empty list')
    specs=[]; names=set()
    for x in m['required_jobs']:
        if not isinstance(x,dict): raise LivenessError('job spec must be object')
        exact(x,{'name','required_gate_names','depends_on'},'job spec'); name=s(x['name'],'job name')
        if name in names: raise LivenessError('duplicate job name')
        names.add(name)
        if not isinstance(x['required_gate_names'],list) or not x['required_gate_names'] or any(not isinstance(v,str) or not v for v in x['required_gate_names']): raise LivenessError('invalid gate list')
        if not isinstance(x['depends_on'],list) or any(not isinstance(v,str) or not v for v in x['depends_on']): raise LivenessError('invalid dependency list')
        if len(set(x['required_gate_names']))!=len(x['required_gate_names']) or len(set(x['depends_on']))!=len(x['depends_on']) or name in x['depends_on']: raise LivenessError('duplicate/self dependency')
        specs.append({'name':name,'required_gate_names':sorted(x['required_gate_names']),'depends_on':sorted(x['depends_on'])})
    specs.sort(key=lambda x:x['name']); req={x['name'] for x in specs}; graph={x['name']:[d for d in x['depends_on'] if d in req] for x in specs}
    visiting=set(); done=set()
    def visit(n):
        if n in done:return
        if n in visiting: raise LivenessError('cycle in required-job dependency graph')
        visiting.add(n)
        for d in graph[n]:visit(d)
        visiting.remove(n);done.add(n)
    for n in sorted(req):visit(n)
    norm={**m,'required_jobs':specs}; return specs,hashlib.sha256(MD+canon(norm)).hexdigest()

def project_run(r:dict,m:dict)->dict:
    for k in ('id','head_sha','path','status','conclusion'):
        if k not in r: raise LivenessError(f'run missing {k}')
    if not isinstance(r['id'],int) or isinstance(r['id'],bool) or r['id']<=0: raise LivenessError('invalid run id')
    if r['head_sha']!=m['qualification_head'] or r['path']!=m['workflow_path']: raise LivenessError('run identity mismatch')
    return {k:r[k] for k in ('id','head_sha','path','status','conclusion')}

def project_jobs(raw:Any,run:dict)->dict[str,dict]:
    rows=raw.get('jobs') if isinstance(raw,dict) else raw
    if not isinstance(rows,list): raise LivenessError('jobs must be list or {jobs:[...]}')
    out={}
    for j in rows:
        for k in ('id','name','status','conclusion','steps','runner_id','runner_name'):
            if k not in j: raise LivenessError(f'job missing {k}')
        name=s(j['name'],'job.name')
        if name in out: raise LivenessError('duplicate observed job name')
        if 'run_id' in j and j['run_id']!=run['id']: raise LivenessError('job run_id mismatch')
        if 'head_sha' in j and j['head_sha']!=run['head_sha']: raise LivenessError('job head_sha mismatch')
        out[name]={k:j[k] for k in ('id','name','status','conclusion','steps','runner_id','runner_name')}
    return out

def gate_state(job:dict,gates:list[str])->str:
    steps=job['steps']
    if steps is None:return 'Unknown'
    if not isinstance(steps,list): raise LivenessError('steps must be list/null')
    by={}
    for st in steps:
        if not isinstance(st,dict) or 'name' not in st or 'status' not in st or 'conclusion' not in st or 'started_at' not in st: raise LivenessError('malformed step')
        if st['name'] in by: raise LivenessError('duplicate step name')
        by[st['name']]=st
    if any(g not in by for g in gates): return 'Unknown' if steps else 'NoTheoremStepExecuted'
    gs=[by[g] for g in gates]; started=[x for x in gs if x['started_at'] is not None or x['status'] in {'in_progress','completed'}]
    if not started:return 'NoTheoremStepExecuted'
    if all(x['status']=='completed' for x in gs):return 'AllRegisteredTheoremStepsExecuted'
    return 'SomeTheoremStepsExecuted'

def dep_state(spec:dict,jobs:dict[str,dict])->str:
    if not spec['depends_on']:return 'EligibleForRunner'
    vals=[]
    for d in spec['depends_on']:
        x=jobs.get(d)
        if x is None:return 'Unknown'
        if x['status']!='completed': vals.append('wait')
        elif x['conclusion']=='success': vals.append('ok')
        elif x['conclusion']=='skipped': vals.append('skip')
        else: vals.append('fail')
    if 'fail' in vals:return 'DependencyFailed'
    if 'skip' in vals:return 'DependencySkipped'
    if 'wait' in vals:return 'WaitingOnRequiredDependency'
    return 'EligibleForRunner'

def classify(req:list[dict])->str:
    if any(x['step_metadata']=='Missing' for x in req):return 'Indeterminate'
    if any(x['status']!='completed' and x['runner_started'] for x in req):return 'RequiredJobExecuting'
    terminal=all(x['status']=='completed' for x in req); nostart=all(x['theorem_gate_execution']=='NoTheoremStepExecuted' for x in req); norunner=all(x['runner_started'] is False for x in req)
    if terminal:
        cancelled=all(x['conclusion']=='cancelled' for x in req)
        if cancelled and any(x['theorem_gate_execution']=='Unknown' for x in req):return 'TerminalCancelledTheoremStartUnknown'
        if cancelled and nostart:return 'TerminalCancelledNoStart' if norunner else 'TerminalCancelledBeforeTheorem'
        if any(x['conclusion'] in INTERRUPT and x['theorem_gate_execution']!='NoTheoremStepExecuted' for x in req):return 'InfrastructureInterrupted'
        if all(x['conclusion'] in SUCCESS and x['theorem_gate_execution']=='AllRegisteredTheoremStepsExecuted' for x in req):return 'CompletedConjunctivePass'
        return 'CompletedConjunctiveFail'
    waiting=all(x['status'] in {'queued','waiting','pending'} for x in req)
    if waiting and nostart:
        ds={x['dependency_state'] for x in req}
        if ds <= {'WaitingOnRequiredDependency','DependencyFailed','DependencySkipped'}:return 'WaitingOnRequiredDependencies'
        if ds=={'EligibleForRunner'} and norunner:return 'AllRequiredJobsNoStart'
    if any(x['status']=='completed' and x['theorem_gate_execution']!='NoTheoremStepExecuted' for x in req) and any(x['status'] in {'queued','waiting','pending'} for x in req):return 'PartiallyExecutedAwaitingRequiredJob'
    return 'Indeterminate'

def classify_liveness(run:dict,jobs_raw:Any,m:dict,observed_at_utc:str)->dict:
    if not UTC.fullmatch(s(observed_at_utc,'observed_at_utc')): raise LivenessError('observed_at_utc must be RFC3339 UTC Z')
    try:
        if datetime.fromisoformat(observed_at_utc[:-1]+'+00:00').tzinfo!=timezone.utc: raise ValueError
    except ValueError as e: raise LivenessError('invalid observed_at_utc') from e
    specs,mc=manifest(m); rv=project_run(run,m); jobs=project_jobs(jobs_raw,rv); req=[]
    for sp in specs:
        j=jobs.get(sp['name'])
        if j is None: raise LivenessError(f'missing required job: {sp["name"]}')
        rid=j['runner_id']; rn=j['runner_name']; runner=None if rid is None and rn is None else bool((rid or 0)!=0 or (rn or '')!='')
        req.append({'job_id':j['id'],'name':j['name'],'status':j['status'],'conclusion':j['conclusion'],'runner_started':runner,'step_metadata':'Missing' if j['steps'] is None else 'Present','theorem_gate_execution':gate_state(j,sp['required_gate_names']),'dependency_state':dep_state(sp,jobs)})
    c=classify(req); runners=[x['runner_started'] for x in req]; observed=True if any(v is True for v in runners) else False if all(v is False for v in runners) else None
    receipt={'schema':RECEIPT_SCHEMA,'repository':m['repository'],'theorem_id':m['theorem_id'],'workflow_run_id':rv['id'],'workflow_head_sha':rv['head_sha'],'workflow_path':rv['path'],'observation_time_utc':observed_at_utc,'observed_run_status':rv['status'],'observed_run_conclusion':rv['conclusion'],'classification':c,'classifier_implementation_commitment_sha256':IMPLEMENTATION_COMMITMENT_SHA256,'required_job_manifest_commitment_sha256':mc,'required_jobs':req,'runner_assignment_observed':observed,'terminal_without_theorem_start':c in {'TerminalCancelledNoStart','TerminalCancelledBeforeTheorem'},'cancellation_cause':'Unknown' if rv['conclusion']=='cancelled' or any(x['conclusion']=='cancelled' for x in req) else None,'qualification_result':None,'theorem_result':None,'qualification_authority':False,'merge_authority':False,'deployment_authority':False}
    return {**receipt,'liveness_receipt_commitment_sha256':hashlib.sha256(RD+canon(receipt)).hexdigest()}

def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('--run',required=True);p.add_argument('--jobs',required=True);p.add_argument('--manifest',required=True);p.add_argument('--observed-at',required=True);a=p.parse_args()
    read=lambda x:loads(Path(x).read_text(encoding='utf-8'))
    print(canon(classify_liveness(read(a.run),read(a.jobs),read(a.manifest),a.observed_at)).decode());return 0
if __name__=='__main__':raise SystemExit(main())
