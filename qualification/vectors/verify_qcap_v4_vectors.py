#!/usr/bin/env python3
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
CAP=b'MYCELIX_QUALIFICATION_CAPSULE_V1\0'
LIM=b'MYCELIX_QCAP_EXECUTION_LIMITS_V1\0'
CON=b'MYCELIX_QCAP_CONTAINMENT_PROFILE_V1\0'
REC=b'MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V4\0'
EMPTY=hashlib.sha256(b'').hexdigest()
REASONS={'OutputLimitExceeded','Timeout','ProcessStartFailure','OutputDrainTimeout','WorktreeMaterializationFailure','WorktreeCleanupFailure','ArtifactIntegrityFailure','GateReportedRunnerFailure','UnexpectedExitCode','RunnerInternalFailure','ContainmentFailure'}
def cj(v):
 def e(x):
  if x is None:return'null'
  if x is True:return'true'
  if x is False:return'false'
  if isinstance(x,int)and not isinstance(x,bool):return str(x)
  if isinstance(x,float):raise ValueError('float')
  if isinstance(x,str):return json.dumps(x,ensure_ascii=False,separators=(',',':'))
  if isinstance(x,list):return'['+','.join(e(i) for i in x)+']'
  if isinstance(x,dict):
   if not all(isinstance(k,str)for k in x):raise ValueError('key')
   return'{'+','.join(json.dumps(k,ensure_ascii=False,separators=(',',':'))+':'+e(x[k])for k in sorted(x,key=lambda z:z.encode()))+'}'
  raise ValueError(type(x).__name__)
 return e(v).encode()
def com(d,v):
 b=cj(v);return hashlib.sha256(d+len(b).to_bytes(8,'big')+b).hexdigest()
def pref(p,d):return{'id':p['profile_id'],'revision':p['profile_revision'],'digest':com(d,p)}
def vr(r):
 s=r['status'];c=r['effective_exit_code'];n=r['captured_output_bytes'];t=r['output_truncated'];x=r['runner_failure_reason']
 if s=='GateNotRun':return c is None and n==0 and r['captured_output_sha256']==EMPTY and not t and x is None
 if s=='GatePass':return c==0 and x is None and not t
 if s=='GateFail':return c==10 and x is None and not t
 if s=='RunnerInfrastructureFailure':return c==20 and x in REASONS and((x=='OutputLimitExceeded')==t)
 return False
def verdict(rs):
 infra=False;fail=False
 for r in rs:
  if not vr(r):raise ValueError('bad result')
  if infra:
   if r['status']!='GateNotRun':raise ValueError('bad suffix')
   continue
  if r['status']=='GateNotRun':raise ValueError('early notrun')
  if r['status']=='RunnerInfrastructureFailure':infra=True
  elif r['status']=='GateFail':fail=True
 return'RunnerInfrastructureFailure'if infra else('CompletedConjunctiveFail'if fail else'CompletedConjunctivePass')
def receipt(m,c,attempt,rs):
 b={'receipt_format_revision':4,'capsule_commitment':com(CAP,m),'theorem_id':m['theorem_id'],'theorem_revision':m['theorem_revision'],'repository_identity':m['repository_identity'],'product_subject_sha':m['product_subject_sha'],'attempt_id':attempt,'execution_context':c,'gate_results':rs,'verdict':verdict(rs),'claim':m['claim'],'nonclaims':m['nonclaims']};b['receipt_commitment']=com(REC,b);return b
def main():
 v=json.loads((HERE/'qcap-v4-vector-001.json').read_text());m=v['manifest'];l=v['execution_limits'];p=v['containment_profile'];c=v['execution_context']
 assert com(CAP,m)==v['expected_capsule_commitment']
 assert com(LIM,l)==v['expected_execution_limits_digest']
 assert com(CON,p)==v['expected_containment_profile_digest']
 assert c['execution_limits_profile_ref']==pref(l,LIM)
 assert c['containment_profile_ref']==pref(p,CON)
 for s in v['scenarios']:
  r=receipt(m,c,s['attempt_id'],s['gate_results']);assert r['verdict']==s['expected_verdict'];assert r['receipt_commitment']==s['expected_receipt_commitment'],s['name']
 cf=v['scenarios'][2]['gate_results']
 assert cf[0]['runner_failure_reason']=='ContainmentFailure'and cf[1]['status']=='GateNotRun'
 bad=[dict(cf[1]),dict(cf[0])]
 try:verdict(bad);raise AssertionError('malformed GateNotRun order accepted')
 except ValueError:pass
 x=dict(cf[0]);x['output_truncated']=True
 try:vr(x)or(_ for _ in()).throw(ValueError());raise AssertionError('contradictory containment truncation accepted')
 except ValueError:pass
 badc=dict(c);badc['containment_profile_ref']=dict(c['containment_profile_ref']);badc['containment_profile_ref']['digest']='0'*64
 assert badc['containment_profile_ref']!=pref(p,CON)
 try:cj({'x':1.5});raise AssertionError('float accepted')
 except ValueError:pass
 print('qcap_v4_independent_vectors=PASS')
 print('capsule_commitment='+v['expected_capsule_commitment'])
 print('execution_limits_digest='+v['expected_execution_limits_digest'])
 print('containment_profile_digest='+v['expected_containment_profile_digest'])
 for s in v['scenarios']:print(s['name']+'_receipt='+s['expected_receipt_commitment'])
if __name__=='__main__':main()
