import importlib.util,json,pathlib,sys,unittest
from unittest import mock
HERE=pathlib.Path(__file__).parent
S=importlib.util.spec_from_file_location('a',str(HERE/'ci_qualification_admission_adviser.py'));a=importlib.util.module_from_spec(S);sys.modules[S.name]=a;S.loader.exec_module(a)
A='a'*40;B='b'*40

def pr(labels=None,head=A,state='open',draft=True,num=7,head_repo='Luminous-Dynamics/mycelix',base_repo='Luminous-Dynamics/mycelix'):
 return json.dumps({'number':num,'state':state,'draft':draft,'head':{'sha':head,'repo':{'full_name':head_repo}},'base':{'repo':{'full_name':base_repo}},'labels':[{'name':x} for x in (labels or [])]}).encode()
def group(ms):return json.dumps({'group_name':a.observer.GROUP,'group_url':a.observer.API_URL,'total_count':len(ms),'group_members':ms}).encode()
def member(i=1,status='pending'):return {'run_id':i,'status':status}
class Clock:
 def __init__(self,*v):self.v=iter(v)
 def __call__(self):return next(self.v)
class T(unittest.TestCase):
 def ev(self,pre=None,post=None,g=(404,b'{}'),times=(100,101),head=A):
  xs=iter([pre or (200,pr()),post or (200,pr())]);return a.evaluate(7,head,lambda:next(xs),lambda:g,Clock(*times))
 def test_empty_eligible(self):self.assertEqual(self.ev()['adviser_state'],'PolicyLabelAdmissionEligible')
 def test_active_queue_eligible(self):self.assertEqual(self.ev(g=(200,group([member(1,'in_progress')])) )['oracle_receipt']['capacity_state'],'QueueAdmissionEligible')
 def test_pending_queue_eligible(self):self.assertEqual(self.ev(g=(200,group([member()])))['oracle_receipt']['capacity_state'],'QueueAdmissionEligible')
 def test_budget_full(self):self.assertEqual(self.ev(g=(200,group([member(i+1) for i in range(8)])))['adviser_state'],'PolicyLabelAdmissionDeferred')
 def test_stale(self):self.assertEqual(self.ev(times=(100,131))['oracle_receipt']['capacity_state'],'AdmissionDeferredObservationUnknown')
 def test_future_clock(self):self.assertFalse(self.ev(times=(100,99))['complete'])
 def test_head_drift(self):self.assertEqual(self.ev(post=(200,pr(head=B)))['reason'],'pr_changed_during_observation')
 def test_label_drift(self):self.assertFalse(self.ev(post=(200,pr(labels=['x'])))['complete'])
 def test_draft_drift(self):self.assertFalse(self.ev(post=(200,pr(draft=False)))['complete'])
 def test_state_drift(self):self.assertFalse(self.ev(post=(200,pr(state='closed')))['complete'])
 def test_superseded(self):self.assertEqual(self.ev(pre=(200,pr(head=B)),post=(200,pr(head=B)),head=A)['oracle_receipt']['admission_state'],'SupersededOutsideCapacityAuthority')
 def test_closed(self):self.assertEqual(self.ev(pre=(200,pr(state='closed')),post=(200,pr(state='closed')))['adviser_state'],'PolicyAdmissionDenied')
 def test_token_present(self):self.assertEqual(self.ev(pre=(200,pr(labels=['ci:qualify'])),post=(200,pr(labels=['ci:qualify'])))['adviser_state'],'TokenAlreadyPresent')
 def test_proposed_token(self):self.assertEqual(self.ev(pre=(200,pr(labels=['x'])),post=(200,pr(labels=['x'])))['proposed_labels'],['ci:qualify','x'])
 def test_non200(self):self.assertFalse(self.ev(pre=(403,b'{}'))['complete'])
 def test_wrong_number(self):self.assertEqual(self.ev(pre=(200,pr(num=8)))['reason'],'pr_number_mismatch')
 def test_foreign_repo(self):self.assertFalse(self.ev(pre=(200,pr(head_repo='x/y')))['complete'])
 def test_duplicate_labels(self):self.assertFalse(self.ev(pre=(200,pr(labels=['x','x'])))['complete'])
 def test_bad_observer_commitment(self):
  o=a.observer.observe(lambda:(404,b'{}'),lambda:100);o['receipt_commitment']='0'*64
  with mock.patch.object(a.observer,'observe',return_value=o):
   xs=iter([(200,pr()),(200,pr())]);r=a.evaluate(7,A,lambda:next(xs),lambda:(404,b'{}'),Clock(100,101));self.assertEqual(r['reason'],'observer_commitment_mismatch')
 def test_observer_authority(self):
  o=a.observer.observe(lambda:(404,b'{}'),lambda:100);o['grants_queue_admission']=True;b=dict(o);b.pop('receipt_commitment');o['receipt_commitment']=a.digest(b)
  with mock.patch.object(a.observer,'observe',return_value=o):
   xs=iter([(200,pr()),(200,pr())]);r=a.evaluate(7,A,lambda:next(xs),lambda:(404,b'{}'),Clock(100,101));self.assertEqual(r['reason'],'observer_authority_ceiling_mismatch')
 def test_no_authority(self):
  r=self.ev();self.assertFalse(r['grants_operational_label_mutation']);self.assertTrue(r['requires_live_scheduler_qualification'])
 def test_commitment_time(self):self.assertNotEqual(self.ev(times=(100,101))['receipt_commitment'],self.ev(times=(100,102))['receipt_commitment'])
 def test_invalid_sha(self):self.assertFalse(a.evaluate(7,'x',lambda:(200,pr()),lambda:(404,b'{}'),Clock(100,101))['complete'])
 def test_fixed_pr_endpoint(self):self.assertEqual(a.pr_api_url(7),'https://api.github.com/repos/Luminous-Dynamics/mycelix/pulls/7')
 def test_invalid_timeout(self):
  with self.assertRaises(a.AdviserError):a.http_get_pr(7,None,0)
 def test_redirect_disabled(self):self.assertIsNone(a.NoRedirect().redirect_request(None,None,302,'x',{},'https://evil.invalid'))
 def test_token_not_in_receipt(self):
  self.assertEqual(a.headers('secret')['Authorization'],'Bearer secret');self.assertNotIn('secret',json.dumps(self.ev()))
 def test_source_no_mutator(self):
  t=(HERE/'ci_qualification_admission_adviser.py').read_text()
  for s in ('POST','PATCH','PUT','DELETE','add_labels','cancel_run(','workflow_dispatch','merge_pull_request'):self.assertNotIn(s,t)
if __name__=='__main__':unittest.main()
