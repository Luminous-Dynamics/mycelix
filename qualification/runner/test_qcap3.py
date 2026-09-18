import hashlib,importlib.util,json,os,subprocess,tempfile,unittest
from pathlib import Path
H=Path(__file__).resolve().parent
S=importlib.util.spec_from_file_location('qcap3',H/'qcap3.py');q=importlib.util.module_from_spec(S);S.loader.exec_module(q)
from qcap_canon import CapsuleError,canonical_json
from qcap3_limits import account_manifest_resources,checked_add,limits_digest,limits_ref,U64_MAX
from qcap3_receipt import gate_result,not_run_result,compose_receipt_v3,verify_receipt_v3

def pf(n):return {'id':n,'revision':1,'digest':hashlib.sha256(n.encode()).hexdigest()}
def repo(z):
 r=z/'r';r.mkdir();subprocess.run(['git','init','-q',str(r)],check=True);subprocess.run(['git','-C',str(r),'config','user.email','q@invalid'],check=True);subprocess.run(['git','-C',str(r),'config','user.name','Q'],check=True)
 (r/'base').write_text('b\n');subprocess.run(['git','-C',str(r),'add','base'],check=True);subprocess.run(['git','-C',str(r),'commit','-q','-m','b'],check=True);p=subprocess.check_output(['git','-C',str(r),'rev-parse','HEAD'],text=True).strip()
 (r/'a.txt').write_text('subject\n');subprocess.run(['git','-C',str(r),'add','a.txt'],check=True);subprocess.run(['git','-C',str(r),'commit','-q','-m','s'],check=True);s=subprocess.check_output(['git','-C',str(r),'rev-parse','HEAD'],text=True).strip();b=subprocess.check_output(['git','-C',str(r),'rev-parse',f'{s}:a.txt'],text=True).strip();return r,p,s,b

def limits(**kw):
 x={'execution_limits_format_revision':1,'profile_id':'qcap:test','profile_revision':1,'max_gate_output_bytes':64,'max_gate_count':8,'max_gate_script_bytes':4096,'max_total_gate_script_bytes':8192,'max_args_per_gate':8,'max_total_arg_bytes':1024,'max_manifest_canonical_bytes':16384,'max_claim_nonclaim_utf8_bytes':2048};x.update(kw);return x

def manifest(cr,p,s,b,defs):
 gs=[]
 for i,body,args in defs:
  f=cr/(i+'.sh');f.write_text('#!/bin/sh\n'+body+'\n');f.chmod(0o755);gs.append({'id':i,'class':'theorem','script':f.name,'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'args':list(args),'timeout_seconds':3})
 gs.sort(key=lambda g:g['id']);return {'capsule_format_revision':1,'theorem_id':'TEST-A3A','theorem_revision':1,'repository_identity':'Luminous-Dynamics/mycelix','product_subject_sha':s,'predecessor_sha':p,'expected_changed_paths':['a.txt'],'expected_object_blobs':{'a.txt':b},'toolchain_profile_ref':pf('tool'),'environment_profile_ref':pf('env'),'gates':gs,'verdict':{'kind':'all','required_gate_ids':[g['id'] for g in gs]},'claim':'claim','nonclaims':['nonclaim']}
def context(m,l):return {'execution_context_format_revision':3,'runner_profile_ref':pf('runner3'),'toolchain_profile_ref':m['toolchain_profile_ref'],'environment_profile_ref':m['environment_profile_ref'],'execution_limits_profile_ref':limits_ref(l),'resolved_runner_commitment':q.runner_commitment(),'resolved_toolchain_commitment':'4'*64,'resolved_environment_commitment':'5'*64}

class Qcap3(unittest.TestCase):
 def test_limits_digest_sensitive(self):
  a=limits();b=limits(max_gate_output_bytes=65);self.assertNotEqual(limits_digest(a),limits_digest(b));self.assertNotEqual(limits_ref(a),limits_ref(b))
 def test_output_exact_limit_passes(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=16);m=manifest(cr,p,s,b,(("a","printf '1234567890abcdef'",()),));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-exact',context(m,l),l);g=x['gate_results'][0];self.assertEqual(x['verdict'],'CompletedConjunctivePass');self.assertEqual(g['captured_output_bytes'],16);self.assertFalse(g['output_truncated'])
 def test_output_limit_plus_one_is_runner_failure_and_suffix_not_run(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=16);m=manifest(cr,p,s,b,(("a","printf '1234567890abcdefX'",()),("b","true",())));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-over',context(m,l),l);a,bg=x['gate_results'];self.assertEqual(x['verdict'],'RunnerInfrastructureFailure');self.assertEqual(a['runner_failure_reason'],'OutputLimitExceeded');self.assertEqual(a['captured_output_bytes'],16);self.assertTrue(a['output_truncated']);self.assertEqual(bg['status'],'GateNotRun')
 def test_unbounded_output_capture_stays_bounded(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=1024);m=manifest(cr,p,s,b,(("a","while :; do printf xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx; done",()),));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-stream',context(m,l),l);g=x['gate_results'][0];self.assertEqual(g['captured_output_bytes'],1024);self.assertEqual(g['runner_failure_reason'],'OutputLimitExceeded')
 def test_context_limits_mismatch_fails_before_gate(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();marker=z/'marker';l=limits();m=manifest(cr,p,s,b,(("a",f"touch {marker}",()),));c=context(m,l);other=limits(max_gate_output_bytes=65)
   with self.assertRaises(CapsuleError):q.run_capsule(m,cr,r,m['repository_identity'],'attempt-mismatch',c,other)
   self.assertFalse(marker.exists())
 def test_receipt_rejects_truncation_reason_contradictions(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits();m=manifest(cr,p,s,b,(("a","true",()),));c=context(m,l);bad=gate_result('a','RunnerInfrastructureFailure',20,b'x',False,'OutputLimitExceeded')
   with self.assertRaises(CapsuleError):compose_receipt_v3(m,'attempt-bad',c,l,[bad])
 def test_gate_fail_can_follow_process_success_after_subject_mutation(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits();m=manifest(cr,p,s,b,(("a","echo changed > a.txt",()),));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-mutate',context(m,l),l);g=x['gate_results'][0];self.assertEqual(g['status'],'GateFail');self.assertEqual(g['effective_exit_code'],10);self.assertEqual(x['verdict'],'CompletedConjunctiveFail')
 def test_resource_exact_boundaries_and_one_under_rejects(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();m=manifest(cr,p,s,b,(("a","true",('xy','z')),));wide=limits();u=account_manifest_resources(m,cr,wide);script_size=(cr/'a.sh').stat().st_size
   exact=limits(max_gate_count=1,max_gate_script_bytes=script_size,max_total_gate_script_bytes=u['total_gate_script_bytes'],max_args_per_gate=2,max_total_arg_bytes=u['total_arg_bytes'],max_manifest_canonical_bytes=u['manifest_canonical_bytes'],max_claim_nonclaim_utf8_bytes=u['claim_nonclaim_utf8_bytes']);account_manifest_resources(m,cr,exact)
   cases=[('max_gate_script_bytes',script_size-1),('max_total_gate_script_bytes',u['total_gate_script_bytes']-1),('max_args_per_gate',1),('max_total_arg_bytes',u['total_arg_bytes']-1),('max_manifest_canonical_bytes',u['manifest_canonical_bytes']-1),('max_claim_nonclaim_utf8_bytes',u['claim_nonclaim_utf8_bytes']-1)]
   for k,v in cases:
    with self.subTest(k=k):
     bad=dict(exact);bad[k]=v
     with self.assertRaises(CapsuleError):account_manifest_resources(m,cr,bad)
 def test_gate_count_over_rejects(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();m=manifest(cr,p,s,b,(("a","true",()),("b","true",())));l=limits(max_gate_count=1)
   with self.assertRaises(CapsuleError):account_manifest_resources(m,cr,l)

 def test_retained_stdout_descriptor_has_short_post_exit_ceiling(self):
  with tempfile.TemporaryDirectory() as d:
   import time
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=64);m=manifest(cr,p,s,b,(("a","(sleep 5) & exit 0",()),));t=time.monotonic();x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-drain',context(m,l),l);elapsed=time.monotonic()-t;g=x['gate_results'][0];self.assertLess(elapsed,2.0);self.assertEqual(g['runner_failure_reason'],'OutputDrainTimeout');self.assertEqual(x['verdict'],'RunnerInfrastructureFailure')

 def test_closed_v3_schemas_and_independent_vectors(self):
  sd=H.parent/'schema'
  for n,k,v in (("qualification-execution-limits-v1.schema.json","execution_limits_format_revision",1),("qualification-execution-context-v3.schema.json","execution_context_format_revision",3),("qualification-attempt-receipt-v3.schema.json","receipt_format_revision",3)):
   x=json.loads((sd/n).read_text());self.assertFalse(x['additionalProperties']);self.assertEqual(x['properties'][k]['const'],v)
  o=H.parent/'vectors'/'verify_qcap_v3_vectors.py';r=subprocess.run([str(o)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stdout+r.stderr);self.assertIn('qcap_v3_independent_vectors=PASS',r.stdout)

 def test_checked_accounting_overflow_fails_closed(self):
  with self.assertRaises(CapsuleError):checked_add(U64_MAX,1,'test')

 def test_receipt_round_trip(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits();m=manifest(cr,p,s,b,(("a","true",()),));c=context(m,l);rr=compose_receipt_v3(m,'attempt-vector',c,l,[gate_result('a','GatePass',0,b'')]);self.assertTrue(verify_receipt_v3(rr,m,l))

if __name__=='__main__':unittest.main()
