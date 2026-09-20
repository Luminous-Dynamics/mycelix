import copy, hashlib, importlib.util, pathlib, tempfile, unittest
P=pathlib.Path(__file__).with_name('github_rest_adapter_policy.py')
S=importlib.util.spec_from_file_location('pol',P); pol=importlib.util.module_from_spec(S); S.loader.exec_module(pol)

def base_result(conclusion='Cancelled',failure='RunnerInfrastructureBeforeTheoremGate',status='Completed',gate='NoTheoremStepExecuted'):
 return {'supported_core_head':pol.SEMANTIC_CORE_HEAD,'core_manifest_v1':{'theorem_id':'AMSAP-004A','repository_id':1176351975,'workflow_path':'.github/workflows/a.yml','qualification_head':'a'*40,'required_jobs':[{'job_key':'qualify'}]},'core_observation_v1':{'repository_id':1176351975,'workflow_run_id':42,'workflow_path':'.github/workflows/a.yml','qualification_head':'a'*40,'jobs':[{'job_id':1,'job_key':'qualify','status':status,'conclusion':conclusion,'gate_execution':gate,'dependency_state':'EligibleForRunner','queue_age_seconds':None,'failure_class':failure}]},'observation_source_authenticity_verified':False,'github_api_response_authenticity_verified':False,'runner_identity_attested':False,'semantic_classification_performed':False,'qualification_authority':False,'evidence_authority':False,'failover_authority':False,'rerun_authority':False,'dispatch_authority':False,'qualification_result':None,'theorem_result':None}
class T(unittest.TestCase):
 def test_stale_tuple_degrades_to_core_valid_unknown(self):
  x=pol._postprocess(base_result('Unknown')); self.assertEqual(x['core_observation_v1']['jobs'][0]['failure_class'],'Unknown')
 def test_terminal_run_with_queued_job_rejected(self):
  with self.assertRaises(pol.PolicyError): pol._snapshot_consistent({'status':'completed'},{'jobs':[{'status':'queued'}]})
 def test_prestart_run_with_completed_job_rejected(self):
  with self.assertRaises(pol.PolicyError): pol._snapshot_consistent({'status':'queued'},{'jobs':[{'status':'completed'}]})
 def test_valid_snapshot_shapes(self):
  pol._snapshot_consistent({'status':'completed'},{'jobs':[{'status':'completed'}]}); pol._snapshot_consistent({'status':'queued'},{'jobs':[{'status':'queued'}]})
 def test_evidence_id_matches_rust_boundary(self):
  self.assertEqual(pol._evidence_id('x'*256,'x'),'x'*256)
  for bad in ('x'*257,'ok\nno'):
   with self.assertRaises(pol.PolicyError): pol._evidence_id(bad,'x')
 def test_all_core_terminal_tuples(self):
  valid=[('Success','NotApplicable','AllRegisteredTheoremStepsExecuted'),('Failure','RegisteredTheoremGate','SomeTheoremStepsExecuted'),('Cancelled','RunnerInfrastructureBeforeTheoremGate','NoTheoremStepExecuted'),('TimedOut','RunnerInfrastructureBeforeTheoremGate','NoTheoremStepExecuted'),('StartupFailure','RunnerInfrastructureBeforeTheoremGate','NoTheoremStepExecuted'),('Unknown','Unknown','NoTheoremStepExecuted')]
  for c,f,g in valid: pol._postprocess(base_result(c,f,'Completed',g))
 def test_invalid_core_tuple_rejected(self):
  with self.assertRaises(pol.PolicyError): pol._postprocess(base_result('Failure','NotApplicable'))
 def test_authority_broadening_rejected(self):
  x=base_result(); x['dispatch_authority']=True
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
 def test_no_semantic_results(self):
  x=base_result(); x['theorem_result']='PASS'
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
 def test_pin_constants(self):
  self.assertEqual(pol.CORE_ADAPTER_GIT_BLOB,'603fd2b77bc588701dbcdae376ae31f5b49ce13b'); self.assertEqual(pol.SEMANTIC_CORE_HEAD,'4190f855eb0f3c03a7a6b0decee84dd7edba07b4')
 def test_loader_executes_private_materialization(self):
  with tempfile.TemporaryDirectory() as td:
   src=pathlib.Path(td)/'core.py'
   fake=b'IMPLEMENTATION_COMMITMENT_SHA256="fake-impl"\nSUPPORTED_CORE_HEAD="fake-head"\nLOADED_FROM=__file__\n'
   src.write_bytes(fake)
   old=(pol.CORE_ADAPTER_GIT_BLOB,pol.CORE_ADAPTER_IMPLEMENTATION,pol.SEMANTIC_CORE_HEAD)
   pol.CORE_ADAPTER_GIT_BLOB=pol._git_blob(fake); pol.CORE_ADAPTER_IMPLEMENTATION='fake-impl'; pol.SEMANTIC_CORE_HEAD='fake-head'
   try:
    mod=pol._load_core(src)
    self.assertNotEqual(pathlib.Path(mod.LOADED_FROM),src)
    self.assertFalse(pathlib.Path(mod.LOADED_FROM).exists())
   finally:
    pol.CORE_ADAPTER_GIT_BLOB,pol.CORE_ADAPTER_IMPLEMENTATION,pol.SEMANTIC_CORE_HEAD=old

 def test_required_job_bound_matches_rust_core(self):
  x=base_result(); x['core_manifest_v1']['required_jobs']=[{'job_key':f'j{i}'} for i in range(129)]; x['core_observation_v1']['jobs']=[{'job_id':i+1,'job_key':f'j{i}','status':'Queued','conclusion':None,'gate_execution':'NoTheoremStepExecuted','dependency_state':'EligibleForRunner','queue_age_seconds':0,'failure_class':'NotApplicable'} for i in range(129)]
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
 def test_duplicate_observed_job_rejected(self):
  x=base_result(); x['core_observation_v1']['jobs'].append(copy.deepcopy(x['core_observation_v1']['jobs'][0]))
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
 def test_manifest_observation_identity_mismatch_rejected(self):
  x=base_result(); x['core_observation_v1']['qualification_head']='b'*40
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
 def test_unsupported_dependency_state_rejected(self):
  x=base_result(); x['core_observation_v1']['jobs'][0]['dependency_state']='Magic'
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
 def test_missing_required_observed_job_rejected(self):
  x=base_result(); x['core_observation_v1']['jobs']=[]
  with self.assertRaises(pol.PolicyError): pol._postprocess(x)
if __name__=='__main__': unittest.main()
