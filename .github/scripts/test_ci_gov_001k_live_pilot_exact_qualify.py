import importlib.util,json,os,pathlib,sys,tempfile,unittest
from unittest import mock
HERE=pathlib.Path(__file__).parent
S=importlib.util.spec_from_file_location('q',str(HERE/'ci_gov_001k_live_pilot_exact_qualify.py'));q=importlib.util.module_from_spec(S);sys.modules[S.name]=q;S.loader.exec_module(q)
class T(unittest.TestCase):
 def test_subject(self):self.assertEqual(q.SUBJECT,'55b6de9f8e39ea9c639b890730f4f48b9889267c')
 def test_fixture_sha(self):self.assertEqual(q.FIXTURE_SHA256,'c1479745b168901f67931ff392b7c3e70ec4959a3841328943380719e9117cfe')
 def test_lock_queue(self):self.assertEqual(q.EXPECTED_LOCK['pilot_contract']['queue'],'max');self.assertFalse(q.EXPECTED_LOCK['pilot_contract']['cancel_in_progress'])
 def test_lock_canary(self):self.assertTrue(q.EXPECTED_LOCK['bounded_protocol']['admit_canary_first']);self.assertEqual(q.EXPECTED_LOCK['bounded_protocol']['max_new_runs_when_runner_unavailable'],1)
 def test_authority(self):self.assertFalse(q.EXPECTED_LOCK['authority_ceiling']['workflow_activation']);self.assertFalse(q.EXPECTED_LOCK['authority_ceiling']['actions_mutation'])
 def test_path_counts(self):self.assertEqual(len(q.PATHS),5);self.assertEqual(len(q.EXEC_PATHS),3);self.assertEqual(len(q.QUALIFIER_PATHS),3)
 def test_replace_disabled(self):
  with mock.patch.dict(os.environ,{},clear=True):self.assertEqual(q.git_env()['GIT_NO_REPLACE_OBJECTS'],'1')
 def test_redirect_rejected(self):
  with mock.patch.dict(os.environ,{'GIT_DIR':'/x'},clear=True):
   with self.assertRaises(q.QualificationError):q.git_env()
 def test_lock_exact(self):
  raw=json.dumps(q.EXPECTED_LOCK).encode()
  with mock.patch.object(q,'git',return_value=raw):self.assertEqual(q.verify_lock(),q.sha256(raw))
 def test_lock_drift(self):
  x=json.loads(json.dumps(q.EXPECTED_LOCK));x['pilot_contract']['queue']='single'
  with mock.patch.object(q,'git',return_value=json.dumps(x).encode()):
   with self.assertRaises(q.QualificationError):q.verify_lock()
 def test_receipt_inside_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   root=pathlib.Path(td)
   with mock.patch.object(q,'git',return_value=str(root)):
    with self.assertRaises(q.QualificationError):q.outside(root/'x')
 def test_no_network_client(self):
  t=(HERE/'ci_gov_001k_live_pilot_exact_qualify.py').read_text()
  for s in ('urllib.request','requests.','urlopen(','add_labels','cancel_run(','workflow_dispatch'):self.assertNotIn(s,t)
 def test_test_command_isolated(self):
  cp=mock.Mock(returncode=0,stdout=b'x',stderr=b'')
  with tempfile.TemporaryDirectory() as td,mock.patch.object(q.tempfile,'TemporaryDirectory') as mgr,mock.patch.object(q,'git',return_value=b'x'),mock.patch.object(q.subprocess,'run',return_value=cp) as run:
   mgr.return_value.__enter__.return_value=td;q.run_tests();self.assertEqual(run.call_args.args[0][1:],['-E','-s','-S','-B','test_ci_gov_001k_pilot_fixture_verify.py'])
 def test_schema(self):self.assertEqual(q.SCHEMA,'mycelix.ci-gov.001k.live-pilot-fixture-qualification.receipt.v0.1')
 def test_evidence_states(self):self.assertEqual(q.EXPECTED_LOCK['evidence_states'],['PASS','RUNNER_UNAVAILABLE','INCONCLUSIVE','FAIL'])
if __name__=='__main__':unittest.main()
