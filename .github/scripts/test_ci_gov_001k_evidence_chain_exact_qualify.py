import importlib.util,json,os,pathlib,sys,tempfile,unittest
from unittest import mock
HERE=pathlib.Path(__file__).parent
S=importlib.util.spec_from_file_location('q',str(HERE/'ci_gov_001k_evidence_chain_exact_qualify.py'))
q=importlib.util.module_from_spec(S);sys.modules[S.name]=q;S.loader.exec_module(q)
class T(unittest.TestCase):
 def test_subject(self):self.assertEqual(q.SUBJECT,'1b0d00342faea04fa09bae05931af7ced21f69dd')
 def test_parent(self):self.assertEqual(q.PARENT,'bf07e3fa9701e947cb7ac5c03e43052661e8a155')
 def test_four_source_paths(self):self.assertEqual(len(q.PATHS),4)
 def test_three_qualifier_paths(self):self.assertEqual(len(q.QUALIFIER_PATHS),3)
 def test_lock_components(self):self.assertEqual(list(q.EXPECTED_LOCK['components']),['A','B','C','D'])
 def test_lock_cases(self):self.assertEqual(q.EXPECTED_LOCK['local_verifier_preflight_cases'],34)
 def test_lock_authority_false(self):self.assertFalse(any(q.EXPECTED_LOCK['authority_ceiling'].values()))
 def test_git_replace_disabled(self):
  with mock.patch.dict(os.environ,{},clear=True):self.assertEqual(q.git_env()['GIT_NO_REPLACE_OBJECTS'],'1')
 def test_git_redirect_rejected(self):
  with mock.patch.dict(os.environ,{'GIT_DIR':'/evil'},clear=True):
   with self.assertRaises(q.QualificationError):q.git_env()
 def test_lock_exact(self):
  raw=json.dumps(q.EXPECTED_LOCK).encode()
  with mock.patch.object(q,'git',return_value=raw):self.assertEqual(q.verify_lock(),q.sha256(raw))
 def test_lock_drift_rejected(self):
  x=json.loads(json.dumps(q.EXPECTED_LOCK));x['aggregate_contract']['canonical_component_order']=['D','C','B','A']
  with mock.patch.object(q,'git',return_value=json.dumps(x).encode()):
   with self.assertRaises(q.QualificationError):q.verify_lock()
 def test_receipt_inside_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   root=pathlib.Path(td)
   with mock.patch.object(q,'git',return_value=str(root)):
    with self.assertRaises(q.QualificationError):q.outside(root/'receipt.json')
 def test_source_audit_rejects_network(self):
  with mock.patch.object(q,'git',return_value='import urllib.request'):
   with self.assertRaises(q.QualificationError):q.source_audit()
 def test_source_audit_accepts_plain(self):
  with mock.patch.object(q,'git',return_value='import json\nprint(1)'):q.source_audit()
 def test_test_command_isolated(self):
  cp=mock.Mock(returncode=0,stdout=b'ok',stderr=b'')
  with tempfile.TemporaryDirectory() as td,mock.patch.object(q.tempfile,'TemporaryDirectory') as mgr,mock.patch.object(q,'git',return_value=b'x'),mock.patch.object(q.subprocess,'run',return_value=cp) as run:
   mgr.return_value.__enter__.return_value=td;q.run_tests();self.assertEqual(run.call_args.args[0][1:],['-E','-s','-S','-B','test_ci_gov_001k_evidence_chain_verify.py'])
 def test_schema(self):self.assertEqual(q.SCHEMA,'mycelix.ci-gov.001k.evidence-chain-qualification.receipt.v0.1')
if __name__=='__main__':unittest.main()
