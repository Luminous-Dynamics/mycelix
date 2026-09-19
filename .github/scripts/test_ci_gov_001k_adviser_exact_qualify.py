import importlib.util,json,os,pathlib,sys,tempfile,unittest
from unittest import mock
HERE=pathlib.Path(__file__).parent
S=importlib.util.spec_from_file_location('q',str(HERE/'ci_gov_001k_adviser_exact_qualify.py'));q=importlib.util.module_from_spec(S);sys.modules[S.name]=q;S.loader.exec_module(q)
class T(unittest.TestCase):
 def test_subject(self):self.assertEqual(q.SUBJECT,'8ee69ad386e4189fb4b1d549dc95ea649c282c1d')
 def test_parent(self):self.assertEqual(q.PARENT,'d3de3a4d24c7459b80f939fee5c3c2bb4843ad71')
 def test_lock_no_mutation(self):self.assertFalse(q.EXPECTED_LOCK['authority_ceiling']['label_mutation']);self.assertFalse(q.EXPECTED_LOCK['authority_ceiling']['actions_mutation'])
 def test_lock_budget(self):self.assertEqual(q.EXPECTED_LOCK['decision_contract']['soft_pending_budget'],8)
 def test_three_reads(self):self.assertEqual(q.EXPECTED_LOCK['read_contract']['max_reads'],3)
 def test_paths(self):self.assertEqual(len(q.PATHS),4);self.assertEqual(len(q.INHERITED),4);self.assertEqual(len(q.QUALIFIER_PATHS),3);self.assertEqual(len(q.EXEC_PATHS),4)
 def test_replace_disabled(self):
  with mock.patch.dict(os.environ,{},clear=True):self.assertEqual(q.git_env()['GIT_NO_REPLACE_OBJECTS'],'1')
 def test_redirect_rejected(self):
  with mock.patch.dict(os.environ,{'GIT_DIR':'/evil'},clear=True):
   with self.assertRaises(q.QualificationError):q.git_env()
 def test_lock_exact(self):
  raw=json.dumps(q.EXPECTED_LOCK).encode()
  with mock.patch.object(q,'git',return_value=raw):self.assertEqual(q.verify_lock(),q.sha256(raw))
 def test_lock_drift(self):
  x=json.loads(json.dumps(q.EXPECTED_LOCK));x['decision_contract']['soft_pending_budget']=9
  with mock.patch.object(q,'git',return_value=json.dumps(x).encode()):
   with self.assertRaises(q.QualificationError):q.verify_lock()
 def test_receipt_inside_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   root=pathlib.Path(td)
   with mock.patch.object(q,'git',return_value=str(root)):
    with self.assertRaises(q.QualificationError):q.outside(root/'x')
 def test_no_network_client(self):
  t=(HERE/'ci_gov_001k_adviser_exact_qualify.py').read_text()
  for s in ('urllib.request','requests.','urlopen(','add_labels','cancel_run(','merge_pull_request'):self.assertNotIn(s,t)
 def test_test_command_isolated(self):
  cp=mock.Mock(returncode=0,stdout=b'x',stderr=b'')
  with tempfile.TemporaryDirectory() as td,mock.patch.object(q.tempfile,'TemporaryDirectory') as mgr,mock.patch.object(q,'git',return_value=b'x'),mock.patch.object(q.subprocess,'run',return_value=cp) as run:
   mgr.return_value.__enter__.return_value=td;q.run_tests();self.assertEqual(run.call_args.args[0][1:],['-E','-s','-S','-B','test_ci_qualification_admission_adviser.py'])
 def test_schema(self):self.assertEqual(q.SCHEMA,'mycelix.ci-gov.001k.admission-adviser-qualification.receipt.v0.1')
 def test_proposition_nonauthority(self):
  self.assertFalse(q.EXPECTED_LOCK['authority_ceiling']['product_pass_authority']);self.assertFalse(q.EXPECTED_LOCK['authority_ceiling']['scientific_pass_authority'])
if __name__=='__main__':unittest.main()
