import importlib.util,json,os,pathlib,sys,tempfile,unittest
from unittest import mock
HERE=pathlib.Path(__file__).parent
S=importlib.util.spec_from_file_location("q",str(HERE/"ci_gov_001k_observer_exact_qualify.py"))
q=importlib.util.module_from_spec(S); sys.modules[S.name]=q; S.loader.exec_module(q)
class T(unittest.TestCase):
    def test_lock_endpoint(self): self.assertEqual(q.EXPECTED_LOCK["endpoint"]["method"],"GET")
    def test_lock_bounds(self): self.assertEqual(q.EXPECTED_LOCK["platform_bounds"],{"max_active":1,"max_pending":100,"allowed_member_statuses":["in_progress","pending"]})
    def test_no_mutation_authority(self): self.assertFalse(q.EXPECTED_LOCK["authority_ceiling"]["actions_mutation"]); self.assertFalse(q.EXPECTED_LOCK["authority_ceiling"]["queue_admission_authority"])
    def test_git_replace_disabled(self):
        with mock.patch.dict(os.environ,{},clear=True): self.assertEqual(q.git_env()["GIT_NO_REPLACE_OBJECTS"],"1")
    def test_redirect_env_rejected(self):
        with mock.patch.dict(os.environ,{"GIT_DIR":"/x"},clear=True):
            with self.assertRaises(q.QualificationError): q.git_env()
    def test_identity_fail_closed(self):
        with mock.patch.object(q,"reject_rewrite"),mock.patch.object(q,"git",return_value="wrong"):
            with self.assertRaises(q.QualificationError): q.verify_identity()
    def test_lock_exact(self):
        raw=json.dumps(q.EXPECTED_LOCK).encode()
        with mock.patch.object(q,"git",return_value=raw): self.assertEqual(q.verify_lock(),q.sha256(raw))
    def test_lock_drift_rejected(self):
        x=json.loads(json.dumps(q.EXPECTED_LOCK)); x["endpoint"]["method"]="POST"
        with mock.patch.object(q,"git",return_value=json.dumps(x).encode()):
            with self.assertRaises(q.QualificationError): q.verify_lock()
    def test_three_qualifier_paths(self): self.assertEqual(len(q.QUALIFIER_PATHS),3)
    def test_four_subject_paths(self): self.assertEqual(len(q.PATHS),4)
    def test_two_exec_paths(self): self.assertEqual(len(q.EXEC_PATHS),2)
    def test_schema(self): self.assertEqual(q.SCHEMA,"mycelix.ci-gov.001k.capacity-observer-qualification.receipt.v0.1")
    def test_receipt_inside_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            with mock.patch.object(q,"git",return_value=str(root)):
                with self.assertRaises(q.QualificationError): q.outside(root/"x")
    def test_qualifier_has_no_network_client(self):
        text=(HERE/"ci_gov_001k_observer_exact_qualify.py").read_text()
        for s in ("urllib.request","requests.","urlopen(","cancel_run(","rerun","merge_pull_request"): self.assertNotIn(s,text)
    def test_test_command_isolated(self):
        cp=mock.Mock(returncode=0,stdout=b"x",stderr=b"")
        with tempfile.TemporaryDirectory() as td,mock.patch.object(q.tempfile,"TemporaryDirectory") as mgr,mock.patch.object(q,"git",return_value=b"x"),mock.patch.object(q.subprocess,"run",return_value=cp) as run:
            mgr.return_value.__enter__.return_value=td; q.run_tests()
            self.assertEqual(run.call_args.args[0][1:],["-E","-s","-S","-B","test_ci_qualification_capacity_observer.py"])
if __name__=="__main__":unittest.main()
