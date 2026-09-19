import importlib.util, json, os, pathlib, sys, tempfile, unittest
from unittest import mock
HERE=pathlib.Path(__file__).parent
SPEC=importlib.util.spec_from_file_location("q",str(HERE/"ci_gov_001k_exact_qualify.py"))
q=importlib.util.module_from_spec(SPEC); sys.modules[SPEC.name]=q; SPEC.loader.exec_module(q)
class T(unittest.TestCase):
    def test_expected_lock_policy(self):
        p=q.EXPECTED_LOCK["policy"]; self.assertEqual(p["queue"],"max"); self.assertFalse(p["cancel_in_progress"]); self.assertEqual(p["max_pending"],8); self.assertEqual(p["platform_pending_cap"],100); self.assertFalse(p["ready_admits"]); self.assertEqual(p["max_snapshot_age_seconds"],30)
    def test_commitment_changes(self): self.assertNotEqual(q.sha256(q.canonical({"x":1})),q.sha256(q.canonical({"x":2})))
    def test_git_env_disables_replace(self):
        with mock.patch.dict(os.environ,{},clear=True): self.assertEqual(q.git_env()["GIT_NO_REPLACE_OBJECTS"],"1")
    def test_git_env_rejects_redirect(self):
        with mock.patch.dict(os.environ,{"GIT_DIR":"/evil"},clear=True):
            with self.assertRaises(q.QualificationError): q.git_env()
    def test_identity_fail_closed(self):
        with mock.patch.object(q,"reject_git_rewrite_state"), mock.patch.object(q,"git",return_value="wrong"):
            with self.assertRaises(q.QualificationError): q.verify_identity()
    def test_lock_exact(self):
        raw=json.dumps(q.EXPECTED_LOCK).encode()
        with mock.patch.object(q,"git",return_value=raw): self.assertEqual(q.verify_lock(),q.sha256(raw))
    def test_lock_rejects_change(self):
        x=json.loads(json.dumps(q.EXPECTED_LOCK)); x["policy"]["max_pending"]=9
        with mock.patch.object(q,"git",return_value=json.dumps(x).encode()):
            with self.assertRaises(q.QualificationError): q.verify_lock()
    def test_receipt_inside_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            with mock.patch.object(q,"git",return_value=str(root)):
                with self.assertRaises(q.QualificationError): q.ensure_receipt_outside_checkout(root/"x")
    def test_receipt_outside_accepted(self):
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)/"repo"; root.mkdir()
            with mock.patch.object(q,"git",return_value=str(root)): q.ensure_receipt_outside_checkout(pathlib.Path(td)/"x")
    def test_no_network_or_mutator(self):
        text=(HERE/"ci_gov_001k_exact_qualify.py").read_text()
        for s in ("api.github.com","urllib.request","requests.","cancel_run(","rerun","merge_pull_request"): self.assertNotIn(s,text)
    def test_qualifier_paths_exact(self): self.assertEqual(len(q.QUALIFIER_PATHS),3)
    def test_source_paths_exact(self): self.assertEqual(len(q.PATHS),4)
    def test_exec_paths_exact(self): self.assertEqual(len(q.EXEC_PATHS),2)
    def test_schema_exact(self): self.assertEqual(q.SCHEMA,"mycelix.ci-gov.001k.source-qualification.receipt.v0.2")
    def test_subject_execution_command_isolated(self):
        cp=mock.Mock(returncode=0,stdout=b"ok",stderr=b"")
        with tempfile.TemporaryDirectory() as td, mock.patch.object(q.tempfile,"TemporaryDirectory") as mgr, mock.patch.object(q,"git",return_value=b"x"), mock.patch.object(q.subprocess,"run",return_value=cp) as run:
            mgr.return_value.__enter__.return_value=td; q.run_subject_tests(); cmd=run.call_args.args[0]; self.assertEqual(cmd[1:],["-E","-s","-S","-B","test_ci_qualification_capacity_oracle.py"])
if __name__=="__main__": unittest.main()
