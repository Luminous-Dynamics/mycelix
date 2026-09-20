#!/usr/bin/env python3
from __future__ import annotations
import copy, hashlib, importlib.util, json, shutil, subprocess, tempfile, unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
def loadmod(name,path):
    spec=importlib.util.spec_from_file_location(name,path); assert spec and spec.loader
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); return mod
m=loadmod("oracle_policy",HERE/"workflow_object_oracle_policy.py")
core=loadmod("oracle_core",HERE/"workflow_object_oracle.py")
fixtures=loadmod("oracle_fixtures",HERE/"test_workflow_object_oracle.py")

class T(unittest.TestCase):
    def fixture(self):
        td,repo,commit,blob=fixtures.make_repo();self.addCleanup(td.cleanup)
        i=fixtures.intent(commit,blob);w=fixtures.wfid(repo,i)
        return td,repo,i,w

    def test_exact_pair_binds_repository_identity_with_zero_authority(self):
        _,repo,i,w=self.fixture()
        out=m.evaluate(repo,w,i)
        self.assertEqual(out["classification"],"WORKFLOW_OBJECT_CHAIN_CONFIRMED")
        self.assertTrue(out["repository_identity_verified"])
        self.assertEqual(out["repository_origin"],"Luminous-Dynamics/mycelix")
        self.assertEqual(out["workflow_object_oracle_core_git_blob_sha1"],m.CORE_BLOB)
        self.assertEqual(out["workflow_object_oracle_core_implementation_commitment"],m.CORE_IMPL)
        self.assertRegex(out["workflow_object_oracle_policy_commitment"],r"^[0-9a-f]{64}$")
        for field in ("registration_authority","workflow_dispatched","qualification_authority","git_implementation_trust_verified","receipt_authenticity_verified","sha1_collision_resistance_claimed"):
            self.assertFalse(out[field])
        self.assertIsNone(out["qualification_result"])

    def test_same_objects_wrong_repository_origin_refused(self):
        td,repo,i,w=self.fixture()
        other=Path(td.name)/"other"
        subprocess.run(["git","clone","--local","-q",str(repo),str(other)],check=True)
        fixtures.git(other,"remote","set-url","origin","https://github.com/Other/repo.git")
        with self.assertRaisesRegex(m.Refused,"repository origin mismatch"):
            m.evaluate(other,w,i)

    def test_origin_change_after_oracle_refused(self):
        _,repo,i,w=self.fixture()
        with patch.object(m,"observed_origin",side_effect=["Luminous-Dynamics/mycelix","Other/repo"]):
            with self.assertRaisesRegex(m.Refused,"origin changed"):
                m.evaluate_with_core(core,None,repo,w,i)

    def test_origin_normalization_supported_forms(self):
        expected="Luminous-Dynamics/mycelix"
        for raw in (
            "https://github.com/Luminous-Dynamics/mycelix.git",
            "http://github.com/Luminous-Dynamics/mycelix.git",
            "git://github.com/Luminous-Dynamics/mycelix.git",
            "ssh://git@github.com/Luminous-Dynamics/mycelix.git",
            "git@github.com:Luminous-Dynamics/mycelix.git",
        ):
            with self.subTest(raw=raw):self.assertEqual(m.normalize_origin(raw),expected)
        self.assertIsNone(m.normalize_origin("https://example.com/Luminous-Dynamics/mycelix.git"))

    def test_core_file_is_pinned_by_blob_and_implementation(self):
        p=HERE/"workflow_object_oracle.py"
        self.assertEqual(m.verify_core(p),p.read_bytes())
        with tempfile.TemporaryDirectory() as td:
            q=Path(td)/"core.py";q.write_bytes(p.read_bytes()+b"# drift")
            with self.assertRaisesRegex(m.Invalid,"Git blob mismatch"):m.verify_core(q)

    def test_core_authority_broadening_rejected(self):
        _,repo,i,w=self.fixture()
        inner=core.confirm(repo,w,i)
        inner["workflow_dispatched"]=True
        with patch.object(core,"confirm",return_value=inner):
            with self.assertRaisesRegex(m.Invalid,"broadened authority"):
                m.evaluate_with_core(core,None,repo,w,i)

    def test_core_result_identity_mismatch_rejected(self):
        _,repo,i,w=self.fixture()
        inner=core.confirm(repo,w,i);inner["workflow_object_oracle_implementation_commitment"]="0"*64
        with patch.object(core,"confirm",return_value=inner):
            with self.assertRaisesRegex(m.Invalid,"result implementation identity mismatch"):
                m.evaluate_with_core(core,None,repo,w,i)

    def test_core_errors_are_mapped_at_input_origin_and_confirm_boundaries(self):
        class FakeInvalid(RuntimeError):pass
        def bad_load(*a,**k):raise FakeInvalid("bad input")
        fake=SimpleNamespace(Invalid=FakeInvalid,Refused=None,Unavailable=None,load=bad_load)
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/"bad.json";p.write_text("not-json")
            with self.assertRaisesRegex(m.Invalid,"bad input"):m.core_load(fake,p,"fixture")
        def bad_bind(*a,**k):raise FakeInvalid("bad git")
        fake=SimpleNamespace(Invalid=FakeInvalid,Refused=None,Unavailable=None,bind_git=bad_bind)
        with self.assertRaisesRegex(m.Invalid,"bad git"):m.observed_origin(fake,Path("."),{})

    def test_policy_commitment_binds_repository_marker(self):
        _,repo,i,w=self.fixture()
        out=m.evaluate(repo,w,i);original=out["workflow_object_oracle_policy_commitment"]
        material=dict(out);material.pop("workflow_object_oracle_policy_commitment")
        self.assertEqual(original,m.policy_commitment(material))
        material["repository_identity_verified"]=False
        self.assertNotEqual(original,m.policy_commitment(material))

    def test_failure_result_never_claims_repository_or_authority(self):
        for cls in ("REFUSED","UNAVAILABLE","INVALID"):
            out=m.failure(cls,"fixture")
            self.assertFalse(out["repository_identity_verified"])
            for field in ("registration_authority","workflow_dispatched","workflow_identity_verified","workflow_object_chain_confirmed","qualification_authority","git_implementation_trust_verified","receipt_authenticity_verified","sha1_collision_resistance_claimed"):
                self.assertFalse(out[field])
            self.assertIsNone(out["qualification_result"])

    def test_implementation_commitment_frozen_at_import(self):
        with tempfile.TemporaryDirectory() as td:
            src=Path(m.__file__);dst=Path(td)/"policy.py";shutil.copy2(src,dst)
            mod=loadmod("policy_copy",dst);before=mod.IMPLEMENTATION_COMMITMENT
            dst.write_text(dst.read_text()+"\n# drift\n")
            self.assertEqual(before,mod.IMPLEMENTATION_COMMITMENT)
            self.assertNotEqual(before,hashlib.sha256(mod.IMPL_DOMAIN+dst.read_bytes()).hexdigest())

    def test_static_surface_has_no_network_or_mutating_git_commands(self):
        source=Path(m.__file__).read_text().lower()
        for forbidden in ("import requests","import urllib","from github","import github","git fetch","git push","git checkout","git reset","update-ref","gh workflow","curl ","wget "):
            self.assertNotIn(forbidden,source)

if __name__=="__main__":unittest.main()