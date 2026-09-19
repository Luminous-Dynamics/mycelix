#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("workflow_identity", HERE / "workflow_identity.py")
assert SPEC and SPEC.loader
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

H64 = "a" * 64
H64B = "b" * 64
POLICY_CORE = "1e965637f95566e286c5c5c01f4ac7e7b0137368c5dffd27c11bf63119759ecc"
SUPPORTED_PREFLIGHT = "1038c07ed7529dff979edba34f44428a759ff6e7fb85a74b862ba75701ec421e"
SUPPORTED_ENV = "89838000d0f673b669fbf00ad648a86c54638e2c637a73760db54d1cd8cae855"
WORKFLOW_PATH = ".github/workflows/amsap-004a-subject-topology.yml"


def git(repo: Path, *args: str, env: dict[str, str] | None = None) -> str:
    cp = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    return cp.stdout.strip()


def make_repo(*, workflow_mode: int = 0o644, workflow_kind: str = "file") -> tuple[tempfile.TemporaryDirectory, Path, str, str]:
    td = tempfile.TemporaryDirectory()
    repo = Path(td.name) / "repo"
    repo.mkdir()
    git(repo, "init", "-q")
    git(repo, "config", "user.email", "test@example.invalid")
    git(repo, "config", "user.name", "Test")
    git(repo, "remote", "add", "origin", "https://github.com/Luminous-Dynamics/mycelix.git")
    wf = repo / WORKFLOW_PATH
    wf.parent.mkdir(parents=True)
    if workflow_kind == "file":
        wf.write_text("name: exact\non: workflow_dispatch\n", encoding="utf-8")
        wf.chmod(workflow_mode)
    elif workflow_kind == "symlink":
        target = repo / "target.yml"
        target.write_text("name: target\n", encoding="utf-8")
        wf.symlink_to(Path("../../target.yml"))
    else:
        raise ValueError(workflow_kind)
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "fixture")
    commit = git(repo, "rev-parse", "HEAD")
    blob = git(repo, "rev-parse", f"{commit}:{WORKFLOW_PATH}")
    return td, repo, commit, blob


def make_intent(commit: str, blob: str, **updates) -> dict:
    i = {
        "schema": "mycelix-qualification-registration-intent-v1",
        "intent_id": "amsap-004a-r1-request-v1",
        "repository": "Luminous-Dynamics/mycelix",
        "subject_sha": "1" * 40,
        "predecessor_sha": "2" * 40,
        "preflight_profile_id": "amsap-004a-rustfmt-v1",
        "preflight_profile_commitment": H64,
        "preflight_implementation_commitment": SUPPORTED_PREFLIGHT,
        "environment_adapter_implementation_commitment": SUPPORTED_ENV,
        "preflight_environment_commitment": H64B,
        "qualification_workflow_path": WORKFLOW_PATH,
        "qualification_workflow_commit_sha": commit,
        "qualification_workflow_blob_sha1": blob,
        "registration_mode": "manual-request-v1",
    }
    i.update(updates)
    return i


def make_policy(intent: dict, **updates) -> dict:
    p = {
        "schema": m.POLICY_SCHEMA,
        "classification": "ADMISSIBLE_TO_REQUEST",
        "intent_commitment": m.commit(m.INTENT_DOMAIN, intent),
        "preflight_receipt_commitment": "3" * 64,
        "repository": intent["repository"],
        "subject_sha": intent["subject_sha"],
        "predecessor_sha": intent["predecessor_sha"],
        "qualification_workflow_path": intent["qualification_workflow_path"],
        "qualification_workflow_commit_sha": intent["qualification_workflow_commit_sha"],
        "qualification_workflow_blob_sha1": intent["qualification_workflow_blob_sha1"],
        "registration_mode": intent["registration_mode"],
        "registration_authority": False,
        "workflow_dispatched": False,
        "workflow_identity_verified": False,
        "qualification_result": None,
        "qualification_authority": False,
        "registration_admission_policy_implementation_commitment": m.POLICY_IMPL,
        "registration_admission_core_implementation_commitment": POLICY_CORE,
        "supported_preflight_implementation_commitment": SUPPORTED_PREFLIGHT,
        "supported_environment_adapter_implementation_commitment": SUPPORTED_ENV,
        "producer_revision_supported": True,
        "receipt_authenticity_verified": False,
        "registration_admission_core_commitment": "4" * 64,
    }
    p.update(updates)
    material = dict(p)
    material["registration_admission_policy_commitment"] = "0" * 64
    material.pop("registration_admission_policy_commitment")
    p["registration_admission_policy_commitment"] = m.commit(m.POLICY_DOMAIN, material)
    return p


class WorkflowIdentityTests(unittest.TestCase):
    def test_exact_commit_path_blob_verifies_with_zero_authority(self):
        td, repo, commit, blob = make_repo()
        self.addCleanup(td.cleanup)
        i = make_intent(commit, blob)
        p = make_policy(i)
        out = m.verify(repo, p, i)
        self.assertEqual(out["classification"], "WORKFLOW_IDENTITY_VERIFIED")
        self.assertTrue(out["workflow_identity_verified"])
        self.assertFalse(out["workflow_dispatched"])
        self.assertFalse(out["registration_authority"])
        self.assertFalse(out["qualification_authority"])
        self.assertIsNone(out["qualification_result"])
        self.assertFalse(out["receipt_authenticity_verified"])
        self.assertEqual(out["tree_entry"]["oid"], blob)
        self.assertEqual(out["git"]["object_format"], "sha1")
        self.assertRegex(out["workflow_identity_verification_commitment"], r"^[0-9a-f]{64}$")

    def test_wrong_workflow_blob_refuses(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, "f" * 40); p = make_policy(i)
        with self.assertRaisesRegex(m.Refused, "workflow blob identity mismatch"):
            m.verify(repo, p, i)

    def test_missing_exact_commit_is_unavailable(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent("f" * 40, blob); p = make_policy(i)
        with self.assertRaisesRegex(m.Unavailable, "workflow commit unavailable locally"):
            m.verify(repo, p, i)

    def test_missing_workflow_path_refuses(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob, qualification_workflow_path=".github/workflows/missing.yml"); p = make_policy(i)
        with self.assertRaisesRegex(m.Refused, "workflow path does not resolve exactly once"):
            m.verify(repo, p, i)

    def test_symlink_workflow_refuses(self):
        td, repo, commit, blob = make_repo(workflow_kind="symlink"); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        with self.assertRaisesRegex(m.Refused, "not canonical regular blob"):
            m.verify(repo, p, i)

    def test_executable_workflow_refuses(self):
        td, repo, commit, blob = make_repo(workflow_mode=0o755); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        with self.assertRaisesRegex(m.Refused, "not canonical regular blob"):
            m.verify(repo, p, i)

    def test_noncanonical_or_out_of_scope_paths_are_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        for path in ("../x.yml", "/tmp/x.yml", ".github/workflows/a/../x.yml", ".github/workflows/a//x.yml", "workflow.yml"):
            with self.subTest(path=path):
                i = make_intent(commit, blob, qualification_workflow_path=path)
                with self.assertRaisesRegex(m.Invalid, "invalid workflow path"):
                    m.validate_intent(i)

    def test_repository_origin_mismatch_refuses(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        git(repo, "remote", "set-url", "origin", "https://github.com/Other/repo.git")
        i = make_intent(commit, blob); p = make_policy(i)
        with self.assertRaisesRegex(m.Refused, "repository origin mismatch"):
            m.verify(repo, p, i)

    def test_branch_move_does_not_change_exact_commit_result(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        first = m.verify(repo, p, i)
        (repo / "README.md").write_text("branch moved\n")
        git(repo, "add", "README.md"); git(repo, "commit", "-q", "-m", "move branch")
        second = m.verify(repo, p, i)
        self.assertEqual(first["tree_entry"], second["tree_entry"])
        self.assertEqual(first["qualification_workflow_commit_sha"], commit)
        self.assertNotEqual(first["caller_head"], second["caller_head"])

    def test_dirty_caller_state_is_preserved(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        tracked = repo / WORKFLOW_PATH
        tracked.write_text(tracked.read_text() + "# dirty\n")
        (repo / "untracked.txt").write_text("u\n")
        (repo / ".gitignore").write_text("ignored.txt\n")
        (repo / "ignored.txt").write_text("i\n")
        before_head = git(repo, "rev-parse", "HEAD")
        before_status = subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain=v1", "-z", "--untracked-files=all", "--ignored=matching"])
        i = make_intent(commit, blob); p = make_policy(i)
        m.verify(repo, p, i)
        after_head = git(repo, "rev-parse", "HEAD")
        after_status = subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain=v1", "-z", "--untracked-files=all", "--ignored=matching"])
        self.assertEqual(before_head, after_head)
        self.assertEqual(before_status, after_status)

    def test_wrong_004c_policy_implementation_is_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i, registration_admission_policy_implementation_commitment="0" * 64)
        material = dict(p); material.pop("registration_admission_policy_commitment")
        p["registration_admission_policy_commitment"] = m.commit(m.POLICY_DOMAIN, material)
        with self.assertRaisesRegex(m.Invalid, "unsupported 004C policy implementation"):
            m.verify(repo, p, i)

    def test_nonadmissible_004c_policy_refuses(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i, classification="REFUSED")
        material = dict(p); material.pop("registration_admission_policy_commitment")
        p["registration_admission_policy_commitment"] = m.commit(m.POLICY_DOMAIN, material)
        with self.assertRaisesRegex(m.Refused, "004C result is not admissible"):
            m.verify(repo, p, i)

    def test_004c_authority_broadening_is_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob)
        for field, value in (("registration_authority", True), ("workflow_dispatched", True), ("workflow_identity_verified", True), ("qualification_authority", True), ("qualification_result", "PASS")):
            with self.subTest(field=field):
                p = make_policy(i, **{field: value})
                material = dict(p); material.pop("registration_admission_policy_commitment")
                p["registration_admission_policy_commitment"] = m.commit(m.POLICY_DOMAIN, material)
                with self.assertRaisesRegex(m.Invalid, "broadened authority|authority/provenance"):
                    m.verify(repo, p, i)

    def test_intent_commitment_mismatch_refuses(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        p["intent_commitment"] = H64
        material = dict(p); material.pop("registration_admission_policy_commitment")
        p["registration_admission_policy_commitment"] = m.commit(m.POLICY_DOMAIN, material)
        with self.assertRaisesRegex(m.Refused, "intent commitment mismatch"):
            m.verify(repo, p, i)

    def test_004c_field_mismatch_refuses_even_with_valid_policy_commitment(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        p["subject_sha"] = "9" * 40
        material = dict(p); material.pop("registration_admission_policy_commitment")
        p["registration_admission_policy_commitment"] = m.commit(m.POLICY_DOMAIN, material)
        with self.assertRaisesRegex(m.Refused, "004C field mismatch: subject_sha"):
            m.verify(repo, p, i)

    def test_unsupported_object_format_is_unavailable(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        real_run = m.run
        def fake_run(g, r, e, args, allow_fail=False):
            if args == ["rev-parse", "--show-object-format"]:
                return b"sha256\n"
            return real_run(g, r, e, args, allow_fail)
        with patch.object(m, "run", side_effect=fake_run):
            with self.assertRaisesRegex(m.Unavailable, "unsupported Git object format"):
                m.verify(repo, p, i)

    def test_git_executable_drift_before_verification_is_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        with patch.object(m, "git_same", return_value=False):
            with self.assertRaisesRegex(m.Invalid, "drift before verification"):
                m.verify(repo, p, i)

    def test_git_executable_drift_after_verification_is_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        with patch.object(m, "git_same", side_effect=[True, False]):
            with self.assertRaisesRegex(m.Invalid, "drift after verification"):
                m.verify(repo, p, i)

    def test_closed_git_environment_disables_lazy_fetch(self):
        with tempfile.TemporaryDirectory() as d:
            e = m.env(Path(d) / "home", Path(shutil.which("git") or "/usr/bin/git"))
            self.assertEqual(e["GIT_NO_LAZY_FETCH"], "1")
            self.assertEqual(e["GIT_OPTIONAL_LOCKS"], "0")
            self.assertEqual(e["GIT_TERMINAL_PROMPT"], "0")
            self.assertEqual(e["GIT_CONFIG_NOSYSTEM"], "1")
            self.assertEqual(e["GIT_CONFIG_GLOBAL"], os.devnull)

    def test_duplicate_json_keys_and_non_i_json_are_invalid(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x.json"
            p.write_text('{"a":1,"a":2}')
            with self.assertRaisesRegex(m.Invalid, "duplicate key"):
                m.load(p, "fixture")
            p.write_text('{"a":NaN}')
            with self.assertRaisesRegex(m.Invalid, "non-I-JSON"):
                m.load(p, "fixture")
        with self.assertRaisesRegex(m.Invalid, "float"):
            m.canon({"x": 1.5})
        with self.assertRaisesRegex(m.Invalid, "unsafe integer"):
            m.canon({"x": 9007199254740992})

    def test_policy_commitment_mutation_is_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = make_intent(commit, blob); p = make_policy(i)
        p["preflight_receipt_commitment"] = "f" * 64
        with self.assertRaisesRegex(m.Invalid, "004C policy commitment mismatch"):
            m.verify(repo, p, i)

    def test_origin_normalization_supported_forms(self):
        expected = "Luminous-Dynamics/mycelix"
        for raw in (
            "https://github.com/Luminous-Dynamics/mycelix.git",
            "http://github.com/Luminous-Dynamics/mycelix.git",
            "git://github.com/Luminous-Dynamics/mycelix.git",
            "ssh://git@github.com/Luminous-Dynamics/mycelix.git",
            "git@github.com:Luminous-Dynamics/mycelix.git",
        ):
            with self.subTest(raw=raw):
                self.assertEqual(m.normalize_origin(raw), expected)
        self.assertIsNone(m.normalize_origin("https://example.com/Luminous-Dynamics/mycelix.git"))

    def test_failure_results_never_broaden_authority(self):
        for classification in ("REFUSED", "UNAVAILABLE", "INVALID"):
            out = m.failure(classification, "fixture")
            self.assertFalse(out["registration_authority"])
            self.assertFalse(out["workflow_dispatched"])
            self.assertFalse(out["workflow_identity_verified"])
            self.assertFalse(out["qualification_authority"])
            self.assertIsNone(out["qualification_result"])
            self.assertFalse(out["receipt_authenticity_verified"])

    def test_implementation_commitment_is_frozen_at_import(self):
        with tempfile.TemporaryDirectory() as d:
            dst = Path(d) / "workflow_identity.py"
            shutil.copy2(Path(m.__file__), dst)
            spec = importlib.util.spec_from_file_location("workflow_identity_copy", dst)
            assert spec and spec.loader
            mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
            before = mod.IMPLEMENTATION_COMMITMENT
            dst.write_text(dst.read_text() + "\n# drift after import\n")
            self.assertEqual(before, mod.IMPLEMENTATION_COMMITMENT)
            self.assertNotEqual(before, hashlib.sha256(mod.IMPL_DOMAIN + dst.read_bytes()).hexdigest())

    def test_static_surface_has_no_github_or_fetch_dispatch(self):
        source = Path(m.__file__).read_text().lower()
        for forbidden in ("import requests", "import urllib", "from github", "import github", "git fetch", "git push", "git checkout", "git reset", "update-ref", "gh workflow", "curl ", "wget "):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
