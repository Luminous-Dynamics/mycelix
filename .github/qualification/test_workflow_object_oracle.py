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
SPEC = importlib.util.spec_from_file_location("workflow_object_oracle", HERE / "workflow_object_oracle.py")
assert SPEC and SPEC.loader
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

H64 = "a" * 64
H64B = "b" * 64
WORKFLOW_PATH = ".github/workflows/amsap-004a-subject-topology.yml"


def git(repo: Path, *args: str) -> str:
    cp = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return cp.stdout.strip()


def git_bytes(repo: Path, *args: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(repo), *args])


def make_repo(*, executable: bool = False, symlink: bool = False, intermediate_file: bool = False, final_tree: bool = False):
    td = tempfile.TemporaryDirectory()
    repo = Path(td.name) / "repo"
    repo.mkdir()
    git(repo, "init", "-q")
    git(repo, "config", "user.email", "test@example.invalid")
    git(repo, "config", "user.name", "Test")
    git(repo, "remote", "add", "origin", "https://github.com/Luminous-Dynamics/mycelix.git")
    if intermediate_file:
        d = repo / ".github"
        d.mkdir()
        (d / "workflows").write_text("not a tree\n", encoding="utf-8")
        git(repo, "add", "-A")
        git(repo, "commit", "-q", "-m", "fixture")
        return td, repo, git(repo, "rev-parse", "HEAD"), None
    wf = repo / WORKFLOW_PATH
    wf.parent.mkdir(parents=True)
    if final_tree:
        wf.mkdir()
        (wf / "inside.txt").write_text("inside\n", encoding="utf-8")
    elif symlink:
        target = repo / "target.yml"
        target.write_text("name: target\n", encoding="utf-8")
        wf.symlink_to(Path("../../target.yml"))
    else:
        wf.write_text("name: exact\non: workflow_dispatch\n", encoding="utf-8")
        if executable:
            wf.chmod(0o755)
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "fixture")
    commit = git(repo, "rev-parse", "HEAD")
    blob = git(repo, "rev-parse", f"{commit}:{WORKFLOW_PATH}")
    return td, repo, commit, blob


def intent(commit: str, blob: str, path: str = WORKFLOW_PATH) -> dict:
    return {
        "schema": "mycelix-qualification-registration-intent-v1",
        "intent_id": "amsap-004a-r1-request-v1",
        "repository": "Luminous-Dynamics/mycelix",
        "subject_sha": "1" * 40,
        "predecessor_sha": "2" * 40,
        "preflight_profile_id": "amsap-004a-rustfmt-v1",
        "preflight_profile_commitment": H64,
        "preflight_implementation_commitment": "1038c07ed7529dff979edba34f44428a759ff6e7fb85a74b862ba75701ec421e",
        "environment_adapter_implementation_commitment": "89838000d0f673b669fbf00ad648a86c54638e2c637a73760db54d1cd8cae855",
        "preflight_environment_commitment": H64B,
        "qualification_workflow_path": path,
        "qualification_workflow_commit_sha": commit,
        "qualification_workflow_blob_sha1": blob,
        "registration_mode": "manual-request-v1",
    }


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def wfid(repo: Path, i: dict) -> dict:
    gp = Path(shutil.which("git") or "git").resolve(strict=True)
    version = subprocess.check_output([str(gp), "--version"], text=True).strip()
    status = subprocess.check_output([
        str(gp), "-C", str(repo), "status", "--porcelain=v1", "-z",
        "--untracked-files=all", "--ignored=matching",
    ])
    r = {
        "schema": m.WFID_SCHEMA,
        "workflow_identity_verifier_implementation_commitment": m.WFID_IMPL,
        "classification": "WORKFLOW_IDENTITY_VERIFIED",
        "registration_admission_policy_commitment": "3" * 64,
        "registration_intent_commitment": m.commitment(m.INTENT_DOMAIN, i),
        "repository": i["repository"],
        "subject_sha": i["subject_sha"],
        "predecessor_sha": i["predecessor_sha"],
        "qualification_workflow_path": i["qualification_workflow_path"],
        "qualification_workflow_commit_sha": i["qualification_workflow_commit_sha"],
        "qualification_workflow_blob_sha1": i["qualification_workflow_blob_sha1"],
        "git": {"path": str(gp), "sha256": sha256_file(gp), "version": version, "object_format": "sha1"},
        "tree_entry": {"mode": "100644", "type": "blob", "oid": i["qualification_workflow_blob_sha1"], "path": i["qualification_workflow_path"]},
        "caller_head": git(repo, "rev-parse", "HEAD"),
        "caller_status_sha256": hashlib.sha256(status).hexdigest(),
        "receipt_authenticity_verified": False,
        "registration_authority": False,
        "workflow_dispatched": False,
        "workflow_identity_verified": True,
        "qualification_result": None,
        "qualification_authority": False,
    }
    r["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, r)
    return r


def tree_record(mode: bytes, name: bytes, oid_hex: str) -> bytes:
    return mode + b" " + name + b"\0" + bytes.fromhex(oid_hex)


class T(unittest.TestCase):
    def test_exact_object_chain_confirms_and_keeps_nonclaims(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        out = m.confirm(repo, d, i)
        self.assertEqual(out["classification"], "WORKFLOW_OBJECT_CHAIN_CONFIRMED")
        self.assertTrue(out["workflow_identity_verified"])
        self.assertTrue(out["workflow_object_chain_confirmed"])
        self.assertFalse(out["git_implementation_trust_verified"])
        self.assertFalse(out["sha1_collision_resistance_claimed"])
        self.assertFalse(out["receipt_authenticity_verified"])
        self.assertFalse(out["registration_authority"])
        self.assertFalse(out["workflow_dispatched"])
        self.assertFalse(out["qualification_authority"])
        self.assertIsNone(out["qualification_result"])
        self.assertEqual(out["qualification_workflow_blob_sha1"], blob)
        self.assertRegex(out["commit_bytes_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(out["workflow_blob_bytes_sha256"], r"^[0-9a-f]{64}$")
        self.assertGreaterEqual(len(out["traversed_trees"]), 3)
        self.assertRegex(out["workflow_object_oracle_commitment"], r"^[0-9a-f]{64}$")

    def test_transport_rehash_rejects_wrong_commit_tree_and_blob_bytes(self):
        for object_type in ("commit", "tree", "blob"):
            with self.subTest(object_type=object_type):
                fake_oid = "1" * 40
                with patch.object(m, "run", return_value=b"malicious bytes"):
                    with self.assertRaisesRegex(m.Invalid, f"{object_type} transport bytes do not hash"):
                        m.obj("/bin/false", Path("."), {}, object_type, fake_oid)

    def test_parse_tree_duplicate_name_rejected(self):
        raw = tree_record(b"100644", b"x", "1" * 40) + tree_record(b"100644", b"x", "2" * 40)
        with self.assertRaisesRegex(m.Invalid, "duplicate tree entry name"):
            m.parse_tree(raw)

    def test_parse_tree_malformed_binary_rejected(self):
        cases = [b"100644", b"100644 x\0" + b"0" * 10, b"999999 x\0" + bytes.fromhex("1" * 40), b"100644 ../x\0" + bytes.fromhex("1" * 40)]
        for raw in cases:
            with self.subTest(raw=raw[:16]):
                with self.assertRaises(m.Invalid):
                    m.parse_tree(raw)

    def test_missing_literal_component_refused(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        missing = ".github/workflows/missing.yml"
        i = intent(commit, blob, missing); d = wfid(repo, i)
        with self.assertRaisesRegex(m.Refused, "workflow path component missing"):
            m.confirm(repo, d, i)

    def test_intermediate_component_must_be_tree(self):
        td, repo, commit, _ = make_repo(intermediate_file=True); self.addCleanup(td.cleanup)
        fake_blob = "1" * 40
        i = intent(commit, fake_blob); d = wfid(repo, i)
        with self.assertRaisesRegex(m.Refused, "intermediate component is not tree"):
            m.confirm(repo, d, i)

    def test_final_symlink_and_executable_are_refused(self):
        for kwargs in ({"symlink": True}, {"executable": True}):
            with self.subTest(kwargs=kwargs):
                td, repo, commit, blob = make_repo(**kwargs); self.addCleanup(td.cleanup)
                i = intent(commit, blob); d = wfid(repo, i)
                with self.assertRaisesRegex(m.Refused, "final component is not canonical 100644"):
                    m.confirm(repo, d, i)


    def test_final_tree_is_refused(self):
        td, repo, commit, tree_oid = make_repo(final_tree=True); self.addCleanup(td.cleanup)
        i = intent(commit, tree_oid); d = wfid(repo, i)
        with self.assertRaisesRegex(m.Refused, "final component is not canonical 100644"):
            m.confirm(repo, d, i)

    def test_independent_blob_mismatch_from_intent_refused(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, "f" * 40); d = wfid(repo, i)
        with self.assertRaisesRegex(m.Refused, "independent tree traversal blob differs from intent"):
            m.confirm(repo, d, i)

    def test_commit_root_tree_requires_first_tree_header(self):
        with self.assertRaisesRegex(m.Invalid, "does not begin with tree"):
            m.root_tree(b"parent " + b"1" * 40 + b"\n")
        with self.assertRaises(m.Invalid):
            m.root_tree(b"tree not-an-oid\n")

    def test_branch_move_does_not_change_exact_object_chain(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        first = m.confirm(repo, d, i)
        (repo / "README.md").write_text("move\n")
        git(repo, "add", "README.md"); git(repo, "commit", "-q", "-m", "move")
        # 004D result remains bound to the old caller HEAD, but 004D1's theorem is exact-object based.
        second = m.confirm(repo, d, i)
        self.assertEqual(first["commit_bytes_sha256"], second["commit_bytes_sha256"])
        self.assertEqual(first["workflow_blob_bytes_sha256"], second["workflow_blob_bytes_sha256"])
        self.assertEqual(first["traversed_trees"], second["traversed_trees"])

    def test_dirty_caller_checkout_preserved(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        wf = repo / WORKFLOW_PATH; wf.write_text(wf.read_text() + "# dirty\n")
        (repo / "untracked.txt").write_text("u\n")
        (repo / ".gitignore").write_text("ignored.txt\n")
        (repo / "ignored.txt").write_text("i\n")
        before_head = git(repo, "rev-parse", "HEAD")
        before_status = git_bytes(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all", "--ignored=matching")
        m.confirm(repo, d, i)
        self.assertEqual(before_head, git(repo, "rev-parse", "HEAD"))
        self.assertEqual(before_status, git_bytes(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all", "--ignored=matching"))

    def test_closed_environment_disables_lazy_fetch_and_replace_objects(self):
        gp = Path(shutil.which("git") or "/usr/bin/git").resolve()
        with tempfile.TemporaryDirectory() as td:
            e = m.env(Path(td) / "home", gp)
            self.assertEqual(e["GIT_NO_LAZY_FETCH"], "1")
            self.assertEqual(e["GIT_NO_REPLACE_OBJECTS"], "1")
            self.assertEqual(e["GIT_OPTIONAL_LOCKS"], "0")
            self.assertEqual(e["GIT_TERMINAL_PROMPT"], "0")
            self.assertEqual(e["GIT_CONFIG_NOSYSTEM"], "1")
            self.assertEqual(e["GIT_CONFIG_GLOBAL"], os.devnull)

    def test_exact_supported_004d_implementation_required(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        d["workflow_identity_verifier_implementation_commitment"] = "0" * 64
        material = dict(d); material.pop("workflow_identity_verification_commitment")
        d["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, material)
        with self.assertRaisesRegex(m.Invalid, "unsupported 004D implementation"):
            m.confirm(repo, d, i)

    def test_004d_verification_commitment_tampering_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        d["caller_status_sha256"] = "f" * 64
        with self.assertRaisesRegex(m.Invalid, "004D verification commitment mismatch"):
            m.confirm(repo, d, i)

    def test_004d_authority_or_pass_broadening_invalid(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob)
        for field, value in (("registration_authority", True), ("workflow_dispatched", True), ("qualification_authority", True), ("qualification_result", "PASS")):
            with self.subTest(field=field):
                d = wfid(repo, i); d[field] = value
                material = dict(d); material.pop("workflow_identity_verification_commitment")
                d["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, material)
                with self.assertRaisesRegex(m.Invalid, "broadened authority|attempted qualification"):
                    m.confirm(repo, d, i)

    def test_004d_nonverified_result_refused(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i); d["classification"] = "REFUSED"
        material = dict(d); material.pop("workflow_identity_verification_commitment")
        d["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, material)
        with self.assertRaisesRegex(m.Refused, "004D result is not verified"):
            m.confirm(repo, d, i)

    def test_intent_commitment_mismatch_refused(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        d["registration_intent_commitment"] = H64
        material = dict(d); material.pop("workflow_identity_verification_commitment")
        d["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, material)
        with self.assertRaisesRegex(m.Refused, "004D intent commitment mismatch"):
            m.confirm(repo, d, i)

    def test_git_executable_drift_pre_and_post_rejected(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        with patch.object(m, "git_same", return_value=False):
            with self.assertRaisesRegex(m.Invalid, "drift before object oracle"):
                m.confirm(repo, d, i)
        with patch.object(m, "git_same", side_effect=[True, False]):
            with self.assertRaisesRegex(m.Invalid, "drift after object oracle"):
                m.confirm(repo, d, i)

    def test_git_version_and_object_format_are_rechecked(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        d["git"]["version"] = "git version 0.0.0"
        material = dict(d); material.pop("workflow_identity_verification_commitment")
        d["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, material)
        with self.assertRaisesRegex(m.Invalid, "Git version drift"):
            m.confirm(repo, d, i)
        d = wfid(repo, i); d["git"]["object_format"] = "sha256"
        material = dict(d); material.pop("workflow_identity_verification_commitment")
        d["workflow_identity_verification_commitment"] = m.commitment(m.WFID_DOMAIN, material)
        with self.assertRaisesRegex(m.Unavailable, "unsupported Git object format"):
            m.confirm(repo, d, i)

    def test_result_commitment_binds_independent_byte_observations(self):
        td, repo, commit, blob = make_repo(); self.addCleanup(td.cleanup)
        i = intent(commit, blob); d = wfid(repo, i)
        out = m.confirm(repo, d, i)
        original = out["workflow_object_oracle_commitment"]
        changed = copy.deepcopy(out); changed.pop("workflow_object_oracle_commitment")
        changed["workflow_blob_bytes_sha256"] = "f" * 64
        self.assertNotEqual(original, m.commitment(m.ORACLE_DOMAIN, changed))

    def test_duplicate_json_keys_and_non_i_json_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "x.json"
            p.write_text('{"a":1,"a":2}')
            with self.assertRaisesRegex(m.Invalid, "duplicate JSON key"):
                m.load(p, "fixture")
            p.write_text('{"a":NaN}')
            with self.assertRaisesRegex(m.Invalid, "non-I-JSON"):
                m.load(p, "fixture")
        with self.assertRaisesRegex(m.Invalid, "float"):
            m.canonical({"x": 1.5})
        with self.assertRaisesRegex(m.Invalid, "unsafe integer"):
            m.canonical({"x": 9007199254740992})

    def test_failure_outputs_keep_all_authority_false(self):
        for cls in ("REFUSED", "UNAVAILABLE", "INVALID"):
            out = m.failure(cls, "fixture")
            self.assertFalse(out["sha1_collision_resistance_claimed"])
            self.assertFalse(out["git_implementation_trust_verified"])
            self.assertFalse(out["receipt_authenticity_verified"])
            self.assertFalse(out["registration_authority"])
            self.assertFalse(out["workflow_dispatched"])
            self.assertFalse(out["workflow_identity_verified"])
            self.assertFalse(out["workflow_object_chain_confirmed"])
            self.assertFalse(out["qualification_authority"])
            self.assertIsNone(out["qualification_result"])

    def test_implementation_commitment_is_frozen_at_import(self):
        with tempfile.TemporaryDirectory() as td:
            dst = Path(td) / "workflow_object_oracle.py"
            shutil.copy2(Path(m.__file__), dst)
            spec = importlib.util.spec_from_file_location("oracle_copy", dst)
            assert spec and spec.loader
            mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
            before = mod.IMPLEMENTATION_COMMITMENT
            dst.write_text(dst.read_text() + "\n# drift after import\n")
            self.assertEqual(before, mod.IMPLEMENTATION_COMMITMENT)
            self.assertNotEqual(before, hashlib.sha256(mod.IMPL_DOMAIN + dst.read_bytes()).hexdigest())

    def test_static_surface_has_no_network_or_mutating_git_commands(self):
        source = Path(m.__file__).read_text().lower()
        for forbidden in ("import requests", "import urllib", "from github", "import github", "git fetch", "git push", "git checkout", "git reset", "update-ref", "ls-tree", "gh workflow", "curl ", "wget "):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()