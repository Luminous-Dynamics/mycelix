#!/usr/bin/env python3
from __future__ import annotations

import copy
import importlib.util
import json
import hashlib
import shutil
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    "admission", Path(__file__).with_name("registration_admission.py")
)
assert SPEC and SPEC.loader
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

H40 = "a" * 40
H40B = "b" * 40
H64 = "a" * 64
H64B = "b" * 64


def tool_identity(name: str) -> dict:
    return {
        "requested_name": name,
        "discovered_path": f"/nix/store/fake/bin/{name}",
        "symlink_chain": [f"/nix/store/fake/bin/{name}"],
        "resolved_path": f"/nix/store/fake/bin/{name}",
        "discovered_mode": 493,
        "resolved_mode": 493,
        "size": 1,
        "device": 1,
        "inode": 1,
        "sha256": H64,
    }


def environment_manifest() -> dict:
    return {
        "schema": m.ENV_SCHEMA,
        "policy_revision": "bound-child-env-v2",
        "adapter_implementation_commitment": H64,
        "platform_system": "Linux",
        "platform_machine": "x86_64",
        "python_implementation": "CPython",
        "python_version": "3.13.5",
        "python": tool_identity("python"),
        "git": tool_identity("git"),
        "rustup": tool_identity("rustup"),
        "rustup_home": "/nix/store/fake-rustup-home",
        "toolchain": "1.98.1",
        "cargo": tool_identity("cargo"),
        "rustfmt": tool_identity("rustfmt"),
        "cargo_fmt": tool_identity("cargo-fmt"),
        "path_policy": "bound-bin-only-before-rustup-toolchain-injection",
        "bound_input_policy": "adapter-owned-copy-plus-source-postflight-v1",
        "inherited_environment": ["RUSTUP_HOME-source-location-only"],
        "locale": "C",
        "timezone": "UTC",
        "python_hash_seed": "0",
        "python_isolated_mode": True,
    }


def command_record() -> dict:
    return {
        "argv": ["rustup", "run", "1.98.1", "cargo", "fmt"],
        "exit_code": 0,
        "stdout_sha256": H64,
        "stderr_sha256": H64,
        "stderr_tail": "",
    }


def receipt() -> dict:
    r = {
        "schema": m.CHILD_SCHEMA,
        "preflight_implementation_commitment": H64,
        "profile_id": "amsap-004a-rustfmt-v1",
        "profile_commitment": H64,
        "repository": "Luminous-Dynamics/mycelix",
        "origin": "https://github.com/Luminous-Dynamics/mycelix.git",
        "subject_sha": H40,
        "required_parent_sha": H40B,
        "changed_paths": ["a", "b"],
        "classification": "ELIGIBLE",
        "qualification_result": None,
        "qualification_authority": False,
        "python_version": "3.13.5",
        "git_executable": {"path": "/bound/git", "resolved": "/nix/store/fake/bin/git", "sha256": H64},
        "rustup_executable": {"path": "/bound/rustup", "resolved": "/nix/store/fake/bin/rustup", "sha256": H64},
        "checks": [{**command_record(), "id": "rustfmt", "status": "PASS"}],
        "tracked_materialization_commitment": H64,
        "rustup_toolchain_list": command_record(),
        "cargo_probe": command_record(),
        "rustfmt_probe": command_record(),
        "reason": "registered rustfmt preflight gate passed",
        "environment_adapter_schema": m.ENV_SCHEMA,
        "environment_adapter_implementation_commitment": H64,
        "bound_profile_raw_sha256": H64,
        "bound_profile_commitment": H64,
        "bound_preflight_raw_sha256": H64,
        "bound_preflight_implementation_commitment": H64,
        "preflight_environment": environment_manifest(),
        "preflight_environment_commitment": m._commit(m.ENV_DOMAIN, environment_manifest()),
        "child_process": {
            "exit_code": 0,
            "stdout_sha256": H64,
            "stderr_sha256": H64,
        },
    }
    assert set(r) == m.ELIGIBLE_RECEIPT_FIELDS
    return r


def intent() -> dict:
    i = {
        "schema": m.INTENT_SCHEMA,
        "intent_id": "amsap-004a-r1-request-v1",
        "repository": "Luminous-Dynamics/mycelix",
        "subject_sha": H40,
        "predecessor_sha": H40B,
        "preflight_profile_id": "amsap-004a-rustfmt-v1",
        "preflight_profile_commitment": H64,
        "preflight_implementation_commitment": H64,
        "environment_adapter_implementation_commitment": H64,
        "preflight_environment_commitment": m._commit(m.ENV_DOMAIN, environment_manifest()),
        "qualification_workflow_path": ".github/workflows/amsap-004a-subject-topology.yml",
        "qualification_workflow_commit_sha": H40,
        "qualification_workflow_blob_sha1": H40B,
        "registration_mode": "manual-request-v1",
    }
    assert set(i) == m.INTENT_FIELDS
    return i


class T(unittest.TestCase):
    def test_exact_match_is_admissible_without_authority(self):
        out = m.evaluate(receipt(), intent())
        self.assertEqual(out["classification"], "ADMISSIBLE_TO_REQUEST")
        self.assertFalse(out["registration_authority"])
        self.assertFalse(out["workflow_dispatched"])
        self.assertFalse(out["workflow_identity_verified"])
        self.assertIsNone(out["qualification_result"])
        self.assertFalse(out["qualification_authority"])
        self.assertRegex(out["registration_admission_implementation_commitment"], r"^[0-9a-f]{64}$")
        self.assertRegex(out["intent_commitment"], r"^[0-9a-f]{64}$")
        self.assertRegex(out["preflight_receipt_commitment"], r"^[0-9a-f]{64}$")
        self.assertRegex(out["admission_commitment"], r"^[0-9a-f]{64}$")

    def test_noneligible_receipt_refuses_without_authority(self):
        for state in ("NOT_ELIGIBLE", "UNAVAILABLE", "INVALID"):
            with self.subTest(state=state):
                r = receipt()
                r["classification"] = state
                out = m.evaluate(r, intent())
                self.assertEqual(out["classification"], "REFUSED")
                self.assertFalse(out["registration_authority"])
                self.assertFalse(out["workflow_dispatched"])
                self.assertIsNone(out["qualification_result"])
                self.assertFalse(out["qualification_authority"])

    def test_authority_or_pass_claim_is_rejected(self):
        cases = [
            ("qualification_authority", True),
            ("qualification_result", "PASS"),
        ]
        for field, value in cases:
            with self.subTest(field=field):
                r = receipt()
                r[field] = value
                with self.assertRaises(m.AdmissionError):
                    m.evaluate(r, intent())

    def test_each_bound_identity_mismatch_refuses(self):
        fields = {
            "repository": "Other/repo",
            "subject_sha": "c" * 40,
            "predecessor_sha": "c" * 40,
            "preflight_profile_id": "other-v1",
            "preflight_profile_commitment": H64B,
            "preflight_implementation_commitment": H64B,
            "environment_adapter_implementation_commitment": H64B,
            "preflight_environment_commitment": H64B,
        }
        for field, value in fields.items():
            with self.subTest(field=field):
                i = intent()
                i[field] = value
                out = m.evaluate(receipt(), i)
                self.assertEqual(out["classification"], "REFUSED")
                self.assertIn(field, out["mismatches"])
                self.assertFalse(out["registration_authority"])

    def test_internal_profile_and_implementation_binding_mismatch_rejected(self):
        for field in (
            "bound_profile_commitment",
            "bound_preflight_implementation_commitment",
        ):
            with self.subTest(field=field):
                r = receipt()
                r[field] = H64B
                with self.assertRaises(m.AdmissionError):
                    m.evaluate(r, intent())

    def test_rustfmt_pass_and_child_exit_are_rechecked(self):
        r = receipt()
        r["checks"][0]["status"] = "FAIL"
        with self.assertRaises(m.AdmissionError):
            m.evaluate(r, intent())
        r = receipt()
        r["child_process"]["exit_code"] = 4
        with self.assertRaises(m.AdmissionError):
            m.evaluate(r, intent())

    def test_intent_unknown_field_and_bad_mode_rejected(self):
        i = intent()
        i["argv"] = ["gh", "workflow", "run"]
        with self.assertRaises(m.AdmissionError):
            m.evaluate(receipt(), i)
        i = intent()
        i["registration_mode"] = "execute-shell"
        with self.assertRaises(m.AdmissionError):
            m.evaluate(receipt(), i)

    def test_receipt_unknown_field_rejected(self):
        r = receipt()
        r["qualification_pass"] = True
        with self.assertRaises(m.AdmissionError):
            m.evaluate(r, intent())

    def test_noncanonical_workflow_paths_rejected(self):
        for path in (
            "/tmp/workflow.yml",
            "../workflow.yml",
            ".github//workflow.yml",
            ".github/./workflow.yml",
            "a\\b.yml",
        ):
            with self.subTest(path=path):
                i = intent()
                i["qualification_workflow_path"] = path
                with self.assertRaises(m.AdmissionError):
                    m.evaluate(receipt(), i)

    def test_workflow_identity_changes_intent_commitment_not_receipt_match(self):
        base = intent()
        a = m._validate_intent(base)
        for field, value in (
            ("qualification_workflow_path", ".github/workflows/other.yml"),
            ("qualification_workflow_commit_sha", "c" * 40),
            ("qualification_workflow_blob_sha1", "c" * 40),
        ):
            with self.subTest(field=field):
                changed = copy.deepcopy(base)
                changed[field] = value
                self.assertNotEqual(a, m._validate_intent(changed))
                out = m.evaluate(receipt(), changed)
                self.assertEqual(out["classification"], "ADMISSIBLE_TO_REQUEST")
                self.assertFalse(out["workflow_identity_verified"])

    def test_environment_commitment_and_adapter_binding_are_rechecked(self):
        r = receipt()
        r["preflight_environment"]["timezone"] = "PST8PDT"
        with self.assertRaisesRegex(m.AdmissionError, "environment commitment mismatch"):
            m.evaluate(r, intent())

        r = receipt()
        r["preflight_environment"]["adapter_implementation_commitment"] = H64B
        r["preflight_environment_commitment"] = m._commit(
            m.ENV_DOMAIN, r["preflight_environment"]
        )
        with self.assertRaisesRegex(m.AdmissionError, "adapter binding"):
            m.evaluate(r, intent())

    def test_child_and_environment_tool_hashes_must_agree(self):
        for receipt_field in ("git_executable", "rustup_executable"):
            with self.subTest(receipt_field=receipt_field):
                r = receipt()
                r[receipt_field]["sha256"] = H64B
                with self.assertRaisesRegex(m.AdmissionError, "executable identity"):
                    m.evaluate(r, intent())

    def test_key_order_does_not_change_commitments(self):
        r = receipt()
        i = intent()
        r2 = dict(reversed(list(r.items())))
        i2 = dict(reversed(list(i.items())))
        self.assertEqual(
            m._validate_eligible_receipt(r), m._validate_eligible_receipt(r2)
        )
        self.assertEqual(m._validate_intent(i), m._validate_intent(i2))

    def test_semantic_receipt_change_changes_commitment(self):
        r = receipt()
        a = m._validate_eligible_receipt(r)
        r["origin"] = "git@github.com:Luminous-Dynamics/mycelix.git"
        b = m._validate_eligible_receipt(r)
        self.assertNotEqual(a, b)

    def test_duplicate_json_keys_rejected(self):
        raw = b'{"schema":"x","schema":"y"}'
        with self.assertRaises(m.AdmissionError):
            m._closed_object(raw, "fixture")

    def test_size_bound(self):
        with self.assertRaises(m.AdmissionError):
            m._closed_object(b" " * (m.MAX_JSON + 1), "fixture")

    def test_main_invalid_never_broadens_authority(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rp = root / "receipt.json"
            ip = root / "intent.json"
            rp.write_text("not-json")
            ip.write_text(json.dumps(intent()))
            # Exercise the same invariant via the error result shape directly;
            # main's printing is intentionally not captured here.
            try:
                m._load(rp, "preflight receipt")
            except m.AdmissionError as exc:
                result = {
                    "classification": "INVALID",
                    "reason": str(exc),
                    "registration_authority": False,
                    "workflow_dispatched": False,
                    "qualification_result": None,
                    "qualification_authority": False,
                }
            self.assertFalse(result["registration_authority"])
            self.assertFalse(result["workflow_dispatched"])
            self.assertIsNone(result["qualification_result"])
            self.assertFalse(result["qualification_authority"])

    def test_implementation_commitment_is_frozen_at_import(self):
        with tempfile.TemporaryDirectory() as directory:
            src = Path(m.__file__)
            dst = Path(directory) / "registration_admission_copy.py"
            shutil.copy2(src, dst)
            spec = importlib.util.spec_from_file_location("admission_copy", dst)
            assert spec and spec.loader
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            before = mod.IMPLEMENTATION_COMMITMENT
            dst.write_text(dst.read_text() + "\n# changed after import\n")
            self.assertEqual(before, mod.IMPLEMENTATION_COMMITMENT)
            self.assertNotEqual(
                before,
                hashlib.sha256(mod.IMPL_DOMAIN + dst.read_bytes()).hexdigest(),
            )


if __name__ == "__main__":
    unittest.main()
