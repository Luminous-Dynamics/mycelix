#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import importlib.util
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

SPEC = importlib.util.spec_from_file_location(
    "policy", Path(__file__).with_name("registration_admission_policy.py")
)
assert SPEC and SPEC.loader
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

H40 = "a" * 40
H40B = "b" * 40
H64 = "a" * 64


class FakeAdmissionError(RuntimeError):
    pass


def intent() -> dict:
    return {
        "preflight_implementation_commitment": m.SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT,
        "environment_adapter_implementation_commitment": m.SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT,
        "qualification_workflow_path": ".github/workflows/amsap-004a-subject-topology.yml",
        "qualification_workflow_commit_sha": H40,
        "qualification_workflow_blob_sha1": H40B,
    }


def receipt(classification: str = "ELIGIBLE") -> dict:
    result = {
        "classification": classification,
        "qualification_result": None,
        "qualification_authority": False,
    }
    if classification == "ELIGIBLE":
        result.update({
            "preflight_implementation_commitment": m.SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT,
            "bound_preflight_implementation_commitment": m.SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT,
            "environment_adapter_implementation_commitment": m.SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT,
            "preflight_environment": {
                "adapter_implementation_commitment": m.SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT,
            },
        })
    return result


def core_result(classification: str = "ADMISSIBLE_TO_REQUEST") -> dict:
    result = {
        "schema": "mycelix-qualification-registration-admission-v1",
        "registration_admission_implementation_commitment": m.SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT,
        "classification": classification,
        "registration_authority": False,
        "workflow_dispatched": False,
        "workflow_identity_verified": False,
        "qualification_result": None,
        "qualification_authority": False,
    }
    if classification == "ADMISSIBLE_TO_REQUEST":
        result.update({
            "admission_commitment": H64,
            "qualification_workflow_path": intent()["qualification_workflow_path"],
            "qualification_workflow_commit_sha": H40,
            "qualification_workflow_blob_sha1": H40B,
        })
    return result


def fake_core(result: dict | None = None):
    return SimpleNamespace(
        AdmissionError=FakeAdmissionError,
        IMPLEMENTATION_COMMITMENT=m.SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT,
        evaluate=lambda _receipt, _intent: copy.deepcopy(result or core_result()),
    )


class T(unittest.TestCase):
    def test_core_file_verifier_accepts_exact_fixture_identity(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "core.py"
            p.write_bytes(b"core fixture\n")
            data = p.read_bytes()
            self.assertEqual(
                m._verify_core_file(
                    p,
                    expected_blob=m._git_blob_sha1(data),
                    expected_commitment=m._core_commitment(data),
                ),
                data,
            )

    def test_core_file_verifier_rejects_blob_or_commitment_drift(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "core.py"
            p.write_bytes(b"core fixture\n")
            with self.assertRaisesRegex(m.PolicyError, "Git blob mismatch"):
                m._verify_core_file(
                    p,
                    expected_blob="0" * 40,
                    expected_commitment=m._core_commitment(p.read_bytes()),
                )
            with self.assertRaisesRegex(m.PolicyError, "implementation mismatch"):
                m._verify_core_file(
                    p,
                    expected_blob=m._git_blob_sha1(p.read_bytes()),
                    expected_commitment="0" * 64,
                )

    def test_exact_supported_pair_is_admissible_with_zero_authority(self):
        out = m.evaluate_with_core(fake_core(), None, receipt(), intent())
        self.assertEqual(out["classification"], "ADMISSIBLE_TO_REQUEST")
        self.assertEqual(out["schema"], m.POLICY_SCHEMA)
        self.assertTrue(out["producer_revision_supported"])
        self.assertFalse(out["receipt_authenticity_verified"])
        for field in (
            "registration_authority",
            "workflow_dispatched",
            "workflow_identity_verified",
            "qualification_authority",
        ):
            self.assertFalse(out[field])
        self.assertIsNone(out["qualification_result"])
        self.assertEqual(
            out["registration_admission_core_implementation_commitment"],
            m.SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT,
        )
        self.assertRegex(out["registration_admission_policy_commitment"], r"^[0-9a-f]{64}$")

    def test_mutually_consistent_fake_producer_pair_is_rejected(self):
        r = receipt()
        i = intent()
        fake_pre = "c" * 64
        fake_env = "d" * 64
        r["preflight_implementation_commitment"] = fake_pre
        r["bound_preflight_implementation_commitment"] = fake_pre
        r["environment_adapter_implementation_commitment"] = fake_env
        r["preflight_environment"]["adapter_implementation_commitment"] = fake_env
        i["preflight_implementation_commitment"] = fake_pre
        i["environment_adapter_implementation_commitment"] = fake_env
        with self.assertRaisesRegex(m.PolicyError, "unsupported"):
            m.evaluate_with_core(fake_core(), None, r, i)

    def test_fake_producer_in_intent_or_receipt_is_rejected(self):
        i = intent()
        i["preflight_implementation_commitment"] = "c" * 64
        with self.assertRaises(m.PolicyError):
            m.evaluate_with_core(fake_core(), None, receipt(), i)
        r = receipt()
        r["environment_adapter_implementation_commitment"] = "d" * 64
        with self.assertRaises(m.PolicyError):
            m.evaluate_with_core(fake_core(), None, r, intent())

    def test_noneligible_receipt_stays_refused_and_non_authoritative(self):
        inner = core_result("REFUSED")
        out = m.evaluate_with_core(fake_core(inner), None, receipt("UNAVAILABLE"), intent())
        self.assertEqual(out["classification"], "REFUSED")
        self.assertFalse(out["receipt_authenticity_verified"])
        self.assertNotIn("registration_admission_policy_commitment", out)

    def test_core_authority_broadening_is_rejected(self):
        for field, value in (
            ("registration_authority", True),
            ("workflow_dispatched", True),
            ("workflow_identity_verified", True),
            ("qualification_authority", True),
            ("qualification_result", "PASS"),
        ):
            with self.subTest(field=field):
                inner = core_result()
                inner[field] = value
                with self.assertRaisesRegex(m.PolicyError, "broadened authority|qualification result"):
                    m.evaluate_with_core(fake_core(inner), None, receipt(), intent())

    def test_wrong_core_result_identity_or_classification_is_rejected(self):
        inner = core_result()
        inner["registration_admission_implementation_commitment"] = "0" * 64
        with self.assertRaisesRegex(m.PolicyError, "core result implementation"):
            m.evaluate_with_core(fake_core(inner), None, receipt(), intent())
        inner = core_result()
        inner["classification"] = "PASS"
        with self.assertRaisesRegex(m.PolicyError, "unsupported classification"):
            m.evaluate_with_core(fake_core(inner), None, receipt(), intent())

    def test_policy_commitment_binds_wrapper_core_and_supported_producers(self):
        out = m.evaluate_with_core(fake_core(), None, receipt(), intent())
        original = out["registration_admission_policy_commitment"]
        material = dict(out)
        material.pop("registration_admission_policy_commitment")
        self.assertEqual(original, m._policy_commitment(material))
        material["receipt_authenticity_verified"] = True
        self.assertNotEqual(original, m._policy_commitment(material))

    def test_invalid_shape_never_claims_authenticity_or_authority(self):
        out = m._invalid("bad")
        self.assertEqual(out["classification"], "INVALID")
        self.assertFalse(out["producer_revision_supported"])
        self.assertFalse(out["receipt_authenticity_verified"])
        self.assertFalse(out["registration_authority"])
        self.assertFalse(out["workflow_dispatched"])
        self.assertFalse(out["qualification_authority"])
        self.assertIsNone(out["qualification_result"])

    def test_core_input_errors_are_converted_to_policy_invalid_boundary(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            rp = root / "receipt.json"
            ip = root / "intent.json"
            rp.write_text("not-json")
            ip.write_text("{}")

            def bad_load(path: Path, label: str):
                if "receipt" in label:
                    raise FakeAdmissionError("preflight receipt is not valid JSON")
                return {}

            core = fake_core()
            core._load = bad_load
            with self.assertRaisesRegex(m.PolicyError, "preflight receipt is not valid JSON"):
                m._load_inputs(core, rp, ip)
            out = m._invalid("preflight receipt is not valid JSON")
            self.assertEqual(out["classification"], "INVALID")
            self.assertFalse(out["registration_authority"])
            self.assertFalse(out["receipt_authenticity_verified"])

    def test_unexpected_core_loader_error_is_not_reclassified(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            rp = root / "receipt.json"
            ip = root / "intent.json"
            rp.write_text("{}")
            ip.write_text("{}")
            core = fake_core()
            core._load = lambda _path, _label: (_ for _ in ()).throw(RuntimeError("unexpected"))
            with self.assertRaisesRegex(RuntimeError, "unexpected"):
                m._load_inputs(core, rp, ip)

    def test_wrapper_implementation_commitment_is_frozen_at_import(self):
        with tempfile.TemporaryDirectory() as d:
            src = Path(m.__file__)
            dst = Path(d) / "policy.py"
            shutil.copy2(src, dst)
            spec = importlib.util.spec_from_file_location("policy_copy", dst)
            assert spec and spec.loader
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            before = mod.IMPLEMENTATION_COMMITMENT
            dst.write_text(dst.read_text() + "\n# post-import drift\n")
            self.assertEqual(before, mod.IMPLEMENTATION_COMMITMENT)
            self.assertNotEqual(
                before,
                hashlib.sha256(mod.POLICY_IMPL_DOMAIN + dst.read_bytes()).hexdigest(),
            )

    def test_wrapper_has_no_network_subprocess_or_dispatch_client_import(self):
        source = Path(m.__file__).read_text()
        for forbidden in (
            "import subprocess",
            "import requests",
            "import urllib",
            "import github",
            "from github",
        ):
            self.assertNotIn(forbidden, source.lower())


if __name__ == "__main__":
    unittest.main()
