"""Execution tests for the synthetic-only SA-DID policy oracle."""
from __future__ import annotations

import copy
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
import sys

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import sa_did_policy_engine  # noqa: E402


class SyntheticPolicyEngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = json.loads(
            (IDENTITY_DIR / "SA_DID_PROFILE_V1.json").read_text(encoding="utf-8")
        )
        cls.vectors = cls.profile["test_vectors"]
        cls.valid = copy.deepcopy(next(
            vector for vector in cls.vectors
            if vector["scenario"] == "synthetic_valid_reference"
        ))

    def _evaluate_with(self, field: str, value: object) -> tuple[str, str]:
        vector = copy.deepcopy(self.valid)
        vector["policy_inputs"][field] = value
        return sa_did_policy_engine.evaluate_synthetic_vector(vector)

    def test_every_frozen_vector_executes_to_its_exact_declared_outcome(self) -> None:
        for vector in self.vectors:
            with self.subTest(vector=vector["id"], scenario=vector["scenario"]):
                self.assertEqual(
                    sa_did_policy_engine.evaluate_synthetic_vector(vector),
                    (vector["expected_disposition"], vector["expected_reason"]),
                )

    def test_real_or_non_synthetic_vector_is_refused(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["synthetic_fixture"] = False
        with self.assertRaisesRegex(ValueError, "only explicitly synthetic"):
            sa_did_policy_engine.evaluate_synthetic_vector(vector)

    def test_production_eligible_vector_is_refused(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["production_eligible"] = True
        with self.assertRaisesRegex(ValueError, "production-eligible"):
            sa_did_policy_engine.evaluate_synthetic_vector(vector)

    def test_policy_inputs_must_have_exactly_the_closed_field_set(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["policy_inputs"]["production_override"] = True
        with self.assertRaisesRegex(ValueError, "closed synthetic input schema"):
            sa_did_policy_engine.evaluate_synthetic_vector(vector)

    def test_missing_policy_input_is_refused(self) -> None:
        vector = copy.deepcopy(self.valid)
        del vector["policy_inputs"]["issuer_trusted"]
        with self.assertRaisesRegex(ValueError, "closed synthetic input schema"):
            sa_did_policy_engine.evaluate_synthetic_vector(vector)

    def test_truthy_string_cannot_substitute_for_boolean_evidence(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be a Boolean"):
            self._evaluate_with("issuer_trusted", "true")

    def test_trusted_issuer_requires_explicit_test_only_anchor(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["trust_anchor_ref"] = "dha-production"
        with self.assertRaisesRegex(ValueError, "test-only reference"):
            sa_did_policy_engine.evaluate_synthetic_vector(vector)

    def test_issuer_trust_failure_precedes_other_positive_signals(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["policy_inputs"]["issuer_trusted"] = False
        vector["policy_inputs"]["proof_valid"] = False
        self.assertEqual(
            sa_did_policy_engine.evaluate_synthetic_vector(vector),
            ("rejected", "issuer_not_trusted"),
        )

    def test_invalid_proof_is_rejected(self) -> None:
        self.assertEqual(self._evaluate_with("proof_valid", False), ("rejected", "invalid_proof"))

    def test_unavailable_source_is_indeterminate(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["policy_inputs"]["source_state"] = "unavailable"
        self.assertEqual(
            sa_did_policy_engine.evaluate_synthetic_vector(vector),
            ("indeterminate", "source_unavailable"),
        )

    def test_unsupported_profile_is_indeterminate(self) -> None:
        self.assertEqual(
            self._evaluate_with("profile_supported", False),
            ("indeterminate", "unsupported_profile"),
        )

    def test_unknown_and_stale_status_never_verify(self) -> None:
        for status, reason in (("unknown", "status_unknown"), ("stale", "status_stale")):
            with self.subTest(status=status):
                self.assertEqual(
                    self._evaluate_with("credential_status", status),
                    ("indeterminate", reason),
                )

    def test_revoked_credential_is_rejected(self) -> None:
        self.assertEqual(
            self._evaluate_with("credential_status", "revoked"),
            ("rejected", "credential_revoked"),
        )

    def test_missing_time_evidence_is_indeterminate(self) -> None:
        self.assertEqual(
            self._evaluate_with("time_state", "unknown"),
            ("indeterminate", "credential_time_unknown"),
        )

    def test_alternative_route_takes_precedence_over_unavailable_source(self) -> None:
        vector = copy.deepcopy(self.valid)
        vector["policy_inputs"]["alternative_route_required"] = True
        vector["policy_inputs"]["source_state"] = "unavailable"
        self.assertEqual(
            sa_did_policy_engine.evaluate_synthetic_vector(vector),
            ("indeterminate", "alternative_route_required"),
        )

    def test_sensitive_receipt_and_authority_scope_violations_reject(self) -> None:
        self.assertEqual(
            self._evaluate_with("privacy_safe", False),
            ("rejected", "privacy_boundary_violation"),
        )
        self.assertEqual(
            self._evaluate_with("authorization_scope_valid", False),
            ("rejected", "authorization_scope_violation"),
        )

    def test_wrong_audience_nonce_replay_and_holder_mismatch_reject(self) -> None:
        cases = (
            ("audience_matches", False, "wrong_audience"),
            ("nonce_fresh", False, "nonce_replay"),
            ("holder_binding_valid", False, "holder_binding_failed"),
        )
        for field, value, reason in cases:
            with self.subTest(field=field):
                self.assertEqual(self._evaluate_with(field, value), ("rejected", reason))


    def test_unknown_validity_time_has_distinct_reason_from_unknown_status(self) -> None:
        self.assertEqual(
            self._evaluate_with("time_state", "unknown"),
            ("indeterminate", "credential_time_unknown"),
        )


    def test_cli_emits_valid_machine_readable_evidence_report(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            report_path = Path(temporary_dir) / "sa-did-policy-report.json"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(IDENTITY_DIR / "sa_did_policy_engine.py"),
                    "--report",
                    str(report_path),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("SA_DID_POLICY_ORACLE=PASS vectors=16 matched=16", completed.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["vectors_total"], 16)
            self.assertEqual(report["vectors_matched"], 16)
            self.assertTrue(report["passed"])
            self.assertFalse(report["cryptographic_verification_performed"])
            self.assertFalse(report["dha_endpoint_contacted"])


if __name__ == "__main__":
    unittest.main()
