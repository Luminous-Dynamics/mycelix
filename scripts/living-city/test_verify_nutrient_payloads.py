#!/usr/bin/env python3
"""Adversarial tests for offline nutrient payload integrity and envelope binding."""
from __future__ import annotations
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from verify_nutrient_payloads import verify_receipt_payload, verify_review_payloads


class NutrientPayloadVerifierTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.payload_path = self.root / "fixtures/living-city/payloads/native-assessment.json"
        self.payload_path.parent.mkdir(parents=True)
        self.native = {
            "scenario_id": "scenario-001", "target_id": "target-vitamin-c",
            "nutrient_id": "vitamin-c", "nutrient_definition_ref": "nutrient-def-v1",
            "unit": "milligrams", "target_scope_ref": "beneficiary-cohort-001",
            "period_ref": "period-2026", "value_scale": "1000",
            "total_edible_delivered_grams": "180000", "supply_numerator": "100000000",
            "target_numerator": "50000000", "denominator": "100000",
            "shortfall_numerator": "0", "composition_source_refs": ["composition-source-v1"],
            "delivery_source_refs": ["delivery-batch-77"], "target_source_ref": "target-source-v1",
            "status": "meets_declared_minimum", "line_receipts": [], "blockers": [],
        }
        self.payload_bytes = json.dumps(self.native, separators=(",", ":")).encode("utf-8")
        self.payload_path.write_bytes(self.payload_bytes)
        fields = (
            "scenario_id", "target_id", "nutrient_id", "nutrient_definition_ref", "unit",
            "target_scope_ref", "period_ref", "value_scale", "total_edible_delivered_grams",
            "supply_numerator", "target_numerator", "denominator", "shortfall_numerator",
            "composition_source_refs", "delivery_source_refs", "target_source_ref", "status",
        )
        self.receipt = {
            "receipt_id": "receipt-001", "receipt_origin": "symtropy_output",
            "payload_ref": "repo-file:fixtures/living-city/payloads/native-assessment.json",
            "payload_sha256": hashlib.sha256(self.payload_bytes).hexdigest(),
            **{key: self.native[key] for key in fields},
        }

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_exact_bytes_and_all_envelope_fields_verify(self) -> None:
        result = verify_receipt_payload(self.receipt, self.root)
        self.assertEqual(result["status"], "verified")
        self.assertEqual(len(result["binding_fields_checked"]), 17)

    def test_changed_bytes_fail_even_when_json_would_still_parse(self) -> None:
        self.payload_path.write_bytes(self.payload_bytes + b"\n")
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "hash_mismatch")

    def test_rehashed_payload_with_different_scope_fails_envelope_binding(self) -> None:
        altered = dict(self.native)
        altered["target_scope_ref"] = "different-cohort"
        raw = json.dumps(altered, separators=(",", ":")).encode("utf-8")
        self.payload_path.write_bytes(raw)
        self.receipt["payload_sha256"] = hashlib.sha256(raw).hexdigest()
        result = verify_receipt_payload(self.receipt, self.root)
        self.assertEqual(result["status"], "payload_binding_mismatch")
        self.assertEqual(result["mismatched_fields"], ["target_scope_ref"])

    def test_missing_file_is_not_verified(self) -> None:
        self.payload_path.unlink()
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "missing_payload")

    def test_parent_traversal_is_rejected(self) -> None:
        self.receipt["payload_ref"] = "repo-file:../native-assessment.json"
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "invalid_reference")

    def test_absolute_path_is_rejected(self) -> None:
        self.receipt["payload_ref"] = "repo-file:/etc/passwd"
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "invalid_reference")

    def test_symlink_escape_is_rejected(self) -> None:
        outside = self.root.parent / (self.root.name + "-outside.json")
        outside.write_bytes(self.payload_bytes)
        link = self.root / "fixtures/living-city/payloads/escape.json"
        try:
            link.symlink_to(outside)
        except OSError as exc:
            outside.unlink(missing_ok=True)
            self.skipTest(f"symlink creation unavailable: {exc}")
        try:
            self.receipt["payload_ref"] = "repo-file:fixtures/living-city/payloads/escape.json"
            self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "invalid_reference")
        finally:
            outside.unlink(missing_ok=True)

    def test_remote_locator_is_explicitly_unverified_without_network(self) -> None:
        self.receipt["payload_ref"] = "https://example.invalid/nutrient.json"
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "external_unverified")

    def test_malformed_digest_is_rejected(self) -> None:
        self.receipt["payload_sha256"] = "not-a-hash"
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "invalid_commitment")

    def test_payload_over_size_cap_is_rejected(self) -> None:
        with self.payload_path.open("wb") as stream:
            stream.truncate(16 * 1024 * 1024 + 1)
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "payload_too_large")

    def test_duplicate_keys_in_native_payload_are_rejected(self) -> None:
        raw = b'{"scenario_id":"first","scenario_id":"second"}'
        self.payload_path.write_bytes(raw)
        self.receipt["payload_sha256"] = hashlib.sha256(raw).hexdigest()
        self.assertEqual(verify_receipt_payload(self.receipt, self.root)["status"], "invalid_payload_json")

    def test_review_with_only_synthetic_receipts_is_not_a_pass(self) -> None:
        report = verify_review_payloads(
            {"nutrient_assessments": [{"receipt_id": "synthetic", "receipt_origin": "synthetic_test_only"}]},
            self.root,
        )
        self.assertEqual(report["status"], "no_real_receipts")
        self.assertFalse(report["ok"])

    def test_duplicate_real_receipt_ids_block_report(self) -> None:
        report = verify_review_payloads(
            {"nutrient_assessments": [dict(self.receipt), dict(self.receipt)]}, self.root
        )
        self.assertEqual(report["status"], "blocked")
        self.assertTrue(all(r["status"] == "duplicate_receipt_id" for r in report["records"]))

    def test_non_object_receipt_entry_fails_closed(self) -> None:
        report = verify_review_payloads({"nutrient_assessments": [None]}, self.root)
        self.assertEqual(report["status"], "blocked")
        self.assertEqual(report["records"][0]["status"], "invalid_receipt")

    def test_unknown_receipt_origin_fails_closed(self) -> None:
        report = verify_review_payloads(
            {"nutrient_assessments": [{"receipt_id": "odd", "receipt_origin": "asserted_valid"}]},
            self.root,
        )
        self.assertEqual(report["status"], "blocked")
        self.assertEqual(report["records"][0]["status"], "invalid_receipt")

    def test_missing_receipt_id_fails_closed(self) -> None:
        receipt = dict(self.receipt)
        receipt.pop("receipt_id")
        report = verify_review_payloads({"nutrient_assessments": [receipt]}, self.root)
        self.assertEqual(report["status"], "blocked")
        self.assertEqual(report["records"][0]["status"], "invalid_receipt")


if __name__ == "__main__":
    unittest.main(verbosity=2)
