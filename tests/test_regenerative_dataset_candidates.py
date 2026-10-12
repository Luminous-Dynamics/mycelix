"""Regression tests for fail-closed dataset discovery and license gating."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from validate_regenerative_dataset_candidates import validate_manifest  # noqa: E402

MANIFEST = ROOT / "docs/economy-atlas/regenerative-dataset-candidates.v1.json"
INDICATORS = ROOT / "docs/economy-atlas/regenerative-indicators.v1.json"


class RegenerativeDatasetCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.fixture = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.indicator_catalog = json.loads(INDICATORS.read_text(encoding="utf-8"))

    def setUp(self) -> None:
        self.manifest = copy.deepcopy(self.fixture)

    def test_current_candidates_are_structurally_valid_and_blocked(self) -> None:
        self.assertEqual(validate_manifest(self.manifest, self.indicator_catalog), [])
        for item in self.manifest["candidates"]:
            self.assertEqual(item["ingestion_gate"]["status"], "blocked")
            self.assertFalse(item["ingestion_gate"]["ingestion_authorized"])
            self.assertFalse(item["ingestion_gate"]["values_imported"])

    def test_current_candidates_cover_license_states(self) -> None:
        states = {item["discovery_status"] for item in self.manifest["candidates"]}
        self.assertEqual(states, {
            "license_not_verified",
            "license_metadata_conflict",
            "restricted_noncommercial_use",
        })

    def test_duplicate_candidate_id_fails(self) -> None:
        self.manifest["candidates"].append(copy.deepcopy(self.manifest["candidates"][0]))
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("duplicate candidate_id" in error for error in errors), errors)

    def test_ingestion_authorization_cannot_be_enabled_in_discovery_catalog(self) -> None:
        self.manifest["candidates"][0]["ingestion_gate"]["ingestion_authorized"] = True
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("ingestion_authorized must be false" in error for error in errors), errors)

    def test_global_qualification_cannot_be_enabled(self) -> None:
        self.manifest["qualification"]["real_observations_imported"] = True
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("all qualification flags must remain false" in error for error in errors), errors)

    def test_unverified_license_cannot_be_marked_verified(self) -> None:
        self.manifest["candidates"][0]["license_assessment"]["license_status"] = "verified"
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("discovery_status conflicts with license_assessment" in error for error in errors), errors)

    def test_unknown_indicator_fails(self) -> None:
        self.manifest["candidates"][0]["related_indicator_ids"] = ["not.registered"]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("unknown indicator_id" in error for error in errors), errors)

    def test_non_https_candidate_url_fails(self) -> None:
        self.manifest["candidates"][0]["dataset_url"] = "http://example.invalid/dataset"
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("dataset_url must be HTTPS" in error for error in errors), errors)

    def test_real_calendar_dates_required(self) -> None:
        self.manifest["candidates"][0]["temporal_coverage"]["start_date"] = "2026-02-31"
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("start_date must be a real" in error for error in errors), errors)

    def test_conflicting_license_status_is_required(self) -> None:
        self.manifest["candidates"][1]["license_assessment"]["license_status"] = "unverified"
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("discovery_status conflicts" in error for error in errors), errors)

    def test_missing_rationale_fails(self) -> None:
        self.manifest["candidates"][2]["ingestion_gate"]["required_evidence"] = []
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("required_evidence must be a non-empty" in error for error in errors), errors)

    def test_date_window_discrepancy_is_preserved_as_a_finding(self) -> None:
        swam = self.manifest["candidates"][0]
        findings = {finding["finding_id"]: finding for finding in swam["known_findings"]}
        finding = findings["SWAM-PREVIEW-DATE-WINDOW-001"]
        self.assertIn("2025-08-30", finding["observation"])
        self.assertIn("2025-11-07", finding["observation"])
        self.assertEqual(validate_manifest(self.manifest, self.indicator_catalog), [])

    def test_duplicate_finding_id_fails(self) -> None:
        findings = self.manifest["candidates"][0]["known_findings"]
        findings.append(copy.deepcopy(findings[0]))
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("duplicate finding_id" in error for error in errors), errors)

    def test_malformed_finding_fails_closed(self) -> None:
        self.manifest["candidates"][0]["known_findings"] = [None]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("known_findings[0] must be an object" in error for error in errors), errors)

    def test_non_object_root_fails_closed(self) -> None:
        self.assertEqual(
            validate_manifest([], self.indicator_catalog),
            ["dataset candidate catalog root must be an object"],
        )

    def test_unhashable_related_indicator_fails_closed(self) -> None:
        self.manifest["candidates"][0]["related_indicator_ids"] = [{"not": "an ID"}]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("unknown or malformed indicator_id" in error for error in errors), errors)

    def test_unhashable_reason_code_does_not_crash(self) -> None:
        self.manifest["candidates"][0]["ingestion_gate"]["reason_codes"] = [{"not": "a string"}]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("reason_codes must be a non-empty string list" in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main()
