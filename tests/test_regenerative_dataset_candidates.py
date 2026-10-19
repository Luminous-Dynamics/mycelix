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
            "license_declared_access_private",
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

    def test_malformed_discovery_status_fails_closed_without_crashing(self) -> None:
        self.manifest["candidates"][0]["discovery_status"] = ["license_not_verified"]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any(".discovery_status is invalid or missing" in error for error in errors), errors)

    def test_jukskei_candidate_is_gauteng_but_blocked_on_private_access(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.jukskei-water-quality-study.2021.v1")
        self.assertIn("Johannesburg", candidate["spatial_coverage"]["geography"])
        self.assertEqual(candidate["license_assessment"]["license_status"], "unverified")
        self.assertEqual(candidate["ingestion_gate"]["status"], "blocked")
        self.assertFalse(candidate["ingestion_gate"]["ingestion_authorized"])
        self.assertIn("JUKSKEI-SIX-SAMPLE-WINDOW-001", {x["finding_id"] for x in candidate["known_findings"]})

    def test_2025_asm_jukskei_supplement_stays_rights_blocked(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.jukskei-river-microbial-data.2025.v1")
        self.assertEqual(candidate["reported_data_shape"]["study_sample_count"], 60)
        self.assertEqual(candidate["reported_data_shape"]["site_count"], 5)
        self.assertIsNone(candidate["temporal_coverage"]["start_date"])
        self.assertIsNone(candidate["temporal_coverage"]["end_date"])
        self.assertFalse(candidate["ingestion_gate"]["ingestion_authorized"])
        self.assertIn("ASM-JUKSKEI-SUPPLEMENT-REUSE-RIGHTS-001", {f["finding_id"] for f in candidate["known_findings"]})

    def test_2025_asm_jukskei_coordinates_preserve_source_and_normalize_decimal_degrees(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.jukskei-river-microbial-data.2025.v1")
        sites = {site["site"]: site for site in candidate["spatial_coverage"]["entities"]}
        self.assertEqual(set(sites), {"A", "B", "C", "D", "E"})
        self.assertEqual(sites["A"]["coordinates_as_published"], "26,18147°S, 28,10237°E")
        self.assertEqual(sites["A"]["coordinates_decimal_degrees"]["latitude"], -26.18147)
        self.assertEqual(sites["A"]["coordinates_decimal_degrees"]["longitude"], 28.10237)
        self.assertIn("coordinates_as_published", sites["A"]["coordinates_decimal_degrees"]["conversion_note"])
        self.assertEqual(validate_manifest(self.manifest, self.indicator_catalog), [])

    def test_2025_asm_jukskei_rejects_coordinate_transcription_drift(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.jukskei-river-microbial-data.2025.v1")
        site_a = next(site for site in candidate["spatial_coverage"]["entities"] if site["site"] == "A")
        site_a["coordinates_decimal_degrees"]["longitude"] = 28.10273
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("source-pinned decimal conversion" in error for error in errors), errors)

    def test_2025_asm_jukskei_rejects_overflowing_coordinate_without_crashing(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.jukskei-river-microbial-data.2025.v1")
        site_a = next(site for site in candidate["spatial_coverage"]["entities"] if site["site"] == "A")
        site_a["coordinates_decimal_degrees"]["longitude"] = 10**1000
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("coordinates_decimal_degrees.longitude must be finite" in error for error in errors), errors)

    def test_2025_asm_jukskei_rejects_duplicate_site_id(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.jukskei-river-microbial-data.2025.v1")
        candidate["spatial_coverage"]["entities"][1]["site"] = "A"
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("duplicate ASM site ID" in error for error in errors), errors)

    def test_university_of_pretoria_candidate_does_not_invent_license_or_period(self) -> None:
        candidate = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.up-hartbeespoort-roodeplaat-water-quality.2025.v1")
        self.assertIsNone(candidate["temporal_coverage"]["start_date"])
        self.assertIsNone(candidate["temporal_coverage"]["end_date"])
        self.assertIn("CC BY 4.0", candidate["license_assessment"]["declared_file_license"])
        self.assertFalse(candidate["ingestion_gate"]["ingestion_authorized"])

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

    def test_same_day_multi_overpass_is_preserved(self) -> None:
        swam = self.manifest["candidates"][0]
        finding_ids = {finding["finding_id"] for finding in swam["known_findings"]}
        self.assertIn("SWAM-MULTI-OVERPASS-DAY-001", finding_ids)
        inspection = swam["reported_data_shape"]["preview_only_inspection"]
        self.assertFalse(inspection["digest_verified_locally"])
        self.assertIn("DateTime_UTC", inspection["additional_preview_checks"][0]["required_handling"])

    def test_wro_source_counts_remain_approximate(self) -> None:
        wro = next(c for c in self.manifest["candidates"] if c["candidate_id"] == "south-africa.national-water-chemistry.1972-2011.v1")
        shape = wro["reported_data_shape"]
        self.assertNotIn("reported_sample_count", shape)
        self.assertNotIn("reported_station_count", shape)
        self.assertIn("more than 500,000", shape["reported_sample_count_description"].lower())
        self.assertIn("over 2,000", shape["reported_station_count_description"].lower())

    def test_swam_scope_is_western_cape_not_gauteng(self) -> None:
        swam = self.manifest["candidates"][0]
        self.assertIn("Western Cape", swam["spatial_coverage"]["geography"])
        self.assertIn("target_geography_mismatch_for_johannesburg_pilot", swam["ingestion_gate"]["reason_codes"])
        self.assertIn("https://github.com/DalinJiang/EO-Africa-SWAM", swam["license_assessment"]["evidence_urls"])
        finding_ids = {finding["finding_id"] for finding in swam["known_findings"]}
        self.assertIn("SWAM-DATA-CODE-LICENSE-SCOPE-001", finding_ids)

    def test_code_license_is_not_treated_as_dataset_license(self) -> None:
        swam = self.manifest["candidates"][0]
        assessment = swam["license_assessment"]["assessment_note"].lower()
        self.assertIn("does not clearly license", assessment)
        self.assertEqual(swam["license_assessment"]["license_status"], "unverified")
        self.assertFalse(swam["ingestion_gate"]["ingestion_authorized"])

    def test_duplicate_finding_id_fails(self) -> None:
        findings = self.manifest["candidates"][0]["known_findings"]
        findings.append(copy.deepcopy(findings[0]))
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("duplicate finding_id" in error for error in errors), errors)

    def test_malformed_finding_fails_closed(self) -> None:
        self.manifest["candidates"][0]["known_findings"] = [None]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("known_findings[0] must be an object" in error for error in errors), errors)

    def test_finding_ids_are_unique_across_candidates(self) -> None:
        first = self.manifest["candidates"][0]["known_findings"][0]
        self.manifest["candidates"][1]["known_findings"] = [copy.deepcopy(first)]
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any("duplicate finding_id across dataset catalog" in error for error in errors), errors)

    def test_malformed_finding_severity_fails_closed(self) -> None:
        self.manifest["candidates"][0]["known_findings"][0]["severity"] = {"not": "a severity"}
        errors = validate_manifest(self.manifest, self.indicator_catalog)
        self.assertTrue(any(".severity is invalid" in error for error in errors), errors)

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
