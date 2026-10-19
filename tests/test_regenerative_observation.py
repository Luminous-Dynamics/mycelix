"""Tests for explicit missingness and provenance in observation receipts."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from validate_regenerative_observation import validate_observation  # noqa: E402

CATALOG_PATH = REPO_ROOT / "docs" / "economy-atlas" / "regenerative-indicators.v1.json"
TEST_DIGEST = "a" * 64


def fixture_record() -> dict:
    """Synthetic unit-test data only; it is not an environmental observation."""
    return {
        "schema_version": "1.0.0",
        "record_type": "regenerative_observation_record",
        "observation_id": "test:soil-carbon:sample-001",
        "qualification_status": "unreviewed_source_observation",
        "indicator_id": "soil.organic-carbon-stock.v1",
        "indicator_catalog_sha256": TEST_DIGEST,
        "indicator_native_unit_contract": "tonnes of carbon per hectare for a stated depth (t C/ha, depth specified)",
        "unit_mapping_note": "Synthetic test unit is an illustrative depth-qualified choice under the catalog's native-unit contract.",
        "denominator_contract_note": "The synthetic plot/soil-depth denominator maps to the indicator's stated area-and-depth contract.",
        "observation_status": "observed",
        "source": {
            "publisher": "Synthetic Test Publisher",
            "dataset_id": "TEST-DATASET",
            "series_id": "TEST-SERIES",
            "source_url": "https://example.invalid/synthetic-test-only",
            "source_revision": "test-revision-1",
            "retrieved_on": "2026-10-12",
            "artifact_sha256": TEST_DIGEST,
            "license_or_access_policy": "Synthetic fixture for automated unit tests only"
        },
        "geography": {
            "name": "Synthetic Test Plot",
            "geography_id": "TEST-GEO",
            "boundary_id": "TEST-BOUNDARY",
            "boundary_version": "1",
            "spatial_scope": "Synthetic plot used only for a validator test"
        },
        "period": {
            "start_date": "2026-01-01",
            "end_date": "2026-12-31",
            "temporal_resolution": "synthetic annual test period"
        },
        "native_unit": "t C/ha, sampled depth 0-30 cm (synthetic test unit)",
        "denominator_definition": "Synthetic sampled test plot area and soil-depth convention",
        "measurement": {"kind": "point", "value": 12.5},
        "transformation": {
            "kind": "identity",
            "method_id": "test.identity",
            "version": "1",
            "description": "Synthetic no-op transformation for tests",
            "parameters": {},
            "output_unit": "t C/ha, sampled depth 0-30 cm (synthetic test unit)",
            "input_sha256": TEST_DIGEST
        },
        "coverage": {
            "state": "complete",
            "coverage_fraction": 1.0,
            "definition": "All synthetic test samples in the test fixture"
        },
        "uncertainty": {
            "status": "qualitative",
            "method": "synthetic test note",
            "description": "No real measurement uncertainty exists; this fixture is only for validator tests"
        },
        "quality_flags": ["synthetic-test-data"]
    }


class RegenerativeObservationReceiptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.catalog = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))

    def setUp(self) -> None:
        self.record = fixture_record()

    def test_synthetic_point_fixture_passes(self) -> None:
        self.assertEqual(validate_observation(self.record, self.catalog), [])

    def test_malformed_observation_status_fails_closed(self) -> None:
        self.record["observation_status"] = ["observed"]
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("observation_status must be an allowed" in error for error in errors), errors)

    def test_invalid_observation_id_fails(self) -> None:
        self.record["observation_id"] = "../not-safe"
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("observation_id must use lowercase" in error for error in errors), errors)

    def test_catalog_reference_is_required(self) -> None:
        self.record["indicator_id"] = "not.registered"
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("not present in the candidate catalog" in error for error in errors), errors)

    def test_catalog_digest_must_match_when_expected_digest_is_supplied(self) -> None:
        errors = validate_observation(self.record, self.catalog, expected_catalog_sha256="b" * 64)
        self.assertTrue(any("does not match the checked-in catalog bytes" in error for error in errors), errors)

    def test_identity_transform_input_digest_must_match_source_artifact(self) -> None:
        self.record["transformation"]["input_sha256"] = "b" * 64
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("identity transformation input_sha256 must match" in error for error in errors), errors)

    def test_source_artifact_digest_is_required(self) -> None:
        self.record["source"]["artifact_sha256"] = "not-a-digest"
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("source.artifact_sha256 must be a lowercase SHA-256 digest" in error for error in errors), errors)

    def test_partial_coverage_requires_fraction_between_zero_and_one(self) -> None:
        self.record["coverage"] = {"state": "partial", "coverage_fraction": None, "definition": "Synthetic partial coverage"}
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("partial coverage requires a finite" in error for error in errors), errors)

    def test_malformed_coverage_state_fails_closed_without_crashing(self) -> None:
        self.record["coverage"]["state"] = ["complete"]
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("coverage.state must be explicit" in error for error in errors), errors)

    def test_invalid_calendar_date_fails(self) -> None:
        self.record["period"]["start_date"] = "2026-02-31"
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("period.start_date must be a real" in error for error in errors), errors)

    def test_reversed_period_fails(self) -> None:
        self.record["period"]["start_date"] = "2026-12-31"
        self.record["period"]["end_date"] = "2026-01-01"
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("must not be after" in error for error in errors), errors)

    def test_missing_value_cannot_be_zero_imputed(self) -> None:
        self.record["observation_status"] = "not_collected"
        self.record["measurement"] = {"kind": "missing", "value": 0}
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("missing states require kind=missing, null value" in error for error in errors), errors)

    def test_below_detection_limit_must_be_censored_not_zero(self) -> None:
        self.record["observation_status"] = "below_detection_limit"
        self.record["measurement"] = {
            "kind": "censored",
            "value": None,
            "censoring": {"operator": "<", "threshold": 0.01, "unit": "test-unit"}
        }
        self.assertEqual(validate_observation(self.record, self.catalog), [])

    def test_observation_cannot_claim_independent_qualification(self) -> None:
        self.record["qualification_status"] = "validated"
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("qualification_status must remain unreviewed_source_observation" in error for error in errors), errors)

    def test_non_finite_measurement_fails(self) -> None:
        self.record["measurement"]["value"] = float("nan")
        errors = validate_observation(self.record, self.catalog)
        self.assertTrue(any("finite numeric point value" in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main()
