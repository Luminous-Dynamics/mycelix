"""Fail-closed tests for the regenerative indicator catalog contract."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from validate_regenerative_indicators import validate_catalog  # noqa: E402

CATALOG_PATH = REPO_ROOT / "docs" / "economy-atlas" / "regenerative-indicators.v1.json"
REGISTRY_PATH = REPO_ROOT / "docs" / "economy-atlas" / "model-registry.v1.json"


class RegenerativeIndicatorCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.fixture = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
        cls.model_registry = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))

    def setUp(self) -> None:
        self.catalog = copy.deepcopy(self.fixture)

    def test_checked_in_catalog_passes(self) -> None:
        self.assertEqual(validate_catalog(self.catalog, self.model_registry), [])

    def test_non_object_catalog_root_fails_closed(self) -> None:
        errors = validate_catalog([], self.model_registry)
        self.assertEqual(errors, ["catalog root must be an object"])

    def test_malformed_missingness_vocabulary_fails_closed(self) -> None:
        self.catalog["missingness_vocabulary"] = [{"unknown": True}]
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("missingness_vocabulary must be a non-empty string list" in error for error in errors), errors)

    def test_required_indicator_set_is_present(self) -> None:
        ids = {indicator["indicator_id"] for indicator in self.catalog["indicators"]}
        self.assertEqual(len(ids), 21)
        for required in (
            "water.physicochemical-profile.v1",
            "water.microbial-contamination.v1",
            "water.toxic-elements-concentration.v1",
        ):
            self.assertIn(required, ids)

    def test_duplicate_indicator_id_fails(self) -> None:
        self.catalog["indicators"].append(copy.deepcopy(self.catalog["indicators"][0]))
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("duplicate indicator_id" in error for error in errors), errors)

    def test_unknown_research_ref_fails(self) -> None:
        self.catalog["indicators"][0]["research_ref_ids"].append("made-up-citation")
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("unknown research refs" in error for error in errors), errors)

    def test_non_https_reference_fails(self) -> None:
        self.catalog["research_refs"][0]["url"] = "http://example.invalid/reference"
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any(".url must be HTTPS" in error for error in errors), errors)

    def test_impossible_retrieval_date_fails(self) -> None:
        self.catalog["research_refs"][0]["retrieved_on"] = "2026-02-31"
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("real YYYY-MM-DD date" in error for error in errors), errors)

    def test_indicator_cannot_claim_computed_result(self) -> None:
        self.catalog["indicators"][0]["computed"] = True
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("computed must be false" in error for error in errors), errors)

    def test_catalog_cannot_claim_independent_validation(self) -> None:
        self.catalog["qualification"]["independently_validated"] = True
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("qualification must keep all evidence/execution flags false" in error for error in errors), errors)

    def test_unknown_framework_model_fails(self) -> None:
        self.catalog["indicators"][0]["framework_ids"] = ["not.a.registered.model"]
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("unknown model IDs" in error for error in errors), errors)

    def test_removing_required_indicator_fails(self) -> None:
        self.catalog["indicators"] = self.catalog["indicators"][1:]
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("required indicator definitions missing" in error for error in errors), errors)

    def test_empty_comparability_key_fails(self) -> None:
        self.catalog["indicators"][0]["comparability_key"] = []
        errors = validate_catalog(self.catalog, self.model_registry)
        self.assertTrue(any("comparability_key must be a non-empty string list" in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main()
