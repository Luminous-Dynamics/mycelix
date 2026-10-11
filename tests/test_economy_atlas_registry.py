"""Fail-closed tests for the Economy Atlas discovery registry contract."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from validate_economy_atlas_registry import validate_registry  # noqa: E402

REGISTRY_PATH = REPO_ROOT / "docs" / "economy-atlas" / "model-registry.v1.json"


class EconomyAtlasRegistryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        with REGISTRY_PATH.open("r", encoding="utf-8") as handle:
            cls.fixture = json.load(handle)

    def setUp(self) -> None:
        self.registry = copy.deepcopy(self.fixture)

    def test_current_discovery_registry_passes(self) -> None:
        self.assertEqual(validate_registry(self.registry), [])

    def test_duplicate_model_ids_fail(self) -> None:
        self.registry["models"].append(copy.deepcopy(self.registry["models"][0]))
        errors = validate_registry(self.registry)
        self.assertTrue(any("duplicate model_id" in error for error in errors), errors)

    def test_execution_cannot_be_marked_qualified_in_v1(self) -> None:
        self.registry["qualified_for_execution"] = True
        errors = validate_registry(self.registry)
        self.assertTrue(any("qualified_for_execution=false" in error for error in errors), errors)

    def test_mutable_source_ref_fails(self) -> None:
        source = self.registry["models"][0]["source_refs"][0]
        source["ref"] = "main"
        errors = validate_registry(self.registry)
        self.assertTrue(any("immutable lowercase 40-hex" in error for error in errors), errors)

    def test_source_url_commit_must_match_ref(self) -> None:
        source = self.registry["models"][0]["source_refs"][0]
        source["url"] = source["url"].replace(source["ref"], "0000000000000000000000000000000000000000")
        errors = validate_registry(self.registry)
        self.assertTrue(any("url commit does not match source.ref" in error for error in errors), errors)

    def test_discovery_item_is_the_only_source_less_model(self) -> None:
        model = next(m for m in self.registry["models"] if m["model_id"] == "mycelix.finance.tricurrency.v1")
        model["source_refs"] = []
        errors = validate_registry(self.registry)
        self.assertTrue(any("no sources but is not marked discovery_required" in error for error in errors), errors)

    def test_currency_semantics_remain_separate_variants(self) -> None:
        self.registry["models"] = [
            m for m in self.registry["models"]
            if m["model_id"] != "symthaea.concepts.tricurrency.essay16.v1"
        ]
        errors = validate_registry(self.registry)
        self.assertTrue(any("required semantically distinct currency variants missing" in error for error in errors), errors)

    def test_snapshot_sha_must_match_each_permalink(self) -> None:
        source = self.registry["models"][0]["source_refs"][0]
        repo = source["repository"]
        self.registry["generated_from"]["pinned_source_snapshots"][repo] = "f" * 40
        errors = validate_registry(self.registry)
        self.assertTrue(any("does not match the registry's pinned snapshot" in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main()
