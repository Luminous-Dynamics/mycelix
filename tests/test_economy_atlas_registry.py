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

    def test_malformed_unhashable_source_fields_fail_cleanly(self) -> None:
        model = next(
            item for item in self.registry["models"]
            if item["model_id"] == "mycelix.finance.tricurrency.v1"
        )
        model["source_refs"][0]["repository"] = ["not", "a", "repository"]
        errors = validate_registry(self.registry)
        self.assertTrue(any("repository must use owner/repo format" in error for error in errors), errors)

    def test_validation_findings_bind_to_pinned_source_refs(self) -> None:
        finance = next(
            model for model in self.registry["models"]
            if model["model_id"] == "mycelix.finance.tricurrency.v1"
        )
        self.assertEqual(len(finance["validation_findings"]), 4)
        for finding in finance["validation_findings"]:
            self.assertTrue(any(
                source["repository"] == finding["source_repository"]
                and source["path"] == finding["source_file"]
                and source["ref"] == finding["source_commit"]
                for source in finance["source_refs"]
            ))
        self.assertEqual(validate_registry(self.registry), [])

    def test_hearth_compost_delivery_status_finding_is_explicit(self) -> None:
        finance = next(
            model for model in self.registry["models"]
            if model["model_id"] == "mycelix.finance.tricurrency.v1"
        )
        finding = next(
            item for item in finance["validation_findings"]
            if item["finding_id"] == "FIN-ATLAS-COMPOST-DELIVERY-STATUS-001"
        )
        coordinator = "mycelix-finance/zomes/payments/coordinator/src/lib.rs"
        self.assertEqual(finding["source_file"], coordinator)
        self.assertTrue(any(
            source["path"] == coordinator
            and source["ref"] == finding["source_commit"]
            and source["url"].endswith(f"/{finding['source_commit']}/{coordinator}")
            for source in finance["source_refs"]
        ))
        self.assertIn("redistributed: true", finding["observation"])
        self.assertIn("failed hearth deliveries", finding["required_follow_up"])
        self.assertEqual(validate_registry(self.registry), [])

    def test_dangling_validation_finding_source_fails(self) -> None:
        finance = next(
            model for model in self.registry["models"]
            if model["model_id"] == "mycelix.finance.tricurrency.v1"
        )
        finance["validation_findings"][0]["source_file"] = "missing/source.rs"
        errors = validate_registry(self.registry)
        self.assertTrue(
            any("exact source_refs repository/path/commit" in error for error in errors),
            errors,
        )

    def test_duplicate_validation_finding_ids_fail(self) -> None:
        finance = next(
            model for model in self.registry["models"]
            if model["model_id"] == "mycelix.finance.tricurrency.v1"
        )
        duplicate = copy.deepcopy(finance["validation_findings"][0])
        finance["validation_findings"].append(duplicate)
        errors = validate_registry(self.registry)
        self.assertTrue(any("duplicate validation finding ID" in error for error in errors), errors)

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
