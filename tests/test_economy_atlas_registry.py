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

    def test_all_findings_are_structured_and_bound_to_exact_sources(self) -> None:
        seen_ids: set[str] = set()
        for model in self.registry["models"]:
            sources = {
                (source["repository"], source["path"], source["ref"])
                for source in model["source_refs"]
            }
            for finding in model.get("validation_findings", []):
                self.assertIsInstance(finding, dict)
                for field in (
                    "finding_id",
                    "severity",
                    "source_repository",
                    "source_file",
                    "source_commit",
                    "observation",
                    "required_follow_up",
                ):
                    self.assertIsInstance(finding.get(field), str)
                    self.assertTrue(finding[field].strip(), (model["model_id"], field))
                self.assertNotIn(finding["finding_id"], seen_ids)
                seen_ids.add(finding["finding_id"])
                self.assertIn(
                    (
                        finding["source_repository"],
                        finding["source_file"],
                        finding["source_commit"],
                    ),
                    sources,
                )
        self.assertEqual(len(seen_ids), 7)

    def test_regenerative_frameworks_are_first_class_external_models(self) -> None:
        expected = {
            "regenerative.economics.energy-network-science.v1",
            "regenerative.doughnut.place-portrait.v1",
            "regenerative.circular-economy.v1",
            "regenerative.land.agroecology-outcomes.v1",
        }
        models = {model["model_id"]: model for model in self.registry["models"]}
        self.assertTrue(expected.issubset(models))
        for model_id in expected:
            model = models[model_id]
            self.assertEqual(model["epistemic_status"], "external_framework")
            self.assertIn(
                model["execution_status"].split(";")[0],
                {"research_framework", "external_methodology", "principles_documented", "outcomes_framework"},
            )
            self.assertEqual(model["source_refs"], [])
            self.assertGreaterEqual(len(model["research_refs"]), 1)

    def test_external_research_references_are_https_and_typed(self) -> None:
        for model in self.registry["models"]:
            for ref in model.get("research_refs", []):
                self.assertTrue(ref["url"].startswith("https://"), ref)
                for field in ("title", "publisher", "reference_type", "retrieved_on"):
                    self.assertTrue(ref[field].strip(), (model["model_id"], field))
        self.assertEqual(validate_registry(self.registry), [])

    def test_required_regenerative_model_families_cannot_be_removed(self) -> None:
        regenerative_ids = {
            "regenerative.economics.energy-network-science.v1",
            "regenerative.doughnut.place-portrait.v1",
            "regenerative.circular-economy.v1",
        }
        self.registry["models"] = [
            model for model in self.registry["models"]
            if model["model_id"] != "regenerative.doughnut.place-portrait.v1"
        ]
        errors = validate_registry(self.registry)
        self.assertTrue(any(
            "required regenerative-economics frameworks missing" in error
            and "regenerative.doughnut.place-portrait.v1" in error
            for error in errors
        ), errors)
        self.assertTrue(regenerative_ids.difference(
            model["model_id"] for model in self.registry["models"]
        ))

    def test_external_research_reference_requires_real_retrieval_date(self) -> None:
        model = next(
            item for item in self.registry["models"]
            if item["model_id"] == "regenerative.economics.energy-network-science.v1"
        )
        model["research_refs"][0]["retrieved_on"] = "2026-02-31"
        errors = validate_registry(self.registry)
        self.assertTrue(any("real ISO-8601 calendar date" in error for error in errors), errors)

    def test_external_framework_without_research_reference_fails(self) -> None:
        model = next(
            item for item in self.registry["models"]
            if item["model_id"] == "regenerative.doughnut.place-portrait.v1"
        )
        model["research_refs"] = []
        errors = validate_registry(self.registry)
        self.assertTrue(any("requires at least one research_ref" in error for error in errors), errors)

    def test_external_research_reference_rejects_non_https_url(self) -> None:
        model = next(
            item for item in self.registry["models"]
            if item["model_id"] == "regenerative.circular-economy.v1"
        )
        model["research_refs"][0]["url"] = "javascript:alert(1)"
        errors = validate_registry(self.registry)
        self.assertTrue(any("absolute HTTPS URL" in error for error in errors), errors)

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
