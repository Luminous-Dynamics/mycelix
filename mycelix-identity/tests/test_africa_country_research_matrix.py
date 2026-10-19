"""Fail-closed tests for the source-backed African country research matrix."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import validate_africa_country_research_matrix  # noqa: E402


class AfricaCountryResearchMatrixTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.matrix = json.loads(
            (IDENTITY_DIR / "AFRICA_COUNTRY_RESEARCH_MATRIX_V1.json").read_text(
                encoding="utf-8"
            )
        )

    def test_checked_matrix_is_valid_and_has_zero_qualified_integrations(self) -> None:
        self.assertEqual(
            validate_africa_country_research_matrix.validate_matrix(self.matrix), []
        )
        self.assertEqual(len(self.matrix["route_records"]), 5)
        self.assertEqual(self.matrix["qualified_integrations"], [])
        self.assertFalse(self.matrix["production_enabled"])

    def test_all_routes_remain_research_only(self) -> None:
        for route in self.matrix["route_records"]:
            with self.subTest(route=route["route_key"]):
                self.assertEqual(route["status"], "research_only")
                self.assertFalse(route["production_enabled"])
                self.assertFalse(route["external_verifier_protocol_reviewed"])
                self.assertFalse(route["written_authorization_obtained"])

    def test_south_african_ovs_and_smartphone_routes_are_distinct(self) -> None:
        routes = {route["route_key"]: route for route in self.matrix["route_records"]}
        ovs = routes["ZA-DHA-OVS"]
        wallet = routes["ZA-DHA-SMARTPHONE-DIGITAL-ID"]
        self.assertNotEqual(ovs["route_name"], wallet["route_name"])
        self.assertNotEqual(
            {source["url"] for source in ovs["official_sources"]},
            {source["url"] for source in wallet["official_sources"]},
        )
        self.assertIn("server-side", ovs["established_facts"][0])
        self.assertIn("prototype", wallet["established_facts"][0])

    def test_official_protocol_review_cannot_be_claimed_by_metadata_only(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["route_records"][0]["external_verifier_protocol_reviewed"] = True
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("external_verifier_protocol_reviewed must remain false" in e for e in errors))

    def test_written_authorization_cannot_be_assumed(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["route_records"][2]["written_authorization_obtained"] = True
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("written_authorization_obtained must remain false" in e for e in errors))

    def test_each_country_release_gate_must_remain_closed(self) -> None:
        for gate in sorted(validate_africa_country_research_matrix.GATE_KEYS):
            with self.subTest(gate=gate):
                mutated = copy.deepcopy(self.matrix)
                mutated["route_records"][0]["release_gates"][gate] = True
                errors = validate_africa_country_research_matrix.validate_matrix(mutated)
                self.assertTrue(any(gate in error for error in errors))

    def test_pinned_official_source_cannot_be_removed_or_replaced(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["route_records"][1]["official_sources"].pop()
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("pinned official sources" in error for error in errors))

    def test_unknown_country_or_route_cannot_be_added_without_matrix_revision(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        extra = copy.deepcopy(mutated["route_records"][2])
        extra["route_key"] = "XX-UNKNOWN-ID"
        mutated["route_records"].append(extra)
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("unknown route_key requires a reviewed matrix revision" in e for e in errors))

    def test_status_enum_cannot_gain_unreviewed_production_shortcuts(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["common_status_values"].append("mostly_ready")
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("closed v1 status enum" in error for error in errors))

    def test_unresolved_questions_must_remain_explicit(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["route_records"][4]["unresolved_questions"] = []
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("unresolved_questions must be a non-empty array" in error for error in errors))

    def test_route_with_missing_country_identity_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["route_records"][3]["country_code"] = ""
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("country_code must be a non-empty string" in error for error in errors))

    def test_boolean_schema_version_is_not_accepted_as_integer_one(self) -> None:
        mutated = copy.deepcopy(self.matrix)
        mutated["schema_version"] = True
        errors = validate_africa_country_research_matrix.validate_matrix(mutated)
        self.assertTrue(any("schema_version must be the integer 1" in error for error in errors))

    def test_duplicate_json_object_keys_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            validate_africa_country_research_matrix._reject_duplicate_object_keys(
                [("production_enabled", False), ("production_enabled", True)]
            )


if __name__ == "__main__":
    unittest.main()
