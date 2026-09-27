#!/usr/bin/env python3
"""Regression controls for MYC-INT-006J conventional neutral-protocol migration."""

from __future__ import annotations

import copy
import hashlib
import json
import unittest
from pathlib import Path

import build_candidate_input as legacy_input
import build_candidate_input_v2 as neutral_input
import conventional_i0_adapter as legacy_adapter
import conventional_i0_adapter_v2 as neutral_adapter


class NeutralMigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        here = Path(__file__).resolve().parent
        cls.corpus_raw = (here / "myc-int-006c-water-i0-corpus.v1.json").read_bytes()
        cls.schema_raw = (here / "myc-int-006c-water-i0-corpus.schema.json").read_bytes()
        cls.stimulus_raw = (here / "myc-int-006e-water-i0-stimulus.v1.json").read_bytes()
        cls.neutral_document = neutral_input.build_from_bytes(cls.corpus_raw, cls.schema_raw, cls.stimulus_raw)
        cls.neutral_raw = neutral_input.canonical_bytes(cls.neutral_document)

    def test_neutral_input_has_exact_source_commitments(self) -> None:
        commitments = self.neutral_document["source_commitments"]
        self.assertEqual(commitments["corpus_sha256"], hashlib.sha256(self.corpus_raw).hexdigest())
        self.assertEqual(commitments["stimulus_sha256"], hashlib.sha256(self.stimulus_raw).hexdigest())
        self.assertEqual(self.neutral_document["candidate_input_id"], neutral_input.INPUT_ID)
        self.assertNotIn("conventional", self.neutral_document["candidate_input_id"])

    def test_oracle_fields_are_absent_from_neutral_input(self) -> None:
        encoded = self.neutral_raw.decode("utf-8")
        for forbidden in ("expected_disposition", "reason_code", '"assertions"', '"predicate"'):
            self.assertNotIn(forbidden, encoded)

    def test_result_binds_exact_candidate_input_bytes(self) -> None:
        result = neutral_adapter.run_bytes(self.neutral_raw)
        self.assertEqual(result["candidate_input_sha256"], hashlib.sha256(self.neutral_raw).hexdigest())
        self.assertEqual(result["result_protocol_id"], neutral_adapter.RESULT_PROTOCOL_ID)
        self.assertEqual(result["implementation"]["family"], "conventional-sqlite")

    def test_neutral_candidate_rejects_oracle_bearing_input(self) -> None:
        mutated = copy.deepcopy(self.neutral_document)
        mutated["cases"]["I0-HOSTILE-001"]["expected_disposition"] = {"class": "Rejected"}
        with self.assertRaises(ValueError):
            neutral_adapter.run_bytes(neutral_input.canonical_bytes(mutated))

    def test_all_19_operations_remain_supported(self) -> None:
        result = neutral_adapter.run_bytes(self.neutral_raw)
        self.assertEqual(len(result["results"]), 19)
        self.assertFalse([row for row in result["results"] if row["disposition"] == "Unsupported"])

    def test_semantics_match_generation_one_candidate(self) -> None:
        legacy_document = legacy_input.build_from_bytes(self.corpus_raw, self.schema_raw, self.stimulus_raw)
        legacy = legacy_adapter.run_document(legacy_document)
        neutral = neutral_adapter.run_bytes(self.neutral_raw)
        legacy_by_case = {row["case_id"]: row for row in legacy["results"]}
        neutral_by_case = {row["case_id"]: row for row in neutral["results"]}
        self.assertEqual(set(legacy_by_case), set(neutral_by_case))
        for case_id in sorted(legacy_by_case):
            old = legacy_by_case[case_id]
            new = neutral_by_case[case_id]
            self.assertEqual(new["disposition"], old["disposition"], case_id)
            self.assertEqual(new["effect_count"], old["effect_count"], case_id)
            self.assertEqual(new["facts"], old["facts"], case_id)

    def test_byte_change_changes_bound_commitment_without_semantic_drift(self) -> None:
        altered = self.neutral_raw + b" "
        original_result = neutral_adapter.run_bytes(self.neutral_raw)
        altered_result = neutral_adapter.run_bytes(altered)
        self.assertNotEqual(original_result["candidate_input_sha256"], altered_result["candidate_input_sha256"])
        self.assertEqual(original_result["results"], altered_result["results"])


if __name__ == "__main__":
    unittest.main()
