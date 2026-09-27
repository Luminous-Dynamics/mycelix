#!/usr/bin/env python3
"""Mutation and positive controls for MYC-INT-006K neutral protocol validator."""

from __future__ import annotations

import copy
import hashlib
import json
import unittest
from pathlib import Path

import build_candidate_input_v2 as neutral_input
import conventional_i0_adapter_v2 as neutral_adapter
import validate_candidate_protocol as validator


class NeutralProtocolValidatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        here = Path(__file__).resolve().parent
        cls.corpus_raw = (here / "myc-int-006c-water-i0-corpus.v1.json").read_bytes()
        cls.corpus_schema_raw = (here / "myc-int-006c-water-i0-corpus.schema.json").read_bytes()
        cls.stimulus_raw = (here / "myc-int-006e-water-i0-stimulus.v1.json").read_bytes()
        cls.input_schema_raw = (here / "myc-int-006i-i0-candidate-input.schema.json").read_bytes()
        cls.result_schema_raw = (here / "myc-int-006i-i0-candidate-results.schema.json").read_bytes()
        cls.input_document = neutral_input.build_from_bytes(
            cls.corpus_raw, cls.corpus_schema_raw, cls.stimulus_raw
        )
        cls.input_raw = neutral_input.canonical_bytes(cls.input_document)
        cls.result_document = neutral_adapter.run_bytes(cls.input_raw)
        cls.result_raw = neutral_adapter.canonical_bytes(cls.result_document)

    def test_actual_006j_pair_validates(self) -> None:
        summary = validator.validate_pair(
            self.input_raw,
            self.result_raw,
            input_schema_raw=self.input_schema_raw,
            result_schema_raw=self.result_schema_raw,
            corpus_raw=self.corpus_raw,
            stimulus_raw=self.stimulus_raw,
        )
        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["input"]["cases"], 19)
        self.assertEqual(summary["results"]["cases"], 19)

    def test_duplicate_json_member_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            validator.validate_candidate_input_bytes(b'{"x":1,"x":2}')

    def test_unknown_input_field_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.input_document)
        mutated["unknown_field"] = True
        with self.assertRaises(ValueError):
            validator.validate_candidate_input_bytes(validator.canonical_bytes(mutated))

    def test_oracle_field_leak_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.input_document)
        mutated["cases"]["I0-HOSTILE-001"]["expected_disposition"] = {"class": "Rejected"}
        with self.assertRaises(ValueError):
            validator.validate_candidate_input_bytes(validator.canonical_bytes(mutated))

    def test_dangling_subject_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.input_document)
        mutated["cases"]["I0-HOSTILE-001"]["subjects"] = ["missing.subject"]
        with self.assertRaises(ValueError):
            validator.validate_candidate_input_bytes(validator.canonical_bytes(mutated))

    def test_source_commitment_mismatch_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            validator.validate_candidate_input_bytes(
                self.input_raw,
                corpus_raw=self.corpus_raw + b"x",
                stimulus_raw=self.stimulus_raw,
            )

    def test_result_input_commitment_mismatch_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.result_document)
        mutated["candidate_input_sha256"] = "0" * 64
        with self.assertRaises(ValueError):
            validator.validate_candidate_results_bytes(
                validator.canonical_bytes(mutated), candidate_input_raw=self.input_raw
            )

    def test_missing_result_case_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.result_document)
        mutated["results"] = mutated["results"][:-1]
        with self.assertRaises(ValueError):
            validator.validate_candidate_results_bytes(
                validator.canonical_bytes(mutated), candidate_input_raw=self.input_raw
            )

    def test_unknown_fact_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.result_document)
        mutated["results"][0]["facts"]["mystery_fact"] = True
        with self.assertRaises(ValueError):
            validator.validate_candidate_results_bytes(
                validator.canonical_bytes(mutated), candidate_input_raw=self.input_raw
            )

    def test_unknown_disposition_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.result_document)
        mutated["results"][0]["disposition"] = "Maybe"
        with self.assertRaises(ValueError):
            validator.validate_candidate_results_bytes(
                validator.canonical_bytes(mutated), candidate_input_raw=self.input_raw
            )

    def test_duplicate_tag_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.result_document)
        mutated["results"][0]["tags"] = ["same", "same"]
        with self.assertRaises(ValueError):
            validator.validate_candidate_results_bytes(
                validator.canonical_bytes(mutated), candidate_input_raw=self.input_raw
            )

    def test_schema_registry_drift_is_rejected(self) -> None:
        input_schema = json.loads(self.input_schema_raw)
        operations = input_schema["$defs"]["operation"]["enum"]
        input_schema["$defs"]["operation"]["enum"] = operations[:-1]
        with self.assertRaises(ValueError):
            validator.validate_schema_registries(
                validator.canonical_bytes(input_schema), self.result_schema_raw
            )

    def test_exact_candidate_input_binding_is_accepted(self) -> None:
        expected = hashlib.sha256(self.input_raw).hexdigest()
        self.assertEqual(self.result_document["candidate_input_sha256"], expected)
        summary = validator.validate_candidate_results_bytes(
            self.result_raw, candidate_input_raw=self.input_raw
        )
        self.assertEqual(summary["cases"], 19)


if __name__ == "__main__":
    unittest.main()
