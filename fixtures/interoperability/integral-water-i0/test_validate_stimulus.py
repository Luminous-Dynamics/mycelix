#!/usr/bin/env python3
"""Mutation controls for the MYC-INT-006E stimulus validator."""

from __future__ import annotations

import copy
import json
import unittest

import validate_stimulus as v


SUBJECT_IDS = {
    "water.system.alpha", "water.issue.maintenance", "water.observation.pressure",
    "water.report.operator", "water.evidence.bundle", "water.alternative.repair",
    "water.alternative.replace", "water.objection.access", "water.prediction.pressure",
    "water.recommendation.repair", "water.decision.repair", "water.decision.foreign",
    "water.authorization.repair", "water.attempt.repair", "water.receipt.repair",
    "water.outcome.pressure-low", "water.review.repair", "water.decision.supersede",
    "water.credential.operator", "water.certification.external", "water.summary.derived",
    "water.authority.foreign", "water.receipt.delivery",
}


def corpus_for(stimulus: dict) -> dict:
    return {
        "corpus_id": v.CORPUS_ID,
        "corpus_version": v.CORPUS_VERSION,
        "subjects": {subject: {} for subject in SUBJECT_IDS},
        "cases": {case_id: {} for case_id in stimulus["cases"]},
    }


class StimulusValidatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        with open("myc-int-006e-water-i0-stimulus.v1.json", "rb") as handle:
            cls.raw = handle.read()
        cls.document = v.parse_strict_json(cls.raw, label="stimulus")
        cls.corpus = corpus_for(cls.document)
        cls.corpus_raw = json.dumps(cls.corpus).encode()

    def validate_doc(self, document: dict) -> None:
        v.validate_bytes(json.dumps(document).encode(), self.corpus_raw)

    def test_pristine_validates(self) -> None:
        result = v.validate_bytes(self.raw, self.corpus_raw)
        self.assertEqual(result["cases"], 19)
        self.assertEqual(result["operations"], 19)

    def test_oracle_bearing_key_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.document)
        mutated["cases"]["I0-HOSTILE-001"]["expected_disposition"] = {"class": "Rejected"}
        with self.assertRaises(v.StimulusValidationError):
            self.validate_doc(mutated)

    def test_dangling_subject_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.document)
        mutated["cases"]["I0-HOSTILE-001"]["subjects"][0] = "water.missing"
        with self.assertRaises(v.StimulusValidationError):
            self.validate_doc(mutated)

    def test_unknown_operation_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.document)
        mutated["cases"]["I0-HOSTILE-001"]["operation"] = "EchoOracle"
        with self.assertRaises(v.StimulusValidationError):
            self.validate_doc(mutated)

    def test_case_set_drift_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.document)
        del mutated["cases"]["I0-HOSTILE-018"]
        with self.assertRaises(v.StimulusValidationError):
            self.validate_doc(mutated)

    def test_duplicate_json_member_is_rejected(self) -> None:
        raw = b'{"stimulus_id":"a","stimulus_id":"b"}'
        with self.assertRaises(v.StimulusValidationError):
            v.parse_strict_json(raw, label="stimulus")

    def test_commitment_mismatch_is_rejected(self) -> None:
        with self.assertRaises(v.StimulusValidationError):
            v.validate_bytes(self.raw, self.corpus_raw, expected_sha256="0" * 64)


if __name__ == "__main__":
    unittest.main()
