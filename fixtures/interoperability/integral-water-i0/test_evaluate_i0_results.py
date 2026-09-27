#!/usr/bin/env python3
"""Mutation controls for MYC-INT-006G semantic evaluator."""

from __future__ import annotations

import copy
import unittest

import evaluate_i0_results as e


def oracle() -> dict:
    return {
        "corpus_id": e.CORPUS_ID,
        "corpus_version": e.CORPUS_VERSION,
        "cases": {
            "I0-A": {
                "expected_disposition": {"class": "Rejected"},
                "assertions": [
                    {"predicate": "NoEffectAuthority"},
                    {"predicate": "NoHistoricalMutation"},
                ],
            },
            "I0-B": {
                "expected_disposition": {"class": "Accepted"},
                "assertions": [
                    {"predicate": "IdempotentReplay"},
                    {"predicate": "SingleLogicalEffect"},
                ],
            },
        },
    }


def candidate() -> dict:
    return {
        "result_profile": e.RESULT_PROFILE,
        "adapter_profile": "candidate-test-v1",
        "candidate_input_id": "input",
        "candidate_input_version": "1",
        "results": [
            {
                "case_id": "I0-A",
                "disposition": "Rejected",
                "candidate_rule": "r1",
                "effect_count": 0,
                "tags": [],
                "facts": {
                    "effect_authority_granted": False,
                    "historical_mutation": False,
                },
            },
            {
                "case_id": "I0-B",
                "disposition": "Accepted",
                "candidate_rule": "r2",
                "effect_count": 1,
                "tags": [],
                "facts": {
                    "idempotent_replay": True,
                    "logical_effect_count": 1,
                },
            },
        ],
    }


class EvaluatorTests(unittest.TestCase):
    def test_pristine_passes(self) -> None:
        report = e.evaluate_documents(oracle(), candidate())
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["disposition_matches"], 2)
        self.assertEqual(report["assertion_checks"], 4)
        self.assertEqual(report["assertion_passes"], 4)

    def test_wrong_disposition_fails(self) -> None:
        c = candidate()
        c["results"][0]["disposition"] = "Accepted"
        self.assertEqual(e.evaluate_documents(oracle(), c)["status"], "FAIL")

    def test_missing_fact_fails(self) -> None:
        c = candidate()
        del c["results"][0]["facts"]["historical_mutation"]
        self.assertEqual(e.evaluate_documents(oracle(), c)["status"], "FAIL")

    def test_wrong_fact_value_fails(self) -> None:
        c = candidate()
        c["results"][0]["facts"]["effect_authority_granted"] = True
        self.assertEqual(e.evaluate_documents(oracle(), c)["status"], "FAIL")

    def test_case_set_mismatch_is_invalid(self) -> None:
        c = candidate()
        c["results"].pop()
        with self.assertRaises(e.EvaluationError):
            e.evaluate_documents(oracle(), c)

    def test_oracle_key_leak_is_invalid(self) -> None:
        c = candidate()
        c["results"][0]["reason_code"] = "COPIED_FROM_ORACLE"
        with self.assertRaises(e.EvaluationError):
            e.evaluate_documents(oracle(), c)

    def test_single_logical_effect_requires_both_counts(self) -> None:
        c = candidate()
        c["results"][1]["effect_count"] = 2
        self.assertEqual(e.evaluate_documents(oracle(), c)["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
