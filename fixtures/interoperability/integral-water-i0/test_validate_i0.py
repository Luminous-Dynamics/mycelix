#!/usr/bin/env python3
"""Mutation controls for validate_i0.py."""

from __future__ import annotations

import copy
import hashlib
import json
import unittest
from pathlib import Path

import validate_i0 as validator

HERE = Path(__file__).resolve().parent
CORPUS = HERE / validator.CORPUS_FILENAME
SCHEMA = HERE / validator.SCHEMA_FILENAME


class WaterI0ValidatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.corpus_raw = CORPUS.read_bytes()
        cls.schema_raw = SCHEMA.read_bytes()
        cls.document = validator.parse_strict_json(cls.corpus_raw, label="corpus")
        cls.digest = hashlib.sha256(cls.corpus_raw).hexdigest()

    def assert_rejected_document(self, document: dict) -> None:
        raw = json.dumps(document, sort_keys=True, separators=(",", ":")).encode("utf-8")
        with self.assertRaises(validator.ValidationError):
            validator.validate_bytes(raw, schema_raw=self.schema_raw)

    def test_pristine_fixture_validates(self) -> None:
        summary = validator.validate_bytes(
            self.corpus_raw,
            schema_raw=self.schema_raw,
            expected_sha256=self.digest,
        )
        self.assertEqual(summary["corpus_id"], validator.CORPUS_ID)
        self.assertEqual(summary["corpus_version"], validator.CORPUS_VERSION)
        self.assertEqual(summary["cases"], 19)
        self.assertEqual(summary["subjects"], 23)
        self.assertEqual(summary["sha256"], self.digest)

    def test_duplicate_json_member_rejected(self) -> None:
        stripped = self.corpus_raw.lstrip()
        self.assertTrue(stripped.startswith(b"{"))
        mutated = b'{"corpus_id":"duplicate",' + stripped[1:]
        with self.assertRaisesRegex(validator.ValidationError, "duplicate JSON object member"):
            validator.validate_bytes(mutated, schema_raw=self.schema_raw)

    def test_dangling_input_reference_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["cases"]["I0-HOSTILE-001"]["inputs"][0] = "water.missing.subject"
        self.assert_rejected_document(doc)

    def test_unknown_disposition_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["cases"]["I0-HOSTILE-001"]["expected_disposition"]["class"] = "Maybe"
        self.assert_rejected_document(doc)

    def test_unknown_predicate_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["cases"]["I0-HOSTILE-001"]["assertions"][0]["predicate"] = "LooksFine"
        self.assert_rejected_document(doc)

    def test_malformed_semantic_ref_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["subjects"]["water.system.alpha"]["semantic_ref"]["namespace"] = "Bad Namespace"
        self.assert_rejected_document(doc)

    def test_changed_corpus_identity_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["corpus_id"] = "myc-int-006c-water-i0-mutated"
        self.assert_rejected_document(doc)

    def test_empty_assertions_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["cases"]["I0-HOSTILE-001"]["assertions"] = []
        self.assert_rejected_document(doc)

    def test_unregistered_subject_kind_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["subjects"]["water.system.alpha"]["kind"] = "MagicResource"
        self.assert_rejected_document(doc)

    def test_unregistered_reason_code_rejected(self) -> None:
        doc = copy.deepcopy(self.document)
        doc["cases"]["I0-HOSTILE-001"]["expected_disposition"]["reason_code"] = "NEW_UNREGISTERED_REASON"
        self.assert_rejected_document(doc)

    def test_commitment_mismatch_rejected(self) -> None:
        mutated = self.corpus_raw + b" "
        with self.assertRaisesRegex(validator.ValidationError, "commitment mismatch"):
            validator.validate_bytes(
                mutated,
                schema_raw=self.schema_raw,
                expected_sha256=self.digest,
            )

    def test_schema_registry_drift_rejected(self) -> None:
        schema = validator.parse_strict_json(self.schema_raw, label="schema")
        schema["$defs"]["subject"]["properties"]["kind"]["enum"].append("UnregisteredKind")
        mutated_schema = json.dumps(schema, separators=(",", ":")).encode("utf-8")
        with self.assertRaisesRegex(validator.ValidationError, "subject-kind registry"):
            validator.validate_bytes(self.corpus_raw, schema_raw=mutated_schema)


if __name__ == "__main__":
    unittest.main()
