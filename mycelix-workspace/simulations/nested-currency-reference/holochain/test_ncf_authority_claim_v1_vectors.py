#!/usr/bin/env python3
"""Independent conformance checks for NCF-AUTHORITY-CLAIM-V1 golden vectors.

This test reads the golden JSON and reimplements the encoder instead of importing
the fixture generator. It tests content binding only—not signatures, PostgreSQL
commit truth, or Holochain validation.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import unittest
import uuid

FIXTURE = Path(__file__).with_name("ncf-authority-claim-v1-vectors.json")
DOMAIN = b"NCF-AUTHORITY-CLAIM-V1\x00"
FIELD_ORDER = (
    "schema_version", "outbox_event_id", "obligation_id", "request_id",
    "canonical_work_digest", "principal_id", "community_id", "instrument_id",
    "profile_revision", "amount_units", "agreement_id", "direction",
    "reservation_id", "shard_id", "epoch", "verifier_version",
    "verifier_receipt_digest", "sql_receipt_digest", "authority_manifest_digest",
)
UUID_FIELDS = ("outbox_event_id", "request_id", "agreement_id", "reservation_id")
POSITIVE_DECIMALS = ("obligation_id", "profile_revision", "amount_units")
DIGEST_FIELDS = (
    "canonical_work_digest", "verifier_receipt_digest",
    "sql_receipt_digest", "authority_manifest_digest",
)


def _is_canonical_decimal(value: str, positive: bool) -> bool:
    if not value or not value.isascii() or not value.isdigit():
        return False
    if len(value) > 1 and value.startswith("0"):
        return False
    return not positive or int(value) > 0


def canonical_preimage(fields: dict[str, object]) -> bytes:
    """Encode only the canonical ordered string map defined by NCF-AUTHORITY-CLAIM-V1."""
    if tuple(fields.keys()) != FIELD_ORDER:
        raise ValueError("fields must appear in canonical field order")
    if any(not isinstance(fields[name], str) for name in FIELD_ORDER):
        raise ValueError("all fields must be strings")
    values = [fields[name] for name in FIELD_ORDER]
    if any((not value.isascii()) or ("\x00" in value) for value in values):
        raise ValueError("field values must be NUL-free ASCII")
    if fields["schema_version"] != "1":
        raise ValueError("unsupported schema version")
    for name in UUID_FIELDS:
        value = fields[name]
        try:
            parsed = str(uuid.UUID(value))
        except (ValueError, AttributeError, TypeError) as exc:
            raise ValueError(f"invalid UUID field: {name}") from exc
        if parsed != value:
            raise ValueError(f"non-canonical UUID field: {name}")
    for name in POSITIVE_DECIMALS:
        if not _is_canonical_decimal(fields[name], positive=True):
            raise ValueError(f"non-canonical positive decimal: {name}")
    if not _is_canonical_decimal(fields["epoch"], positive=False):
        raise ValueError("non-canonical epoch")
    if fields["direction"] not in ("a_to_b", "b_to_a"):
        raise ValueError("invalid direction")
    for name in DIGEST_FIELDS:
        value = fields[name]
        if (len(value) != 64) or any(ch not in "0123456789abcdef" for ch in value):
            raise ValueError(f"invalid digest field: {name}")
    result = bytearray(DOMAIN)
    for name in FIELD_ORDER:
        encoded = fields[name].encode("ascii")
        result += len(encoded).to_bytes(4, "big", signed=False)
        result += encoded
    return bytes(result)


class AuthorityClaimGoldenVectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))

    def test_fixture_schema_and_explicit_claim_ceiling(self):
        fixture = self.fixture
        self.assertEqual(fixture["schema"], "ncf.holochain-authority-claim-vectors")
        self.assertEqual(fixture["schema_version"], "1.0.0")
        self.assertEqual(fixture["profile"], "NCF-AUTHORITY-CLAIM-V1")
        self.assertIn("content binding only", fixture["claim_ceiling"])
        self.assertEqual(fixture["encoding"]["domain_separator_utf8"], "NCF-AUTHORITY-CLAIM-V1\\0")
        self.assertEqual(tuple(fixture["encoding"]["field_order"]), FIELD_ORDER)

    def test_all_golden_vectors_match_preimage_length_and_sha256(self):
        vectors = self.fixture["vectors"]
        self.assertEqual(len(vectors), 2)
        names = {vector["name"] for vector in vectors}
        self.assertEqual(names, {"basic-unit-claim", "integer-above-js-safe-range"})
        for vector in vectors:
            with self.subTest(vector=vector["name"]):
                preimage = canonical_preimage(vector["fields"])
                self.assertEqual(len(preimage), vector["preimage_length_bytes"])
                digest = hashlib.sha256(preimage).hexdigest()
                self.assertEqual(digest, vector["preimage_sha256"])
                self.assertEqual(digest, vector["expected_claim_digest_sha256"])

    def test_large_integer_is_a_string_and_preserves_exact_value(self):
        vector = next(v for v in self.fixture["vectors"] if v["name"] == "integer-above-js-safe-range")
        amount = vector["fields"]["amount_units"]
        self.assertIsInstance(amount, str)
        self.assertEqual(amount, "9007199254740993")
        self.assertEqual(int(amount), 9007199254740993)

    def test_field_order_is_security_relevant(self):
        fields = self.fixture["vectors"][0]["fields"]
        reordered = dict(reversed(list(fields.items())))
        with self.assertRaisesRegex(ValueError, "canonical field order"):
            canonical_preimage(reordered)

    def test_numeric_json_values_are_rejected(self):
        fields = copy.deepcopy(self.fixture["vectors"][0]["fields"])
        fields["amount_units"] = 42
        with self.assertRaisesRegex(ValueError, "all fields must be strings"):
            canonical_preimage(fields)

    def test_leading_zero_amount_or_epoch_is_rejected(self):
        for field, value in (("amount_units", "042"), ("epoch", "07"), ("profile_revision", "01")):
            with self.subTest(field=field):
                fields = copy.deepcopy(self.fixture["vectors"][0]["fields"])
                fields[field] = value
                with self.assertRaises(ValueError):
                    canonical_preimage(fields)

    def test_uuid_must_be_lowercase_canonical(self):
        fields = copy.deepcopy(self.fixture["vectors"][0]["fields"])
        fields["agreement_id"] = fields["agreement_id"].upper()
        with self.assertRaisesRegex(ValueError, "non-canonical UUID"):
            canonical_preimage(fields)

    def test_digest_must_be_64_lowercase_hex_characters(self):
        for bad in ("A" * 64, "a" * 63, "g" * 64):
            with self.subTest(bad=bad[:4]):
                fields = copy.deepcopy(self.fixture["vectors"][0]["fields"])
                fields["sql_receipt_digest"] = bad
                with self.assertRaisesRegex(ValueError, "invalid digest field"):
                    canonical_preimage(fields)

    def test_unicode_and_embedded_nul_are_rejected(self):
        for bad in ("café", "community\x00a"):
            fields = copy.deepcopy(self.fixture["vectors"][0]["fields"])
            fields["community_id"] = bad
            with self.subTest(value=repr(bad)):
                with self.assertRaisesRegex(ValueError, "NUL-free ASCII"):
                    canonical_preimage(fields)

    def test_direction_and_schema_version_fail_closed(self):
        for field, bad in (("direction", "A_TO_B"), ("schema_version", "2")):
            fields = copy.deepcopy(self.fixture["vectors"][0]["fields"])
            fields[field] = bad
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    canonical_preimage(fields)

    def test_domain_separator_is_part_of_the_hashed_preimage(self):
        vector = self.fixture["vectors"][0]
        preimage = canonical_preimage(vector["fields"])
        self.assertTrue(preimage.startswith(DOMAIN))
        alternate = b"NCF-AUTHORITY-CLAIM-V2\x00" + preimage[len(DOMAIN):]
        self.assertNotEqual(hashlib.sha256(preimage).digest(), hashlib.sha256(alternate).digest())


if __name__ == "__main__":
    unittest.main()
