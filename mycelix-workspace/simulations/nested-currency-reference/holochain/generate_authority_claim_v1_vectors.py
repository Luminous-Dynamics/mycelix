#!/usr/bin/env python3
"""Generate deterministic NCF-AUTHORITY-CLAIM-V1 research golden vectors.

This is a reference fixture generator, not production authorization logic.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import uuid

FIELD_ORDER = [
    "schema_version", "outbox_event_id", "obligation_id", "request_id",
    "canonical_work_digest", "principal_id", "community_id", "instrument_id",
    "profile_revision", "amount_units", "agreement_id", "direction",
    "reservation_id", "shard_id", "epoch", "verifier_version",
    "verifier_receipt_digest", "sql_receipt_digest", "authority_manifest_digest",
]
DOMAIN = b"NCF-AUTHORITY-CLAIM-V1\x00"


def canonical_preimage(fields: dict[str, str]) -> bytes:
    if list(fields) != FIELD_ORDER:
        raise ValueError("fields must appear in canonical field order")
    values = list(fields.values())
    if not all(isinstance(v, str) for v in values):
        raise ValueError("all fields must be strings")
    if not all(v.isascii() and "\x00" not in v for v in values):
        raise ValueError("fields must be NUL-free ASCII")
    for name in ("outbox_event_id", "request_id", "agreement_id", "reservation_id"):
        if str(uuid.UUID(fields[name])) != fields[name]:
            raise ValueError(f"non-canonical UUID: {name}")
    if fields["schema_version"] != "1":
        raise ValueError("schema version must be 1")
    if fields["direction"] not in ("a_to_b", "b_to_a"):
        raise ValueError("invalid direction")
    for name in ("obligation_id", "profile_revision", "amount_units"):
        value = fields[name]
        if not value.isdigit() or (len(value) > 1 and value[0] == "0") or int(value) < 1:
            raise ValueError(f"non-canonical positive decimal: {name}")
    value = fields["epoch"]
    if not value.isdigit() or (len(value) > 1 and value[0] == "0"):
        raise ValueError("non-canonical epoch")
    for name in ("canonical_work_digest", "verifier_receipt_digest", "sql_receipt_digest", "authority_manifest_digest"):
        value = fields[name]
        if len(value) != 64 or any(ch not in "0123456789abcdef" for ch in value):
            raise ValueError(f"{name} must be 64 lowercase hex characters")
    out = bytearray(DOMAIN)
    for name in FIELD_ORDER:
        value = fields[name].encode("ascii")
        out.extend(len(value).to_bytes(4, "big"))
        out.extend(value)
    return bytes(out)


def make_fields(index: int, amount: int, principal: str) -> dict[str, str]:
    return {
        "schema_version": "1",
        "outbox_event_id": f"20000000-0000-0000-0000-{index:012d}",
        "obligation_id": "17" if index == 10 else "922337203685477580",
        "request_id": f"20000000-0000-0000-0000-{index + 1:012d}",
        "canonical_work_digest": "a" * 64,
        "principal_id": principal,
        "community_id": "community-a",
        "instrument_id": "NCF-UNIT-01" if index == 10 else "NCF-ENERGY-001",
        "profile_revision": "1" if index == 10 else "12",
        "amount_units": str(amount),
        "agreement_id": "aa000000-0000-0000-0000-000000000010",
        "direction": "a_to_b",
        "reservation_id": "bb000000-0000-0000-0000-000000000010",
        "shard_id": "home-shard-0",
        "epoch": "7" if index == 10 else "42",
        "verifier_version": "service-proof-v1" if index == 10 else "symthaea-verifier-v2+build.17",
        "verifier_receipt_digest": "b" * 64,
        "sql_receipt_digest": "c" * 64,
        "authority_manifest_digest": "d" * 64,
    }


def make_fixture() -> dict:
    vectors = []
    for name, fields in [
        ("basic-unit-claim", make_fields(10, 42, "principal-a")),
        ("integer-above-js-safe-range", make_fields(11, 9007199254740993, "principal-923")),
    ]:
        raw = canonical_preimage(fields)
        digest = hashlib.sha256(raw).hexdigest()
        vectors.append({
            "name": name,
            "fields": fields,
            "preimage_length_bytes": len(raw),
            "preimage_sha256": digest,
            "expected_claim_digest_sha256": digest,
        })
    return {
        "schema": "ncf.holochain-authority-claim-vectors",
        "schema_version": "1.0.0",
        "profile": "NCF-AUTHORITY-CLAIM-V1",
        "status": "cross_language_golden_vectors_reference_only",
        "encoding": {
            "domain_separator_utf8": "NCF-AUTHORITY-CLAIM-V1\\0",
            "field_order": FIELD_ORDER,
            "value_encoding": "NUL-free ASCII bytes only",
            "field_framing": "4-byte unsigned big-endian length followed by exact field bytes",
            "hash": "SHA-256 over domain separator and ordered length-framed values",
            "numeric_fields": "canonical base-10 strings with no leading zeros; amounts are strings, never JSON numbers",
            "digest_fields": "64 lowercase hexadecimal ASCII characters; this envelope hashes the textual hex representation and does not itself imply a signature",
            "uuid_fields": "canonical lowercase hyphenated UUID text",
            "authority_manifest": "manifest digest is bound in this envelope; actual Holochain ActionHash is a separate typed reference validated against this digest and the authorized author",
        },
        "claim_ceiling": "Test vectors define content binding only. They do not sign a PostgreSQL commit or prove that a verifier's real-world work occurred. Holochain action authorship must match a governance-authorized AuthorityManifest; PostgreSQL/Holochain remain separate commit domains.",
        "vectors": vectors,
    }


if __name__ == "__main__":
    target = Path(__file__).with_name("ncf-authority-claim-v1-vectors.json")
    target.write_text(json.dumps(make_fixture(), indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {target}")
