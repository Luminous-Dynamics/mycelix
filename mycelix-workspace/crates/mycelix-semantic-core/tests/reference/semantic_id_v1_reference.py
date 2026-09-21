#!/usr/bin/env python3
"""Independent reference oracle for MYC-SEM-001D0 SemanticIdV1.

This script intentionally has no dependency on Rust or mycelix-semantic-core.
It validates the frozen JSON corpus and independently reconstructs the repaired
MYC-SEM-001C-r2 compatibility commitments.

Passing this oracle establishes only agreement with the draft identifier
syntax/corpus and commitment byte specification. It establishes no authority,
trust, external URI/DID validity, or semantic equivalence.
"""

from __future__ import annotations

import hashlib
import json
import struct
import sys
from pathlib import Path

PROFILE = "mycelix-semantic-id/ascii-protocol-token-v1"
VERSION = 1
MAX_BYTES = 256

UNRESERVED = frozenset(
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-._~"
)
GEN_DELIMS = frozenset(":/?#[]@")
SUB_DELIMS = frozenset("!$&'()*+,;=")
RESERVED = GEN_DELIMS | SUB_DELIMS
ALLOWED_LITERAL_ASCII = UNRESERVED | RESERVED
UPPER_HEX = frozenset("0123456789ABCDEF")

EXPECTED_ENVIRONMENT = (
    "283f04d533916528a7054f9afe8958526cd3fdd94e29d9e990828af18dd12343"
)
EXPECTED_SUBJECT = (
    "5a88314b454bf23478c91af6adeff8a03bef0b8d4bd2f0c031b27321a50eb323"
)


def _validate_percent_run(value: str, start: int) -> tuple[bool, int]:
    """Validate one maximal contiguous run of uppercase percent triplets."""
    decoded = bytearray()
    index = start

    while index < len(value) and value[index] == "%":
        if index + 2 >= len(value):
            return False, index
        high = value[index + 1]
        low = value[index + 2]
        if high not in UPPER_HEX or low not in UPPER_HEX:
            return False, index

        byte = int(high + low, 16)

        if byte < 0x80:
            char = chr(byte)

            # RFC 3986 unreserved bytes have one canonical literal spelling.
            if char in UNRESERVED:
                return False, index

            # Encoded ASCII is admitted only for RFC 3986 reserved bytes or
            # literal percent data (%25). This rejects controls, SP, DEL, and
            # unsafe ASCII such as backslash, quotes, and angle brackets.
            if char not in RESERVED and byte != 0x25:
                return False, index

        decoded.append(byte)
        index += 3

    # Percent syntax is textual identity material, not arbitrary binary data.
    try:
        bytes(decoded).decode("utf-8")
    except UnicodeDecodeError:
        return False, start

    return True, index


def is_semantic_id_v1(value: str) -> bool:
    """Validate only the frozen Mycelix protocol-token syntax."""
    if not isinstance(value, str):
        return False

    encoded = value.encode("utf-8")
    if not (1 <= len(encoded) <= MAX_BYTES):
        return False

    # Stored protocol identifiers are byte-obvious ASCII. Internationalized
    # source identifiers must be transformed by a qualified adapter/profile.
    if any(ord(char) > 0x7F for char in value):
        return False

    index = 0
    while index < len(value):
        char = value[index]

        if char == "%":
            valid, next_index = _validate_percent_run(value, index)
            if not valid:
                return False
            index = next_index
            continue

        if char not in ALLOWED_LITERAL_ASCII:
            return False

        index += 1

    return True


def _text(value: str) -> bytes:
    raw = value.encode("utf-8")
    return struct.pack(">I", len(raw)) + raw


def _profile(profile_id: str, revision: int, digest_byte: int) -> bytes:
    return _text(profile_id) + struct.pack(">Q", revision) + bytes([digest_byte]) * 32


def reconstruct_001c_r2_vectors() -> tuple[bytes, str, bytes, str]:
    """Independently reconstruct the repaired 001C-r2 test vectors."""
    environment_preimage = (
        b"MYCELIX_SEMANTIC_ENVIRONMENT_V1\0"
        + struct.pack(">H", 1)
        + _profile("schema/base", 1, 1)
        + _profile("interpretation/base", 2, 2)
        + _profile("identity/base", 3, 3)
        + _profile("authority/base", 4, 4)
        + _profile("temporal/base", 5, 5)
        + _profile("canonical/domain-v1", 1, 6)
    )
    environment = hashlib.sha256(environment_preimage).hexdigest()

    subject_preimage = (
        b"MYCELIX_SEMANTIC_SUBJECT_V1\0"
        + struct.pack(">H", 1)
        + struct.pack(">H", 1)
        + bytes.fromhex(environment)
        + _text("personal")
        + _profile("schema/base", 1, 1)
        + _text("did:mycelix:test/profile")
    )
    subject = hashlib.sha256(subject_preimage).hexdigest()

    return environment_preimage, environment, subject_preimage, subject


def main() -> int:
    fixture_path = Path(__file__).resolve().parents[1] / "fixtures" / "semantic_id_v1.json"
    corpus = json.loads(fixture_path.read_text(encoding="utf-8"))

    assert corpus["profile"] == PROFILE
    assert corpus["version"] == VERSION
    assert corpus["max_bytes"] == MAX_BYTES
    assert corpus["percent_hex_policy"] == "uppercase-only"
    assert corpus["percent_decoded_ascii_policy"] == "reserved-or-percent-only"
    assert corpus["percent_decoded_non_ascii_policy"] == "valid-utf8"

    for value in corpus["valid"]:
        assert is_semantic_id_v1(value), f"valid corpus value rejected: {value!r}"

    for entry in corpus["invalid"]:
        value = entry["value"]
        assert not is_semantic_id_v1(value), (
            f"invalid corpus value accepted: {value!r} ({entry['reason']})"
        )

    # Explicit syntax boundaries not represented by huge fixture strings.
    assert is_semantic_id_v1("a" * MAX_BYTES)
    assert not is_semantic_id_v1("a" * (MAX_BYTES + 1))

    # The kernel intentionally does not collapse external-scheme semantics.
    for left, right in corpus["non_equivalence_examples"]:
        assert is_semantic_id_v1(left)
        assert is_semantic_id_v1(right)
        assert left.encode("ascii") != right.encode("ascii")

    env_preimage, environment, subject_preimage, subject = reconstruct_001c_r2_vectors()
    assert len(env_preimage) == 387
    assert len(subject_preimage) == 159
    assert environment == EXPECTED_ENVIRONMENT
    assert subject == EXPECTED_SUBJECT
    assert corpus["compatibility_vectors"]["environment"] == environment
    assert corpus["compatibility_vectors"]["subject"] == subject

    receipt = {
        "profile": PROFILE,
        "version": VERSION,
        "valid_cases": len(corpus["valid"]),
        "invalid_cases": len(corpus["invalid"]),
        "non_equivalence_cases": len(corpus["non_equivalence_examples"]),
        "environment_preimage_bytes": len(env_preimage),
        "environment": environment,
        "subject_preimage_bytes": len(subject_preimage),
        "subject": subject,
        "status": "PASS",
        "authority_ceiling": (
            "syntax/corpus and deterministic commitment compatibility only; "
            "not URI/DID validity, equivalence, authenticity, trust, or authority"
        ),
    }
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
