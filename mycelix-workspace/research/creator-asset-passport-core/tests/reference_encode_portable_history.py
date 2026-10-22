#!/usr/bin/env python3
"""Test-only independent encoder for the CSCP portable grant-history v1 vector.

This is a conformance fixture tool, not application/runtime code. It uses only
the Python standard library and deliberately encodes the binary field layout
without invoking the Rust implementation or parsing Rust-produced bytes.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MAGIC = b"MYCELIX-CSCP-GRANT-HISTORY\0"
VERSION = 1
FIXTURES = {
    "creator-grant-history-v1.hex",
    "creator-grant-history-v1-unicode-no-expiry.hex",
}
FIXTURE_DIR = Path(__file__).parent / "fixtures"


def _u16(value: int) -> bytes:
    return struct.pack(">H", value)


def _u64(value: int) -> bytes:
    return struct.pack(">Q", value)


def _string(value: str) -> bytes:
    raw = value.encode("utf-8")
    if len(raw) > 256:
        raise ValueError("fixture contains an oversized string")
    return struct.pack(">I", len(raw)) + raw


def encode_reference_vector() -> bytes:
    """Build two events from the v1 wire contract in explicit field order."""
    issue = b"".join(
        (
            _string("event-issue"),
            _u64(5),
            _string("creator"),
            _string("evidence://issue"),
            b"\x01",  # issued event
            _string("grant-a"),
            _string("recording-a"),
            _string("distributor"),
            b"\x03",  # UseAction::Distribute
            _string("release-a"),
            _string("creator"),
            _string("consent://grant-a"),
            _u64(10),
            b"\x01",  # expiry present
            _u64(100),
        )
    )
    revoke = b"".join(
        (
            _string("event-revoke"),
            _u64(50),
            _string("creator"),
            _string("evidence://revoke"),
            b"\x02",  # revoked event
            _string("grant-a"),
        )
    )
    return MAGIC + _u16(VERSION) + _u64(2) + issue + revoke


def encode_unicode_no_expiry_vector() -> bytes:
    """Exercise UTF-8 byte lengths, action tag 7, and an absent expiry."""
    issue = b"".join(
        (
            _string("event-unicode"),
            _u64(0),
            _string("artist-Δ"),
            _string("evidence://approved"),
            b"\x01",  # issued event
            _string("grant-unicode"),
            _string("recording-δ"),
            _string("voice-service"),
            b"\x07",  # UseAction::SynthesizeVoiceOrLikeness
            _string("one-off-clone-demo"),
            _string("artist-Δ"),
            _string("consent://clip-specific"),
            _u64(1),
            b"\x00",  # no expiry
        )
    )
    return MAGIC + _u16(VERSION) + _u64(1) + issue


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="compare reference encoders against committed hex fixtures",
    )
    args = parser.parse_args()
    vectors = {
        "creator-grant-history-v1.hex": encode_reference_vector(),
        "creator-grant-history-v1-unicode-no-expiry.hex": (
            encode_unicode_no_expiry_vector()
        ),
    }

    if not args.check:
        for filename, encoded in vectors.items():
            print(f"{filename}\n{encoded.hex()}")
        return 0

    for filename in sorted(FIXTURES):
        encoded_hex = vectors[filename].hex() + "\n"
        fixture = FIXTURE_DIR / filename
        expected = fixture.read_text(encoding="ascii")
        if expected != encoded_hex:
            print(f"portable-history v1 vector mismatch: {fixture}", file=sys.stderr)
            print(f"expected: {expected.strip()}", file=sys.stderr)
            print(f"actual:   {encoded_hex.strip()}", file=sys.stderr)
            return 1

    total_bytes = sum(len(value) for value in vectors.values())
    print(
        "portable-history v1 reference vectors passed "
        f"({len(vectors)} vectors, {total_bytes} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
