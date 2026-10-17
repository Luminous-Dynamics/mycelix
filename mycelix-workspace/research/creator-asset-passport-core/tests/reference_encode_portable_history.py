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
FIXTURE = Path(__file__).parent / "fixtures" / "creator-grant-history-v1.hex"


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="compare the reference encoder against the committed hex fixture",
    )
    args = parser.parse_args()
    encoded_hex = encode_reference_vector().hex() + "\n"

    if not args.check:
        sys.stdout.write(encoded_hex)
        return 0

    expected = FIXTURE.read_text(encoding="ascii")
    if expected != encoded_hex:
        print(
            f"portable-history v1 vector mismatch: {FIXTURE}",
            file=sys.stderr,
        )
        print(f"expected: {expected.strip()}", file=sys.stderr)
        print(f"actual:   {encoded_hex.strip()}", file=sys.stderr)
        return 1

    print(
        f"portable-history v1 reference vector passed "
        f"({len(encode_reference_vector())} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
