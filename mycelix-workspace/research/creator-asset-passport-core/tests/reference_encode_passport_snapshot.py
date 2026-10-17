#!/usr/bin/env python3
"""Independent test-only encoder for a CSCP full passport snapshot v1 vector.

This reference uses only Python's standard library. It does not call or import
the Rust crate. This is conformance tooling only, not application/runtime code.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MAGIC = b"MYCELIX-CSCP-ASSET-PASSPORT\0"
HISTORY_MAGIC = b"MYCELIX-CSCP-GRANT-HISTORY\0"
VERSION = 1
FIXTURE = Path(__file__).parent / "fixtures" / "creator-asset-passport-v1.hex"


def _u16(value: int) -> bytes:
    return struct.pack(">H", value)


def _u32(value: int) -> bytes:
    return struct.pack(">I", value)


def _u64(value: int) -> bytes:
    return struct.pack(">Q", value)


def _string(value: str) -> bytes:
    raw = value.encode("utf-8")
    if len(raw) > 256:
        raise ValueError("reference vector field exceeds the v1 limit")
    return _u32(len(raw)) + raw


def encode_reference_snapshot() -> bytes:
    """Encode one work, one ISWC, one complete split, and empty grant history."""
    work = b"".join(
        (
            _string("work-a"),
            b"\x01",  # AssetKind::MusicalWork
            b"\x01",  # title present
            _string("Song A"),
            _u64(1),  # external identifier count
            b"\x01",  # ISWC identifier
            _string("T-034.524.680-1"),
        )
    )
    claim = b"".join(
        (
            _string("work-a"),
            _string("writer-a"),
            b"\x01",  # RightsPool::Composition
            b"\x01",  # share present
            _u16(10_000),
            b"\x02",  # ClaimState::Confirmed
            _string("split://writer-a"),
        )
    )
    # Empty but fully versioned inner history envelope.
    empty_history = HISTORY_MAGIC + _u16(VERSION) + _u64(0)
    return b"".join(
        (
            MAGIC,
            _u16(VERSION),
            _u64(1),  # asset count
            work,
            _u64(1),  # claim count
            claim,
            _u64(len(empty_history)),
            empty_history,
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="compare independent reference bytes with the committed fixture",
    )
    args = parser.parse_args()
    actual = encode_reference_snapshot().hex() + "\n"

    if not args.check:
        sys.stdout.write(actual)
        return 0

    expected = FIXTURE.read_text(encoding="ascii")
    if expected != actual:
        print(f"full passport snapshot v1 mismatch: {FIXTURE}", file=sys.stderr)
        print(f"expected: {expected.strip()}", file=sys.stderr)
        print(f"actual:   {actual.strip()}", file=sys.stderr)
        return 1

    print(
        "full passport snapshot v1 reference vector passed "
        f"({len(encode_reference_snapshot())} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
