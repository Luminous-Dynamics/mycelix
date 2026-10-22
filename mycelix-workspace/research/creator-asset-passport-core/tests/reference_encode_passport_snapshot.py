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
FIXTURE_DIR = Path(__file__).parent / "fixtures"


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


def encode_unicode_disputed_snapshot() -> bytes:
    """Exercise ISRC/other IDs, UTF-8, absent title/share, and a disputed claim."""
    asset = b"".join(
        (
            _string("recording-δ"),
            b"\x02",  # AssetKind::SoundRecording
            b"\x00",  # title absent
            _u64(3),  # ISRC plus two other identifiers
            b"\x02",  # ISRC
            _string("US-ABC-23-00001"),
            b"\x03",  # other identifier
            _string("a"),
            _string("z"),
            b"\x03",  # other identifier
            _string("a:"),
            _string("x"),
        )
    )
    claim = b"".join(
        (
            _string("recording-δ"),
            _string("performer-Δ"),
            b"\x02",  # RightsPool::MasterRecording
            b"\x00",  # share unknown
            b"\x03",  # ClaimState::Disputed
            _string("evidence://争議"),
        )
    )
    empty_history = HISTORY_MAGIC + _u16(VERSION) + _u64(0)
    return b"".join(
        (
            MAGIC,
            _u16(VERSION),
            _u64(1),  # asset count
            asset,
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
        help="compare independent reference bytes with committed fixtures",
    )
    args = parser.parse_args()
    vectors = {
        "creator-asset-passport-v1.hex": encode_reference_snapshot(),
        "creator-asset-passport-v1-unicode-disputed.hex": (
            encode_unicode_disputed_snapshot()
        ),
    }

    if not args.check:
        for filename, encoded in vectors.items():
            print(f"{filename}\n{encoded.hex()}")
        return 0

    for filename in sorted(vectors):
        fixture = FIXTURE_DIR / filename
        expected = fixture.read_text(encoding="ascii")
        actual = vectors[filename].hex() + "\n"
        if expected != actual:
            print(f"full passport snapshot v1 mismatch: {fixture}", file=sys.stderr)
            print(f"expected: {expected.strip()}", file=sys.stderr)
            print(f"actual:   {actual.strip()}", file=sys.stderr)
            return 1

    total_bytes = sum(len(value) for value in vectors.values())
    print(
        "full passport snapshot v1 reference vectors passed "
        f"({len(vectors)} vectors, {total_bytes} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
