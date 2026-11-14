#!/usr/bin/env python3
"""Independent test-only encoder for the CSCP import-provenance v1 vector.

Uses only Python's standard library and hard-coded BLAKE3(empty) for the
fixture payload. Not production code and not an authenticity verifier.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MAGIC = b"MYCELIX-CSCP-IMPORT-PROVENANCE\0"
VERSION = 1
EMPTY_BLAKE3 = (
    "af1349b9f5f9a1a6a0404dea36dcc949"
    "9bcb25c9adc112b7cc9a93cae41f3262"
)
FIXTURE = Path(__file__).parent / "fixtures" / "creator-import-provenance-v1.hex"


def u16(value: int) -> bytes:
    return struct.pack(">H", value)


def u32(value: int) -> bytes:
    return struct.pack(">I", value)


def u64(value: int) -> bytes:
    return struct.pack(">Q", value)


def string(value: str) -> bytes:
    raw = value.encode("utf-8")
    if len(raw) > 4096:
        raise ValueError("fixture text exceeds provenance v1 limit")
    return u32(len(raw)) + raw


def optional_string(value: str | None) -> bytes:
    if value is None:
        return b"\x00"
    return b"\x01" + string(value)


def encode_reference_manifest() -> bytes:
    # Source document: record IDs are deliberately optional, not fabricated.
    source = b"".join(
        (
            string("test-source"),
            optional_string(None),
            string("rdr-n"),
            string("1.5"),
            string("test-parser"),
            string("0.1"),
            u64(1),
            optional_string(None),
            string("vault://artifact-1"),
            u64(0),
            string(EMPTY_BLAKE3),
        )
    )
    fields = b"".join(
        (
            string("/Report/Id"),
            b"\x01",
            string("/statement/id"),
            string("id-map"),
            string("1"),
            string("/Report/Period"),
            b"\x01",
            string("/statement/period"),
            string("period-map"),
            string("1"),
        )
    )
    return b"".join(
        (
            MAGIC,
            u16(VERSION),
            source,
            b"\x01",  # inventory complete as reported by parser
            b"\x01",  # imported without warnings
            string("00" * 32),
            optional_string("11" * 32),
            u64(2),
            fields,
            u64(0),  # diagnostics
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
    actual = encode_reference_manifest().hex() + "\n"
    if not args.check:
        sys.stdout.write(actual)
        return 0

    expected = FIXTURE.read_text(encoding="ascii")
    if expected != actual:
        print(f"import provenance v1 mismatch: {FIXTURE}", file=sys.stderr)
        print(f"expected: {expected.strip()}", file=sys.stderr)
        print(f"actual:   {actual.strip()}", file=sys.stderr)
        return 1

    print(
        "import provenance v1 reference vector passed "
        f"({len(encode_reference_manifest())} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
