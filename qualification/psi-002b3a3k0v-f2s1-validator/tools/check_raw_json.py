#!/usr/bin/env python3
"""Strict raw-JSON boundary: reject duplicate object names before schema use.

This module deliberately does not canonicalize JSON or claim schema validity.
It preserves the raw-byte digest separately from the decoded Python value.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path


class DuplicateMemberError(ValueError):
    """Raised when an object repeats a member name after JSON unescaping."""


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise DuplicateMemberError(f"duplicate JSON object member: {key!r}")
        result[key] = value
    return result


def parse_raw_json(raw: bytes):
    """Return (raw_sha256, value); reject invalid UTF-8, BOM, malformed JSON, duplicates."""
    digest = hashlib.sha256(raw).hexdigest()
    if raw.startswith(b"\\xef\\xbb\\xbf"):
        raise ValueError("UTF-8 BOM is not permitted")
    try:
        source = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ValueError(f"input is not valid UTF-8: {exc}") from exc
    try:
        value = json.loads(source, object_pairs_hook=_unique_object)
    except (json.JSONDecodeError, DuplicateMemberError) as exc:
        raise ValueError(f"invalid or ambiguous JSON: {exc}") from exc
    return digest, value


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} JSON_FILE", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    try:
        digest, _ = parse_raw_json(path.read_bytes())
    except (OSError, ValueError) as exc:
        print(f"RAW JSON BOUNDARY: FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"RAW JSON BOUNDARY: ACCEPT (duplicate-free syntax only; sha256={digest})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
