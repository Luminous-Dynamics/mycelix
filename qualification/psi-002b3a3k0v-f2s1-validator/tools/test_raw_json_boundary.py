#!/usr/bin/env python3
"""Focused tests for duplicate-member rejection and raw-byte identity."""
from __future__ import annotations

import hashlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_raw_json import parse_raw_json


def must_reject(raw: bytes, fragment: str) -> None:
    try:
        parse_raw_json(raw)
    except ValueError as exc:
        assert fragment in str(exc), (fragment, str(exc))
    else:
        raise AssertionError(f"accepted invalid/ambiguous JSON: {raw!r}")


def main() -> int:
    # Duplicate names are rejected at every object depth, regardless of values.
    must_reject(b'{"a":1,"a":1}', "duplicate JSON object member")
    must_reject(b'{"outer":{"x":1,"x":2}}', "duplicate JSON object member")
    must_reject(b'{"a":null,"a":{}}', "duplicate JSON object member")
    # JSON member names are compared after unescaping.
    must_reject(b'{"a":1,"\\u0061":2}', "duplicate JSON object member")

    # Absent, null, empty scalar/container, and distinct numeric spellings parse
    # distinctly here; this boundary does not normalize them.
    cases = [
        (b'{}', {}),
        (b'{"v":null}', {"v": None}),
        (b'{"v":""}', {"v": ""}),
        (b'{"v":[]}', {"v": []}),
        (b'{"v":{}}', {"v": {}}),
        (b'{"v":1}', {"v": 1}),
        (b'{"v":1.0}', {"v": 1.0}),
        (b'{"v":1e0}', {"v": 1.0}),
    ]
    for raw, expected in cases:
        digest, value = parse_raw_json(raw)
        assert value == expected
        assert digest == hashlib.sha256(raw).hexdigest()

    # Whitespace and member ordering may change raw identity; no semantic
    # canonicalization is attempted by this gate.
    a = b'{"a":1,"b":2}'
    b = b'{ "b": 2, "a": 1 }'
    da, va = parse_raw_json(a)
    db, vb = parse_raw_json(b)
    assert va == vb and da != db

    must_reject(b'{"a":', "invalid or ambiguous JSON")
    must_reject(b'{"a":1} trailing', "invalid or ambiguous JSON")
    must_reject(b'{"a":1,', "invalid or ambiguous JSON")
    must_reject(b'\xef\xbb\xbf{"a":1}', "BOM")
    must_reject(b'{"a":1}\xff', "valid UTF-8")

    print("RAW JSON BOUNDARY TESTS: PASS (duplicate rejection and raw-byte identity only)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
