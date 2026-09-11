#!/usr/bin/env python3
"""Independent stdlib implementation of Mycelix accounting-wire v1.

This module intentionally does not import or invoke the TypeScript implementation.
Python ``int`` corresponds to a wire ``integer`` / JavaScript ``bigint`` and Python
``float`` corresponds to a wire ``number`` / JavaScript IEEE-754 binary64 value.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
from typing import Any

WIRE_FORMAT = "mycelix-accounting-wire"
WIRE_VERSION = 1
WIRE_DIGEST_DOMAIN = b"mycelix-accounting-wire-v1\0"
CANONICAL_INTEGER = re.compile(r"-?(0|[1-9][0-9]*)\Z")
BINARY64_HEX = re.compile(r"[0-9a-f]{16}\Z")


class WireError(ValueError):
    """Raised when accounting-wire data is not canonical."""


def _scalar_string(label: str, value: str) -> str:
    for character in value:
        code_point = ord(character)
        if 0xD800 <= code_point <= 0xDFFF:
            raise WireError(f"{label} must not contain lone UTF-16 surrogates")
    return value


def _utf8_key(value: str) -> bytes:
    return _scalar_string("accounting wire object key", value).encode("utf-8", errors="strict")


def _binary64_hex(value: float) -> str:
    if not math.isfinite(value):
        raise WireError("accounting wire numbers must be finite")
    return struct.pack(">d", value).hex()


def _decode_binary64_hex(value: Any) -> float:
    if not isinstance(value, str) or BINARY64_HEX.fullmatch(value) is None:
        raise WireError("accounting wire number must be canonical IEEE-754 binary64 hex")
    decoded = struct.unpack(">d", bytes.fromhex(value))[0]
    if not math.isfinite(decoded) or _binary64_hex(decoded) != value:
        raise WireError("accounting wire number must be canonical finite IEEE-754 binary64 hex")
    return decoded


def encode_node(value: Any) -> list[Any]:
    if value is None:
        return ["null"]
    if isinstance(value, str):
        return ["string", _scalar_string("accounting wire string", value)]
    if isinstance(value, bool):
        return ["boolean", value]
    if isinstance(value, int):
        return ["integer", str(value)]
    if isinstance(value, float):
        return ["number", _binary64_hex(value)]
    if isinstance(value, (list, tuple)):
        return ["array", [encode_node(item) for item in value]]
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise WireError("accounting wire object keys must be strings")
        keys = sorted((_scalar_string("accounting wire object key", key) for key in value), key=_utf8_key)
        return ["object", [[key, encode_node(value[key])] for key in keys]]
    raise WireError(f"unsupported accounting wire value: {type(value).__name__}")


def _canonical_integer(value: Any) -> int:
    if not isinstance(value, str) or CANONICAL_INTEGER.fullmatch(value) is None or value == "-0":
        raise WireError("accounting wire integer must be canonical decimal text")
    decoded = int(value, 10)
    if str(decoded) != value:
        raise WireError("accounting wire integer must be canonical decimal text")
    return decoded


def decode_node(node: Any) -> Any:
    if not isinstance(node, list) or not node or not isinstance(node[0], str):
        raise WireError("accounting wire node must be a tagged array")
    tag = node[0]
    if tag == "null":
        if len(node) != 1:
            raise WireError("accounting wire null node has invalid arity")
        return None
    if tag == "string":
        if len(node) != 2 or not isinstance(node[1], str):
            raise WireError("accounting wire string node is invalid")
        return _scalar_string("accounting wire string", node[1])
    if tag == "boolean":
        if len(node) != 2 or type(node[1]) is not bool:
            raise WireError("accounting wire boolean node is invalid")
        return node[1]
    if tag == "integer":
        if len(node) != 2:
            raise WireError("accounting wire integer node has invalid arity")
        return _canonical_integer(node[1])
    if tag == "number":
        if len(node) != 2:
            raise WireError("accounting wire number node has invalid arity")
        return _decode_binary64_hex(node[1])
    if tag == "array":
        if len(node) != 2 or not isinstance(node[1], list):
            raise WireError("accounting wire array node is invalid")
        return [decode_node(item) for item in node[1]]
    if tag == "object":
        if len(node) != 2 or not isinstance(node[1], list):
            raise WireError("accounting wire object node is invalid")
        decoded: dict[str, Any] = {}
        previous: bytes | None = None
        for entry in node[1]:
            if not isinstance(entry, list) or len(entry) != 2 or not isinstance(entry[0], str):
                raise WireError("accounting wire object entry must contain string key and value node")
            key = _scalar_string("accounting wire object key", entry[0])
            key_bytes = key.encode("utf-8", errors="strict")
            if previous is not None and key_bytes <= previous:
                raise WireError("accounting wire object keys must be strictly sorted and unique by UTF-8 bytes")
            previous = key_bytes
            decoded[key] = decode_node(entry[1])
        return decoded
    raise WireError(f"unsupported accounting wire node tag: {tag}")


def _json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def create_envelope(value: Any) -> dict[str, Any]:
    return {"format": WIRE_FORMAT, "version": WIRE_VERSION, "value": encode_node(value)}


def serialize_wire(value: Any) -> str:
    return _json_text(create_envelope(value))


def _reject_json_constant(value: str) -> None:
    raise WireError(f"accounting wire JSON forbids non-finite constant: {value}")


def deserialize_wire(text: str) -> Any:
    if not text or text != text.strip():
        raise WireError("accounting wire text must be canonical JSON without surrounding whitespace")
    try:
        parsed = json.loads(text, parse_constant=_reject_json_constant)
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise WireError("accounting wire text must be valid JSON") from exc
    if not isinstance(parsed, dict):
        raise WireError("accounting wire envelope must be an object")
    if list(parsed.keys()) != ["format", "version", "value"]:
        raise WireError("accounting wire envelope fields must be exactly format, version, value in canonical order")
    if parsed["format"] != WIRE_FORMAT or type(parsed["version"]) is not int or parsed["version"] != WIRE_VERSION:
        raise WireError("unsupported accounting wire format/version")
    decoded = decode_node(parsed["value"])
    if serialize_wire(decoded) != text:
        raise WireError("accounting wire text is not in canonical serialized form")
    return decoded


def wire_digest_from_text(text: str) -> str:
    deserialize_wire(text)
    return hashlib.sha256(WIRE_DIGEST_DOMAIN + text.encode("utf-8", errors="strict")).hexdigest()


def wire_digest(value: Any) -> str:
    return wire_digest_from_text(serialize_wire(value))


GOLDEN_VECTORS: tuple[tuple[str, Any, str, str], ...] = (
    (
        "null",
        None,
        '{"format":"mycelix-accounting-wire","version":1,"value":["null"]}',
        "a0b055b121a7e855f6eb22fec26eec04b847653c9eed355f6f9bbb2c7b3a0c26",
    ),
    (
        "arbitrary precision integer",
        123456789012345678901234567890,
        '{"format":"mycelix-accounting-wire","version":1,"value":["integer","123456789012345678901234567890"]}',
        "df6a69677ee7d5d063ba77f773859806ab97a4507f85a6ae710311aa8fd31e71",
    ),
    (
        "negative zero binary64",
        -0.0,
        '{"format":"mycelix-accounting-wire","version":1,"value":["number","8000000000000000"]}',
        "71f20fe7a02b0319cfbfa4f569e148863db49dc6cbd8b195ecf87616d1bc9a9a",
    ),
    (
        "one-and-a-half binary64",
        1.5,
        '{"format":"mycelix-accounting-wire","version":1,"value":["number","3ff8000000000000"]}',
        "56c9bfcd0e7109fdca8031f0274e278d2eba02cf6a795e6a89007553951e5894",
    ),
    (
        "sorted unicode object",
        {"z": 3, "a": "é"},
        '{"format":"mycelix-accounting-wire","version":1,"value":["object",[["a",["string","é"]],["z",["integer","3"]]]]}',
        "52cee7923ac978c5bdd8c0d1d1b25b6c324b4a2845ae96615381793d4d0c7909",
    ),
    (
        "UTF-8 key order differs from JavaScript UTF-16 sort",
        {"\uE000": "bmp", "𐀀": "astral"},
        '{"format":"mycelix-accounting-wire","version":1,"value":["object",[["",["string","bmp"]],["𐀀",["string","astral"]]]]}',
        "dba4eb609b3466d6b42b49fa360390559fc755955012855453a31c7c365c556f",
    ),
)


def _expect_rejected(action: Any, label: str) -> None:
    try:
        action()
    except (WireError, UnicodeError, ValueError):
        return
    raise AssertionError(f"expected rejection: {label}")


def run_conformance() -> None:
    for name, value, expected_text, expected_digest in GOLDEN_VECTORS:
        actual_text = serialize_wire(value)
        assert actual_text == expected_text, f"{name}: canonical bytes mismatch"
        assert wire_digest_from_text(expected_text) == expected_digest, f"{name}: digest mismatch"
        assert wire_digest(value) == expected_digest, f"{name}: value digest mismatch"
        decoded = deserialize_wire(expected_text)
        assert serialize_wire(decoded) == expected_text, f"{name}: round-trip mismatch"

    # This is the exact ordering difference that caught the original JS UTF-16-sort bug.
    assert "𐀀".encode("utf-16-be") < "\uE000".encode("utf-16-be")
    assert "\uE000".encode("utf-8") < "𐀀".encode("utf-8")

    _expect_rejected(lambda: deserialize_wire(
        '{"format":"mycelix-accounting-wire","version":1,"value":["integer","01"]}'
    ), "noncanonical integer")
    _expect_rejected(lambda: deserialize_wire(
        '{"format":"mycelix-accounting-wire","version":1,"value":["number","3FF0000000000000"]}'
    ), "uppercase binary64")
    _expect_rejected(lambda: deserialize_wire(
        '{"format":"mycelix-accounting-wire","version":1,"value":["number","7ff0000000000000"]}'
    ), "positive infinity")
    _expect_rejected(lambda: deserialize_wire(
        '{"format":"mycelix-accounting-wire","version":1,"value":["object",[["𐀀",["string","astral"]],["",["string","bmp"]]]]}'
    ), "UTF-16 key ordering")
    _expect_rejected(lambda: serialize_wire("\ud800"), "lone high surrogate")
    _expect_rejected(lambda: serialize_wire({"\udc00": "value"}), "lone low-surrogate key")
    _expect_rejected(lambda: deserialize_wire(
        '{"format":"mycelix-accounting-wire","version":1,"value":["string","\\ud800"]}'
    ), "decoded lone surrogate")
    canonical = serialize_wire({"value": 1})
    _expect_rejected(lambda: deserialize_wire(" " + canonical), "surrounding whitespace")
    _expect_rejected(lambda: deserialize_wire(canonical.replace(",", ", ", 1)), "alternate JSON whitespace")
    _expect_rejected(lambda: serialize_wire(float("inf")), "non-finite number")


def main() -> int:
    parser = argparse.ArgumentParser(description="Independent Mycelix accounting-wire v1 conformance verifier")
    parser.add_argument("--verify", type=Path, help="verify one canonical accounting-wire file and print its digest")
    args = parser.parse_args()
    if args.verify is not None:
        text = args.verify.read_text(encoding="utf-8")
        print(wire_digest_from_text(text))
        return 0
    run_conformance()
    print("accounting wire Python stdlib conformance: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
