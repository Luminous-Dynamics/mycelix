#!/usr/bin/env python3
"""Independent D6S-CANON-1 verifier.

This intentionally does not import the Rust reference implementation.
It validates the frozen golden-vector contract using Python's standard
library and rejects ambiguous/non-integral values.
"""

import hashlib
import json
import sys
from pathlib import Path


DOMAIN = b"MYCELIX-INTEGRAL-D6S-RECEIPT-V1\0"


def reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate object property: {key!r}")
        result[key] = value
    return result


def load_json(text):
    return json.loads(
        text,
        object_pairs_hook=reject_duplicate_keys,
        parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError(f"non-finite number: {value}")
        ),
    )


def validate_string(value):
    # Python can represent lone surrogates; D6S-CANON-1 cannot.
    for char in value:
        codepoint = ord(char)
        if 0xD800 <= codepoint <= 0xDFFF:
            raise ValueError("lone surrogate is not valid D6S-CANON-1 Unicode")


def utf16_key(value):
    validate_string(value)
    return value.encode("utf-16-be")


def canonical_json(value):
    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    if isinstance(value, float):
        raise ValueError("D6S-CANON-1 rejects non-integral numeric values")
    if isinstance(value, str):
        validate_string(value)
        return json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
        )
    if isinstance(value, list):
        return "[" + ",".join(canonical_json(item) for item in value) + "]"
    if isinstance(value, dict):
        keys = sorted(value, key=utf16_key)
        return "{" + ",".join(
            canonical_json(key) + ":" + canonical_json(value[key])
            for key in keys
        ) + "}"
    raise TypeError(f"unsupported JSON value: {type(value)!r}")


def commitment(canonical_bytes):
    return hashlib.sha256(DOMAIN + canonical_bytes).hexdigest()


def main():
    vector_path = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).parents[2]
        / "docs"
        / "integral"
        / "d6s-canon-1-golden-vectors.json"
    )

    corpus = load_json(vector_path.read_text(encoding="utf-8"))
    if corpus["profile"] != "D6S-CANON-1":
        raise SystemExit("unexpected canonicalization profile")
    if corpus["hash_domain"].encode("utf-8").decode("unicode_escape").encode(
        "utf-8"
    ) != DOMAIN:
        raise SystemExit("unexpected hash domain")

    for case in corpus["cases"]:
        actual = canonical_json(case["value"])
        if actual != case["canonical"]:
            raise SystemExit(
                f"{case['name']}: canonical bytes mismatch\n"
                f"expected: {case['canonical']!r}\n"
                f"actual:   {actual!r}"
            )
        actual_commitment = commitment(actual.encode("utf-8"))
        if actual_commitment != case["commitment"]:
            raise SystemExit(
                f"{case['name']}: commitment mismatch\n"
                f"expected: {case['commitment']}\n"
                f"actual:   {actual_commitment}"
            )

    print(f"verified {len(corpus['cases'])} D6S-CANON-1 vectors")


if __name__ == "__main__":
    main()
