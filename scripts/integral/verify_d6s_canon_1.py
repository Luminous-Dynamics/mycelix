#!/usr/bin/env python3
"""Independent D6S-CANON-1 verifier.

This intentionally does not import the Rust reference implementation.
It validates the frozen golden-vector contract using Python's standard
library and rejects ambiguous/non-integral values.

The manifest binds the verifier run to an exact corpus byte hash and
expected vector cardinalities, so "passed" cannot silently drift to a
different corpus.
"""

import hashlib
import json
import sys
from pathlib import Path


DOMAIN = b"MYCELIX-INTEGRAL-D6S-RECEIPT-V1\0"
PROFILE = "D6S-CANON-1"


def reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate object property: {key!r}")
        result[key] = value
    return result


def parse_integer(token):
    if token == "-0":
        raise ValueError("negative zero is rejected at the parser boundary")
    value = int(token)
    if value < -(2**63) or value > 2**64 - 1:
        raise ValueError("integer outside the frozen i64/u64 domain")
    return value


def load_json(text):
    return json.loads(
        text,
        object_pairs_hook=reject_duplicate_keys,
        parse_int=parse_integer,
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
    root = Path(__file__).parents[2]
    vector_path = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else root / "docs" / "integral" / "d6s-canon-1-golden-vectors.json"
    )
    manifest_path = root / "docs" / "integral" / "d6s-canon-1-manifest.json"

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest["profile"] != PROFILE:
        raise SystemExit("manifest profile mismatch")
    if manifest["canonicalization_version"] != PROFILE:
        raise SystemExit("manifest canonicalization version mismatch")
    if manifest["hash_domain"].encode("utf-8") != DOMAIN:
        raise SystemExit("manifest hash domain mismatch")
    if manifest["corpus_path"] != vector_path.relative_to(root).as_posix():
        raise SystemExit("manifest corpus path mismatch")
    if manifest["independent_verifier"] != "scripts/integral/verify_d6s_canon_1.py":
        raise SystemExit("manifest verifier path mismatch")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        raise SystemExit("manifest claim ceiling mismatch")

    corpus_bytes = vector_path.read_bytes()
    actual_corpus_sha256 = hashlib.sha256(corpus_bytes).hexdigest()
    if actual_corpus_sha256 != manifest["corpus_sha256"]:
        raise SystemExit(
            "corpus identity mismatch\n"
            f"expected: {manifest['corpus_sha256']}\n"
            f"actual:   {actual_corpus_sha256}"
        )

    corpus = load_json(corpus_bytes.decode("utf-8"))
    if corpus["profile"] != PROFILE:
        raise SystemExit("unexpected canonicalization profile")
    if corpus["hash_domain"].encode("utf-8") != DOMAIN:
        raise SystemExit("unexpected hash domain")

    cases = corpus["cases"]
    rejections = corpus.get("rejections", [])
    if len(cases) != manifest["expected_vector_count"]:
        raise SystemExit(
            f"vector count mismatch: expected {manifest['expected_vector_count']}, "
            f"actual {len(cases)}"
        )
    if len(rejections) != manifest["expected_rejection_count"]:
        raise SystemExit(
            f"rejection count mismatch: expected {manifest['expected_rejection_count']}, "
            f"actual {len(rejections)}"
        )

    for case in cases:
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

    for case in rejections:
        try:
            value = load_json(case["json"])
            canonical_json(value)
        except (TypeError, ValueError):
            continue
        raise SystemExit(
            f"{case['name']}: malformed input was unexpectedly accepted"
        )

    print(
        f"verified {len(cases)} D6S-CANON-1 vectors and "
        f"{len(rejections)} rejection vectors"
    )
    print(f"corpus_sha256={actual_corpus_sha256}")
    print("claim_ceiling=ReferenceModelOnly")


if __name__ == "__main__":
    main()
