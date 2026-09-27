#!/usr/bin/env python3
"""Build implementation-neutral MYC-INT-006I candidate input."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

INPUT_SCHEMA_REF = "./myc-int-006i-i0-candidate-input.schema.json"
INPUT_ID = "myc-int-i0-candidate-input"
INPUT_VERSION = "1.0.0"
INPUT_PROFILE = "runtime-neutral-candidate-input-v1"


class NeutralInputError(ValueError):
    pass


def _walk_forbidden(value: Any, ctx: str = "$") -> None:
    forbidden = {
        "expected_disposition", "expected_result", "expected", "oracle",
        "reason_code", "assertion", "assertions", "predicate", "predicates",
    }
    if isinstance(value, dict):
        for key, child in value.items():
            if key in forbidden or key.startswith("expected_"):
                raise NeutralInputError(f"{ctx}: oracle-bearing key leaked: {key!r}")
            _walk_forbidden(child, f"{ctx}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _walk_forbidden(child, f"{ctx}[{index}]")


def build_candidate_input(corpus: dict[str, Any], stimulus: dict[str, Any], *, corpus_raw: bytes, stimulus_raw: bytes) -> dict[str, Any]:
    subjects: dict[str, Any] = {}
    for subject_id, subject in corpus["subjects"].items():
        subjects[subject_id] = {
            "kind": subject["kind"],
            "semantic_ref": {
                "namespace": subject["semantic_ref"]["namespace"],
                "name": subject["semantic_ref"]["name"],
                "version": subject["semantic_ref"]["version"],
            },
        }

    result = {
        "$schema": INPUT_SCHEMA_REF,
        "candidate_input_id": INPUT_ID,
        "candidate_input_version": INPUT_VERSION,
        "profile": INPUT_PROFILE,
        "corpus_id": corpus["corpus_id"],
        "corpus_version": corpus["corpus_version"],
        "stimulus_id": stimulus["stimulus_id"],
        "stimulus_version": stimulus["stimulus_version"],
        "source_commitments": {
            "corpus_sha256": hashlib.sha256(corpus_raw).hexdigest(),
            "stimulus_sha256": hashlib.sha256(stimulus_raw).hexdigest(),
        },
        "subjects": subjects,
        "cases": stimulus["cases"],
    }
    _walk_forbidden(result)
    return result


def canonical_bytes(document: dict[str, Any]) -> bytes:
    return (json.dumps(document, indent=2, sort_keys=True) + "\n").encode("utf-8")


def build_from_bytes(corpus_raw: bytes, schema_raw: bytes, stimulus_raw: bytes) -> dict[str, Any]:
    import validate_i0
    import validate_stimulus

    validate_i0.validate_bytes(corpus_raw, schema_raw=schema_raw)
    validate_stimulus.validate_bytes(stimulus_raw, corpus_raw)
    corpus = validate_i0.parse_strict_json(corpus_raw, label="corpus")
    stimulus = validate_stimulus.parse_strict_json(stimulus_raw, label="stimulus")
    return build_candidate_input(corpus, stimulus, corpus_raw=corpus_raw, stimulus_raw=stimulus_raw)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    here = Path(__file__).resolve().parent
    parser.add_argument("--corpus", type=Path, default=here / "myc-int-006c-water-i0-corpus.v1.json")
    parser.add_argument("--schema", type=Path, default=here / "myc-int-006c-water-i0-corpus.schema.json")
    parser.add_argument("--stimulus", type=Path, default=here / "myc-int-006e-water-i0-stimulus.v1.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        document = build_from_bytes(args.corpus.read_bytes(), args.schema.read_bytes(), args.stimulus.read_bytes())
    except (OSError, ValueError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    encoded = canonical_bytes(document)
    if args.output:
        args.output.write_bytes(encoded)
    else:
        sys.stdout.buffer.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
