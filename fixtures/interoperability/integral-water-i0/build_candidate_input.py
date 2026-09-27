#!/usr/bin/env python3
"""Build oracle-blind candidate input for MYC-INT-006F."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

INPUT_ID = "myc-int-006f-conventional-candidate-input"
INPUT_VERSION = "1.0.0"
INPUT_PROFILE = "oracle-blind-candidate-input-v1"

FORBIDDEN_KEYS = frozenset({
    "expected_disposition", "expected_result", "expected", "oracle",
    "reason_code", "assertion", "assertions", "predicate", "predicates",
})


class CandidateInputError(ValueError):
    pass


def _walk_forbidden(value: Any, ctx: str = "$") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in FORBIDDEN_KEYS or key.startswith("expected_"):
                raise CandidateInputError(f"{ctx}: oracle-bearing key leaked: {key!r}")
            _walk_forbidden(child, f"{ctx}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _walk_forbidden(child, f"{ctx}[{index}]")


def build_candidate_input(corpus: dict[str, Any], stimulus: dict[str, Any]) -> dict[str, Any]:
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
        "candidate_input_id": INPUT_ID,
        "candidate_input_version": INPUT_VERSION,
        "profile": INPUT_PROFILE,
        "corpus_id": corpus["corpus_id"],
        "corpus_version": corpus["corpus_version"],
        "stimulus_id": stimulus["stimulus_id"],
        "stimulus_version": stimulus["stimulus_version"],
        "subjects": subjects,
        "cases": stimulus["cases"],
    }
    _walk_forbidden(result)
    return result


def build_from_bytes(corpus_raw: bytes, schema_raw: bytes, stimulus_raw: bytes) -> dict[str, Any]:
    import validate_i0
    import validate_stimulus

    validate_i0.validate_bytes(corpus_raw, schema_raw=schema_raw)
    validate_stimulus.validate_bytes(stimulus_raw, corpus_raw)
    corpus = validate_i0.parse_strict_json(corpus_raw, label="corpus")
    stimulus = validate_stimulus.parse_strict_json(stimulus_raw, label="stimulus")
    return build_candidate_input(corpus, stimulus)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    here = Path(__file__).resolve().parent
    parser.add_argument("--corpus", type=Path, default=here / "myc-int-006c-water-i0-corpus.v1.json")
    parser.add_argument("--schema", type=Path, default=here / "myc-int-006c-water-i0-corpus.schema.json")
    parser.add_argument(
        "--stimulus",
        type=Path,
        default=here / "myc-int-006e-water-i0-stimulus.v1.json",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        result = build_from_bytes(
            args.corpus.read_bytes(),
            args.schema.read_bytes(),
            args.stimulus.read_bytes(),
        )
    except (OSError, ValueError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
