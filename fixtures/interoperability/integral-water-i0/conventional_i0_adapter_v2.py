#!/usr/bin/env python3
"""Neutral-protocol wrapper for the existing conventional I0 semantic engine."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import build_candidate_input as legacy_input
import build_candidate_input_v2 as neutral_input
import conventional_i0_adapter as legacy_adapter

RESULT_SCHEMA_REF = "./myc-int-006i-i0-candidate-results.schema.json"
RESULT_PROTOCOL_ID = "myc-int-i0-candidate-results"
RESULT_PROTOCOL_VERSION = "1.0.0"
RESULT_PROFILE = "runtime-neutral-candidate-results-v1"
IMPLEMENTATION = {
    "implementation_id": "mycelix.i0.conventional.sqlite",
    "family": "conventional-sqlite",
    "version": "2.0.0",
}


class NeutralAdapterError(ValueError):
    pass


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise NeutralAdapterError(f"duplicate JSON object member: {key!r}")
        out[key] = value
    return out


def parse_input(raw: bytes) -> dict[str, Any]:
    try:
        document = json.loads(raw.decode("utf-8", errors="strict"), object_pairs_hook=_strict_object)
    except NeutralAdapterError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise NeutralAdapterError(f"invalid neutral candidate input: {exc}") from exc
    if not isinstance(document, dict):
        raise NeutralAdapterError("neutral candidate input must be an object")
    neutral_input._walk_forbidden(document)
    expected = {
        "$schema": neutral_input.INPUT_SCHEMA_REF,
        "candidate_input_id": neutral_input.INPUT_ID,
        "candidate_input_version": neutral_input.INPUT_VERSION,
        "profile": neutral_input.INPUT_PROFILE,
    }
    for key, value in expected.items():
        if document.get(key) != value:
            raise NeutralAdapterError(f"unexpected {key}: {document.get(key)!r}")
    if not isinstance(document.get("source_commitments"), dict):
        raise NeutralAdapterError("source commitments missing")
    if not isinstance(document.get("subjects"), dict) or not isinstance(document.get("cases"), dict):
        raise NeutralAdapterError("subjects/cases missing")
    return document


def _legacy_projection(document: dict[str, Any]) -> dict[str, Any]:
    projected = {
        "candidate_input_id": legacy_input.INPUT_ID,
        "candidate_input_version": legacy_input.INPUT_VERSION,
        "profile": legacy_input.INPUT_PROFILE,
        "corpus_id": document["corpus_id"],
        "corpus_version": document["corpus_version"],
        "stimulus_id": document["stimulus_id"],
        "stimulus_version": document["stimulus_version"],
        "subjects": document["subjects"],
        "cases": document["cases"],
    }
    legacy_input._walk_forbidden(projected)
    return projected


def run_bytes(raw: bytes) -> dict[str, Any]:
    document = parse_input(raw)
    legacy_result = legacy_adapter.run_document(_legacy_projection(document))
    translated_results = []
    for row in legacy_result["results"]:
        translated_results.append({
            "case_id": row["case_id"],
            "disposition": row["disposition"],
            "implementation_rule": row["candidate_rule"],
            "effect_count": row["effect_count"],
            "tags": row["tags"],
            "facts": row["facts"],
        })
    return {
        "$schema": RESULT_SCHEMA_REF,
        "result_protocol_id": RESULT_PROTOCOL_ID,
        "result_protocol_version": RESULT_PROTOCOL_VERSION,
        "profile": RESULT_PROFILE,
        "implementation": dict(IMPLEMENTATION),
        "candidate_input_id": neutral_input.INPUT_ID,
        "candidate_input_version": neutral_input.INPUT_VERSION,
        "candidate_input_sha256": hashlib.sha256(raw).hexdigest(),
        "results": translated_results,
    }


def canonical_bytes(document: dict[str, Any]) -> bytes:
    return (json.dumps(document, indent=2, sort_keys=True) + "\n").encode("utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate_input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        raw = args.candidate_input.read_bytes()
        result = run_bytes(raw)
    except (OSError, ValueError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    encoded = canonical_bytes(result)
    if args.output:
        args.output.write_bytes(encoded)
    else:
        sys.stdout.buffer.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
