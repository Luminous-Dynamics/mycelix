#!/usr/bin/env python3
"""Structural validator for the artificial sovereignty qualification candidate corpus.

This validator is deliberately non-authoritative. A successful validation means only
that the frozen candidate corpus satisfies its own structural schema. It is not a
qualification result and must not be used as the authoritative verifier.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

SCHEMA = "artificial-sovereignty-qualification-corpus-v1"
STATUS = "research-candidate"
AUTHORITY = "non-authoritative"
COUNT = 60
PREFIX = "SOV-AI-"
OUTCOMES = {
    "reject-inference",
    "preserve-dispute",
    "safe-state-only",
    "review-required",
    "authority-recompute-required",
    "preserve-history",
    "external-recognition-required",
    "independent-qualification-required",
}

REQUIRED_VECTOR_KEYS = {
    "id",
    "scenario",
    "expected_outcome",
    "forbidden_inference",
    "dependencies",
}


def fail(reason: str) -> "NoReturn":
    raise SystemExit(f"INVALID: {reason}")


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "docs/governance/ARTIFICIAL_SOVEREIGNTY_QUALIFICATION_CORPUS_V1.json"
    )
    try:
        corpus = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"cannot parse corpus: {exc}")

    if corpus.get("schema_version") != SCHEMA:
        fail("unexpected schema_version")
    if corpus.get("status") != STATUS:
        fail("candidate status changed")
    if corpus.get("authority") != AUTHORITY:
        fail("authority boundary changed")
    if corpus.get("qualified") is not False:
        fail("candidate corpus must remain explicitly unqualified")

    vectors = corpus.get("vectors")
    if not isinstance(vectors, list) or len(vectors) != COUNT:
        fail(f"expected exactly {COUNT} vectors")

    ids = []
    scenarios = set()

    for index, vector in enumerate(vectors, start=1):
        if not isinstance(vector, dict):
            fail(f"vector {index} is not an object")
        if set(vector) != REQUIRED_VECTOR_KEYS:
            fail(f"{index}: unexpected/missing vector keys")

        expected_id = f"{PREFIX}{index:03d}"
        if vector["id"] != expected_id:
            fail(f"expected {expected_id}, found {vector['id']!r}")
        if vector["id"] in ids:
            fail(f"duplicate id {vector['id']}")
        ids.append(vector["id"])

        scenario = vector["scenario"]
        if not isinstance(scenario, str) or not scenario.strip():
            fail(f"{vector['id']}: empty scenario")
        if scenario in scenarios:
            fail(f"duplicate scenario {scenario}")
        scenarios.add(scenario)

        outcome = vector["expected_outcome"]
        if outcome not in OUTCOMES:
            fail(f"{vector['id']}: unregistered outcome {outcome!r}")

        forbidden = vector["forbidden_inference"]
        if not isinstance(forbidden, str) or not forbidden.strip():
            fail(f"{vector['id']}: empty forbidden_inference")

        dependencies = vector["dependencies"]
        if (
            not isinstance(dependencies, list)
            or not dependencies
            or any(not isinstance(dep, str) or not dep.strip() for dep in dependencies)
        ):
            fail(f"{vector['id']}: dependencies must be non-empty strings")

    print(json.dumps({
        "implementation": "artificial-sovereignty-structural-validator-v1",
        "result": "valid",
        "vector_count": len(vectors),
        "first_id": ids[0],
        "last_id": ids[-1],
        "qualified": corpus["qualified"],
        "authority": corpus["authority"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
