#!/usr/bin/env python3
"""Independent, zero-network verifier for the frozen artificial-sovereignty corpus."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PROFILE_SCHEMA = "qual-001-independent-verifier-profile-v1"
SEMANTIC_ENCODING = "UTF-8 compact JSON array of [id,scenario,expected_outcome,forbidden_inference], sorted by id"
REQUIRED_KEYS = {"id", "scenario", "expected_outcome", "forbidden_inference", "dependencies"}


def canonical_semantics(vectors: list[dict]) -> bytes:
    rows = [
        [v["id"], v["scenario"], v["expected_outcome"], v["forbidden_inference"]]
        for v in sorted(vectors, key=lambda item: item["id"])
    ]
    return json.dumps(rows, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def git_blob_sha1(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def load_json(path: Path) -> tuple[dict, bytes]:
    raw = path.read_bytes()
    return json.loads(raw.decode("utf-8")), raw


def fail(message: str) -> None:
    raise SystemExit(f"QUALIFICATION_FAIL: {message}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--subject", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--subject-commit", required=True)
    parser.add_argument("--subject-tree", required=True)
    parser.add_argument("--verifier-commit", required=True)
    args = parser.parse_args()

    profile, profile_bytes = load_json(args.profile)
    if profile.get("profile_schema") != PROFILE_SCHEMA:
        fail("unexpected verifier profile schema")
    if profile.get("role") != "independent-qualification-verifier":
        fail("profile is not an independent verifier")
    if profile.get("expected_semantics", {}).get("canonical_encoding") != SEMANTIC_ENCODING:
        fail("unexpected canonical semantic encoding")

    target = profile["subject"]
    if args.subject_commit != target["head_sha"]:
        fail("subject commit does not match frozen profile")
    if profile["required_candidate_metadata"].get("qualified") is not False:
        fail("profile itself permits a qualified candidate state")

    corpus, corpus_bytes = load_json(args.subject)
    required = profile["required_candidate_metadata"]
    for key, expected in required.items():
        if corpus.get(key) != expected:
            fail(f"candidate metadata {key!r} does not match frozen profile")

    vectors = corpus.get("vectors")
    if not isinstance(vectors, list) or len(vectors) != profile["expected_semantics"]["vector_count"]:
        fail("candidate vector count mismatch")

    seen = set()
    for index, vector in enumerate(vectors, start=1):
        if not isinstance(vector, dict):
            fail(f"vector {index} is not an object")
        if set(vector) != REQUIRED_KEYS:
            fail(f"{index}: vector keys differ from profile")
        expected_id = f"SOV-AI-{index:03d}"
        if vector["id"] != expected_id:
            fail(f"expected {expected_id}, found {vector['id']!r}")
        if vector["scenario"] in seen:
            fail(f"duplicate scenario {vector['scenario']!r}")
        seen.add(vector["scenario"])
        if not isinstance(vector["dependencies"], list) or not vector["dependencies"]:
            fail(f"{vector['id']}: empty dependencies")

    actual_blob = git_blob_sha1(corpus_bytes)
    if actual_blob != target["corpus_git_blob_sha"]:
        fail("candidate corpus bytes do not match frozen subject commitment")

    semantics = canonical_semantics(vectors)
    actual_semantic = hashlib.sha256(semantics).hexdigest()
    expected_semantic = profile["expected_semantics"]["sha256"]
    if actual_semantic != expected_semantic:
        fail("candidate semantic projection does not match independent commitment")

    receipt = {
        "receipt_schema": "sovereignty-qualification-receipt-v1",
        "result": "QualifiedExactHead",
        "claim_scope": profile["authoritative_for"],
        "subject": {
            "repository": target["repository"],
            "head_sha": args.subject_commit,
            "tree_sha": args.subject_tree,
            "corpus_path": target["corpus_path"],
            "corpus_git_blob_sha": actual_blob,
        },
        "verifier": {
            "profile_id": profile["profile_id"],
            "head_sha": args.verifier_commit,
            "profile_sha256": hashlib.sha256(profile_bytes).hexdigest(),
            "implementation": "independent-zero-network-v1",
            "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        },
        "semantic_commitment": {
            "encoding": SEMANTIC_ENCODING,
            "sha256": actual_semantic,
            "vector_count": len(vectors),
        },
        "nonclaims": profile["evidence_boundary"],
    }
    print(json.dumps(receipt, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
