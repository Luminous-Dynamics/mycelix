#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any, Callable

FIXTURE_SCHEMA = "mycelix-amsap-004a-rest-replay-v1"
VECTOR_SCHEMA = "mycelix-evidence-ci-core-adapter-vectors-v1"
INTEGRITY_GIT_BLOB = "1e063f678e19574d796d22ecc1eb6d7805700edd"
INTEGRITY_IMPLEMENTATION = "cb252abe2a45d7309e165b7b2352a236f591daf35eb9e7fa21f051636156a8e5"
SEMANTIC_CORE_HEAD = "4190f855eb0f3c03a7a6b0decee84dd7edba07b4"
FROZEN_VECTORS_GIT_BLOB = "4174c9b53db828b108196778fb1585a0988a91ed"

class ReplayError(ValueError):
    pass

def git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

def load_json(path: Path, expected_blob: str | None = None) -> Any:
    data = path.read_bytes()
    if expected_blob is not None and git_blob(data) != expected_blob:
        raise ReplayError(f"Git blob mismatch: {path}")
    return json.loads(data)

def _exact(obj: dict[str, Any], fields: set[str], where: str) -> None:
    if not isinstance(obj, dict) or set(obj) != fields:
        raise ReplayError(f"{where} field mismatch")

def validate_fixture_document(doc: Any) -> list[dict[str, Any]]:
    _exact(doc, {"schema", "fixture_kind", "source_authenticity_claimed", "cases"}, "fixture")
    if doc["schema"] != FIXTURE_SCHEMA:
        raise ReplayError("fixture schema mismatch")
    if doc["fixture_kind"] != "deterministic-rest-shaped-replay":
        raise ReplayError("fixture kind mismatch")
    if doc["source_authenticity_claimed"] is not False:
        raise ReplayError("fixture must not claim source authenticity")
    cases = doc["cases"]
    if not isinstance(cases, list) or len(cases) != 2:
        raise ReplayError("expected exactly two AMSAP replay cases")
    ids: set[str] = set()
    for case in cases:
        _exact(case, {"id", "observed_at_utc", "profile", "run", "jobs", "expected_vector_id"}, "case")
        if case["id"] in ids or case["id"] != case["expected_vector_id"]:
            raise ReplayError("case/vector identity mismatch")
        ids.add(case["id"])
        profile = case["profile"]
        required = profile.get("required_jobs") if isinstance(profile, dict) else None
        if not isinstance(required, list) or len(required) != 1:
            raise ReplayError("expected one required job")
        gates = required[0].get("required_gate_names") if isinstance(required[0], dict) else None
        if not isinstance(gates, list) or len(gates) != 5 or len(set(gates)) != len(gates):
            raise ReplayError("AMSAP gate profile mismatch")
    return cases

def vector_index(doc: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(doc, dict) or doc.get("schema") != VECTOR_SCHEMA:
        raise ReplayError("vector schema mismatch")
    if doc.get("supported_core_head") != SEMANTIC_CORE_HEAD:
        raise ReplayError("vector semantic-core head mismatch")
    vectors = doc.get("vectors")
    if not isinstance(vectors, list) or len(vectors) != 2:
        raise ReplayError("expected exactly two frozen vectors")
    out: dict[str, dict[str, Any]] = {}
    for vector in vectors:
        if not isinstance(vector, dict) or not isinstance(vector.get("id"), str) or vector["id"] in out:
            raise ReplayError("invalid vector identity")
        out[vector["id"]] = vector
    return out

def evaluate(
    fixture_doc: Any,
    vector_doc: Any,
    adapt: Callable[[Any, Any, Any, str], dict[str, Any]],
) -> list[dict[str, Any]]:
    cases = validate_fixture_document(fixture_doc)
    vectors = vector_index(vector_doc)
    receipts: list[dict[str, Any]] = []
    for case in cases:
        vector = vectors.get(case["expected_vector_id"])
        if vector is None:
            raise ReplayError("fixture references unknown vector")
        receipt = adapt(case["run"], case["jobs"], case["profile"], case["observed_at_utc"])
        if receipt.get("supported_core_head") != SEMANTIC_CORE_HEAD:
            raise ReplayError("adapter semantic-core head mismatch")
        if receipt.get("core_manifest_v1") != vector.get("core_manifest_v1"):
            raise ReplayError(f"{case['id']}: manifest differs from frozen vector")
        if receipt.get("core_observation_v1") != vector.get("core_observation_v1"):
            raise ReplayError(f"{case['id']}: observation differs from frozen vector")
        diagnostics = receipt.get("provider_diagnostics")
        if not isinstance(diagnostics, list) or len(diagnostics) != 1:
            raise ReplayError("provider diagnostic shape mismatch")
        if diagnostics[0].get("provider_state") != vector.get("expected_provider_state"):
            raise ReplayError(f"{case['id']}: provider state mismatch")
        for key in (
            "observation_source_authenticity_verified",
            "github_api_response_authenticity_verified",
            "runner_identity_attested",
            "semantic_classification_performed",
            "qualification_authority",
            "evidence_authority",
            "failover_authority",
            "rerun_authority",
            "dispatch_authority",
        ):
            if receipt.get(key) is not False:
                raise ReplayError(f"authority/authenticity ceiling broadened: {key}")
        if receipt.get("qualification_result") is not None or receipt.get("theorem_result") is not None:
            raise ReplayError("provider layer surfaced semantic result")
        receipts.append(receipt)
    return receipts

def load_integrity(path: Path):
    data = path.read_bytes()
    if git_blob(data) != INTEGRITY_GIT_BLOB:
        raise ReplayError("integrity adapter blob mismatch")
    spec = importlib.util.spec_from_file_location("github_rest_adapter_integrity", path)
    if spec is None or spec.loader is None:
        raise ReplayError("cannot load integrity adapter")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if getattr(module, "IMPLEMENTATION_COMMITMENT_SHA256", None) != INTEGRITY_IMPLEMENTATION:
        raise ReplayError("integrity implementation commitment mismatch")
    return module

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture", type=Path)
    parser.add_argument("vectors", type=Path)
    parser.add_argument("integrity", type=Path)
    args = parser.parse_args()
    fixture_doc = load_json(args.fixture)
    vector_doc = load_json(args.vectors, FROZEN_VECTORS_GIT_BLOB)
    integrity = load_integrity(args.integrity)
    receipts = evaluate(fixture_doc, vector_doc, integrity.adapt)
    for case, receipt in zip(fixture_doc["cases"], receipts, strict=True):
        print(f"{case['id']}: {receipt['provider_diagnostics'][0]['provider_state']} -> normalized vector exact")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())