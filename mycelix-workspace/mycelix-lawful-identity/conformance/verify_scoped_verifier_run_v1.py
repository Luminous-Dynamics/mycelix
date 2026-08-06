#!/usr/bin/env python3
"""Independent verifier-run transcript conformance for lawful identity."""

from __future__ import annotations

import base64
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
VECTOR = ROOT / "conformance" / "scoped_verifier_run_v1.json"


def b64decode32(name: str, value: str) -> bytes:
    try:
        decoded = base64.b64decode(value, validate=True)
    except Exception as error:
        raise ValueError(f"{name} is not canonical standard base64") from error
    if len(decoded) != 32 or base64.b64encode(decoded).decode("ascii") != value:
        raise ValueError(f"{name} must be canonical standard-base64 32 bytes")
    return decoded


def lp(value: bytes) -> bytes:
    return len(value).to_bytes(8, "little") + value


def sha_b64(value: bytes) -> str:
    return base64.b64encode(hashlib.sha256(value).digest()).decode("ascii")


def main() -> int:
    try:
        vector = json.loads(VECTOR.read_text(encoding="utf-8"))
        request = vector["request"]
        proof = base64.b64decode(request["proof_b64"], validate=True)
        if len(proof) != 96:
            raise ValueError("proof must be exactly 96 bytes")

        request_transcript = b"".join(
            [
                lp(b"MYCELIX:ScopedSubjectBindingVerifier:Request:v1"),
                lp(request["protocol"].encode()),
                lp(request["relation_id"].encode()),
                lp(request["verifier_implementation"].encode()),
                b64decode32("circuit_digest", request["circuit_digest_b64"]),
                b64decode32("verifier_parameter_id", request["verifier_parameter_id_b64"]),
                b64decode32(
                    "verifier_release_manifest_hash",
                    request["verifier_release_manifest_hash_b64"],
                ),
                b64decode32("pseudonym_scope_hash", request["pseudonym_scope_hash_b64"]),
                b64decode32("primary_pseudonym", request["primary_pseudonym_b64"]),
                b64decode32("lawful_pseudonym", request["lawful_pseudonym_b64"]),
                b64decode32(
                    "statement_context_hash", request["statement_context_hash_b64"]
                ),
                b64decode32(
                    "challenge_nonce_hash", request["challenge_nonce_hash_b64"]
                ),
                lp(proof),
            ]
        )
        request_hash = sha_b64(request_transcript)
        if request_hash != vector["expected_request_hash_b64"]:
            raise ValueError("request transcript hash mismatch")

        proof_hash_transcript = (
            (len(b"MYCELIX:RistrettoDleqScopedSubjectBinding:ProofHash:v2")).to_bytes(
                8, "little"
            )
            + b"MYCELIX:RistrettoDleqScopedSubjectBinding:ProofHash:v2"
            + len(proof).to_bytes(8, "little")
            + proof
        )
        proof_hash = sha_b64(proof_hash_transcript)
        if proof_hash != vector["expected_proof_hash_b64"]:
            raise ValueError("proof transcript hash mismatch")

        result_transcript = b"".join(
            [
                lp(b"MYCELIX:ScopedSubjectBindingVerifier:Result:v1"),
                lp(b"mycelix-scoped-subject-binding-verifier-result-v1"),
                b64decode32("request_hash", request_hash),
                b"\x01",
                b64decode32("proof_hash", proof_hash),
                b"\x01",
                b64decode32(
                    "verification_evidence_hash",
                    vector["expected_evidence_hash_b64"],
                ),
                b"\x00",
            ]
        )
        result_hash = sha_b64(result_transcript)
        if result_hash != vector["expected_result_hash_b64"]:
            raise ValueError("result transcript hash mismatch")

        integrity = (
            ROOT / "zomes" / "cross-did-zkp" / "integrity" / "src" / "lib.rs"
        ).read_text(encoding="utf-8")
        for value in (
            request["protocol"],
            "mycelix-scoped-subject-binding-verifier-result-v1",
            request["verifier_release_manifest_hash_b64"],
        ):
            if value not in integrity:
                raise ValueError(f"lawful integrity constants drifted from vector: {value}")

        print("scoped verifier-run conformance: PASS")
        print(f"request_hash_b64={request_hash}")
        print(f"result_hash_b64={result_hash}")
        return 0
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as error:
        print(f"scoped verifier-run conformance: FAIL: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
