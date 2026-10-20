#!/usr/bin/env python3
"""Structural verifier for the D6S-CANON-2 authority-boundary reference fixture.

This gate checks fixture identity, matrix completeness, and the explicit mapping
to Holochain 0.7 invocation/authorization concepts. It is ReferenceModelOnly:
it does not execute Holochain authorization and must not be interpreted as
runtime qualification.
"""

import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).parents[2]
FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"
MANIFEST = ROOT / "docs/integral/d6s-canon-2-manifest.json"
CANON1_MANIFEST = ROOT / "docs/integral/d6s-canon-1-manifest.json"
CANON1_CORPUS = ROOT / "docs/integral/d6s-canon-1-golden-vectors.json"

EXPECTED = {
    "author-grant",
    "authorized-semantic-rejection",
    "blocked-provenance",
    "canonical-payload-accepted",
    "expired-invocation",
    "nonce-replay",
    "nonce-stale",
    "payload-mutation",
    "provenance-mismatch",
    "revoked-capability",
    "valid-capability",
    "wire-signature-invalid",
    "wire-signature-valid",
    "wrong-capability",
    "wrong-cell",
    "wrong-function",
    "wrong-zome",
}

PRE_ZOME_RESULTS = {
    "d6s-commitment-mismatch",
    "holochain-routing-or-binding-rejection",
    "holochain-binding-or-authorization-rejection",
    "holochain-authorization-rejection",
    "holochain-nonce-rejection",
    "holochain-expiry-rejection",
    "holochain-signature-authentication-rejection",
}

AUTHORIZED = {
    "author-grant",
    "authorized-semantic-rejection",
    "canonical-payload-accepted",
    "valid-capability",
}


def git_blob_sha(path: Path) -> str:
    return subprocess.run(
        ["git", "hash-object", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    canon1_manifest = json.loads(CANON1_MANIFEST.read_text(encoding="utf-8"))

    assert fixture["profile"] == "D6S-CANON-2"
    assert fixture["kind"] == "authority-boundary-reference-fixture"
    assert fixture["version"] == 1
    assert fixture["depends_on"]["canonicalization_profile"] == "D6S-CANON-1"
    assert fixture["depends_on"]["claim_ceiling"] == "ReferenceModelOnly"

    assert manifest["profile"] == "D6S-CANON-2"
    assert manifest["kind"] == "authority-boundary-reference-manifest"
    assert manifest["version"] == 1
    assert manifest["fixture_path"] == "docs/integral/d6s-canon-2-authority-boundary-fixture.json"
    assert manifest["fixture_blob_sha"] == git_blob_sha(FIXTURE)
    assert manifest["expected_case_count"] == len(EXPECTED)
    assert set(manifest["expected_case_ids"]) == EXPECTED
    assert manifest["claim_ceiling"] == "ReferenceModelOnly"

    ledger_schema = manifest["ledger_schema"]
    assert ledger_schema["terminal_gate_field"] == "terminal_gate"
    assert ledger_schema["authority_state_field"] == "authority_state"
    assert ledger_schema["gate_vocabulary"] == [
        "wire-authentication",
        "d6s-integrity",
        "invocation-routing-or-binding",
        "invocation-binding-or-authorization",
        "capability-authorization",
        "nonce-replay-protection",
        "invocation-expiry",
        "zome-semantic-validation",
    ]
    assert ledger_schema["authority_states"] == [
        "unauthenticated",
        "authenticated-not-authorized",
        "not-authorized",
        "authorized",
    ]

    deps = manifest["dependencies"]
    assert deps["d6s_canon_1_manifest_path"] == "docs/integral/d6s-canon-1-manifest.json"
    assert deps["d6s_canon_1_manifest_blob_sha"] == git_blob_sha(CANON1_MANIFEST)
    assert deps["d6s_canon_1_corpus_path"] == "docs/integral/d6s-canon-1-golden-vectors.json"
    assert deps["d6s_canon_1_corpus_blob_sha"] == git_blob_sha(CANON1_CORPUS)
    assert deps["d6s_canon_1_corpus_sha256"] == file_sha256(CANON1_CORPUS)

    assert canon1_manifest["profile"] == "D6S-CANON-1"
    assert canon1_manifest["canonicalization_version"] == "D6S-CANON-1"
    assert canon1_manifest["corpus_path"] == "docs/integral/d6s-canon-1-golden-vectors.json"
    assert canon1_manifest["corpus_sha256"] == file_sha256(CANON1_CORPUS)
    assert canon1_manifest["expected_vector_count"] == 8
    assert canon1_manifest["expected_rejection_count"] == 9
    assert canon1_manifest["claim_ceiling"] == "ReferenceModelOnly"

    runtime_evidence = manifest["runtime_evidence"]
    assert runtime_evidence["status"] == "NotExecuted"
    assert runtime_evidence["harness_issue"] == 3851
    assert runtime_evidence["required_substrate"] == {
        "holochain": "0.7.0",
        "hdk": "0.7.0",
        "hdi": "0.8.0",
    }
    assert runtime_evidence["required_fields"] == [
        "source_commit",
        "runtime_version",
        "hdk_version",
        "hdi_version",
        "fixture_identity",
        "call_authentication",
        "d6s_commitment_result",
        "invocation_binding_result",
        "capability_result",
        "nonce_result",
        "expiry_result",
        "zome_reached",
        "semantic_result",
        "claim_ceiling",
    ]

    reference = manifest["holochain_reference"]
    assert reference["version"] == "0.7.0"
    assert reference["runtime_binding_status"] == "ReferenceMappingOnly"
    assert reference["invocation_type"] == "ZomeCallInvocation"
    assert reference["invocation_fields"] == [
        "cell_id",
        "zome",
        "cap_secret",
        "fn_name",
        "payload",
        "provenance",
        "nonce",
        "expires_at",
    ]
    assert reference["authorization_methods"] == [
        "verify_nonce",
        "verify_grant",
        "verify_blocked_provenance",
        "is_authorized",
    ]
    assert reference["authorization_order"] == [
        "verify_nonce",
        "verify_grant",
        "verify_blocked_provenance",
    ]

    cases = fixture["boundary"]
    assert len(cases) == len(EXPECTED), (len(cases), len(EXPECTED))
    ids = {case["case_id"] for case in cases}
    assert ids == EXPECTED, sorted(ids ^ EXPECTED)

    for case in cases:
        result = case["boundary_result"]
        reached = case["zome_reached"]
        semantic = case["semantic_result"]
        terminal_gate = case["terminal_gate"]
        authority_state = case["authority_state"]

        assert terminal_gate in ledger_schema["gate_vocabulary"], case["case_id"]
        assert authority_state in ledger_schema["authority_states"], case["case_id"]
        if reached:
            assert authority_state == "authorized", case["case_id"]
        else:
            assert authority_state != "authorized", case["case_id"]

        if result in PRE_ZOME_RESULTS:
            assert reached is False, case["case_id"]
            assert semantic == "not-reached", case["case_id"]

        if case["case_id"] == "wire-signature-valid":
            assert result == "authenticated"
            assert reached is False
            assert semantic == "authorization-not-yet-evaluated"

        if reached:
            assert result == "authorized", case["case_id"]
            assert semantic in {
                "subject-to-zome-validation",
                "rejected-by-zome",
            }, case["case_id"]

    authorized = {case["case_id"] for case in cases if case["zome_reached"]}
    assert authorized == AUTHORIZED

    invalid_signature = next(
        case for case in cases if case["case_id"] == "wire-signature-invalid"
    )
    assert invalid_signature["boundary_result"] == "holochain-signature-authentication-rejection"
    assert invalid_signature["zome_reached"] is False

    expected_gates = {
        "wire-signature-invalid": "wire-authentication",
        "wire-signature-valid": "wire-authentication",
        "payload-mutation": "d6s-integrity",
        "wrong-cell": "invocation-routing-or-binding",
        "wrong-zome": "invocation-binding-or-authorization",
        "wrong-function": "invocation-binding-or-authorization",
        "valid-capability": "capability-authorization",
        "author-grant": "capability-authorization",
        "wrong-capability": "capability-authorization",
        "revoked-capability": "capability-authorization",
        "provenance-mismatch": "capability-authorization",
        "blocked-provenance": "capability-authorization",
        "nonce-replay": "nonce-replay-protection",
        "nonce-stale": "nonce-replay-protection",
        "expired-invocation": "invocation-expiry",
        "canonical-payload-accepted": "capability-authorization",
        "authorized-semantic-rejection": "zome-semantic-validation",
    }
    assert {case["case_id"]: case["terminal_gate"] for case in cases} == expected_gates

    author_grant = next(case for case in cases if case["case_id"] == "author-grant")
    assert author_grant["capability_state"] == "author-grant"
    assert author_grant["provenance_state"] == "current-author"

    print(f"verified {len(cases)} D6S-CANON-2 authority-boundary cases")
    print("fixture_blob_sha=" + git_blob_sha(FIXTURE))
    print("canon1_manifest_blob_sha=" + git_blob_sha(CANON1_MANIFEST))
    print("canon1_corpus_blob_sha=" + git_blob_sha(CANON1_CORPUS))
    print("canon1_corpus_sha256=" + file_sha256(CANON1_CORPUS))
    print("holochain_reference=0.7.0")
    print("runtime_binding_status=ReferenceMappingOnly")
    print("zome_reached_cases=" + ",".join(sorted(authorized)))
    print("authority_ledger_schema=v1")
    print("claim_ceiling=ReferenceModelOnly")


if __name__ == "__main__":
    main()

