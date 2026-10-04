#!/usr/bin/env python3
"""Static verifier for the D6U Holochain 0.7 runtime evidence contract.

This script verifies fixture/manifest identity and coverage declarations only.
It does not execute Holochain and must not be treated as runtime qualification.
"""

import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"
D6S2_MANIFEST = ROOT / "docs/integral/d6s-canon-2-manifest.json"
D6S2_FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"

SUPPLEMENTAL_SUBSTRATE_CHECKS = {
    "future-expiry-rejection",
    "wrong-zome-routing",
    "wrong-function-routing",
    "wrong-cell-routing",
}

EVIDENCE_ARTIFACT_FILES = {
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
}

SUPPLEMENTAL_APPLICATION_CHECKS = {
    "probe-local-d6s-commitment-mutation",
}

UNSUPPORTED_REASONS = {
    "wire-signature-valid": "The isolated authenticated-but-not-yet-authorized intermediate state is not exposed as a standalone Holochain 0.7 app-interface result.",
    "nonce-stale": "Holochain 0.7 uses random 256-bit nonces and witnesses Fresh, Duplicate, Expired, or Future; the D6S stale/older-nonce state is not independently reproducible on this substrate.",
    "payload-mutation": "D6S-CANON-2 defines this as a pre-zome D6S-integrity rejection with zome_reached=false, but Holochain 0.7 has no native invocation-payload D6S commitment gate; the harness retains a probe-local commitment check only as supplemental application evidence.",
}

EXPECTED_EVIDENCE_CLASSES = [
    "accepted",
    "semantic-rejected",
    "d6s-commitment-mismatch",
    "authentication-failed",
    "authorization-failed",
    "routing-failed",
]


def git_blob_sha(path: Path) -> str:
    return subprocess.run(
        ["git", "hash-object", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive_case_outcomes(fixture_cases: list[dict]) -> dict[str, str]:
    """Derive D6U protocol outcomes from the canonical D6S-CANON-2 fixture.

    The case IDs themselves remain canonical fixture data. Only the translation
    from reference boundary results to D6U observation classes is encoded here.
    """
    outcomes: dict[str, str] = {}
    for case in fixture_cases:
        case_id = case["case_id"]
        boundary_result = case["boundary_result"]
        semantic_result = case["semantic_result"]

        if boundary_result == "authorized":
            outcomes[case_id] = (
                "semantic-rejected"
                if semantic_result == "rejected-by-zome"
                else "accepted"
            )
        elif boundary_result == "holochain-signature-authentication-rejection":
            outcomes[case_id] = "authentication-failed"
        elif boundary_result in {
            "holochain-authorization-rejection",
            "holochain-nonce-rejection",
            "holochain-expiry-rejection",
        }:
            outcomes[case_id] = "authorization-failed"
        elif boundary_result in {
            "holochain-routing-or-binding-rejection",
            "holochain-binding-or-authorization-rejection",
        }:
            outcomes[case_id] = "routing-failed"
    return outcomes


def main() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    d6s2 = json.loads(D6S2_MANIFEST.read_text(encoding="utf-8"))
    fixture = json.loads(D6S2_FIXTURE.read_text(encoding="utf-8"))

    assert manifest["profile"] == "D6U-RUNTIME-1"
    assert manifest["kind"] == "holochain-0.7-authority-boundary-runtime-manifest"
    assert isinstance(manifest["version"], int) and manifest["version"] > 0
    assert manifest["claim_ceiling"] == "ReferenceModelOnly"
    assert manifest["evidence_verifier_path"] == "scripts/integral/verify_d6u_runtime_evidence.py"
    assert manifest["evidence_record_verifier_path"] == "scripts/integral/verify_d6u_runtime_record.py"
    assert manifest["evidence_fields"] == [
        "source_commit",
        "workflow_run_id",
        "workflow_run_attempt",
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
        "case_outcome",
        "application_check_coverage",
    ]
    assert manifest["evidence_status"] == "NotExecuted"

    substrate = manifest["substrate"]
    assert substrate == {
        "holochain": "0.7.0",
        "hdk": "0.7.0",
        "hdi": "0.8.0",
        "rust": "1.96.1",
        "holochain_serialized_bytes": "0.0.57",
    }

    fixture_cases = fixture["boundary"]
    fixture_case_ids = {case["case_id"] for case in fixture_cases}
    assert fixture["profile"] == "D6S-CANON-2"
    assert d6s2["expected_case_count"] == len(fixture_case_ids)
    assert fixture_case_ids == set(d6s2["expected_case_ids"])
    assert len(fixture_cases) == len(fixture_case_ids)

    supported = set(manifest["supported_reference_cases"])
    unsupported = set(manifest["unsupported_reference_cases"])
    assert supported | unsupported == fixture_case_ids
    assert supported.isdisjoint(unsupported)
    assert manifest["unsupported_reference_case_reasons"] == UNSUPPORTED_REASONS
    assert set(manifest["unsupported_reference_case_reasons"]) == unsupported
    assert len(supported) == 14
    assert len(unsupported) == 3

    derived_outcomes = derive_case_outcomes(fixture_cases)
    assert supported <= set(derived_outcomes)
    expected_case_outcomes = {
        case_id: derived_outcomes[case_id] for case_id in supported
    }
    assert manifest["case_outcomes"] == expected_case_outcomes
    assert set(manifest["case_outcomes"]) == supported
    assert set(manifest["case_outcomes"].values()) <= set(
        EXPECTED_EVIDENCE_CLASSES
    )

    assert manifest["evidence_outcome_classes"] == EXPECTED_EVIDENCE_CLASSES
    assert set(manifest["supplemental_substrate_checks"]) == SUPPLEMENTAL_SUBSTRATE_CHECKS
    assert set(manifest["supplemental_application_checks"]) == SUPPLEMENTAL_APPLICATION_CHECKS
    assert set(manifest["evidence_artifact_files"]) == EVIDENCE_ARTIFACT_FILES

    deps = manifest["dependencies"]
    assert deps["d6s_canon_2_manifest_path"] == "docs/integral/d6s-canon-2-manifest.json"
    assert deps["d6s_canon_2_manifest_blob_sha"] == git_blob_sha(D6S2_MANIFEST)
    assert deps["d6s_canon_2_fixture_path"] == "docs/integral/d6s-canon-2-authority-boundary-fixture.json"
    assert deps["d6s_canon_2_fixture_blob_sha"] == git_blob_sha(D6S2_FIXTURE)
    assert deps["d6s_canon_1_corpus_sha256"] == (
        "9d61cdb2e625c13c5813fffb7cceea4af2f93ed60d7d64c068dc6d3f6f6b614d"
    )

    assert d6s2["profile"] == "D6S-CANON-2"
    assert d6s2["claim_ceiling"] == "ReferenceModelOnly"
    assert d6s2["runtime_evidence"]["status"] == "NotExecuted"

    print("verified D6U runtime manifest")
    print(f"holochain={substrate['holochain']}")
    print(f"hdk={substrate['hdk']}")
    print(f"hdi={substrate['hdi']}")
    print(f"supported_cases={len(supported)}")
    print(f"unsupported_cases={len(unsupported)}")
    print(f"claim_ceiling={manifest['claim_ceiling']}")


if __name__ == "__main__":
    main()
