#!/usr/bin/env python3
"""Static verifier for the D6U Holochain 0.7 runtime evidence contract.

This script verifies fixture/manifest identity and coverage declarations only.
It does not execute Holochain and must not be treated as runtime qualification.
"""

import hashlib
import subprocess
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"
D6S2_MANIFEST = ROOT / "docs/integral/d6s-canon-2-manifest.json"
D6S2_FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"

SUPPORTED = {
    "canonical-payload-accepted",
    "authorized-semantic-rejection",
    "payload-mutation",
    "wire-signature-invalid",
    "author-grant",
    "valid-capability",
    "wrong-capability",
    "revoked-capability",
    "provenance-mismatch",
    "nonce-replay",
    "expired-invocation",
    "wrong-zome",
    "wrong-function",
    "wrong-cell",
    "blocked-provenance",
}

UNSUPPORTED = {
    "wire-signature-valid",
    "nonce-stale",
}

UNSUPPORTED_REASONS = {
    "wire-signature-valid": "The isolated authenticated-but-not-yet-authorized intermediate state is not exposed as a standalone Holochain 0.7 app-interface result.",
    "nonce-stale": "Holochain 0.7 uses random 256-bit nonces and witnesses Fresh, Duplicate, Expired, or Future; the D6S stale/older-nonce state is not independently reproducible on this substrate.",
}


def git_blob_sha(path: Path) -> str:
    return subprocess.run(
        ["git", "hash-object", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    manifest = __import__("json").loads(MANIFEST.read_text(encoding="utf-8"))
    d6s2 = __import__("json").loads(D6S2_MANIFEST.read_text(encoding="utf-8"))

    assert manifest["profile"] == "D6U-RUNTIME-1"
    assert manifest["kind"] == "holochain-0.7-authority-boundary-runtime-manifest"
    assert manifest["version"] == 2
    assert manifest["claim_ceiling"] == "ReferenceModelOnly"
    assert manifest["evidence_verifier_path"] == "scripts/integral/verify_d6u_runtime_evidence.py"
    assert manifest["evidence_status"] == "NotExecuted"

    substrate = manifest["substrate"]
    assert substrate == {
        "holochain": "0.7.0",
        "hdk": "0.7.0",
        "hdi": "0.8.0",
        "rust": "1.96.1",
        "holochain_serialized_bytes": "0.0.57",
    }

    supported = set(manifest["supported_reference_cases"])
    unsupported = set(manifest["unsupported_reference_cases"])
    assert supported == SUPPORTED
    assert unsupported == UNSUPPORTED
    assert manifest["unsupported_reference_case_reasons"] == UNSUPPORTED_REASONS
    assert len(supported) == 15
    assert len(unsupported) == 2
    assert supported.isdisjoint(unsupported)

    deps = manifest["dependencies"]
    assert deps["d6s_canon_2_manifest_path"] == "docs/integral/d6s-canon-2-manifest.json"
    assert deps["d6s_canon_2_manifest_blob_sha"] == git_blob_sha(D6S2_MANIFEST)
    assert deps["d6s_canon_2_fixture_path"] == "docs/integral/d6s-canon-2-authority-boundary-fixture.json"
    assert deps["d6s_canon_2_fixture_blob_sha"] == git_blob_sha(D6S2_FIXTURE)
    assert deps["d6s_canon_1_corpus_sha256"] == \
        "9d61cdb2e625c13c5813fffb7cceea4af2f93ed60d7d64c068dc6d3f6f6b614d"

    assert d6s2["profile"] == "D6S-CANON-2"
    assert d6s2["claim_ceiling"] == "ReferenceModelOnly"
    assert d6s2["runtime_evidence"]["status"] == "NotExecuted"

    print("verified D6U runtime manifest")
    print("holochain=0.7.0")
    print("hdk=0.7.0")
    print("hdi=0.8.0")
    print("supported_cases=15")
    print("unsupported_cases=2")
    print("claim_ceiling=ReferenceModelOnly")


if __name__ == "__main__":
    main()
