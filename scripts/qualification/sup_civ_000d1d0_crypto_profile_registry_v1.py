#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

EXPECTED_PROFILES = [
    "MycelixHybridKemV1Experimental",
    "PulseV2HybridPqcExperimental",
    "ProtectedEnvelopeCmsMlKemV1",
]
EXPECTED_MIGRATION_STATES = [
    "KnownExistingHybridProfile",
    "KnownPulseV2Profile",
    "KnownProtectedEnvelopeCmsMlKemV1",
    "LegacyEncryptedEnvelopeProfileUnknown",
    "InvalidOrConflictingProfileEvidence",
]
EXPECTED_MODULES = [
    "protected_envelope::profile",
    "protected_envelope::codec",
    "protected_envelope::payload",
    "protected_envelope::recipient",
    "protected_envelope::interop",
]
EXPECTED_NON_EQ = {
    "AlgorithmId != ProtocolProfile",
    "EnvelopeFormatVersion != CryptoProfile",
    "SameMLKEMAlgorithm != SameKDFTranscript",
    "SameAESGCMAlgorithm != SameAADSemantics",
    "HybridKemV1 != PulseV2HybridPqc",
    "PulseV2HybridPqc != ProtectedEnvelopeCmsMlKemV1",
    "ExistingEnvelopeDecrypts != CivicStorageAdmitted",
    "LegacyEncryptedEnvelopeWithoutProfile != ProtectedEnvelopeCmsMlKemV1",
    "SuccessfulLegacyDecrypt != NewProfileQualified",
    "InteropVectorPass != RuntimeAuthorization",
    "ProfileAvailable != ProfileAuthorizedForDatum",
    "CryptoProfileQualified != StorageProfileQualified",
    "CryptoProfileQualified != ProtectedReadAuthorized",
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    args = ap.parse_args()
    m = json.loads(args.manifest.read_text())

    assert m["schema"] == "sup-civ-000d1d0-crypto-profile-registry-v1"
    assert m["program"] == "SUP-CIV-000D1D0"
    assert m["authority"] == "semantic-profile-boundary-only"
    assert m["source_audit"]["repository_head"] == "a85369699099d4c7524e502e531735eed4ab36f4"

    source = m["source_audit"]
    assert source == {
        "repository_head": "a85369699099d4c7524e502e531735eed4ab36f4",
        "workspace_cargo_blob": "9053f6f0e7a09aa88ed4743df81c4cb06066e3bd",
        "mycelix_crypto_cargo_blob": "e49c9fa098b780d2632a03de499e3b2f74ac91fe",
        "envelope_rs_blob": "bf7dd16570817fe29bf2f66d4f60a17e66d1ef5c",
        "hybrid_kem_rs_blob": "b469b16fc51eec39bf9741ec290007409f5d50ea",
        "pulse_v2_rs_blob": "11bb5f4cb8ce866aa37c75875ed32ffbf04c9fca",
        "lib_rs_blob": "966589dd6111e193a7b55e6b84fd68521ef8d2d3",
    }

    profiles = m["profiles"]
    assert [p["id"] for p in profiles] == EXPECTED_PROFILES
    assert len({p["id"] for p in profiles}) == 3
    assert all(p["storage_profile_qualified"] is False for p in profiles)
    assert profiles[0]["status"] == "ExistingExperimental"
    assert profiles[1]["status"] == "ExistingExperimental"
    assert profiles[2]["status"] == "EvidenceDefinedRuntimeDeferred"
    assert profiles[2]["wire_owner"] == "SUP-CIV-000D1B canonical transcript"

    deps = m["protected_profile_dependencies"]
    assert len(deps) == 8
    assert len({d["id"] for d in deps}) == 8
    assert all(d["status"] == "ReferencePendingQualification" for d in deps)

    assert m["migration_states"] == EXPECTED_MIGRATION_STATES
    assert m["implementation_modules"] == EXPECTED_MODULES
    assert set(m["required_non_equivalences"]) == EXPECTED_NON_EQ

    fallback = set(m["no_fallback"])
    assert fallback == {
        "ProtectedEnvelopeCmsMlKemV1->PulseV2HybridPqcExperimental",
        "ProtectedEnvelopeCmsMlKemV1->MycelixHybridKemV1Experimental",
        "ProtectedEnvelopeCmsMlKemV1->classical-only",
        "ProtectedEnvelopeCmsMlKemV1->plaintext",
        "unknown-profile->guess-from-length",
        "legacy-profile-unknown->claim-new-profile",
    }

    policy = m["dependency_policy"]
    assert policy["enable_ml_kem_globally"] is False
    assert policy["perturb_workspace_aead_resolution"] is False
    assert policy["first_runtime_step"] == "crypto-free-codec-and-profile-types"
    assert policy["ml_kem_runtime_adapter"] == "isolated-or-feature-gated-until-dependency-convergence-qualified"

    assert m["runtime_promotion"] == "blocked-until-selected-profile-load-bearing-dependencies-have-exact-qualified-evidence"
    assert set(m["forbidden_claims"]) == {
        "upstream-qualification",
        "runtime-crypto-correctness",
        "legacy-data-migrated",
        "storage-profile-qualified",
        "protected-read-authorized",
        "recipient-authorized",
        "legal-compliance",
        "deployment-readiness",
    }

    print("PASS_SUP_CIV_000D1D0_CRYPTO_PROFILE_REGISTRY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
