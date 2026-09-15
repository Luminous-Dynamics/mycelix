#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from validate_ig007s0_threshold_signing_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-threshold-signing-counterexamples-v1"
PROFILE_SHA256 = "c15dfd860b759747938af2a13129d729fa0af1e75284418c9ea6b9c172f643ac"
EXPECTED_CORPUS_SHA256 = "0f6532ae8e2c2e421da625592dbb3b38aa2b90c5342f46f3a305bdbec89b0269"


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def build_corpus(profile: dict) -> dict:
    ref = validate(profile)
    if ref["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("S1 requires exact S0 threshold-signing profile")

    integrity = profile["threshold_signature_integrity"]
    source_test = profile["source_test_observation"]
    link = profile["proposal_to_signature_link"]
    producer = profile["producer_api"]

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": ref["profile_id"],
            "content_sha256": ref["profile_content_sha256"],
            "authority_class": ref["authority_class"],
        },
        "issues": [959, 960],
        "counterexamples": [
            {
                "id": "CE-SIG-01",
                "revision": 1,
                "kind": "StructuralValidatorObservation",
                "inputs": {
                    "algorithm": "Ecdsa",
                    "signature_bytes": source_test["signature_bytes"],
                    "signed_content_hash": source_test["signed_content_hash"],
                    "signer_count": source_test["signer_count"],
                    "signers": source_test["signers"],
                    "verified": source_test["verified"],
                },
                "comparison": {
                    "pure_validator_result": source_test["pure_validator_result"],
                    "evidence_class": source_test["evidence_class"],
                    "cryptographic_validity": "NotEstablished",
                },
                "non_claim": (
                    "Models the exact source test shape and pure structural validator result; "
                    "does not claim a cryptographically valid signature."
                ),
            },
            {
                "id": "CE-SIG-02",
                "revision": 1,
                "kind": "StoredFlagAuthorityDifferential",
                "inputs": {
                    "otherwise_identical": True,
                    "verified_values": [False, True],
                },
                "comparison": {
                    "verified_field_recomputed": integrity["verified_field_recomputed"],
                    "structural_validity_depends_on_verified": False,
                    "result": "VerifiedFlagDoesNotAffectObservedStructuralValidity",
                },
                "non_claim": (
                    "Shows the observed pure validator does not derive cryptographic truth from "
                    "or recompute the stored verified flag."
                ),
            },
            {
                "id": "CE-SIG-03",
                "revision": 1,
                "kind": "CommitteeAuthorityPredicateObservation",
                "inputs": {
                    "modeled_committee_threshold": 3,
                    "fixture_signer_count": 1,
                },
                "comparison": {
                    "committee_lookup": integrity["committee_lookup"],
                    "committee_active_epoch_check": integrity["committee_active_epoch_check"],
                    "committee_threshold_check": integrity["committee_threshold_check"],
                    "qualified_signer_membership_check": integrity["qualified_signer_membership_check"],
                    "committee_scope_check": integrity["committee_scope_check"],
                    "result": "NoObservedCommitteeAuthorityPredicateInSignatureCreateValidation",
                },
                "non_claim": (
                    "Records absent source-visible committee authorization predicates; "
                    "does not claim a live threshold bypass."
                ),
            },
            {
                "id": "CE-SIG-04",
                "revision": 1,
                "kind": "AssociationAuthorityDifferential",
                "inputs": {"link_type": "ProposalToSignature"},
                "comparison": {
                    "create_validation": link["create_validation"],
                    "exact_subject_authorization_reconstruction": link[
                        "exact_subject_authorization_reconstruction"
                    ],
                    "result": "LinkDoesNotEstablishObservedProposalSignatureAuthorization",
                },
                "non_claim": (
                    "Treats the observed link as association only; no live proposal-signature "
                    "authorization claim is made."
                ),
            },
            {
                "id": "CE-SIG-05",
                "revision": 1,
                "kind": "ProducerConsumerApiContractObservation",
                "inputs": {
                    "consumer_expected_queries": producer["consumer_expected_queries"],
                },
                "comparison": {
                    "producer_observed_externs": producer["observed_externs"],
                    "get_proposal_signature": producer["get_proposal_signature"],
                    "get_committee": producer["get_committee"],
                    "result": "ExpectedSignatureQueryContractAbsentFromObservedProducer",
                },
                "non_claim": (
                    "Records the exact source API mismatch without asserting a particular "
                    "runtime failure mode."
                ),
            },
        ],
        "non_claims": [
            "no_forged_signature_acceptance_claim",
            "no_cryptographic_break",
            "no_live_threshold_bypass",
            "no_deployment_exploit",
            "no_governance_safety_claim",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    first = build_corpus(profile)
    second = build_corpus(profile)
    if canonical(first) != canonical(second):
        raise AssertionError("non-deterministic threshold-signing corpus")
    if first["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError(
            f"threshold-signing corpus commitment drift: {first['corpus_sha256']}"
        )

    by_id = {item["id"]: item for item in first["counterexamples"]}
    if set(by_id) != {
        "CE-SIG-01",
        "CE-SIG-02",
        "CE-SIG-03",
        "CE-SIG-04",
        "CE-SIG-05",
    }:
        raise AssertionError("counterexample roster drift")

    assert by_id["CE-SIG-01"]["comparison"]["pure_validator_result"] == "AcceptedByCheckSignatureValidity"
    assert by_id["CE-SIG-01"]["comparison"]["cryptographic_validity"] == "NotEstablished"
    assert by_id["CE-SIG-02"]["comparison"]["verified_field_recomputed"] == "NoneObserved"
    assert by_id["CE-SIG-02"]["comparison"]["structural_validity_depends_on_verified"] is False
    assert by_id["CE-SIG-03"]["comparison"]["committee_threshold_check"] == "NoneObserved"
    assert by_id["CE-SIG-04"]["comparison"]["create_validation"] == "UnconditionalValidObserved"
    assert by_id["CE-SIG-05"]["comparison"]["producer_observed_externs"] == ["create_committee"]
    assert by_id["CE-SIG-05"]["comparison"]["get_proposal_signature"] == "NoneObserved"
    assert by_id["CE-SIG-05"]["comparison"]["get_committee"] == "NoneObserved"

    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 5,
        "issues": [959, 960],
        "self_test": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--corpus", action="store_true")
    args = parser.parse_args()

    profile = load(args.profile)
    if args.self_test:
        out = self_test(profile)
    elif args.corpus:
        out = build_corpus(profile)
    else:
        parser.error("choose --self-test or --corpus")

    print(json.dumps(out, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()
