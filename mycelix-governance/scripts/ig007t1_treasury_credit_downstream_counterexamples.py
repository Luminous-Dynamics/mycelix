#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from validate_ig007t0_treasury_credit_downstream_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-treasury-credit-downstream-counterexamples-v1"
PROFILE_SHA256 = "77db81e40ebe7b0ec9278f8c9aad7584faa08b6784cde4779393d1d6b88cf58d"
EXPECTED_CORPUS_SHA256 = "89b795cea150c1d0f18aa63bc6adfe2ffd31389ef9396bbde810f647540695de"


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


def build_corpus(profile: dict) -> dict:
    ref = validate(profile)
    if ref["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("T1 requires exact T0 profile")

    legacy = profile["legacy_execution_transfer"]
    finance = profile["governance_bridge_finance_transfer"]
    plane = profile["treasury_allocation_plane"]
    integrity = profile["treasury_integrity_plane"]

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": profile["profile_id"],
            "content_sha256": PROFILE_SHA256,
            "authority_class": profile["authority_class"],
        },
        "issues": [1085, 1086],
        "counterexamples": [
            {
                "id": "CE-TC-01",
                "revision": 1,
                "kind": "LegacyDispatchContainment",
                "inputs": {
                    "action": "TransferCredits",
                    "dispatch_target": "governance_bridge::transfer_credits",
                    "bridge_module_count": legacy["bridge_target_observation"]["exact_module_count"],
                    "observed_target_occurrences": legacy["bridge_target_observation"]["target_occurrences"],
                },
                "comparison": {
                    "on_call_error": legacy["bridge_target_observation"]["on_call_error"],
                    "result": "LegacyTransferCreditsDispatchFailClosedByMissingTarget",
                },
                "non_claim": "No live value transfer is performed or alleged.",
            },
            {
                "id": "CE-TC-02",
                "revision": 1,
                "kind": "BridgeFinanceDispatchContainment",
                "inputs": {
                    "bridge_entrypoint": finance["entrypoint"],
                    "dispatch_target": "finance/treasury::execute_governance_transfer",
                    "observed_target_occurrences": finance["treasury_target_observation"]["target_occurrences"],
                    "error_helper": finance["treasury_target_observation"]["error_helper"],
                },
                "comparison": {
                    "helper_failure_semantics": finance["treasury_target_observation"]["helper_failure_semantics"],
                    "result": "ApprovedTransferBridgeDispatchFailClosedByMissingTreasuryEntrypoint",
                },
                "non_claim": "Records source-visible fail-closed dispatch behavior; no transfer is attempted.",
            },
            {
                "id": "CE-TC-03",
                "revision": 1,
                "kind": "CoordinatorIntegrityAuthorityMismatch",
                "inputs": {
                    "coordinator_approval_rule": plane["approve_allocation"]["approval_rule"],
                    "coordinator_approver_must_be_manager": plane["approve_allocation"]["approver_must_be_manager"],
                    "integrity_update_checks": integrity["validate_update_allocation"]["checks"],
                    "integrity_manager_authority": integrity["validate_update_allocation"]["approved_by_authority_reconstruction"],
                    "integrity_transition_graph": integrity["validate_update_allocation"]["status_transition_graph"],
                },
                "comparison": {
                    "result": "CoordinatorApprovalPolicyNotReconstructedByIntegrity",
                },
                "non_claim": "Does not perform an unauthorized Allocation publication.",
            },
            {
                "id": "CE-TC-04",
                "revision": 1,
                "kind": "TreasuryMutationAuthorityObservation",
                "inputs": {
                    "candidate_balance_change": True,
                    "candidate_manager_change": True,
                    "reserve_ratio": 0.5,
                    "observed_integrity_checks": integrity["validate_update_treasury"]["checks"],
                    "balance_authority": integrity["validate_update_treasury"]["balance_authority_reconstruction"],
                    "manager_authority": integrity["validate_update_treasury"]["manager_authority_reconstruction"],
                },
                "comparison": {
                    "result": "TreasuryShapeValidityDoesNotEstablishMutationAuthority",
                },
                "non_claim": "Pure source-contract fixture; no Treasury update is published.",
            },
            {
                "id": "CE-TC-05",
                "revision": 1,
                "kind": "AllocationTransitionAuthorityObservation",
                "inputs": {
                    "amount": 1,
                    "candidate_status_change": "ProposedToExecuted",
                    "candidate_recipient_change": True,
                    "candidate_approved_by_change": True,
                    "observed_integrity_checks": integrity["validate_update_allocation"]["checks"],
                    "transition_graph": integrity["validate_update_allocation"]["status_transition_graph"],
                    "immutable_field_binding": integrity["validate_update_allocation"]["immutable_field_binding"],
                },
                "comparison": {
                    "result": "AllocationShapeValidityDoesNotEstablishAuthorizedTransition",
                },
                "non_claim": "Pure source-contract fixture; no Allocation update is published.",
            },
            {
                "id": "CE-TC-06",
                "revision": 1,
                "kind": "ProposalLinkageAuthorityObservation",
                "inputs": {
                    "propose_allocation_proposal_id": "None",
                    "proposal_id_shape": plane["propose_allocation"]["proposal_id"],
                    "proposal_authority_reconstruction": plane["propose_allocation"]["proposal_authority_reconstruction"],
                },
                "comparison": {
                    "result": "ObservedAllocationProposalLinkageIsOptionalAndUnreconstructed",
                },
                "non_claim": "Does not claim that all Treasury allocations must originate from governance proposals.",
            },
        ],
        "positive_controls": [
            "LegacyTransferCallFailsClosedOnMissingTarget",
            "BridgeFinanceCallFailsClosedOnMissingTarget",
            "CanonicalAllocationCoordinatorUsesManagerMajorityApproval",
            "TreasuryDebitUsesCheckedSub",
        ],
        "non_claims": [
            "no_live_unauthorized_transfer",
            "no_forged_treasury_or_allocation_publication",
            "no_stolen_funds_claim",
            "no_deployment_exploit",
            "no_deployment_currentness",
            "no_governance_safety_claim",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    first = build_corpus(profile)
    second = build_corpus(profile)
    if canonical(first) != canonical(second):
        raise AssertionError("non-deterministic Treasury/Credit corpus")
    if first["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError(f"Treasury/Credit corpus commitment drift: {first['corpus_sha256']}")

    by_id = {item["id"]: item for item in first["counterexamples"]}
    expected = {
        "CE-TC-01": "LegacyTransferCreditsDispatchFailClosedByMissingTarget",
        "CE-TC-02": "ApprovedTransferBridgeDispatchFailClosedByMissingTreasuryEntrypoint",
        "CE-TC-03": "CoordinatorApprovalPolicyNotReconstructedByIntegrity",
        "CE-TC-04": "TreasuryShapeValidityDoesNotEstablishMutationAuthority",
        "CE-TC-05": "AllocationShapeValidityDoesNotEstablishAuthorizedTransition",
        "CE-TC-06": "ObservedAllocationProposalLinkageIsOptionalAndUnreconstructed",
    }
    if set(by_id) != set(expected):
        raise AssertionError("counterexample roster drift")
    for key, result in expected.items():
        if by_id[key]["comparison"]["result"] != result:
            raise AssertionError(f"{key} result drift")

    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 6,
        "issues": [1085, 1086],
        "positive_control_count": 4,
        "self_test": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--corpus", action="store_true")
    args = parser.parse_args()
    profile = load(args.profile)
    result = self_test(profile) if args.self_test else build_corpus(profile)
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()
