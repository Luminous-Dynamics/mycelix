#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-treasury-credit-downstream-observed-fca2c107-v1"
PROFILE_SHA256 = "eec7006f9f8dcc4a59d54cf795d87764fa73ed54f3494e65c1ca87209474e3f8"
PRODUCTION_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
AUTHORING_HEAD = "feb30a89257e96592fdbd40b249258581d42fde7"
AUTHORITY = "ObservedSourceBound"

BRIDGE_FILES = {
    "mycelix-governance/zomes/bridge/coordinator/src/attestation.rs": "6d7938084ba699b144ee5951966b1421c034f579",
    "mycelix-governance/zomes/bridge/coordinator/src/consciousness.rs": "d0acabf306594ab3ccfae220a9c9bd9aceed0ca6",
    "mycelix-governance/zomes/bridge/coordinator/src/consciousness_config.rs": "26da234e588bf26d0e25c10dbec34502e00c191a",
    "mycelix-governance/zomes/bridge/coordinator/src/consensus.rs": "3842dfa365953a01ca90bb79059da53f9cb00a6f",
    "mycelix-governance/zomes/bridge/coordinator/src/cross_cluster.rs": "3eb0ade8633d6e711fd266bca6eb2ebab616eac2",
    "mycelix-governance/zomes/bridge/coordinator/src/lib.rs": "fb278023c269a89f300504c18538ab85b09f1178",
    "mycelix-governance/zomes/bridge/coordinator/src/query.rs": "5a02812880a2fdaa8f3ed767a4686afc00c566f9",
    "mycelix-governance/zomes/bridge/coordinator/src/validation.rs": "9e13ba58939880738eccb997e54f729da7a11304",
}


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError("profile root must be an object")
    return value


def payload_digest(profile: dict) -> str:
    payload = dict(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def validate(profile: dict) -> dict:
    if profile.get("schema") != "mycelix-treasury-credit-downstream-observed-profile-v1":
        raise ValueError("profile schema drift")
    if profile.get("profile_id") != PROFILE_ID or profile.get("profile_revision") != 1:
        raise ValueError("profile identity drift")
    if profile.get("authority_class") != AUTHORITY:
        raise ValueError("profile authority promotion/drift")
    if profile.get("profile_content_sha256") != PROFILE_SHA256:
        raise ValueError("profile commitment field drift")
    if payload_digest(profile) != PROFILE_SHA256:
        raise ValueError("profile payload commitment mismatch")

    source = profile.get("source_binding", {})
    if source.get("repository") != "Luminous-Dynamics/mycelix":
        raise ValueError("repository drift")
    if source.get("semantic_production_subject_sha") != PRODUCTION_SUBJECT:
        raise ValueError("semantic production subject drift")
    if source.get("evidence_authoring_source_equivalent_head") != AUTHORING_HEAD:
        raise ValueError("evidence-authoring head drift")
    if source.get("source_equivalence_scope") != "ExactBoundFilesOnly":
        raise ValueError("source equivalence must remain file-scoped")

    if source.get("execution_coordinator") != {
        "path": "mycelix-governance/zomes/execution/coordinator/src/lib.rs",
        "git_blob_sha1": "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    }:
        raise ValueError("execution source binding drift")
    if source.get("finance_treasury_coordinator") != {
        "path": "mycelix-finance/zomes/treasury/coordinator/src/lib.rs",
        "git_blob_sha1": "840e66bcb6fdedb27fe2451752511d1317eb50b8",
    }:
        raise ValueError("Treasury coordinator source binding drift")
    if source.get("finance_treasury_integrity") != {
        "path": "mycelix-finance/zomes/treasury/integrity/src/lib.rs",
        "git_blob_sha1": "5ee9b72f7c138a5283818b873ceae36dff3305d4",
    }:
        raise ValueError("Treasury integrity source binding drift")

    bridge = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("governance_bridge_coordinator_census", [])
        if isinstance(item, dict)
    }
    if bridge != BRIDGE_FILES or len(source.get("governance_bridge_coordinator_census", [])) != 8:
        raise ValueError("governance bridge module census drift")

    legacy = profile.get("legacy_execution_transfer", {})
    if legacy.get("action_variant") != "TransferCredits":
        raise ValueError("legacy action drift")
    if legacy.get("payload_fields") != ["from", "to", "amount"]:
        raise ValueError("legacy payload drift")
    if legacy.get("amount_representation") != "f64":
        raise ValueError("legacy amount representation drift")
    if legacy.get("validation") != [
        "FromNonEmpty", "ToNonEmpty", "AmountPositive", "AmountFinite"
    ]:
        raise ValueError("legacy validation drift")
    if legacy.get("dispatch") != {
        "call_type": "Local", "zome": "governance_bridge", "function": "transfer_credits"
    }:
        raise ValueError("legacy dispatch drift")
    legacy_target = legacy.get("bridge_target_observation", {})
    if legacy_target != {
        "exact_module_count": 8,
        "target_occurrences": 0,
        "absence_scope": "ExactBoundGovernanceBridgeCoordinatorCensus",
        "on_call_error": "ActionFailsClosed",
    }:
        raise ValueError("legacy target observation drift")

    bridge_finance = profile.get("governance_bridge_finance_transfer", {})
    if bridge_finance.get("entrypoint") != "execute_approved_transfer":
        raise ValueError("bridge Finance entrypoint drift")
    if bridge_finance.get("input_fields") != [
        "proposal_hash", "recipient_did", "amount_sap", "purpose"
    ]:
        raise ValueError("bridge Finance payload drift")
    if bridge_finance.get("dispatch") != {
        "call_type": "OtherRole",
        "role": "finance",
        "zome": "treasury",
        "function": "execute_governance_transfer",
    }:
        raise ValueError("bridge Finance dispatch drift")
    if bridge_finance.get("treasury_target_observation") != {
        "target_occurrences": 0,
        "absence_scope": "ExactBoundFinanceTreasuryCoordinator",
        "on_role_call_error": "ExternReturnsError",
    }:
        raise ValueError("Treasury target observation drift")

    plane = profile.get("treasury_allocation_plane", {})
    if plane.get("propose_allocation") != {
        "proposal_id": "OptionalCallerSupplied",
        "proposal_authority_reconstruction": "NoneObserved",
    }:
        raise ValueError("propose-allocation semantics drift")
    if plane.get("approve_allocation") != {
        "required_status": "Proposed",
        "caller_did_binding": True,
        "approver_must_be_manager": True,
        "approval_rule": "ManagerMajority",
    }:
        raise ValueError("approval semantics drift")
    if plane.get("execute_allocation") != {
        "required_status": "Approved",
        "caller_role_check": "NoneObserved",
        "effect": "DebitTreasuryThenMarkExecuted",
        "debit_underflow": "CheckedSubRejects",
        "optimistic_rmw_retries": 3,
    }:
        raise ValueError("execution allocation semantics drift")
    if plane.get("large_dkg_allocation") != {
        "threshold_micro_sap": 10_000_000_000,
        "governance_signature_verification": "FailClosedCrossRoleCall",
        "proposal_id_recorded": False,
        "separate_path": True,
    }:
        raise ValueError("DKG allocation semantics drift")

    integrity = profile.get("treasury_integrity_plane", {})
    if integrity.get("validate_update_treasury") != {
        "checks": ["ReserveRatioFinite", "ReserveRatioUnitInterval"],
        "author_binding": "NoneObserved",
        "balance_authority_reconstruction": "NoneObserved",
        "manager_authority_reconstruction": "NoneObserved",
        "original_state_transition_binding": "NoneObserved",
    }:
        raise ValueError("Treasury integrity semantics drift")
    if integrity.get("validate_create_allocation") != {
        "checks": ["RecipientDidShape", "StringLengthBounds", "AmountPositive"],
        "author_binding": "NoneObserved",
        "proposal_authority_reconstruction": "NoneObserved",
        "manager_authority_reconstruction": "NoneObserved",
    }:
        raise ValueError("Allocation create integrity semantics drift")
    if integrity.get("validate_update_allocation") != {
        "checks": ["AmountPositive"],
        "author_binding": "NoneObserved",
        "status_transition_graph": "NoneObserved",
        "approved_by_authority_reconstruction": "NoneObserved",
        "proposal_authority_reconstruction": "NoneObserved",
        "immutable_field_binding": "NoneObserved",
    }:
        raise ValueError("Allocation update integrity semantics drift")

    if profile.get("known_gaps") != [
        {"issue": 1085, "class": "GovernanceTreasuryDispatchAuthorityContinuityGap", "status": "Observed"},
        {"issue": 1086, "class": "TreasuryAllocationIntegrityAuthorityGap", "status": "Observed"},
    ]:
        raise ValueError("known gap registry drift")

    expected_positive = [
        "LegacyTransferCallFailsClosedOnMissingTarget",
        "BridgeFinanceCallFailsClosedOnMissingTarget",
        "TransferAmountMustBePositiveAndFinite",
        "TreasuryDebitUsesCheckedSub",
        "CanonicalAllocationCoordinatorUsesManagerMajorityApproval",
        "LargeDkgAllocationFailsClosedOnSignatureVerificationFailure",
    ]
    if profile.get("positive_containment") != expected_positive:
        raise ValueError("positive-containment registry drift")

    expected_unqualified = [
        "AuthorizedGovernanceValueTransfer",
        "GovernanceToTreasuryAuthorityContinuity",
        "DecentralizedTreasuryMutationAuthority",
        "DecentralizedAllocationTransitionAuthority",
        "TreasuryCreditDeploymentCurrentnessQualified",
        "TreasuryCreditGovernanceSafety",
    ]
    if profile.get("unsupported_or_unqualified") != expected_unqualified:
        raise ValueError("unqualified-property registry drift")

    return {
        "profile_id": PROFILE_ID,
        "profile_content_sha256": PROFILE_SHA256,
        "authority_class": AUTHORITY,
        "semantic_production_subject_sha": PRODUCTION_SUBJECT,
        "evidence_authoring_source_equivalent_head": AUTHORING_HEAD,
        "bridge_module_count": 8,
        "known_issues": [1085, 1086],
        "positive_containment_count": len(expected_positive),
        "unqualified_property_count": len(expected_unqualified),
        "valid": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(validate(load(args.profile)), sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
