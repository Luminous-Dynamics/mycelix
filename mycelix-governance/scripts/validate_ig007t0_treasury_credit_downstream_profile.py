#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-treasury-credit-downstream-observed-fca2c107-v1"
PROFILE_SHA256 = "77db81e40ebe7b0ec9278f8c9aad7584faa08b6784cde4779393d1d6b88cf58d"
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
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError("profile root must be object")
    return value


def payload_digest(profile: dict) -> str:
    payload = dict(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def require_equal(actual: object, expected: object, name: str) -> None:
    if actual != expected:
        raise ValueError(f"{name} drift: {actual!r}")


def validate(profile: dict) -> dict:
    require_equal(profile.get("schema"), "mycelix-treasury-credit-downstream-observed-profile-v1", "schema")
    require_equal(profile.get("profile_id"), PROFILE_ID, "profile id")
    require_equal(profile.get("profile_revision"), 1, "profile revision")
    require_equal(profile.get("authority_class"), AUTHORITY, "authority")
    require_equal(profile.get("profile_content_sha256"), PROFILE_SHA256, "commitment field")
    require_equal(payload_digest(profile), PROFILE_SHA256, "payload commitment")

    source = profile.get("source_binding", {})
    require_equal(source.get("repository"), "Luminous-Dynamics/mycelix", "repository")
    require_equal(source.get("semantic_production_subject_sha"), PRODUCTION_SUBJECT, "semantic subject")
    require_equal(source.get("evidence_authoring_source_equivalent_head"), AUTHORING_HEAD, "authoring head")
    require_equal(source.get("source_equivalence_scope"), "ExactBoundFilesOnly", "source-equivalence scope")
    require_equal(source.get("execution_coordinator"), {
        "path": "mycelix-governance/zomes/execution/coordinator/src/lib.rs",
        "git_blob_sha1": "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    }, "execution source")
    require_equal(source.get("finance_treasury_coordinator"), {
        "path": "mycelix-finance/zomes/treasury/coordinator/src/lib.rs",
        "git_blob_sha1": "840e66bcb6fdedb27fe2451752511d1317eb50b8",
    }, "Treasury coordinator source")
    require_equal(source.get("finance_treasury_integrity"), {
        "path": "mycelix-finance/zomes/treasury/integrity/src/lib.rs",
        "git_blob_sha1": "5ee9b72f7c138a5283818b873ceae36dff3305d4",
    }, "Treasury integrity source")
    require_equal(source.get("governance_utils"), {
        "path": "mycelix-governance/crates/governance-utils/src/lib.rs",
        "git_blob_sha1": "282888816cc101c2743ef5c5905119defc3fee6d",
    }, "governance-utils source")
    bridge = {
        item.get("path"): item.get("git_blob_sha1")
        for item in source.get("governance_bridge_coordinator_census", [])
        if isinstance(item, dict)
    }
    require_equal(bridge, BRIDGE_FILES, "bridge module census")
    require_equal(len(source.get("governance_bridge_coordinator_census", [])), 8, "bridge module count")

    legacy = profile.get("legacy_execution_transfer", {})
    require_equal(legacy.get("action_variant"), "TransferCredits", "legacy action")
    require_equal(legacy.get("payload_fields"), ["from", "to", "amount"], "legacy payload")
    require_equal(legacy.get("amount_representation"), "f64", "legacy amount representation")
    require_equal(legacy.get("validation"), ["FromNonEmpty", "ToNonEmpty", "AmountPositive", "AmountFinite"], "legacy validation")
    require_equal(legacy.get("dispatch"), {"call_type": "Local", "zome": "governance_bridge", "function": "transfer_credits"}, "legacy dispatch")
    require_equal(legacy.get("bridge_target_observation"), {
        "exact_module_count": 8,
        "target_occurrences": 0,
        "absence_scope": "ExactBoundGovernanceBridgeCoordinatorCensus",
        "on_call_error": "ActionFailsClosed",
    }, "legacy target observation")

    finance = profile.get("governance_bridge_finance_transfer", {})
    require_equal(finance.get("entrypoint"), "execute_approved_transfer", "Finance bridge entrypoint")
    require_equal(finance.get("input_fields"), ["proposal_hash", "recipient_did", "amount_sap", "purpose"], "Finance bridge payload")
    require_equal(finance.get("dispatch"), {
        "call_type": "OtherRole", "role": "finance", "zome": "treasury", "function": "execute_governance_transfer"
    }, "Finance bridge dispatch")
    require_equal(finance.get("treasury_target_observation"), {
        "target_occurrences": 0,
        "absence_scope": "ExactBoundFinanceTreasuryCoordinator",
        "on_role_call_error": "ExternReturnsError",
        "error_helper": "governance_utils::call_role",
        "helper_failure_semantics": "ReturnsErrOnTransportNetworkOrUnexpectedResponse",
    }, "Treasury target observation")

    plane = profile.get("treasury_allocation_plane", {})
    require_equal(plane.get("propose_allocation"), {
        "proposal_id": "OptionalCallerSupplied", "proposal_authority_reconstruction": "NoneObserved"
    }, "propose-allocation semantics")
    require_equal(plane.get("approve_allocation"), {
        "required_status": "Proposed", "caller_did_binding": True,
        "approver_must_be_manager": True, "approval_rule": "ManagerMajority"
    }, "approve-allocation semantics")
    require_equal(plane.get("execute_allocation"), {
        "required_status": "Approved", "caller_role_check": "NoneObserved",
        "effect": "DebitTreasuryThenMarkExecuted", "debit_underflow": "CheckedSubRejects",
        "optimistic_rmw_retries": 3
    }, "execute-allocation semantics")
    require_equal(plane.get("large_dkg_allocation"), {
        "threshold_micro_sap": 10_000_000_000,
        "governance_signature_verification": "FailClosedCrossRoleCall",
        "proposal_id_recorded": False, "separate_path": True
    }, "DKG allocation semantics")

    integrity = profile.get("treasury_integrity_plane", {})
    require_equal(integrity.get("validate_update_treasury"), {
        "checks": ["ReserveRatioFinite", "ReserveRatioUnitInterval"],
        "author_binding": "NoneObserved", "balance_authority_reconstruction": "NoneObserved",
        "manager_authority_reconstruction": "NoneObserved", "original_state_transition_binding": "NoneObserved"
    }, "Treasury integrity")
    require_equal(integrity.get("validate_create_allocation"), {
        "checks": ["RecipientDidShape", "StringLengthBounds", "AmountPositive"],
        "author_binding": "NoneObserved", "proposal_authority_reconstruction": "NoneObserved",
        "manager_authority_reconstruction": "NoneObserved"
    }, "Allocation create integrity")
    require_equal(integrity.get("validate_update_allocation"), {
        "checks": ["AmountPositive"], "author_binding": "NoneObserved",
        "status_transition_graph": "NoneObserved", "approved_by_authority_reconstruction": "NoneObserved",
        "proposal_authority_reconstruction": "NoneObserved", "immutable_field_binding": "NoneObserved"
    }, "Allocation update integrity")

    require_equal(profile.get("known_gaps"), [
        {"issue": 1085, "class": "GovernanceTreasuryDispatchAuthorityContinuityGap", "status": "Observed"},
        {"issue": 1086, "class": "TreasuryAllocationIntegrityAuthorityGap", "status": "Observed"},
    ], "known gaps")

    positive = [
        "LegacyTransferCallFailsClosedOnMissingTarget",
        "BridgeFinanceCallFailsClosedOnMissingTarget",
        "TransferAmountMustBePositiveAndFinite",
        "TreasuryDebitUsesCheckedSub",
        "CanonicalAllocationCoordinatorUsesManagerMajorityApproval",
        "LargeDkgAllocationFailsClosedOnSignatureVerificationFailure",
    ]
    require_equal(profile.get("positive_containment"), positive, "positive containment")
    unqualified = [
        "AuthorizedGovernanceValueTransfer",
        "GovernanceToTreasuryAuthorityContinuity",
        "DecentralizedTreasuryMutationAuthority",
        "DecentralizedAllocationTransitionAuthority",
        "TreasuryCreditDeploymentCurrentnessQualified",
        "TreasuryCreditGovernanceSafety",
    ]
    require_equal(profile.get("unsupported_or_unqualified"), unqualified, "unqualified properties")

    return {
        "profile_id": PROFILE_ID,
        "profile_content_sha256": PROFILE_SHA256,
        "authority_class": AUTHORITY,
        "semantic_production_subject_sha": PRODUCTION_SUBJECT,
        "evidence_authoring_source_equivalent_head": AUTHORING_HEAD,
        "bridge_module_count": 8,
        "governance_utils_bound": True,
        "known_issues": [1085, 1086],
        "positive_containment_count": len(positive),
        "unqualified_property_count": len(unqualified),
        "valid": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(validate(load(args.profile)), sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
