#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-treasury-asset-unit-observed-fca2c107-v1"
PROFILE_SHA256 = "58280f735acecd84071b71876c5b38d12d6776b8505016b22a020a05cca2ffb5"
PRODUCTION_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
AUTHORING_HEAD = "feb30a89257e96592fdbd40b249258581d42fde7"
AUTHORITY = "ObservedSourceBound"


def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


def load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("profile root must be object")
    return value


def payload_digest(profile: dict) -> str:
    payload = dict(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()


def eq(actual: object, expected: object, name: str) -> None:
    if actual != expected:
        raise ValueError(f"{name} drift: {actual!r}")


def validate(profile: dict) -> dict:
    eq(profile.get("schema"), "mycelix-treasury-asset-unit-observed-profile-v1", "schema")
    eq(profile.get("profile_id"), PROFILE_ID, "profile id")
    eq(profile.get("profile_revision"), 1, "profile revision")
    eq(profile.get("authority_class"), AUTHORITY, "authority")
    eq(profile.get("profile_content_sha256"), PROFILE_SHA256, "commitment field")
    eq(payload_digest(profile), PROFILE_SHA256, "payload commitment")

    source = profile.get("source_binding", {})
    eq(source.get("repository"), "Luminous-Dynamics/mycelix", "repository")
    eq(source.get("semantic_production_subject_sha"), PRODUCTION_SUBJECT, "semantic subject")
    eq(source.get("evidence_authoring_head"), AUTHORING_HEAD, "authoring head")
    eq(source.get("treasury_coordinator"), {
        "path": "mycelix-finance/zomes/treasury/coordinator/src/lib.rs",
        "git_blob_sha1": "840e66bcb6fdedb27fe2451752511d1317eb50b8",
    }, "Treasury coordinator")
    eq(source.get("treasury_integrity"), {
        "path": "mycelix-finance/zomes/treasury/integrity/src/lib.rs",
        "git_blob_sha1": "5ee9b72f7c138a5283818b873ceae36dff3305d4",
    }, "Treasury integrity")
    eq(source.get("finance_types"), {
        "path": "mycelix-finance/types/src/lib.rs",
        "git_blob_sha1": "354a377c88a57e8a5612a518d8c8b8b519fb67e2",
    }, "Finance domain types")

    eq(profile.get("finance_domain_semantics"), {
        "canonical_currency_type": "mycelix_finance_types::Currency",
        "currency_variants": ["Mycel", "Sap", "Tend"],
        "mycel_semantics": "NonTransferableReputationSubstrate",
        "sap_semantics": "TransferableCirculationMedium",
        "tend_semantics": "TransferableMutualCredit",
        "canonical_quantity_type": "NoneObserved",
        "treasury_consumes_canonical_currency_type": False,
    }, "Finance domain semantics")

    eq(profile.get("treasury_asset_semantics"), {
        "storage_field": "currency:String",
        "create_input": "currency:String",
        "canonical_asset_id": "CurrencyEnumExistsButNotConsumedByTreasury",
        "base_unit_scale": "NoneObserved",
        "create_integrity_asset_binding": "NoneObserved",
        "update_integrity_asset_binding": "NoneObserved",
        "asset_identity_immutability": "NoneObserved",
    }, "Treasury asset semantics")

    eq(profile.get("contribution_semantics"), {
        "input_currency": "currency:String",
        "balance_mutation_helper": "credit_treasury(treasury_id, amount)",
        "currency_equality_to_treasury": "NoneObserved",
        "integrity_asset_binding": "NoneObserved",
    }, "Contribution semantics")

    eq(profile.get("allocation_semantics"), {
        "input_currency": "currency:String",
        "execution_balance_mutation": "debit_treasury(alloc.treasury_id, alloc.amount)",
        "currency_equality_to_treasury": "NoneObserved",
        "create_integrity_asset_binding": "NoneObserved",
        "update_integrity_asset_binding": "NoneObserved",
        "asset_identity_immutability": "NoneObserved",
    }, "Allocation semantics")

    eq(profile.get("dkg_semantics"), {
        "amount_documented_unit": "micro-SAP",
        "threshold_base_units": 10_000_000_000,
        "threshold_documented_unit": "micro-SAP",
        "resulting_allocation_currency": "treasury.currency.clone()",
        "threshold_asset_binding": "NotEstablished",
        "threshold_base_unit_scale_binding": "NotEstablished",
        "signed_subject_template": "treasury_allocation:{treasury_id}:{amount}:{recipient_did}",
        "signed_subject_binds": ["treasury_id", "amount", "recipient_did"],
        "signed_subject_explicit_asset_id_binding": "NoneObserved",
        "signed_subject_explicit_currency_binding": "NoneObserved",
        "signed_subject_explicit_base_unit_scale_binding": "NoneObserved",
        "signed_subject_explicit_policy_revision_binding": "NoneObserved",
    }, "DKG semantics")

    positive = [
        "TreasuryBalanceUsesU64",
        "CreditUsesCheckedAdd",
        "DebitUsesCheckedSub",
        "InsufficientBalanceFailsClosed",
        "DkgThresholdExplicitlyDocumentedAsMicroSap",
        "CanonicalFinanceCurrencyEnumExists",
    ]
    eq(profile.get("positive_controls"), positive, "positive controls")
    eq(profile.get("known_gaps"), [
        {"issue": 1222, "class": "CanonicalTreasuryAssetUnitBindingGap", "status": "Observed"}
    ], "known gaps")
    unqualified = [
        "CanonicalTreasuryAssetUnitBinding",
        "TreasuryUsesCanonicalFinanceCurrencyType",
        "ContributionToTreasuryAssetEquality",
        "AllocationToTreasuryAssetEquality",
        "TreasuryAssetIdentityImmutability",
        "DkgThresholdAssetBinding",
        "DkgSignedSubjectAssetUnitPolicyBinding",
        "TreasuryAssetUnitDeploymentCurrentnessQualified",
    ]
    eq(profile.get("unsupported_or_unqualified"), unqualified, "unqualified properties")
    eq(profile.get("non_claims"), [
        "no_live_cross_currency_transfer",
        "no_stolen_funds_claim",
        "no_deployment_exploit",
        "no_deployment_currentness",
        "no_treasury_safety_claim",
    ], "non-claims")

    return {
        "profile_id": PROFILE_ID,
        "profile_content_sha256": PROFILE_SHA256,
        "authority_class": AUTHORITY,
        "semantic_production_subject_sha": PRODUCTION_SUBJECT,
        "evidence_authoring_head": AUTHORING_HEAD,
        "known_issues": [1222],
        "canonical_finance_currency_enum_bound": True,
        "treasury_consumes_canonical_currency_type": False,
        "positive_control_count": len(positive),
        "unqualified_property_count": len(unqualified),
        "dkg_signed_subject_asset_unit_policy_binding": "NotEstablished",
        "canonical_treasury_asset_unit_binding": "NotEstablished",
        "valid": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(validate(load(args.profile)), sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
