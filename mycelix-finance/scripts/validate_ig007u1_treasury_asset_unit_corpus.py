#!/usr/bin/env python3
import argparse, hashlib, json
from pathlib import Path

EXPECTED_U0_PROFILE_CONTENT_SHA256 = "58280f735acecd84071b71876c5b38d12d6776b8505016b22a020a05cca2ffb5"
EXPECTED_IDS = [
    "CanonicalFinanceCurrencyEnumExists",
    "TreasuryDoesNotConsumeCanonicalFinanceCurrencyType",
    "CanonicalFinanceQuantityTypeNotEstablished",
    "TreasuryAssetIdentityImmutabilityNotEstablished",
    "ContributionToTreasuryAssetEqualityNotEstablished",
    "AllocationToTreasuryAssetEqualityNotEstablished",
    "DkgThresholdAssetBindingNotEstablished",
    "DkgSignedSubjectAssetUnitPolicyBindingNotEstablished",
]
COMMON_NONCLAIMS = {
    "LiveCrossCurrencyTransfer", "StolenFunds", "DeploymentExploit",
    "DeploymentCurrentness", "OverallTreasurySafety", "SuccessorSapAmountCorrectness",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    return Path(path).read_text(encoding="utf-8")


def canonical(obj):
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def git_blob_sha1(path):
    raw = Path(path).read_bytes()
    framed = b"blob " + str(len(raw)).encode("ascii") + b"\0" + raw
    return hashlib.sha1(framed).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--corpus", required=True)
    p.add_argument("--profile", required=True)
    p.add_argument("--coordinator", required=True)
    p.add_argument("--integrity", required=True)
    p.add_argument("--finance-types", required=True)
    args = p.parse_args()

    raw = Path(args.corpus).read_bytes()
    corpus = json.loads(raw)
    profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
    coordinator = read(args.coordinator)
    integrity = read(args.integrity)
    finance = read(args.finance_types)

    require(corpus.get("schema") == "mycelix-treasury-asset-unit-source-contract-corpus-v1", "wrong corpus schema")
    require(corpus.get("authority_class") == "HistoricalSourceContractEvidence", "wrong authority class")
    receipts = corpus.get("receipts")
    require(isinstance(receipts, list), "receipts must be a list")
    require(corpus.get("receipt_count") == 8 == len(receipts), "receipt count mismatch")
    require([r.get("id") for r in receipts] == EXPECTED_IDS, "receipt identities/order drift")
    require(set(corpus.get("non_claims", [])) == COMMON_NONCLAIMS, "common non-claims drift")
    for receipt in receipts:
        rid = receipt.get("id", "<unknown>")
        require(
            receipt.get("qualification_result") in {
                "PropertyEstablishedWithinBoundSource",
                "PropertyNotEstablishedWithinBoundSource",
            },
            f"invalid qualification result for {rid}",
        )
        require(
            COMMON_NONCLAIMS.issubset(set(receipt.get("forbidden_conclusions", []))),
            f"missing forbidden conclusions for {rid}",
        )
        require(bool(receipt.get("observed_source_predicates")), f"missing observed predicates for {rid}")
        require(bool(receipt.get("positive_controls")), f"missing positive controls for {rid}")
        require(bool(receipt.get("required_property")), f"missing required property for {rid}")

    binding = corpus.get("source_binding")
    require(isinstance(binding, dict), "source_binding must be an object")
    require(
        binding.get("semantic_production_subject_sha") == "fca2c107a1ea5108823ce617ba4111b6f7f77230",
        "semantic production subject drift",
    )
    require(
        binding.get("u0_evidence_head") == "fd801d9b46515d333eccd3f42a1664042a1b06d5",
        "U0 evidence-head drift",
    )

    # Reproduce Git's blob-object identity directly from supplied bytes so this
    # validator does not need a Git checkout to prove the source binding.
    require(binding.get("treasury_coordinator_blob") == git_blob_sha1(args.coordinator), "Treasury coordinator blob mismatch")
    require(binding.get("treasury_integrity_blob") == git_blob_sha1(args.integrity), "Treasury integrity blob mismatch")
    require(binding.get("finance_types_blob") == git_blob_sha1(args.finance_types), "Finance types blob mismatch")

    # U0's commitment is over canonical semantic payload bytes, not over the
    # literal self-describing JSON file. Reproduce that exact qualified rule.
    require(isinstance(profile, dict), "profile root must be an object")
    profile_payload = dict(profile)
    declared_profile_commitment = profile_payload.pop("profile_content_sha256", None)
    derived_profile_commitment = hashlib.sha256(canonical(profile_payload)).hexdigest()
    require(declared_profile_commitment == EXPECTED_U0_PROFILE_CONTENT_SHA256, "declared U0 profile commitment drift")
    require(derived_profile_commitment == EXPECTED_U0_PROFILE_CONTENT_SHA256, "derived U0 profile commitment drift")
    require(binding.get("u0_profile_sha256") == EXPECTED_U0_PROFILE_CONTENT_SHA256, "corpus U0 profile binding drift")
    profile_source = profile.get("source_binding")
    require(isinstance(profile_source, dict), "profile source_binding must be an object")
    require(
        profile_source.get("semantic_production_subject_sha") == binding.get("semantic_production_subject_sha"),
        "profile/corpus semantic subject mismatch",
    )

    # Positive control: canonical Finance currency namespace exists and lanes differ.
    require("pub enum Currency {" in finance, "canonical Currency enum not found")
    block = finance.split("pub enum Currency {", 1)[1].split("}", 1)[0]
    for variant in ("Mycel", "Sap", "Tend"):
        require(variant in block, f"Currency::{variant} not found")
    require("pub fn is_transferable(&self) -> bool {" in finance, "Currency transferability function not found")
    transfer = finance.split("pub fn is_transferable(&self) -> bool {", 1)[1].split("}", 2)[0]
    require("Currency::Mycel => false" in transfer, "MYCEL transferability semantic drift")
    require("Currency::Sap => true" in transfer, "SAP transferability semantic drift")
    require("Currency::Tend => true" in transfer, "TEND transferability semantic drift")

    # Treasury remains stringly typed on the frozen subject.
    require("pub currency: String" in coordinator, "Treasury String currency field not found")
    require("mycelix_finance_types::Currency" not in coordinator, "Treasury now consumes canonical Currency type")
    require("pub fn create_treasury(" in coordinator and "pub struct CreateTreasuryInput" in coordinator, "create_treasury boundary not found")
    create = coordinator.split("pub fn create_treasury(", 1)[1].split("pub struct CreateTreasuryInput", 1)[0]
    require("currency: input.currency" in create, "create_treasury currency flow drift")

    require("pub fn contribute(" in coordinator and "pub struct ContributeInput" in coordinator, "contribute boundary not found")
    contribute = coordinator.split("pub fn contribute(", 1)[1].split("pub struct ContributeInput", 1)[0]
    require("currency: input.currency" in contribute, "contribution currency flow drift")
    require("credit_treasury(&input.treasury_id, input.amount)" in contribute, "contribution credit path drift")
    require("treasury.currency" not in contribute, "contribution now checks Treasury currency in bound slice")
    require(".checked_add(amount)" in coordinator, "checked-add positive control missing")

    require("pub fn propose_allocation(" in coordinator and "pub struct ProposeAllocationInput" in coordinator, "proposal boundary not found")
    propose = coordinator.split("pub fn propose_allocation(", 1)[1].split("pub struct ProposeAllocationInput", 1)[0]
    require("currency: input.currency" in propose, "allocation currency flow drift")
    require("pub fn execute_allocation(" in coordinator and "/// Internal helper: fetch an allocation" in coordinator, "allocation execution boundary not found")
    ordinary = coordinator.split("pub fn execute_allocation(", 1)[1].split("/// Internal helper: fetch an allocation", 1)[0]
    require("debit_treasury(&alloc.treasury_id, alloc.amount)" in ordinary, "allocation debit path drift")
    require("alloc.currency" not in ordinary, "allocation execution now checks allocation currency in bound slice")
    require(".checked_sub(amount)" in coordinator, "checked-sub positive control missing")
    require("Insufficient treasury balance" in coordinator, "insufficient-balance fail-closed control missing")

    require("pub fn execute_dkg_allocation(" in coordinator and "/// Input for a DKG-gated treasury allocation." in coordinator, "DKG allocation boundary not found")
    dkg = coordinator.split("pub fn execute_dkg_allocation(", 1)[1].split("/// Input for a DKG-gated treasury allocation.", 1)[0]
    require("const DKG_THRESHOLD_AMOUNT: u64 = 10_000_000_000;" in coordinator, "DKG threshold drift")
    require("Minimum amount (in micro-SAP)" in coordinator, "DKG threshold unit documentation missing")
    require("Amount in micro-SAP" in coordinator, "DKG amount unit documentation missing")
    require("currency: treasury.currency.clone()" in dkg, "DKG allocation currency flow drift")
    require("let message = format!(" in dkg, "DKG signed-subject message not found")
    message = dkg.split("let message = format!(", 1)[1].split(");", 1)[0]
    for required in ("input.treasury_id", "input.amount", "input.recipient_did"):
        require(required in message, f"DKG signed subject missing {required}")
    for forbidden in ("asset_id", "currency", "base_unit", "policy_revision"):
        require(forbidden not in message, f"DKG signed subject unexpectedly binds {forbidden}")

    for marker in (
        "fn validate_create_treasury(",
        "fn validate_update_treasury(",
        "fn validate_create_contribution(",
        "fn validate_create_allocation(",
        "fn validate_update_allocation(",
        "fn validate_create_savings_pool(",
    ):
        require(marker in integrity, f"integrity boundary not found: {marker}")
    cti = integrity.split("fn validate_create_treasury(", 1)[1].split("fn validate_update_treasury(", 1)[0]
    uti = integrity.split("fn validate_update_treasury(", 1)[1].split("fn validate_create_contribution(", 1)[0]
    cai = integrity.split("fn validate_create_allocation(", 1)[1].split("fn validate_update_allocation(", 1)[0]
    uai = integrity.split("fn validate_update_allocation(", 1)[1].split("fn validate_create_savings_pool(", 1)[0]
    require("treasury.currency" not in cti and "treasury.currency" not in uti, "Treasury integrity now binds currency in frozen slices")
    require("allocation.currency" not in cai and "allocation.currency" not in uai, "Allocation integrity now binds currency in frozen slices")

    result = {
        "valid": True,
        "corpus_sha256": hashlib.sha256(raw).hexdigest(),
        "receipt_count": len(receipts),
        "receipt_ids": EXPECTED_IDS,
        "semantic_production_subject_sha": binding["semantic_production_subject_sha"],
        "u0_profile_content_sha256": derived_profile_commitment,
        "source_blob_binding_count": 3,
        "authority_class": corpus["authority_class"],
        "nonclaim_count": len(COMMON_NONCLAIMS),
    }
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
