#!/usr/bin/env python3
import argparse
import hashlib
import json
import re
from pathlib import Path

EXPECTED_CORPUS_SHA256 = "b9212ce3da3cfed4b69335b59cf3c4a15e76d1c95819f3628bcc2a239d732190"
EXPECTED_U0_PROFILE_CONTENT_SHA256 = "58280f735acecd84071b71876c5b38d12d6776b8505016b22a020a05cca2ffb5"
EXPECTED_SEMANTIC_SUBJECT = "fca2c107a1ea5108823ce617ba4111b6f7f77230"
EXPECTED_U0_HEAD = "fd801d9b46515d333eccd3f42a1664042a1b06d5"
EXPECTED_SOURCE_BLOBS = {
    "treasury_coordinator_blob": "840e66bcb6fdedb27fe2451752511d1317eb50b8",
    "treasury_integrity_blob": "5ee9b72f7c138a5283818b873ceae36dff3305d4",
    "finance_types_blob": "354a377c88a57e8a5612a518d8c8b8b519fb67e2",
}
COMMON_NONCLAIMS = {
    "LiveCrossCurrencyTransfer",
    "StolenFunds",
    "DeploymentExploit",
    "DeploymentCurrentness",
    "OverallTreasurySafety",
    "SuccessorSapAmountCorrectness",
}
EXPECTED_RECEIPTS = [
    {
        "id": "CanonicalFinanceCurrencyEnumExists",
        "observed_source_predicates": [
            "mycelix_finance_types defines Currency::{Mycel,Sap,Tend}",
            "Currency::Mycel is non-transferable",
            "Currency::Sap is transferable",
            "Currency::Tend is transferable",
        ],
        "positive_controls": [
            "CanonicalFinanceCurrencyEnumExists",
            "DistinctLaneTransferSemantics",
        ],
        "required_property": "A canonical Finance currency namespace with distinct lane transfer semantics exists in the bound source.",
    },
    {
        "id": "TreasuryDoesNotConsumeCanonicalFinanceCurrencyType",
        "observed_source_predicates": [
            "Treasury create/input/storage uses currency:String",
            "Treasury coordinator does not reference mycelix_finance_types::Currency",
        ],
        "positive_controls": [
            "CanonicalFinanceCurrencyEnumExists",
            "TreasuryBalanceUsesU64",
        ],
        "required_property": "The bound Treasury implementation does not consume the canonical Finance Currency type.",
    },
    {
        "id": "CanonicalFinanceQuantityTypeNotEstablished",
        "observed_source_predicates": [
            "Bound Finance types defines canonical Currency enum",
            "U0 source census observes no canonical per-currency quantity/base-unit authority consumed by Treasury",
        ],
        "positive_controls": [
            "CanonicalFinanceCurrencyEnumExists",
            "DkgThresholdExplicitlyDocumentedAsMicroSap",
        ],
        "required_property": "A canonical Finance quantity/base-unit type used by Treasury is not established within the bound source.",
    },
    {
        "id": "TreasuryAssetIdentityImmutabilityNotEstablished",
        "observed_source_predicates": [
            "Treasury stores free-form currency:String",
            "Treasury create/update integrity does not reconstruct typed currency equality or immutability",
        ],
        "positive_controls": [
            "TreasuryBalanceUsesU64",
            "CreditUsesCheckedAdd",
            "DebitUsesCheckedSub",
        ],
        "required_property": "Treasury asset/currency identity immutability is not established within the bound source.",
    },
    {
        "id": "ContributionToTreasuryAssetEqualityNotEstablished",
        "observed_source_predicates": [
            "ContributeInput carries currency:String",
            "contribute records input.currency",
            "credit_treasury mutates by treasury_id and numeric amount",
            "No bound source-visible equality check ties contribution currency to Treasury currency before credit",
        ],
        "positive_controls": [
            "CreditUsesCheckedAdd",
            "CanonicalFinanceCurrencyEnumExists",
        ],
        "required_property": "Contribution-to-Treasury asset/currency equality is not established within the bound source.",
    },
    {
        "id": "AllocationToTreasuryAssetEqualityNotEstablished",
        "observed_source_predicates": [
            "ProposeAllocationInput carries currency:String",
            "Allocation stores caller-supplied currency",
            "execute_allocation debits treasury_id and alloc.amount",
            "No bound source-visible equality check ties Allocation currency to Treasury currency before debit",
        ],
        "positive_controls": [
            "DebitUsesCheckedSub",
            "InsufficientBalanceFailsClosed",
        ],
        "required_property": "Allocation-to-Treasury asset/currency equality is not established within the bound source.",
    },
    {
        "id": "DkgThresholdAssetBindingNotEstablished",
        "observed_source_predicates": [
            "DKG_THRESHOLD_AMOUNT is 10000000000",
            "DKG threshold/input comments identify micro-SAP",
            "Treasury currency representation remains free-form String",
        ],
        "positive_controls": [
            "DkgThresholdExplicitlyDocumentedAsMicroSap",
            "CanonicalFinanceCurrencyEnumExists",
        ],
        "required_property": "The DKG high-value threshold is not cryptographically/type-bound to canonical SAP identity and base-unit scale within the bound source.",
    },
    {
        "id": "DkgSignedSubjectAssetUnitPolicyBindingNotEstablished",
        "observed_source_predicates": [
            "DKG signed subject template is treasury_allocation:{treasury_id}:{amount}:{recipient_did}",
            "Signed subject binds treasury_id, amount, recipient_did",
            "Signed subject does not explicitly bind canonical Currency, base-unit scale, or policy revision",
        ],
        "positive_controls": [
            "DkgSignedSubjectBindsTreasuryAmountRecipient",
            "DkgThresholdExplicitlyDocumentedAsMicroSap",
        ],
        "required_property": "Explicit canonical asset/unit/policy-revision binding in the DKG signed subject is not established within the bound source.",
    },
]


def require(condition, message):
    if not condition:
        raise SystemExit(message)


def read_text(path):
    return Path(path).read_text(encoding="utf-8")


def git_blob_sha1(path):
    raw = Path(path).read_bytes()
    framed = b"blob " + str(len(raw)).encode("ascii") + b"\0" + raw
    return hashlib.sha1(framed).hexdigest()


def canonical_json(obj):
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def between(text, start, end, label):
    start_at = text.find(start)
    require(start_at >= 0, f"independent oracle: missing start marker for {label}: {start}")
    end_at = text.find(end, start_at + len(start))
    require(end_at >= 0, f"independent oracle: missing end marker for {label}: {end}")
    return text[start_at:end_at]


def replace_once(text, old, new, label):
    require(text.count(old) == 1, f"independent oracle self-test fixture drift for {label}")
    return text.replace(old, new, 1)


def expect_rejection(label, function):
    try:
        function()
    except SystemExit:
        return label
    raise SystemExit(f"independent oracle negative control was incorrectly accepted: {label}")


def exact_receipt_contract(receipts):
    require(len(receipts) == len(EXPECTED_RECEIPTS), "independent oracle: receipt count drift")
    for actual, expected in zip(receipts, EXPECTED_RECEIPTS):
        rid = expected["id"]
        require(actual.get("id") == rid, f"independent oracle: receipt id/order drift at {rid}")
        require(
            actual.get("qualification_result") == "PropertyEstablishedWithinBoundSource",
            f"independent oracle: qualification result drift for {rid}",
        )
        require(
            actual.get("observed_source_predicates") == expected["observed_source_predicates"],
            f"independent oracle: observed predicate contract drift for {rid}",
        )
        require(
            actual.get("positive_controls") == expected["positive_controls"],
            f"independent oracle: positive-control contract drift for {rid}",
        )
        require(
            actual.get("required_property") == expected["required_property"],
            f"independent oracle: required-property contract drift for {rid}",
        )
        require(
            COMMON_NONCLAIMS.issubset(set(actual.get("forbidden_conclusions", []))),
            f"independent oracle: claim ceiling weakened for {rid}",
        )


def derive_source_contracts(coordinator, integrity, finance):
    checks = {}

    currency_enum = between(finance, "pub enum Currency {", "impl Currency {", "Currency enum")
    require(all(v in currency_enum for v in ("Mycel", "Sap", "Tend")), "independent oracle: canonical Currency variants drift")
    transferability = between(finance, "pub fn is_transferable(&self) -> bool {", "impl core::fmt::Display for Currency", "Currency transferability")
    require("Currency::Mycel => false" in transferability, "independent oracle: MYCEL transferability drift")
    require("Currency::Sap => true" in transferability, "independent oracle: SAP transferability drift")
    require("Currency::Tend => true" in transferability, "independent oracle: TEND transferability drift")
    checks["CanonicalFinanceCurrencyEnumExists"] = True

    treasury_entry = between(integrity, "pub struct Treasury {", "pub struct Contribution {", "Treasury entry")
    create_input = between(coordinator, "pub struct CreateTreasuryInput {", "pub fn contribute(", "CreateTreasuryInput")
    require("pub currency: String" in treasury_entry, "independent oracle: Treasury storage currency is no longer String")
    require("pub balance: u64" in treasury_entry, "independent oracle: Treasury balance no longer raw u64")
    require("pub currency: String" in create_input, "independent oracle: Treasury create input currency is no longer String")
    require("mycelix_finance_types::Currency" not in coordinator, "independent oracle: Treasury now references canonical Finance Currency")
    checks["TreasuryDoesNotConsumeCanonicalFinanceCurrencyType"] = True

    combined = finance + "\n" + coordinator + "\n" + integrity
    require("SapAmount" not in combined, "independent oracle: SapAmount now exists in bound source")
    require("TendAmount" not in combined, "independent oracle: TendAmount now exists in bound source")
    require("MycelAmount" not in combined, "independent oracle: MycelAmount now exists in bound source")
    contribution_entry = between(integrity, "pub struct Contribution {", "pub enum ContributionType", "Contribution entry")
    allocation_entry = between(integrity, "pub struct Allocation {", "pub enum AllocationStatus", "Allocation entry")
    require("pub amount: u64" in contribution_entry, "independent oracle: Contribution amount representation drift")
    require("pub amount: u64" in allocation_entry, "independent oracle: Allocation amount representation drift")
    checks["CanonicalFinanceQuantityTypeNotEstablished"] = True

    create_treasury_validation = between(integrity, "fn validate_create_treasury(", "fn validate_update_treasury(", "create Treasury integrity")
    update_treasury_validation = between(integrity, "fn validate_update_treasury(", "fn validate_create_contribution(", "update Treasury integrity")
    require("currency" not in create_treasury_validation, "independent oracle: create Treasury integrity now reasons about currency")
    require("currency" not in update_treasury_validation, "independent oracle: update Treasury integrity now reasons about currency")
    checks["TreasuryAssetIdentityImmutabilityNotEstablished"] = True

    contribute = between(coordinator, "pub fn contribute(", "pub struct ContributeInput {", "contribute")
    contribute_input = between(coordinator, "pub struct ContributeInput {", "fn credit_treasury(", "ContributeInput")
    contribution_validation = between(integrity, "fn validate_create_contribution(", "fn validate_create_allocation(", "Contribution integrity")
    require("pub currency: String" in contribute_input, "independent oracle: ContributeInput currency is no longer String")
    require("currency: input.currency" in contribute, "independent oracle: contribution no longer records caller currency")
    require("credit_treasury(&input.treasury_id, input.amount)" in contribute, "independent oracle: contribution credit path drift")
    require("get_treasury_record" not in contribute, "independent oracle: contribution now loads Treasury before credit")
    require("treasury.currency" not in contribute, "independent oracle: contribution now checks Treasury currency")
    require("currency" not in contribution_validation, "independent oracle: Contribution integrity now reasons about currency")
    credit = between(coordinator, "fn credit_treasury(", "fn debit_treasury(", "credit_treasury")
    require(".checked_add(amount)" in credit, "independent oracle: checked-add control missing")
    checks["ContributionToTreasuryAssetEqualityNotEstablished"] = True

    propose = between(coordinator, "pub fn propose_allocation(", "pub struct ProposeAllocationInput {", "propose_allocation")
    propose_input = between(coordinator, "pub struct ProposeAllocationInput {", "pub fn execute_allocation(", "ProposeAllocationInput")
    execute = between(coordinator, "pub fn execute_allocation(", "/// Internal helper: fetch an allocation", "execute_allocation")
    allocation_create_validation = between(integrity, "fn validate_create_allocation(", "fn validate_update_allocation(", "create Allocation integrity")
    allocation_update_validation = between(integrity, "fn validate_update_allocation(", "fn validate_create_savings_pool(", "update Allocation integrity")
    require("pub currency: String" in propose_input, "independent oracle: ProposeAllocationInput currency is no longer String")
    require("currency: input.currency" in propose, "independent oracle: proposal no longer stores caller currency")
    require("debit_treasury(&alloc.treasury_id, alloc.amount)" in execute, "independent oracle: allocation debit path drift")
    require("currency" not in execute, "independent oracle: execute_allocation now reasons about currency")
    require("currency" not in allocation_create_validation, "independent oracle: create Allocation integrity now reasons about currency")
    require("currency" not in allocation_update_validation, "independent oracle: update Allocation integrity now reasons about currency")
    debit = between(coordinator, "fn debit_treasury(", "/// Internal helper: fetch a treasury", "debit_treasury")
    require(".checked_sub(amount)" in debit, "independent oracle: checked-sub control missing")
    require("Insufficient treasury balance" in debit, "independent oracle: insufficient-balance control missing")
    checks["AllocationToTreasuryAssetEqualityNotEstablished"] = True

    require("const DKG_THRESHOLD_AMOUNT: u64 = 10_000_000_000;" in coordinator, "independent oracle: DKG threshold value/type drift")
    require("Minimum amount (in micro-SAP)" in coordinator, "independent oracle: DKG threshold micro-SAP documentation missing")
    require("Amount in micro-SAP (must exceed DKG_THRESHOLD_AMOUNT)" in coordinator, "independent oracle: DKG input unit documentation missing")
    dkg = between(coordinator, "pub fn execute_dkg_allocation(", "/// Input for a DKG-gated treasury allocation.", "execute_dkg_allocation")
    require("SapAmount" not in dkg and "Currency::Sap" not in dkg, "independent oracle: DKG path now type-binds canonical SAP")
    checks["DkgThresholdAssetBindingNotEstablished"] = True

    signed_subject = re.search(
        r'let\s+message\s*=\s*format!\(\s*"treasury_allocation:\{\}:\{\}:\{\}"\s*,\s*input\.treasury_id\s*,\s*input\.amount\s*,\s*input\.recipient_did\s*\);',
        dkg,
        flags=re.DOTALL,
    )
    require(signed_subject is not None, "independent oracle: DKG signed-subject template/arguments drift")
    require("message: message.as_bytes().to_vec()" in dkg, "independent oracle: threshold verifier no longer receives derived message bytes")
    message_source = signed_subject.group(0)
    for forbidden in ("currency", "asset_id", "base_unit", "policy_revision", "committee_id"):
        require(forbidden not in message_source, f"independent oracle: signed subject now binds {forbidden}")
    checks["DkgSignedSubjectAssetUnitPolicyBindingNotEstablished"] = True

    require(list(checks) == [r["id"] for r in EXPECTED_RECEIPTS], "independent oracle: semantic check ordering drift")
    require(all(checks.values()), "independent oracle: not all semantic checks established")
    return checks


def run_negative_controls(coordinator, integrity, finance, receipts):
    rejected = []

    receipt_mutant = json.loads(json.dumps(receipts))
    receipt_mutant[0]["observed_source_predicates"][0] += " [mutated]"
    rejected.append(expect_rejection(
        "receipt_predicate_drift",
        lambda: exact_receipt_contract(receipt_mutant),
    ))

    finance_transfer_mutant = replace_once(
        finance,
        "Currency::Sap => true",
        "Currency::Sap => false",
        "sap_transferability_flip",
    )
    rejected.append(expect_rejection(
        "sap_transferability_flip",
        lambda: derive_source_contracts(coordinator, integrity, finance_transfer_mutant),
    ))

    finance_quantity_mutant = finance + "\n#[derive(Clone, Copy)]\npub struct SapAmount(pub u64);\n"
    rejected.append(expect_rejection(
        "sap_amount_type_appears",
        lambda: derive_source_contracts(coordinator, integrity, finance_quantity_mutant),
    ))

    integrity_guard_mutant = replace_once(
        integrity,
        "fn validate_update_treasury(\n    _action: Update,\n    treasury: Treasury,\n) -> ExternResult<ValidateCallbackResult> {\n",
        "fn validate_update_treasury(\n    _action: Update,\n    treasury: Treasury,\n) -> ExternResult<ValidateCallbackResult> {\n    let _currency_guard = &treasury.currency;\n",
        "treasury_integrity_currency_guard",
    )
    rejected.append(expect_rejection(
        "treasury_integrity_currency_guard",
        lambda: derive_source_contracts(coordinator, integrity_guard_mutant, finance),
    ))

    contribution_guard_mutant = replace_once(
        coordinator,
        "    // Update treasury balance\n    credit_treasury(&input.treasury_id, input.amount)?;",
        "    // Counterfactual asset-equality guard\n    let (_, treasury_for_asset_check) = get_treasury_record(&input.treasury_id)?;\n    if input.currency != treasury_for_asset_check.currency {\n        return Err(wasm_error!(WasmErrorInner::Guest(\"asset mismatch\".into())));\n    }\n    credit_treasury(&input.treasury_id, input.amount)?;",
        "contribution_currency_equality_guard",
    )
    rejected.append(expect_rejection(
        "contribution_currency_equality_guard",
        lambda: derive_source_contracts(contribution_guard_mutant, integrity, finance),
    ))

    allocation_guard_mutant = replace_once(
        coordinator,
        "    debit_treasury(&alloc.treasury_id, alloc.amount)?;",
        "    if alloc.currency != \"SAP\" {\n        return Err(wasm_error!(WasmErrorInner::Guest(\"asset mismatch\".into())));\n    }\n    debit_treasury(&alloc.treasury_id, alloc.amount)?;",
        "allocation_currency_equality_guard",
    )
    rejected.append(expect_rejection(
        "allocation_currency_equality_guard",
        lambda: derive_source_contracts(allocation_guard_mutant, integrity, finance),
    ))

    dkg_subject_mutant = replace_once(
        coordinator,
        "    let message = format!(\n        \"treasury_allocation:{}:{}:{}\",\n        input.treasury_id, input.amount, input.recipient_did\n    );",
        "    let message = format!(\n        \"treasury_allocation:{}:{}:{}:{}\",\n        input.treasury_id, input.amount, input.recipient_did, treasury.currency\n    );",
        "dkg_signed_subject_asset_binding",
    )
    rejected.append(expect_rejection(
        "dkg_signed_subject_asset_binding",
        lambda: derive_source_contracts(dkg_subject_mutant, integrity, finance),
    ))

    expected = [
        "receipt_predicate_drift",
        "sap_transferability_flip",
        "sap_amount_type_appears",
        "treasury_integrity_currency_guard",
        "contribution_currency_equality_guard",
        "allocation_currency_equality_guard",
        "dkg_signed_subject_asset_binding",
    ]
    require(rejected == expected, "independent oracle: negative-control identity/order drift")
    return rejected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--corpus", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--coordinator", required=True)
    parser.add_argument("--integrity", required=True)
    parser.add_argument("--finance-types", required=True)
    args = parser.parse_args()

    corpus_raw = Path(args.corpus).read_bytes()
    require(hashlib.sha256(corpus_raw).hexdigest() == EXPECTED_CORPUS_SHA256, "independent oracle: corpus raw SHA-256 mismatch")
    corpus = json.loads(corpus_raw)
    require(corpus.get("schema") == "mycelix-treasury-asset-unit-source-contract-corpus-v1", "independent oracle: corpus schema mismatch")
    require(corpus.get("authority_class") == "HistoricalSourceContractEvidence", "independent oracle: authority class mismatch")
    require(corpus.get("receipt_count") == len(EXPECTED_RECEIPTS), "independent oracle: declared receipt count mismatch")
    require(set(corpus.get("non_claims", [])) == COMMON_NONCLAIMS, "independent oracle: common nonclaim set drift")
    receipts = corpus.get("receipts")
    require(isinstance(receipts, list), "independent oracle: receipts must be a list")
    exact_receipt_contract(receipts)

    binding = corpus.get("source_binding")
    require(isinstance(binding, dict), "independent oracle: source_binding must be an object")
    require(binding.get("semantic_production_subject_sha") == EXPECTED_SEMANTIC_SUBJECT, "independent oracle: semantic subject drift")
    require(binding.get("u0_evidence_head") == EXPECTED_U0_HEAD, "independent oracle: U0 evidence-head drift")
    require(binding.get("u0_profile_sha256") == EXPECTED_U0_PROFILE_CONTENT_SHA256, "independent oracle: U0 profile binding drift")

    actual_blobs = {
        "treasury_coordinator_blob": git_blob_sha1(args.coordinator),
        "treasury_integrity_blob": git_blob_sha1(args.integrity),
        "finance_types_blob": git_blob_sha1(args.finance_types),
    }
    for field, expected in EXPECTED_SOURCE_BLOBS.items():
        require(actual_blobs[field] == expected, f"independent oracle: actual {field} bytes drift")
        require(binding.get(field) == expected, f"independent oracle: corpus {field} drift")

    profile = json.loads(read_text(args.profile))
    require(isinstance(profile, dict), "independent oracle: U0 profile root must be object")
    payload = dict(profile)
    declared = payload.pop("profile_content_sha256", None)
    derived = hashlib.sha256(canonical_json(payload)).hexdigest()
    require(declared == EXPECTED_U0_PROFILE_CONTENT_SHA256, "independent oracle: declared U0 profile commitment drift")
    require(derived == EXPECTED_U0_PROFILE_CONTENT_SHA256, "independent oracle: derived U0 profile commitment drift")
    profile_binding = profile.get("source_binding")
    require(isinstance(profile_binding, dict), "independent oracle: U0 profile source_binding must be object")
    require(profile_binding.get("semantic_production_subject_sha") == EXPECTED_SEMANTIC_SUBJECT, "independent oracle: profile semantic subject drift")

    coordinator = read_text(args.coordinator)
    integrity = read_text(args.integrity)
    finance = read_text(args.finance_types)
    semantic_checks = derive_source_contracts(coordinator, integrity, finance)
    negative_controls = run_negative_controls(coordinator, integrity, finance, receipts)

    result = {
        "valid": True,
        "authority_class": corpus["authority_class"],
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "u0_profile_content_sha256": derived,
        "semantic_production_subject_sha": EXPECTED_SEMANTIC_SUBJECT,
        "source_blob_binding_count": len(actual_blobs),
        "receipt_count": len(receipts),
        "independent_semantic_check_count": len(semantic_checks),
        "independent_semantic_check_ids": list(semantic_checks),
        "negative_control_count": len(negative_controls),
        "negative_control_ids": negative_controls,
        "oracle_kind": "IndependentFrozenSourceContractOracleV1",
    }
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
