#!/usr/bin/env python3
"""Ephemeral FIN-SAFE-024 complete ABI-isolation migration.

This script is qualification tooling only. It runs against the frozen product
subject after the exact Phase-A state has been reconstructed. It must not relax
any exact-action, exact-entry, DNA-order, or fail-closed authority theorem.
"""

from pathlib import Path

ROOT = Path("mycelix-finance")
CONTRACTS = ROOT / "crates/finance-holochain-contracts"


def read(path: str | Path) -> str:
    return Path(path).read_text()


def write(path: str | Path, text: str) -> None:
    Path(path).write_text(text)


def require_once(text: str, needle: str, label: str) -> None:
    count = text.count(needle)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one occurrence, found {count}: {needle!r}")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    require_once(text, old, label)
    return text.replace(old, new, 1)


def remove_dep(path: str | Path, line: str) -> None:
    text = read(path)
    write(path, replace_once(text, line, "", f"{path} dependency removal"))


def add_dep(path: str | Path, anchor: str, line: str) -> None:
    text = read(path)
    if line in text:
        return
    write(path, replace_once(text, anchor, anchor + line, f"{path} dependency insertion"))


def extract_block(path: str | Path, start_marker: str, end_marker: str) -> str:
    text = read(path)
    require_once(text, start_marker, f"{path} block start")
    start = text.index(start_marker)
    end = text.index(end_marker, start)
    block = text[start:end]
    write(path, text[:start] + text[end:])
    return block


def function_span(text: str, signature_prefix: str) -> tuple[int, int]:
    require_once(text, signature_prefix, f"function {signature_prefix}")
    start = text.index(signature_prefix)
    brace = text.index("{", start)
    depth = 0
    for index in range(brace, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return start, index + 1
    raise SystemExit(f"unterminated function: {signature_prefix}")


def replace_function(path: str | Path, signature_prefix: str, replacement: str) -> None:
    text = read(path)
    start, end = function_span(text, signature_prefix)
    write(path, text[:start] + replacement.rstrip() + text[end:])


FOREIGN_ENTRY_HELPER = r'''fn foreign_public_entry_def(
    zome_name: &str,
    entry_index: u8,
) -> ExternResult<AppEntryDef> {
    let info = dna_info()?;
    let zome_position = info
        .zome_names
        .iter()
        .position(|name| name.to_string() == zome_name)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Required integrity zome {zome_name:?} is absent from this DNA"
            )))
        })?;
    let zome_index = u8::try_from(zome_position).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Integrity zome index for {zome_name:?} exceeds u8"
        )))
    })?;
    Ok(AppEntryDef::new(
        EntryDefIndex::from(entry_index),
        ZomeIndex::from(zome_index),
        EntryVisibility::Public,
    ))
}
'''

EXACT_APP_HELPER = r'''fn require_exact_app_entry_type(
    record: &Record,
    expected: AppEntryDef,
    label: &str,
) -> ExternResult<()> {
    match record.action().entry_type() {
        Some(EntryType::App(actual)) if actual == &expected => Ok(()),
        Some(EntryType::App(actual)) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
        )))),
        Some(actual) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} is not an application entry: got {actual:?}"
        )))),
        None => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has no entry type"
        )))),
    }
}
'''

EXACT_CREATE_HELPER = r'''fn require_exact_create_entry(
    record: &Record,
    expected: AppEntryDef,
    label: &str,
) -> ExternResult<()> {
    if record.action().action_type() != ActionType::Create {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} reference is not an exact Create action"
        ))));
    }
    match record.action().entry_type() {
        Some(EntryType::App(actual)) if actual == &expected => Ok(()),
        Some(EntryType::App(actual)) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
        )))),
        Some(actual) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} is not an application entry: got {actual:?}"
        )))),
        None => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has no entry type"
        )))),
    }
}
'''


def extend_contract_crate() -> None:
    manifest = CONTRACTS / "Cargo.toml"
    text = read(manifest)
    anchor = 'finance-collateral-auth = { path = "../finance-collateral-auth" }\n'
    additions = (
        'finance-collateral-issuance-persistence = { path = "../finance-collateral-issuance-persistence" }\n'
        'finance-sap-account-v2 = { path = "../finance-sap-account-v2" }\n'
        'finance-sap-transfer-v2 = { path = "../finance-sap-transfer-v2" }\n'
    )
    text = replace_once(text, anchor, anchor + additions, "contract manifest")
    write(manifest, text)

    source = CONTRACTS / "src/lib.rs"
    text = read(source)
    imports = '''use finance_collateral_issuance_persistence::{
    CollateralSapIssuanceReceiptRecordV2, CollateralSapMintAuthorizationRecordV2,
    CollateralSapMintRecordV2Compact,
};
use finance_sap_account_v2::{SapAccountOpenedV2, SapAccountV2Config, SapCollateralClaimV2};
use finance_sap_transfer_v2::{SapTransferSpendRecordV2, SapTransferV2Config};
'''
    # This semantic anchor is stable across rustfmt layouts of the preceding imports.
    text = replace_once(text, "use hdi::prelude::*;\n", imports + "use hdi::prelude::*;\n", "contract imports")
    write(source, text)

    issuance_block = extract_block(
        ROOT / "zomes/collateral-issuance-v2/integrity/src/lib.rs",
        "#[hdk_entry_helper]\n#[derive(Clone, PartialEq)]\npub struct CollateralSapMintAuthorizationV2Entry",
        "#[hdk_entry_types]",
    )
    account_block = extract_block(
        ROOT / "zomes/sap-account-v2/integrity/src/lib.rs",
        "#[dna_properties]",
        "#[hdk_entry_types]",
    )
    transfer_block = extract_block(
        ROOT / "zomes/sap-transfer-v2/integrity/src/lib.rs",
        "#[dna_properties]",
        "#[hdk_entry_types]",
    )
    source_text = read(source).rstrip() + "\n\n" + issuance_block + account_block + transfer_block
    write(source, source_text)


def decouple_integrity_zomes() -> None:
    issuance = ROOT / "zomes/collateral-issuance-v2/integrity/src/lib.rs"
    text = read(issuance)
    anchor = "use finance_holochain_contracts::{\n"
    require_once(text, anchor, "issuance contract import")
    addition = (
        "use finance_holochain_contracts::{\n"
        "    CollateralSapIssuanceReceiptV2Entry, CollateralSapMintAuthorizationV2Entry,\n"
        "    CollateralSapMintRecordV2Entry,\n"
    )
    text = text.replace(anchor, addition, 1)
    write(issuance, text)

    account = ROOT / "zomes/sap-account-v2/integrity/src/lib.rs"
    text = read(account)
    text = replace_once(
        text,
        "use collateral_issuance_v2_integrity::CollateralSapIssuanceReceiptV2Entry;\n",
        "use finance_holochain_contracts::{\n"
        "    CollateralSapIssuanceReceiptV2Entry, FinanceSapAccountV2DnaProperties,\n"
        "    SapAccountOpenedV2Entry, SapCollateralClaimV2Entry,\n"
        "};\n",
        "sap account foreign payload import",
    )
    write(account, text)
    account_manifest = ROOT / "zomes/sap-account-v2/integrity/Cargo.toml"
    remove_dep(
        account_manifest,
        'collateral_issuance_v2_integrity = { path = "../../collateral-issuance-v2/integrity" }\n',
    )
    add_dep(
        account_manifest,
        'finance-collateral-issuance-persistence = { path = "../../../crates/finance-collateral-issuance-persistence" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )

    transfer = ROOT / "zomes/sap-transfer-v2/integrity/src/lib.rs"
    text = read(transfer)
    text = replace_once(
        text,
        "use collateral_issuance_v2_integrity::CollateralSapIssuanceReceiptV2Entry;\n",
        "use finance_holochain_contracts::{\n"
        "    CollateralSapIssuanceReceiptV2Entry, FinanceSapTransferV2DnaProperties,\n"
        "    SapCollateralClaimV2Entry, SapTransferSpendV2Entry,\n"
        "};\n",
        "sap transfer issuance payload import",
    )
    text = replace_once(
        text,
        "use sap_account_v2_integrity::SapCollateralClaimV2Entry;\n",
        "",
        "sap transfer account payload import",
    )
    write(transfer, text)
    transfer_manifest = ROOT / "zomes/sap-transfer-v2/integrity/Cargo.toml"
    remove_dep(
        transfer_manifest,
        'sap_account_v2_integrity = { path = "../../sap-account-v2/integrity" }\n',
    )
    remove_dep(
        transfer_manifest,
        'collateral_issuance_v2_integrity = { path = "../../collateral-issuance-v2/integrity" }\n',
    )
    add_dep(
        transfer_manifest,
        'finance-collateral-issuance-persistence = { path = "../../../crates/finance-collateral-issuance-persistence" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )


def insert_after_import(path: Path, import_line: str, payload: str, label: str) -> None:
    text = read(path)
    text = replace_once(text, import_line, import_line + payload, label)
    write(path, text)


def insert_helper_before(path: Path, marker: str, helper: str, label: str) -> None:
    text = read(path)
    require_once(text, marker, label)
    write(path, text.replace(marker, helper.rstrip() + "\n\n" + marker, 1))


def decouple_coordinators() -> None:
    # Deposit coordinator: own authority enums remain local; payload moves direct.
    path = ROOT / "zomes/collateral-deposit-v2/coordinator/src/lib.rs"
    text = read(path)
    old = '''use collateral_deposit_v2_integrity::{
    CollateralDepositRequestV2Entry, EntryTypes, MAX_CREATE_TIMESTAMP_SKEW_MICROS, UnitEntryTypes,
};'''
    new = '''use collateral_deposit_v2_integrity::{EntryTypes, UnitEntryTypes};
use finance_holochain_contracts::{
    CollateralDepositRequestV2Entry, MAX_CREATE_TIMESTAMP_SKEW_MICROS,
};'''
    text = replace_once(text, old, new, "deposit coordinator imports")
    write(path, text)
    add_dep(
        ROOT / "zomes/collateral-deposit-v2/coordinator/Cargo.toml",
        'finance-collateral-request-binding = { path = "../../../crates/finance-collateral-request-binding" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )

    # Settlement-auth coordinator: own enums local, foreign deposit identity from DNA.
    path = ROOT / "zomes/collateral-settlement-auth/coordinator/src/lib.rs"
    text = read(path)
    old = '''use collateral_deposit_v2_integrity::{
    CollateralDepositRequestV2Entry, MAX_CREATE_TIMESTAMP_SKEW_MICROS,
    UnitEntryTypes as DepositUnitEntryTypes,
};
use collateral_settlement_auth_integrity::{
    CustodyAttestationV1Entry, EntryTypes, PriceAttestationV1Entry,
    UnitEntryTypes as AuthUnitEntryTypes, load_collateral_auth_config,
};'''
    new = '''use collateral_settlement_auth_integrity::{EntryTypes, UnitEntryTypes as AuthUnitEntryTypes};
use finance_holochain_contracts::{
    CollateralDepositRequestV2Entry, CustodyAttestationV1Entry,
    MAX_CREATE_TIMESTAMP_SKEW_MICROS, PriceAttestationV1Entry, load_collateral_auth_config,
};'''
    text = replace_once(text, old, new, "settlement-auth coordinator imports")
    text = replace_once(
        text,
        "use mycelix_bridge_entry_types::did_for_author;\n",
        "use mycelix_bridge_entry_types::did_for_author;\n\n"
        "const COLLATERAL_DEPOSIT_INTEGRITY_ZOME: &str = \"collateral_deposit_v2_integrity\";\n"
        "const COLLATERAL_DEPOSIT_REQUEST_ENTRY_INDEX: u8 = 0;\n",
        "settlement-auth foreign constants",
    )
    text = replace_once(
        text,
        "AppEntryDef::try_from(DepositUnitEntryTypes::CollateralDepositRequestV2)?",
        "foreign_public_entry_def(\n            COLLATERAL_DEPOSIT_INTEGRITY_ZOME,\n            COLLATERAL_DEPOSIT_REQUEST_ENTRY_INDEX,\n        )?",
        "settlement-auth deposit identity",
    )
    write(path, text)
    insert_helper_before(path, "fn require_create_action(", FOREIGN_ENTRY_HELPER, "settlement-auth foreign helper")
    replace_function(path, "fn require_exact_app_entry_type(", EXACT_APP_HELPER)
    manifest = ROOT / "zomes/collateral-settlement-auth/coordinator/Cargo.toml"
    remove_dep(manifest, 'collateral_deposit_v2_integrity = { path = "../../collateral-deposit-v2/integrity" }\n')
    add_dep(
        manifest,
        'finance-collateral-settlement = { path = "../../../crates/finance-collateral-settlement" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )

    # Issuance coordinator: own authority enums local, local payload/config contract direct.
    path = ROOT / "zomes/collateral-issuance-v2/coordinator/src/lib.rs"
    text = read(path)
    old = '''use collateral_issuance_v2_integrity::{
    CollateralSapIssuanceReceiptV2Entry, CollateralSapMintAuthorizationV2Entry,
    CollateralSapMintRecordV2Entry, EntryTypes, UnitEntryTypes,
};
use collateral_settlement_auth_integrity::load_collateral_auth_config;'''
    new = '''use collateral_issuance_v2_integrity::{EntryTypes, UnitEntryTypes};
use finance_holochain_contracts::{
    CollateralSapIssuanceReceiptV2Entry, CollateralSapMintAuthorizationV2Entry,
    CollateralSapMintRecordV2Entry, load_collateral_auth_config,
};'''
    text = replace_once(text, old, new, "issuance coordinator imports")
    write(path, text)
    replace_function(path, "fn require_exact_create_entry(", EXACT_CREATE_HELPER)
    manifest = ROOT / "zomes/collateral-issuance-v2/coordinator/Cargo.toml"
    remove_dep(
        manifest,
        'collateral_settlement_auth_integrity = { path = "../../collateral-settlement-auth/integrity" }\n',
    )
    add_dep(
        manifest,
        'finance-sap-conservation = { path = "../../../crates/finance-sap-conservation" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )

    # SAP account coordinator: foreign issuance payload/identity no longer links authority crate.
    path = ROOT / "zomes/sap-account-v2/coordinator/src/lib.rs"
    text = read(path)
    old1 = '''use collateral_issuance_v2_integrity::{
    CollateralSapIssuanceReceiptV2Entry, UnitEntryTypes as CollateralIssuanceUnitEntryTypes,
};'''
    new1 = '''use finance_holochain_contracts::{
    CollateralSapIssuanceReceiptV2Entry, SapAccountOpenedV2Entry, SapCollateralClaimV2Entry,
};'''
    old2 = '''use sap_account_v2_integrity::{
    EntryTypes, SapAccountOpenedV2Entry, SapCollateralClaimV2Entry, UnitEntryTypes,
    load_sap_account_v2_config,
};'''
    new2 = '''use sap_account_v2_integrity::{EntryTypes, UnitEntryTypes, load_sap_account_v2_config};'''
    text = replace_once(text, old1, new1, "sap account coordinator issuance imports")
    text = replace_once(text, old2, new2, "sap account coordinator own imports")
    text = replace_once(
        text,
        "use mycelix_bridge_entry_types::did_for_author;\n",
        "use mycelix_bridge_entry_types::did_for_author;\n\n"
        "const COLLATERAL_ISSUANCE_INTEGRITY_ZOME: &str = \"collateral_issuance_v2_integrity\";\n"
        "const COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX: u8 = 2;\n",
        "sap account foreign constants",
    )
    text = replace_once(
        text,
        "AppEntryDef::try_from(CollateralIssuanceUnitEntryTypes::CollateralSapIssuanceReceiptV2)?",
        "foreign_public_entry_def(\n            COLLATERAL_ISSUANCE_INTEGRITY_ZOME,\n            COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX,\n        )?",
        "sap account receipt identity",
    )
    write(path, text)
    insert_helper_before(path, "fn create_and_reload(", FOREIGN_ENTRY_HELPER, "sap account foreign helper")
    replace_function(path, "fn require_exact_create_entry(", EXACT_CREATE_HELPER)
    manifest = ROOT / "zomes/sap-account-v2/coordinator/Cargo.toml"
    remove_dep(manifest, 'collateral_issuance_v2_integrity = { path = "../../collateral-issuance-v2/integrity" }\n')
    add_dep(
        manifest,
        'finance-collateral-issuance-persistence = { path = "../../../crates/finance-collateral-issuance-persistence" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )

    # SAP transfer coordinator: two foreign identities, neither imports foreign authority.
    path = ROOT / "zomes/sap-transfer-v2/coordinator/src/lib.rs"
    text = read(path)
    old1 = '''use collateral_issuance_v2_integrity::{
    CollateralSapIssuanceReceiptV2Entry, UnitEntryTypes as CollateralIssuanceUnitEntryTypes,
};'''
    new1 = '''use finance_holochain_contracts::{
    CollateralSapIssuanceReceiptV2Entry, SapCollateralClaimV2Entry, SapTransferSpendV2Entry,
};'''
    old2 = '''use sap_account_v2_integrity::{
    SapCollateralClaimV2Entry, UnitEntryTypes as SapAccountUnitEntryTypes,
};
'''
    old3 = '''use sap_transfer_v2_integrity::{
    EntryTypes, SapTransferSpendV2Entry, UnitEntryTypes as SapTransferUnitEntryTypes,
    load_sap_transfer_v2_config,
};'''
    new3 = '''use sap_transfer_v2_integrity::{
    EntryTypes, UnitEntryTypes as SapTransferUnitEntryTypes, load_sap_transfer_v2_config,
};'''
    text = replace_once(text, old1, new1, "sap transfer coordinator issuance imports")
    text = replace_once(text, old2, "", "sap transfer coordinator account imports")
    text = replace_once(text, old3, new3, "sap transfer coordinator own imports")
    text = replace_once(
        text,
        "use mycelix_bridge_entry_types::did_for_author;\n",
        "use mycelix_bridge_entry_types::did_for_author;\n\n"
        "const SAP_ACCOUNT_V2_INTEGRITY_ZOME: &str = \"sap_account_v2_integrity\";\n"
        "const SAP_COLLATERAL_CLAIM_ENTRY_INDEX: u8 = 1;\n"
        "const COLLATERAL_ISSUANCE_INTEGRITY_ZOME: &str = \"collateral_issuance_v2_integrity\";\n"
        "const COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX: u8 = 2;\n",
        "sap transfer foreign constants",
    )
    text = replace_once(
        text,
        "AppEntryDef::try_from(SapAccountUnitEntryTypes::SapCollateralClaimV2)?",
        "foreign_public_entry_def(\n            SAP_ACCOUNT_V2_INTEGRITY_ZOME,\n            SAP_COLLATERAL_CLAIM_ENTRY_INDEX,\n        )?",
        "sap transfer claim identity",
    )
    text = replace_once(
        text,
        "AppEntryDef::try_from(CollateralIssuanceUnitEntryTypes::CollateralSapIssuanceReceiptV2)?",
        "foreign_public_entry_def(\n            COLLATERAL_ISSUANCE_INTEGRITY_ZOME,\n            COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX,\n        )?",
        "sap transfer receipt identity",
    )
    write(path, text)
    insert_helper_before(path, "fn require_create_action(", FOREIGN_ENTRY_HELPER, "sap transfer foreign helper")
    replace_function(path, "fn require_exact_app_entry_type(", EXACT_APP_HELPER)
    manifest = ROOT / "zomes/sap-transfer-v2/coordinator/Cargo.toml"
    remove_dep(manifest, 'sap_account_v2_integrity = { path = "../../sap-account-v2/integrity" }\n')
    remove_dep(manifest, 'collateral_issuance_v2_integrity = { path = "../../collateral-issuance-v2/integrity" }\n')
    add_dep(
        manifest,
        'finance-sap-value-notes = { path = "../../../crates/finance-sap-value-notes" }\n',
        'finance-holochain-contracts = { path = "../../../crates/finance-holochain-contracts" }\n',
    )


def prove_manifest_boundary() -> None:
    allowed = {
        ROOT / "zomes/collateral-deposit-v2/coordinator/Cargo.toml": {"collateral_deposit_v2_integrity"},
        ROOT / "zomes/collateral-settlement-auth/coordinator/Cargo.toml": {"collateral_settlement_auth_integrity"},
        ROOT / "zomes/collateral-issuance-v2/coordinator/Cargo.toml": {"collateral_issuance_v2_integrity"},
        ROOT / "zomes/sap-account-v2/coordinator/Cargo.toml": {"sap_account_v2_integrity"},
        ROOT / "zomes/sap-transfer-v2/coordinator/Cargo.toml": {"sap_transfer_v2_integrity"},
    }
    manifests = [
        ROOT / "zomes/collateral-deposit-v2/integrity/Cargo.toml",
        ROOT / "zomes/collateral-settlement-auth/integrity/Cargo.toml",
        ROOT / "zomes/collateral-issuance-v2/integrity/Cargo.toml",
        ROOT / "zomes/sap-account-v2/integrity/Cargo.toml",
        ROOT / "zomes/sap-transfer-v2/integrity/Cargo.toml",
        *allowed.keys(),
    ]
    for manifest in manifests:
        dependencies = {
            line.split("=", 1)[0].strip()
            for line in read(manifest).splitlines()
            if "_integrity = { path =" in line
        }
        expected = allowed.get(manifest, set())
        if dependencies != expected:
            raise SystemExit(f"{manifest}: integrity dependencies {dependencies!r}, expected {expected!r}")

    contract_text = read(CONTRACTS / "src/lib.rs")
    for forbidden in ("#[hdk_entry_types]", "#[hdk_link_types]", "#[hdk_extern]"):
        if forbidden in contract_text:
            raise SystemExit(f"contract crate contains forbidden authority macro {forbidden}")


if __name__ == "__main__":
    extend_contract_crate()
    decouple_integrity_zomes()
    decouple_coordinators()
    prove_manifest_boundary()
