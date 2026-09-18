#!/usr/bin/env python3
"""Independent source-bound validator for MYC-CONST-003D1D-F0.

Uses only the Python standard library and Git. This validates the closed F0 claim
surface, exact historical source bindings, local pure-model invariants and a set
of capability-inflation mutations. It does not qualify provider truth, public-
fund authority, or live Treasury execution.
"""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "mycelix-governance/specs/constitutional-treasury-effect-contract.v1.json"
RUST = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider/src/lib.rs"
MANIFEST = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider/Cargo.toml"

EXPECTED_TOP_LEVEL = {
    "schema",
    "contract_id",
    "contract_revision",
    "authority_class",
    "activation_allowed",
    "source_binding",
    "request_identity",
    "execution_identity",
    "d1c_mapping",
    "semantic_uniqueness",
    "state_machine",
    "retry_and_reconciliation",
    "upstream_dependencies",
    "legacy_source_findings",
    "f1_activation_requirements",
    "non_claims",
}

EXPECTED_SOURCE_BINDINGS = {
    "legacy_execution_dispatch": (
        "mycelix-governance/zomes/execution/coordinator/src/lib.rs",
        "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    ),
    "governance_finance_bridge": (
        "mycelix-governance/zomes/bridge/coordinator/src/cross_cluster.rs",
        "3eb0ade8633d6e711fd266bca6eb2ebab616eac2",
    ),
    "legacy_treasury_coordinator": (
        "mycelix-finance/zomes/treasury/coordinator/src/lib.rs",
        "840e66bcb6fdedb27fe2451752511d1317eb50b8",
    ),
    "legacy_treasury_integrity": (
        "mycelix-finance/zomes/treasury/integrity/src/lib.rs",
        "5ee9b72f7c138a5283818b873ceae36dff3305d4",
    ),
    "d1c_identity_source": (
        "mycelix-governance/crates/constitutional-effect-ledger/src/lib.rs",
        "bb3b8d6a865bbf514ff1e43a41c520b890a5515d",
    ),
}


def fail(message: str) -> None:
    raise AssertionError(message)


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.rstrip("\n")


def git_show(ref: str, path: str) -> str:
    return git("show", f"{ref}:{path}")


def load_profile() -> dict[str, Any]:
    return json.loads(PROFILE.read_text())


def validate_profile(data: dict[str, Any]) -> None:
    require(set(data) == EXPECTED_TOP_LEVEL, "top-level F0 profile surface drift")
    require(
        data["schema"] == "mycelix.constitutional-treasury-effect-contract.v1",
        "schema drift",
    )
    require(data["contract_id"] == "MYC-CONST-003D1D-F0", "contract id drift")
    require(data["contract_revision"] == 1, "contract revision drift")
    require(
        data["authority_class"] == "InertProviderEffectContractPendingQualifiedUpstreams",
        "authority ceiling drift",
    )
    require(data["activation_allowed"] is False, "F0 must remain non-activating")

    source = data["source_binding"]
    require(source["repository"] == "Luminous-Dynamics/mycelix", "repository binding drift")
    require(
        source["d1d0_semantic_parent"] == "7bb43d0af6590f3794956d12c47d6fa376b59821",
        "D1D0 semantic parent drift",
    )
    require(
        source["runtime_subject"] == "15b9c89adf0ac3c6c5a73681614d6bfcd368820a",
        "legacy runtime subject drift",
    )
    require(len(source["files"]) == 5, "source binding census must contain five files")
    roles = {entry["role"] for entry in source["files"]}
    require(roles == set(EXPECTED_SOURCE_BINDINGS), "source role census drift")
    for entry in source["files"]:
        expected_path, expected_blob = EXPECTED_SOURCE_BINDINGS[entry["role"]]
        require(entry["path"] == expected_path, f"path drift for {entry['role']}")
        require(entry["git_blob_sha1"] == expected_blob, f"blob drift for {entry['role']}")

    request = data["request_identity"]
    required_request_fields = {
        "schema_version",
        "operation_id",
        "action_id",
        "proposal_id",
        "claim_binding_commitment",
        "action_commitment",
        "authorization_id",
        "authorization_subject_commitment",
        "treasury_descriptor_commitment",
        "allocation_subject_commitment",
        "approval_projection_commitment",
        "capacity_allocation_commitment_optional",
        "recipient_did",
        "recipient_commitment",
        "value_profile_id",
        "value_authority_commitment",
        "policy_revision_commitment",
        "adapter_profile_id",
        "effect_target_commitment",
    }
    require(set(request["includes"]) == required_request_fields, "request identity census drift")
    require(request["upstream_subjects_are_opaque_exact_refs"] is True, "upstream refs must remain opaque")
    require(request["raw_currency_string_is_authority"] is False, "raw currency string cannot be authority")
    require(request["raw_u64_amount_is_authority"] is False, "raw u64 amount cannot be authority")

    execution = data["execution_identity"]
    require(execution["provider_key"] == "execution_id", "provider key drift")
    for required in {
        "schema_version",
        "action_id",
        "authorization_id",
        "capacity_allocation_commitment_optional",
        "adapter_profile_id",
        "request_commitment",
    }:
        require(required in execution["includes"], f"missing execution identity field {required}")
    for forbidden in {
        "attempt_id",
        "attempt_ordinal",
        "committed_at_unix_ms",
        "started_at_unix_ms",
        "worker_id",
        "random_retry_nonce",
        "external_receipt_id",
    }:
        require(forbidden in execution["excludes"], f"missing execution exclusion {forbidden}")
    require(execution["retry_mints_fresh_execution_id"] is False, "retry cannot mint fresh execution identity")
    require(execution["physical_exactly_once_claimed"] is False, "F0 cannot claim physical exactly-once")

    mapping = data["d1c_mapping"]
    require(mapping["operation_id"] == "ConstitutionalOperation.operation_id", "operation mapping drift")
    require(mapping["proposal_id"] == "ConstitutionalOperation.proposal_id", "proposal mapping drift")
    require(mapping["claim_binding_commitment"] == "ConstitutionalOperation.claim_binding", "claim binding mapping drift")
    require(mapping["action_id"] == "ActionIntent.action_id", "action mapping drift")
    require(mapping["action_commitment"] == "ActionIntent.action_commitment", "action commitment mapping drift")
    require(mapping["provider_identity_regenerated"] is False, "provider identity regeneration forbidden")
    require(mapping["qualification_inherited_from_D1C"] is False, "D1C qualification cannot be inherited")

    uniqueness = data["semantic_uniqueness"]
    require(uniqueness["same_execution_same_request"] == "ExistingSame", "same request replay drift")
    require(uniqueness["same_execution_different_request"] == "IntegrityConflict", "execution collision must conflict")
    require(uniqueness["same_action_different_request"] == "IntegrityConflictAndHalt", "action collision must halt")
    require(uniqueness["authorization_id_reuse_for_different_effect"] == "IntegrityConflictAndHalt", "authorization reuse must halt")
    require(uniqueness["capacity_allocation_reuse_for_different_effect"] == "IntegrityConflictAndHalt", "capacity reuse must halt")
    require(uniqueness["last_write_wins_allowed"] is False, "last-write-wins forbidden")

    state = data["state_machine"]
    require(
        set(state["states"])
        == {"Pending", "InFlight", "UnknownOutcome", "KnownNoEffect", "KnownSuccess", "IntegrityHalted"},
        "state census drift",
    )
    require(state["initial_state"] == "Pending", "initial state drift")
    require(state["transport_failure_result"] == "UnknownOutcome", "transport failure must remain unknown")
    require(state["transport_failure_implies_known_no_effect"] is False, "transport error cannot prove no-effect")
    require(state["known_success_terminal_for_dispatch_retry"] is True, "known success must stop dispatch retry")
    require(state["conflicting_success_effect_commitments"] == "IntegrityHalted", "conflicting success must halt")
    require(state["success_and_no_effect_same_or_later_horizon"] == "IntegrityHalted", "success/no-effect contradiction must halt")
    require(state["stale_provider_observation_can_overwrite_newer_attempt"] is False, "stale observation cannot overwrite newer attempt")

    retry = data["retry_and_reconciliation"]
    require(retry["unknown_outcome_blind_retry_allowed"] is False, "blind unknown retry forbidden")
    require(retry["unknown_outcome_retry_requires_qualified_provider_replay_evidence"] is True, "unknown retry must require provider replay theorem")
    require(retry["known_no_effect_retry_requires_exact_reconciliation_observation"] is True, "no-effect retry must bind exact observation")
    require(retry["retry_reuses_same_execution_id"] is True, "retry must reuse execution identity")
    require(retry["known_no_effect_requires_provider_evidence"] is True, "KnownNoEffect needs evidence")
    require(retry["known_success_requires_provider_receipt_evidence"] is True, "KnownSuccess needs receipt evidence")
    require(retry["provider_observation_binds_execution_and_request"] is True, "provider evidence must bind execution/request")
    require(retry["provider_observation_carries_attempt_horizon"] is True, "provider evidence needs attempt horizon")
    require(retry["retry_basis_itself_qualified_here"] is False, "F0 must not authenticate retry basis")

    upstream = data["upstream_dependencies"]
    require(upstream["D1C_effect_identity"] == "StructurallyMappedQualificationNotInherited", "D1C dependency ceiling drift")
    for key in ["action_bound_authorization_1247", "SapAmount_1268", "TreasuryV2_1269", "allocation_approval_1411", "provider_adapter_capability"]:
        require(upstream[key] == "PendingExternalQualification", f"{key} qualification inflated")
    require(upstream["reserved_capacity_1467"] == "PendingExternalQualificationWhereProfileRequires", "capacity dependency drift")

    findings = data["legacy_source_findings"]
    require(findings["execution_dispatch_target"] == "governance_bridge::transfer_credits", "legacy dispatch target drift")
    require(findings["execution_dispatch_shape"] == "{from,to,amount:f64}", "legacy dispatch shape drift")
    require(findings["newer_bridge_target"] == "finance/treasury::execute_governance_transfer", "new bridge target drift")
    require(findings["execute_governance_transfer_observed_in_exact_treasury"] is False, "missing endpoint cannot be promoted")
    require(findings["treasury_allocation_has_stable_id_lookup"] is True, "stable allocation identity positive control lost")
    require(findings["treasury_debit_uses_checked_subtraction"] is True, "checked subtraction positive control lost")
    require(findings["legacy_treasury_currency_representation"] == "String", "legacy currency representation drift")
    require(findings["legacy_treasury_amount_representation"] == "u64", "legacy amount representation drift")
    require(findings["source_ordering_alone_establishes_local_partial_commit_bug"] is False, "source ordering cannot be inflated into local crash theorem")
    require(findings["large_value_DKG_policy_owned_by_separate_lane"] is True, "F0 must not absorb DKG policy")

    gates = set(data["f1_activation_requirements"])
    for required in {
        "exact qualified D1C identity consumed",
        "exact qualified action-bound authorization consumed",
        "exact qualified SAP value semantics consumed for SAP execution",
        "one selected authoritative effect adapter",
        "persistent intent before declared irreversible dispatch boundary",
        "provider-origin reconciliation by execution_id",
        "qualified replay/idempotency theorem before uncertain automatic retry",
        "F1 exact-head qualification before live governance routing",
    }:
        require(required in gates, f"missing F1 gate: {required}")

    nonclaims = set(data["non_claims"])
    for required in {
        "not_a_holochain_entry_or_link_surface",
        "not_a_live_treasury_debit_path",
        "not_wired_to_GovernanceAction_TransferCredits",
        "not_public_fund_authority_qualified",
        "not_SapAmount_qualified",
        "not_provider_truth_qualified",
        "not_provider_replay_evidence_authenticated_here",
        "not_physical_exactly_once",
        "not_external_settlement_finality",
        "not_deployment_currentness_qualified",
    }:
        require(required in nonclaims, f"missing non-claim: {required}")


def verify_source_bindings(data: dict[str, Any]) -> None:
    source = data["source_binding"]
    runtime_subject = source["runtime_subject"]
    d1d0_parent = source["d1d0_semantic_parent"]
    materialized: dict[str, str] = {}

    for entry in source["files"]:
        ref = d1d0_parent if entry["role"] == "d1c_identity_source" else runtime_subject
        actual_blob = git("rev-parse", f"{ref}:{entry['path']}")
        require(actual_blob == entry["git_blob_sha1"], f"actual blob drift for {entry['role']}")
        materialized[entry["role"]] = git_show(ref, entry["path"])

    execution = materialized["legacy_execution_dispatch"]
    require("TransferCredits {" in execution, "legacy TransferCredits variant missing")
    require("amount: f64" in execution, "legacy transfer f64 amount shape changed")
    require(
        'serde_json::json!({"from": from, "to": to, "amount": amount})' in execution,
        "legacy from/to/amount dispatch shape changed",
    )
    require('"governance_bridge"' in execution and '"transfer_credits"' in execution, "legacy bridge dispatch target changed")

    bridge = materialized["governance_finance_bridge"]
    require("pub fn execute_approved_transfer" in bridge, "approved-transfer bridge missing")
    require('"finance", "treasury", "execute_governance_transfer"' in bridge, "newer bridge target changed")
    require("pub amount_sap: u64" in bridge, "approved-transfer raw amount shape changed")

    treasury = materialized["legacy_treasury_coordinator"]
    require("pub fn execute_allocation" in treasury, "execute_allocation missing")
    require("debit_treasury(&alloc.treasury_id, alloc.amount)?;" in treasury, "allocation debit call changed")
    require("checked_sub(amount)" in treasury, "checked subtraction positive control missing")
    require("pub amount: u64" in treasury and "pub currency: String" in treasury, "legacy allocation input shape changed")
    require("pub fn execute_governance_transfer" not in treasury, "previously missing execute_governance_transfer now exists; profile must be superseded")

    integrity = materialized["legacy_treasury_integrity"]
    require("pub struct Treasury" in integrity, "Treasury entry missing")
    require("pub currency: String" in integrity and "pub balance: u64" in integrity, "legacy Treasury value representation changed")
    require("pub struct Allocation" in integrity and "pub amount: u64" in integrity, "legacy Allocation amount representation changed")

    d1c = materialized["d1c_identity_source"]
    for snippet in [
        "pub struct ConstitutionalOperation",
        "pub operation_id: String",
        "pub proposal_id: String",
        "pub claim_binding: String",
        "pub struct ActionIntent",
        "pub action_id: String",
        "pub action_commitment: String",
    ]:
        require(snippet in d1c, f"D1C identity source drift: {snippet}")


def verify_local_model_surface() -> None:
    rust = RUST.read_text()
    manifest = MANIFEST.read_text()

    for required in [
        "pub struct TreasuryEffectSubject",
        "pub authorization_id: String",
        "pub capacity_allocation_commitment: Option<String>",
        "pub struct TreasuryEffectRequest",
        "pub fn execution_id(&self)",
        "pub enum EffectState",
        "UnknownOutcome",
        "KnownNoEffect",
        "KnownSuccess",
        "IntegrityHalted",
        "pub fn record_transport_failure",
        "UnknownOutcome cannot be retried without qualified replay evidence",
        "pub authorization_index: BTreeMap<String, String>",
        "pub capacity_index: BTreeMap<String, String>",
        "source ordering",
    ]:
        # The source-ordering phrase lives in docs/profile rather than Rust; skip that
        # exact phrase here while keeping this list compact.
        if required != "source ordering":
            require(required in rust, f"F0 Rust surface missing: {required}")

    require("hdk" not in manifest.lower(), "F0 crate must remain HDK-free")
    require("hdi" not in manifest.lower(), "F0 crate must remain HDI-free")
    require("amount: f64" not in rust, "F0 must not expose legacy f64 amount authority")
    require("pub currency: String" not in rust, "F0 must not expose legacy currency-string authority")
    require("external_receipt_id" in rust, "receipt identity separation missing")
    require("attempt_id" in rust, "attempt identity separation missing")


def expect_rejected(name: str, data: dict[str, Any], mutate: Callable[[dict[str, Any]], None]) -> None:
    candidate = copy.deepcopy(data)
    mutate(candidate)
    try:
        validate_profile(candidate)
    except AssertionError:
        return
    fail(f"mutation unexpectedly survived: {name}")


def mutation_self_tests(data: dict[str, Any]) -> None:
    mutations: list[tuple[str, Callable[[dict[str, Any]], None]]] = [
        ("activate_f0", lambda d: d.__setitem__("activation_allowed", True)),
        ("inherit_d1c_qualification", lambda d: d["d1c_mapping"].__setitem__("qualification_inherited_from_D1C", True)),
        ("raw_u64_authority", lambda d: d["request_identity"].__setitem__("raw_u64_amount_is_authority", True)),
        ("fresh_execution_on_retry", lambda d: d["execution_identity"].__setitem__("retry_mints_fresh_execution_id", True)),
        ("claim_exactly_once", lambda d: d["execution_identity"].__setitem__("physical_exactly_once_claimed", True)),
        ("transport_means_no_effect", lambda d: d["state_machine"].__setitem__("transport_failure_implies_known_no_effect", True)),
        ("blind_unknown_retry", lambda d: d["retry_and_reconciliation"].__setitem__("unknown_outcome_blind_retry_allowed", True)),
        ("drop_action_conflict_halt", lambda d: d["semantic_uniqueness"].__setitem__("same_action_different_request", "ExistingSame")),
        ("allow_authorization_reuse", lambda d: d["semantic_uniqueness"].__setitem__("authorization_id_reuse_for_different_effect", "Allowed")),
        ("allow_capacity_reuse", lambda d: d["semantic_uniqueness"].__setitem__("capacity_allocation_reuse_for_different_effect", "Allowed")),
        ("promote_missing_endpoint", lambda d: d["legacy_source_findings"].__setitem__("execute_governance_transfer_observed_in_exact_treasury", True)),
        ("reassert_local_partial_commit_from_order", lambda d: d["legacy_source_findings"].__setitem__("source_ordering_alone_establishes_local_partial_commit_bug", True)),
        ("absorb_dkg_policy", lambda d: d["legacy_source_findings"].__setitem__("large_value_DKG_policy_owned_by_separate_lane", False)),
        ("authenticate_retry_basis_here", lambda d: d["retry_and_reconciliation"].__setitem__("retry_basis_itself_qualified_here", True)),
    ]
    for name, mutate in mutations:
        expect_rejected(name, data, mutate)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-source", action="store_true", help="skip Git-object/source checks")
    args = parser.parse_args()

    data = load_profile()
    validate_profile(data)
    if not args.skip_source:
        verify_source_bindings(data)
    verify_local_model_surface()
    mutation_self_tests(data)
    print("MYC-CONST-003D1D-F0 validator: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
