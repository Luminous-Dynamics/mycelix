#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-F1B2A without third-party Python dependencies."""
from __future__ import annotations

import argparse
import copy
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
PROFILE = ROOT / "specs/constitutional-payments-provider-authority.v1.json"
SOURCE = ROOT / "crates/constitutional-payments-provider-authority/src/lib.rs"
CARGO = ROOT / "crates/constitutional-payments-provider-authority/Cargo.toml"
WORKSPACE = ROOT / "Cargo.toml"

EXPECTED = {
    "f1b2": "1ded163920ef7ea2829a807cf61c89f224751714",
    "constitutional_authority_blob": "2eb0781f205bff1aaeded87632734b1abd4ca596",
    "threshold_integrity_blob": "3fec8344635600c044a494fa72bbbfe408fbe5ec",
    "threshold_coordinator_blob": "3449df8b03a4dd1774a5f22756d06931c72855b2",
}


def fail(msg: str) -> None:
    raise ValueError(msg)


def require(cond: bool, msg: str) -> None:
    if not cond:
        fail(msg)


def keys(obj: dict, expected: set[str], label: str) -> None:
    require(set(obj) == expected, f"{label} key census drift: {sorted(set(obj) ^ expected)}")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT.parent, text=True).strip()


def validate_profile(p: dict) -> None:
    keys(p, {
        "schema", "profile_id", "authority_class", "lineage", "role_model",
        "constitutional_anchor", "grant", "direct_author", "threshold_author",
        "rotation", "revocation_and_history", "activation", "next_tranche", "non_claims",
    }, "root")
    require(p["schema"] == "mycelix.constitutional-payments-provider-authority.v1", "schema drift")
    require(p["profile_id"] == "mycelix-payments-provider-authority-f1b2a-v1", "profile id drift")
    require(p["authority_class"] == "InertSemanticContract", "authority inflation")

    lineage = p["lineage"]
    keys(lineage, {"f1b2_semantic_head", "constitutional_authority", "threshold_signing_integrity", "threshold_signing_coordinator"}, "lineage")
    require(lineage["f1b2_semantic_head"] == EXPECTED["f1b2"], "F1B2 lineage drift")
    require(lineage["constitutional_authority"]["git_blob_sha"] == EXPECTED["constitutional_authority_blob"], "constitutional authority blob drift")
    require(lineage["threshold_signing_integrity"]["git_blob_sha"] == EXPECTED["threshold_integrity_blob"], "threshold integrity blob drift")
    require(lineage["threshold_signing_coordinator"]["git_blob_sha"] == EXPECTED["threshold_coordinator_blob"], "threshold coordinator blob drift")

    role = p["role_model"]
    keys(role, {"OperationIntent", "DispatchAttempt", "OperationIndex", "Observation", "single_expected_author_is_sufficient_for_live_activation", "cross_role_substitution_allowed"}, "role_model")
    require(role["OperationIntent"] == "OrchestratorIntent", "intent role drift")
    require(role["DispatchAttempt"] == "OrchestratorDispatch", "dispatch role drift")
    require(role["OperationIndex"] == "OperationIndex", "index role drift")
    require(role["Observation"] == "ProviderObservation", "observation role drift")
    require(role["single_expected_author_is_sufficient_for_live_activation"] is False, "single-author F1B2 model promoted to live sufficiency")
    require(role["cross_role_substitution_allowed"] is False, "cross-role substitution enabled")

    anchor = p["constitutional_anchor"]
    keys(anchor, {"required_holder", "required_power", "automated_provider_becomes_constitutional_principal", "operational_grant_can_transfer_value", "root_capability_validation_required", "delegated_constitutional_parent_supported_in_v1"}, "constitutional_anchor")
    require(anchor["required_holder"] == "Branch::Stewardship", "public-funds holder drift")
    require(anchor["required_power"] == "ExecuteAppropriation", "public-funds power drift")
    require(anchor["automated_provider_becomes_constitutional_principal"] is False, "automated-provider sovereignty inflation")
    require(anchor["operational_grant_can_transfer_value"] is False, "author grant gained transfer authority")
    require(anchor["root_capability_validation_required"] is True, "root capability validation removed")
    require(anchor["delegated_constitutional_parent_supported_in_v1"] is False, "unqualified delegated parent enabled")

    grant = p["grant"]
    keys(grant, {
        "binds_provider_profile_id", "binds_provider_profile_commitment", "binds_jurisdiction",
        "binds_upstream_authority_commitment", "binds_record_roles", "binds_author_mode",
        "binds_validity_interval", "binds_epoch", "binds_predecessor_on_rotation",
        "domain_separated_commitment", "wall_clock_is_authority_identity", "issuance_proof_is_qualified",
    }, "grant")
    for k in ["binds_provider_profile_id", "binds_provider_profile_commitment", "binds_jurisdiction", "binds_upstream_authority_commitment", "binds_record_roles", "binds_author_mode", "binds_validity_interval", "binds_epoch", "binds_predecessor_on_rotation"]:
        require(grant[k] is True, f"grant binding weakened: {k}")
    require(grant["domain_separated_commitment"] == "MYCELIX-PAYMENTS-PROVIDER-AUTHOR-GRANT\\0V1\\0", "grant domain drift")
    require(grant["wall_clock_is_authority_identity"] is False, "wall clock entered authority identity")
    require(grant["issuance_proof_is_qualified"] is False, "grant issuance overclaimed")

    direct = p["direct_author"]
    keys(direct, {"exact_action_author_required", "author_id_is_opaque_nonempty"}, "direct_author")
    require(all(direct.values()), "direct author theorem weakened")

    threshold = p["threshold_author"]
    keys(threshold, {
        "descriptor_binds_committee_id", "descriptor_binds_epoch", "descriptor_binds_threshold_and_member_count",
        "descriptor_binds_committee_commitment", "descriptor_binds_public_key_commitment", "descriptor_binds_scope_commitment",
        "proof_must_bind_exact_record_commitment", "existing_threshold_zome_qualifies_provider_authority",
        "existing_threshold_signature_create_verifies_cryptographic_signature_against_committee_key",
        "existing_threshold_entry_create_binds_action_author_to_committee_membership",
        "existing_threshold_link_validation_is_authority_sufficient",
    }, "threshold_author")
    for k in ["descriptor_binds_committee_id", "descriptor_binds_epoch", "descriptor_binds_threshold_and_member_count", "descriptor_binds_committee_commitment", "descriptor_binds_public_key_commitment", "descriptor_binds_scope_commitment", "proof_must_bind_exact_record_commitment"]:
        require(threshold[k] is True, f"threshold binding weakened: {k}")
    for k in ["existing_threshold_zome_qualifies_provider_authority", "existing_threshold_signature_create_verifies_cryptographic_signature_against_committee_key", "existing_threshold_entry_create_binds_action_author_to_committee_membership", "existing_threshold_link_validation_is_authority_sufficient"]:
        require(threshold[k] is False, f"threshold authority inflated: {k}")

    rotation = p["rotation"]
    keys(rotation, {
        "successor_epoch_must_equal_predecessor_plus_one", "successor_binds_exact_predecessor_commitment",
        "provider_profile_continuity_required", "jurisdiction_continuity_required", "upstream_authority_continuity_required",
        "role_amplification_forbidden", "successor_valid_from_equals_cutover", "same_epoch_competing_successor_allowed",
        "stateful_unique_successor_enforced_in_f1b2a",
    }, "rotation")
    for k in ["successor_epoch_must_equal_predecessor_plus_one", "successor_binds_exact_predecessor_commitment", "provider_profile_continuity_required", "jurisdiction_continuity_required", "upstream_authority_continuity_required", "role_amplification_forbidden", "successor_valid_from_equals_cutover"]:
        require(rotation[k] is True, f"rotation theorem weakened: {k}")
    require(rotation["same_epoch_competing_successor_allowed"] is False, "same-epoch competing successor allowed")
    require(rotation["stateful_unique_successor_enforced_in_f1b2a"] is False, "pairwise rotation overclaimed as stateful uniqueness")

    history = p["revocation_and_history"]
    keys(history, {
        "revocation_effective_at_candidate_action_time", "supersession_effective_at_candidate_action_time",
        "historical_pre_revocation_evidence_remains_valid", "historical_pre_cutover_evidence_remains_valid",
        "later_current_time_must_not_retroactively_invalidate_history",
        "historical_evidence_binds_grant_commitment_epoch_role_record_and_action_time",
    }, "revocation_and_history")
    for k, v in history.items():
        require(v is True, f"historical authority invariant weakened: {k}")

    activation = p["activation"]
    keys(activation, {"provider_authority_qualified", "threshold_provider_authority_qualified", "entry_types_registered", "link_types_registered", "validate_extern_active", "dht_write_surface_active", "provider_query_active", "replay_capability_minting", "automatic_replay", "payments_zome_wired", "governance_routing_active"}, "activation")
    for k, v in activation.items():
        require(v is False, f"premature activation: {k}")

    nxt = p["next_tranche"]
    keys(nxt, {
        "id", "must_refine_f1b2_admission_to_role_aware_authority",
        "must_verify_grant_issuance_against_upstream_capability",
        "must_cryptographically_verify_threshold_attestation",
        "must_reconstruct_threshold_committee_state_at_candidate_action_time",
        "must_reconstruct_grant_status_at_candidate_action_time",
        "must_enforce_stateful_unique_successor_epoch",
        "must_preserve_historical_author_evidence", "must_not_activate_live_payment_dispatch",
    }, "next_tranche")
    require(nxt["id"] == "MYC-CONST-003D1D-F1B2B", "next tranche drift")
    for k, v in nxt.items():
        if k != "id":
            require(v is True, f"next-tranche guard weakened: {k}")

    required_nonclaims = {
        "not_provider_authority_qualified", "not_grant_issuance_qualified",
        "not_stateful_authority_timeline_established", "not_threshold_signing_qualified_for_provider_authority",
        "not_holochain_persistent", "not_payments_zome_correctness_established",
        "not_public_fund_authority_delegated_to_automated_agent", "not_replay_qualified",
        "not_capability_minting_enabled", "not_exactly_once_physical_settlement",
        "not_external_finality_established", "not_governance_routing_active",
        "not_deployment_currentness_qualified", "not_qualified",
    }
    require(required_nonclaims <= set(p["non_claims"]), "required non-claims missing")


def validate_source(source: str, cargo: str, workspace: str) -> None:
    for marker in [
        "ProviderAuthorRole", "OrchestratorIntent", "OrchestratorDispatch", "ProviderObservation",
        "PublicFundsAuthorityAnchor", "Branch::Stewardship", "ConstitutionalPower::ExecuteAppropriation",
        "ProviderAuthorGrant", "ThresholdCommitteeDescriptor", "RecordAuthorProof", "action_time_us",
        "ProviderGrantStatus::", "validate_rotation", "rotation cannot amplify record roles",
        "signed_record_commitment", "provider profile binding mismatch",
    ]:
        require(marker in source, f"authority source missing semantic marker: {marker}")
    for forbidden in ["#[hdk_entry_types]", "#[hdk_link_types]", "#[hdk_extern]", "create_entry(", "create_link(", "mint_replay", "mint_capability", "execute_payment", "transfer_sap("]:
        require(forbidden not in source, f"authority source prematurely activates runtime surface: {forbidden}")
    require("constitutional-authority = { workspace = true }" in cargo, "lost constitutional-authority dependency")
    require("constitutional-payments-provider-journal = { workspace = true }" in cargo, "lost journal dependency")
    require('"crates/constitutional-payments-provider-authority"' in workspace, "missing workspace member")


def validate_bound_sources(p: dict) -> None:
    for key in ["constitutional_authority", "threshold_signing_integrity", "threshold_signing_coordinator"]:
        item = p["lineage"][key]
        actual = git("rev-parse", f"{EXPECTED['f1b2']}:{item['path']}")
        require(actual == item["git_blob_sha"], f"{key} blob re-derivation mismatch: {actual}")

    authority = git("show", f"{EXPECTED['f1b2']}:{p['lineage']['constitutional_authority']['path']}")
    for marker in ["automated agents do not hold constitutional sovereignty", "AuthorityPrincipal::AutomatedAgent => false", "delegated constitutional authority may narrow but never amplify its parent", "holder_id"]:
        require(marker in authority, f"constitutional-authority observation missing: {marker}")

    threshold = git("show", f"{EXPECTED['f1b2']}:{p['lineage']['threshold_signing_integrity']['path']}")
    for marker in ["pub threshold: u32", "pub member_count: u32", "pub public_key: Option<Vec<u8>>", "pub scope: CommitteeScope", "pub active: bool", "pub epoch: u32", "fn validate_create_signature(\n    _action: Create", "if sig.signature.len() < 64", "FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid)"]:
        require(marker in threshold, f"threshold observation missing: {marker}")


def self_test(profile: dict, source: str, cargo: str, workspace: str) -> None:
    mutations = [
        ("single-author-live", lambda p: p["role_model"].__setitem__("single_expected_author_is_sufficient_for_live_activation", True)),
        ("cross-role", lambda p: p["role_model"].__setitem__("cross_role_substitution_allowed", True)),
        ("automated-sovereignty", lambda p: p["constitutional_anchor"].__setitem__("automated_provider_becomes_constitutional_principal", True)),
        ("value-transfer", lambda p: p["constitutional_anchor"].__setitem__("operational_grant_can_transfer_value", True)),
        ("issuance-overclaim", lambda p: p["grant"].__setitem__("issuance_proof_is_qualified", True)),
        ("threshold-qualified", lambda p: p["threshold_author"].__setitem__("existing_threshold_zome_qualifies_provider_authority", True)),
        ("threshold-crypto-overclaim", lambda p: p["threshold_author"].__setitem__("existing_threshold_signature_create_verifies_cryptographic_signature_against_committee_key", True)),
        ("role-amplification", lambda p: p["rotation"].__setitem__("role_amplification_forbidden", False)),
        ("same-epoch-successor", lambda p: p["rotation"].__setitem__("same_epoch_competing_successor_allowed", True)),
        ("stateful-timeline-overclaim", lambda p: p["rotation"].__setitem__("stateful_unique_successor_enforced_in_f1b2a", True)),
        ("retroactive-revocation", lambda p: p["revocation_and_history"].__setitem__("historical_pre_revocation_evidence_remains_valid", False)),
        ("activate-authority", lambda p: p["activation"].__setitem__("provider_authority_qualified", True)),
        ("activate-dht", lambda p: p["activation"].__setitem__("entry_types_registered", True)),
        ("drop-issuance-next", lambda p: p["next_tranche"].__setitem__("must_verify_grant_issuance_against_upstream_capability", False)),
        ("drop-nonclaim", lambda p: p.__setitem__("non_claims", [x for x in p["non_claims"] if x != "not_qualified"])),
    ]
    for name, mutate in mutations:
        candidate = copy.deepcopy(profile)
        mutate(candidate)
        try:
            validate_profile(candidate)
        except ValueError:
            pass
        else:
            fail(f"profile mutation survived: {name}")

    source_mutations = [
        ("runtime-extern", source + "\n#[hdk_extern]\nfn bad() {}\n"),
        ("drop-role-marker", source.replace("ProviderObservation", "ProviderResult")),
        ("drop-execute-appropriation", source.replace("ConstitutionalPower::ExecuteAppropriation", "ConstitutionalPower::ExecuteLaw")),
    ]
    for name, candidate in source_mutations:
        try:
            validate_source(candidate, cargo, workspace)
        except ValueError:
            pass
        else:
            fail(f"source mutation survived: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    profile = json.loads(PROFILE.read_text())
    source = SOURCE.read_text()
    cargo = CARGO.read_text()
    workspace = WORKSPACE.read_text()
    validate_profile(profile)
    validate_source(source, cargo, workspace)
    validate_bound_sources(profile)
    if args.self_test:
        self_test(profile, source, cargo, workspace)
    print(json.dumps({
        "validated": True,
        "self_test": args.self_test,
        "profile_id": profile["profile_id"],
        "authority_class": profile["authority_class"],
        "f1b2_semantic_head": profile["lineage"]["f1b2_semantic_head"],
        "provider_authority_qualified": profile["activation"]["provider_authority_qualified"],
        "issuance_proof_is_qualified": profile["grant"]["issuance_proof_is_qualified"],
        "stateful_unique_successor_enforced_in_f1b2a": profile["rotation"]["stateful_unique_successor_enforced_in_f1b2a"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
