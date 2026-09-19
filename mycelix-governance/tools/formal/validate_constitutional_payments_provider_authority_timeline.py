#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-F1B2B0 using stdlib + Git."""

from __future__ import annotations

import argparse
import copy
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
REPO = ROOT.parent
PROFILE = ROOT / "specs/constitutional-payments-provider-authority-timeline.v1.json"
SCHEMA = ROOT / "specs/constitutional-payments-provider-authority-timeline.v1.schema.json"
SOURCE = ROOT / "crates/constitutional-payments-provider-authority-timeline/src/lib.rs"
CARGO = ROOT / "crates/constitutional-payments-provider-authority-timeline/Cargo.toml"
WORKSPACE = ROOT / "Cargo.toml"

EXPECTED_F1B2A_HEAD = "d9f1ad518e2943eb64c9991b2589cf9a221fc047"
EXPECTED_F1B2A_PATH = "mycelix-governance/crates/constitutional-payments-provider-authority/src/lib.rs"
EXPECTED_F1B2A_BLOB = "be010d8e63805ce7b788fd3ca554aa2d6a555f1c"

REQUIRED_NONCLAIMS = {
    "not_provider_authority_timeline_qualified",
    "not_cryptographic_grant_issuance_verified",
    "not_cryptographic_revocation_verified",
    "not_threshold_attestation_verified",
    "not_threshold_committee_state_reconstructed",
    "not_holochain_persistent",
    "not_live_admission_composed",
    "not_payments_zome_correctness_established",
    "not_replay_qualified",
    "not_capability_minting_enabled",
    "not_exactly_once_physical_settlement",
    "not_governance_routing_active",
    "not_deployment_currentness_qualified",
    "not_qualified",
}


def fail(message: str) -> None:
    raise ValueError(message)


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def exact_keys(obj: dict, expected: set[str], label: str) -> None:
    require(set(obj) == expected, f"{label} key census drift: {sorted(set(obj) ^ expected)}")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def require_all_true(section: dict, keys: set[str], label: str) -> None:
    for key in keys:
        require(section[key] is True, f"{label} invariant weakened: {key}")


def validate_profile(profile: dict) -> None:
    exact_keys(
        profile,
        {
            "schema", "profile_id", "authority_class", "lineage", "timeline", "issuance",
            "rotation", "revocation", "historical_authorization", "threshold", "activation",
            "next_tranche", "non_claims",
        },
        "root",
    )
    require(
        profile["schema"] == "mycelix.constitutional-payments-provider-authority-timeline.v1",
        "schema drift",
    )
    require(
        profile["profile_id"] == "mycelix-payments-provider-authority-timeline-f1b2b0-v1",
        "profile id drift",
    )
    require(profile["authority_class"] == "InertStatefulTimeline", "authority class inflation")

    lineage = profile["lineage"]
    exact_keys(
        lineage,
        {"f1b2a_semantic_head", "f1b2a_source_path", "f1b2a_source_git_blob_sha"},
        "lineage",
    )
    require(lineage["f1b2a_semantic_head"] == EXPECTED_F1B2A_HEAD, "F1B2A head drift")
    require(lineage["f1b2a_source_path"] == EXPECTED_F1B2A_PATH, "F1B2A path drift")
    require(lineage["f1b2a_source_git_blob_sha"] == EXPECTED_F1B2A_BLOB, "F1B2A blob drift")

    timeline_true = {
        "append_only_events", "contiguous_event_sequence", "stable_event_identity",
        "exact_event_replay_is_idempotent", "same_sequence_conflict_halts",
        "event_id_reuse_halts", "integrity_halt_is_sticky", "wall_clock_is_not_authority_ordering",
    }
    exact_keys(profile["timeline"], timeline_true, "timeline")
    require_all_true(profile["timeline"], timeline_true, "timeline")

    issuance_true = {
        "binds_exact_grant_commitment", "binds_upstream_capability_id", "binds_upstream_holder_id",
        "binds_upstream_jurisdiction", "binds_upstream_authority_commitment", "binds_issuance_time",
        "binds_external_proof_commitment", "issuance_not_after_grant_valid_from",
        "upstream_authority_valid_at_issuance", "conflicting_issuance_halts",
    }
    issuance = profile["issuance"]
    exact_keys(issuance, issuance_true | {"cryptographic_issuance_proof_verified"}, "issuance")
    require_all_true(issuance, issuance_true, "issuance")
    require(issuance["cryptographic_issuance_proof_verified"] is False, "cryptographic issuance overclaim")

    rotation_true = {
        "uses_f1b2a_validate_rotation", "exact_predecessor_binding_required", "successor_epoch_monotone",
        "profile_jurisdiction_upstream_continuity_required", "role_amplification_forbidden",
        "successor_valid_from_equals_cutover", "one_canonical_successor_per_predecessor",
        "competing_successor_halts", "same_successor_different_cutover_halts",
        "cutover_not_after_predecessor_expiry",
    }
    rotation = profile["rotation"]
    exact_keys(rotation, rotation_true, "rotation")
    require_all_true(rotation, rotation_true, "rotation")

    revocation_true = {
        "binds_exact_grant_commitment", "binds_exact_upstream_authority",
        "authorization_not_after_effective_time", "upstream_authority_valid_at_authorization_and_effective_time",
        "effective_time_not_before_grant_validity", "effective_time_not_after_grant_expiry",
        "one_terminal_event_per_grant", "conflicting_terminal_history_halts",
    }
    revocation = profile["revocation"]
    exact_keys(revocation, revocation_true | {"cryptographic_revocation_proof_verified"}, "revocation")
    require_all_true(revocation, revocation_true, "revocation")
    require(revocation["cryptographic_revocation_proof_verified"] is False, "cryptographic revocation overclaim")

    history_true = {
        "status_reconstructed_at_candidate_action_time", "pre_revocation_history_preserved",
        "pre_cutover_history_preserved", "expiry_is_time_relative", "composes_f1b2a_authorize_record",
        "role_profile_jurisdiction_checks_preserved", "integrity_halt_blocks_authorization",
    }
    history = profile["historical_authorization"]
    exact_keys(history, history_true | {"current_wall_clock_used_for_status"}, "historical_authorization")
    require_all_true(history, history_true, "historical_authorization")
    require(history["current_wall_clock_used_for_status"] is False, "current wall clock entered status")

    threshold = profile["threshold"]
    exact_keys(
        threshold,
        {
            "f1b2a_threshold_descriptor_preserved",
            "cryptographic_threshold_attestation_verified",
            "threshold_committee_state_reconstructed",
        },
        "threshold",
    )
    require(threshold["f1b2a_threshold_descriptor_preserved"] is True, "threshold descriptor dropped")
    require(threshold["cryptographic_threshold_attestation_verified"] is False, "threshold crypto overclaim")
    require(threshold["threshold_committee_state_reconstructed"] is False, "threshold state overclaim")

    activation = profile["activation"]
    exact_keys(
        activation,
        {
            "provider_authority_timeline_qualified", "grant_issuance_cryptographically_qualified",
            "threshold_provider_authority_qualified", "entry_types_registered", "link_types_registered",
            "validate_extern_active", "dht_write_surface_active", "live_admission_composed",
            "payments_zome_wired", "governance_routing_active", "replay_capability_minting",
        },
        "activation",
    )
    for key, value in activation.items():
        require(value is False, f"premature activation: {key}")

    next_tranche = profile["next_tranche"]
    exact_keys(
        next_tranche,
        {
            "id", "must_compose_timeline_with_f1b2_admission", "must_bind_actual_holochain_action_author",
            "must_verify_issuance_proof_before_live_authority", "must_verify_threshold_attestation_cryptographically",
            "must_reconstruct_threshold_committee_state_at_action_time", "must_not_activate_live_payment_dispatch",
        },
        "next_tranche",
    )
    require(next_tranche["id"] == "MYC-CONST-003D1D-F1B2B1", "next tranche drift")
    for key, value in next_tranche.items():
        if key != "id":
            require(value is True, f"next-tranche gate weakened: {key}")

    require(REQUIRED_NONCLAIMS <= set(profile["non_claims"]), "required non-claims missing")


def validate_source(source: str, cargo: str, workspace: str) -> None:
    required = [
        "GrantIssuanceEvidence", "GrantRevocationEvidence", "AuthorityTimelineEvent",
        "ProviderAuthorityTimeline", "TimelineFault", "CompetingSuccessor",
        "ConflictingTerminalHistory", "ConflictingIssuance", "SameSequenceConflict",
        "EventIdConflict", "status_at", "authorize_at", "validate_rotation", "authorize_record",
        "HistoricalGrantStatus::Active", "evidence.action_time_us", "self.events.len() as u64",
        "same_successor_with_different_cutover_halts", "integrity_fault",
    ]
    for marker in required:
        require(marker in source, f"timeline source missing semantic marker: {marker}")

    forbidden = [
        "#[hdk_entry_types]", "#[hdk_link_types]", "#[hdk_extern]", "create_entry(", "create_link(",
        "sys_time()", "SystemTime::now", "Utc::now", "mint_replay", "mint_capability",
        "execute_payment", "transfer_sap(",
    ]
    for marker in forbidden:
        require(marker not in source, f"timeline source prematurely activates runtime/time surface: {marker}")

    require(
        "constitutional-payments-provider-authority = { workspace = true }" in cargo,
        "timeline crate lost F1B2A dependency",
    )
    require("hdk" not in cargo and "hdi" not in cargo, "timeline crate gained Holochain host dependency")
    require(
        '"crates/constitutional-payments-provider-authority-timeline"' in workspace,
        "timeline crate missing workspace member",
    )
    require(
        "constitutional-payments-provider-authority-timeline = { path = \"crates/constitutional-payments-provider-authority-timeline\" }" in workspace,
        "timeline workspace dependency missing",
    )


def validate_bound_parent(profile: dict) -> None:
    lineage = profile["lineage"]
    actual = git("rev-parse", f"{lineage['f1b2a_semantic_head']}:{lineage['f1b2a_source_path']}")
    require(actual == lineage["f1b2a_source_git_blob_sha"], f"F1B2A source blob mismatch: {actual}")
    parent = git("show", f"{lineage['f1b2a_semantic_head']}:{lineage['f1b2a_source_path']}")
    for marker in [
        "pub fn validate_rotation(", "pub fn authorize_record(", "ProviderGrantStatus",
        "action_time_us", "rotation cannot amplify record roles", "provider profile binding mismatch",
    ]:
        require(marker in parent, f"F1B2A source observation missing: {marker}")


def self_test(profile: dict, source: str, cargo: str, workspace: str) -> None:
    mutations: list[tuple[str, dict]] = []

    def mutate(name: str, fn) -> None:
        candidate = copy.deepcopy(profile)
        fn(candidate)
        mutations.append((name, candidate))

    mutate("claim-crypto-issuance", lambda p: p["issuance"].__setitem__("cryptographic_issuance_proof_verified", True))
    mutate("claim-crypto-revocation", lambda p: p["revocation"].__setitem__("cryptographic_revocation_proof_verified", True))
    mutate("claim-threshold-crypto", lambda p: p["threshold"].__setitem__("cryptographic_threshold_attestation_verified", True))
    mutate("claim-threshold-state", lambda p: p["threshold"].__setitem__("threshold_committee_state_reconstructed", True))
    mutate("drop-unique-successor", lambda p: p["rotation"].__setitem__("one_canonical_successor_per_predecessor", False))
    mutate("drop-exact-cutover", lambda p: p["rotation"].__setitem__("same_successor_different_cutover_halts", False))
    mutate("drop-pre-revocation-history", lambda p: p["historical_authorization"].__setitem__("pre_revocation_history_preserved", False))
    mutate("enable-wall-clock", lambda p: p["historical_authorization"].__setitem__("current_wall_clock_used_for_status", True))
    mutate("activate-dht", lambda p: p["activation"].__setitem__("dht_write_surface_active", True))
    mutate("parent-blob-drift", lambda p: p["lineage"].__setitem__("f1b2a_source_git_blob_sha", "0" * 40))
    mutate("drop-not-qualified", lambda p: p.__setitem__("non_claims", [x for x in p["non_claims"] if x != "not_qualified"]))

    for name, candidate in mutations:
        try:
            validate_profile(candidate)
            validate_bound_parent(candidate)
        except (ValueError, subprocess.CalledProcessError):
            continue
        fail(f"profile mutation survived: {name}")

    source_mutations = [
        ("runtime-entry", source + "\n#[hdk_extern]\nfn bad() {}\n"),
        ("drop-competing-successor", source.replace("CompetingSuccessor", "AlternateSuccessor")),
        ("drop-cutover-test", source.replace("same_successor_with_different_cutover_halts", "cutover_test_removed")),
        ("drop-authorize-composition", source.replace("authorize_record", "authorize_record_removed")),
        ("drop-status-query", source.replace("status_at", "status_query_removed")),
    ]
    for name, candidate in source_mutations:
        try:
            validate_source(candidate, cargo, workspace)
        except ValueError:
            continue
        fail(f"source mutation survived: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    profile = json.loads(PROFILE.read_text())
    schema = json.loads(SCHEMA.read_text())
    source = SOURCE.read_text()
    cargo = CARGO.read_text()
    workspace = WORKSPACE.read_text()

    require(schema.get("$schema") == "https://json-schema.org/draft/2020-12/schema", "wrong schema draft")
    require(
        schema.get("$id") == "https://mycelix.dev/schemas/constitutional-payments-provider-authority-timeline.v1.schema.json",
        "wrong schema id",
    )
    require(schema.get("additionalProperties") is False, "root schema not closed")

    validate_profile(profile)
    validate_source(source, cargo, workspace)
    validate_bound_parent(profile)
    if args.self_test:
        self_test(profile, source, cargo, workspace)

    print(
        json.dumps(
            {
                "validated": True,
                "self_test": args.self_test,
                "profile_id": profile["profile_id"],
                "authority_class": profile["authority_class"],
                "f1b2a_semantic_head": profile["lineage"]["f1b2a_semantic_head"],
                "same_successor_different_cutover_halts": profile["rotation"]["same_successor_different_cutover_halts"],
                "cryptographic_issuance_proof_verified": profile["issuance"]["cryptographic_issuance_proof_verified"],
                "cryptographic_threshold_attestation_verified": profile["threshold"]["cryptographic_threshold_attestation_verified"],
                "dht_write_surface_active": profile["activation"]["dht_write_surface_active"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
