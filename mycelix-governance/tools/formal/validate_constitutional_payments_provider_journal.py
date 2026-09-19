#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-F1B0 without third-party Python dependencies."""

from __future__ import annotations

import argparse
import copy
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
PROFILE = ROOT / "specs/constitutional-payments-provider-journal.v1.json"
SOURCE = ROOT / "crates/constitutional-payments-provider-journal/src/lib.rs"
CARGO = ROOT / "crates/constitutional-payments-provider-journal/Cargo.toml"
WORKSPACE = ROOT / "Cargo.toml"

EXPECTED = {
    "f0c": "36bb486794f3e2720a2e02b3045c270b147cbacc",
    "f0p0": "0442bc487dc5ea3303144b3f7be8979c673d6018",
    "f1a": "4879c83c4ef21c15e124730a1518ec68812d1611",
    "f1a_blob": "6e4df936446b40db09b2f3fee0565051b3fac747",
}


def fail(message: str) -> None:
    raise ValueError(message)


def require(value: bool, message: str) -> None:
    if not value:
        fail(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT.parent, text=True).strip()


def validate_profile(profile: dict) -> None:
    require(profile.get("schema") == "mycelix.constitutional-payments-provider-journal.v1", "schema drift")
    require(profile.get("profile_id") == "mycelix-payments-provider-journal-f1b0-v1", "profile id drift")
    require(profile.get("authority_class") == "InertSemanticContract", "authority inflation")

    lineage = profile["lineage"]
    require(lineage["f0c_semantic_head"] == EXPECTED["f0c"], "F0C lineage drift")
    require(lineage["f0p0_semantic_head"] == EXPECTED["f0p0"], "F0P0 lineage drift")
    require(lineage["f1a_semantic_head"] == EXPECTED["f1a"], "F1A lineage drift")
    require(lineage["f1a_source"]["git_blob_sha"] == EXPECTED["f1a_blob"], "F1A source blob drift")

    journal = profile["journal"]
    require(journal["record_types"] == ["OperationIntent", "DispatchAttempt", "Observation"], "record census drift")
    for key in ["append_only", "hash_linked_per_operation", "contiguous_sequence_required", "query_state_derived_by_replay", "integrity_halt_sticky"]:
        require(journal[key] is True, f"journal invariant weakened: {key}")
    require(journal["wall_clock_in_semantic_record_identity"] is False, "wall clock entered identity")
    require(journal["mutable_authoritative_status_stored"] is False, "mutable status became authoritative")

    identity = profile["operation_identity"]
    for key in [
        "provider_operation_key_reused_from_f1a",
        "execution_id_bound",
        "request_commitment_bound",
        "provider_profile_id_bound",
        "provider_profile_commitment_bound",
    ]:
        require(identity[key] is True, f"operation identity weakened: {key}")
    require(identity["historical_payment_id_is_operation_key"] is False, "historical payment id promoted to operation authority")

    dispatch = profile["dispatch"]
    for key in [
        "attempt_ids_unique",
        "attempt_ordinals_contiguous",
        "unknown_outcome_blocks_blind_redispatch",
        "known_no_effect_through_current_horizon_permits_next_attempt",
        "dispatch_after_integrity_halt_forbidden",
    ]:
        require(dispatch[key] is True, f"dispatch invariant weakened: {key}")
    require(dispatch["qualified_replay_capability_consumption_present"] is False, "unqualified replay consumption added")
    require(dispatch["automatic_replay_enabled"] is False, "automatic replay enabled")

    observations = profile["observations"]
    require(observations["outcomes"] == ["KnownSuccess", "KnownNoEffect", "UnknownOutcome"], "outcome census drift")
    for key in [
        "attempt_horizon_bound",
        "exact_operation_key_bound",
        "exact_execution_id_bound",
        "exact_request_commitment_bound",
        "observation_id_collision_halts",
        "same_horizon_success_no_effect_contradiction_halts",
        "later_attempt_success_after_earlier_no_effect_allowed",
        "stale_no_effect_cannot_demote_newer_inflight",
        "stale_unknown_cannot_demote_newer_inflight",
        "conflicting_success_receipts_halt",
        "later_unknown_cannot_demote_terminal_success",
        "forensic_observations_after_halt_cannot_clear_halt",
    ]:
        require(observations[key] is True, f"observation invariant weakened: {key}")

    query = profile["query"]
    require(query["results"] == ["NotFound", "IntentCommitted", "InFlight", "UnknownOutcome", "KnownNoEffect", "KnownSuccess", "IntegrityHalted"], "query result census drift")
    for key in [
        "journal_head_commitment_returned",
        "query_by_provider_operation_key",
        "query_by_execution_id",
        "lost_ack_success_recoverable_by_query",
    ]:
        require(query[key] is True, f"query invariant weakened: {key}")

    for key, value in profile["activation"].items():
        require(value is False, f"premature activation: {key}")

    next_tranche = profile["next_tranche"]
    require(next_tranche["id"] == "MYC-CONST-003D1D-F1B1", "next tranche drift")
    for key in [
        "must_not_activate_live_payment_dispatch",
        "must_preserve_separate_historical_payment_id",
        "must_bind_receipts_to_provider_operation_key_and_f0_request",
        "must_preserve_unknown_outcome",
        "must_preserve_stale_evidence_monotonicity",
        "must_preserve_sticky_integrity_halt",
    ]:
        require(next_tranche[key] is True, f"F1B1 guard weakened: {key}")

    non_claims = set(profile["non_claims"])
    required_non_claims = {
        "not_holochain_persistent",
        "not_payments_zome_wired",
        "not_provider_truth_established",
        "not_provider_replay_qualified",
        "not_capability_minting_enabled",
        "not_public_fund_authority_established",
        "not_exactly_once_physical_settlement",
        "not_external_finality_established",
        "not_governance_routing_active",
        "not_qualified",
    }
    require(required_non_claims <= non_claims, "required non-claims missing")


def validate_source(source: str, cargo: str, workspace: str) -> None:
    required = [
        "previous_record_commitment",
        "covers_through_attempt_ordinal",
        "UnknownOutcome blocks another dispatch",
        "dispatch cannot occur after journal integrity halt",
        "ProviderQueryResult",
        "journal_head_commitment",
        "SuccessContradictsNoEffect",
        "NoEffectContradictsSuccess",
        "ConflictingSuccessEvidence",
        "stale_no_effect_does_not_demote_newer_in_flight_attempt",
        "stale_unknown_does_not_demote_newer_in_flight_attempt",
        "integrity_halt_is_sticky_under_later_forensic_observation",
        "recorded_at_unix_ms",
    ]
    for needle in required:
        require(needle in source, f"journal source missing required semantic marker: {needle}")

    forbidden = [
        "#[hdk_extern]",
        "create_entry(",
        "create_link(",
        "record_mut(",
        "pub fn delete",
        "pub fn set_status",
        "QualifiedProviderReplayCapability",
        "mint_capability",
        "mint_replay",
    ]
    for needle in forbidden:
        require(needle not in source, f"journal source prematurely activates/bypasses protected surface: {needle}")

    require("constitutional-payments-provider = { workspace = true }" in cargo, "journal crate lost F1A dependency")
    require("constitutional-treasury-effect-provider = { workspace = true }" in cargo, "journal tests lost explicit F0 dev-dependency")
    require('"crates/constitutional-payments-provider-journal"' in workspace, "journal crate missing workspace registration")
    require("constitutional-payments-provider-journal = { path = \"crates/constitutional-payments-provider-journal\" }" in workspace, "journal workspace dependency missing")


def validate_git_bindings(profile: dict) -> None:
    source = profile["lineage"]["f1a_source"]
    actual_blob = git("rev-parse", f"{EXPECTED['f1a']}:{source['path']}")
    require(actual_blob == EXPECTED["f1a_blob"], f"F1A blob re-derivation mismatch: {actual_blob}")


def self_test(profile: dict, source: str, cargo: str, workspace: str) -> None:
    profile_mutations = [
        ("append-only", lambda p: p["journal"].__setitem__("append_only", False)),
        ("wall-clock-identity", lambda p: p["journal"].__setitem__("wall_clock_in_semantic_record_identity", True)),
        ("mutable-status", lambda p: p["journal"].__setitem__("mutable_authoritative_status_stored", True)),
        ("clear-halt", lambda p: p["journal"].__setitem__("integrity_halt_sticky", False)),
        ("blind-replay", lambda p: p["dispatch"].__setitem__("unknown_outcome_blocks_blind_redispatch", False)),
        ("dispatch-after-halt", lambda p: p["dispatch"].__setitem__("dispatch_after_integrity_halt_forbidden", False)),
        ("fake-capability-consumption", lambda p: p["dispatch"].__setitem__("qualified_replay_capability_consumption_present", True)),
        ("automatic-replay", lambda p: p["dispatch"].__setitem__("automatic_replay_enabled", True)),
        ("remove-horizon", lambda p: p["observations"].__setitem__("attempt_horizon_bound", False)),
        ("collapse-later-success", lambda p: p["observations"].__setitem__("later_attempt_success_after_earlier_no_effect_allowed", False)),
        ("stale-noeffect-demotes", lambda p: p["observations"].__setitem__("stale_no_effect_cannot_demote_newer_inflight", False)),
        ("stale-unknown-demotes", lambda p: p["observations"].__setitem__("stale_unknown_cannot_demote_newer_inflight", False)),
        ("forensic-clears-halt", lambda p: p["observations"].__setitem__("forensic_observations_after_halt_cannot_clear_halt", False)),
        ("activate-persistence", lambda p: p["activation"].__setitem__("persistent_holochain_entries", True)),
        ("activate-minting", lambda p: p["activation"].__setitem__("provider_replay_capability_minting", True)),
        ("promote-payment-id", lambda p: p["operation_identity"].__setitem__("historical_payment_id_is_operation_key", True)),
        ("remove-not-qualified", lambda p: p.__setitem__("non_claims", [x for x in p["non_claims"] if x != "not_qualified"])),
    ]
    for name, mutate in profile_mutations:
        candidate = copy.deepcopy(profile)
        mutate(candidate)
        try:
            validate_profile(candidate)
        except ValueError:
            pass
        else:
            fail(f"self-test mutation survived: {name}")

    source_mutations = [
        ("runtime-extern", source + "\n#[hdk_extern]\nfn bad() {}\n"),
        ("capability-mint", source + "\nfn mint_capability() {}\n"),
        ("mutable-escape", source + "\npub fn record_mut() {}\n"),
        ("remove-horizon-marker", source.replace("covers_through_attempt_ordinal", "covers_attempt", 1)),
        ("remove-predecessor", source.replace("previous_record_commitment", "prior", 1)),
        ("remove-stale-noeffect-test", source.replace("stale_no_effect_does_not_demote_newer_in_flight_attempt", "removed_test", 1)),
        ("remove-sticky-halt-test", source.replace("integrity_halt_is_sticky_under_later_forensic_observation", "removed_test", 1)),
    ]
    for name, candidate in source_mutations:
        try:
            validate_source(candidate, cargo, workspace)
        except ValueError:
            pass
        else:
            fail(f"source self-test mutation survived: {name}")


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
    validate_git_bindings(profile)
    if args.self_test:
        self_test(profile, source, cargo, workspace)

    print(json.dumps({
        "validated": True,
        "self_test": args.self_test,
        "profile_id": profile["profile_id"],
        "authority_class": profile["authority_class"],
        "f1a_semantic_head": profile["lineage"]["f1a_semantic_head"],
        "automatic_replay": profile["activation"]["automatic_replay"],
        "persistent_holochain_entries": profile["activation"]["persistent_holochain_entries"],
        "provider_replay_capability_minting": profile["activation"]["provider_replay_capability_minting"],
        "integrity_halt_sticky": profile["journal"]["integrity_halt_sticky"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
