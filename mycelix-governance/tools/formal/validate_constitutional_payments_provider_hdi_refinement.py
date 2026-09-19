#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-F1B1 without third-party Python dependencies."""

from __future__ import annotations

import argparse
import copy
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
PROFILE = ROOT / "specs/constitutional-payments-provider-hdi-refinement.v1.json"
SOURCE = ROOT / "crates/constitutional-payments-provider-hdi/src/lib.rs"
CARGO = ROOT / "crates/constitutional-payments-provider-hdi/Cargo.toml"
WORKSPACE = ROOT / "Cargo.toml"

F1B0 = "1d0dc3f07ac29dccba9f33b45aef13380ecdb870"
F1B0_PATH = "mycelix-governance/crates/constitutional-payments-provider-journal/src/lib.rs"
F1B0_BLOB = "96abe9b514074904c33d7ef85fcdf008e4320932"


def fail(message: str) -> None:
    raise ValueError(message)


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT.parent, text=True).strip()


def validate_profile(profile: dict) -> None:
    require(profile.get("schema") == "mycelix.constitutional-payments-provider-hdi-refinement.v1", "schema drift")
    require(profile.get("profile_id") == "mycelix-payments-provider-hdi-f1b1-v1", "profile id drift")
    require(profile.get("authority_class") == "InactiveHdiRefinement", "authority inflation")

    lineage = profile["lineage"]
    require(lineage["f1b0_semantic_head"] == F1B0, "F1B0 lineage drift")
    require(lineage["f1b0_source"]["path"] == F1B0_PATH, "F1B0 source path drift")
    require(lineage["f1b0_source"]["git_blob_sha"] == F1B0_BLOB, "F1B0 source blob drift")

    projection = profile["entry_projection"]
    require(projection["hdk_entry_helper_derived"] is True, "HDI serialization proof removed")
    require(projection["registered_in_hdk_entry_types"] is False, "entry type prematurely registered")
    require(projection["exact_projection_from_f1b0_record"] is True, "exact refinement weakened")
    require(projection["second_record_hash_algorithm"] is False, "second record hash introduced")
    require(projection["second_state_reducer"] is False, "second reducer introduced")
    for key in [
        "journal_record_commitment_preserved",
        "previous_record_commitment_preserved",
        "sequence_preserved",
        "provider_operation_key_preserved",
        "audit_timestamp_preserved_as_metadata",
    ]:
        require(projection[key] is True, f"entry projection weakened: {key}")

    namespaces = profile["semantic_namespaces"]
    for key, value in namespaces.items():
        require(value is True, f"identity namespace collapsed: {key}")

    index = profile["index_projection"]
    require(index["execution_id_to_provider_operation_key"] is True, "execution index binding removed")
    require(index["genesis_record_commitment_bound"] is True, "genesis commitment removed from index")
    require(index["registered_in_hdk_link_types"] is False, "link type prematurely registered")
    require(index["live_link_creation"] is False, "live link creation enabled")

    authority = profile["authority"]
    require(authority["provider_author_binding"] == "PendingQualifiedProviderAuthority", "provider authority status inflated")
    for key in ["provider_authority_is_qualified", "entry_write_authority_established", "observation_write_authority_established"]:
        require(authority[key] is False, f"provider authority prematurely established: {key}")

    for key, value in profile["activation"].items():
        require(value is False, f"premature activation: {key}")

    requirements = set(profile["future_active_integrity_requirements"])
    required = {
        "bind every non-genesis record to exact predecessor record commitment",
        "prove provider author authority for intent, dispatch, and observation writers",
        "bind KnownSuccess receipt evidence to exact provider operation and F0 request",
        "forbid update/delete of constitutional journal facts",
        "keep historical Payment.id separate from provider operation key",
        "do not expose live payment dispatch while integrity refinement is unqualified",
    }
    require(required <= requirements, "future active-integrity requirements incomplete")

    non_claims = set(profile["non_claims"])
    required_non_claims = {
        "not_dht_registered",
        "not_dht_persistent",
        "not_provider_authorized",
        "not_payments_zome_wired",
        "not_provider_truth_established",
        "not_provider_replay_qualified",
        "not_capability_minting_enabled",
        "not_public_fund_authority_established",
        "not_exactly_once_physical_settlement",
        "not_governance_routing_active",
        "not_qualified",
    }
    require(required_non_claims <= non_claims, "required non-claims missing")


def validate_source(source: str, cargo: str, workspace: str) -> None:
    required = [
        "#[hdk_entry_helper]",
        "ProviderJournalEntryEnvelope",
        "ProviderOperationIndexProjection",
        "from_journal_record",
        "validate_refines_record",
        "from_journal",
        "validate_refines_journal",
        "provider_authority_is_qualified() -> bool",
        "false",
        "journal_record_commitment",
        "previous_record_commitment",
        "provider_operation_key",
        "payment_id",
    ]
    for needle in required:
        require(needle in source, f"HDI refinement source missing required marker: {needle}")

    forbidden = [
        "#[hdk_entry_types]",
        "#[hdk_link_types]",
        "#[hdk_extern]",
        "create_entry(",
        "update_entry(",
        "delete_entry(",
        "create_link(",
        "delete_link(",
        "fn validate(op: Op)",
        "QualifiedProviderReplayCapability",
        "mint_capability",
        "mint_replay",
    ]
    for needle in forbidden:
        require(needle not in source, f"inactive refinement opened forbidden runtime surface: {needle}")

    require("let canonical = Self::from_journal_record(record);" in source, "record refinement no longer canonical-forward projection")
    require("if self != &canonical" in source, "record refinement no longer exact equality")
    require("let canonical = Self::from_journal(journal)?;" in source, "index refinement no longer canonical-forward projection")
    require("assert_ne!(payment_id, entry.provider_operation_key);" in source, "historical payment ID separation test missing")
    require("assert!(!provider_authority_is_qualified());" in source, "fail-closed provider authority test missing")

    require("hdi = { workspace = true }" in cargo, "HDI dependency missing")
    require("constitutional-payments-provider-journal = { workspace = true }" in cargo, "F1B0 dependency missing")
    require("constitutional-treasury-effect-provider = { workspace = true }" in cargo, "F0 fixture dev dependency missing")
    require('"crates/constitutional-payments-provider-hdi"' in workspace, "HDI refinement workspace member missing")
    require("constitutional-payments-provider-hdi = { path = \"crates/constitutional-payments-provider-hdi\" }" in workspace, "HDI refinement workspace dependency missing")


def validate_git_binding(profile: dict) -> None:
    actual = git("rev-parse", f"{F1B0}:{F1B0_PATH}")
    require(actual == F1B0_BLOB, f"F1B0 source blob re-derivation mismatch: {actual}")
    require(profile["lineage"]["f1b0_source"]["git_blob_sha"] == actual, "profile does not bind re-derived F1B0 blob")


def self_test(profile: dict, source: str, cargo: str, workspace: str) -> None:
    profile_mutations = [
        ("register-entry", lambda p: p["entry_projection"].__setitem__("registered_in_hdk_entry_types", True)),
        ("second-hash", lambda p: p["entry_projection"].__setitem__("second_record_hash_algorithm", True)),
        ("second-reducer", lambda p: p["entry_projection"].__setitem__("second_state_reducer", True)),
        ("promote-payment-id", lambda p: p["semantic_namespaces"].__setitem__("historical_payment_id_distinct", False)),
        ("register-link", lambda p: p["index_projection"].__setitem__("registered_in_hdk_link_types", True)),
        ("live-link", lambda p: p["index_projection"].__setitem__("live_link_creation", True)),
        ("qualify-author", lambda p: p["authority"].__setitem__("provider_authority_is_qualified", True)),
        ("establish-write", lambda p: p["authority"].__setitem__("entry_write_authority_established", True)),
        ("activate-dht", lambda p: p["activation"].__setitem__("dht_write_surface_open", True)),
        ("activate-query", lambda p: p["activation"].__setitem__("provider_query_endpoint_live", True)),
        ("activate-minting", lambda p: p["activation"].__setitem__("provider_replay_capability_minting", True)),
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
            fail(f"profile self-test mutation survived: {name}")

    source_mutations = [
        ("entry-types", source + "\n#[hdk_entry_types]\nenum BadEntryTypes { Bad(ProviderJournalEntryEnvelope) }\n"),
        ("link-types", source + "\n#[hdk_link_types]\nenum BadLinkTypes { Bad }\n"),
        ("extern", source + "\n#[hdk_extern]\nfn bad(_: ()) -> ExternResult<()> { Ok(()) }\n"),
        ("create-entry", source + "\nfn bad_create() { let _ = create_entry(()); }\n"),
        ("authority-true", source.replace("pub fn provider_authority_is_qualified() -> bool {\n    false\n}", "pub fn provider_authority_is_qualified() -> bool {\n    true\n}")),
        ("weaken-record-equality", source.replace("if self != &canonical", "if false && self != &canonical", 1)),
        ("remove-payment-separation-test", source.replace("assert_ne!(payment_id, entry.provider_operation_key);", "assert_eq!(payment_id, payment_id);", 1)),
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
    validate_git_binding(profile)
    if args.self_test:
        self_test(profile, source, cargo, workspace)

    print(json.dumps({
        "validated": True,
        "self_test": args.self_test,
        "profile_id": profile["profile_id"],
        "authority_class": profile["authority_class"],
        "f1b0_semantic_head": profile["lineage"]["f1b0_semantic_head"],
        "dht_write_surface_open": profile["activation"]["dht_write_surface_open"],
        "provider_authority_is_qualified": profile["authority"]["provider_authority_is_qualified"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
