#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-F1B2 without third-party Python dependencies."""

from __future__ import annotations

import argparse
import copy
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
PROFILE = ROOT / "specs/constitutional-payments-provider-admission.v1.json"
SOURCE = ROOT / "crates/constitutional-payments-provider-admission/src/lib.rs"
CARGO = ROOT / "crates/constitutional-payments-provider-admission/Cargo.toml"
WORKSPACE = ROOT / "Cargo.toml"

EXPECTED = {
    "f1b0": "1d0dc3f07ac29dccba9f33b45aef13380ecdb870",
    "f1b0_blob": "96abe9b514074904c33d7ef85fcdf008e4320932",
    "f1b0_path": "mycelix-governance/crates/constitutional-payments-provider-journal/src/lib.rs",
    "f1b1": "c3ab0cad027ad3a3d8da0cb05390f3fb47ca3c08",
    "f1b1_blob": "92aa8dd3205feb20f25c7e6b2cc40291ab63e4ca",
    "f1b1_path": "mycelix-governance/crates/constitutional-payments-provider-hdi/src/lib.rs",
}


def fail(message: str) -> None:
    raise ValueError(message)


def require(value: bool, message: str) -> None:
    if not value:
        fail(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT.parent, text=True).strip()


def validate_profile(profile: dict) -> None:
    require(profile.get("schema") == "mycelix.constitutional-payments-provider-admission.v1", "schema drift")
    require(profile.get("profile_id") == "mycelix-payments-provider-admission-f1b2-v1", "profile id drift")
    require(profile.get("authority_class") == "PreRegistrationThreatModel", "authority inflation")

    lineage = profile["lineage"]
    for key, expected in [
        ("f1b0_semantic_head", EXPECTED["f1b0"]),
        ("f1b0_source_blob", EXPECTED["f1b0_blob"]),
        ("f1b1_semantic_head", EXPECTED["f1b1"]),
        ("f1b1_source_blob", EXPECTED["f1b1_blob"]),
    ]:
        require(lineage[key] == expected, f"lineage drift: {key}")

    authority = profile["authority"]
    require(authority["provider_authority_live"] is False, "provider authority prematurely live")
    require(authority["reference_qualified_authority_is_serializable"] is False, "reference authority became portable")
    require(authority["candidate_supplies_expected_authority"] is False, "candidate supplies expected authority")
    require(authority["author_must_match_expected_provider"] is True, "author binding weakened")
    require(authority["provider_profile_commitment_bound"] is True, "provider profile binding weakened")

    records = profile["record_admission"]
    for key in [
        "exact_f1b1_projection_required",
        "genesis_sequence_zero_required",
        "non_genesis_predecessor_required",
        "contiguous_sequence_required",
        "same_slot_exact_duplicate_is_idempotent",
        "same_slot_changed_content_halts",
        "updates_forbidden",
        "deletes_forbidden",
        "post_halt_writes_forbidden",
    ]:
        require(records[key] is True, f"record admission weakened: {key}")

    index = profile["index_admission"]
    for key in [
        "genesis_must_be_admitted_first",
        "execution_to_operation_to_genesis_binding",
        "exact_duplicate_is_idempotent",
        "conflicting_index_halts",
        "delete_forbidden",
    ]:
        require(index[key] is True, f"index admission weakened: {key}")
    require(index["last_write_wins"] is False, "last-write-wins enabled")

    transplants = profile["transplant_defenses"]
    for key in [
        "operation_key_bound",
        "execution_id_bound",
        "request_commitment_bound",
        "receipt_evidence_bound",
        "attempt_horizon_bound",
    ]:
        require(transplants[key] is True, f"transplant defense weakened: {key}")
    require(transplants["historical_payment_id_is_operation_authority"] is False, "Payment.id promoted to authority")

    required_attacks = {
        "direct_dht_spoofed_author", "orphan_non_genesis_record", "forged_predecessor",
        "same_sequence_changed_content", "duplicate_operation_index_race",
        "conflicting_operation_index_race", "receipt_transplant", "request_transplant",
        "attempt_horizon_transplant", "record_update", "record_delete", "index_delete",
        "post_halt_write",
    }
    require(required_attacks <= set(profile["attacks"]), "attack census incomplete")

    for key, value in profile["activation"].items():
        require(value is False, f"premature activation: {key}")

    next_tranche = profile["next_tranche"]
    require(next_tranche["id"] == "MYC-CONST-003D1D-F1B3", "next tranche drift")
    require(next_tranche["must_reconstruct_oracle_from_dht_facts"] is True, "future DHT reconstruction requirement removed")
    require(next_tranche["must_not_trust_candidate_supplied_authority"] is True, "candidate authority trust introduced")
    require(next_tranche["must_keep_payment_id_non_authoritative"] is True, "Payment.id authority boundary weakened")

    required_non_claims = {
        "not_holochain_persistent", "not_provider_authority_established", "not_provider_truth_established",
        "not_payments_zome_wired", "not_replay_qualified", "not_capability_minting_enabled",
        "not_public_fund_authority_established", "not_exactly_once_physical_settlement",
        "not_external_finality_established", "not_governance_routing_active", "not_qualified",
    }
    require(required_non_claims <= set(profile["non_claims"]), "required non-claims missing")


def validate_source(source: str, cargo: str, workspace: str) -> None:
    required = [
        "ReferenceProviderAuthority", "QualifiedForModelOnly", "validate_refines_record",
        "MissingPredecessor", "PredecessorMismatch", "SameSequenceConflict",
        "ConflictingOperationIndex", "MutationForbidden", "receipt_transplant_is_rejected",
        "request_transplant_is_rejected", "attempt_horizon_transplant_is_rejected",
        "same_sequence_changed_content_halts_and_is_sticky",
    ]
    for needle in required:
        require(needle in source, f"source missing threat-model marker: {needle}")

    forbidden = [
        "#[hdk_entry_types]", "#[hdk_link_types]", "#[hdk_extern]", "create_entry(",
        "update_entry(", "delete_entry(", "create_link(", "delete_link(",
        "Serialize, Deserialize", "last_write_wins",
    ]
    for needle in forbidden:
        require(needle not in source, f"source activates or weakens admission surface: {needle}")

    require("constitutional-payments-provider-hdi = { workspace = true }" in cargo, "F1B1 dependency missing")
    require("constitutional-payments-provider-journal = { workspace = true }" in cargo, "F1B0 dependency missing")
    require('"crates/constitutional-payments-provider-admission"' in workspace, "workspace member missing")


def validate_git_bindings() -> None:
    f1b0_blob = git("rev-parse", f"{EXPECTED['f1b0']}:{EXPECTED['f1b0_path']}")
    f1b1_blob = git("rev-parse", f"{EXPECTED['f1b1']}:{EXPECTED['f1b1_path']}")
    require(f1b0_blob == EXPECTED["f1b0_blob"], f"F1B0 blob mismatch: {f1b0_blob}")
    require(f1b1_blob == EXPECTED["f1b1_blob"], f"F1B1 blob mismatch: {f1b1_blob}")


def self_test(profile: dict, source: str, cargo: str, workspace: str) -> None:
    mutations = [
        ("authority-live", lambda p: p["authority"].__setitem__("provider_authority_live", True)),
        ("serializable-authority", lambda p: p["authority"].__setitem__("reference_qualified_authority_is_serializable", True)),
        ("remove-author-binding", lambda p: p["authority"].__setitem__("author_must_match_expected_provider", False)),
        ("allow-update", lambda p: p["record_admission"].__setitem__("updates_forbidden", False)),
        ("last-write-wins", lambda p: p["index_admission"].__setitem__("last_write_wins", True)),
        ("receipt-transplant", lambda p: p["transplant_defenses"].__setitem__("receipt_evidence_bound", False)),
        ("payment-id-authority", lambda p: p["transplant_defenses"].__setitem__("historical_payment_id_is_operation_authority", True)),
        ("activate-entry-types", lambda p: p["activation"].__setitem__("entry_types_registered", True)),
        ("activate-write-surface", lambda p: p["activation"].__setitem__("dht_write_surface_live", True)),
        ("remove-not-qualified", lambda p: p.__setitem__("non_claims", [x for x in p["non_claims"] if x != "not_qualified"])),
    ]
    for name, mutate in mutations:
        candidate = copy.deepcopy(profile)
        mutate(candidate)
        try:
            validate_profile(candidate)
        except ValueError:
            pass
        else:
            fail(f"profile self-test mutation survived: {name}")

    source_mutations = [
        ("entry-registration", source + "\n#[hdk_entry_types]\nenum Bad {}\n"),
        ("extern", source + "\n#[hdk_extern]\nfn bad() {}\n"),
        ("portable-authority", source + "\n#[derive(Serialize, Deserialize)]\nstruct PortableAuthority;\n"),
        ("remove-predecessor", source.replace("PredecessorMismatch", "PredecessorBypass", 1)),
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
    validate_git_bindings()
    if args.self_test:
        self_test(profile, source, cargo, workspace)
    print(json.dumps({
        "validated": True,
        "self_test": args.self_test,
        "profile_id": profile["profile_id"],
        "authority_class": profile["authority_class"],
        "entry_types_registered": profile["activation"]["entry_types_registered"],
        "dht_write_surface_live": profile["activation"]["dht_write_surface_live"],
        "provider_authority_live": profile["authority"]["provider_authority_live"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(f"validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
