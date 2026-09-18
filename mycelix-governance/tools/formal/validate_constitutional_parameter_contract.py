#!/usr/bin/env python3
"""Independent source-bound validator for MYC-CONST-003D1D-P0.

This validator intentionally uses only the Python standard library and Git. It
checks the machine-readable P0 claim ceiling against exact historical source
objects and runs mutation-style self-tests for the most important fail-closed
properties. It does not qualify Holochain runtime authority.
"""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "mycelix-governance/specs/constitutional-parameter-contract.v1.json"

EXPECTED_TOP_LEVEL = {
    "schema",
    "contract_id",
    "contract_revision",
    "authority_class",
    "activation_allowed",
    "provider_key",
    "parameter_key",
    "source_binding",
    "identity",
    "effect_ledger_mapping",
    "value_profile",
    "revision_semantics",
    "retry_semantics",
    "query_contract",
    "legacy_source_findings",
    "p1_activation_requirements",
    "non_claims",
}

EXPECTED_SOURCE_BINDINGS = {
    "legacy_update_parameter_dispatch": (
        "mycelix-governance/zomes/execution/coordinator/src/lib.rs",
        "3dbb8a8f69b377e494ccf24164c94bd80f54e0ef",
    ),
    "legacy_parameter_coordinator": (
        "mycelix-governance/zomes/constitution/coordinator/src/lib.rs",
        "923a1ce789c8319c79df7f33a9241af50804ec55",
    ),
    "legacy_parameter_integrity": (
        "mycelix-governance/zomes/constitution/integrity/src/lib.rs",
        "f83a457a8ff40b0003c07dba9da598a478c5e6f6",
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
    require(set(data) == EXPECTED_TOP_LEVEL, "top-level profile surface drift")
    require(data["schema"] == "mycelix.constitutional-parameter-contract.v1", "schema drift")
    require(data["contract_id"] == "MYC-CONST-003D1D-P0", "contract id drift")
    require(data["contract_revision"] == 1, "contract revision drift")
    require(
        data["authority_class"] == "InertContractPendingQualifiedAuthority",
        "authority ceiling drift",
    )
    require(data["activation_allowed"] is False, "P0 must remain non-activating")
    require(data["provider_key"] == "action_id", "provider key must be action_id")
    require(data["parameter_key"] == "parameter_name", "parameter key drift")

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
    require(len(source["files"]) == 4, "source binding census must contain exactly four files")
    roles = {entry["role"] for entry in source["files"]}
    require(roles == set(EXPECTED_SOURCE_BINDINGS), "source binding role census drift")
    for entry in source["files"]:
        expected_path, expected_blob = EXPECTED_SOURCE_BINDINGS[entry["role"]]
        require(entry["path"] == expected_path, f"source path drift for {entry['role']}")
        require(entry["git_blob_sha1"] == expected_blob, f"blob declaration drift for {entry['role']}")

    identity = data["identity"]
    required_request = {
        "schema_version",
        "operation_id",
        "action_id",
        "proposal_id",
        "claim_binding_commitment",
        "action_commitment",
        "publisher_did",
        "parameter_name",
        "typed_canonical_value",
        "expected_prior_revision",
    }
    require(set(identity["request_includes"]) == required_request, "request identity census drift")
    require("request_commitment" in identity["revision_includes"], "revision must bind request commitment")
    require(
        "prior_revision_commitment" in identity["revision_includes"],
        "revision must bind predecessor commitment",
    )
    require(
        "committed_at_unix_ms" in identity["excludes"],
        "wall clock must remain excluded from semantic identity",
    )

    mapping = data["effect_ledger_mapping"]
    require(mapping["operation_id"] == "ConstitutionalOperation.operation_id", "operation mapping drift")
    require(mapping["proposal_id"] == "ConstitutionalOperation.proposal_id", "proposal mapping drift")
    require(
        mapping["claim_binding_commitment"] == "ConstitutionalOperation.claim_binding",
        "ClaimBinding mapping drift",
    )
    require(mapping["action_id"] == "ActionIntent.action_id", "action mapping drift")
    require(
        mapping["action_commitment"] == "ActionIntent.action_commitment",
        "action commitment mapping drift",
    )
    require(mapping["provider_key_regenerated"] is False, "provider key regeneration forbidden")
    require(
        mapping["action_commitment_regenerated"] is False,
        "action commitment regeneration forbidden",
    )
    require(
        mapping["qualification_inherited_from_D1C"] is False,
        "P0 cannot inherit D1C qualification",
    )

    value = data["value_profile"]
    require(value["profile"] == "constitutional-parameter-value-v1", "value profile drift")
    require(
        set(value["types"])
        == {
            "integer-i64",
            "decimal-string-v1",
            "percentage-decimal-string-v1",
            "duration-millis-u64",
            "boolean",
            "utf8-string",
        },
        "value type census drift",
    )
    require(value["decimal_exponent_notation_allowed"] is False, "decimal exponents forbidden")
    require(value["decimal_negative_zero_normalized"] is True, "negative zero normalization required")
    require(
        value["decimal_trailing_fraction_zeros_removed"] is True,
        "decimal trailing-zero normalization required",
    )
    require(value["ieee754_constitutional_identity"] is False, "IEEE-754 identity promotion forbidden")
    require(value["value_commitment"] == "domain-separated-blake3-256", "value commitment drift")
    require(
        value["commitment_encoding"] == "blake3-256:<64-lowercase-hex>",
        "commitment encoding drift",
    )

    revision = data["revision_semantics"]
    require(revision["first_revision"] == 1, "revision chain must start at one")
    require(revision["first_write_precondition"] == "CreateOnly", "genesis precondition drift")
    require(
        revision["later_write_precondition"] == "ExactCurrentRevisionAndCommitment",
        "later revisions require exact CAS predecessor",
    )
    require(revision["revisions_contiguous"] is True, "revision chain must be contiguous")
    require(
        revision["predecessor_commitment_required_after_revision_one"] is True,
        "predecessor commitment required",
    )
    require(
        revision["stored_revision_reconstructs_request_commitment"] is True,
        "stored revision must reconstruct inner request commitment",
    )
    require(
        revision["wall_clock_participates_in_revision_identity"] is False,
        "wall clock cannot define parameter revision identity",
    )
    require(
        revision["current_projection_from_timestamp"] is False,
        "timestamp-defined current state forbidden",
    )

    retry = data["retry_semantics"]
    require(retry["historical_action_index_required"] is True, "historical action index required")
    require(
        retry["same_action_same_request"] == "ExistingSameAtOriginalRevision",
        "same action retry semantics drift",
    )
    require(
        retry["same_action_different_request"] == "IntegrityConflictAndHalt",
        "action identity collision must halt",
    )
    require(
        retry["different_action_stale_predecessor"] == "StaleRevisionNoMutation",
        "stale CAS must not mutate",
    )
    require(retry["last_write_wins_allowed"] is False, "last-write-wins forbidden")
    require(retry["blind_retry_after_stale_revision"] is False, "blind stale retry forbidden")
    require(
        retry["recovery_from_integrity_halt_defined"] is False,
        "P0 must not invent recovery authority",
    )

    query = data["query_contract"]
    require(query["by_parameter_revision"] is True, "revision lookup required")
    require(query["by_action_id"] is True, "action lookup required")
    require(
        query["current_is_highest_contiguous_revision"] is True,
        "current must derive from contiguous revision chain",
    )
    require(
        query["historical_action_lookup_survives_later_revisions"] is True,
        "delayed retry lookup requirement missing",
    )

    findings = data["legacy_source_findings"]
    require(findings["execution_request_shape"] == "{parameter,value}", "legacy request finding drift")
    require(findings["existing_parameter_change_requires_proposal_id"] is True, "proposal requirement hidden")
    require(findings["missing_parameter_can_enter_create_path_without_proposal_id"] is True, "legacy create gap hidden")
    require(findings["integrity_requires_value_valid_json"] is True, "legacy JSON requirement hidden")
    require(findings["execution_value_canonicalized_before_dispatch"] is False, "legacy canonicalization overstated")
    require(findings["parameter_index_selected_by_latest_link_timestamp"] is True, "legacy timestamp lookup hidden")
    require(findings["parameter_writer_authority_bound_at_integrity"] is False, "writer authority overstated")
    require(findings["proposal_authority_bound_at_parameter_integrity"] is False, "proposal authority overstated")
    require(findings["parameter_index_authority_bound"] is False, "index authority overstated")
    require(findings["phi_config_is_derived_best_effort_projection"] is True, "Phi projection boundary drift")

    gates = set(data["p1_activation_requirements"])
    for required in {
        "MYC-CONST-003B4 qualified ClaimBinding",
        "MYC-CONST-003CR1 qualified refinement crosswalk",
        "MYC-CONST-003D1C exact qualified effect-ledger identity",
        "MYC-CONST-003D1D-P0 exact-head qualification",
        "integrity-verifiable publisher-author binding",
        "integrity-verifiable proposal-and-ClaimBinding authorization",
        "stale predecessor rejection under direct DHT writes",
        "P1 exact-head adversarial qualification",
    }:
        require(required in gates, f"missing P1 activation gate: {required}")

    nonclaims = set(data["non_claims"])
    for required in {
        "not_a_holochain_entry_type",
        "not_a_live_parameter_write_extern",
        "not_wired_to_GovernanceAction_UpdateParameter",
        "claim_binding_commitment_is_opaque_in_P0",
        "not_parameter_authority_qualified",
        "not_legacy_parameter_migration_qualified",
        "not_phi_projection_delivery_qualified",
        "not_deployment_currentness_qualified",
        "not_P1_qualified",
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

    execution = materialized["legacy_update_parameter_dispatch"]
    require(
        'UpdateParameter {\n        parameter: String,\n        value: String,' in execution,
        "legacy execution UpdateParameter shape changed",
    )
    require(
        'let update_input = serde_json::json!({"parameter": parameter, "value": value});' in execution,
        "legacy execution no longer matches observed parameter/value-only dispatch",
    )

    coordinator = materialized["legacy_parameter_coordinator"]
    require("pub struct UpdateParameterInput" in coordinator, "UpdateParameterInput missing")
    require("pub proposal_id: Option<String>" in coordinator, "provider proposal_id field missing")
    require("if input.proposal_id.is_none()" in coordinator, "existing-parameter proposal gate changed")
    require("max_by_key(|l| l.timestamp)" in coordinator, "timestamp-defined legacy lookup changed")
    require("call_local_best_effort" in coordinator, "Phi best-effort projection finding changed")
    require("do NOT fail the parameter update" in coordinator, "Phi projection non-authority comment changed")

    integrity = materialized["legacy_parameter_integrity"]
    require("pub struct GovernanceParameter" in integrity, "GovernanceParameter missing")
    require("pub changed_by_proposal: Option<String>" in integrity, "proposal provenance field missing")
    require(
        "serde_json::from_str::<serde_json::Value>(&param.value).is_err()" in integrity,
        "legacy JSON-shape validation changed",
    )
    require(
        "LinkTypes::ParameterIndex => Ok(ValidateCallbackResult::Valid)" in integrity,
        "ParameterIndex validation finding changed",
    )
    require(
        "fn validate_create_parameter(\n    _action: Create," in integrity,
        "parameter create author-binding finding changed",
    )
    require(
        "fn validate_update_parameter(\n    _action: Update," in integrity,
        "parameter update author-binding finding changed",
    )

    d1c = materialized["d1c_identity_source"]
    for needle in [
        "pub struct ConstitutionalOperation",
        "pub operation_id: String",
        "pub proposal_id: String",
        "pub claim_binding: String",
        "pub struct ActionIntent",
        "pub action_id: String",
        "pub action_commitment: String",
        "pub fn validate_against_operation",
    ]:
        require(needle in d1c, f"D1C identity source missing {needle!r}")


def expect_profile_failure(data: dict[str, Any], description: str) -> None:
    try:
        validate_profile(data)
    except AssertionError:
        return
    fail(f"mutation survived: {description}")


def self_test(baseline: dict[str, Any]) -> None:
    mutations: list[tuple[str, callable]] = []

    def mutated(path: tuple[str, ...], value: Any) -> dict[str, Any]:
        candidate = copy.deepcopy(baseline)
        cursor: Any = candidate
        for key in path[:-1]:
            cursor = cursor[key]
        cursor[path[-1]] = value
        return candidate

    cases = [
        ("premature activation", mutated(("activation_allowed",), True)),
        (
            "regenerated provider identity",
            mutated(("effect_ledger_mapping", "provider_key_regenerated"), True),
        ),
        (
            "IEEE-754 promoted to constitutional identity",
            mutated(("value_profile", "ieee754_constitutional_identity"), True),
        ),
        (
            "exponent notation admitted",
            mutated(("value_profile", "decimal_exponent_notation_allowed"), True),
        ),
        (
            "timestamp defines current",
            mutated(("revision_semantics", "current_projection_from_timestamp"), True),
        ),
        (
            "inner request reconstruction removed",
            mutated(("revision_semantics", "stored_revision_reconstructs_request_commitment"), False),
        ),
        (
            "historical action index removed",
            mutated(("retry_semantics", "historical_action_index_required"), False),
        ),
        (
            "last-write-wins enabled",
            mutated(("retry_semantics", "last_write_wins_allowed"), True),
        ),
        (
            "stale blind retry enabled",
            mutated(("retry_semantics", "blind_retry_after_stale_revision"), True),
        ),
        (
            "legacy writer authority inflated",
            mutated(("legacy_source_findings", "parameter_writer_authority_bound_at_integrity"), True),
        ),
    ]
    for description, candidate in cases:
        expect_profile_failure(candidate, description)

    candidate = copy.deepcopy(baseline)
    candidate["identity"]["request_includes"].remove("proposal_id")
    expect_profile_failure(candidate, "proposal removed from request identity")

    candidate = copy.deepcopy(baseline)
    candidate["source_binding"]["files"][0]["git_blob_sha1"] = "0" * 40
    expect_profile_failure(candidate, "declared source blob drift")

    print(f"parameter-contract validator self-tests: PASS ({len(cases) + 2} mutants killed)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--no-source-check", action="store_true")
    args = parser.parse_args()

    data = load_profile()
    validate_profile(data)
    if not args.no_source_check:
        verify_source_bindings(data)
    if args.self_test:
        self_test(data)

    print("constitutional parameter P0 contract: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        print(f"constitutional parameter P0 contract: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
