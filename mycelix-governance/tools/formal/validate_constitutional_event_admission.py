#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-E1A constitutional event admission crosswalk.

This validator is deliberately cross-lineage and fail-closed. It verifies exact
Git object identities for the frozen B4, D1C, E0, and CR1 inputs, independently
inspects their source-visible contracts, and refuses activation while required
qualification evidence remains pending.

It is not a Holochain runtime qualifier and does not turn pending relationships
into qualified authority.
"""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
DEFAULT_PROFILE = (
    REPO
    / "mycelix-governance"
    / "specs"
    / "constitutional-event-admission-crosswalk.v1.json"
)
DEFAULT_SCHEMA = (
    REPO
    / "mycelix-governance"
    / "specs"
    / "constitutional-event-admission-crosswalk.v1.schema.json"
)

EXPECTED_BINDINGS = {
    "b4-claim-binding": (
        "037f61c15ff367a1518d98f7acc8ec0fa962c2f6",
        "mycelix-governance/crates/constitutional-consumption/src/binding.rs",
        "f61e327cac047824b5d561b2d865e863389dcaa6",
    ),
    "d1c-effect-ledger": (
        "47d1d764323dbfaf991b5574cfde83abb7a3e4a4",
        "mycelix-governance/crates/constitutional-effect-ledger/src/lib.rs",
        "bb3b8d6a865bbf514ff1e43a41c520b890a5515d",
    ),
    "e0-event-provider": (
        "36a4fcffb6ca806570ebf439f9c36cf76401e7b2",
        "mycelix-governance/crates/constitutional-event-provider/src/lib.rs",
        "5d110f95c0d78f494b61a16d5afe5776401820bb",
    ),
    "e0-d1c-identity-test": (
        "36a4fcffb6ca806570ebf439f9c36cf76401e7b2",
        "mycelix-governance/crates/constitutional-event-provider/tests/effect_ledger_identity.rs",
        "897668782e09fac5ffbc506e1e8b11f653de2950",
    ),
    "cr1-crosswalk-scaffold": (
        "4edb56bd3c36ddc6277e2de5e0584a65e8c99b3c",
        "mycelix-governance/specs/constitutional-refinement-crosswalk.v1.json",
        "35476cb5b2dbacaca659b117e6dfaaacebed818d",
    ),
}

EXPECTED_DEPENDENCIES = {
    "MYC-CONST-003B4",
    "MYC-CONST-003CR1",
    "MYC-CONST-003D1C",
    "MYC-CONST-003D1D-E0",
}

EXPECTED_B4_FIELDS = [
    "claim_id",
    "envelope_digest",
    "nonce",
    "use_index",
    "jurisdiction",
    "matter.namespace",
    "matter.stable_id",
    "target_digest",
    "payload_digest",
    "budget_id",
]

EXPECTED_TARGET_FIELDS = [
    "provider_lane",
    "operation_id",
    "action_id",
    "proposal_id",
    "event_name",
]

PENDING_STATES = {"queued", "pending", "failed", "cancelled"}


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def run_git(*args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(REPO), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValidationError(
            f"git {' '.join(args)} failed: {result.stderr.strip() or result.stdout.strip()}"
        )
    return result.stdout.strip()


def git_blob(commit: str, path: str) -> str:
    return run_git("rev-parse", f"{commit}:{path}")


def git_show(commit: str, path: str) -> str:
    return run_git("show", f"{commit}:{path}")


def require_tokens(label: str, text: str, tokens: list[str]) -> None:
    missing = [token for token in tokens if token not in text]
    require(not missing, f"{label} missing required source tokens: {missing}")


def validate_schema_ceiling(schema: dict[str, Any]) -> None:
    require(schema.get("type") == "object", "schema root must be an object")
    require(
        schema.get("additionalProperties") is False,
        "schema root must reject unknown properties",
    )
    properties = schema.get("properties", {})
    require(
        properties.get("crosswalk_id", {}).get("const") == "MYC-CONST-003D1D-E1A",
        "schema must pin E1A crosswalk_id",
    )
    require(
        properties.get("provider_lane", {}).get("const") == "EmitEvent",
        "schema must pin EmitEvent provider lane",
    )


def validate_source_bindings(data: dict[str, Any], check_git: bool) -> None:
    bindings = data.get("source_bindings")
    require(isinstance(bindings, list), "source_bindings must be a list")
    by_id = {item.get("id"): item for item in bindings if isinstance(item, dict)}
    require(
        set(by_id) == set(EXPECTED_BINDINGS),
        "source binding census must exactly match frozen E1A inputs",
    )

    for binding_id, (commit, path, blob_sha) in EXPECTED_BINDINGS.items():
        item = by_id[binding_id]
        require(item.get("commit") == commit, f"{binding_id} commit drift")
        require(item.get("path") == path, f"{binding_id} path drift")
        require(
            item.get("git_blob_sha1") == blob_sha,
            f"{binding_id} declared blob drift",
        )
        if check_git:
            actual = git_blob(commit, path)
            require(actual == blob_sha, f"{binding_id} actual Git blob drift: {actual}")


def validate_dependencies(data: dict[str, Any]) -> None:
    deps = data.get("qualification_dependencies")
    require(isinstance(deps, list), "qualification_dependencies must be a list")
    by_id = {item.get("id"): item for item in deps if isinstance(item, dict)}
    require(
        set(by_id) == EXPECTED_DEPENDENCIES,
        "qualification dependency census must be exact",
    )

    for dep_id, item in by_id.items():
        state = item.get("observed_state")
        receipt = item.get("qualification_receipt")
        require(
            state in PENDING_STATES | {"qualified"},
            f"{dep_id} has unsupported observed_state {state!r}",
        )
        if state == "qualified":
            require(
                isinstance(receipt, str) and bool(receipt.strip()),
                f"{dep_id} cannot be qualified without a retained receipt",
            )
        else:
            require(
                receipt is None,
                f"{dep_id} pending/non-green state must not carry a qualification receipt",
            )

    activation_allowed = data.get("activation_allowed")
    if activation_allowed:
        require(
            all(item.get("observed_state") == "qualified" for item in deps),
            "activation cannot be allowed while any dependency is not qualified",
        )
        require(
            data.get("status") == "QualifiedCrosswalk",
            "activation requires QualifiedCrosswalk status",
        )
        require(
            data.get("activation_status") == "EligibleForE1Activation",
            "activation requires EligibleForE1Activation status",
        )
    else:
        require(
            data.get("status") == "PendingQualifiedCrosswalk",
            "blocked E1A profile must remain PendingQualifiedCrosswalk",
        )
        require(
            data.get("activation_status") == "Blocked",
            "blocked E1A profile must have activation_status=Blocked",
        )


def validate_identity_mapping(data: dict[str, Any]) -> None:
    identity = data.get("identity_mapping")
    require(isinstance(identity, dict), "identity_mapping must be an object")
    require(identity.get("status") == "LocallySourceBound", "identity mapping status drift")
    for key in (
        "provider_key_regeneration_allowed",
        "operation_id_regeneration_allowed",
        "action_commitment_regeneration_allowed",
        "qualification_inheritance_allowed",
    ):
        require(identity.get(key) is False, f"{key} must remain false")

    expected_relations = {
        "operation_id": "E0.EventAuthorityBinding.operation_id == D1C.ConstitutionalOperation.operation_id",
        "proposal_id": "E0.EventAuthorityBinding.proposal_id == D1C.ConstitutionalOperation.proposal_id",
        "action_id": "E0.EventAuthorityBinding.action_id == D1C.ActionIntent.action_id",
        "action_commitment": "E0.EventAuthorityBinding.action_commitment == D1C.ActionIntent.action_commitment",
        "claim_binding_reference": "E0.EventAuthorityBinding.claim_binding_commitment == D1C.ConstitutionalOperation.claim_binding",
        "provider_key": "E0.DurableConstitutionalEvent.provider_key == D1C.ActionIntent.action_id",
    }
    for key, value in expected_relations.items():
        require(identity.get(key) == value, f"identity mapping {key} drift")


def validate_claim_binding_requirements(data: dict[str, Any]) -> None:
    req = data.get("claim_binding_requirements")
    require(isinstance(req, dict), "claim_binding_requirements must be an object")
    require(
        req.get("all_b4_security_fields_required") == EXPECTED_B4_FIELDS,
        "all B4 security-relevant ClaimBinding fields must remain in the admission census",
    )

    target = req.get("target_relationship", {})
    require(
        target.get("status") == "PendingQualifiedCrosswalk",
        "target refinement must remain pending until qualified crosswalk evidence exists",
    )
    require(
        target.get("required_relation")
        == "ClaimBinding.target_digest == EventTargetDescriptorV1.commitment",
        "target refinement relation drift",
    )
    descriptor = target.get("descriptor", {})
    require(
        descriptor.get("domain_separator")
        == "MYCELIX-CONSTITUTIONAL-EVENT-TARGET\\0V1\\0",
        "event target descriptor domain separator drift",
    )
    require(
        descriptor.get("encoding") == "u64-length-prefixed-utf8-fields",
        "event target descriptor encoding drift",
    )
    require(
        descriptor.get("digest") == "blake3-256:<64-lowercase-hex>",
        "event target descriptor digest encoding drift",
    )
    require(
        descriptor.get("ordered_fields") == EXPECTED_TARGET_FIELDS,
        "event target descriptor field order/census drift",
    )
    require(
        descriptor.get("provider_lane_constant") == "EmitEvent",
        "event target descriptor must pin EmitEvent lane",
    )

    payload = req.get("payload_relationship", {})
    require(
        payload.get("status") == "PendingQualifiedCrosswalk",
        "payload refinement must remain pending until qualified crosswalk evidence exists",
    )
    require(
        payload.get("required_relation")
        == "ClaimBinding.payload_digest == DurableConstitutionalEvent.payload_commitment",
        "payload refinement relation drift",
    )
    require(payload.get("payload_source") == "E0 canonical_payload", "payload source drift")
    require(
        payload.get("payload_commitment_encoding")
        == "blake3-256:<64-lowercase-hex>",
        "payload commitment encoding drift",
    )

    claim_ref = req.get("claim_binding_commitment_relationship", {})
    require(
        claim_ref.get("status") == "PendingQualifiedCrosswalk",
        "ClaimBinding commitment relationship must remain pending",
    )
    require(
        "exact B4 ClaimBinding canonical bytes" in claim_ref.get("required_relation", ""),
        "ClaimBinding reference must identify exact B4 canonical bytes",
    )
    require(
        "B4 itself deliberately leaves hash/signature algorithm selection outside the crate"
        in claim_ref.get("algorithm_selection", ""),
        "profile must not attribute a digest algorithm choice to B4",
    )


def validate_integrity_requirements(data: dict[str, Any]) -> None:
    integrity = data.get("integrity_boundary_requirements")
    require(isinstance(integrity, dict), "integrity_boundary_requirements must be an object")
    require(
        integrity.get("publisher_must_equal_dht_action_author_did") is True,
        "publisher identity must bind to DHT action author",
    )
    require(
        integrity.get("coordinator_only_authorization_is_sufficient") is False,
        "coordinator-only authorization must never be sufficient",
    )
    require(integrity.get("event_entry_must_be_immutable") is True, "event entry must be immutable")
    require(
        integrity.get("action_key_link_must_be_integrity_validated") is True,
        "action-key link must be integrity validated",
    )
    require(
        integrity.get("action_key_link_delete_allowed") is False,
        "constitutional action-key link deletion must remain forbidden",
    )
    require(
        integrity.get("same_action_same_semantics") == "ExistingSame",
        "same-action idempotent outcome drift",
    )
    require(
        integrity.get("same_action_conflicting_semantics") == "IntegrityConflict",
        "same-action conflict outcome drift",
    )
    require(
        integrity.get("conflicting_same_action_last_write_wins_allowed") is False,
        "last-write-wins conflict resolution must remain forbidden",
    )
    require(
        integrity.get("signal_projection_can_define_constitutional_completion") is False,
        "signal projection cannot define constitutional completion",
    )


def validate_non_claims(data: dict[str, Any]) -> None:
    claims = set(data.get("non_claims", []))
    required = {
        "not_a_holochain_entry_type",
        "not_a_holochain_link_type",
        "not_a_live_write_extern",
        "not_wired_to_GovernanceAction_EmitEvent",
        "not_B4_qualified",
        "not_CR1_qualified",
        "not_D1C_qualified",
        "not_E0_qualified",
        "not_target_payload_refinement_qualified",
        "not_constitutional_event_admission_active",
        "not_deployment_currentness_qualified",
    }
    require(required <= claims, "E1A non-claim ceiling was weakened")


def validate_source_semantics(data: dict[str, Any]) -> None:
    by_id = {item["id"]: item for item in data["source_bindings"]}

    b4 = git_show(by_id["b4-claim-binding"]["commit"], by_id["b4-claim-binding"]["path"])
    require_tokens(
        "B4 ClaimBinding",
        b4,
        [
            "pub struct ClaimBinding",
            "pub claim_id: String",
            "pub envelope_digest: String",
            "pub nonce: String",
            "pub use_index: u32",
            "pub jurisdiction: String",
            "pub matter: MatterId",
            "pub target_digest: String",
            "pub payload_digest: String",
            "pub budget_id: String",
            "pub fn canonical_bytes",
            "CLAIM_BINDING_DOMAIN_SEPARATOR",
            "push_str(&mut out, &self.target_digest)?",
            "push_str(&mut out, &self.payload_digest)?",
        ],
    )

    d1c = git_show(by_id["d1c-effect-ledger"]["commit"], by_id["d1c-effect-ledger"]["path"])
    require_tokens(
        "D1C effect ledger",
        d1c,
        [
            "pub struct ConstitutionalOperation",
            "pub operation_id: String",
            "pub proposal_id: String",
            "pub claim_binding: String",
            "pub struct ActionIntent",
            "pub action_id: String",
            "pub action_commitment: String",
            "pub fn validate_against_operation",
        ],
    )

    e0 = git_show(by_id["e0-event-provider"]["commit"], by_id["e0-event-provider"]["path"])
    require_tokens(
        "E0 durable event provider",
        e0,
        [
            "pub struct EventAuthorityBinding",
            "pub operation_id: String",
            "pub action_id: String",
            "pub proposal_id: String",
            "pub claim_binding_commitment: String",
            "pub action_commitment: String",
            "pub publisher_did: String",
            "pub fn provider_key(&self) -> &str",
            "&self.authority.action_id",
            "pub payload_commitment: String",
            "pub event_commitment: String",
            "pub fn constitutional_event_complete",
            "Ok(true)",
        ],
    )

    identity_test = git_show(
        by_id["e0-d1c-identity-test"]["commit"],
        by_id["e0-d1c-identity-test"]["path"],
    )
    require_tokens(
        "E0/D1C identity bridge tests",
        identity_test,
        [
            "operation_id: operation.operation_id.clone()",
            "action_id: intent.action_id.clone()",
            "proposal_id: operation.proposal_id.clone()",
            "claim_binding_commitment: operation.claim_binding.clone()",
            "action_commitment: intent.action_commitment.clone()",
            "assert_eq!(authority.action_id, intent.action_id)",
            "assert_eq!(authority.action_commitment, intent.action_commitment)",
        ],
    )

    cr1_text = git_show(
        by_id["cr1-crosswalk-scaffold"]["commit"],
        by_id["cr1-crosswalk-scaffold"]["path"],
    )
    cr1 = json.loads(cr1_text)
    require(cr1.get("status") == "draft", "CR1 scaffold must remain draft in bound source")
    pending = {item.get("id"): item for item in cr1.get("pending_extensions", [])}
    require("claim-binding-v1" in pending, "CR1 scaffold must retain ClaimBinding pending extension")
    require(
        pending["claim-binding-v1"].get("status") == "pending_qualification",
        "CR1 bound scaffold unexpectedly promotes ClaimBinding",
    )
    require(
        pending["claim-binding-v1"].get("candidate_semantic_head")
        == "037f61c15ff367a1518d98f7acc8ec0fa962c2f6",
        "CR1 ClaimBinding candidate semantic head drift",
    )


def validate_profile(data: dict[str, Any], *, check_git: bool) -> None:
    require(
        data.get("schema") == "mycelix.constitutional-event-admission-crosswalk.v1",
        "profile schema id drift",
    )
    require(data.get("crosswalk_id") == "MYC-CONST-003D1D-E1A", "crosswalk id drift")
    require(data.get("provider_lane") == "EmitEvent", "provider lane drift")
    require(data.get("activation_allowed") is False, "E1A activation must remain blocked")

    validate_source_bindings(data, check_git)
    validate_dependencies(data)
    validate_identity_mapping(data)
    validate_claim_binding_requirements(data)
    validate_integrity_requirements(data)
    validate_non_claims(data)

    gates = data.get("activation_gates")
    require(isinstance(gates, list) and len(gates) >= 8, "activation gate census unexpectedly weak")

    if check_git:
        validate_source_semantics(data)


def expect_rejected(label: str, data: dict[str, Any]) -> None:
    try:
        validate_profile(data, check_git=False)
    except ValidationError:
        return
    raise ValidationError(f"self-test mutant survived: {label}")


def run_self_tests(canonical: dict[str, Any]) -> None:
    mutant = copy.deepcopy(canonical)
    mutant["activation_allowed"] = True
    mutant["activation_status"] = "EligibleForE1Activation"
    mutant["status"] = "QualifiedCrosswalk"
    expect_rejected("premature activation", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["claim_binding_requirements"]["target_relationship"]["descriptor"]["ordered_fields"].remove("action_id")
    expect_rejected("target descriptor omits action_id", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["claim_binding_requirements"]["payload_relationship"]["required_relation"] = (
        "ClaimBinding.payload_digest == event_name"
    )
    expect_rejected("payload refinement weakened", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["integrity_boundary_requirements"]["publisher_must_equal_dht_action_author_did"] = False
    mutant["integrity_boundary_requirements"]["coordinator_only_authorization_is_sufficient"] = True
    expect_rejected("author binding weakened to coordinator", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["source_bindings"][0]["git_blob_sha1"] = "0" * 40
    expect_rejected("source blob drift", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["identity_mapping"]["provider_key_regeneration_allowed"] = True
    expect_rejected("provider identity regeneration", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["integrity_boundary_requirements"]["conflicting_same_action_last_write_wins_allowed"] = True
    expect_rejected("last-write-wins conflict", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["qualification_dependencies"][0]["observed_state"] = "qualified"
    expect_rejected("qualification without receipt", mutant)

    mutant = copy.deepcopy(canonical)
    mutant["claim_binding_requirements"]["all_b4_security_fields_required"].remove("budget_id")
    expect_rejected("B4 field census omission", mutant)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--no-git",
        action="store_true",
        help="skip exact Git object/source inspection (intended only for validator self-development)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data = json.loads(args.profile.read_text())
    schema = json.loads(args.schema.read_text())

    validate_schema_ceiling(schema)
    validate_profile(data, check_git=not args.no_git)
    if args.self_test:
        run_self_tests(data)

    print("MYC-CONST-003D1D-E1A admission crosswalk: PASS")
    if args.self_test:
        print("mutation-sensitive self-tests: PASS")
    print("activation: BLOCKED (qualification/refinement dependencies remain external)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
