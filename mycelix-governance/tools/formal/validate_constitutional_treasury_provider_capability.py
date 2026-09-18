#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-F0C provider replay capability gating.

This validator is deliberately standard-library-only. It validates both the
machine-readable contract and the Rust public surface, and re-resolves the exact
F0 parent Git objects rather than trusting copied SHAs in the profile.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROFILE_PATH = ROOT / "mycelix-governance/specs/constitutional-treasury-provider-capability.v1.json"
FACADE_PATH = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider/src/capability_public.rs"
MANIFEST_PATH = ROOT / "mycelix-governance/crates/constitutional-treasury-effect-provider/Cargo.toml"

F0_HEAD = "5e5c6097f8de8f0dc4b66695b87f6229de551bae"
F0_PUBLIC_PATH = "mycelix-governance/crates/constitutional-treasury-effect-provider/src/public.rs"
F0_PUBLIC_BLOB = "718d689be933103ee55c3c3d8ad0b3641cf971d4"
F0_MANIFEST_PATH = "mycelix-governance/crates/constitutional-treasury-effect-provider/Cargo.toml"
F0_MANIFEST_BLOB = "48b2b522e92459abe4f84fc31875720183e79b9e"

EXPECTED_BINDINGS = {
    "provider_profile_id",
    "provider_profile_commitment",
    "qualification_receipt_id",
    "qualification_receipt_commitment",
    "execution_id",
    "request_commitment",
    "covers_through_attempt_ordinal",
}
EXPECTED_OUTCOMES = {"KnownSuccess", "KnownNoEffect", "UnknownOutcome"}
EXPECTED_RECEIPT_BINDINGS = {
    "provider_operation_key",
    "execution_id",
    "request_commitment",
    "provider_outcome",
    "provider_receipt_identity",
    "provider_receipt_commitment",
}
EXPECTED_NONCLAIMS = {
    "not_provider_qualified",
    "not_provider_truth_established",
    "not_external_settlement_finality",
    "not_physical_exactly_once",
    "not_public_fund_authority",
    "not_sap_amount_qualified",
    "not_treasury_v2_qualified",
    "not_capacity_qualified",
    "not_live_treasury_adapter",
    "not_deployment_current",
}


def fail(message: str) -> None:
    raise ValueError(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def validate_git_bindings(profile: dict) -> None:
    parent = profile["parent"]
    if parent["f0_semantic_head"] != F0_HEAD:
        fail("F0 semantic head drift")
    public_blob = git("rev-parse", f"{F0_HEAD}:{F0_PUBLIC_PATH}")
    manifest_blob = git("rev-parse", f"{F0_HEAD}:{F0_MANIFEST_PATH}")
    if public_blob != F0_PUBLIC_BLOB or public_blob != parent["f0_public_facade_blob"]:
        fail("F0 public facade blob drift")
    if manifest_blob != F0_MANIFEST_BLOB or manifest_blob != parent["f0_manifest_blob"]:
        fail("F0 manifest blob drift")


def validate_profile(profile: dict) -> None:
    expected_top = {
        "schema",
        "profile_id",
        "revision",
        "authority_class",
        "parent",
        "public_api",
        "capability_token",
        "retry_gate",
        "required_provider_qualification",
        "qualification_state",
        "activation_allowed",
        "minting_path",
        "non_claims",
    }
    if set(profile) != expected_top:
        fail("closed top-level profile key set drift")
    if profile["schema"] != "mycelix.constitutional-treasury-provider-capability.v1":
        fail("schema drift")
    if profile["profile_id"] != "mycelix-treasury-provider-replay-capability-v1":
        fail("profile id drift")
    if profile["revision"] != 1:
        fail("revision drift")
    if profile["authority_class"] != "TypeGatedPendingProviderQualification":
        fail("authority class inflated")

    public = profile["public_api"]
    if public != {
        "crate_entrypoint": "src/capability_public.rs",
        "legacy_f0_facade_private": True,
        "raw_kernel_private": True,
        "caller_string_replay_removed": True,
        "public_mint_path": False,
        "portable_positive_token": False,
    }:
        fail("public API boundary weakened")

    token = profile["capability_token"]
    if token["type"] != "QualifiedProviderReplayCapability":
        fail("token type drift")
    for key in ("serializable", "deserializable", "cloneable", "copyable", "public_constructor"):
        if token[key] is not False:
            fail(f"token portability/construction inflated: {key}")
    if set(token["bindings"]) != EXPECTED_BINDINGS:
        fail("capability binding census drift")
    if token["valid_only_when_state"] != "UnknownOutcome":
        fail("capability state scope widened")
    if token["horizon_must_equal_current_unknown_outcome"] is not True:
        fail("capability horizon binding weakened")
    if token["provider_profile_must_equal_effect_adapter_profile"] is not True:
        fail("provider profile binding weakened")
    if token["consumed_by_retry_decision"] is not True:
        fail("capability consumption semantics weakened")

    retry = profile["retry_gate"]
    if retry["unknown_outcome_automatic_replay_requires"] != "QualifiedProviderReplayCapability":
        fail("automatic replay gate weakened")
    if retry["known_no_effect_retry_requires"] != "ExactReconciliationObservation":
        fail("no-effect reconciliation binding weakened")
    if retry["transport_failure_is_no_effect"] is not False:
        fail("transport failure promoted to no-effect")
    if retry["fresh_execution_identity_on_retry"] is not False:
        fail("fresh execution identity allowed on retry")
    if retry["no_effect_reconciliation_remains_available_without_replay_capability"] is not True:
        fail("authoritative no-effect path accidentally removed")

    required = profile["required_provider_qualification"]
    opkey = required["operation_key"]
    if not opkey["deterministic"] or not opkey["bound_to_execution_id"]:
        fail("provider operation key not bound deterministically")
    if not opkey["random_retry_nonce_forbidden"]:
        fail("random retry nonce admitted into operation key")
    if opkey["changed_request_same_key"] != "Conflict":
        fail("changed request under same provider key not fail-closed")

    duplicate = required["duplicate_semantics"]
    if duplicate["same_key_same_request"] != "SameEffectOrSameAuthoritativeReceipt":
        fail("same-key same-request semantics weakened")
    if duplicate["same_key_different_request"] != "Conflict":
        fail("same-key different-request conflict weakened")
    if duplicate["concurrent_same_key"] != "OneSemanticEffectOrExplicitConflict":
        fail("concurrent duplicate semantics weakened")

    query = required["authoritative_query"]
    if query["required"] is not True:
        fail("authoritative provider query made optional")
    if query["query_key"] != "ProviderOperationKeyBoundToExecutionId":
        fail("provider query key drift")
    if set(query["outcomes"]) != EXPECTED_OUTCOMES:
        fail("provider query outcome census drift")
    if query["transport_error_may_establish_known_no_effect"] is not False:
        fail("transport failure allowed to establish KnownNoEffect")

    receipt = required["receipt_schema"]
    if receipt["machine_readable"] is not True:
        fail("provider receipt no longer machine-readable")
    if set(receipt["must_bind"]) != EXPECTED_RECEIPT_BINDINGS:
        fail("provider receipt binding census drift")

    replay = required["replay_policy"]
    if replay["automatic_replay_from_unknown_requires_provider_idempotency_theorem"] is not True:
        fail("provider idempotency theorem no longer required")
    if replay["authoritative_known_no_effect_allows_same_execution_retry"] is not True:
        fail("no-effect same-execution retry semantics drift")
    if replay["conflicting_provider_evidence_halts"] is not True:
        fail("conflicting provider evidence no longer halts")

    if profile["qualification_state"] != "Pending":
        fail("provider qualification state inflated")
    if profile["activation_allowed"] is not False:
        fail("provider activation prematurely allowed")
    if profile["minting_path"] != "AbsentUntilExactProviderQualification":
        fail("capability mint path prematurely present")
    if set(profile["non_claims"]) != EXPECTED_NONCLAIMS:
        fail("non-claim ceiling drift")


def validate_source(facade: str, manifest: str) -> None:
    if 'path = "src/capability_public.rs"' not in manifest:
        fail("crate entrypoint is not capability_public.rs")
    if "pub mod f0 {" in facade or "pub(crate) mod f0 {" in facade:
        fail("legacy F0 facade became externally visible")
    if 'mod f0 {\n    include!("public.rs");\n}' not in facade:
        fail("private F0 facade inclusion missing")

    struct_marker = "pub struct QualifiedProviderReplayCapability {"
    struct_pos = facade.find(struct_marker)
    if struct_pos < 0:
        fail("qualified provider replay capability type missing")
    derive_window = facade[max(0, struct_pos - 200):struct_pos]
    if "#[derive(Debug, PartialEq, Eq)]" not in derive_window:
        fail("capability derive surface drift")
    for forbidden in ("Serialize", "Deserialize", "Clone", "Copy"):
        if forbidden in derive_window:
            fail(f"capability became portable/duplicable via derive: {forbidden}")

    impl_pos = facade.find("impl QualifiedProviderReplayCapability {", struct_pos)
    retry_doc = facade.find("/// Public retry basis.", impl_pos)
    if impl_pos < 0 or retry_doc < 0:
        fail("capability impl boundary missing")
    impl_text = facade[impl_pos:retry_doc]
    for forbidden_ctor in ("pub fn new(", "pub fn from_", "pub fn mint_", "pub const fn new("):
        if forbidden_ctor in impl_text:
            fail("public capability constructor introduced")
    if "#[cfg(test)]\n    fn mint_for_test(" not in impl_text:
        fail("test-only capability fixture seam missing")

    required_fields = [
        "provider_profile_id: String",
        "provider_profile_commitment: String",
        "qualification_receipt_id: String",
        "qualification_receipt_commitment: String",
        "execution_id: String",
        "request_commitment: String",
        "covers_through_attempt_ordinal: u32",
        "_sealed: ()",
    ]
    struct_end = facade.find("}\n\nimpl QualifiedProviderReplayCapability", struct_pos)
    struct_text = facade[struct_pos:struct_end]
    for field in required_fields:
        if field not in struct_text:
            fail(f"capability binding field missing: {field}")

    if "ProviderReplayQualified(QualifiedProviderReplayCapability)" not in facade:
        fail("public retry basis is not capability-typed")
    public_retry_start = facade.find("pub enum RetryBasis")
    public_retry_end = facade.find("impl RetryBasis", public_retry_start)
    if "capability_evidence_id: String" in facade[public_retry_start:public_retry_end]:
        fail("caller string replay assertion reintroduced")
    if "capability.validate_for(record)?;" not in facade:
        fail("capability is not validated against current effect")
    if "self.provider_profile_id != record.intent().request.subject.adapter_profile_id" not in facade:
        fail("provider profile is not bound to effect adapter profile")
    if "self.execution_id != record.intent().execution_id" not in facade:
        fail("capability execution binding missing")
    if "self.request_commitment != record.intent().request.request_commitment" not in facade:
        fail("capability request binding missing")
    if "*through_attempt_ordinal == self.covers_through_attempt_ordinal" not in facade:
        fail("capability UnknownOutcome horizon binding missing")


def expect_rejected(name: str, fn) -> None:
    try:
        fn()
    except (ValueError, KeyError, TypeError):
        return
    raise AssertionError(f"mutation survived: {name}")


def self_test(profile: dict, facade: str, manifest: str) -> None:
    mutations = []

    def profile_mutation(name, mutator):
        def run():
            candidate = copy.deepcopy(profile)
            mutator(candidate)
            validate_profile(candidate)
        mutations.append((name, run))

    profile_mutation("premature-activation", lambda p: p.__setitem__("activation_allowed", True))
    profile_mutation("fake-qualified-state", lambda p: p.__setitem__("qualification_state", "Qualified"))
    profile_mutation("public-mint-path", lambda p: p["public_api"].__setitem__("public_mint_path", True))
    profile_mutation("serializable-token", lambda p: p["capability_token"].__setitem__("serializable", True))
    profile_mutation("drop-execution-binding", lambda p: p["capability_token"].__setitem__("bindings", [x for x in p["capability_token"]["bindings"] if x != "execution_id"]))
    profile_mutation("allow-transport-no-effect", lambda p: p["required_provider_qualification"]["authoritative_query"].__setitem__("transport_error_may_establish_known_no_effect", True))
    profile_mutation("remove-authoritative-query", lambda p: p["required_provider_qualification"]["authoritative_query"].__setitem__("required", False))
    profile_mutation("duplicate-request-conflict-weakened", lambda p: p["required_provider_qualification"]["duplicate_semantics"].__setitem__("same_key_different_request", "Accept"))
    profile_mutation("receipt-request-binding-removed", lambda p: p["required_provider_qualification"]["receipt_schema"].__setitem__("must_bind", [x for x in p["required_provider_qualification"]["receipt_schema"]["must_bind"] if x != "request_commitment"]))

    source_mutations = [
        ("public-f0-module", facade.replace("mod f0 {", "pub mod f0 {", 1)),
        ("serializable-capability", facade.replace("#[derive(Debug, PartialEq, Eq)]\npub struct QualifiedProviderReplayCapability", "#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]\npub struct QualifiedProviderReplayCapability", 1)),
        ("public-constructor", facade.replace("impl QualifiedProviderReplayCapability {", "impl QualifiedProviderReplayCapability {\n    pub fn new() -> Self { panic!(\"forbidden\") }", 1)),
        ("remove-horizon-binding", facade.replace("*through_attempt_ordinal == self.covers_through_attempt_ordinal", "true", 1)),
        ("string-replay-variant", facade.replace("ProviderReplayQualified(QualifiedProviderReplayCapability),", "ProviderReplayQualified { capability_evidence_id: String },", 1)),
    ]

    for name, run in mutations:
        expect_rejected(name, run)
    for name, mutated in source_mutations:
        expect_rejected(name, lambda mutated=mutated: validate_source(mutated, manifest))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    profile = json.loads(PROFILE_PATH.read_text())
    facade = FACADE_PATH.read_text()
    manifest = MANIFEST_PATH.read_text()

    validate_profile(profile)
    validate_source(facade, manifest)
    validate_git_bindings(profile)
    if args.self_test:
        self_test(profile, facade, manifest)

    canonical = json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()
    result = {
        "validated": True,
        "self_test": args.self_test,
        "profile_id": profile["profile_id"],
        "authority_class": profile["authority_class"],
        "qualification_state": profile["qualification_state"],
        "activation_allowed": profile["activation_allowed"],
        "public_mint_path": profile["public_api"]["public_mint_path"],
        "canonical_profile_sha256": hashlib.sha256(canonical).hexdigest(),
    }
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
