#!/usr/bin/env python3
"""Source-bound validator for MYC-CONST-003D1D-F0P0.

Classifies the exact payments source as a candidate provider for F0C without
promoting it to replay-qualified capability authority.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROFILE_PATH = ROOT / "mycelix-governance/specs/constitutional-payments-provider-observed.v1.json"

RUNTIME = "15b9c89adf0ac3c6c5a73681614d6bfcd368820a"
F0C = "36bb486794f3e2720a2e02b3045c270b147cbacc"
BINDINGS = {
    "payments_coordinator": (
        "mycelix-finance/zomes/payments/coordinator/src/lib.rs",
        "7df81b5374526e4c145a4b7506e7295aa20d079c",
    ),
    "payments_integrity": (
        "mycelix-finance/zomes/payments/integrity/src/lib.rs",
        "b551f1ea439f6899d67c0ba55519b12cbee1fe7e",
    ),
    "governance_finance_bridge": (
        "mycelix-governance/zomes/bridge/coordinator/src/cross_cluster.rs",
        "3eb0ade8633d6e711fd266bca6eb2ebab616eac2",
    ),
}

BLOCKERS = {
    "provider_operation_key_not_bound_to_execution_id",
    "retry_mints_new_wall_clock_payment_identity",
    "duplicate_same_key_semantics_not_established",
    "duplicate_conflict_semantics_not_established",
    "authoritative_known_no_effect_query_absent",
    "provider_unknown_outcome_query_absent",
    "receipt_not_bound_to_execution_id",
    "receipt_not_bound_to_request_commitment",
    "receipt_signature_not_verified_by_integrity",
    "qualified_sap_value_subject_not_consumed",
    "payments_not_current_governance_transfer_route",
}

NONCLAIMS = {
    "not_live_duplicate_payment_demonstrated",
    "not_exploitability_established",
    "not_payment_system_unsafe_verdict",
    "not_provider_qualified",
    "not_provider_truth_established",
    "not_physical_exactly_once",
    "not_external_settlement_finality",
    "not_public_fund_authority",
    "not_live_governance_route",
    "not_deployment_current",
}


def fail(message: str) -> None:
    raise ValueError(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def source_at(path: str) -> str:
    return subprocess.check_output(["git", "show", f"{RUNTIME}:{path}"], cwd=ROOT, text=True)


def validate_bindings(profile: dict) -> dict[str, str]:
    if profile["runtime_subject"] != RUNTIME:
        fail("runtime subject drift")
    if profile["f0c_semantic_head"] != F0C:
        fail("F0C semantic head drift")
    resolved = {}
    for name, (path, expected_blob) in BINDINGS.items():
        entry = profile["source_bindings"][name]
        if entry["path"] != path or entry["blob"] != expected_blob:
            fail(f"profile source binding drift: {name}")
        actual = git("rev-parse", f"{RUNTIME}:{path}")
        if actual != expected_blob:
            fail(f"Git source binding drift: {name}")
        resolved[name] = source_at(path)
    return resolved


def require(source: str, needle: str, label: str) -> None:
    if needle not in source:
        fail(f"source predicate not observed: {label}")


def forbid(source: str, needle: str, label: str) -> None:
    if needle in source:
        fail(f"source predicate unexpectedly present: {label}")


def validate_sources(sources: dict[str, str]) -> None:
    pay = sources["payments_coordinator"]
    integ = sources["payments_integrity"]
    bridge = sources["governance_finance_bridge"]

    # Payment identity is generated from sender + wall-clock time, not F0 execution identity.
    require(pay, 'id: format!("payment:{}:{}", input.from_did, now.as_micros())', "wall-clock payment ID")
    require(pay, "status: TransferStatus::Completed, // Simplified: immediate completion", "immediate Completed status")
    require(pay, "LinkTypes::PaymentIdToPayment", "payment ID index")
    require(pay, "pub fn get_payment(payment_id: String)", "payment lookup")
    require(pay, "pub fn get_receipt(payment_id: String)", "receipt lookup")
    require(pay, "let sig = sign(agent, sig_data.into_bytes())?;", "sender-agent receipt signature")
    require(pay, "payment.id,", "receipt signing includes payment id")

    # Current governance finance bridge does not route constitutional transfer through payments.
    require(bridge, '"finance", "treasury", "execute_governance_transfer"', "current governance route to treasury")
    forbid(bridge, '"finance", "payments", "send_payment"', "governance route to payments send_payment")

    # DHT integrity has positive author/immutability checks, but no cryptographic receipt theorem.
    require(integ, 'require_did_is_author("Payment", "from_did"', "payment author binding")
    require(integ, '"Receipts cannot be updated"', "receipt immutability")
    require(integ, "Full cryptographic verification requires the sender's public key", "receipt signature verification deferred")
    require(integ, "LinkTypes::PaymentIdToPayment | LinkTypes::MintIdToMintRecord", "payment ID link validation arm")

    # Exact provider-key/reconciliation vocabulary required by F0C is absent on this frozen source.
    forbid(pay, "execution_id", "F0 execution identity in payments provider")
    forbid(pay, "request_commitment", "F0 request commitment in payments provider")
    forbid(pay, "KnownNoEffect", "authoritative KnownNoEffect provider query")
    forbid(pay, "UnknownOutcome", "provider UnknownOutcome query")


def validate_profile(profile: dict) -> None:
    if profile["schema"] != "mycelix.constitutional-payments-provider-observed.v1":
        fail("schema drift")
    if profile["profile_id"] != "mycelix-payments-provider-observed-15b9c89a-v1":
        fail("profile id drift")
    if profile["revision"] != 1 or profile["authority_class"] != "ObservedSourceBound":
        fail("profile authority/revision drift")

    c = profile["candidate"]
    if c["provider"] != "finance::payments" or c["operation"] != "send_payment":
        fail("candidate provider drift")
    if c["current_governance_route"] is not False:
        fail("payments falsely promoted to current governance route")
    key = c["provider_operation_key"]
    if key["source"] != "GeneratedWallClockPaymentId":
        fail("provider key source drift")
    if key["deterministically_bound_to_f0_execution_id"] is not False:
        fail("provider key falsely bound to F0 execution ID")
    if key["stable_across_transport_retry"] is not False:
        fail("provider key falsely marked retry-stable")

    dup = c["duplicate_semantics"]
    if dup["same_execution_same_request_converges_to_same_payment_id"] is not False:
        fail("duplicate convergence inflated")
    if dup["same_key_same_request_returns_existing_effect"] != "NotEstablished":
        fail("same-key duplicate theorem inflated")
    if dup["same_key_different_request_conflicts"] != "NotEstablished":
        fail("changed-request conflict theorem inflated")
    if dup["concurrent_same_key_single_effect"] != "NotEstablished":
        fail("concurrent duplicate theorem inflated")

    out = c["outcome_model"]
    if out["created_payment_status"] != "CompletedImmediately":
        fail("observed payment status drift")
    if out["provider_unknown_outcome_query"] is not False:
        fail("UnknownOutcome query inflated")
    if out["authoritative_known_no_effect_query"] is not False:
        fail("KnownNoEffect query inflated")

    receipt = c["receipt"]
    positives = ("exists", "immutable", "signed_by_sender_agent", "binds_payment_id", "binds_from_to_amount_currency_timestamp")
    if not all(receipt[k] is True for k in positives):
        fail("receipt positive controls weakened")
    negatives = ("binds_f0_execution_id", "binds_f0_request_commitment", "integrity_cryptographically_verifies_signature", "qualified_as_authoritative_provider_receipt")
    if not all(receipt[k] is False for k in negatives):
        fail("receipt capability inflated")

    value = c["value_surface"]
    if value != {"amount_type": "u64", "currency_type": "String", "qualified_sap_amount_consumed": False}:
        fail("historical value-surface classification drift")

    eligibility = profile["f0c_capability_eligibility"]
    if eligibility["automatic_replay_eligible"] is not False:
        fail("automatic replay prematurely enabled")
    if eligibility["qualified_provider_replay_capability_may_be_minted"] is not False:
        fail("capability minting prematurely enabled")
    if set(eligibility["blocking_properties"]) != BLOCKERS:
        fail("blocking property census drift")

    successor = profile["required_successor"]
    if successor["operation_key"] != "DeterministicProviderOperationKeyDerivedFromF0ExecutionId":
        fail("successor operation-key requirement weakened")
    if successor["request_binding"] != "ExactF0RequestCommitment":
        fail("successor request binding weakened")
    if set(successor["query_outcomes"]) != {"KnownSuccess", "KnownNoEffect", "UnknownOutcome"}:
        fail("successor query outcome census drift")
    required_receipt = {
        "provider_operation_key", "execution_id", "request_commitment", "provider_outcome",
        "provider_receipt_identity", "provider_receipt_commitment",
    }
    if set(successor["receipt_bindings"]) != required_receipt:
        fail("successor receipt binding census drift")
    if successor["activation"] != "SeparateF1AfterExactQualification":
        fail("successor activation boundary weakened")

    if set(profile["non_claims"]) != NONCLAIMS:
        fail("non-claim ceiling drift")


def expect_rejected(name: str, fn) -> None:
    try:
        fn()
    except (ValueError, KeyError, TypeError):
        return
    raise AssertionError(f"mutation survived: {name}")


def self_test(profile: dict) -> None:
    def mutate(name, mutator):
        def run():
            p = copy.deepcopy(profile)
            mutator(p)
            validate_profile(p)
        expect_rejected(name, run)

    mutate("mint-capability", lambda p: p["f0c_capability_eligibility"].__setitem__("qualified_provider_replay_capability_may_be_minted", True))
    mutate("automatic-replay", lambda p: p["f0c_capability_eligibility"].__setitem__("automatic_replay_eligible", True))
    mutate("stable-payment-key", lambda p: p["candidate"]["provider_operation_key"].__setitem__("stable_across_transport_retry", True))
    mutate("execution-bound-key", lambda p: p["candidate"]["provider_operation_key"].__setitem__("deterministically_bound_to_f0_execution_id", True))
    mutate("authoritative-receipt", lambda p: p["candidate"]["receipt"].__setitem__("qualified_as_authoritative_provider_receipt", True))
    mutate("verified-signature", lambda p: p["candidate"]["receipt"].__setitem__("integrity_cryptographically_verifies_signature", True))
    mutate("known-no-effect", lambda p: p["candidate"]["outcome_model"].__setitem__("authoritative_known_no_effect_query", True))
    mutate("unknown-query", lambda p: p["candidate"]["outcome_model"].__setitem__("provider_unknown_outcome_query", True))
    mutate("qualified-sap", lambda p: p["candidate"]["value_surface"].__setitem__("qualified_sap_amount_consumed", True))
    mutate("drop-blocker", lambda p: p["f0c_capability_eligibility"].__setitem__("blocking_properties", p["f0c_capability_eligibility"]["blocking_properties"][:-1]))
    mutate("current-route", lambda p: p["candidate"].__setitem__("current_governance_route", True))
    mutate("drop-request-receipt-binding", lambda p: p["required_successor"].__setitem__("receipt_bindings", [x for x in p["required_successor"]["receipt_bindings"] if x != "request_commitment" ]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    profile = json.loads(PROFILE_PATH.read_text())
    validate_profile(profile)
    sources = validate_bindings(profile)
    validate_sources(sources)
    if args.self_test:
        self_test(profile)

    canonical = json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()
    print(json.dumps({
        "validated": True,
        "self_test": args.self_test,
        "profile_id": profile["profile_id"],
        "authority_class": profile["authority_class"],
        "automatic_replay_eligible": profile["f0c_capability_eligibility"]["automatic_replay_eligible"],
        "capability_mint_allowed": profile["f0c_capability_eligibility"]["qualified_provider_replay_capability_may_be_minted"],
        "canonical_profile_sha256": hashlib.sha256(canonical).hexdigest(),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
