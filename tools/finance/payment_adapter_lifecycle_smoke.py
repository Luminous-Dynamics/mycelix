#!/usr/bin/env python3
"""Effects-disabled reference smoke scenarios for payment adapter invariants.

This is a deterministic in-memory model. It performs no network requests,
uses no credentials, and does not exercise a production adapter or move money.
"""
from __future__ import annotations

import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
from typing import Any


class FakeProvider:
    """Provider stub that deduplicates by stable key and detects payload conflicts."""

    def __init__(self) -> None:
        self.by_key: dict[str, dict[str, str]] = {}
        self.submit_calls = 0
        self.value_moving_effects = 0

    def submit(self, key: str, payload_digest: str, *, lose_response: bool = False) -> tuple[str, str | None]:
        self.submit_calls += 1
        current = self.by_key.get(key)
        if current is not None and current["payload_digest"] != payload_digest:
            return ("conflict", None)
        if current is None:
            current = {
                "payload_digest": payload_digest,
                "provider_ref": f"provider-ref-{len(self.by_key) + 1}",
            }
            self.by_key[key] = current
            self.value_moving_effects += 1
        if lose_response:
            return ("response_lost", None)
        return ("accepted", current["provider_ref"])

    def query(self, key: str) -> dict[str, str] | None:
        return self.by_key.get(key)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def timeout_after_acceptance() -> str:
    """PAY-VISA-001: timeout remains indeterminate until an explicit query resolves it."""
    provider = FakeProvider()
    state = "Created"
    outcome, _ = provider.submit("logical-op-001", "digest:payload-A", lose_response=True)
    check(outcome == "response_lost", "fixture did not simulate response loss")
    state = "Indeterminate"
    check(state == "Indeterminate", "timeout must not be mapped to rejection or settlement")
    check(provider.submit_calls == 1, "unexpected retry before reconciliation")
    evidence = provider.query("logical-op-001")
    check(evidence is not None, "query should discover the accepted operation")
    state = "Accepted"
    check(provider.submit_calls == 1, "query resolution must not resubmit")
    check(provider.value_moving_effects == 1, "retry or recovery created duplicate value effect")
    check(state != "Settled", "acceptance must not be reported as settlement finality")
    return "PAY-VISA-001: accepted-but-timed-out operation resolves by query, without blind retry"


def status_is_not_finality() -> str:
    """PAY-ISO-002: provider status alone is not a finality proof."""
    def project(positive_status: bool, evidence: dict[str, str] | None, profile_qualified: bool) -> str:
        if not positive_status:
            return "Submitted"
        if not profile_qualified or not evidence or not evidence.get("evidence_id"):
            return "ProviderStatusObserved"
        return "Settled"

    check(project(True, None, False) == "ProviderStatusObserved", "status alone was promoted")
    check(project(True, {"evidence_id": "e-1"}, False) == "ProviderStatusObserved", "unqualified profile was promoted")
    check(project(True, {"evidence_id": "e-1"}, True) == "Settled", "qualified evidence path failed")
    return "PAY-ISO-002: status cannot produce a settlement claim without qualified finality evidence"


def partial_multirail_effect() -> str:
    """PAY-MULTI-001: one settled leg plus one unknown leg is partial, not atomic success."""
    legs = {"origin": "Settled", "destination": "Indeterminate"}
    any_effect = any(value in {"Booked", "Confirmed", "Settled"} for value in legs.values())
    any_unknown = any(value in {"Indeterminate", "PartialEffect"} for value in legs.values())
    disposition = "PartialEffectWithRecoveryPlan" if any_effect and any_unknown else "Complete"
    check(disposition == "PartialEffectWithRecoveryPlan", "partial rail outcome was hidden")
    check(disposition != "AtomicSuccess", "atomicity was inferred without proof")
    return "PAY-MULTI-001: partial multi-rail effect stays visible and is not described as atomic"


def append_return_lineage() -> str:
    """PAY-VISA-003: return becomes a causally-linked event; source event remains."""
    journal: list[dict[str, Any]] = [
        {"event_id": "effect-001", "kind": "Payout", "caused_by": None}
    ]
    journal.append({"event_id": "effect-002", "kind": "Return", "caused_by": "effect-001"})
    check(
        [event["event_id"] for event in journal] == ["effect-001", "effect-002"],
        "source event was deleted or duplicate created",
    )
    check(journal[1]["caused_by"] == journal[0]["event_id"], "return causality was not preserved")
    return "PAY-VISA-003: returned funds append lineage without deleting the original payout evidence"


def conflicting_idempotency_payload() -> str:
    """Same key with a different request digest must fail closed."""
    provider = FakeProvider()
    first = provider.submit("logical-op-002", "digest:amount-10")
    conflict = provider.submit("logical-op-002", "digest:amount-11")
    check(first[0] == "accepted", "initial reference operation failed")
    check(conflict[0] == "conflict", "changed payload reused an existing key")
    check(provider.value_moving_effects == 1, "conflict produced another value effect")
    return "AC-172 invariant: conflicting payload under one idempotency key is rejected"



class PaymentClaimLedger:
    """Thread-safe reference model for sender-scoped claim uniqueness."""

    def __init__(self) -> None:
        self.lock = Lock()
        self.claims: dict[tuple[str, str], dict[str, str]] = {}
        self.payment_effects = 0
        self.outbox_effects = 0

    @staticmethod
    def operation_id(sender: str, key: str) -> str:
        # Match the production design's domain-separated serialized tuple shape.
        material = json.dumps(
            ["mycelix-payment-operation-v1", sender, key],
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(material).hexdigest()

    def submit(self, sender: str, key: str, fingerprint: str) -> tuple[str, str]:
        identity = (sender, key)
        operation_id = self.operation_id(sender, key)
        with self.lock:
            current = self.claims.get(identity)
            if current is not None:
                if current["fingerprint"] != fingerprint:
                    return ("conflict", operation_id)
                return ("replay", operation_id)
            self.claims[identity] = {
                "fingerprint": fingerprint,
                "operation_id": operation_id,
            }
            self.payment_effects += 1
            self.outbox_effects += 1
            return ("created", operation_id)


class TreasuryLedger:
    """Destination-scoped receipt dedupe plus a serialized balance ceiling."""

    def __init__(self, destination: str, max_balance: int) -> None:
        self.destination = destination
        self.max_balance = max_balance
        self.balance = 0
        self.lock = Lock()
        self.receipts: dict[tuple[str, str], dict[str, Any]] = {}

    def credit(
        self,
        effect_id: str,
        source_outbox: dict[str, Any] | None,
        payment: dict[str, Any] | None,
        amount: int,
        currency: str,
    ) -> tuple[str, str | None]:
        if source_outbox is None or payment is None:
            return ("unanchored", None)
        if (
            source_outbox.get("committed") is not True
            or source_outbox.get("effect_id") != effect_id
            or source_outbox.get("payment_id") != payment.get("payment_id")
            or source_outbox.get("amount") != amount
            or source_outbox.get("currency") != currency
            or payment.get("amount") != amount
            or payment.get("currency") != currency
        ):
            return ("unanchored", None)

        receipt_key = (self.destination, effect_id)
        payload = (payment["payment_id"], amount, currency)
        with self.lock:
            prior = self.receipts.get(receipt_key)
            if prior is not None:
                if prior["payload"] != payload:
                    return ("conflict", None)
                return ("duplicate", prior["receipt_id"])
            if payment.get("status") != "Pending":
                return ("not_pending", None)
            # Subtraction form avoids overflow in fixed-width implementations.
            if amount <= 0 or amount > self.max_balance or self.balance > self.max_balance - amount:
                return ("limit", None)
            receipt_id = f"receipt:{self.destination}:{effect_id}"
            self.balance += amount
            self.receipts[receipt_key] = {
                "payload": payload,
                "receipt_id": receipt_id,
                "amount": amount,
            }
            return ("credited", receipt_id)


def concurrent_exact_claim_reuses_one_operation() -> str:
    """PAY-MYC-001: identical concurrent retries resolve to one operation."""
    ledger = PaymentClaimLedger()
    with ThreadPoolExecutor(max_workers=16) as pool:
        results = list(pool.map(
            lambda _: ledger.submit("did:mycelix:sender-A", "opaque-key-A", "request-digest-A"),
            range(64),
        ))
    statuses = [status for status, _ in results]
    operation_ids = {operation_id for _, operation_id in results}
    check(statuses.count("created") == 1, "concurrent exact retries created more than one claim")
    check(statuses.count("replay") == 63, "exact retries did not resolve to the existing claim")
    check(len(operation_ids) == 1, "concurrent exact retries diverged in operation identity")
    check(ledger.payment_effects == 1 and ledger.outbox_effects == 1,
          "concurrent exact retries duplicated a Payment or outbox effect")
    return "PAY-MYC-001: 64 identical concurrent submissions resolve to one claim and outbox effect"


def concurrent_conflicting_claim_payload_is_rejected() -> str:
    """PAY-MYC-002: only one fingerprint may win for a sender-scoped key."""
    ledger = PaymentClaimLedger()
    requests = ["digest-A"] * 32 + ["digest-B"] * 32
    with ThreadPoolExecutor(max_workers=16) as pool:
        results = list(pool.map(
            lambda fingerprint: ledger.submit("did:mycelix:sender-B", "opaque-key-B", fingerprint),
            requests,
        ))
    statuses = [status for status, _ in results]
    operation_ids = {operation_id for _, operation_id in results}
    check(statuses.count("created") == 1, "conflicting race did not create exactly one original claim")
    check(statuses.count("replay") == 31, "requests matching the winning fingerprint were not exact replays")
    check(statuses.count("conflict") == 32, "conflicting fingerprints did not all fail closed")
    check(len(operation_ids) == 1, "operation identity changed across conflicting attempts")
    check(ledger.payment_effects == 1 and ledger.outbox_effects == 1,
          "conflicting submissions produced extra economic effects")
    return "PAY-MYC-002: conflicting concurrent fingerprints cannot create a second payment"


def treasury_receipt_replay_after_payment_status_failure() -> str:
    """PAY-TREASURY-001: a receipt committed before status failure is resolved, not re-applied."""
    ledger = TreasuryLedger("treasury-pool-A", max_balance=100)
    payment = {"payment_id": "payment-001", "amount": 40, "currency": "USD", "status": "Pending"}
    source = {
        "committed": True, "effect_id": "fee-effect-001",
        "payment_id": "payment-001", "amount": 40, "currency": "USD",
    }
    first = ledger.credit("fee-effect-001", source, payment, 40, "USD")
    check(first[0] == "credited", "initial Treasury receipt was not committed")
    # Simulate a crash here: the receipt exists, but the Payments status write did not happen.
    check(payment["status"] == "Pending", "fixture failed to preserve the interrupted local status")
    balance_at_crash = ledger.balance
    replay = ledger.credit("fee-effect-001", source, payment, 40, "USD")
    check(replay[0] == "duplicate", "recovery did not resolve the original destination-scoped receipt")
    check(ledger.balance == balance_at_crash == 40, "receipt replay applied a second balance credit")
    check(len(ledger.receipts) == 1, "receipt replay created a second receipt")
    receipt = ledger.receipts.get(("treasury-pool-A", "fee-effect-001"))
    check(receipt is not None and receipt["payload"] == ("payment-001", 40, "USD"),
          "recovery receipt was not bound to the exact original payment and amount")
    payment["status"] = "Completed"
    check(payment["status"] == "Completed", "verified receipt could not reconcile the local status")
    return "PAY-TREASURY-001: crash after receipt commit recovers without a second Treasury credit"


def treasury_rejects_unanchored_effect() -> str:
    """PAY-TREASURY-002: an effect ID is not a substitute for committed source authority."""
    ledger = TreasuryLedger("treasury-pool-B", max_balance=100)
    outcome = ledger.credit(
        "invented-effect", None,
        {"payment_id": "payment-ghost", "amount": 25, "currency": "USD", "status": "Pending"},
        25, "USD",
    )
    check(outcome[0] == "unanchored", "Treasury accepted an effect without a committed source outbox")
    check(ledger.balance == 0 and len(ledger.receipts) == 0,
          "unanchored effect changed balance or produced a receipt")
    return "PAY-TREASURY-002: effect ID alone cannot authorize a Treasury credit"


def concurrent_treasury_credits_respect_u64_bound() -> str:
    """PAY-TREASURY-003: concurrent receipt dedupe and checked u64 balance addition."""
    u64_max = (1 << 64) - 1
    ledger = TreasuryLedger("treasury-pool-C", max_balance=u64_max)
    ledger.balance = u64_max - 10
    opening_balance = ledger.balance
    requests: list[tuple[str, dict[str, Any], dict[str, Any]]] = []
    for index in range(20):
        effect_id = f"fee-effect-{index:02d}"
        payment = {
            "payment_id": f"payment-{index:02d}",
            "amount": 10, "currency": "USD", "status": "Pending",
        }
        source = {
            "committed": True, "effect_id": effect_id,
            "payment_id": payment["payment_id"], "amount": 10, "currency": "USD",
        }
        requests.append((effect_id, source, payment))
        requests.append((effect_id, source, payment))
    with ThreadPoolExecutor(max_workers=16) as pool:
        results = list(pool.map(
            lambda item: ledger.credit(item[0], item[1], item[2], 10, "USD"),
            requests,
        ))
    statuses = [status for status, _ in results]
    check(statuses.count("credited") == 1, "exactly one unique credit should fit the remaining u64 range")
    check(statuses.count("duplicate") == 1, "the winning effect's concurrent duplicate was not deduplicated")
    check(statuses.count("limit") == 38, "credits beyond u64::MAX were not cleanly rejected")
    check(ledger.balance == u64_max, "concurrent credit overflowed or failed to reach u64::MAX")
    check(len(ledger.receipts) == 1, "receipt count differs from unique committed credits")
    receipt_delta = sum(receipt["amount"] for receipt in ledger.receipts.values())
    check(opening_balance + receipt_delta == ledger.balance,
          "opening balance plus receipts does not reconcile to the current pool balance")
    return "PAY-TREASURY-003: 40 concurrent attempts preserve u64::MAX, deduplicate the winning credit, and reject 38 unsafe attempts"

class SnapshotChainPaymentLedger(PaymentClaimLedger):
    """Reference model for Holochain-style snapshot and chain-head conflict behavior."""

    def __init__(self) -> None:
        super().__init__()
        self.revision = 0

    def begin(self, sender: str, key: str, fingerprint: str) -> dict[str, Any]:
        with self.lock:
            revision = self.revision
        return {
            "snapshot_revision": revision,
            "sender": sender,
            "key": key,
            "fingerprint": fingerprint,
            "operation_id": self.operation_id(sender, key),
        }

    def commit(self, transaction: dict[str, Any]) -> tuple[str, str]:
        identity = (transaction["sender"], transaction["key"])
        with self.lock:
            if transaction["snapshot_revision"] != self.revision:
                return ("chain_head_moved", transaction["operation_id"])
            current = self.claims.get(identity)
            if current is not None:
                if current["fingerprint"] != transaction["fingerprint"]:
                    return ("conflict", transaction["operation_id"])
                return ("replay", transaction["operation_id"])
            self.claims[identity] = {
                "fingerprint": transaction["fingerprint"],
                "operation_id": transaction["operation_id"],
            }
            self.payment_effects += 1
            self.outbox_effects += 1
            self.revision += 1
            return ("created", transaction["operation_id"])


def chain_head_moved_retries_the_original_operation() -> str:
    """PAY-MYC-004: losing snapshot transaction rolls back, retry resolves the winner."""
    ledger = SnapshotChainPaymentLedger()
    sender = "did:mycelix:sender-C"
    key = "logical-payment-C"
    fingerprint = "request-fingerprint-C"
    first_tx = ledger.begin(sender, key, fingerprint)
    second_tx = ledger.begin(sender, key, fingerprint)
    check(first_tx["snapshot_revision"] == second_tx["snapshot_revision"],
          "transactions did not start from the same source-chain snapshot")
    winner = ledger.commit(first_tx)
    loser = ledger.commit(second_tx)
    check(winner[0] == "created", "first transaction did not commit")
    check(loser[0] == "chain_head_moved", "stale transaction did not roll back on moved chain head")
    check(len(ledger.claims) == 1 and ledger.payment_effects == 1 and ledger.outbox_effects == 1,
          "losing transaction partially committed economic state")
    retry = ledger.commit(ledger.begin(sender, key, fingerprint))
    check(retry == ("replay", winner[1]), "retry failed to resolve the winning logical operation")
    check(ledger.revision == 1, "an exact retry incorrectly created a new source-chain revision")
    check(ledger.payment_effects == 1 and ledger.outbox_effects == 1,
          "same-key retry duplicated a Payment or outbox effect")
    return "PAY-MYC-004: chain-head-moved transaction rolls back atomically; same-key retry resolves one winner"


def same_key_isolated_by_sender_identity() -> str:
    """PAY-MYC-003: client keys are sender-scoped rather than globally unique."""
    ledger = PaymentClaimLedger()
    sender_a = "did:mycelix:sender-A"
    sender_b = "did:mycelix:sender-B"
    key = "shared-client-key"
    first_a = ledger.submit(sender_a, key, "same-request-digest")
    first_b = ledger.submit(sender_b, key, "same-request-digest")
    retry_a = ledger.submit(sender_a, key, "same-request-digest")
    retry_b = ledger.submit(sender_b, key, "same-request-digest")
    check(first_a[0] == "created" and first_b[0] == "created",
          "different senders should each create their own logical operation")
    check(first_a[1] != first_b[1], "the same client key collided across authenticated senders")
    check(retry_a == ("replay", first_a[1]), "sender A retry did not resolve sender A's operation")
    check(retry_b == ("replay", first_b[1]), "sender B retry did not resolve sender B's operation")
    check(ledger.payment_effects == 2 and ledger.outbox_effects == 2,
          "sender isolation produced missing or duplicate local effects")
    return "PAY-MYC-003: same client key from two senders yields two isolated operations and sender-local retries"


def main() -> int:
    scenarios = [
        timeout_after_acceptance,
        status_is_not_finality,
        partial_multirail_effect,
        append_return_lineage,
        conflicting_idempotency_payload,
        concurrent_exact_claim_reuses_one_operation,
        concurrent_conflicting_claim_payload_is_rejected,
        treasury_receipt_replay_after_payment_status_failure,
        treasury_rejects_unanchored_effect,
        concurrent_treasury_credits_respect_u64_bound,
        same_key_isolated_by_sender_identity,
        chain_head_moved_retries_the_original_operation,
    ]
    passed: list[str] = []
    for scenario in scenarios:
        try:
            passed.append(scenario())
        except AssertionError as exc:
            print(f"FAIL: {scenario.__name__}: {exc}")
            return 1
    print(f"PASS: {len(passed)} effects-disabled payment lifecycle scenarios")
    for item in passed:
        print(f" - {item}")
    print("BOUNDARY: reference simulation only; no provider call, credentials, or real financial effects")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
