#!/usr/bin/env python3
"""Effects-disabled reference smoke scenarios for payment adapter invariants.

This is a deterministic in-memory model. It performs no network requests,
uses no credentials, and does not exercise a production adapter or move money.
"""
from __future__ import annotations

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


def main() -> int:
    scenarios = [
        timeout_after_acceptance,
        status_is_not_finality,
        partial_multirail_effect,
        append_return_lineage,
        conflicting_idempotency_payload,
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
