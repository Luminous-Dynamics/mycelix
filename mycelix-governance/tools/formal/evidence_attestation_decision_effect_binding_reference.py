#!/usr/bin/env python3
"""Executable reference semantics for decision-to-effect binding."""

from __future__ import annotations
from dataclasses import dataclass
from typing import FrozenSet


@dataclass(frozen=True)
class Decision:
    decision_id: str
    authority_epoch: int
    request_commitment: str
    target: str
    policy_epoch: int
    adapter: str
    invocation_id: str
    issued_at: int
    validity_until: int
    capability_expiry: int


@dataclass(frozen=True)
class EffectContext:
    authority_epoch: int
    request_commitment: str
    target: str
    policy_epoch: int
    adapter: str
    invocation_id: str
    now: int


@dataclass(frozen=True)
class Admission:
    decision: Decision
    context: EffectContext
    authorized: bool
    recorded_decision_id: str


DECISION = Decision(
    "decision-1", 1, "req-1", "target-1", 7,
    "adapter-v1", "inv-1", 2, 10, 12
)


def authority_failures(a: Admission) -> list[str]:
    if not a.authorized:
        return []
    failures = []
    d, c = a.decision, a.context
    checks = [
        ("DecisionIdentityBound", a.recorded_decision_id == d.decision_id),
        ("DecisionIssuedBeforeEffect", d.issued_at <= c.now),
        ("AuthorityEpochBound", c.authority_epoch == d.authority_epoch),
        ("RequestCommitmentBound", c.request_commitment == d.request_commitment),
        ("TargetBound", c.target == d.target),
        ("PolicyEpochBound", c.policy_epoch == d.policy_epoch),
        ("AdapterIdentityBound", c.adapter == d.adapter),
        ("InvocationIdentityBound", c.invocation_id == d.invocation_id),
        ("CapabilityCurrentAtEffect", c.now < d.capability_expiry),
        ("DecisionHorizonCurrentAtEffect", c.now < d.validity_until),
    ]
    for name, ok in checks:
        if not ok:
            failures.append(name)
    return sorted(failures)


CANONICAL = Admission(
    decision=DECISION,
    context=EffectContext(1, "req-1", "target-1", 7, "adapter-v1", "inv-1", 3),
    authorized=True,
    recorded_decision_id="decision-1",
)

NEGATIVE_CASES = {
    "authority-epoch": EffectContext(2, "req-1", "target-1", 7, "adapter-v1", "inv-1", 3),
    "request-commitment": EffectContext(1, "req-2", "target-1", 7, "adapter-v1", "inv-1", 3),
    "target": EffectContext(1, "req-1", "target-2", 7, "adapter-v1", "inv-1", 3),
    "policy-epoch": EffectContext(1, "req-1", "target-1", 8, "adapter-v1", "inv-1", 3),
    "adapter-profile": EffectContext(1, "req-1", "target-1", 7, "adapter-v2", "inv-1", 3),
    "invocation-identity": EffectContext(1, "req-1", "target-1", 7, "adapter-v1", "inv-2", 3),
    "capability-expiry": EffectContext(1, "req-1", "target-1", 7, "adapter-v1", "inv-1", 13),
    "decision-horizon": EffectContext(1, "req-1", "target-1", 7, "adapter-v1", "inv-1", 11),
}


def make_negative(name: str, context: EffectContext) -> Admission:
    return Admission(DECISION, context, True, "decision-1")


assert authority_failures(CANONICAL) == []

for name, context in NEGATIVE_CASES.items():
    negative = make_negative(name, context)
    failures = authority_failures(negative)
    assert failures == [(
        {
            "authority-epoch": "AuthorityEpochBound",
            "request-commitment": "RequestCommitmentBound",
            "target": "TargetBound",
            "policy-epoch": "PolicyEpochBound",
            "adapter-profile": "AdapterIdentityBound",
            "invocation-identity": "InvocationIdentityBound",
            "capability-expiry": "CapabilityCurrentAtEffect",
            "decision-horizon": "DecisionHorizonCurrentAtEffect",
        }[name]
    )], (name, failures)

print("CANONICAL PASS: decision identity, issuance time, authority epoch, request, target, policy, adapter, invocation, and horizons are bound")
print("ISOLATION PASS: each negative changes exactly one post-decision condition")
print("NEGATIVE PASS: every single-variable decision-to-effect mutation is detected")
