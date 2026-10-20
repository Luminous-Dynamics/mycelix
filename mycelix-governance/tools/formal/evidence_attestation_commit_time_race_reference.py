#!/usr/bin/env python3
"""Reference semantics for preflight -> mutation -> commit authorization races."""

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class State:
    authority_epoch: int
    request_commitment: str
    target: str
    policy_epoch: int
    adapter: str
    invocation_id: str
    capability_expiry: int
    current_time: int


@dataclass(frozen=True)
class Snapshot:
    authority_epoch: int
    request_commitment: str
    target: str
    policy_epoch: int
    adapter: str
    invocation_id: str
    capability_expiry: int
    observed_time: int


def preflight(state: State) -> Snapshot:
    return Snapshot(
        state.authority_epoch, state.request_commitment, state.target,
        state.policy_epoch, state.adapter, state.invocation_id,
        state.capability_expiry, state.current_time
    )


def commit_safe(snapshot: Snapshot, current: State) -> bool:
    return (
        current.authority_epoch == snapshot.authority_epoch
        and current.request_commitment == snapshot.request_commitment
        and current.target == snapshot.target
        and current.policy_epoch == snapshot.policy_epoch
        and current.adapter == snapshot.adapter
        and current.invocation_id == snapshot.invocation_id
        and current.capability_expiry == snapshot.capability_expiry
        and current.current_time < current.capability_expiry
        and current.current_time >= snapshot.observed_time
    )


def commit_unsafe(snapshot: Snapshot, current: State) -> bool:
    return current.current_time >= snapshot.observed_time


BASE = State(1, "req-1", "target-1", 7, "adapter-v1", "inv-1", 10, 3)
SNAPSHOT = preflight(BASE)

CASES = {
    "authority-epoch": replace(BASE, authority_epoch=2),
    "request-commitment": replace(BASE, request_commitment="req-2"),
    "target": replace(BASE, target="target-2"),
    "policy-epoch": replace(BASE, policy_epoch=8),
    "adapter-profile": replace(BASE, adapter="adapter-v2"),
    "invocation-identity": replace(BASE, invocation_id="inv-2"),
    "capability-expiry-change": replace(BASE, capability_expiry=20),
    "time-expiry": replace(BASE, current_time=11),
}

assert commit_safe(SNAPSHOT, BASE)
assert commit_unsafe(SNAPSHOT, BASE)

for name, current in CASES.items():
    assert not commit_safe(SNAPSHOT, current), (name, "safe commit unexpectedly allowed")
    assert commit_unsafe(SNAPSHOT, current), (name, "unsafe stale-preflight commit not reproducible")

print("CANONICAL PASS: preflight snapshot is revalidated at commit time")
print("ISOLATION PASS: each race mutates exactly one current authority condition")
print("NEGATIVE PASS: unsafe check-then-commit accepts each stale preflight, while safe commit rejects it")
