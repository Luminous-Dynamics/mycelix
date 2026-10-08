#!/usr/bin/env python3
"""Reference trace semantics for commit-time authorization races."""

from dataclasses import dataclass


@dataclass(frozen=True)
class State:
    authority_epoch: int
    request_commitment: str
    policy_epoch: int
    capability_expiry: int
    current_time: int


@dataclass(frozen=True)
class Snapshot:
    authority_epoch: int
    request_commitment: str
    policy_epoch: int
    capability_expiry: int
    observed_time: int


def preflight(state: State) -> Snapshot:
    return Snapshot(
        state.authority_epoch,
        state.request_commitment,
        state.policy_epoch,
        state.capability_expiry,
        state.current_time,
    )


def commit_safe(snapshot: Snapshot, current: State) -> bool:
    return (
        current.authority_epoch == snapshot.authority_epoch
        and current.request_commitment == snapshot.request_commitment
        and current.policy_epoch == snapshot.policy_epoch
        and current.current_time < current.capability_expiry
        and current.current_time >= snapshot.observed_time
    )


def commit_unsafe(snapshot: Snapshot, current: State) -> bool:
    return current.current_time >= snapshot.observed_time


BASE = State(1, "req-1", 7, 10, 3)
SNAP = preflight(BASE)

CASES = {
    "authority-epoch": State(2, "req-1", 7, 10, 3),
    "request-commitment": State(1, "req-2", 7, 10, 3),
    "policy-epoch": State(1, "req-1", 8, 10, 3),
    "capability-expiry": State(1, "req-1", 7, 3, 3),
}

assert commit_safe(SNAP, BASE)
assert commit_unsafe(SNAP, BASE)

for name, current in CASES.items():
    assert not commit_safe(SNAP, current), (name, "safe commit unexpectedly allowed")
    assert commit_unsafe(SNAP, current), (name, "unsafe stale-preflight commit not reproducible")

print("CANONICAL PASS: preflight snapshot is revalidated at commit time")
print("ISOLATION PASS: each race mutates exactly one current authority condition")
print("NEGATIVE PASS: unsafe check-then-commit accepts each stale preflight, while safe commit rejects it")
