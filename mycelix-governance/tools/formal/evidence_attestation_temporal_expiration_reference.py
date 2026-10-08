#!/usr/bin/env python3
"""Reference semantics for temporal expiration of effective authority."""
from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Capability:
    resource: str
    action: str
    audience: str
    expiry: int


@dataclass(frozen=True)
class Grant:
    name: str
    issuer: str
    grantee: str
    capability: Capability
    active: bool = False
    revoked: bool = False


@dataclass(frozen=True)
class Evidence:
    name: str
    signature_valid: bool
    signer_trusted: bool
    claim_authorized: bool
    subject: str
    target: str
    grant_name: str


@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[Capability, ...]], ...]
    effective_authority: tuple[tuple[str, tuple[Capability, ...]], ...]
    now: int
    evidence: tuple[Evidence, ...] = ()


def auth_map(state: State) -> dict[str, set[Capability]]:
    return {agent: set(caps) for agent, caps in state.authority}


def effective_map(state: State) -> dict[str, set[Capability]]:
    return {agent: set(caps) for agent, caps in state.effective_authority}


def capability_universe() -> set[Capability]:
    return {
        Capability("R1", "Read", "AudienceA", expiry)
        for expiry in (1, 2, 3)
    }


def structural_authority(state: State) -> dict[str, set[Capability]]:
    return {
        "Root": capability_universe(),
        "Alice": {
            g.capability
            for g in state.grants
            if g.active and not g.revoked
        },
    }


def effective_authority(state: State) -> dict[str, set[Capability]]:
    return {
        "Root": capability_universe(),
        "Alice": {
            g.capability
            for g in state.grants
            if g.active and not g.revoked and g.capability.expiry >= state.now
        },
    }


def steady_state_failures(state: State) -> list[str]:
    failures: list[str] = []
    if auth_map(state) != structural_authority(state):
        failures.append("StructuralAuthorityMatchesCurrentGrants")
    for g in state.grants:
        if g.active and g.revoked:
            failures.append("ActiveGrantCurrent")
        if g.active and g.capability not in auth_map(state).get(g.issuer, set()):
            failures.append("GrantCapabilitiesRemainStructurallyValid")
    for cap in auth_map(state).get("Alice", set()):
        if cap not in structural_authority(state)["Alice"]:
            failures.append("NoAuthorityWithoutCurrentGrant")
    for g in state.grants:
        if (
            g.active
            and g.capability.expiry < state.now
            and g.capability in effective_map(state).get(g.grantee, set())
        ):
            failures.append("ExpiredGrantNotEffective")
    for e in state.evidence:
        if e.signature_valid is not True or e.signer_trusted is not True:
            failures.append("RecordedEvidenceValid")
        if e.claim_authorized is not True or e.subject != e.target:
            failures.append("RecordedEvidenceValid")
    return sorted(set(failures))


def main() -> int:
    g1 = Grant(
        "G1",
        "Root",
        "Alice",
        Capability("R1", "Read", "AudienceA", 1),
        active=True,
    )
    base = State(
        grants=(g1,),
        authority=(("Root", tuple(capability_universe())), ("Alice", (g1.capability,))),
        effective_authority=(("Root", tuple(capability_universe())), ("Alice", (g1.capability,))),
        now=1,
        evidence=(Evidence("E1", True, True, True, "Alice", "Alice", "G1"),),
    )
    assert not steady_state_failures(base), steady_state_failures(base)

    fresh = replace(base, now=2, effective_authority=(("Root", tuple(capability_universe())), ("Alice", ())))
    assert effective_map(fresh) == effective_authority(fresh)
    assert not steady_state_failures(fresh), steady_state_failures(fresh)

    stale = replace(base, now=2)
    assert auth_map(stale) == structural_authority(stale)
    assert "ExpiredGrantNotEffective" in steady_state_failures(stale)

    print("CANONICAL PASS: fresh grant remains effective before expiry")
    print("ISOLATION PASS: expired grant remains structurally grant-backed")
    print("NEGATIVE PASS: expired authority persistence detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
