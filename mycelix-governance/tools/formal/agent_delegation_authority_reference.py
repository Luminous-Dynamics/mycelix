#!/usr/bin/env python3
"""Exploratory reference model for transitive agent delegation authority."""
from __future__ import annotations

from dataclasses import dataclass, replace

@dataclass(frozen=True)
class Grant:
    name: str
    issuer: str
    grantee: str
    power: str
    parent: str | None = None
    active: bool = False
    revoked: bool = False

@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[str, ...]], ...]
    evidence: tuple[str, ...] = ()
    provider_failed: bool = False
    authority_before_failure: tuple[tuple[str, tuple[str, ...]], ...] = ()

def auth_map(state: State) -> dict[str, set[str]]:
    return {a: set(powers) for a, powers in state.authority}

def grants_map(state: State) -> dict[str, Grant]:
    return {g.name: g for g in state.grants}

def invariant_failures(state: State) -> list[str]:
    amap = auth_map(state)
    gmap = grants_map(state)
    failures: list[str] = []

    for g in state.grants:
        if g.active and g.revoked:
            failures.append("ActiveGrantCurrent")
        if g.active and g.power not in amap.get(g.issuer, set()):
            failures.append("DelegationNonAmplification")
        if g.parent and g.parent in gmap and g.active:
            parent = gmap[g.parent]
            if parent.grantee != g.issuer:
                failures.append("GrantParentMatchesIssuer")
            if parent.revoked:
                failures.append("RevocationPropagates")
    for agent, powers in amap.items():
        if agent != "Root":
            for power in powers:
                if not any(g.active and not g.revoked and g.grantee == agent and g.power == power for g in state.grants):
                    failures.append("NoAuthorityWithoutCurrentGrant")
    if state.evidence and state.evidence:
        for agent, powers in amap.items():
            if agent != "Root":
                for power in powers:
                    if not any(g.active and not g.revoked and g.grantee == agent and g.power == power for g in state.grants):
                        failures.append("EvidenceDoesNotMintAuthority")
    if state.provider_failed and amap != auth_map(
        replace(state, authority=state.authority_before_failure)
    ):
        failures.append("FailureDoesNotMintAuthority")
    return sorted(set(failures))

def main() -> int:
    g1 = Grant("G1", "Root", "A", "P1")
    g2 = Grant("G2", "A", "B", "P1", parent="G1")
    g3 = Grant("G3", "B", "C", "P1", parent="G2")

    initial = State(
        grants=(g1, g2, g3),
        authority=(("Root", ("P1", "P2")), ("A", ()), ("B", ()), ("C", ())),
        authority_before_failure=(("Root", ("P1", "P2")), ("A", ()), ("B", ()), ("C", ())),
    )

    valid = replace(
        initial,
        grants=(replace(g1, active=True), replace(g2, active=True), replace(g3, active=True)),
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1",)), ("C", ("P1",))),
    )
    assert not invariant_failures(valid), invariant_failures(valid)

    bad_child = replace(
        valid,
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1", "P2")), ("C", ("P1",))),
    )
    assert "NoAuthorityWithoutCurrentGrant" in invariant_failures(bad_child)

    bad_revocation = replace(
        valid,
        grants=(replace(g1, active=True, revoked=True), replace(g2, active=True), g3),
    )
    assert "ActiveGrantCurrent" in invariant_failures(bad_revocation)
    assert "RevocationPropagates" in invariant_failures(bad_revocation)

    evidence_bad = replace(
        valid,
        evidence=("E1",),
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1", "P2")), ("C", ("P1",))),
    )
    assert "EvidenceDoesNotMintAuthority" in invariant_failures(evidence_bad)

    failure_bad = replace(
        valid,
        provider_failed=True,
        authority=(("Root", ("P1", "P2")), ("A", ("P1", "P2")), ("B", ("P1",)), ("C", ("P1",))),
    )
    assert "FailureDoesNotMintAuthority" in invariant_failures(failure_bad)

    print("CANONICAL PASS: bounded valid delegation state")
    print("NEGATIVE PASS: undelegated child authority detected")
    print("NEGATIVE PASS: revoked ancestor with active descendant detected")
    print("NEGATIVE PASS: evidence-only authority mint detected")
    print("NEGATIVE PASS: provider failure authority amplification detected")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
