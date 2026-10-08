#!/usr/bin/env python3
"""Exploratory reference model for transitive agent delegation and evidence provenance."""
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
class EvidenceTransition:
    name: str
    grants_before: tuple[str, ...]
    grants_after: tuple[str, ...]
    authority_before: tuple[tuple[str, tuple[str, ...]], ...]
    authority_after: tuple[tuple[str, tuple[str, ...]], ...]
    recorded: bool = True


@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[str, ...]], ...]
    evidence: tuple[EvidenceTransition, ...] = ()
    provider_failed: bool = False
    authority_before_failure: tuple[tuple[str, tuple[str, ...]], ...] = ()


def auth_map(state: State) -> dict[str, set[str]]:
    return {a: set(powers) for a, powers in state.authority}


def grants_map(state: State) -> dict[str, Grant]:
    return {g.name: g for g in state.grants}


def ancestor_chain(grant: Grant, grants: dict[str, Grant]) -> set[str]:
    seen: set[str] = set()
    parent = grant.parent
    while parent and parent in grants and parent not in seen:
        seen.add(parent)
        parent = grants[parent].parent
    return seen


def steady_state_failures(state: State) -> list[str]:
    amap = auth_map(state)
    gmap = grants_map(state)
    failures: list[str] = []

    for g in state.grants:
        if g.active and g.revoked:
            failures.append("ActiveGrantCurrent")
        if g.active and g.power not in amap.get(g.issuer, set()):
            failures.append("DelegationNonAmplification")
        for ancestor_name in ancestor_chain(g, gmap):
            ancestor = gmap[ancestor_name]
            if g.active and g.power not in amap.get(ancestor.issuer, set()):
                failures.append("TransitiveDelegationBounded")
        if g.parent and g.parent in gmap and g.active:
            parent = gmap[g.parent]
            if parent.grantee != g.issuer:
                failures.append("GrantParentMatchesIssuer")
        if g.active and any(
            gmap[a].revoked for a in ancestor_chain(g, gmap) if a in gmap
        ):
            failures.append("RevocationPropagates")

    for agent, powers in amap.items():
        if agent != "Root":
            for power in powers:
                if not any(
                    g.active
                    and not g.revoked
                    and g.grantee == agent
                    and g.power == power
                    for g in state.grants
                ):
                    failures.append("NoAuthorityWithoutCurrentGrant")

    if state.provider_failed and amap != auth_map(
        replace(state, authority=state.authority_before_failure)
    ):
        failures.append("FailureDoesNotMintAuthority")

    return sorted(set(failures))


def transition_failures(state: State) -> list[str]:
    failures: list[str] = []
    for evidence in state.evidence:
        if evidence.recorded and evidence.authority_before != evidence.authority_after:
            failures.append("EvidenceDoesNotMintAuthority")
    return sorted(set(failures))


def main() -> int:
    g1 = Grant("G1", "Root", "A", "P1")
    g2 = Grant("G2", "A", "B", "P1", parent="G1")
    g3 = Grant("G3", "B", "C", "P1", parent="G2")
    g4 = Grant("G4", "Root", "B", "P2")

    empty = (("Root", ("P1", "P2")), ("A", ()), ("B", ()), ("C", ()))
    valid = State(
        grants=(replace(g1, active=True), replace(g2, active=True), replace(g3, active=True)),
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1",)), ("C", ("P1",))),
        authority_before_failure=empty,
    )

    # Preserve the previously exercised steady-state controls.
    bad_child = replace(
        valid,
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1", "P2")), ("C", ("P1",))),
    )
    assert "NoAuthorityWithoutCurrentGrant" in steady_state_failures(bad_child)

    valid_with_four = replace(
        valid,
        grants=(
            replace(g1, active=True),
            replace(g2, active=True),
            replace(g3, active=True),
            replace(g4, active=True),
        ),
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1", "P2")), ("C", ("P1",))),
    )
    assert not steady_state_failures(valid_with_four), steady_state_failures(valid_with_four)

    g3_bad = replace(g3, power="P2")
    bad_transitive = replace(
        valid_with_four,
        grants=(
            replace(g1, active=True),
            replace(g2, active=True),
            replace(g3_bad, active=True),
            replace(g4, active=True),
        ),
        authority=(("Root", ("P1", "P2")), ("A", ("P1",)), ("B", ("P1", "P2")), ("C", ("P2",))),
    )
    assert "TransitiveDelegationBounded" in steady_state_failures(bad_transitive)

    bad_revocation = replace(
        valid,
        grants=(replace(g1, active=True, revoked=True), replace(g2, active=True), replace(g3, active=True)),
    )
    assert "ActiveGrantCurrent" in steady_state_failures(bad_revocation)
    assert "RevocationPropagates" in steady_state_failures(bad_revocation)

    failure_bad = replace(
        valid,
        provider_failed=True,
        authority=(("Root", ("P1", "P2")), ("A", ("P1", "P2")), ("B", ("P1",)), ("C", ("P1",))),
    )
    assert "FailureDoesNotMintAuthority" in steady_state_failures(failure_bad)

    # Canonical evidence recording preserves authority and does not mint it.
    canonical_evidence = replace(
        valid,
        evidence=(
            EvidenceTransition(
                name="E1",
                grants_before=("G1", "G2", "G3"),
                grants_after=("G1", "G2", "G3"),
                authority_before=valid.authority,
                authority_after=valid.authority,
            ),
        ),
    )
    assert not steady_state_failures(canonical_evidence), steady_state_failures(canonical_evidence)
    assert not transition_failures(canonical_evidence), transition_failures(canonical_evidence)

    # Isolated evidence negative: the resulting authority is fully grant-backed,
    # yet the evidence-recording transition itself changes authority.
    after_grant = valid_with_four
    bad_evidence = replace(
        after_grant,
        evidence=(
            EvidenceTransition(
                name="E1",
                grants_before=("G1", "G2", "G3"),
                grants_after=("G1", "G2", "G3", "G4"),
                authority_before=valid.authority,
                authority_after=after_grant.authority,
            ),
        ),
    )
    assert not steady_state_failures(bad_evidence), steady_state_failures(bad_evidence)
    assert "EvidenceDoesNotMintAuthority" in transition_failures(bad_evidence)

    print("CANONICAL PASS: bounded valid delegation state")
    print("CANONICAL PASS: evidence recording preserves authority")
    print("NEGATIVE PASS: undelegated child authority detected")
    print("NEGATIVE PASS: transitive delegation authority amplification detected")
    print("NEGATIVE PASS: revoked ancestor with active descendant detected")
    print("NEGATIVE PASS: provider failure authority amplification detected")
    print("ISOLATION PASS: evidence delta leaves steady-state grant provenance valid")
    print("NEGATIVE PASS: evidence-only transition attribution detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
