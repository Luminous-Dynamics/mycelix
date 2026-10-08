#!/usr/bin/env python3
"""Reference semantics for exact contributor provenance."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Capability:
    resource: str
    action: str


@dataclass(frozen=True)
class Claim:
    name: str
    capability: Capability
    authorized: bool
    grant_backed: bool


@dataclass(frozen=True)
class Composition:
    input_claims: frozenset[str]
    contributor_claims: frozenset[str]
    authority: frozenset[Capability]


@dataclass(frozen=True)
class State:
    claims: tuple[Claim, ...]
    composition: Composition


def claim_map(state: State) -> dict[str, Claim]:
    return {c.name: c for c in state.claims}


def failures(state: State) -> list[str]:
    claims = claim_map(state)
    failures: list[str] = []
    for name in state.composition.input_claims:
        claim = claims[name]
        if not (claim.authorized and claim.grant_backed):
            failures.append("AllInputClaimsValid")
    expected = frozenset(
        claims[name].capability for name in state.composition.input_claims
    )
    if not state.composition.authority.issubset(expected):
        failures.append("AuthorityComesFromInputClaims")
    for name in state.composition.contributor_claims:
        claim = claims[name]
        if not (
            claim.authorized
            and claim.grant_backed
            and claim.capability in state.composition.authority
        ):
            failures.append("ContributorsAuthorizeTheirCapabilities")
    if state.composition.contributor_claims != state.composition.input_claims:
        failures.append("CompositionProvenanceMatchesInputs")
    return sorted(set(failures))


def main() -> int:
    cap1 = Capability("R1", "Read")
    cap2 = Capability("R2", "Write")

    state = State(
        claims=(
            Claim("Claim1", cap1, True, True),
            Claim("Claim2", cap2, True, True),
            Claim("Claim3", cap2, True, True),
        ),
        composition=Composition(
            input_claims=frozenset({"Claim1", "Claim2"}),
            contributor_claims=frozenset({"Claim1", "Claim2"}),
            authority=frozenset({cap1, cap2}),
        ),
    )
    assert not failures(state), failures(state)
    print("CANONICAL PASS: exact contributing claim identities match the composition inputs")

    bad = State(
        claims=state.claims,
        composition=Composition(
            input_claims=frozenset({"Claim1", "Claim2"}),
            contributor_claims=frozenset({"Claim1", "Claim3"}),
            authority=frozenset({cap1, cap2}),
        ),
    )
    assert "AllInputClaimsValid" not in failures(bad)
    assert "AuthorityComesFromInputClaims" not in failures(bad)
    assert "ContributorsAuthorizeTheirCapabilities" not in failures(bad)
    assert failures(bad) == ["CompositionProvenanceMatchesInputs"]
    print("ISOLATION PASS: contributor substitution leaves capability and grant validity unchanged")
    print("NEGATIVE PASS: exact composition contributor provenance substitution detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
