#!/usr/bin/env python3
"""Reference semantics for atomic claim composition."""
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
    authority: frozenset[Capability]


@dataclass(frozen=True)
class State:
    claims: tuple[Claim, ...]
    composition: Composition


def authorized_capabilities(state: State) -> frozenset[Capability]:
    return frozenset(
        claim.capability
        for claim in state.claims
        if claim.authorized and claim.grant_backed
    )


def failures(state: State) -> list[str]:
    failures: list[str] = []
    for claim in state.claims:
        if claim.authorized and not claim.grant_backed:
            failures.append("AuthorizedClaimsAreGrantBacked")
    if not state.composition.authority.issubset(authorized_capabilities(state)):
        failures.append("CompositionOnlyUsesAtomicAuthorizedCapabilities")
    return sorted(set(failures))


def main() -> int:
    c1 = Capability("R1", "Read")
    c2 = Capability("R2", "Write")
    synthetic = Capability("R1", "Write")

    state = State(
        claims=(
            Claim("Claim1", c1, True, True),
            Claim("Claim2", c2, True, True),
        ),
        composition=Composition(frozenset({c1, c2})),
    )
    assert not failures(state), failures(state)
    print("CANONICAL PASS: composing valid claims preserves atomic authorized capabilities")

    isolated_bad = State(
        claims=state.claims,
        composition=Composition(frozenset({c1, c2, synthetic})),
    )
    assert "AuthorizedClaimsAreGrantBacked" not in failures(isolated_bad)
    assert failures(isolated_bad) == ["CompositionOnlyUsesAtomicAuthorizedCapabilities"]
    print("ISOLATION PASS: cross-product capability is not caused by claim or grant invalidity")
    print("NEGATIVE PASS: cartesian composition authority amplification detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
