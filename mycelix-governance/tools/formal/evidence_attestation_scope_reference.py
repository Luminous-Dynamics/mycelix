#!/usr/bin/env python3
"""Reference semantics for claim-scope attenuation."""
from __future__ import annotations
from dataclasses import dataclass, replace

@dataclass(frozen=True)
class Grant:
    name: str
    issuer: str
    grantee: str
    scope: frozenset[str]
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
    claim_scope: frozenset[str]
    grants_before: tuple[str, ...]
    grants_after: tuple[str, ...]
    authority_before: frozenset[str]
    authority_after: frozenset[str]

@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[str, ...]], ...]
    evidence: tuple[Evidence, ...] = ()

def auth_map(state: State) -> dict[str, set[str]]:
    return {a: set(s) for a, s in state.authority}

def grant_authority(state: State) -> dict[str, set[str]]:
    out = {"Root": {"S1", "S2"}, "Alice": set()}
    for g in state.grants:
        if g.active and not g.revoked:
            out.setdefault(g.grantee, set()).update(g.scope)
    return out

def steady_state_failures(state: State) -> list[str]:
    failures: list[str] = []
    if auth_map(state) != grant_authority(state):
        failures.append("AuthorityMatchesCurrentGrants")
    amap = auth_map(state)
    for g in state.grants:
        if g.active and g.revoked:
            failures.append("ActiveGrantCurrent")
        if g.active and not g.scope.issubset(amap.get(g.issuer, set())):
            failures.append("GrantScopeWithinIssuerAuthority")
    for power in amap.get("Alice", set()):
        if power not in grant_authority(state).get("Alice", set()):
            failures.append("NoAuthorityWithoutCurrentGrant")
    return sorted(set(failures))

def transition_failures(state: State) -> list[str]:
    failures: list[str] = []
    for e in state.evidence:
        if (
            e.signature_valid and e.signer_trusted and e.claim_authorized
            and e.subject == e.target
            and not (e.authority_after - e.authority_before).issubset(e.claim_scope)
        ):
            failures.append("ClaimAuthorizationScopeAttenuated")
    return sorted(set(failures))

def main() -> int:
    g1 = Grant("G1", "Root", "Alice", frozenset({"S1"}), active=True)
    g2 = Grant("G2", "Root", "Alice", frozenset({"S2"}))
    base = State(
        grants=(g1, g2),
        authority=(("Root", ("S1", "S2")), ("Alice", ("S1",))),
    )
    assert not steady_state_failures(base), steady_state_failures(base)

    valid = replace(
        base,
        grants=(g1, replace(g2, active=True)),
        authority=(("Root", ("S1", "S2")), ("Alice", ("S1", "S2"))),
        evidence=(Evidence(
            "E1", True, True, True, "Alice", "Alice", frozenset({"S1", "S2"}),
            ("G1",), ("G1", "G2"),
            frozenset({"S1"}), frozenset({"S1", "S2"}),
        ),),
    )
    assert not steady_state_failures(valid), steady_state_failures(valid)
    assert not transition_failures(valid), transition_failures(valid)

    bad = replace(
        valid,
        evidence=(replace(valid.evidence[0], claim_scope=frozenset({"S1"})),),
    )
    assert not steady_state_failures(bad), steady_state_failures(bad)
    assert transition_failures(bad) == ["ClaimAuthorizationScopeAttenuated"]

    print("CANONICAL PASS: authorized claim permits only claim-bounded authority scope")
    print("ISOLATION PASS: broad grant remains fully grant-backed outside claim scope")
    print("NEGATIVE PASS: claim-to-authority scope expansion detected")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
