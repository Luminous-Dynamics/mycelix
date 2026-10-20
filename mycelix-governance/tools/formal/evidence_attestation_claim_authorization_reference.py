#!/usr/bin/env python3
"""Reference semantics for claim-specific attestation authorization."""
from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Grant:
    name: str
    issuer: str
    grantee: str
    power: str
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
    claim_type: str
    grants_before: tuple[str, ...]
    grants_after: tuple[str, ...]
    authority_before: tuple[tuple[str, tuple[str, ...]], ...]
    authority_after: tuple[tuple[str, tuple[str, ...]], ...]


@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[str, ...]], ...]
    evidence: tuple[Evidence, ...] = ()


def auth_map(state: State) -> dict[str, set[str]]:
    return {agent: set(powers) for agent, powers in state.authority}


def grant_authority(state: State) -> dict[str, set[str]]:
    result = {"Root": {"P1", "P2"}, "Alice": set()}
    for grant in state.grants:
        if grant.active and not grant.revoked:
            result[grant.grantee].add(grant.power)
    return result


def steady_state_failures(state: State) -> list[str]:
    failures: list[str] = []
    if auth_map(state) != grant_authority(state):
        failures.append("AuthorityMatchesCurrentGrants")
    amap = auth_map(state)
    for grant in state.grants:
        if grant.active and grant.revoked:
            failures.append("ActiveGrantCurrent")
        if grant.active and grant.power not in amap.get(grant.issuer, set()):
            failures.append("GrantCannotExceedIssuerAuthority")
    for agent in ("Alice",):
        for power in amap.get(agent, set()):
            if power not in grant_authority(state)[agent]:
                failures.append("NoAuthorityWithoutCurrentGrant")
    return sorted(set(failures))


def transition_failures(state: State) -> list[str]:
    failures: list[str] = []
    for evidence in state.evidence:
        if (
            evidence.signature_valid
            and evidence.signer_trusted
            and not evidence.claim_authorized
            and evidence.subject == evidence.target
            and evidence.authority_before != evidence.authority_after
        ):
            failures.append("UnauthorizedClaimCannotChangeAuthority")
    return sorted(set(failures))


def main() -> int:
    g1 = Grant("G1", "Root", "Alice", "P1", active=True)
    g2 = Grant("G2", "Root", "Alice", "P2")

    base = State(
        grants=(g1, g2),
        authority=(("Root", ("P1", "P2")), ("Alice", ("P1",))),
    )
    assert not steady_state_failures(base)

    valid = replace(
        base,
        grants=(g1, replace(g2, active=True)),
        authority=(("Root", ("P1", "P2")), ("Alice", ("P1", "P2"))),
        evidence=(Evidence(
            "E1", True, True, True, "Alice", "Alice", "ClaimA",
            ("G1",), ("G1", "G2"),
            base.authority,
            (("Root", ("P1", "P2")), ("Alice", ("P1", "P2"))),
        ),),
    )
    assert not steady_state_failures(valid)
    assert not transition_failures(valid)

    bad = replace(
        valid,
        evidence=(replace(valid.evidence[0], claim_authorized=False),),
    )
    assert not steady_state_failures(bad)
    assert transition_failures(bad) == ["UnauthorizedClaimCannotChangeAuthority"]

    print("CANONICAL PASS: authorized trusted subject-bound claim may accompany a grant-backed authority transition")
    print("ISOLATION PASS: unauthorized claim leaves steady-state grant provenance valid")
    print("NEGATIVE PASS: unauthorized claim authority delta detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
