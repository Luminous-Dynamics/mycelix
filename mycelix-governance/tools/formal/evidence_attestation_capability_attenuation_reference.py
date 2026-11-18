#!/usr/bin/env python3
"""Reference semantics for multidimensional capability attenuation."""
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
    claim_resources: frozenset[str]
    claim_actions: frozenset[str]
    claim_audiences: frozenset[str]
    claim_expiry: int
    grants_before: tuple[str, ...]
    grants_after: tuple[str, ...]
    authority_before: frozenset[Capability]
    authority_after: frozenset[Capability]


@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[Capability, ...]], ...]
    evidence: tuple[Evidence, ...] = ()


def auth_map(state: State) -> dict[str, set[Capability]]:
    return {agent: set(caps) for agent, caps in state.authority}


def capability_universe() -> set[Capability]:
    return {
        Capability(resource, action, audience, expiry)
        for resource in ("R1", "R2")
        for action in ("Read", "Write")
        for audience in ("AudienceA", "AudienceB")
        for expiry in (1, 2)
    }


def grant_authority(state: State) -> dict[str, set[Capability]]:
    return {
        "Root": capability_universe(),
        "Alice": {
            grant.capability
            for grant in state.grants
            if grant.active and not grant.revoked
        },
    }


def steady_state_failures(state: State) -> list[str]:
    failures: list[str] = []
    amap = auth_map(state)
    derived = grant_authority(state)
    if amap != derived:
        failures.append("AuthorityMatchesCurrentGrants")
    for grant in state.grants:
        if grant.active and grant.revoked:
            failures.append("ActiveGrantCurrent")
        if grant.active and grant.capability not in amap.get(grant.issuer, set()):
            failures.append("GrantCapabilitiesWithinIssuerAuthority")
    for capability in amap.get("Alice", set()):
        if capability not in derived["Alice"]:
            failures.append("NoAuthorityWithoutCurrentGrant")
    return sorted(set(failures))


def transition_failures(state: State) -> list[str]:
    failures: list[str] = []
    for evidence in state.evidence:
        if not (
            evidence.signature_valid
            and evidence.signer_trusted
            and evidence.claim_authorized
            and evidence.subject == evidence.target
        ):
            continue
        for capability in evidence.authority_after - evidence.authority_before:
            if capability.resource not in evidence.claim_resources:
                failures.append("ResourceScopeAttenuated")
            if capability.action not in evidence.claim_actions:
                failures.append("ActionScopeAttenuated")
            if capability.audience not in evidence.claim_audiences:
                failures.append("AudienceScopeAttenuated")
            if capability.expiry > evidence.claim_expiry:
                failures.append("ExpiryScopeAttenuated")
    return sorted(set(failures))


def state_for(grant: Grant, evidence: Evidence, root_caps: tuple[Capability, ...]) -> State:
    return State(
        grants=(g1, grant),
        authority=(
            ("Root", root_caps),
            ("Alice", (g1.capability, grant.capability)),
        ),
        evidence=(evidence,),
    )


g1 = Grant("G1", "Root", "Alice", Capability("R1", "Read", "AudienceA", 1), active=True)
g2 = Grant("G2", "Root", "Alice", Capability("R2", "Read", "AudienceA", 1))
root_caps_tuple = tuple(sorted(capability_universe(), key=lambda c: (c.resource, c.action, c.audience, c.expiry)))

base = State(
    grants=(g1, g2),
    authority=(("Root", root_caps_tuple), ("Alice", (g1.capability,))),
)


def main() -> int:
    assert not steady_state_failures(base), steady_state_failures(base)

    valid = replace(
        base,
        grants=(g1, replace(g2, active=True)),
        authority=(("Root", root_caps_tuple), ("Alice", (g1.capability, g2.capability))),
        evidence=(
            Evidence(
                "E1",
                True,
                True,
                True,
                "Alice",
                "Alice",
                frozenset({"R1", "R2"}),
                frozenset({"Read", "Write"}),
                frozenset({"AudienceA", "AudienceB"}),
                2,
                ("G1",),
                ("G1", "G2"),
                frozenset({g1.capability}),
                frozenset({g1.capability, g2.capability}),
            ),
        ),
    )
    assert not steady_state_failures(valid), steady_state_failures(valid)
    assert not transition_failures(valid), transition_failures(valid)

    cases = {
        "resource": (
            replace(g2, active=True),
            replace(valid.evidence[0], claim_resources=frozenset({"R1"})),
        ),
        "action": (
            replace(
                g2,
                capability=Capability("R2", "Write", "AudienceA", 1),
                active=True,
            ),
            replace(
                valid.evidence[0],
                claim_actions=frozenset({"Read"}),
                authority_after=frozenset(
                    {
                        g1.capability,
                        Capability("R2", "Write", "AudienceA", 1),
                    }
                ),
            ),
        ),
        "audience": (
            replace(
                g2,
                capability=Capability("R2", "Read", "AudienceB", 1),
                active=True,
            ),
            replace(
                valid.evidence[0],
                claim_audiences=frozenset({"AudienceA"}),
                authority_after=frozenset(
                    {
                        g1.capability,
                        Capability("R2", "Read", "AudienceB", 1),
                    }
                ),
            ),
        ),
        "expiry": (
            replace(
                g2,
                capability=Capability("R2", "Read", "AudienceA", 2),
                active=True,
            ),
            replace(
                valid.evidence[0],
                claim_expiry=1,
                authority_after=frozenset(
                    {
                        g1.capability,
                        Capability("R2", "Read", "AudienceA", 2),
                    }
                ),
            ),
        ),
    }
    expected = {
        "resource": "ResourceScopeAttenuated",
        "action": "ActionScopeAttenuated",
        "audience": "AudienceScopeAttenuated",
        "expiry": "ExpiryScopeAttenuated",
    }
    for name, (grant, evidence) in cases.items():
        state = state_for(grant, evidence, root_caps_tuple)
        assert not steady_state_failures(state), (name, steady_state_failures(state))
        assert transition_failures(state) == [expected[name]], (
            name,
            transition_failures(state),
        )

    print("CANONICAL PASS: authorized capability remains within all four claim dimensions")
    print("ISOLATION PASS: resource expansion leaves grant provenance valid")
    print("NEGATIVE PASS: resource attenuation violation detected")
    print("ISOLATION PASS: action expansion leaves grant provenance valid")
    print("NEGATIVE PASS: action attenuation violation detected")
    print("ISOLATION PASS: audience expansion leaves grant provenance valid")
    print("NEGATIVE PASS: audience attenuation violation detected")
    print("ISOLATION PASS: expiry extension leaves grant provenance valid")
    print("NEGATIVE PASS: expiry attenuation violation detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
