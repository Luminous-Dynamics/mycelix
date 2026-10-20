#!/usr/bin/env python3
"""Executable reference semantics for composed capability evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet


@dataclass(frozen=True)
class Capability:
    resource: str
    action: str
    audience: str
    expiry: int


@dataclass(frozen=True)
class Claim:
    name: str
    capability: Capability
    authorized: bool = True
    grant_backed: bool = True
    signer_trusted: bool = True
    subject: str = "Alice"
    target: str = "Alice"


@dataclass(frozen=True)
class Request:
    resource: str
    action: str
    audience: str
    now: int


@dataclass(frozen=True)
class Decision:
    authority: FrozenSet[Capability]
    input_claims: FrozenSet[str]
    composition_contributors: tuple[tuple[Capability, FrozenSet[str]], ...]
    request: Request
    authorized: bool
    contributors: FrozenSet[str]
    resource_contributors: FrozenSet[str]
    action_contributors: FrozenSet[str]
    audience_contributors: FrozenSet[str]
    temporal_contributors: FrozenSet[str]


CLAIMS = (
    Claim("Claim1", Capability("R1", "Read", "AudienceA", 2)),
    Claim("Claim2", Capability("R2", "Write", "AudienceA", 5)),
    Claim("Claim3", Capability("R1", "Write", "AudienceB", 5)),
    Claim("Claim4", Capability("R2", "Read", "AudienceB", 2)),
)

INPUT = frozenset(c.name for c in CLAIMS)
AUTHORITY = frozenset(c.capability for c in CLAIMS)
BY_NAME = {c.name: c for c in CLAIMS}


def composition_contributor_map() -> dict[Capability, FrozenSet[str]]:
    return {
        cap: frozenset(c.name for c in CLAIMS if c.capability == cap)
        for cap in AUTHORITY
    }


def exact_contributors(capability: Capability) -> FrozenSet[str]:
    return composition_contributor_map()[capability]


def resource_sources(request: Request) -> FrozenSet[str]:
    return frozenset(
        c.name for c in CLAIMS
        if c.capability.resource == request.resource
    )


def action_sources(request: Request) -> FrozenSet[str]:
    return frozenset(
        c.name for c in CLAIMS
        if c.capability.action == request.action
    )


def audience_sources(request: Request) -> FrozenSet[str]:
    return frozenset(
        c.name for c in CLAIMS
        if c.capability.audience == request.audience
    )


def temporal_sources(request: Request) -> FrozenSet[str]:
    return frozenset(
        c.name for c in CLAIMS
        if request.now < c.capability.expiry
    )


def matching_capabilities(request: Request) -> FrozenSet[Capability]:
    return frozenset(
        cap for cap in AUTHORITY
        if (
            cap.resource == request.resource
            and cap.action == request.action
            and cap.audience == request.audience
            and request.now < cap.expiry
        )
    )


def claim_failures(input_claims: FrozenSet[str]) -> list[str]:
    failures = []
    for name in input_claims:
        c = BY_NAME[name]
        if not (
            c.authorized
            and c.grant_backed
            and c.signer_trusted
            and c.subject == c.target
        ):
            failures.append("AllInputClaimsValid")
    return sorted(set(failures))


def composition_failures(decision: Decision) -> list[str]:
    failures = []
    if decision.authority != AUTHORITY:
        failures.append("CompositionAtomAndProvenanceExact")
    expected = composition_contributor_map()
    actual = dict(decision.composition_contributors)
    if actual != expected:
        failures.append("CompositionAtomAndProvenanceExact")
    for cap, contributors in actual.items():
        if cap not in AUTHORITY or not contributors:
            failures.append("CompositionAtomAndProvenanceExact")
    return sorted(set(failures))


def dimension_source_failures(decision: Decision) -> list[str]:
    expected = {
        "resource": resource_sources(decision.request),
        "action": action_sources(decision.request),
        "audience": audience_sources(decision.request),
        "temporal": temporal_sources(decision.request),
    }
    actual = {
        "resource": decision.resource_contributors,
        "action": decision.action_contributors,
        "audience": decision.audience_contributors,
        "temporal": decision.temporal_contributors,
    }
    failures = []
    for key in expected:
        if actual[key] != expected[key] or not actual[key]:
            failures.append("DecisionDimensionSourcesRemainValid")
    return sorted(set(failures))


def decision_failures(decision: Decision) -> list[str]:
    if not decision.authorized:
        return []
    matches = matching_capabilities(decision.request)
    if not matches:
        return ["DecisionRequiresSingleEffectiveAtom"]
    expected = frozenset().union(*(exact_contributors(cap) for cap in matches))
    if not expected or decision.contributors != expected:
        return ["DecisionRequiresSingleEffectiveAtom"]
    return []


def canonical_decision(request: Request) -> Decision:
    matches = matching_capabilities(request)
    contributors = frozenset().union(*(exact_contributors(cap) for cap in matches))
    return Decision(
        authority=AUTHORITY,
        input_claims=INPUT,
        composition_contributors=tuple(
            sorted(
                composition_contributor_map().items(),
                key=lambda item: (
                    item[0].resource,
                    item[0].action,
                    item[0].audience,
                    item[0].expiry,
                ),
            )
        ),
        request=request,
        authorized=bool(matches),
        contributors=contributors,
        resource_contributors=resource_sources(request),
        action_contributors=action_sources(request),
        audience_contributors=audience_sources(request),
        temporal_contributors=temporal_sources(request),
    )


def negative_decision(request: Request) -> Decision:
    synthetic_provenance = (
        resource_sources(request)
        | action_sources(request)
        | audience_sources(request)
        | temporal_sources(request)
    )
    return Decision(
        authority=AUTHORITY,
        input_claims=INPUT,
        composition_contributors=tuple(
            sorted(
                composition_contributor_map().items(),
                key=lambda item: (
                    item[0].resource,
                    item[0].action,
                    item[0].audience,
                    item[0].expiry,
                ),
            )
        ),
        request=request,
        authorized=True,
        contributors=synthetic_provenance,
        resource_contributors=resource_sources(request),
        action_contributors=action_sources(request),
        audience_contributors=audience_sources(request),
        temporal_contributors=temporal_sources(request),
    )


canonical = canonical_decision(Request("R1", "Write", "AudienceB", 3))
negative = negative_decision(Request("R1", "Read", "AudienceB", 3))

assert not claim_failures(INPUT), claim_failures(INPUT)
assert not composition_failures(canonical), composition_failures(canonical)
assert not dimension_source_failures(canonical), dimension_source_failures(canonical)
assert not decision_failures(canonical), decision_failures(canonical)

assert not claim_failures(negative.input_claims)
assert not composition_failures(negative), composition_failures(negative)
assert not dimension_source_failures(negative), dimension_source_failures(negative)
assert matching_capabilities(negative.request) == frozenset()
negative_failures = decision_failures(negative)
assert negative_failures == ["DecisionRequiresSingleEffectiveAtom"], negative_failures

print("CANONICAL PASS: exact effective context uses one full capability atom with exact provenance")
print("ISOLATION PASS: composition, claims, and all individual request dimensions remain valid")
print("NEGATIVE PASS: contextual decision laundering detected")
