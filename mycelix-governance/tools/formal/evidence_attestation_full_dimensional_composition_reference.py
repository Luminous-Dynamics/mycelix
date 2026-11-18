#!/usr/bin/env python3
"""Reference semantics for full-dimensional capability composition."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product


@dataclass(frozen=True)
class Capability:
    resource: str
    action: str
    audience: str
    expiry: str


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
class Composition:
    input_claims: frozenset[str]
    authority: frozenset[Capability]
    contributor_claims: tuple[tuple[Capability, frozenset[str]], ...]
    resource_contributors: tuple[tuple[Capability, frozenset[str]], ...]
    action_contributors: tuple[tuple[Capability, frozenset[str]], ...]
    audience_contributors: tuple[tuple[Capability, frozenset[str]], ...]
    expiry_contributors: tuple[tuple[Capability, frozenset[str]], ...]


CLAIMS = (
    Claim("Claim1", Capability("R1", "Read", "AudienceA", "T1")),
    Claim("Claim2", Capability("R2", "Write", "AudienceA", "T2")),
    Claim("Claim3", Capability("R1", "Write", "AudienceB", "T2")),
    Claim("Claim4", Capability("R2", "Read", "AudienceB", "T1")),
)

HYBRID = Capability("R1", "Write", "AudienceB", "T1")


def as_map(items):
    return dict(items)


def capability_universe():
    return {
        Capability(r, a, u, t)
        for r, a, u, t in product(
            ("R1", "R2"),
            ("Read", "Write"),
            ("AudienceA", "AudienceB"),
            ("T1", "T2"),
        )
    }


def expected_authority(input_claims):
    return {
        claim.capability
        for claim in CLAIMS
        if claim.name in input_claims
    }


def expected_exact_contributors(input_claims):
    authority = capability_universe()
    return {
        capability: frozenset(
            claim.name
            for claim in CLAIMS
            if claim.name in input_claims and claim.capability == capability
        )
        for capability in authority
    }


def expected_dimension_contributors(input_claims, dimension):
    authority = capability_universe()

    def selected(claim, capability):
        return {
            "resource": claim.capability.resource == capability.resource,
            "action": claim.capability.action == capability.action,
            "audience": claim.capability.audience == capability.audience,
            "expiry": claim.capability.expiry == capability.expiry,
        }[dimension]

    return {
        capability: frozenset(
            claim.name
            for claim in CLAIMS
            if claim.name in input_claims and selected(claim, capability)
        )
        for capability in authority
    }


def input_claim_failures(input_claims):
    failures = []
    by_name = {claim.name: claim for claim in CLAIMS}
    for name in input_claims:
        claim = by_name[name]
        if not (
            claim.authorized
            and claim.grant_backed
            and claim.signer_trusted
            and claim.subject == claim.target
        ):
            failures.append("AllInputClaimsValid")
    return sorted(set(failures))


def dimension_source_failures(comp):
    failures = []
    input_claims = {claim.name for claim in CLAIMS if claim.name in comp.input_claims}
    maps = {
        "resource": as_map(comp.resource_contributors),
        "action": as_map(comp.action_contributors),
        "audience": as_map(comp.audience_contributors),
        "expiry": as_map(comp.expiry_contributors),
    }
    by_name = {claim.name: claim for claim in CLAIMS}

    for capability in comp.authority:
        for dimension, mapping in maps.items():
            actual = mapping.get(capability, frozenset())
            expected = expected_dimension_contributors(comp.input_claims, dimension)[capability]
            if actual != expected:
                failures.append("DimensionSourcesRemainIndividuallyValid")
            for name in actual:
                claim = by_name[name]
                if name not in input_claims:
                    failures.append("DimensionSourcesRemainIndividuallyValid")

    return sorted(set(failures))


def composition_failures(comp):
    expected_authority = expected_authority_for(comp.input_claims)
    expected_contributors = expected_exact_contributors(comp.input_claims)
    contributors = as_map(comp.contributor_claims)
    failures = []

    for capability in comp.authority:
        if capability not in expected_authority:
            failures.append("CompositionAtomAndProvenanceExact")
        if contributors.get(capability, frozenset()) != expected_contributors[capability]:
            failures.append("CompositionAtomAndProvenanceExact")
        if not contributors.get(capability, frozenset()):
            failures.append("CompositionAtomAndProvenanceExact")

    return sorted(set(failures))


def expected_authority_for(input_claims):
    return {
        claim.capability
        for claim in CLAIMS
        if claim.name in input_claims
    }


INPUT = frozenset(claim.name for claim in CLAIMS)
EXACT_AUTHORITY = frozenset(claim.capability for claim in CLAIMS)
EXACT_CONTRIBUTORS = tuple(
    (capability, contributors)
    for capability, contributors in sorted(
        expected_exact_contributors(INPUT).items(),
        key=lambda item: (
            item[0].resource,
            item[0].action,
            item[0].audience,
            item[0].expiry,
        ),
    )
    if contributors
)
EXACT_RESOURCES = tuple(
    (capability, contributors)
    for capability, contributors in sorted(
        expected_dimension_contributors(INPUT, "resource").items(),
        key=lambda item: (
            item[0].resource,
            item[0].action,
            item[0].audience,
            item[0].expiry,
        ),
    )
    if contributors
)
EXACT_ACTIONS = tuple(
    (capability, contributors)
    for capability, contributors in sorted(
        expected_dimension_contributors(INPUT, "action").items(),
        key=lambda item: (
            item[0].resource,
            item[0].action,
            item[0].audience,
            item[0].expiry,
        ),
    )
    if contributors
)
EXACT_AUDIENCES = tuple(
    (capability, contributors)
    for capability, contributors in sorted(
        expected_dimension_contributors(INPUT, "audience").items(),
        key=lambda item: (
            item[0].resource,
            item[0].action,
            item[0].audience,
            item[0].expiry,
        ),
    )
    if contributors
)
EXACT_EXPIRIES = tuple(
    (capability, contributors)
    for capability, contributors in sorted(
        expected_dimension_contributors(INPUT, "expiry").items(),
        key=lambda item: (
            item[0].resource,
            item[0].action,
            item[0].audience,
            item[0].expiry,
        ),
    )
    if contributors
)


def main():
    canonical = Composition(
        input_claims=INPUT,
        authority=EXACT_AUTHORITY,
        contributor_claims=EXACT_CONTRIBUTORS,
        resource_contributors=EXACT_RESOURCES,
        action_contributors=EXACT_ACTIONS,
        audience_contributors=EXACT_AUDIENCES,
        expiry_contributors=EXACT_EXPIRIES,
    )

    assert not input_claim_failures(canonical.input_claims), input_claim_failures(canonical.input_claims)
    assert not dimension_source_failures(canonical), dimension_source_failures(canonical)
    assert not composition_failures(canonical), composition_failures(canonical)

    hybrid_contributors = tuple(
        list(EXACT_CONTRIBUTORS)
        + [(HYBRID, frozenset({"Claim1", "Claim2", "Claim3", "Claim4"}))]
    )
    hybrid_resources = tuple(
        list(EXACT_RESOURCES)
        + [(HYBRID, frozenset({"Claim1", "Claim3"}))]
    )
    hybrid_actions = tuple(
        list(EXACT_ACTIONS)
        + [(HYBRID, frozenset({"Claim2", "Claim3"}))]
    )
    hybrid_audiences = tuple(
        list(EXACT_AUDIENCES)
        + [(HYBRID, frozenset({"Claim3", "Claim4"}))]
    )
    hybrid_expiries = tuple(
        list(EXACT_EXPIRIES)
        + [(HYBRID, frozenset({"Claim1", "Claim4"}))]
    )

    negative = Composition(
        input_claims=INPUT,
        authority=EXACT_AUTHORITY | {HYBRID},
        contributor_claims=hybrid_contributors,
        resource_contributors=hybrid_resources,
        action_contributors=hybrid_actions,
        audience_contributors=hybrid_audiences,
        expiry_contributors=hybrid_expiries,
    )

    assert not input_claim_failures(negative.input_claims)
    assert not dimension_source_failures(negative), dimension_source_failures(negative)
    failures = composition_failures(negative)
    assert failures == ["CompositionAtomAndProvenanceExact"], failures

    print("CANONICAL PASS: exact four-dimensional capability atoms compose with exact contributor provenance")
    print("ISOLATION PASS: all component claims and per-dimension sources remain independently valid")
    print("NEGATIVE PASS: full-dimensional synthetic capability/provenance amplification detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
