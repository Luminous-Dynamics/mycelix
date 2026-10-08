#!/usr/bin/env python3
"""Reference semantics for evidence attestation trust and exact subject binding."""
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
class EvidenceTransition:
    name: str
    signature_valid: bool
    signer_trusted: bool
    subject: str
    target: str
    grants_before: tuple[str, ...]
    grants_after: tuple[str, ...]
    authority_before: tuple[tuple[str, tuple[str, ...]], ...]
    authority_after: tuple[tuple[str, tuple[str, ...]], ...]


@dataclass(frozen=True)
class State:
    grants: tuple[Grant, ...]
    authority: tuple[tuple[str, tuple[str, ...]], ...]
    evidence: tuple[EvidenceTransition, ...] = ()


def auth_map(state: State) -> dict[str, set[str]]:
    return {agent: set(powers) for agent, powers in state.authority}


def grant_authority(state: State) -> dict[str, set[str]]:
    result = {"Root": {"P1", "P2"}, "Alice": set(), "Bob": set()}
    for grant in state.grants:
        if grant.active and not grant.revoked:
            result[grant.grantee].add(grant.power)
    return result


def steady_state_failures(state: State) -> list[str]:
    amap = auth_map(state)
    derived = grant_authority(state)
    failures: list[str] = []
    if amap != derived:
        failures.append("AuthorityMatchesCurrentGrants")
    for grant in state.grants:
        if grant.active and grant.revoked:
            failures.append("ActiveGrantCurrent")
        if grant.active and grant.power not in amap.get(grant.issuer, set()):
            failures.append("GrantCannotExceedIssuerAuthority")
    for agent in ("Alice", "Bob"):
        for power in amap.get(agent, set()):
            if power not in derived.get(agent, set()):
                failures.append("NoAuthorityWithoutCurrentGrant")
    return sorted(set(failures))


def transition_failures(state: State) -> list[str]:
    failures: list[str] = []
    for evidence in state.evidence:
        if (
            evidence.signature_valid
            and not evidence.signer_trusted
            and evidence.authority_before != evidence.authority_after
        ):
            failures.append("UntrustedAttestationCannotChangeAuthority")
        if (
            evidence.signature_valid
            and evidence.signer_trusted
            and evidence.subject != evidence.target
            and evidence.authority_before != evidence.authority_after
        ):
            failures.append("SubjectMismatchCannotChangeAuthority")
    return sorted(set(failures))


def main() -> int:
    g1 = Grant("G1", "Root", "Alice", "P1", active=True)
    g2 = Grant("G2", "Root", "Alice", "P2")
    g3 = Grant("G3", "Root", "Bob", "P2")
    base = State(
        grants=(g1, g2, g3),
        authority=(("Root", ("P1", "P2")), ("Alice", ("P1",)), ("Bob", ())),
    )
    assert not steady_state_failures(base), steady_state_failures(base)

    valid = replace(
        base,
        grants=(g1, replace(g2, active=True), g3),
        authority=(("Root", ("P1", "P2")), ("Alice", ("P1", "P2")), ("Bob", ())),
        evidence=(EvidenceTransition(
            "E1", True, True, "Alice", "Alice",
            ("G1",), ("G1", "G2"),
            base.authority,
            (("Root", ("P1", "P2")), ("Alice", ("P1", "P2")), ("Bob", ())),
        ),),
    )
    assert not steady_state_failures(valid), steady_state_failures(valid)
    assert not transition_failures(valid), transition_failures(valid)

    bad_untrusted = replace(
        valid,
        evidence=(replace(valid.evidence[0], signer_trusted=False),),
    )
    assert not steady_state_failures(bad_untrusted), steady_state_failures(bad_untrusted)
    assert transition_failures(bad_untrusted) == ["UntrustedAttestationCannotChangeAuthority"]

    bad_subject = replace(
        base,
        grants=(g1, g2, replace(g3, active=True)),
        authority=(("Root", ("P1", "P2")), ("Alice", ("P1",)), ("Bob", ("P2",))),
        evidence=(EvidenceTransition(
            "E1", True, True, "Bob", "Alice",
            ("G1",), ("G1", "G3"),
            base.authority,
            (("Root", ("P1", "P2")), ("Alice", ("P1",)), ("Bob", ("P2",))),
        ),),
    )
    assert not steady_state_failures(bad_subject), steady_state_failures(bad_subject)
    assert transition_failures(bad_subject) == ["SubjectMismatchCannotChangeAuthority"]

    print("CANONICAL PASS: trusted, subject-bound evidence may accompany a grant-backed authority transition")
    print("ISOLATION PASS: untrusted signer leaves steady-state grant provenance valid")
    print("NEGATIVE PASS: untrusted attestation authority delta detected")
    print("ISOLATION PASS: subject substitution leaves steady-state grant provenance valid")
    print("NEGATIVE PASS: subject-mismatch authority delta detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
