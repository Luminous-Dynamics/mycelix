# Mycelix Protocol Commons — PROTO-000 through PROTO-003

Status: implementation candidate; not yet qualified.

## Purpose

This profile establishes a thin constitutional interoperability waist for Mycelix. It does not create protocol governance authority, registry legitimacy, certification, end-to-end security, or authority to act. It establishes only exact protocol-profile identity and declared semantic compatibility.

The core theorem is:

> Interoperability infrastructure must not silently become governance infrastructure.

A second theorem constrains composition:

> Wire compatibility does not establish semantic compatibility, semantic compatibility does not establish authority compatibility, and compatibility in one direction does not establish compatibility in the reverse direction.

## PROTO-000 — Protocol Commons Constitution

The protocol commons is coordination infrastructure, not a sovereign authority plane.

The following do not by themselves create political, legal, institutional, physical, emergency, or execution authority:

- authorship of a protocol or extension;
- ownership of the reference implementation;
- control of a registry or namespace;
- passing a conformance suite;
- ecosystem adoption, market share, stake, reputation, or implementation count;
- successful negotiation of a protocol profile.

Protocol negotiation is evidence only about declared semantic overlap. It is not proof that either peer is conformant, trustworthy, secure, authorized, current, or truthful about its advertisement.

## PROTO-001 — Exact Profile Identity

`ProtocolProfileV1` binds:

- protocol family;
- exact major/minor/patch version;
- capabilities;
- semantic descriptors;
- security-suite identifiers.

Its `profile_id()` is a domain-separated SHA-256 commitment over a canonical ordering of set-like fields. Human-readable family/version labels are not sufficient identity.

Each semantic has:

- a stable semantic identifier;
- a semantic class;
- critical/non-critical status;
- declared behavior when the semantic is unknown.

Semantic classes in v1 are presentation, advisory, evidence, privacy constraint, authority constraint, safety constraint, lifecycle constraint, and effect constraint.

A critical semantic may not declare `ignore` as its unknown behavior.

## PROTO-002 — Directional Compatibility and Negotiation

Compatibility is directional.

For sender A and receiver B, `assess_directional_compatibility(A, B)` asks whether B can interpret the exact semantic descriptors emitted by A. The reverse direction is evaluated independently.

The claim ceiling contains only the exact semantic identifiers and semantic classes whose descriptors match in that direction. Missing or conflicting semantics are never silently promoted into the ceiling.

`ProtocolOfferV1` contains a preferred profile and zero or more alternatives. `negotiate_protocol()` searches same-family, same-major candidates and chooses the highest compatible pair while preserving explicit degradation.

A version/profile mismatch is surfaced as degradation rather than being represented as full qualification.

## PROTO-003 — Unknown-Critical and Downgrade Semantics

Unknown non-critical semantics may produce explicit degraded operation. Unknown critical semantics refuse the affected direction. Reuse of the same semantic identifier with a conflicting descriptor also refuses that direction.

Downgrade is monotone with respect to protected critical semantics:

> Compatibility pressure may narrow capabilities or claim ceilings, but it may not silently remove critical privacy, authority, safety, lifecycle, or effect constraints.

The current v1 negotiation automatically protects critical semantics in those classes from being removed by selection of a fallback profile. A candidate that would drop one is not selected.

This does not establish that every desirable semantic is already classified as critical or protected. That is profile governance work outside this tranche.

## Failure semantics

The important outcomes are:

- `Qualified`: exact declared semantics needed by each direction are understood, with no negotiated degradation;
- `Degraded`: operation remains representable, but version mismatch and/or unknown non-critical semantics narrow what may be claimed;
- `Refused`: a critical semantic is unknown, a semantic identifier conflicts, or protected critical semantics would be lost by downgrade.

`Refused` is a protocol-qualification result. It does not itself mean a physical service must stop. Operational halt/continue behavior remains governed by the relevant safety, continuity, authority, and effect policies.

## Explicit non-claims

This tranche does **not** establish:

- independent implementation conformance;
- registry or extension-governance legitimacy;
- security of advertised cryptographic suites;
- authenticity of a peer's offer;
- protocol-governance independence;
- reference-implementation correctness;
- full privacy, safety, or authority correctness;
- interoperability beyond the exact negotiated claim ceiling.

Those are intentionally left for later qualified tranches rather than inferred from a successful handshake.

## Qualification targets

The initial executable tests cover:

1. profile identity is invariant to ordering of set-like fields;
2. critical semantics cannot be configured to ignore unknown meaning;
3. compatibility can differ by direction;
4. unknown non-critical semantics explicitly lower the claim ceiling;
5. conflicting descriptors under the same semantic identifier refuse compatibility;
6. fallback negotiation cannot drop protected critical privacy/authority/safety/lifecycle/effect semantics;
7. non-critical version mismatch is represented as degraded rather than fully qualified.

A test existing in source is not a PASS claim. Qualification requires an actual exact-head execution of the relevant formatter/compiler/test/lint/evidence workflow.
