# Candidate SPEC-IF-01 OAD → COS Interface Reference v1

**Status:** CandidateInterface / PublicDraftReference  
**Scope:** Engineering reference model only; not an Integral-ratified API.

## Source boundary

Integral's public Development Guide states that certified designs are version-locked and that COS verifies the certified version before production; uncertified designs are not allowed into production. The public Technical Specifications page identifies OAD→COS interface contract work as pending and calls out versioning, authentication, error handling, and retry behavior as unresolved Phase-0 work.

This artifact therefore does **not** claim to implement or represent a ratified Integral contract.

## Proposed semantic waist

The candidate interface separates five layers:

1. **Authentication** — establishes the communicating principal.
2. **Authorization** — establishes whether the principal may perform the requested operation.
3. **Transport** — delivery/acceptance of a message.
4. **Semantic admission** — COS accepts a specific OAD design envelope as the semantic input for the next transition.
5. **Production authority** — an independent explicit authorization/effect path.

The critical non-collapse rule is:

`transport acceptance != semantic admission != production authority`

A transport success therefore cannot by itself create a COS production basis or production authorization.

## Identity and retry model

Each logical delivery has:

- `logical_delivery_id`: stable across retries;
- `attempt_id`: unique per delivery attempt;
- `payload_commitment`: binds retries to the exact semantic payload;
- design identity/generation and schema version.

A retry is semantically compatible only when logical delivery identity, payload commitment, design identity, design generation, and schema version remain unchanged. The attempt identifier may change.

This follows the same safety objective used by mature API integration practice: repeated delivery should be deduplicated without re-applying side effects, and ambiguous mutation outcomes should not be silently treated as safe failures. These are engineering principles for this reference model, not claims about Integral's current implementation.

## Fail-closed cases

The reference model rejects or leaves indeterminate:

- missing/invalid authentication;
- missing/denied authorization;\n- missing or mismatched explicit authorization reference;
- stale schema;
- stale design generation;
- superseded design;
- uncertified design (classified as a certification failure, not an authorization failure);
- logical delivery identity mismatch;
- payload mutation under an existing logical delivery;
- transport acceptance without semantic admission;
- timeout/unknown delivery state.

An already semantically admitted logical delivery with the same payload is `DuplicateIdempotent`, not a second semantic effect.

## Provenance

The originating node is carried through the envelope and returned unchanged by recognition. Recognition does not rewrite foreign evidence as locally originated evidence.

## Error semantics

The model intentionally distinguishes:

- **Rejected** — a known contract or authorization violation;
- **Indeterminate** — insufficient evidence to decide the semantic state;
- **DuplicateIdempotent** — the same logical delivery was already admitted;
- **RejectedPayloadMutation** — a replay reused the logical identity with different content.

This prevents a timeout from becoming a fabricated failure and prevents a retry from silently changing the operation.

## Conformance matrix

| Scenario | Expected |
|---|---|
| Valid authn + valid authz + current certified design | eligible for semantic admission |
| Missing/invalid authn | rejected |
| Authn valid but authz absent/denied | rejected |
| Stale schema | rejected |
| Wrong design generation | rejected |
| Superseded design | rejected |
| Uncertified design | rejected |
| Transport accepted only | indeterminate |
| Semantic admission with wrong logical ID | rejected |
| Same logical ID + same payload | idempotent duplicate |
| Same logical ID + mutated payload | rejected |
| Timeout / unknown receipt | indeterminate |
| Foreign-origin recognition | origin preserved |
| Known recipient rejection | rejected, not indeterminate |\n| Any receipt | never mints production authority |

## Relationship to the existing Mycelix COS layer

The model is intentionally layered over the existing:

`SeamProfile -> SeamEnvelope -> Receipt -> OAD/COS admission -> COS projections`

It does not replace the neutral seam profile, ProductiveLoopV1 evidence semantics, Phase-0 draft adapters, or the COS formal obligations. Instead it gives SPEC-IF-01 a concrete candidate semantic contract that can later be refined to a ratified source schema if/when Integral publishes one.

## Formal refinement targets

The current formal tranche now also maps:

- IF01-FV-001: authentication != authorization;
- IF01-FV-002: stale schema cannot be admitted;
- IF01-FV-003: transport acceptance != semantic admission;
- IF01-FV-004: logical delivery identity is stable across retry;
- IF01-FV-005: payload mutation under stable logical identity is rejected;
- IF01-FV-006: duplicate semantic admission is idempotent;
- IF01-FV-007: indeterminate delivery is not definite failure;
- IF01-FV-008: foreign origin is preserved;
- IF01-FV-009: receipt does not mint production authority;\n- IF01-FV-010: certification failure is distinct from authorization failure;\n- IF01-FV-011: granted authorization requires a matching explicit reference;\n- IF01-FV-012: known recipient rejection is distinct from unknown delivery.

Each proof must still identify its production refinement path, assumptions, tool/version, executable witness, counterexample, and claim ceiling. An abstract proof remains an abstract proof until that refinement exists.

## Explicit nonclaims

This reference does not establish:

- Integral ratification or endorsement;
- an official Integral API;
- production authorization;
- manufacturing execution;
- physical delivery guarantees;
- security certification;
- economic, ecological, safety, or productivity outcomes;
- general system reliability from this test corpus alone.
