# SYM-CIVIC-012 — decision finality and currentness boundary v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-011 / 31383e903499a5614bb7b664139b249235d0be78

Tracking: research continuation after PR #3872

## Purpose

Qualify the boundary between an already-qualified PolicyDecision and any later claim that the decision is final, supersession-resistant, revoked, or currently actionable as a finalized result.

The separation is:

PolicyDecision != DecisionFinality != Currentness != Authorization != CivicAuthority

A decision can be provenance-valid and historically reproducible without being currently final. A current-finality assertion is itself a separate, bounded, authority-bound evidence object; it does not authorize an external effect.

## Standards basis

The current in-toto Attestation Framework is v1.2. It separates predicate, statement, envelope, and bundle layers, and its parsing rules emphasize monotonic policy consumption. Its Bundle specification also says the bundle is not authenticated as a whole, warns about deletion, replay, and injection, and requires order-independent processing.

Those specifications do not define a universal civic DecisionFinality or Currentness algebra. This tranche therefore remains a synthetic research model and explicitly does not claim that in-toto or SVR supplies finality semantics.

References:
- https://github.com/in-toto/attestation/blob/main/spec/v1/README.md
- https://github.com/in-toto/attestation/blob/main/spec/v1/bundle.md
- https://github.com/in-toto/attestation/blob/main/spec/predicates/svr.md

## Contract

A finality/currentness result is admissible only when:

- the underlying decision is itself internally identity-consistent;
- finality binds to the exact decision identifier and exact semantic decision identity digest;
- finality uses an exact, non-latest authority profile;
- the issuing principal is allowed by that exact authority profile;
- finality has a bounded interval contained within the decision validity interval and the authority profile interval;
- assertion time does not precede the underlying decision evaluation;
- REVOKED and SUPERSEDED transitions use a new assertion identity and explicitly reference the prior assertion;
- a finality record semantic identity is itself committed by a canonical identity digest;
- a finality identifier cannot be silently reused for changed semantic content;
- an empty finality set does not create current finality;
- a finality assertion outside its interval is historical/non-current rather than silently current;
- conflicting applicable finality states remain FINALITY_UNRESOLVED when conflict preservation is explicitly declared;
- assertion ordering cannot affect the result, including lineage-reference resolution;
- a successor finality does not transfer backward to its predecessor;
- revocation or supersession does not mutate the historical decision receipt;
- finality cannot widen the validity horizon of the underlying decision;
- mutable external latest state cannot substitute for exact finality evidence;
- finality is never promoted into authorization or civic authority.

## Semantic identity

Decision identity is inherited exactly from SYM-CIVIC-011 through decision.identity_digest.

Finality identity is:

- finality assertion ID;
- decision ID;
- decision semantic identity digest;
- issuer;
- exact authority profile reference and digest;
- asserted-at time;
- validity interval;
- status;
- successor and predecessor relations.

The qualifier canonicalizes that material and verifies finality.identity_digest.

This creates two distinct commitments:

decision_identity_digest != finality_identity_digest

Changing either semantic object requires its own identity transition. Refreshing currentness evidence therefore does not mutate the underlying decision identity.

## Typed dispositions

- REJECT_FINALITY_PROVENANCE: finality, decision, authority identity, temporal, transition, or authority-boundary failure.
- FINALITY_CURRENT: an exact FINAL assertion is applicable at the requested currentness time.
- FINALITY_NOT_CURRENT: no applicable final assertion exists, or the applicable state is revoked, superseded, expired, or outside its bounded interval.
- FINALITY_UNRESOLVED: multiple applicable finality states conflict and are preserved as unresolved.

These dispositions do not authorize an external action.

## Temporal rule

The qualifier keeps historical reconstruction separate from currentness:

historical finality at T1 != current finality at T2

A later expiry, revocation, or supersession changes the currentness answer without rewriting the historical assertion or underlying decision identity.

## Corpus

F-01 exact current finality
F-02 missing finality evidence
F-03 finality bound to another decision ID
F-04 finality bound to another decision identity
F-05 mutable finality status
F-06 finality horizon wider than the decision horizon
F-07 untrusted finality issuer
F-08 mutable latest authority profile
F-09 finality asserted before decision evaluation
F-10 currentness check after finality expiry
F-11 explicit revocation transition
F-12 explicit supersession transition
F-13 successor finality does not transfer to predecessor
F-14 conflicting applicable finality states
F-15 multiple equivalent assertions remain order-independent
F-16 exact finality replay is idempotent
F-17 reused finality ID with changed semantic content
F-18 finality promoted to authorization
F-19 finality promoted to civic authority
F-20 historical finality is not current after expiry
F-21 mutable latest registry state substituted for exact evidence

The corpus contains no expected-disposition field. The qualifier derives dispositions and separately computes an independent reference outcome.

## Metamorphic probes

The qualifier independently verifies:

- decision identity mutation -> provenance rejection;
- finality decision binding mutation -> provenance rejection;
- issuer mutation -> provenance rejection;
- authority-profile mutation -> provenance rejection;
- horizon widening -> provenance rejection;
- currentness after expiry -> not current;
- valid revocation transition -> not current;
- valid supersession transition -> not current;
- preserved finality conflict -> unresolved;
- assertion order permutation -> identical current result;
- lineage references resolve by exact assertion ID, never by array position;
- permuting a multi-assertion lineage witness preserves its currentness result;
- finality identity digest tamper -> provenance rejection;
- exact replay -> identical current result;
- reused finality ID after semantic change -> provenance rejection;
- finality-to-authorization promotion -> provenance rejection;
- finality-to-civic-authority promotion -> provenance rejection;
- mutable latest-registry substitution -> provenance rejection.

## Qualification ceiling

PASS establishes only that this synthetic benchmark preserves explicit decision/finality separation, semantic identity binding, bounded finality horizons, immutable transition lineage, historical-versus-current time semantics, conflict preservation, and authority separation.

It does not establish legal finality, judicial finality, civic legitimacy, operational authorization, effect safety, backend atomicity, or production security.

No runtime implementation is proposed.
