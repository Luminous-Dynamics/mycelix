# Integral D6R — Semantic Conservation and Non-Amplification

Status: **ReferenceModelOnly**

## Purpose

D6R makes the evidence/claim-graph layer explicitly non-amplifying.

The governing law is:

    derived semantic authority <= qualified authority of exact inputs

A derived artifact may preserve or narrow the claim carried by its exact inputs. It may not silently strengthen:

- semantic scope;
- currentness;
- claim ceiling;
- authority;
- or the provenance of a claim.

D6R is deliberately not a confidence score, trust score, reputation score, or truth estimator.

## Existing primitives are wrapped, not replaced

D6R consumes existing artifacts:

- D6P CurrentFinalityEligibilityReceiptV1 for current-finality eligibility;
- D6Q ClaimGraphAssessmentReceiptV1 for evidence/claim-graph assessment.

It does not create a parallel observer-confidence or epistemic-ranking system.

The D6P adapter maps:

    EligibleCurrent -> CurrentQualifiedEvidence + Current

Any other D6P disposition becomes:

    Unresolved + Unknown

Therefore lifecycle-invalidated D6P evidence cannot be promoted back to current evidence by a later wrapper.

The D6Q adapter maps the graph assessment to:

    Assessment

and deliberately does not infer currentness from graph reachability, human disposition, or serialization. A D6Q assessment therefore cannot silently manufacture a current-finality claim.

## Exact input binding

Every derivation profile names its authority-bearing, currentness-bearing, and scope-bearing inputs.

The resulting receipt records an exact:

    input_id -> input_commitment

map.

Changing an input commitment without changing the derivation receipt is rejected.

Changing the serialization while retaining the same exact claim commitment cannot amplify the claim.

This prevents:

- evidence substitution;
- stale receipt replay;
- copied-input promotion;
- hidden input omission;
- conclusion promotion by wrapper identity.

## Claim ceiling

SemanticClaimCeilingV1 is a finite claim ceiling, not a confidence score:

    Unresolved
    HistoricalEvidence
    CurrentQualifiedEvidence
    Assessment
    Conclusion

A derived claim must be no stronger than:

1. the minimum ceiling of its exact authority-bearing inputs; and
2. the derivation profile's maximum output ceiling.

Thus a D6P current-finality eligibility receipt cannot be transformed into an Assessment merely by serializing it through D6Q/D6R.

A D6Q assessment cannot become a Conclusion merely because a human disposition is AcceptedForHumanUse.

Authorization is intentionally outside this ceiling algebra.

## Currentness conservation

SemanticCurrentnessV1 is:

    Unknown
    Historical
    Current

Currentness is checked against the exact profile-selected currentness inputs.

Therefore:

    Historical -> Current    blocked
    Unknown    -> Current    blocked
    Current    -> Current    allowed
    Current    -> Historical  allowed
    Current    -> Unknown     allowed

A current claim requires a current qualified input. A historical or unresolved input may narrow a claim, but cannot establish currentness.

## Scope conservation

The derivation declares a scope relation:

- Exact;
- Narrowed;
- Broadened;
- Unknown.

Only Exact and explicitly witnessed Narrowed derivations are accepted.

Broadened and Unknown are fail-closed.

The reference model does not pretend an opaque commitment proves arbitrary set inclusion. A narrowed derivation therefore requires an explicit scope-narrowing witness commitment. That witness is itself evidence requiring whatever downstream qualification the application gives it.

## Missing evidence

Missing or unresolved inputs produce:

    InsufficientEvidence

rather than a semantic rejection.

This preserves the distinction between:

    the claim is contradicted

and:

    the evidence required to assess the claim is not available.

## Authority role boundary

Only inputs explicitly typed AuthorityBearing may constrain the output claim ceiling.

Supporting, Context, and Provenance inputs cannot be promoted to authority by a derivation profile.

This prevents:

    provenance -> causal support
    custody -> authority
    human disposition -> evidence
    serialization -> endorsement

without requiring a new trust score.

## D6N preservation

D6R does not replace D6N's contradiction semantics.

D6N already determines whether observations are corroborating, contradictory, dependent, stale, or otherwise insufficient. D6P then determines whether corroborating independent observations remain lifecycle-eligible.

D6R consumes the resulting qualified artifacts; it does not re-run D6N conflict resolution or choose winners by:

- traversal order;
- observer count;
- arrival order;
- recency;
- majority;
- or serialization order.

## Symthaea boundary

Symthaea may:

- propose derivations;
- identify missing inputs;
- detect possible amplification;
- compare candidate derivation receipts;
- recommend a narrower scope.

Symthaea may not:

- mint an authority-bearing input;
- change an input commitment;
- turn historical evidence current;
- widen scope;
- raise a claim ceiling;
- convert a human disposition into evidence;
- convert a conclusion into authorization.

Mycelix remains the semantic root.

Xenia remains responsible for cryptographic mechanisms and verification rather than semantic promotion.

## Actuation boundary

D6R produces qualification evidence only.

A Conclusion ceiling is still not authorization.

The module provides an explicit helper:

    semantic_conclusion_cannot_create_authorization

Authorization and actuation require separate qualified semantic transitions.

## Adversarial corpus

The source-level tests cover:

1. exact D6P current-witness input binding;
2. lifecycle-invalidated D6P evidence cannot become current;
3. input commitment substitution;
4. claim-ceiling amplification;
5. currentness amplification;
6. scope widening;
7. explicit scope narrowing;
8. unknown scope fail-closed behavior;
9. missing evidence as insufficient rather than rejection;
10. semantic-environment drift;
11. D6Q assessment cannot mint currentness;
12. serialization/replay cannot amplify a claim;
13. Symthaea proposal cannot become authority;
14. conclusion cannot create authorization;
15. provenance/supporting input cannot become authority;
16. exact conserved derivation;
17. D6P and D6Q artifacts retain distinct semantic identities.

## Formal invariants

For every accepted derivation d:

    output_commitment == exact declared claim commitment

    output_ceiling <= min(authority_input_ceiling)

    output_ceiling <= profile.max_output_claim_ceiling

    output_currentness <= min(selected_currentness_input_currentness)

    output_scope in {exact, explicitly-witnessed-narrowing}

    missing_input => insufficient_evidence

    provenance/supporting input cannot become authority

    conclusion != authorization

These are deterministic reference-model relations, not claims about external reality.

## Claim ceiling

**ReferenceModelOnly.**

D6R does not establish:

- source truth;
- physical truth;
- observer trust;
- cryptographic authenticity;
- legal authority;
- production consensus;
- production storage durability;
- production finality;
- economic settlement;
- actuation safety;
- or real-world causality.

Source-level tests are authored evidence. They become execution evidence only when a functioning CI or independent local execution produces a verifiable receipt.
