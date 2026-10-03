# SYM-CIVIC-006 — provenance attestation authenticity v1

Status: synthetic research provenance qualification only

Parent: SYM-CIVIC-005 corrected qualification head / `78bfaaed4bd709306b3c49f6ecf644838152eb46`

Tracking issue: #3848

## Purpose

Test whether an authenticated provenance statement remains bound to the intended immutable subject and attester policy without silently promoting authentication into scientific truth or civic authority.

The conceptual separation is:

`SubjectIdentity != ProvenanceStatement != AttestationAuthenticity != ScientificTruth != CivicAuthority`

The benchmark uses the layered pattern of contemporary attestation systems: a statement binds subjects and predicate type; an envelope/authentication layer identifies and authenticates the attester; predicate content describes the asserted metadata. This is alignment research only, not a production cryptographic profile.

## Contract

An attestation is admissible only when:

- the signed subject identity is the exact immutable subject under review;
- the attester identity is known and permitted under the applicable policy;
- revocation, expiration, delegation scope, and historical key state are evaluated at explicit times;
- the signed predicate type matches the predicate type being interpreted;
- canonical serialization/signature bytes bind the same statement being verified;
- attestation timestamps cannot be backdated across key validity or subject history;
- replay against another study/result identity is rejected;
- multi-subject bindings are checked as the declared subject set rather than silently narrowed;
- the attestation envelope may authenticate a proposition that remains scientifically uncertain or false; authentication does not become truth.

Historical attestations may remain admissible after key rotation when the verification policy explicitly resolves the historical key epoch. Expired credentials may be accepted only when an explicit historical policy permits that condition.

## Adversarial corpus

A-01 subject digest mismatch
A-02 forged or unknown attester
A-03 revoked attester accepted after revocation
A-04 expired key accepted without policy basis
A-05 ambiguous key rotation
A-06 delegated attester exceeds scope
A-07 signed statement has the wrong predicate type
A-08 authenticated provenance statement whose scientific proposition is false
A-09 mutable locator substitutes for immutable subject identity
A-10 canonicalization/signature byte mismatch
A-11 timestamp backdating
A-12 attestation replay against a different study/result identity
A-13 valid exact immutable subject
A-14 valid delegated attestation within scope
A-15 historical attestation verified after key rotation
A-16 authenticated assertion with explicitly uncertain science
A-17 expired key accepted under explicit historical grace policy
A-18 multi-subject statement silently truncated to one subject
A-19 signed predicate altered after attestation
A-20 historical attestation verified after key expiry without explicit historical policy

The qualifier derives dispositions from the contract; fixture files do not contain expected verdicts.

## Qualification ceiling

PASS establishes only that this synthetic corpus distinguishes subject binding, attester authentication, and policy/time semantics from the truth-status of the asserted scientific content.

It does not establish production cryptographic security, secure key storage, protocol interoperability, scientific correctness, causal validity, civic legitimacy, authorization, or deployment readiness.

No runtime implementation is proposed.

## Standards alignment notes

This research profile is intentionally narrower than any production attestation or provenance implementation. Its temporal and binding semantics are informed by established provenance practice: W3C PROV defines validity using normalization plus uniqueness, event-ordering, typing, and impossibility constraints, and treats entity invalidation as a lifetime boundary. RO-Crate 1.3 is the current Recommendation; its provenance model uses CreateAction `object` for inputs and `result` for outputs, and retains prior file versions when representing changes. These references constrain the research vocabulary but do not make this synthetic qualifier standards-compliant.

References:
- https://www.w3.org/TR/prov-constraints/
- https://www.researchobject.org/ro-crate/specification/1.3/
