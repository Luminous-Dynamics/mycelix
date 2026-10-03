# SYM-CIVIC-008 — bundle completeness, deletion, replay, and monotonicity v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-007 / `e9874e38640d5678bf22aa2e663afce71cb870b0`

Tracking issue: #3865

## Purpose

Test collection-level provenance semantics for a bundle of individually authenticated attestations.

The current in-toto Bundle specification states that a bundle is not authenticated as a whole: individual attestations are authenticated, but valid attestations may be deleted, obsolete attestations replayed, or invalid/irrelevant attestations injected. Bundle processing is also order-independent, and consumers must ignore unrecognized lines.

This benchmark therefore separates:

`AttestationAuthenticity != BundleMembership != BundleCompleteness != BundleContentStatus != PolicyDecision != ScientificTruth != CivicAuthority`

An explicit membership manifest is treated as synthetic inclusion evidence. It is not claimed to be a production cryptographic commitment.

## Contract

A bundle is accepted as a complete provenance collection only when:

- recognized required attestations are present under exact immutable subject identity;
- obsolete or replayed evidence is not substituted for current required evidence;
- conflicting duplicate identities are preserved as distinct conflicts rather than collapsed;
- irrelevant recognized attestations are not silently selected as required evidence;
- unsupported/unrecognized predicates are ignored when policy says they are unrecognized, rather than being promoted into evidence;
- a completeness claim has an explicit immutable membership manifest;
- the manifest digest matches its declared entries and the observed recognized membership;
- changing bundle metadata or membership produces a new bundle identity;
- order permutations do not change a declared monotonic decision;
- stale downstream verification results do not survive removal of required evidence;
- historical verification binds to the exact historical membership manifest;
- an explicit empty bundle may be complete when its required membership set is explicitly empty.

## Typed dispositions

- `REJECT_BUNDLE_PROVENANCE`: membership, completeness, identity, replay, conflict, scope, or temporal semantics fail.
- `BUNDLE_CONTENT_UNQUALIFIED`: bundle provenance is intact, but a reported recognized property is explicitly uncertain or failed.
- `BUNDLE_ACCEPTED`: bundle provenance and completeness semantics pass without inferring truth or authority.
- `UNRECOGNIZED_IGNORED`: an unrecognized line is present but explicitly ignored and does not affect recognized membership semantics.

## Corpus

B-01 required attestation deleted
B-02 obsolete attestation replay
B-03 irrelevant recognized injection selected as evidence
B-04 conflicting duplicate attestation identity
B-05 authenticated attestation rebound to a different subject
B-06 order permutation changes declared monotonic result
B-07 unrecognized line explicitly ignored
B-08 unsupported predicate incorrectly accepted as evidence
B-09 partial bundle claimed complete
B-10 completeness claim lacks membership manifest
B-11 membership manifest digest mismatch
B-12 bundle metadata changed without new identity
B-13 stale verification survives required-evidence removal
B-14 monotonic property incorrectly treated as non-monotonic
B-15 conflicting attestations collapsed instead of preserved
B-16 exact immutable manifest and order-independent processing
B-17 explicit empty complete bundle
B-18 historical bundle uses exact historical manifest
B-19 unrecognized line incorrectly treated as evidence
B-20 complete bundle containing irrelevant recognized evidence that is not selected

The qualifier derives dispositions from semantic predicates; fixture files contain no expected verdicts.

## Qualification ceiling

PASS establishes only that the synthetic benchmark detects the declared bundle membership/completeness failures and preserves the stated order/monotonicity distinctions.

It does not establish production bundle security, publication-channel completeness, policy correctness, scientific truth, authorization legitimacy, or deployment readiness.

No runtime implementation is proposed.

## Standards alignment

This research profile follows the current in-toto Bundle semantics where practical: bundles are JSON Lines collections of attestations, bundle order must not affect processing, and unrecognized lines are ignored. The official specification explicitly warns that the bundle itself is not authenticated as a whole and discusses deletion, replay, and injection risks.

Reference:
- https://github.com/in-toto/attestation/blob/main/spec/v1/bundle.md
