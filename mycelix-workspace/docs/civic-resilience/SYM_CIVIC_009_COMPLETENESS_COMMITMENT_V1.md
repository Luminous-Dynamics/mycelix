# SYM-CIVIC-009 — authenticated completeness commitments v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-008 / `e162ffb107d711e63002d414242d310507aaa4ca`

Tracking issue: #3867

## Purpose

Test whether an authenticated completeness commitment is itself correctly bound to a specific bundle and membership manifest.

This layer is deliberately narrower than authenticating a bundle as a whole. The current in-toto Bundle specification states that the bundle itself is not authenticated as a whole; individual attestations carry authentication, while deletion, replay, and injection remain collection-level threats.

The research separation is:

`AttestationAuthenticity != BundleMembership != CompletenessCommitmentAuthenticity != BundleCompleteness != ExternalExhaustiveness != PolicyDecision != ScientificTruth != CivicAuthority`

## Contract

A completeness commitment is accepted only when:

- the commitment names the exact immutable bundle digest under review;
- the referenced manifest has an immutable digest and its actual bytes match that digest;
- the commitment's signed manifest digest matches the evaluated manifest digest;
- mutable `latest` manifest locators are not treated as immutable identity;
- the commitment attester is explicitly known and active under this synthetic policy;
- the commitment timestamp is consistent with the bundle's declared creation/update window;
- a membership change creates a new commitment identity;
- the commitment scope exactly matches the bundle scope it claims to cover;
- historical commitments bind to the exact historical manifest version;
- conflicting commitments remain distinct instead of being collapsed;
- authentication of the commitment is not interpreted as proof that an external publication channel is exhaustive.

A commitment may truthfully authenticate a snapshot while explicitly stating that external exhaustiveness is not established.

## Typed dispositions

- `REJECT_COMPLETENESS_COMMITMENT`: subject, manifest, attester, scope, time, history, identity, or interpretation binding fails.
- `COMMITMENT_CONTENT_UNQUALIFIED`: the commitment is authenticated but its stated completeness property is explicitly uncertain.
- `COMMITMENT_ACCEPTED`: the exact commitment is authenticated and properly scoped, without inferring external exhaustiveness, scientific truth, or civic authority.

## Corpus

C-01 bundle digest mismatch
C-02 unknown commitment attester
C-03 mutable latest manifest locator
C-04 manifest digest mismatch
C-05 signed manifest differs from evaluated manifest
C-06 commitment timestamp outside bundle update window
C-07 stale commitment reused after membership change
C-08 bundle metadata changed without commitment identity change
C-09 commitment authentication incorrectly treated as external exhaustiveness proof
C-10 commitment scope narrower than claimed bundle scope
C-11 historical commitment bound to wrong manifest version
C-12 conflicting commitments collapsed
C-13 authenticated but uncertain completeness property
C-14 exact bundle/manifest/attester binding
C-15 valid historical commitment with exact historical manifest
C-16 valid scoped commitment with explicit scope boundary
C-17 authenticated snapshot that explicitly does not claim external exhaustiveness

The qualifier derives dispositions from semantic predicates; fixture files contain no expected verdicts.

## Qualification ceiling

PASS establishes only that this synthetic benchmark distinguishes authenticated membership commitments from actual external publication exhaustiveness.

It does not establish whole-bundle cryptographic authentication, publication-channel completeness, policy correctness, scientific truth, authorization legitimacy, or deployment readiness.

No runtime implementation is proposed.

## Standards alignment

The current in-toto Bundle specification explicitly says the bundle is not authenticated as a whole and identifies deletion, replay, and injection risks. It also requires order-independent processing and ignoring unrecognized lines. The commitment model here is therefore an additional research construct for making a completeness statement provenance-bearing; it is not claimed as an in-toto standard.

Reference:
- https://github.com/in-toto/attestation/blob/main/spec/v1/bundle.md
