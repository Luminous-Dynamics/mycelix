# AC-050 — Finalization Certificate Self-Integrity

## Purpose

AC-050 makes the finalization certificate itself tamper-evident.

AC-049 already bound a certificate to a fresh finalization assessment and to
the ledger that recorded it. AC-050 adds a deterministic content fingerprint
over the certificate payload so mutation of issuer, evidence, timestamps, or
other certificate fields is detectable.

## Invariant

The certificate fingerprint covers every certificate field except the
fingerprint field itself:

- certificate identifier;
- action reference;
- lifecycle revision;
- scope identifier;
- scope fingerprint;
- AC-048 evidence snapshot fingerprint;
- authority reference;
- issuance evidence references;
- finalization timestamp.

A changed payload therefore cannot retain the old valid fingerprint.

## What this does not prove

A hash proves content continuity, not authorship.

AC-050 therefore does not claim that the authority reference is genuine, that
a signer possessed a private key, or that an external legal authority approved
the action.

This distinction matches the modern data-integrity model: transformation and
hashing establish the data being protected, while a separate cryptographic
proof mechanism establishes authenticity and proof purpose. W3C Data Integrity
1.0 describes this separation explicitly.

## Verification

Ledger validation now checks:

- certificate structural validity;
- valid scope and evidence snapshot fingerprints;
- valid certificate self-fingerprint;
- action identity matching the ledger;
- unique certificate ID;
- single-shot terminal cardinality.

Current-evidence verification still re-runs the AC-045/046/047/048 stack.

Thus there are two distinct questions:

1. Is this certificate exactly the certificate that was recorded?
2. Does the recorded certificate still correspond to current economic evidence?

AC-050 answers the first. AC-049 and AC-048 answer the second.

## Relationship to W3C Data Integrity

W3C Recommendation Data Integrity 1.0 describes data integrity proofs as
cryptographic mechanisms for authenticity and integrity, and describes a
general flow of transformation, hashing, and proof generation/verification.
AC-050 implements only the local content-hash portion of that conceptual
pipeline.

A future interoperability layer could add an actual Data Integrity proof or
another governed signature scheme without changing the certificate payload
semantics.

## Non-goals

AC-050 does not:

- replace digital signatures;
- select a universal cryptographic suite;
- introduce a cross-language canonical JSON standard;
- redefine legal finalization authority;
- permit certificate mutation after issuance.

## Tests

The reference tests cover:

- certificate fingerprint sensitivity to content changes;
- persisted certificate mutation being rejected by validation;
- fresh issuance carrying a valid self-fingerprint;
- stale assessment rejection;
- single-shot finalization;
- current-evidence verification after issuance.

## Research reference

W3C Verifiable Credential Data Integrity 1.0, Recommendation, 15 May 2025:
https://www.w3.org/TR/vc-data-integrity/

