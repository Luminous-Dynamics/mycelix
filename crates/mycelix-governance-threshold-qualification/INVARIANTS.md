# Governance Threshold Qualification v0.1

Status: **normative pure semantic kernel**

This crate does not define another committee model. It consumes the existing normative types from `mycelix-governance-authority`.

## Question answered

The kernel answers only:

> Given an exact ProposalAuthorityContext, an independently institution-verified SigningPolicy, and independently cryptographically verified ThresholdVerificationEvidence, does this threshold authorization qualify for this proposal/action domain now?

It does not create:

- a signing policy;
- committee membership;
- a DKG ceremony;
- a signature;
- an execution nonce;
- an ExecutionAttempt;
- an executor designation;
- a lifecycle claim; or
- effect permission.

## Verified policy wrapper

A `VerifiedSigningPolicy` binds:

- the exact `SigningPolicy`;
- a host-verified policy digest equal to `policy.policy_digest`;
- the exact institutional policy-proof reference named by the policy;
- immutable policy-record identity;
- independent institutional verification provenance; and
- non-future verification time.

A non-empty `policy_proof_ref` inside the policy is not sufficient by itself.

## Verified cryptographic evidence wrapper

A `VerifiedThresholdEvidence` binds:

- exact `ThresholdVerificationEvidence`;
- exact signature-record reference;
- exact cryptographic-proof reference;
- exact independently verified committee-key digest;
- cryptographic verification provenance; and
- non-future verification time no earlier than the evidence's own verification time.

The legacy `ThresholdSignature.verified` boolean is irrelevant to this kernel.

## Cross-field qualification

Qualification requires exact agreement on:

- proposal;
- institution / jurisdiction / rulebook / governing body;
- signing-policy ID + digest;
- permitted action class;
- exact committee ID;
- exact committee verification-key digest;
- exact epoch;
- threshold/member-count constraints;
- allowed signature algorithm;
- PQ requirement;
- exact executable action digest;
- signature/proof references; and
- authority/verification time windows.

The signing policy's own validation retains the protocol-level majority floor, so a malformed 1-of-5 governance policy cannot qualify.

## Output

`QualifiedThresholdAuthorization` is an authorization fact for later composition. It contains no nonce.

Its `valid_until_ms` is bounded by the earlier of proposal-authority expiry and signing-policy expiry. A host may impose an even earlier revocation/lease boundary.

The lifecycle/execution layer must still create its own real claim/attempt under fresh authority.

## Host responsibilities

The pure crate cannot prove that a host receipt corresponds to real institutional or cryptographic verification. Runtime adapters MUST independently verify:

- institutional adoption/proof chain for SigningPolicy;
- policy canonical digest;
- committee-key material/digest;
- threshold-signature cryptography;
- revocation/replacement generations; and
- immutable source-record identity.

The receipt structs force those facts to be explicitly bound rather than hidden behind a generic boolean.

## No legacy authority fallback

A structurally valid legacy SigningCommittee, a valid-looking ThresholdSignature, or `verified=true` is not enough.

Until issue #68's runtime providers exist, this crate is semantics only and does not make governance execution available.
