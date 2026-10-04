# Mycelix Protected Release Boundary v0.1

**Status:** executable reference-model composition  
**Claim ceiling:** `ReferenceModelOnly`

This artifact freezes the Mycelix-side semantics for protected data release across independent security domains.

It is deliberately **not** a Cross Domain Solution, declassification authority, or legal export-control engine.

## Core theorem

```
source object
  -> source authorization
  -> release decision
  -> approved transformation
  -> destination admission
  -> transfer
  -> transfer receipt
```

Every stage is independent.

A valid source authorization does not imply destination admission. A transformation being approved does not itself authorize release. A transfer receipt is historical evidence and cannot become a reusable bearer token.

## Why the separation matters

NIST security-control material treats information-flow enforcement as its own control surface, including enforcing approved authorizations for the flow of information between domains. NIST SP 800-172 Rev. 3 is the current enhanced CUI publication and emphasizes, among other things, access controls and segmentation. citeturn914474search36turn914474search0

This contract therefore treats release as a **state transition with explicit policy inputs**, rather than as a label attached to an object.

## Exact release identity

The release intent binds:

- source and destination security domains;
- source object identity and digest;
- classification/control state;
- compartment;
- releasability;
- export-control profile;
- purpose;
- authorized subject;
- authorization basis;
- source and destination policy versions;
- transform profile identity.

The transferred output then receives a distinct payload digest.

```
input artifact A
 + transform T
 -> output artifact B
```

A receipt for B cannot retroactively authorize releasing A.

## Transfer receipt

The receipt is evidence that one exact transfer reached the declared terminal state.

It binds:

```
transfer ID
source domain
destination domain
source object digest
output payload digest
transform digest
source policy version
destination policy version
purpose
release profile
issuance/expiry
outcome
```

It does **not** mean:

- the source policy remains current;
- the destination remains authorized;
- the same payload may be transferred again;
- a new object may inherit the receipt;
- a different destination may reuse it;
- a future release is pre-approved.

## Conservative failure semantics

A hard policy failure is **DENY**.

Loss of the transfer gateway or other inability to determine the transfer state is **INDETERMINATE**, not success.

This distinction prevents network availability from becoming security authority.

## Hidden metadata

The profile explicitly includes metadata checking.

A transform may produce an acceptable-looking plaintext body while leaving restricted metadata, embedded fields, document properties, alternate streams, or other release-relevant state.

Therefore:

```
payload body approved
!= complete release approved
```

The v0.1 semantic model simply represents this as a required metadata-check result. A production filter must independently qualify the exact metadata extraction/normalization semantics.

## Replay / duplicate transfer

Transfer identity is single-use in the semantic model.

A receipt proves the historical transfer of exactly one transfer identity; it is not a transferable capability.

```
receipt(A -> B)
!= permission to transfer A -> B again
```

Duplicate-transfer prevention therefore requires an authoritative state store or equivalent exact transfer ledger in any production implementation.

## CDS boundary

This profile intentionally stops before the actual controlled transfer mechanism.

```
Mycelix semantic release contract
!= classified CDS
```

An actual classified cross-domain deployment still requires its independently governed and qualified transfer boundary.

## Qualification corpus

27 deterministic vectors cover:

- source/destination mismatch;
- classification mismatch;
- releasability;
- export-control mismatch;
- purpose mismatch;
- stale/revoked authorization;
- policy mismatch;
- metadata;
- payload/transform digest;
- replay and duplicate transfer;
- unapproved transform;
- destination refusal;
- gateway unavailability;
- receipt substitution/expiry;
- label spoofing;
- attestation non-equivalence;
- transformed-output reauthorization.

## Qualification ceiling

A green run establishes only the deterministic semantics of this reference release model.

It does not establish:

- CDS security or approval;
- declassification correctness;
- legal export-control authorization;
- filter correctness for arbitrary formats;
- physical or endpoint security;
- current organizational authorization;
- classified processing authorization;
- CMMC status;
- system-wide superiority over conventional network architectures.
