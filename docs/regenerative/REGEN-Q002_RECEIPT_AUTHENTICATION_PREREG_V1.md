# REGEN-Q002 — Qualification Receipt Authentication Preregistration v1

Status: preregistration only. Parent: qualified REGEN-Q001 ProductHead `9a9dfea07d5b19707df5f5c7c70c5b8bfe68092a`.

## 1. Purpose

REGEN-Q001 makes qualification evidence machine-readable and semantically bounded. It deliberately does not establish who produced a receipt, whether the receipt bytes were altered after production, whether the signer was authorized to issue that class of receipt, or whether the receipt is included in any independently witnessed history.

REGEN-Q002 preregisters a detached authentication layer for Q001 receipts without widening what the underlying qualification campaign proved.

The central firewall is:

```text
structurally valid receipt
!= authenticated receipt

valid signature
!= authorized signer
!= truthful receipt contents
!= scientific validity
!= governance authority
!= physical-action authority
```

Authentication strengthens provenance of the receipt artifact. It does not convert the artifact into truth or authority.

## 2. Composition boundary

Preferred direction:

```text
REGEN-Q001 receipt bytes
        |
        v
exact cryptographic receipt digest
        |
        v
REGEN-Q002 authentication statement
        |
        +--> signature / signer-key evidence
        +--> credential / authorization evidence where required
        +--> optional transparency / witness evidence
```

Q002 SHOULD be a detached envelope. The original Q001 receipt bytes remain independently inspectable and validatable by the Q001 schema/validator.

Q002 MUST NOT rewrite Q001 assertions, proposition, non-claims, result, subject identity, dependency state, or campaign evidence.

## 3. Receipt identity

An authentication envelope MUST bind an exact receipt artifact identity rather than a filename or mutable URL.

Conceptually:

```text
ReceiptCommitment {
    digest_algorithm,
    digest,
    byte_length,
    receipt_schema,
}
```

The exact receipt bytes being authenticated MUST be recoverable or otherwise independently obtainable by the verifier.

```text
same semantic JSON fields
!= same authenticated receipt bytes
```

A reserialized receipt is a different byte artifact unless a separately qualified canonicalization rule establishes equivalence.

## 4. Canonicalization boundary

Q002 MUST NOT assume that arbitrary JSON serialization is canonical.

The first executable implementation MUST choose one reviewed signing-input construction before signatures are treated as interoperable. Acceptable strategies may include:

1. signing a deterministic, qualified canonical encoding of the authentication statement; or
2. signing a fixed binary framing whose fields include the digest of the exact Q001 receipt bytes.

The implementation MUST preregister and test the exact framing. Whitespace, field ordering, Unicode normalization, integer representation, and delimiter ambiguity cannot be left to library defaults if they affect signed bytes.

Until such a framing is qualified:

```text
signature code exists
!= interoperable authenticated receipt protocol
```

## 5. Domain separation

The signed statement MUST include an unambiguous protocol/domain separator so a signature produced for another Mycelix/Xenia object cannot be replayed as a REGEN receipt authentication.

Conceptually:

```text
domain = "mycelix:regen:qualification-receipt-auth:v1"
```

The exact production framing is implementation work and must be qualified.

Cross-protocol signature reuse MUST fail closed.

## 6. Authentication envelope

A future implementation should contain semantics approximately equivalent to:

```text
ReceiptAuthentication {
    schema_version,
    receipt_commitment,
    signer,
    signature_suite,
    signature,
    signing_context,
    signed_at?,
    credential_refs[],
    transparency_refs[],
    witness_refs[],
}
```

Fields are propositions, not trust conclusions.

For example:

```text
credential_refs present
!= credential valid
!= signer authorized
```

## 7. Signer identity

A signer reference MUST bind to exact verification-key material or an exact immutable key/credential reference resolvable under the consuming trust policy.

The protocol must distinguish at least:

- key identity;
- human/service/device identity associated with a key;
- credential issuer;
- credential status;
- authorization to issue a particular receipt class.

These MUST NOT be collapsed into one `trusted: bool`.

```text
cryptographic key possession
!= organizational identity
!= qualification authority
```

## 8. Signature-suite boundary

Q002 should remain algorithm-agile at the abstract contract layer.

Xenia may later provide concrete, qualified adapters for supported suites such as Ed25519 and/or post-quantum signatures already used elsewhere in the Luminous stack. Q002 itself MUST NOT invent custom cryptography.

Every concrete suite profile must bind:

- exact algorithm/suite identifier;
- exact public-key encoding;
- exact signature encoding;
- exact signed-message framing;
- validation rules;
- malformed-input behavior;
- key-size/signature-size bounds;
- implementation/version qualification evidence where material.

Unknown suites fail closed.

## 9. Key lifecycle

Verification of a mathematical signature is distinct from validation of key lifecycle state.

A consuming policy may need evidence for:

- key activation;
- rotation/supersession;
- revocation;
- compromise declarations;
- credential expiry;
- organizational role changes;
- device/service decommissioning.

Q002 MUST preserve the distinction between:

```text
signature verifies under key K
```

and:

```text
key K was authorized for this receipt at the relevant time
```

## 10. Time semantics

`signed_at` is optional evidence, not trusted time by itself.

```text
signed_at field present
!= trusted signing time
```

A strong time claim requires separately qualified evidence such as a trusted timestamp, transparency inclusion with independently established timing semantics, or another reviewed mechanism.

Clock uncertainty and source identity must remain explicit when timing affects authorization/revocation decisions.

## 11. Authorization policy

Q002 authentication MUST NOT hard-code one universal issuer hierarchy.

A verifier may evaluate a separate policy such as:

```text
receipt subject class
+ campaign type
+ organization / project
+ signer credential
+ current key status
+ required witnesses
-> authorized / rejected / indeterminate
```

That policy is a distinct proposition from signature validity.

Q002 core should expose enough evidence for policy evaluation without deciding governance legitimacy globally.

## 12. Multiple signatures and witnesses

Multiple signatures may be attached to the same exact receipt commitment.

The data model SHOULD preserve independent signer statements rather than merging them into one synthetic signer.

```text
N signatures
!= N independent organizations
!= quorum
!= consensus
```

Witness/quorum semantics require an exact consuming policy that defines unique principals, threshold rules, conflicts, revocation behavior, and independence assumptions.

## 13. Transparency evidence

Q002 may reference an append-only transparency system, but a transparency reference is separate from signature verification.

Potential evidence can include:

- log identity;
- entry identity/index;
- inclusion proof;
- checkpoint/root identity;
- checkpoint signer identity;
- consistency proof where appropriate.

The contract must preserve:

```text
receipt logged
!= receipt valid
!= signer authorized
!= log globally complete
```

No single transparency provider becomes mandatory in the abstract core.

## 14. Replay and context binding

A valid authentication for receipt R MUST NOT be reusable as authentication for receipt R2.

The signed statement should bind at least:

- exact receipt commitment;
- Q002 schema/version/domain;
- signer/key identity or key commitment;
- signature-suite identity;
- material signing context.

If a consuming policy needs project, environment, release, or organization context, those values must be signed/bound rather than supplied only by an untrusted wrapper.

## 15. Mutation semantics

Changing any authenticated receipt byte creates a new receipt commitment.

```text
receipt mutation
=> old authentication no longer authenticates new bytes
```

Changing only the authentication envelope may leave the underlying Q001 receipt commitment unchanged, but the new envelope is a distinct authentication artifact.

Historical invalid/revoked authentication records SHOULD remain auditable rather than being overwritten.

## 16. Failure taxonomy

A future validator SHOULD distinguish failure classes rather than returning one generic false value, including approximately:

- unsupported Q002 schema;
- malformed receipt commitment;
- receipt digest mismatch;
- unsupported signature suite;
- malformed key;
- malformed signature;
- signature verification failure;
- signer identity unresolved;
- credential unresolved;
- credential invalid/expired/revoked;
- signer unauthorized for receipt class;
- timestamp evidence unresolved;
- transparency evidence invalid/unresolved;
- witness policy unsatisfied;
- context mismatch;
- replay/domain mismatch.

Not every deployment must require every optional evidence class, but absence must remain distinguishable from failure.

## 17. Verification stages

A robust verifier should preserve staged propositions:

```text
1. Q001 receipt structural validation
2. exact receipt digest verification
3. Q002 envelope structural validation
4. signature cryptographic verification
5. signer/key resolution
6. key/credential lifecycle evaluation
7. issuer/authorization policy evaluation
8. optional transparency/witness evaluation
9. final policy-specific authentication decision
```

A failure at a later stage MUST NOT rewrite earlier-stage facts.

For example, a mathematically valid signature from an unauthorized key remains a valid cryptographic-signature fact plus a failed authorization fact.

## 18. Xenia integration boundary

Xenia is a natural provider for concrete signing, key identity, secure verification, attestation, and post-quantum/hybrid capabilities.

The dependency direction SHOULD remain:

```text
REGEN-Q002 abstract authentication semantics
        ^
        |
Xenia-backed adapter / deployment profile
```

rather than making the REGEN receipt schema dependent on Xenia transport/session/runtime internals.

A future Xenia adapter must carry its own qualification evidence and security non-claims.

## 19. Offline verification

Where practical, receipt authentication should support deterministic offline verification from a frozen evidence bundle containing the receipt, authentication envelope, public key/credential material, policy revision, and any required transparency/witness evidence.

Offline verification does not magically know current revocation state. A verifier must distinguish:

```text
valid as-of frozen evidence snapshot
```

from:

```text
currently non-revoked according to a live authority
```

This aligns naturally with REGEN-021 evidence-snapshot/currentness semantics.

## 20. Privacy and selective disclosure

Authentication SHOULD NOT require public disclosure of unrelated personal or organizational identity data.

Deployments may use pseudonymous service principals, scoped credentials, selective disclosure, or privacy-preserving attestations where those mechanisms are separately qualified.

The core requirement is that the verifier can evaluate the exact required trust proposition—not that all identity data becomes globally public.

## 21. Supply-chain boundary

An authenticated qualification receipt may attest to software that contains vulnerable or malicious dependencies.

```text
authenticated receipt
!= safe software
!= dependency provenance complete
!= vulnerability-free closure
```

REGEN-008, Nix/supply-chain evidence, artifact signatures, SBOMs, and other assurance layers remain distinct and composable.

## 22. Proposed implementation shape

Prefer a dependency-light core, for example:

```text
crates/mycelix-regen-qualification-auth
```

or a narrowly scoped module adjacent to Q001 receipt tooling.

The pure core should own:

- bounded envelope types;
- receipt commitment validation;
- deterministic signed-statement framing;
- signature-suite dispatch interface;
- staged verification result types;
- authorization-policy input types;
- serialization revalidation;
- adversarial test vectors.

Network lookup, live revocation fetching, transparency clients, Holochain, and Xenia runtime code should remain adapters.

## 23. Qualification plan

A first executable Q002 campaign should include at least:

1. exact qualified Q001 parent binding;
2. deterministic framing golden vectors;
3. positive signature vector;
4. one-bit receipt mutation rejection;
5. one-bit signed-context mutation rejection;
6. wrong-domain rejection;
7. wrong-key rejection;
8. malformed key/signature rejection;
9. unknown-suite rejection;
10. valid-signature-but-unauthorized-signer case remaining distinct;
11. revoked/expired credential case remaining distinct;
12. duplicate/multiple-signature handling;
13. serialization revalidation;
14. bounded-input/fuzz/property campaign where useful;
15. independent verifier/vector where practical;
16. exact-head qualification and clean checkout;
17. REGEN-Q001-compatible machine-readable qualification receipt for Q002 itself.

The campaign should use REGEN-008 ProductFrozen dependency semantics where practical.

## 24. Deliberate non-claims

REGEN-Q002 establishes no signer authority merely from key possession, no truth of receipt assertions, no completeness of qualification evidence, no scientific correctness, no agronomic efficacy, no contamination safety, no climate/carbon authority, no legal/regulatory approval, no governance legitimacy, no supply-chain safety, no current non-revocation unless explicitly evaluated, and no physical-action authority.

Its narrow purpose is:

> make the provenance and integrity of a REGEN-Q001 qualification receipt independently verifiable while keeping cryptographic authenticity, issuer authorization, evidence truth, and downstream authority as separate propositions.
