# Mycelix Public Election Anonymous Authority v0.1

Status: **ELECT-007 / ELECT-008 foundation; backend-neutral and not production cryptography**

Parent: `mycelix-public-election-v1`

Profile identifier: `mycelix-public-election-anonymous-authority-v1`

## Purpose

This tranche separates **eligibility authority** from **ballot identity**.

A public-election verifier should be able to learn only:

> an eligible elector possesses one valid authorization for this exact frozen election scope, and the scope-local authority has not been used inconsistently.

It must not need a civil identity, Mycelix DID, account key, reputation value, consciousness/phi score, credential serial, or any other stable cross-election identifier.

This crate deliberately does not choose BBS, STARKs, SNARKs, anonymous credentials, blind signatures, or another proof construction yet. Those are candidate backends that must satisfy the same public contract.

## Research basis

Anonymous credentials are specifically intended to prove authorization without disclosing identity and can support unlinkable presentations. NIST's Privacy-Enhancing Cryptography program has treated anonymous credentials, blind signatures, private authentication, revocation, decentralization, and public auditability as an active privacy/security area.

The W3C Data Integrity BBS cryptosuite specifies selective disclosure and unlinkable derived proofs, but its privacy considerations also show why Mycelix must define policy above the primitive: issuer-key partitioning, message counts, and disclosed indices can create linkability even when the underlying proof is unlinkable.

References:

- https://csrc.nist.gov/Projects/pec/stppa
- https://csrc.nist.gov/Presentations/2022/stppa4-anonym-cred
- https://www.w3.org/TR/vc-di-bbs/

The conclusion for Mycelix is: **cryptographic unlinkability is necessary but not sufficient; issuance, revocation, key partitioning, public fields, and verifier behavior must all preserve the privacy theorem.**

## ELECT-007 — anonymous eligibility statement

`AnonymousEligibilityPublicStatementV1` intentionally exposes only election-scoped public inputs:

- public-election profile identity;
- anonymous-authority profile identity;
- frozen election constitution digest;
- frozen election definition digest;
- exact contest/authorization scope digest;
- eligibility-rules digest;
- credential issuer-policy digest;
- credential schema digest;
- frozen revocation snapshot digest;
- exact proof-system profile digest;
- privacy-policy digest;
- nullifier-derivation profile digest; and
- one scope-local nullifier.

There is no field for a civil identity, DID, account identifier, device identity, social graph identity, governance score, or persistent holder pseudonym.

The future proof relation must establish, from hidden witness material, that the presenter possesses valid non-revoked eligibility authority under the frozen rules and that the public nullifier is derived from that same hidden authority.

The Rust type validates **statement structure only**. It does not verify a cryptographic proof or claim that a candidate backend provides the required privacy properties.

## Privacy requirements

The profile requires all of these properties before a proof backend can be promoted:

1. civil identity remains hidden from the ballot transcript;
2. credential serial/issuance handle remains hidden;
3. ordinary Mycelix account keys remain hidden;
4. governance/reputation/consciousness scores remain hidden;
5. presentations are unlinkable across authorization scopes;
6. verifier-to-verifier correlation is resisted;
7. issuer-to-verifier collusion/linkage is addressed by both cryptography and issuer-key policy;
8. revocation checking preserves ballot unlinkability; and
9. no stable holder pseudonym is embedded in the proof transcript.

These are policy requirements, not booleans that magically make a cryptosystem safe. A later qualification package must bind each requirement to protocol analysis, test vectors, implementation evidence, and where appropriate formal security arguments.

## Authorization scope

A nullifier is not globally stable. Its scope is bound to the exact:

```text
election constitution
+ election definition
+ contest/authorization scope
+ eligibility rules
```

Changing any of those changes the scope.

This permits one hidden credential to authorize multiple legitimate election scopes without publishing a correlatable global pseudonym.

## ELECT-008 — scope-local single-use authority

A public nullifier is permitted only as a **scope-local anti-double-use value**.

The v1 security-property registry requires:

1. deterministic derivation within one exact scope;
2. cryptographic domain separation by scope;
3. unlinkability across different scopes;
4. one-wayness from the public transcript back to the hidden authority;
5. binding to the same hidden witness that proves eligibility; and
6. no encoding of stable civil/account/device/cross-election identity.

The derivation algorithm is intentionally not specified in this tranche.

## Duplicate authority is not first-arrival authority

A distributed network can observe two different ballot claims carrying the same valid scope-local nullifier. Mycelix must never silently interpret network arrival order as democratic authority.

Pairwise classification is therefore:

```text
different scope
    -> DistinctScope
same scope + different nullifier
    -> DistinctAuthority
same scope + same nullifier + exact same claim
    -> IdempotentReplay
same scope + same nullifier + changed claim
    -> ConflictingUseOfSameAuthority
```

The only v1 duplicate-authority policy is:

`FreezePendingEvidenceResolution`

No `first_wins`, `last_wins`, timestamp preference, DHT arrival preference, reputation preference, or administrator override exists in this kernel.

This is intentionally conservative. Later protocol work must decide whether a conflict is resolved by a frozen revoting rule, credential-cleansing protocol, physical ballot evidence, adjudication, or another formally specified mechanism.

## Complete-census requirement before tally

The tally boundary receives a `NullifierCensusV1` bound to an exact complete-checkpoint digest.

Structurally, tally admission requires:

```text
conflicting_nullifier_count == 0
observed_claim_count == unique_nullifier_count
checkpoint_digest != 0
```

This does **not** prove that the checkpoint is actually complete. ELECT-009/010 must establish append-only checkpoint lineage and independent witnessing before a census becomes promotable election evidence.

## Why not simply use BBS now?

BBS is a serious candidate because current W3C work provides selective disclosure and unlinkable derived proofs. But the W3C privacy analysis explicitly notes linkability risks from issuer public-key partitioning and disclosure structure. A government could accidentally—or deliberately—issue rare keys to small groups and make otherwise unlinkable presentations correlatable.

Therefore Mycelix must first freeze:

- issuer-key anonymity-set rules;
- key-rotation rules;
- revocation privacy rules;
- public statement fields;
- presentation domain separation;
- verifier logging policy; and
- cross-scope correlation tests.

Only then should we choose a concrete anonymous-credential backend.

## Coercion boundary

Anonymous eligibility and a scope-local nullifier do **not** solve coercion resistance.

JCJ/Civitas-family research shows why coercion-resistant voting may require substantially stronger mechanisms such as fake credentials, revoting, deniable behavior, or other evasion strategies. Recent work continues to find subtle forced-abstention and last-minute-coercion failures in apparently stronger designs.

For Mycelix's initial polling-place + paper profile, ELECT-007/008 should be treated as privacy and uniqueness foundations, not as a claim of remote-voting coercion resistance.

## Deliberate non-claims

This tranche does not establish:

- a concrete anonymous-credential scheme;
- proof soundness;
- anonymous issuance;
- issuer blindness;
- coercion resistance;
- receipt freeness;
- safe revoting;
- revocation privacy in an implementation;
- completeness of the DHT/election log;
- conflict resolution;
- cast-as-intended verification;
- ballot secrecy of an encrypted ballot construction; or
- readiness for a public election.

## Next tranche

Proceed next with:

- **ELECT-009** append-only election transparency/checkpoint lineage;
- **ELECT-010** independent witness/checkpoint quorum;
- **ELECT-011** minimal offline verifier contract; and
- **ELECT-012** physical ballot/manifest/custody/audit evidence types.

Only after those foundations should a ballot cryptosystem or concrete anonymous-credential backend be selected.
