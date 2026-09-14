# Mycelix Public Election Verifier-Key Authorization v1

Status: **ELECT-017 foundation; immutable pre-election verifier authorization**

Parent: **ELECT-016 / PR #796**, exact hosted-qualified head
`585534c7744474442eec019ccba24d3a346b931c`.

## Purpose

ELECT-016 gives Mycelix one canonical semantic digest for an independently executed verifier receipt.
That digest still must not be allowed to authenticate itself.

ELECT-017 therefore freezes, before certification, **which verifier software releases and which
Xenia signer-key identities are authorized to authenticate those semantic receipts**.

This tranche is deliberately authorization-only:

- Mycelix owns election authorization semantics.
- Xenia owns cryptographic authentication suites and signature verification.
- ELECT-017 does **not** verify signatures.
- ELECT-017 does **not** authorize key rotation or revocation.
- ELECT-017 does **not** let an election administrator add verifier identities after the root is frozen.

## Frozen verifier keyset

Each `AuthorizedVerifierKeysetV1` binds exactly:

1. verifier release digest;
2. verifier implementation-lineage digest;
3. builder-control-domain digest;
4. Xenia Ed25519 signer-key ID;
5. Xenia ML-DSA-65 signer-key ID.

The signer-key IDs are intentionally suite-specific Xenia identities. ELECT-017 pins the frozen
Xenia V1 authentication-suite registry fingerprint:

- suite `1 = ed25519`;
- suite `2 = ml-dsa-65-fips204`;
- registry SHA-256:
  `0255c2b3070e579d52e41ab6a9d767d1700b61fd68787bbae78bd41aa868945f`.

Mycelix does **not** invent a synthetic "hybrid suite". The election policy instead requires both
authorized components. A later authentication tranche can therefore require:

`Ed25519 PASS ∧ ML-DSA-65 PASS`

over the same ELECT-016 semantic subject.

## Independence is not key count

The root fails closed unless it contains at least:

- 3 authorized verifier releases;
- 3 distinct implementation lineages;
- 2 distinct builder-control domains.

It also rejects:

- duplicate verifier-release identities;
- reused Ed25519 signer-key IDs;
- reused ML-DSA-65 signer-key IDs;
- zero release, lineage, builder, or signer identities;
- more than 64 verifier releases;
- an authorization-policy digest that does not recompute exactly.

This prevents one implementation or operator from manufacturing apparent verifier diversity by
minting additional keys.

## Election binding without a hash cycle

The root binds:

- election-definition digest;
- jurisdiction-snapshot digest;
- exact authorization-policy digest;
- canonically sorted verifier keysets.

It intentionally does **not** include the final election-constitution digest because ELECT-017 is
itself meant to become part of certification requirements committed by that constitution.

`VerifierKeyAuthorizationRequirementsBindingV1` preserves the already-existing
`certification_requirements_digest` by hashing:

`previous certification requirements || ELECT-017 authorization root`

under a dedicated domain separator.

The resulting digest can be used as the next `certification_requirements_digest` in ELECT-014's
existing certification-policy binding without replacing prior requirements and without creating a
self-hash cycle.

## Canonicalization

All digests use manually specified language-neutral bytes and SHA-256.

Verifier keysets are sorted canonically by:

1. verifier release;
2. implementation lineage;
3. builder-control domain;
4. Ed25519 signer-key ID;
5. ML-DSA-65 signer-key ID.

Input vector order is therefore non-semantic.

Reference vectors:

```text
authorization policy
91536dd76459b2be14882239b2192510ef940e94c47371beb3ae22d91a44fd98

authorization root
b091acf3f91ca78baf02e1df52965d73eb8619b49a0f0a8c60b879e4fe70afb4

certification-requirements binding
957e054bf72aeff6896f756ec35d5562096162f0e64ca0865cbd27f8adaba1a1
```

The hosted qualifier independently recomputes these vectors in Python as a second implementation.

## Why rotation is not in ELECT-017

A predecessor-bound key rotation is not sufficient by itself: a successor still needs an external
authority theorem proving who was allowed to authorize it.

Therefore the root in this tranche is immutable.

A later **ELECT-018** should introduce an acyclic governance-authorized lifecycle:

`previous state → proposed next-state digest → frozen rotation-authority policy + approval evidence
→ authorization-event digest → next state`.

That adapter may reuse existing Mycelix threshold/governance evidence, but generic governance state
must not become public-election truth merely because it exists.

## Deliberate non-claims

ELECT-017 does not claim:

- signature validity;
- public-key custody;
- key rotation or emergency revocation;
- threshold authorization of key changes;
- remote execution attestation;
- reproducible-build equality;
- ballot or tally cryptography;
- legal election certification.

Its theorem is narrower:

> A certification pipeline can determine exactly which pre-frozen verifier releases, independent
> software lineages, builder control domains, and Xenia signer-key identities are eligible to
> authenticate public-election verifier receipts, without accepting self-selected trust roots from
> those receipts.
