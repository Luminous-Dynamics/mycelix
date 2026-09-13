# Mycelix Public Election Canonical Verifier Receipt Envelope v1

Status: **ELECT-016 foundation; language-neutral receipt bytes and explicit verifier provenance**

Parent stack:

- ELECT-011 offline verifier and independent-verifier receipts;
- ELECT-014 resource-policy anchoring;
- ELECT-015 independent resource-anchor certification consumption.

Profiles:

- `mycelix-public-election-verifier-receipt-envelope-v1`
- `mycelix-public-election-verifier-provenance-v1`

## Purpose

ELECT-015 requires each resource-anchor verifier receipt to bind the exact base verifier-run receipt digest. Until ELECT-016, that digest is an opaque field: the contract did not yet say how Rust, Python, WASM, or another independent implementation must turn the complete `VerifierRunReceiptV1` into identical bytes.

ELECT-016 freezes that semantic byte transcript, freezes the already-qualified ELECT-015 anchor-receipt transcript, adds explicit build/toolchain provenance, and produces one canonical semantic envelope digest suitable for external authentication.

## Mycelix versus Xenia

The boundary is deliberate:

```text
Mycelix
  election receipt semantics
  canonical field order
  canonical enum tags
  canonical length prefixes
  semantic receipt digest
  provenance subject
        |
        v
canonical semantic envelope digest
        |
        v
Xenia / authenticated-proof layer
  signature/proof suite
  key identity
  authentication evidence
  verifier/backend adapter
```

Mycelix does not invent another generic signature framework in this tranche. Xenia should later authenticate the canonical Mycelix semantic digest.

The broader workspace already has an authenticated-proof signing domain (`MYCELIX:AuthenticatedProof:SignedEnvelope:v2`), so duplicating that machinery inside election governance would be the wrong abstraction boundary.

## Canonical base verifier-run receipt

The v1 domain is:

`MYCELIX:PUBLIC-ELECTION:VERIFIER-RUN-RECEIPT:V1\0`

The transcript contains, in order:

1. domain;
2. `offline_verifier_profile_id` as `u32-be length || UTF-8 bytes`;
3. package-root digest;
4. `verifier_implementation_id` as `u32-be length || UTF-8 bytes`;
5. verifier-lineage digest;
6. verifier-release digest;
7. source digest;
8. build-provenance digest;
9. execution-policy digest;
10. `u16-be` stage count; and
11. every verification-stage receipt in canonical stage-ID order.

Each stage encodes:

```text
u8 stage tag
32-byte package root
32-byte subject digest
32-byte verifier release
u8 disposition tag
32-byte finding digest
```

Stage vector order in memory is deliberately **not semantic**. The canonicalizer validates the complete run, then sorts by the explicit v1 stage tag before encoding. Two independent implementations therefore cannot disagree merely because one stores stages in a different vector order.

## Explicit enum tags

Verification-stage tags are frozen as:

```text
0 PackageIntegrity
1 ElectionConstitution
2 TransparencyLineage
3 WitnessQuorum
4 AnonymousAuthorityCensus
5 TallyEvidence
6 PhysicalAudit
7 ChallengeLedger
8 CertificationEvidence
```

Disposition tags are:

```text
0 Pass
1 Fail
2 Indeterminate
```

The canonical certification envelope accepts only a base run classified as `AllRequiredStagesPass`.

## Canonical resource-anchor receipt

ELECT-016 independently writes the exact ELECT-015 anchor-receipt transcript using the already-qualified domain:

`MYCELIX:PUBLIC-ELECTION:RESOURCE-ANCHOR-VERIFIER-RECEIPT:V1\0`

The new digest implementation is required to equal `ELECT-015::resource_anchor_verifier_receipt_digest(...)` exactly. Any parity failure is a hard error.

This is a migration safeguard: ELECT-016 may document and expose the bytes, but it may not silently redefine the receipt identity already consumed by certification.

## Explicit verifier provenance

`VerifierExecutionProvenanceV1` binds:

- package root;
- verifier release;
- source digest;
- existing build-provenance digest;
- execution-policy digest;
- compiler/toolchain digest;
- dependency-lock digest;
- build-recipe digest;
- target-platform digest; and
- builder-control-domain digest.

The v1 provenance domain is:

`MYCELIX:PUBLIC-ELECTION:VERIFIER-PROVENANCE:V1\0`

This transcript is deliberately separate from the base run receipt. The base run already carries the source/build/execution digests, while the richer provenance record cross-checks those exact values and adds toolchain/lock/recipe/platform information.

Keeping the provenance digest outside the base run avoids a self-referential construction such as:

```text
base receipt -> provenance digest -> base receipt
```

## Final semantic envelope

`bind_canonical_verifier_receipt_envelope(...)` requires:

- a valid, fully passing base verifier run;
- a valid passing ELECT-015 anchor receipt;
- anchor package root / lineage / release equal the base run;
- anchor `base_verifier_run_receipt_digest` equal the independently recomputed canonical base digest;
- provenance package root / release / source / build / execution policy equal the base run; and
- provenance builder-control domain equal the anchor receipt.

It then produces `CanonicalVerifierReceiptEnvelopeV1` binding:

- package root;
- verifier lineage;
- verifier release;
- builder-control domain;
- canonical base-run receipt digest;
- canonical resource-anchor receipt digest; and
- canonical provenance digest.

The final envelope domain is:

`MYCELIX:PUBLIC-ELECTION:VERIFIER-RECEIPT-ENVELOPE:V1\0`

That final SHA-256 digest is the semantic subject a later Xenia authentication adapter should sign/prove.

## Fixed golden vectors

The qualification corpus contains fixed SHA-256 golden vectors for a reference receipt set:

```text
base verifier run
  a7892dbba3e9432741f005685c5c7b5a92023affd52f22d8a0abbb6a1f42ac75

resource-anchor receipt
  1b990e3b5da5ead13d489860c4cb7e3f2f4fd6a763c676bbf4eaac1db173ecef

verifier provenance
  cc1a912f027b6a11cf87fb5873b1a67f00a1b4482f4f002108ae5b1185982096

final receipt envelope
  cbbcc8bf5b69fb28ce67bcc1f7b64aa7bdd381702311966ccbcfcc87e947ed3f
```

Independent implementations can use these as cross-language compatibility vectors rather than trusting Rust serialization behavior.

## Why not serde/bincode/JSON hashing?

The canonical digest does not hash a Rust `Debug` representation, JSON object, bincode blob, or default serde output. Those formats can differ across library versions, map orderings, numeric representations, language implementations, or serialization configuration.

ELECT-016 instead uses a tiny explicit language-neutral transcript with fixed domains, lengths, tags, endian rules, and field order.

## Deliberate non-claims

ELECT-016 does not yet provide:

- a cryptographic signature over the envelope;
- signer-key identity or certificate semantics;
- ML-DSA/Ed25519/hybrid suite selection;
- Xenia verification/adaptation code;
- remote attestation of the executing machine;
- reproducible-build equality between independent builders;
- proof that distinct verifier implementations share no common bug;
- ballot/tally cryptography; or
- legal certification.

## Next tranche

After hosted qualification, proceed to **ELECT-017 — Xenia-authenticated election verifier envelope adapter**. It should authenticate the ELECT-016 semantic envelope digest with a crypto-agile Xenia-owned authentication profile while keeping Mycelix election semantics outside the signature/proof layer.
