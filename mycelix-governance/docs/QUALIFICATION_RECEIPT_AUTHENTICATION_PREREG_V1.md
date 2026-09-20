# QREC-002 — Authenticated Qualification Evidence Ingestion Preregistration v1

Status: preregistration only. Parent QREC-001 semantic subject: `2579257a03d502c4dd5319b1c6b1c0768309e59d`.

Tracker: #2088.

## 1. Purpose

QREC-001 gives qualification evidence a deterministic identity and separates caller-presented receipts from the non-constructible `VerifiedQualificationReceiptV1` activation path.

QREC-002 defines what a future ingestion boundary must prove before it may construct that verified type.

Core firewall:

```text
well-formed QREC receipt
!= authenticated hosted qualification

successful hosted qualification
!= scientific truth
!= governance legitimacy
!= provider/runtime authority
```

QREC-002 must authenticate evidence provenance without widening the proposition proved by the underlying qualifier.

## 2. Reuse-first basis

QREC-002 reuses established Luminous patterns rather than defining new cryptography:

- REGEN-Q001 / PR #996: machine-readable qualification receipt != authenticated receipt;
- REGEN-Q002 / PR #1264: detached authentication, exact byte commitment, staged verification, key lifecycle and issuer authorization remain separate;
- Xenia `xenia-peer` PR #320 @ `89b62e2d7be04d0d7e1df446df4f9dace6b608f1`: generic evidence-artifact attestation that recomputes authoritative artifact bytes and returns a verified non-deserializable result;
- Mycelix Identity PRs #540/#541: key association/provenance != trust/authority and no fake public `Verified*` constructor;
- GitHub/Sigstore trusted-builder patterns already used elsewhere in Luminous: candidate execution is privilege-separated from the fresh attestation job.

QREC-002 core must remain crypto-provider-neutral. GitHub/Sigstore and Xenia are adapters/profiles, not semantics embedded in QREC-001.

## 3. Well-founded artifact graph

The authentication profile must avoid self-referential digests.

```text
E = deterministic qualification evidence payload capsule
R = QREC-001 receipt
A = deterministic authentication capsule
```

Exact relation:

```text
sha256(E) = R.artifact_digest

E MUST NOT contain R.

A contains:
  - exact E bytes
  - exact R bytes
  - canonical authentication manifest

GitHub/Sigstore and/or Xenia authenticate exact A bytes.
```

Therefore `R.artifact_digest` names E, never A and never a container recursively containing R.

## 4. Evidence payload E

The first executable profile should use a deterministic archive and a closed member census.

E may contain qualifier evidence such as:

- exact semantic/verifier/parent identities;
- source hashes before/after;
- toolchain versions;
- format/check/test/Clippy logs;
- mutation-control results;
- derived lockfile + digest when applicable;
- formal-model outputs where applicable;
- qualifier-specific receipts or evidence manifests.

E must not contain:

- QREC receipt R;
- GitHub/Sigstore attestation over A;
- Xenia attestation over A;
- mutable URLs/titles as authority-bearing content.

The executable profile must freeze archive format, member ordering, path normalization, mtime, uid/gid, permissions and compression semantics so identical evidence inputs produce identical E bytes.

## 5. QREC receipt R

R is a valid QREC-001 `QualificationReceiptV1` whose authority tuple includes:

- dependency ID;
- trusted evidence issuer ID;
- exact semantic head;
- exact verifier head;
- hosted run ID;
- hosted run attempt;
- `sha256(E)`;
- deterministic receipt commitment.

R remains caller-presentable evidence. Its shape alone is not authenticity.

The ingestion adapter must independently reconstruct the expected tuple from authenticated hosted facts and require exact equality with R.

## 6. Authentication capsule A

A is the object authenticated by external provenance systems.

A should contain exactly:

- E;
- R;
- one canonical manifest.

The manifest must bind at least:

- QREC authentication profile/version;
- exact member names;
- exact byte lengths;
- exact SHA-256 member digests;
- exact QREC receipt commitment;
- exact evidence-payload digest;
- exact authentication-capsule subject label.

A must itself be deterministic and content-addressed.

A different receipt over the same E must produce different A bytes. A different E with a copied R must fail reconstruction before authentication is accepted.

## 7. Hosted GitHub authenticity profile

Preferred GitHub profile uses privilege separation.

### 7.1 Qualification job

The qualification job:

- executes the exact verifier;
- has `contents: read` only;
- receives no OIDC/attestation write authority;
- produces deterministic E;
- may produce a presented R candidate;
- uploads only untrusted/intermediate evidence.

### 7.2 Trusted attestation job

A separate fresh job:

1. starts only after qualification completed successfully;
2. never runs candidate product/effect code merely to sign evidence;
3. downloads the exact intermediate evidence;
4. independently derives trusted repository/workflow/run context;
5. requires exact semantic/verifier lineage;
6. rejects skipped/no-step qualification as PASS evidence;
7. recomputes E from the downloaded evidence or verifies exact deterministic E bytes;
8. independently reconstructs R and requires exact equality/commitment;
9. builds deterministic A;
10. signs/attests exact A with GitHub OIDC/Sigstore;
11. locally verifies the emitted attestation bundle before publishing the verified evidence package.

Only this attestation job receives the minimum required permissions, such as:

- `contents: read`;
- `id-token: write`;
- `attestations: write`;
- `artifact-metadata: write` where required.

Fork/untrusted PR code must not receive repository-issued attestation authority.

## 8. Trusted-builder bootstrap rule

A PR-editable workflow must not self-declare itself a trusted builder.

The preferred production design is a reusable attestation workflow that has already landed at an approved immutable revision. A caller pins that exact approved workflow revision.

The trusted job must independently prove the relationship between:

- approved builder revision;
- evidence subject;
- allowed candidate-source drift;
- exact verifier definition;
- exact qualification evidence.

QREC-002 preregistration itself cannot self-attest or bootstrap its own trusted-builder authority.

## 9. GitHub observation requirements

An authenticated hosted observation must bind/verify at least:

- canonical repository numeric ID;
- canonical repository identity;
- workflow identity/path;
- approved builder revision;
- semantic head;
- verifier head;
- run ID;
- run attempt;
- qualifier job ID;
- `status=completed` and `conclusion=success`;
- non-empty executed qualifier step set;
- evidence payload E digest/length;
- R receipt commitment;
- authentication capsule A digest;
- attestation subject digest;
- GitHub-hosted runner policy where required.

Mutable PR titles, branch labels, display names, URLs and navigation metadata are never qualification authority.

## 10. GitHub/Sigstore meaning

A valid GitHub/Sigstore attestation establishes a provenance statement about exact A bytes under a specific GitHub workflow/source context.

It does not establish:

- truth of the qualification proposition;
- correctness of tests or formal models beyond their bytes/results;
- independence of the builder from repository governance;
- deployment currentness;
- governance legitimacy;
- provider replay safety;
- runtime/effect authority.

## 11. Optional Xenia authentication profile

Xenia PR #320 is the preferred cryptographic external-artifact primitive once independently qualified.

Recommended QREC metadata:

```text
artifact_domain = "mycelix-governance-qualification-evidence"
artifact_schema = "mycelix-governance-qrec-auth-capsule-v1"
subject_ref     = exact R.receipt_commitment
artifact_bytes  = exact A bytes
```

Xenia must recompute the artifact digest from exact A bytes and verify the signature/key/suite binding.

A successful Xenia verification proves only:

> this exact verification key authenticated these exact A bytes and semantic metadata.

It does not prove the key is authorized to issue QREC evidence.

## 12. Signer/key authority remains separate

Where a policy requires an institutional or DID-bound issuer, compose separately qualified evidence for:

- key↔DID/institution association;
- exact key purpose/scope;
- activation/rotation/revocation/compromise state;
- authorization to issue the specific QREC class;
- optional witness/quorum policy.

Candidate reusable work includes Mycelix Identity #540/#541 and Xenia #321, but QREC-002 must not promote those draft/unqualified subjects merely by referencing them.

## 13. VerifiedQualificationReceiptV1 minting

Only a separately-qualified ingestion implementation may construct QREC-001's `VerifiedQualificationReceiptV1`.

Conceptually:

```text
presented R
+ exact E
+ independently authenticated hosted observation
+ trusted builder/issuer policy
+ valid authentication of A
+ optional external signer/key policy when required
--------------------------------------------------
VerifiedQualificationReceiptV1
```

The constructor must remain unavailable to ordinary callers and Serde input.

The verified wrapper is evidence of authenticated qualification provenance under a specific policy. It is not itself provider/runtime activation.

## 14. Staged result model

QREC-002 should retain staged facts instead of one `trusted: bool`:

1. QREC structural validation;
2. evidence-payload digest validation;
3. hosted metadata/run-attempt validation;
4. exact qualifier success/step validation;
5. receipt reconstruction equality;
6. authentication-capsule reconstruction;
7. GitHub/Sigstore verification;
8. optional Xenia signature verification;
9. signer/key lifecycle evaluation;
10. issuer authorization policy evaluation;
11. final verified-receipt eligibility.

A later-stage failure must not rewrite earlier facts.

## 15. Failure taxonomy

Distinguish at least:

- malformed receipt;
- evidence digest mismatch;
- receipt reconstruction mismatch;
- wrong repository/issuer;
- wrong semantic head;
- wrong verifier head;
- wrong run ID;
- wrong run attempt;
- qualifier not completed successfully;
- qualifier skipped/no executed steps;
- wrong trusted-builder revision;
- authentication-capsule mismatch;
- malformed/unverifiable GitHub attestation;
- wrong attestation subject;
- forbidden self-hosted runner;
- required Xenia evidence missing/invalid;
- wrong Xenia domain/schema/subject/key/suite;
- signer association unresolved;
- key revoked/expired/compromised;
- signer unauthorized for QREC class;
- currentness unresolved;
- consumer policy unsatisfied.

## 16. First executable campaign

The first executable QREC-002 tranche should include:

- deterministic E golden vector;
- deterministic A golden vector;
- mutation of any E byte invalidates R/A relationship;
- mutation of R invalidates A;
- wrong repository/workflow/builder revision rejection;
- wrong semantic/verifier/run/attempt rejection;
- skipped qualifier rejection;
- failed qualifier rejection;
- empty-step qualifier rejection;
- wrong artifact digest rejection;
- Sigstore subject mismatch rejection;
- wrong-domain/wrong-key Xenia rejection when Xenia profile is exercised;
- valid cryptographic signer but unauthorized issuer remaining distinct;
- exact-head qualification;
- QREC-001-compatible receipt for QREC-002's own campaign.

## 17. Non-claims

This preregistration does not establish:

- QREC-001 qualification;
- QREC-002 qualification;
- authenticated GitHub evidence today;
- trusted GitHub builder policy today;
- Xenia #320 qualification;
- signer authorization merely from key possession;
- evidence truth;
- scientific validity;
- governance legitimacy;
- Holochain/provider/runtime authority;
- deployment currentness;
- physical exactly-once behavior.

Its narrow purpose is to freeze a well-founded, reusable authentication architecture for exact QREC evidence while keeping structural validity, hosted provenance, cryptographic authenticity, issuer authorization and downstream effect authority as separate propositions.
