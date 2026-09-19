# MYC-ZKP-ATTEST-001R — Authenticated Qualification Receipt Contract

## Status

Design/qualification contract implementing issue #2000 on top of the structural-coherence boundary in #1948 / #1951.

This contract defines what must be true before Mycelix may call a qualification receipt **authenticated**.

It does **not** grant production proof admission or application authority.

## 1. Governing theorem

```text
policy tier
    != measured proof security
    != theorem qualification
    != structural evidence coherence
    != receipt authentication
    != production proof admission
    != application authority
```

Every transition is an independent theorem with its own evidence.

In particular:

```text
EvidenceCoherenceEvaluationV1::is_coherent() == true
    != authenticated receipt
```

and:

```text
AuthenticatedQualificationReceiptV1
    != ProductionAdmissionV1
```

## 2. Threat model

The authentication layer MUST fail closed against at least these attacks:

1. setting a caller-controlled `verified = true` boolean;
2. implementing a public verifier trait that always returns success;
3. directly constructing an authenticated-capability struct;
4. deserializing arbitrary bytes into an authenticated capability;
5. verifying a signature over bytes other than the canonical qualification receipt;
6. replaying a valid detached verification report against a different receipt;
7. accepting a valid signature from an unapproved signer identity;
8. accepting the correct repository with the wrong signer workflow;
9. accepting the correct workflow path under an unapproved workflow revision when exact revision is required;
10. accepting a different OIDC issuer;
11. accepting a different source revision/ref/digest;
12. accepting a different in-toto predicate type;
13. accepting the expected predicate type with the wrong Mycelix predicate schema/version;
14. substituting trusted-root material;
15. omitting required transparency-log or timestamp evidence;
16. accepting expired/revoked/superseded authentication evidence contrary to policy;
17. accepting structurally valid but cryptographically invalid bundles;
18. turning authenticated evidence directly into Governance/Finance/Health/FL authority.

## 3. Canonical qualification receipt

The object being authenticated is a backend-neutral canonical qualification receipt.

Conceptually:

```rust
pub struct QualificationReceiptV1 {
    pub receipt_version: u32,
    pub qualification_profile: String,
    pub statement_profile: String,
    pub theorem_profile: String,
    pub subject_sha: String,
    pub dependency_graph_digest: String,
    pub measured_security_receipt_digest: String,
    pub coherence_policy_id: String,
    pub coherence_result_digest: String,
    pub qualification_corpus_digest: String,
    pub execution_capsule_digest: String,
    pub result: QualificationResultV1,
    pub nonclaims: Vec<String>,
}
```

Exact field types MAY become stronger newtypes during implementation, but these semantics are mandatory.

### Canonicalization

The authenticated digest MUST be computed over deterministic versioned canonical bytes.

The implementation MUST NOT define receipt identity as:

- pretty-printed JSON bytes;
- map iteration order;
- a UI rendering;
- GitHub artifact metadata;
- a filename;
- an unversioned serialization.

A canonicalization profile MUST be identified explicitly, for example:

```text
mycelix-qualification-receipt-canonical-v1
```

The canonical digest MUST identify both:

- the digest algorithm/profile; and
- the canonicalization profile.

## 4. Receipt digest and subject binding

The authentication subject MUST bind the exact canonical receipt digest.

Conceptually:

```text
qualification receipt canonical bytes
          │
          ▼
canonical digest
          │
          ▼
attestation subject / predicate binding
```

Authentication MUST fail if the cryptographically verified attestation refers to any other receipt digest, even when all human-readable fields happen to match.

## 5. Authentication policy

Authentication policy is explicit data, not hidden verifier configuration.

Conceptually:

```rust
pub struct ReceiptAuthenticationPolicyV1 {
    pub profile_id: String,
    pub verifier_profile: VerifierProfileId,
    pub expected_predicate_type: String,
    pub expected_predicate_schema: String,
    pub trusted_root_profile: String,
    pub trusted_root_digest: String,
    pub oidc_issuer: String,
    pub source_repository: String,
    pub source_repository_owner: String,
    pub signer_workflow: String,
    pub signer_workflow_revision: WorkflowRevisionPolicyV1,
    pub source_revision: SourceRevisionPolicyV1,
    pub transparency_policy: TransparencyPolicyV1,
    pub freshness_policy: AuthenticationFreshnessPolicyV1,
}
```

A verifier MUST NOT silently strengthen or weaken this policy.

## 6. Verifier identity is theorem-bearing

`Sigstore`, `GitHub`, `cosign`, or `gh` alone is not a sufficient verifier profile identifier.

At minimum the verifier profile binds:

```text
backend family
backend/version
verification algorithm/profile
trusted-root profile/digest
predicate parser/schema
identity matching semantics
transparency/timestamp policy
source/workflow matching semantics
```

Changing any theorem-bearing verifier component requires a fresh verifier profile and qualification.

## 7. Raw verification evidence is untrusted

Bundle bytes, certificate material, CLI JSON reports, API responses, transparency-log entries and timestamp material are all untrusted inputs until verified.

Structural parsing MUST remain distinct from cryptographic verification.

```text
bundle parses
    != signature verifies
    != certificate chain trusted
    != signer identity authorized
    != predicate accepted
    != receipt authenticated
```

## 8. Authenticated capability

A successful concrete verifier may produce an opaque capability:

```rust
pub struct AuthenticatedQualificationReceiptV1 {
    receipt_digest: ReceiptDigest,
    authentication_policy_id: String,
    verifier_profile_id: String,
    verified_identity: VerifiedSignerIdentityV1,
    verified_predicate: VerifiedQualificationPredicateV1,
    authentication_evidence_digest: String,
}
```

The fields MUST remain private.

### Construction rules

`AuthenticatedQualificationReceiptV1` MUST:

- expose no public unchecked constructor;
- NOT implement `Deserialize`;
- NOT implement `Default`;
- NOT expose a public conversion from raw report/bundle types;
- be constructible only inside concrete verifier code after all mandatory checks pass;
- bind the exact canonical receipt digest and policy profile;
- retain enough verified identity facts for later admission-policy comparison.

A serialized/exported view MAY exist for evidence display, but deserializing that view MUST NOT reconstruct the authenticated capability.

## 9. No public pluggable success oracle

The authority boundary MUST NOT be:

```rust
pub trait ReceiptVerifier {
    fn verify(&self, ...) -> bool;
}
```

or any equivalent public interface where untrusted callers can supply the verifier implementation that creates authenticated capability.

Backend extensibility SHOULD instead use one of these safe patterns:

1. concrete verifier functions/modules compiled into the canonical crate;
2. a sealed internal verifier trait whose implementations are crate-controlled;
3. external verifier processes whose exact executable/profile/output are independently qualified and whose reports are revalidated by a concrete canonical adapter;
4. separately typed backend-specific authenticated capability objects converted only through crate-controlled code.

## 10. Authentication evidence summary is not authority

For UX, logs, receipts and observability, implementations MAY expose a serializable summary:

```rust
pub struct AuthenticationEvidenceSummaryV1 {
    pub verifier_profile_id: String,
    pub receipt_digest: String,
    pub signer_identity: String,
    pub source_repository: String,
    pub signer_workflow: String,
    pub predicate_type: String,
    pub trusted_root_digest: String,
    pub transparency_evidence: String,
}
```

This summary MUST NOT be accepted as proof of authentication merely because it says verification succeeded.

## 11. GitHub public-repository Sigstore profile

The first concrete profile SHOULD target public GitHub Artifact Attestations for this repository.

The repository is public, so the profile SHOULD require the public Sigstore trust path and transparency evidence appropriate to the GitHub public-repository attestation system.

Conceptual profile:

```text
GitHubPublicSigstoreQualificationAttestationV1
```

It MUST bind at least:

- exact `Luminous-Dynamics/mycelix` source repository identity;
- expected repository owner identity;
- GitHub Actions OIDC issuer;
- exact signer-workflow path;
- signer-workflow revision policy;
- exact source commit/ref policy;
- expected in-toto predicate type;
- Mycelix qualification-predicate schema/version;
- canonical qualification receipt digest;
- pinned trusted-root profile/digest;
- required public transparency evidence;
- exact verifier backend/profile/version.

## 12. GitHub private-repository profile is different

GitHub public- and private-repository attestation systems have different transparency properties.

Therefore Mycelix MUST NOT represent them as one verifier profile with a boolean such as `private = true`.

A future private-repository profile MUST be separately identified and qualified, including its timestamp/trusted-root requirements and absence of public Rekor transparency semantics.

## 13. GitHub CLI verifier profile

A first executable backend MAY use a pinned GitHub CLI attestation verifier.

Conceptually:

```text
GitHubCliAttestationVerifierV1
```

The profile MUST pin at minimum:

- exact `gh` version/build identity;
- expected predicate type;
- repository/owner constraint;
- signer-workflow constraint;
- source-ref/digest constraint;
- OIDC issuer constraint;
- trusted-root acquisition/profile semantics;
- exact JSON/report schema consumed by the adapter;
- offline/online verification mode semantics.

The CLI exit code alone is insufficient. The canonical adapter MUST inspect and bind the verified facts required by policy.

## 14. Pure-Rust Sigstore verifier profile

A future pure-Rust backend is desirable but MUST be qualified independently.

It MUST NOT become trusted simply because its dependency is named `sigstore` or because it successfully parses bundle v0.3.

Qualification must demonstrate the exact GitHub attestation profile required by this contract, including custom predicate handling and GitHub-specific identity policy.

Changing Rust verification libraries or major verification behavior changes the verifier profile identity.

## 15. Offline trusted-root profile

Offline verification SHOULD be supported with an explicit trusted-root snapshot profile.

The profile MUST bind:

- trusted-root bytes digest;
- acquisition provenance;
- Sigstore instance(s) covered;
- snapshot/update time;
- revocation/update policy;
- verifier expectations about signatures after the snapshot.

Offline verification MUST NOT imply that the trusted-root snapshot is current indefinitely.

## 16. Xenia-native backend

A future Xenia-native signed qualification receipt MAY coexist with Sigstore.

It MUST use a distinct verifier profile such as:

```text
XeniaQualificationReceiptAuthenticationV1
```

and MUST qualify its own:

- signer identity;
- delegation/capability semantics;
- key rotation/revocation;
- transcript binding;
- host trust/pinning;
- receipt canonicalization and digest binding.

Xenia and Sigstore results MAY converge on the same high-level authenticated receipt semantics only through canonical crate-controlled conversion.

## 17. Authentication revocation and currentness

Successful historical authentication does not imply current admissibility.

Authentication policy MUST define how to interpret at least:

- attestation deletion/revocation;
- trusted-root rotation/revocation;
- signer workflow revocation/supersession;
- source revision supersession where relevant;
- local revocation registries;
- stale offline trusted-root snapshots.

A later production-admission layer MUST re-evaluate currentness rather than trusting a historical boolean.

## 18. Authentication remains distinct from coherence

Receipt authentication MUST bind the same lineage accepted by structural coherence.

At minimum a later bridge must compare:

```text
coherence policy/result digest
qualification receipt digest
authenticated receipt digest
statement/theorem profile
subject SHA
dependency lineage
qualification profile
```

Authentication of one receipt MUST NOT be reusable to authorize a different coherent lineage.

## 19. Authentication remains distinct from production admission

Even a fully authenticated qualification receipt proves only that:

- the exact canonical qualification claim was signed/attested;
- by an identity allowed by the authentication policy;
- under the selected verifier/trust-root/transparency profile.

It does not itself decide whether an application currently permits that theorem/security profile.

```text
AuthenticatedQualificationReceiptV1
    -> input to production-admission policy
    != production-admission authority
```

## 20. Authentication remains distinct from application authority

Production proof admission still does not itself authorize:

- governance execution;
- treasury transfer;
- health action;
- model deployment;
- federated-learning aggregation;
- identity issuance;
- bridge settlement.

Those are application-specific authority theorems.

## 21. Qualification corpus

MYC-ZKP-ATTEST-001A/Q MUST at minimum prove rejection of:

1. valid signature over different receipt bytes;
2. receipt-digest substitution;
3. coherence-result-digest substitution;
4. qualification-profile substitution;
5. correct receipt signed by wrong repository;
6. correct repository but wrong owner identity;
7. correct repository but wrong signer workflow;
8. correct workflow path under an unapproved workflow revision when exact revision is required;
9. wrong OIDC issuer;
10. wrong source SHA/ref/digest;
11. wrong predicate type;
12. correct predicate type with wrong Mycelix predicate version;
13. malformed bundle;
14. structurally valid but cryptographically invalid bundle;
15. trusted-root substitution;
16. missing required public transparency evidence;
17. invalid transparency proof/checkpoint where required;
18. invalid or missing required timestamp evidence;
19. expired/revoked/superseded authentication evidence under a currentness policy;
20. detached verification report replay;
21. public construction of authenticated capability;
22. deserialization into authenticated capability;
23. arbitrary caller-defined verifier producing authenticated capability;
24. authenticated receipt being treated as `ProductionAdmissionV1`;
25. authenticated receipt being treated as application authority.

The corpus SHOULD include at least one independently generated known-good fixture for every admitted backend profile and byte-level mutations of every theorem-bearing identity field.

## 22. Evidence receipt

An authentication qualification receipt SHOULD retain at least:

```text
canonical qualification receipt digest
canonicalization profile
verifier profile + version
attestation predicate type/schema
source repository/owner
source revision/ref/digest
signer workflow + revision
OIDC issuer
trusted-root profile + digest
transparency/timestamp evidence digests
raw bundle/report digest
exact verifier subject SHA/toolchain
qualification corpus digest
result
nonclaims
```

## 23. UI / API truth

Presentation surfaces SHOULD show separate rows rather than one green shield:

```text
Measured security        PASS / FAIL / Not evaluated
Theorem qualification   Qualified / Candidate / Revoked
Structural coherence    PASS / FAIL
Receipt authentication  Authenticated / Failed / Not evaluated
Authentication profile  exact profile ID
Production admission    Admitted / Not admitted / Not evaluated
Application authority   separate domain status
```

Presentation MAY compress this information but MUST NOT strengthen it.

## 24. Implementation sequence

```text
MYC-ZKP-ATTEST-001R
  this contract

MYC-ZKP-RECEIPT-001A
  canonical qualification receipt + deterministic digest

MYC-ZKP-AUTH-CORE-001A
  authentication policy + opaque authenticated capability
  no external verifier backend yet

MYC-ZKP-AUTH-GH-001A
  pinned GitHub public-repository Sigstore/attestation adapter

MYC-ZKP-AUTH-GH-001Q
  known-good + adversarial verifier qualification

MYC-ZKP-ADMISSION-001R/A
  bridge coherent + authenticated + current evidence into
  a separately qualified production proof-admission theorem
```

A pure-Rust Sigstore backend or Xenia backend should be added only as a new separately qualified verifier profile, not as a silent implementation swap.

## Nonclaims

This contract does not establish:

- that any current GitHub Actions artifact is authenticated for Mycelix proof admission;
- that `gh attestation verify` is already qualified for this theorem;
- that any Rust Sigstore library is already qualified for this theorem;
- that an authenticated receipt is production-admitted;
- witness privacy or zero knowledge;
- application authorization;
- governance suitability;
- trusted-root currentness beyond a specific qualified verifier profile.

It freezes the authentication boundary so neither cryptographic verification nor signed CI provenance can silently become application authority.