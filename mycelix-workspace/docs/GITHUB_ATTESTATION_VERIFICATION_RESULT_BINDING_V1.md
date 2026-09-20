# GitHub Attestation Verification Result Binding V1

Status: draft contract

Issue: #2086

## 1. Purpose

This contract defines the first Mycelix post-verification result boundary for GitHub artifact attestations.

It deliberately separates verifier-process success, cryptographically verified certificate/timestamp facts, workflow-controlled signed claims, structural receipt binding, and authenticated-receipt capability minting.

```text
VerifierExecutionReceiptV1 with ExitCode(0)
    != verification-result JSON accepted
    != certificate identity matched
    != transparency witness matched
    != signed predicate independently true
    != trusted-builder claim admitted
    != AuthenticatedQualificationReceiptV1
    != ProductionAdmissionV1
```

The first implementation governed by this contract MUST NOT mint `AuthenticatedQualificationReceiptV1`. It should stop at a structural `GitHubAttestationResultBindingV1` or equivalent result. Capability minting is a later theorem.

## 2. Source schema profile

The initial parser profile is:

`github-cli-attestation-processing-result-json-v1`

The current GitHub CLI JSON output for `gh attestation verify --format json` is an array of `AttestationProcessingResult` values. The GitHub CLI owns the stable outer envelope:

```text
[
  {
    "attestation": ...,
    "verificationResult": ...
  }
]
```

The nested `verificationResult` is produced by the Sigstore verification library used by the pinned CLI. Mycelix MUST therefore distinguish:

1. **outer-envelope exactness** — Mycelix can require the exact outer keys for the admitted CLI profile;
2. **required-path exactness** — authority-relevant nested fields must exist with exact types and semantics;
3. **extension tolerance** — unrelated additional nested fields MAY be preserved/ignored by a versioned parser profile rather than causing authority weakening or needless incompatibility.

This contract does not authorize a generic `serde_json::Value` success path. Required-path extraction is strict: missing, duplicate, malformed, or type-confused authority-bearing fields fail closed.

## 3. Input theorem

The result binder consumes exactly:

```text
raw verification stdout bytes
+ exact VerifierExecutionReceiptV1
+ exact GitHubPublicCommandPlanV1 identity/digests
+ exact ReceiptAuthenticationPolicyV1
+ exact QualificationReceiptV1
+ retained attestation bundle digest
+ retained trusted-root material digest
```

The binder MUST establish all lineage relationships itself. Callers do not supply a boolean such as `verified=true`.

### 3.1 Raw stdout binding

Before parsing:

```text
SHA256(raw_stdout_bytes)
    == VerifierExecutionReceiptV1.verifier_stdout_digest
```

The execution receipt must additionally validate and must report:

```text
process_outcome == ExitCode(0)
```

Signal termination, timeout, or nonzero exit fails closed.

### 3.2 Command/environment binding

The execution receipt must carry the same:

- canonical receipt digest;
- verifier executable identity;
- command-arguments digest;
- environment-profile digest;
- retained attestation-bundle digest;
- retained trusted-root-material digest;

as the exact planning/execution lineage used by the binder.

A detached JSON report is never authentication evidence by itself.

## 4. Resource profile

The initial result parser MUST enforce before or during parsing:

```text
raw verification stdout             <= 4 MiB
verified result entries              <= 8
statement subjects per entry         <= 16
verified timestamp witnesses/entry   <= 16
one textual identity field           <= 4 KiB
custom predicate JSON                <= 256 KiB
certificate object JSON              <= 256 KiB
```

These are parser resource limits, not cryptographic-security parameters.

Exceeding a limit fails closed and must remain distinguishable from signature/policy failure.

## 5. Evidence provenance classes

The implementation MUST not flatten all returned fields into one undifferentiated `Verified*` object.

Every extracted fact belongs to one of three provenance classes:

```rust
pub enum GitHubAttestationFactProvenanceV1 {
    CertificateDerived,
    VerifiedTimestampDerived,
    WorkflowControlledSignedClaim,
}
```

### 5.1 Certificate-derived facts

Certificate-derived facts establish signer/workflow identity only to the extent guaranteed by the verified certificate and the pinned verifier profile.

The initial public-repository profile requires certificate-derived equivalents of:

- OIDC issuer;
- source repository identity/URI;
- source repository owner identity/URI;
- source repository digest;
- source repository ref;
- build signer/config URI;
- build signer/config digest/revision;
- runner environment;
- runner invocation/workflow-run identity when exposed by the admitted CLI schema.

Missing required identity data fails closed. No fallback to same-named predicate fields is allowed.

### 5.2 Verified timestamp facts

`verifiedTimestamps` is separately authenticated verifier output. Each accepted witness must retain enough data to distinguish the witness class and, where the admitted verifier schema permits, its trust instance/log identity.

Suggested model:

```rust
pub enum VerifiedTimestampWitnessKindV1 {
    TransparencyLog,
    TimestampAuthority,
    OtherVerifiedWitness,
}

pub enum GitHubTrustInstanceStatusV1 {
    PublicGoodTransparencyConfirmed,
    TimestampAuthorityOnly,
    UnresolvedTrustInstance,
}
```

A timestamp existing is not equivalent to public transparency-log evidence.

### 5.3 Workflow-controlled signed claims

The following remain signed claims from the originating workflow even after signature verification:

- statement subject;
- predicate type;
- custom predicate contents;
- any equivalent workflow-controlled statement field.

They are cryptographically attributable to the signer, but they are not independently true merely because they are signed.

The initial Mycelix profile admits these claims only after the exact trusted-builder certificate identity has already passed.

## 6. Public-Good transparency theorem

GitHub's default trusted-root output can contain material for both the Sigstore Public Good instance and GitHub's private Sigstore instance.

Therefore:

```text
retained trusted_root.jsonl validated signature
    != PublicGoodTransparencyConfirmed
```

The public Mycelix profile requires:

`PublicGoodTransparencyRequired`

and MUST accept only:

`GitHubTrustInstanceStatusV1::PublicGoodTransparencyConfirmed`

for that policy.

A timestamp-authority-only result does not satisfy this requirement.

If the pinned GitHub CLI result schema cannot unambiguously identify the verified witness/trust instance, the parser/binder MUST emit:

`UnresolvedTrustInstance`

and the public-good profile fails closed.

The implementation must not infer trust-instance identity from repository visibility alone.

## 7. Trusted-builder identity

The custom Mycelix qualification predicate may be trusted only after certificate-derived identity proves the exact admitted builder/workflow profile.

For the first profile, require:

```text
certificate OIDC issuer
    == https://token.actions.githubusercontent.com

certificate source repository
    == Luminous-Dynamics/mycelix

certificate source owner
    == Luminous-Dynamics

certificate source digest
    == ReceiptAuthenticationPolicyV1.source_revision.exact_commit

certificate source ref
    == ReceiptAuthenticationPolicyV1.source_revision.exact_git_ref

certificate build signer/config
    == exact ReceiptAuthenticationPolicyV1.signer_workflow

certificate signer/config digest
    ∈ ReceiptAuthenticationPolicyV1.signer_workflow_revision

certificate runner environment
    == GitHub-hosted profile admitted by this verifier policy
```

If a required certificate field is unavailable in the admitted CLI schema, the profile is not established. Do not substitute workflow-controlled predicate data.

## 8. Statement-subject binding

After trusted-builder identity succeeds, the signed statement must contain exactly one subject that matches the canonical qualification receipt identity required by this profile:

```text
subject.name
    == ReceiptAuthenticationPolicyV1.expected_attestation_subject_name

subject.digest.sha256
    == QualificationReceiptV1.digest().sha256
```

Additional subjects fail the initial profile unless a later version defines deterministic multi-subject semantics.

The binder must not select a matching subject while silently ignoring conflicting extra subjects.

## 9. Predicate binding

The result must carry exactly:

```text
statement.predicateType
    == ReceiptAuthenticationPolicyV1.expected_predicate_type
```

The predicate JSON MUST then pass:

`parse_untrusted_qualification_predicate_json_v1(...)`

from #2066.

After strict parsing, the predicate must redundantly match the canonical qualification receipt on:

- receipt digest;
- qualification profile;
- exact Git subject;
- coherence-result digest;
- qualification result.

A valid signature over a mismatching predicate fails closed.

## 10. Candidate multiplicity

The CLI output is an array. Mycelix MUST NOT use:

- `first()`;
- array order;
- latest-looking timestamp;
- first certificate match;
- first subject match;

as candidate selection authority.

Every result is evaluated independently against the full identity tuple:

```text
certificate identity
+ required transparency policy
+ statement subject name/digest
+ predicate type
+ strict predicate semantics
+ retained bundle lineage
```

The initial profile succeeds only when exactly one result satisfies the complete tuple.

```rust
pub enum GitHubCandidateSelectionFailureV1 {
    NoMatchingResult,
    AmbiguousMatchingResults { count: usize },
}
```

## 11. Retained bundle binding

The accepted result must derive from the retained attestation bundle whose SHA-256 is recorded in `VerifierExecutionReceiptV1.attestation_bundle_digest`.

The implementation must not authenticate a result parsed from one execution while attaching a bundle digest from another.

Where the CLI JSON repeats/embeds the attestation bundle, the binder should canonicalize/hash the retained raw bundle bytes independently and compare identities rather than trusting a copied digest field.

## 12. Execution-lineage binding

Suggested non-authoritative structural output:

```rust
pub struct GitHubAttestationResultBindingV1 {
    parser_profile_id: String,
    verifier_execution_receipt_digest: Sha256DigestV1,
    raw_stdout_digest: Sha256DigestV1,
    retained_bundle_digest: Sha256DigestV1,
    retained_trusted_root_digest: Sha256DigestV1,
    certificate_identity: GitHubVerifiedCertificateIdentityV1,
    verified_timestamps: Vec<VerifiedTimestampEvidenceV1>,
    trust_instance_status: GitHubTrustInstanceStatusV1,
    statement_subject: GitHubVerifiedStatementSubjectV1,
    predicate: QualificationAttestationPredicateV1,
    authority: GitHubAttestationResultBindingAuthorityV1,
}
```

with:

```rust
pub enum GitHubAttestationResultBindingAuthorityV1 {
    VerifiedResultBindingOnly,
}
```

and machine-readable nonclaims:

```text
establishes_authenticated_receipt() == false
grants_production_authority()       == false
grants_application_authority()      == false
```

The name `VerifiedResultBindingOnly` means the result passed the admitted verifier-result structural/policy checks. It does not mean the qualification receipt capability has been minted.

## 13. Strict parsing strategy

Do not make Mycelix depend on every nested presentation field emitted by a third-party Sigstore library.

Use this strategy:

### Outer envelope

Strict private DTO with `deny_unknown_fields` for the GitHub CLI-owned keys:

```text
attestation
verificationResult
```

### Nested authority-bearing paths

Use versioned required-path extraction from `verificationResult` for:

```text
signature.certificate
verifiedTimestamps
statement.subject
statement.predicateType
statement.predicate
```

The parser may tolerate unrelated additional nested fields, but it must reject:

- missing required paths;
- duplicate semantic fields;
- wrong JSON types;
- null where a required object/list/string is expected;
- over-limit data;
- malformed digest/Git identity;
- schema/profile mismatch.

The parser profile must record the admitted GitHub CLI/verifier schema version so future field-semantic changes create a new theorem lineage.

## 14. Error taxonomy

At minimum preserve separate error families for:

```text
InputTooLarge
InvalidJson
OuterSchemaMismatch
RequiredPathMissing
RequiredPathWrongType
ResourceLimitExceeded
ExecutionReceiptInvalid
ProcessDidNotSucceed
StdoutDigestMismatch
CommandPlanMismatch
VerifierIdentityMismatch
BundleDigestMismatch
TrustedRootDigestMismatch
CertificateIdentityMismatch
RunnerEnvironmentMismatch
TransparencyPolicyNotMet
UnresolvedTrustInstance
StatementSubjectMismatch
PredicateTypeMismatch
PredicateParseFailure
PredicateReceiptBindingMismatch
NoMatchingResult
AmbiguousMatchingResults
```

Do not collapse these into `VerificationFailed` or `false`.

## 15. Qualification corpus

The independent exact-head qualifier for the first implementation must include at least:

1. exact one-result fixture accepted structurally;
2. malformed outer JSON;
3. unknown outer envelope field;
4. oversized stdout before parse;
5. too many results;
6. missing `verificationResult`;
7. missing `signature.certificate`;
8. missing `verifiedTimestamps`;
9. wrong certificate OIDC issuer;
10. wrong certificate repository/owner;
11. wrong source digest;
12. wrong source ref;
13. wrong signer workflow;
14. wrong signer digest;
15. self-hosted runner under hosted-only policy;
16. TSA-only witness under public-transparency-required policy;
17. unresolved trust instance under public-good profile;
18. wrong statement subject name;
19. wrong statement SHA-256;
20. extra conflicting statement subject;
21. wrong predicate type;
22. strict predicate parse failure;
23. signed predicate with wrong receipt digest;
24. signed predicate with wrong qualification profile;
25. signed predicate with wrong Git subject;
26. signed predicate with wrong coherence digest;
27. signed predicate with wrong qualification result;
28. execution receipt stdout digest mismatch;
29. execution receipt bundle digest mismatch;
30. verifier/command/environment lineage mismatch;
31. zero matching results;
32. two full matching results => ambiguity failure;
33. parsed result cannot convert into `AuthenticatedQualificationReceiptV1`;
34. result binding reports no production/application authority.

## 16. Explicit nonclaims

This contract does not:

- bless a concrete `gh` executable digest or Nix closure;
- assert that the current nested Sigstore JSON schema will never change;
- authenticate any current Mycelix qualification receipt;
- mint `AuthenticatedQualificationReceiptV1`;
- establish revocation/currentness beyond facts supplied by a later verifier theorem;
- create `ProductionAdmissionV1`;
- grant Governance, Finance, Health, FL, identity, or other application authority.

## 17. Next theorem

Only after this result-binding layer is independently qualified should a concrete GitHub backend be permitted to call the crate-private authenticated-capability minting boundary.

That later theorem must prove:

```text
qualified exact verifier executable
+ qualified command/I/O execution
+ exact retained bundle/root bytes
+ qualified result binding
+ exact authentication policy
+ canonical qualification receipt
    -> AuthenticatedQualificationReceiptV1
```

and must still retain:

```text
AuthenticatedQualificationReceiptV1
    != ProductionAdmissionV1
```
