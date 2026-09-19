# MYC-ZKP-SEC-001R — Proof Security Profile Contract

## Status

Design/qualification contract implementing the separation required by issues #1905 and #1920.

This contract does **not** declare any current proof profile production-safe. It defines the evidence needed before such a claim can be made.

## Governing theorem

```text
policy tier
    != proof parameters
    != measured proof security
    != qualified theorem
    != production admission
```

Mycelix MUST keep those layers distinct.

## 1. SecurityLevel is policy intent, not measured security

The existing serialized variants remain useful as ecosystem policy vocabulary:

```text
Fast
Optimized
Standard
High
```

Their ordering can express increasing policy intent, cost budget, or desired security posture.

They MUST NOT, by themselves, establish an exact cryptographic bit level.

In particular:

```text
SecurityLevel::High
    != 264 measured security bits
    != production proof qualification
```

Changing proof options may improve a STARK profile, but effective security is constrained by the entire construction, including the hash function, field, FRI/query parameters, theorem implementation, verifier admission, and exact backend version.

## 2. Candidate, measured, qualified, production

Every proof profile has one explicit disposition:

```rust
pub enum ProofSecurityDispositionV1 {
    Candidate,
    Measured,
    Qualified,
    ProductionAdmitted,
    HistoricalUnqualified,
    Revoked,
}
```

The transitions are monotonic evidence events, not aliases.

```text
Candidate -> Measured
Measured -> Qualified
Qualified -> ProductionAdmitted
```

A profile can move to `Revoked` from any post-candidate state when assumptions or implementation evidence become invalid.

## 3. Exact security target

A policy tier may resolve to an explicit target, conceptually:

```rust
pub struct ProofSecurityTargetV1 {
    pub profile_id: SecurityTargetId,
    pub policy_tier: SecurityLevel,

    pub min_conjectured_bits: Option<u32>,
    pub min_proven_udr_bits: Option<u32>,
    pub min_proven_ldr_bits: Option<u32>,
    pub min_hash_collision_bits: u32,

    pub allowed_backends: Vec<BackendProfileId>,
    pub allowed_hash_profiles: Vec<HashProfileId>,
    pub allowed_field_profiles: Vec<FieldProfileId>,
    pub require_theorem_qualification: bool,
    pub require_exact_option_profile: bool,
}
```

`None` means a metric is not part of that target. It MUST NOT mean infinite, unknown-but-acceptable, or zero-cost evidence.

A high-stakes target SHOULD normally require at least one proven-security metric, not only a conjectured estimate.

## 4. Measured proof-security evidence

Measured evidence binds the exact proof and software profile that produced the estimate:

```rust
pub struct MeasuredProofSecurityV1 {
    pub evidence_version: u32,
    pub statement_profile: StatementProfileId,
    pub backend_profile: BackendProfileId,
    pub backend_version: String,
    pub proof_options_profile: ProofOptionsProfileId,
    pub hash_profile: HashProfileId,
    pub field_profile: FieldProfileId,

    pub conjectured_bits: u32,
    pub proven_udr_bits: u32,
    pub proven_ldr_bits: u32,
    pub hash_collision_bits: u32,

    pub proof_bytes: u64,
    pub trace_length: u64,
    pub lde_domain_size: u64,
    pub num_queries: u32,
    pub blowup_factor: u32,
    pub grinding_factor: u32,

    pub subject_sha: String,
    pub dependency_graph_digest: String,
    pub measurement_receipt_digest: String,
}
```

Exact concrete field types can be tightened during implementation. The semantics are mandatory.

## 5. Effective-security ceiling

Mycelix MUST NOT report an effective proof-security value above any known cryptographic ceiling of the selected construction.

At minimum:

```text
effective conjectured bits <= conjectured estimator
                         <= hash collision-security ceiling

effective proven UDR bits <= proven UDR estimator
                         <= hash collision-security ceiling

effective proven LDR bits <= proven LDR estimator
                         <= hash collision-security ceiling
```

Additional backend-specific ceilings MAY further reduce the admitted value.

A stronger field extension or larger query count cannot override a lower hash collision-security ceiling.

## 6. Conjectured and proven evidence are different

Winterfell exposes distinct conjectured and proven security estimates.

Mycelix MUST preserve them separately.

```text
conjectured_bits != proven_udr_bits != proven_ldr_bits
```

A UI, receipt, API, or governance policy MUST NOT display one generic `security_bits` field if doing so loses which theorem produced the number.

## 7. UDR and LDR remain distinct

Unique-decoding and list-decoding proven-security estimates are different evidence.

Policies MAY require one or both. They MUST NOT silently choose whichever value is larger.

A conservative generic summary, if one is ever needed, SHOULD be derived by an explicitly named policy such as:

```text
min(required measured components)
```

rather than by an implicit maximum.

## 8. Exact ProofOptions identity

A production proof profile binds the exact protocol parameters used by the verifier.

Conceptually:

```rust
pub struct ProofOptionsProfileV1 {
    pub num_queries: u32,
    pub blowup_factor: u32,
    pub grinding_factor: u32,
    pub field_extension: FieldExtensionProfile,
    pub fri_folding_factor: u32,
    pub fri_remainder_max_degree: u32,
    pub constraint_batching: BatchingProfile,
    pub deep_batching: BatchingProfile,
}
```

The verifier MUST reject proofs outside the admitted profile unless the application explicitly supports a minimum-security admission policy whose semantics have been separately qualified.

RangeMembershipV1 currently follows exact option admission in #1900/#1902.

## 9. Backend/profile identity

`Winterfell` is not a sufficient production profile identifier.

At minimum profile identity includes:

```text
backend family
backend version
field
hash function
ProofOptions
statement/AIR profile
trace profile
verifier admission policy
```

Changing any theorem-bearing or security-bearing component changes the profile identity and requires fresh measurement/qualification.

## 10. Theorem qualification is independent

A proof may have excellent cryptographic parameters and still prove the wrong theorem because its AIR is underconstrained.

Therefore:

```text
security estimator PASS
    != AIR soundness PASS
```

Production admission requires both:

1. a qualified statement/AIR/verifier theorem; and
2. a security profile satisfying the target.

The historical `HealthRangeAir`, `ReviewIntegrityAir`, and `RecursiveAggregationAir` demonstrate why these cannot be collapsed.

## 11. Witness privacy is independent

Security estimates for computational integrity do not establish witness confidentiality.

```text
STARK soundness/security bits
    != zero knowledge
    != witness privacy
```

Issue #1899 governs the RangeMembershipV1 privacy boundary.

A privacy-capable proof profile needs separate mechanism identity and qualification.

## 12. Authentication is independent

Envelope authentication, Dilithium signatures, proof-system soundness, and application authorization remain separate claims.

```text
signature valid
    != proof theorem valid
    != security target satisfied
    != application authorized
```

Security-profile receipts MAY reference authentication evidence but MUST NOT fold it into a single synthetic bit score.

## 13. Production admission

Conceptually:

```rust
pub fn evaluate_security_target(
    target: &ProofSecurityTargetV1,
    measured: &MeasuredProofSecurityV1,
    theorem: &TheoremQualificationReceiptV1,
) -> Result<ProofSecurityAdmissionV1, SecurityAdmissionErrorV1>;
```

Admission fails closed when:

- measured evidence is absent;
- a required metric is absent;
- any required metric is below its target;
- the hash ceiling is below a required target;
- backend/hash/field/options profile is not allowed;
- theorem qualification does not bind the same statement/profile;
- evidence subject/dependency identity does not match;
- evidence has been revoked or superseded.

## 14. No generic production-safe boolean from tier alone

The existing shape:

```text
SecurityLevel::Standard | SecurityLevel::High
    -> is_production_safe() == true
```

is only a policy classification. It cannot establish production cryptographic safety.

The implementation should migrate toward names such as:

```text
is_policy_approved_for_production()
```

and reserve actual production admission for a result carrying measured + theorem-qualified evidence.

Compatibility wrappers may remain temporarily but MUST be documented as policy-only and MUST NOT be used as proof qualification evidence.

## 15. Legacy bit estimates

Existing backend-independent constants such as approximately 40/84/96/264 bits MUST be treated as historical estimates until independently reconstructed for exact profiles.

In particular, no retained generic estimate may exceed a known security ceiling of its declared hash/profile.

Migration SHOULD proceed profile by profile rather than replacing one unverified table with another unverified table.

## 16. RangeMembershipV1 application

For #1900 / #1902:

```text
statement profile = RangeMembershipV1 candidate
proof options = exact candidate options
security evidence = measured by exact-head qualifier
security disposition = RECORDED_ONLY / Measured at most
production floor = NOT ESTABLISHED
witness privacy = NOT ESTABLISHED
commitment opening = NOT ESTABLISHED
```

#1905 remains the security-policy decision point after exact measurements exist.

## 17. Evidence receipt

A security qualification receipt should include at least:

```text
statement profile
backend + version
hash + collision-security profile
field + extension profile
exact ProofOptions
trace dimensions
conjectured security
proven UDR security
proven LDR security
proof size
exact subject SHA
dependency graph digest
toolchain/execution capsule
qualification corpus digest
security target profile
admission result
nonclaims
```

## 18. Qualification corpus

MYC-ZKP-SEC-001A/Q should test at least:

1. exact admitted profile meets its declared target;
2. one fewer/lower security parameter where meaningful does not silently inherit the same profile identity;
3. weaker valid proof options are rejected by exact-option verifiers;
4. hash ceiling lower than target fails admission;
5. missing conjectured evidence fails when required;
6. missing proven UDR fails when required;
7. missing proven LDR fails when required;
8. backend-profile substitution fails;
9. backend-version substitution fails;
10. statement-profile substitution fails;
11. hash-profile substitution fails;
12. field-profile substitution fails;
13. dependency/evidence digest substitution fails;
14. theorem receipt from a different subject fails;
15. revoked qualification fails;
16. policy tier alone cannot construct `ProductionAdmitted` evidence;
17. exact-head execution and checkout immutability;
18. retained raw estimator output for independent review.

## 19. UI / API truth

Presentation surfaces should show exact semantics rather than one green shield.

Example:

```text
Statement theorem       Qualified / Candidate
Backend                 Winterfell 0.13.1
Conjectured security    N bits (measured)
Proven UDR security     N bits (measured)
Proven LDR security     N bits (measured)
Hash ceiling            N bits
Production target       target-id
Target admission        PASS / FAIL / Not evaluated
Witness privacy         Not established
```

Presentation may compress this information but must never strengthen it.

## 20. Implementation sequence

```text
MYC-ZKP-SEC-001R
  this contract

MYC-ZKP-CONFIG-001R/A
  reconcile existing SecurityLevel estimates and policy naming (#1920)

MYC-ZKP-SEC-001A
  typed target + measured evidence + admission result

MYC-ZKP-SEC-001Q
  exact-profile qualification corpus

RangeMembershipV1 security decision (#1905)
  bind measured #1902 evidence to an explicit target
```

## Nonclaims

This contract does not establish:

- a production-safe bit target;
- that any current `SecurityLevel` estimate is correct;
- that RangeMembershipV1 has passed qualification;
- zero knowledge or witness privacy;
- commitment opening;
- application authority;
- governance policy suitability;
- backend security beyond retained upstream/measured evidence.

It freezes the evidence model so policy labels can no longer substitute for cryptographic measurement.