# MYC-ZKP-RANGE-001R — Range Proof Replacement Contract

## Status

Design/qualification contract for replacing the quarantined historical
`HealthRangeAir` circuit tracked by issue #227 and containment PR #1868.

Parent containment subject:

```text
#1868
9f6acd8ce911f10ae8b03e4c4771ec629487bf7b
```

This document defines the theorem boundary before a replacement circuit is
implemented. It does not restore range-proof authority.

## Governing theorem

```text
hidden value encoded in a trace
    !=
range membership proven
    !=
range membership bound to a public commitment
```

Mycelix must maintain separate profiles for bare range membership and committed
range membership.

## 1. Separate proof statements

### RangeMembershipV1

Proves only:

```text
there exists one hidden fixed-point value x
such that
min <= x <= max
```

with an exact numeric encoding and proof-parameter profile.

Conceptually:

```rust
pub struct RangeMembershipPublicInputsV1 {
    pub statement_version: u32,
    pub min_raw: i64,
    pub max_raw: i64,
    pub numeric_profile: NumericProfileRef,
    pub range_profile: RangeProfileRef,
}
```

The hidden value itself is not public.

### CommittedRangeV1

Proves the stronger statement:

```text
there exists one hidden x and opening material r such that
min <= x <= max
and
Commit(profile, x, r) == public_commitment
```

Conceptually:

```rust
pub struct CommittedRangePublicInputsV1 {
    pub statement_version: u32,
    pub min_raw: i64,
    pub max_raw: i64,
    pub numeric_profile: NumericProfileRef,
    pub commitment_profile: CommitmentProfileRef,
    pub value_commitment: Commitment,
}
```

Hard rule:

```text
RangeMembershipV1 PASS
    !=
CommittedRangeV1 PASS
```

FL contribution validity generally needs the committed theorem, because proving
that some hidden value is in range is insufficient unless it is the same value
that is committed/encrypted/submitted for the federated round.

## 2. Numeric representation

The replacement theorem must not inherit the historical ambiguity around the
name `Q16.16`.

The current shared fixed-point representation is:

```text
raw storage: signed i64
fractional bits: 16
scale: 2^16
host source may be f32/f64
```

New proof profiles should name this explicitly, for example:

```rust
NumericProfileV1::SignedI64Frac16TruncateTowardZero
```

or an equivalent stable profile identifier.

The exact name is provisional; the semantics are not.

## 3. Checked witness admission

Authority-bearing witness preparation must use a checked conversion boundary.

The new checked path in #1870 is intended to reject:

```text
NaN
+Infinity
-Infinity
scaled values outside signed-i64 representation
```

before proof construction.

The range theorem must not accept a witness merely because Rust produced an
integer via float-to-int coercion.

Required distinction:

```text
host conversion succeeded
    !=
proof statement valid
```

The AIR must still prove the range statement independently.

## 4. Explicit bit width

The historical circuit hard-codes 16 decomposition bits while accepting `u64`
public bounds. That mismatch is forbidden in the replacement profile.

Each range profile must bind an exact decomposition width sufficient for the
statement domain.

Conceptually:

```rust
pub struct RangeProfileV1 {
    pub difference_bits: u16,
    pub signed_encoding: SignedEncodingV1,
    pub bound_semantics: BoundSemanticsV1,
    pub backend_profile: BackendProfileRef,
}
```

If a profile uses `b` bits for a non-negative difference, it must reject a
statement/witness whose required difference domain is not representable in those
`b` bits.

Hard rule:

```text
trace builder truncates high bits
    !=
range proof
```

## 5. Signed interval theorem

FL gradients and many financial/scientific values can be negative. The
replacement theorem must support signed fixed-point bounds explicitly rather than
pretending all domains are unsigned health integers.

For a hidden signed raw value `x`, one clean statement is:

```text
lo = x - min
hi = max - x
lo >= 0
hi >= 0
```

with subtraction performed in a proof representation whose field/range profile
prevents wraparound from masquerading as non-negativity.

The AIR must prove both differences correspond to the same hidden `x`.

## 6. Trace theorem — minimum constraints

A replacement bit-decomposition AIR must constrain at least:

1. bit binaryity;
2. row/index progression;
3. phase or segment identity if multiple decompositions share one trace;
4. accumulator recurrence;
5. final low-difference reconstruction;
6. final high-difference reconstruction;
7. one shared hidden value tying both differences together;
8. public min/max relation to those differences;
9. exact trace length/profile identity;
10. absence of unconstrained witness columns that can alter the claimed theorem.

A conceptual recurrence for one little-endian decomposition is:

```text
acc_{i+1} = acc_i + bit_i * 2^i
bit_i * (bit_i - 1) = 0
```

The implementation may use another equivalent AIR, but the reconstruction must
be verifier-enforced rather than host-side convention.

## 7. Public inputs must enter the theorem

Winterfell supplies public inputs to `Air::new`; carrying them in `ToElements`
is not itself a constraint.

The replacement AIR must store the public values required by the theorem and use
them in assertions and/or transition/boundary constraints.

Required distinction:

```text
public bytes serialized
    !=
public statement constrained
```

Mutation of any theorem-relevant public input must make a valid proof fail.

## 8. Commitment opening is a separate circuit relation

The old circuit places `value_commitment` in public inputs but does not prove that
it commits to the hidden value.

The replacement contract forbids repeating that pattern.

A `CommittedRangeV1` profile must use one commitment relation that is actually
expressible and constrained in the proof backend.

Possible directions include:

```text
A. algebraic hash/commitment computed inside the AIR
B. field-native commitment relation suitable for Winterfell
C. a separate independently qualified commitment-opening proof composed with
   RangeMembershipV1 through an exact shared witness commitment
D. use another backend/profile whose commitment-opening relation is already
   efficiently expressible
```

Do not claim SHA3-256 or SHA-256 commitment opening merely because a host library
computes that hash outside the AIR.

## 9. Commitment profile compatibility

The existing shared `proofs-commitment` library standardizes SHA3/fixed-point
commitments for host-side ecosystem use.

That is useful for ordinary commitments, but:

```text
host can verify SHA3 commitment opening
    !=
Winterfell AIR proves SHA3 commitment opening
```

If the range proof uses a different algebraic commitment internally, the bridge
between the proof commitment and the ecosystem commitment must itself be explicit
and qualified.

Do not silently treat two 32-byte values as the same commitment theorem.

## 10. Domain separation

Every commitment/proof statement used for authority must domain-separate at least:

```text
protocol / statement version
application domain
numeric profile
model / subject identity where applicable
round or claim identity where applicable
value or vector position where applicable
```

For FL, proving coordinate 17 lies in range must not allow replay as coordinate
18, another model generation, or another round.

## 11. Scalar range vs vector contribution validity

A scalar range theorem is not automatically an FL vector-validity theorem.

For FL, possible profiles include:

```text
PerCoordinateRangeV1
VectorLInfBoundV1
VectorL2NormBoundV1
VectorRangeAndNormV1
```

These are different statements.

`PerCoordinateRangeV1` can enforce:

```text
for every j: lower_j <= x_j <= upper_j
```

but does not establish an L2 norm bound unless separately proven.

## 12. Norm proof direction

A future L2 profile may prove:

```text
sum_j x_j^2 <= B^2
```

under an exact fixed-point and overflow profile.

The theorem must specify:

```text
coordinate encoding
square widening
accumulator width/field constraints
maximum vector length
maximum coordinate magnitude
rounding/truncation semantics
bound encoding
commitment relation
```

This should not be hidden inside a generic `range proof` name.

## 13. Proof parameter identity

The historical range circuit creates proof options internally.

The replacement theorem must give production proof parameters a stable profile
identity.

Conceptually:

```rust
pub struct StarkProofProfileV1 {
    pub backend: BackendId,
    pub field_profile: FieldProfileRef,
    pub hash_profile: HashProfileRef,
    pub queries: u32,
    pub blowup_factor: u32,
    pub grinding_factor: u32,
    pub fri_folding: u32,
    pub remainder_profile: ProfileRef,
    pub batching_profile: ProfileRef,
}
```

Exact fields should reflect Winterfell 0.13.1 semantics.

Verifier admission must reject unsupported/disallowed proof profiles rather than
accept arbitrary prover-chosen security parameters.

## 14. Statement identity

A proof statement should bind a stable semantic ID equivalent to:

```text
statement version
numeric profile
range profile
commitment profile (if committed theorem)
proof backend/security profile
application-domain profile
```

Changing any theorem-bearing profile changes statement identity.

## 15. Error/disposition semantics

Avoid APIs that collapse every failure into `String` for the authoritative path.

Conceptually reserve errors such as:

```rust
pub enum RangeProofErrorV1 {
    InvalidBounds,
    InvalidWitnessEncoding,
    DifferenceOutOfProfile,
    UnsupportedNumericProfile,
    UnsupportedProofProfile,
    CommitmentProfileUnsupported,
    ProofGenerationFailed,
    ProofVerificationFailed,
    PublicInputMismatch,
}
```

Host-facing adapters may stringify them later.

## 16. Prover-side checks are diagnostics, not theorem constraints

The prover should fail early on obvious invalid input, but every claimed property
must still be verifier-enforced.

Hard rule:

```text
assert!(witness_is_valid) in trace builder
    !=
AIR proves witness_is_valid
```

This is the core lesson from the quarantined circuit.

## 17. Adversarial qualification — RangeMembershipV1

A replacement qualification corpus should include at least:

1. valid interior value;
2. exact lower boundary;
3. exact upper boundary;
4. value one raw unit below lower bound rejected;
5. value one raw unit above upper bound rejected;
6. negative interval entirely below zero;
7. interval crossing zero;
8. min > max rejected before proving;
9. equal min/max exact-value theorem;
10. maximum admitted difference for selected bit width;
11. first difference requiring one extra bit rejected;
12. malformed binary bit rejected;
13. fabricated accumulator rejected;
14. wrong bit index rejected;
15. wrong phase/segment identity rejected;
16. low-difference reconstruction mutation rejected;
17. high-difference reconstruction mutation rejected;
18. witness split so low/high correspond to different hidden values rejected;
19. public min mutation rejected;
20. public max mutation rejected;
21. statement/profile identity mutation rejected;
22. disallowed proof options rejected;
23. proof from another numeric profile rejected;
24. proof from another application domain rejected;
25. deterministic public-input privacy census confirms hidden raw value is not
    serialized as a public input;
26. exact-head execution and immutable checkout.

## 18. Adversarial qualification — CommittedRangeV1

In addition to RangeMembershipV1 gates:

1. valid opening accepted;
2. commitment to a different value rejected;
3. correct value with wrong blinding/opening rejected when profile uses blinding;
4. commitment-profile substitution rejected;
5. domain-tag substitution rejected;
6. round/subject substitution rejected where bound;
7. coordinate/index substitution rejected where bound;
8. commitment-byte mutation rejected;
9. proof cannot be transplanted to a second ecosystem commitment unless an
   explicit bridge theorem validates that relation;
10. exact shared-witness binding between range and commitment relation.

## 19. FL contribution-validity profile

Only after the scalar/vector proof profiles qualify should FL define something
like:

```rust
pub struct FlContributionValidityProfileV1 {
    pub model_subject: ModelSubjectRef,
    pub round_id: RoundId,
    pub vector_schema: VectorSchemaRef,
    pub bound_profile: BoundProfileRef,
    pub proof_profile: ProofProfileRef,
    pub commitment_profile: CommitmentProfileRef,
}
```

and a proof envelope binding:

```text
participant
round
model subject
contribution commitment
vector length/schema
bound theorem
proof profile
signature/authentication
```

A valid scalar range proof alone must never authorize an FL contribution.

## 20. Relationship to secure aggregation

A future secure-sum profile may accept only contributions whose validity proof is
verified before encrypted aggregation admission.

That composition can establish something like:

```text
all admitted hidden values satisfy bound profile B
```

without revealing each value.

It still does not establish:

```text
honest local training
Krum robustness
representative sampling
DP
```

Those remain separate theorems.

## 21. Relationship to Health and Finance

The replacement primitive may eventually serve Health/Finance again, but domain
adapters must use theorem-specific profiles.

Examples:

```text
HealthAgeRangeV1
HealthLabRangeV1
FinanceBalanceRangeV1
FlCoordinateRangeV1
```

They can share one sound range primitive without sharing authority semantics.

## 22. Migration policy

Historical proofs emitted by the quarantined `HealthRangeAir` remain:

```text
HistoricalUnqualified
```

A new sound verifier must not reinterpret those bytes as qualified evidence.

There is no automatic migration from:

```text
legacy proof accepted
    ->
replacement theorem established
```

Applications must re-prove under the replacement profile when authority requires
it.

## 23. Implementation sequence

Recommended order:

```text
MYC-ZKP-RANGE-001R
  this theorem contract

MYC-ZKP-RANGE-001A
  RangeMembershipV1 only
  no public commitment claim yet

MYC-ZKP-RANGE-001AQ
  adversarial AIR qualification

MYC-ZKP-RANGE-002R
  commitment-opening profile selection

MYC-ZKP-RANGE-002A/Q
  CommittedRangeV1

MYC-ZKP-FL-001R/A/Q
  vector/norm contribution-validity composition
```

This prevents commitment complexity from hiding whether the basic range AIR is
sound.

## 24. Nonclaims

MYC-ZKP-RANGE-001R does not establish:

- a sound replacement circuit;
- a commitment-opening proof;
- SHA3 or SHA256 inside Winterfell;
- an FL norm proof;
- production STARK security parameters;
- Health/Finance proof validity;
- secure aggregation;
- differential privacy;
- model robustness;
- production readiness.

It freezes the theorem so those claims can no longer be implied by API names or
host-side trace construction.