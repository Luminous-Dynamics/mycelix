# MYC-FL-001A3 — Aggregation Robustness Scope Contract

## Status

Planning/contract child of the MYC-FL-001A canonical aggregation line.

Parent implementation subject:

```text
#1844
KrumV1 exact head:
19dbd3190fe4a40fe70b6c798e271cd4f1211491
```

This document does **not** qualify Krum, Multi-Krum, any attack defense, or any
federated-learning deployment. It freezes the context and evidence that must
accompany future robustness claims.

## Governing theorem

```text
algorithm implemented
    !=
algorithm theorem preconditions satisfied
    !=
robustness established for this deployment
    !=
model safe to promote
```

A robust aggregation result is meaningful only relative to an explicit context.

The same algorithm can be well-defined yet poorly matched to a deployment when
client data are strongly heterogeneous, the assumed Byzantine bound is wrong,
identities are Sybilable, participation is adversarially selected, or attackers
adapt to the selector.

## Why this contract exists

The current canonical line deliberately separates:

```text
ValidatedAggregationBatchV1
        |
        v
algorithm profile
        |
        v
robustness scope
        |
        v
receipt / evidence
```

`ValidatedAggregationBatchV1` proves structural admission properties only.

`KrumV1` freezes one exact selection rule only.

Neither object should imply a broader statement such as:

```text
"this round was Byzantine robust"
```

without recording the assumptions and falsification corpus under which that
statement is supported.

## Research boundary motivating the split

The original Krum family is mathematically meaningful under explicit Byzantine
mean-estimation assumptions. Newer work has strengthened the formal treatment of
Multi-Krum and introduced tighter robustness-coefficient analyses.

At the same time, recent selection-aware attacks explicitly optimize malicious
updates to enter the geometric region favored by Krum/Multi-Krum selectors.

Therefore the Mycelix contract must preserve both facts:

```text
formal robustness theorem exists under stated assumptions
+
attack surface remains under broader/adaptive threat models
```

The receipt must never collapse those into a boolean `robust = true`.

## 1. AggregationContextV1

A canonical aggregation attempt should bind an immutable context equivalent to:

```rust
pub struct AggregationContextV1 {
    pub context_id: AggregationContextId,
    pub round_id: RoundId,
    pub model: ModelSubjectV1,
    pub participant_scope: ParticipantScopeV1,
    pub threat_model: ThreatModelV1,
    pub data_regime: DataRegimeV1,
    pub participation: ParticipationProfileV1,
    pub numeric_profile: NumericProfileV1,
    pub algorithm_profile: AggregationProfileRef,
    pub qualification_profile: QualificationProfileRef,
}
```

The exact Rust representation may differ. The semantic fields must not disappear.

### Context identity

`context_id` must bind the complete semantic context, not only the chosen
algorithm name.

At minimum, changing any of these must change or invalidate the context identity:

```text
model subject
participant roster/scope
Byzantine bound f
identity/Sybil assumption
data-regime profile
participation/sampling profile
algorithm profile
numeric profile
qualification profile
```

## 2. ModelSubjectV1

The current `GradientUpdate::model_version: u64` is sufficient for the first
structural admission boundary but is not sufficient as a long-term model identity.

A future canonical model subject should bind, where available:

```rust
pub struct ModelSubjectV1 {
    pub generation: u64,
    pub architecture_id: Option<ModelArchitectureId>,
    pub parameter_schema_id: Option<ParameterSchemaId>,
    pub base_model_commitment: Option<Commitment>,
    pub optimizer_profile: Option<ProfileRef>,
}
```

Required distinction:

```text
same generation integer
    !=
same model semantics
```

MYC-FL-001A3 does not require migrating every caller immediately. It freezes the
upgrade direction so receipts do not permanently canonize `u64` as sufficient
model identity.

## 3. ParticipantScopeV1

Robustness depends on what `n` and `f` actually mean.

A context must distinguish at least:

```text
eligible population
invited participants
admitted participants
submitted contributions
validated contributions
aggregated contributions
```

Do not use one ambiguous `participant_count` for all six.

Conceptually:

```rust
pub struct ParticipantScopeV1 {
    pub roster_id: Option<RosterId>,
    pub eligible_count: Option<u64>,
    pub invited_count: Option<u64>,
    pub admitted_count: u64,
    pub identity_profile: IdentityProfileV1,
}
```

### Identity profile

At minimum:

```rust
pub enum IdentityProfileV1 {
    UnauthenticatedTestOnly,
    UniqueSessionIdentity,
    RegisteredParticipant,
    SybilResistant { profile: ProfileRef },
    DomainDefined { profile: ProfileRef },
}
```

Hard rule:

```text
f Byzantine identities
    !=
f Byzantine real-world actors
```

unless the identity theorem actually establishes that relation.

## 4. ThreatModelV1

The threat model should be explicit rather than inferred from `f`.

Conceptually:

```rust
pub struct ThreatModelV1 {
    pub max_byzantine_contributions: usize,
    pub attacker_knowledge: AttackerKnowledgeV1,
    pub coordination: CoordinationProfileV1,
    pub adaptivity: AdaptivityProfileV1,
    pub identity_attack_surface: IdentityAttackSurfaceV1,
    pub transport_integrity: TransportIntegrityProfileV1,
}
```

### Attacker knowledge

At least:

```rust
pub enum AttackerKnowledgeV1 {
    Unknown,
    Oblivious,
    KnowsAlgorithmFamily,
    KnowsExactAlgorithmProfile,
    KnowsCurrentHonestDistributionApproximation,
    FullWhiteBoxTestProfile,
}
```

### Coordination

At least:

```rust
pub enum CoordinationProfileV1 {
    Independent,
    Coordinated,
    ArbitraryWithinF,
}
```

### Adaptivity

At least:

```rust
pub enum AdaptivityProfileV1 {
    Static,
    RoundAdaptive,
    HistoryAdaptive,
    SelectionAware,
}
```

`SelectionAware` must be representable because an attacker that optimizes for
selector admission is materially different from a naive sign-flip fixture.

## 5. DataRegimeV1

The robustness context must not treat all honest-client distributions as one
regime.

Conceptually:

```rust
pub enum DataRegimeV1 {
    SyntheticIID { fixture: FixtureRef },
    EmpiricalIIDLike { profile: ProfileRef },
    NonIID { profile: NonIIDProfileV1 },
    Unknown,
}
```

### NonIIDProfileV1

Possible fields include:

```rust
pub struct NonIIDProfileV1 {
    pub partition_family: NonIIDPartitionFamilyV1,
    pub concentration_or_severity: Option<f64>,
    pub label_skew: Option<ProfileRef>,
    pub quantity_skew: Option<ProfileRef>,
    pub feature_skew: Option<ProfileRef>,
    pub concept_shift: Option<ProfileRef>,
    pub temporal_drift: Option<ProfileRef>,
}
```

The point is not to force one benchmark vocabulary into every domain. The point
is to prevent:

```text
robust under IID fixture
    -> silently displayed as
robust under heterogeneous deployment
```

## 6. ParticipationProfileV1

Aggregation assumptions can fail even when the full population would satisfy
them if round participation is selected adversarially or is strongly biased.

Conceptually:

```rust
pub struct ParticipationProfileV1 {
    pub sampling_scheme: SamplingSchemeV1,
    pub selection_authority: SelectionAuthorityV1,
    pub replacement: bool,
    pub expected_participation_rate: Option<f64>,
    pub observed_admitted_count: usize,
}
```

Examples of distinct schemes:

```text
fixed roster
uniform random sample
weighted sample
availability-driven
self-selected
externally scheduled
adversarial/unknown
```

Hard rule:

```text
population Byzantine fraction
    !=
round Byzantine fraction
```

without a sampling theorem connecting them.

## 7. NumericProfileV1

The algorithm profile must bind arithmetic semantics where they can change
selection.

For `KrumV1`, the current canonical child intends:

```text
input coordinates: f32
squared-distance accumulation: f64
score accumulation: f64
selection order: score, participant ID, stable index
non-finite arithmetic: reject
```

Future profiles may use fixed-point, quantized, SIMD, GPU, or different precision.
Those are distinct numeric profiles if they can alter selected contributions.

Required distinction:

```text
mathematically equivalent expression
    !=
byte-identical selector under finite arithmetic
```

## 8. AggregationProfileRef

Algorithm identity must be versioned and theorem-specific.

Illustrative profiles:

```text
FedAvgV1
TrimmedMeanV1 { trim rule }
CoordinateMedianV1 { even-count rule }
KrumV1 { f }
MultiKrumV1 { f, m }
GeometricMedianV1 { solver, tolerance, max_iterations }
```

Historical functions should not be silently rebound to these names unless their
semantics match exactly.

For example:

```text
LegacyKrumNMinus2
    !=
KrumV1 { f }
```

## 9. RobustnessClaimV1

Do not expose `robust: bool`.

Use a typed claim/disposition that says what evidence exists.

Conceptually:

```rust
pub enum RobustnessDispositionV1 {
    StructuralAdmissionOnly,
    AlgorithmExecutedUnqualified,
    QualifiedForProfile {
        qualification: QualificationRef,
        scope: RobustnessScopeRef,
    },
    OutsideQualifiedScope {
        reason: ScopeMismatchV1,
    },
    Indeterminate {
        reason: IndeterminateReasonV1,
    },
}
```

A caller must be able to distinguish:

```text
algorithm ran
algorithm tests passed
algorithm theorem profile qualified
deployment context lies inside qualified scope
```

## 10. RobustnessScopeV1

A qualified robustness scope should bind the exact regimes exercised or proven.

Conceptually:

```rust
pub struct RobustnessScopeV1 {
    pub algorithm_profile: AggregationProfileRef,
    pub theorem_assumptions: Vec<AssumptionRef>,
    pub admitted_attack_families: Vec<AttackFamilyProfileV1>,
    pub data_regimes: Vec<DataRegimeRef>,
    pub participation_profiles: Vec<ParticipationProfileRef>,
    pub identity_profiles: Vec<IdentityProfileRef>,
    pub numeric_profiles: Vec<NumericProfileRef>,
    pub known_exclusions: Vec<ExclusionRef>,
}
```

## 11. AttackFamilyProfileV1

The qualification corpus should classify attacks instead of putting every
malicious fixture in one `byzantine` bucket.

At minimum reserve profiles for:

```text
RandomNoise
SignFlip
Scaling
ALIE-style statistical mimicry
MinMax / MinSum style optimization
InnerProductManipulation
LabelFlip-derived update
Backdoor-derived update
ColludingCluster
SybilDuplication
HistoryAdaptive
SelectionAware / selector-proxy
ModelReplacement-like scaling
Unknown/Other
```

Names are provisional. The important property is that attack identity becomes
machine-readable evidence.

## 12. Honest heterogeneity is not an attack

The qualification system must keep these separate:

```text
honest non-IID divergence
    !=
Byzantine contribution
```

A defense that removes honest heterogeneous clients may look robust against an
outlier fixture while introducing severe selection bias.

Therefore every adversarial benchmark should have a corresponding honest
heterogeneity baseline where practical.

Required comparison examples:

```text
IID honest
non-IID honest
non-IID + Byzantine
```

Do not report only the third condition.

## 13. Independent robustness evidence

Unit tests inside the same module are necessary but not sufficient qualification.

The strongest path should use an independently specified fixture corpus.

Suggested layout:

```text
mycelix-workspace/crates/mycelix-fl-core/fixtures/robustness-v1/
  manifest.json
  krum/
    iid-honest/
    one-outlier/
    exact-score-tie/
    boundary-n-f/
    non-iid-honest/
    selection-aware/
  multikrum/
    ...
```

The current workspace excludes fixture directories, which is useful for keeping
runtime compilation narrow, but the qualification workflow may explicitly consume
frozen fixture bytes.

## 14. Fixture manifest

Each fixture should bind at least:

```text
fixture ID
fixture schema version
algorithm profile
numeric profile
model subject
participant IDs
input vectors
metadata
threat-model profile
expected disposition
expected selected IDs or aggregate
comparison rule/tolerance
source/provenance
```

Where a fixture originates from a paper or independent implementation, retain the
citation and transformation procedure.

Synthetic fixture authorship must also be explicit.

## 15. Negative evidence matters

The system should retain failures rather than only successful benchmark summaries.

For a profile, a result may look like:

```text
KrumV1 / profile Q-17
  IID honest                  PASS
  one gross outlier           PASS
  sign flip                    PASS
  non-IID honest alpha=0.1    DEGRADED
  selection-aware proxy       FAIL
```

This is more useful than collapsing the matrix to:

```text
Krum robust: yes
```

A failing attack fixture does not necessarily mean the implementation is wrong.
It may establish a boundary of the algorithm's robustness scope.

## 16. AggregationReceiptV1

A future aggregation receipt should bind enough information to determine whether
its context lies within a qualification scope.

Conceptually:

```rust
pub struct AggregationReceiptV1 {
    pub receipt_id: AggregationReceiptId,
    pub context_id: AggregationContextId,
    pub input_batch_commitment: Commitment,
    pub algorithm_profile: AggregationProfileRef,
    pub selected_participants: Vec<ParticipantId>,
    pub excluded_participants: Vec<ParticipantId>,
    pub output_commitment: Commitment,
    pub numeric_profile: NumericProfileRef,
    pub robustness_disposition: RobustnessDispositionV1,
    pub implementation_qualification_ref: Option<QualificationRef>,
}
```

The receipt should not need to contain every gradient value. It must bind their
canonical commitment and the selection/output evidence.

## 17. Selection evidence

For selectors such as Krum/Multi-Krum, consider optionally retaining:

```text
candidate IDs
score commitment or score vector
neighbor-count profile
selected IDs
tie-break rule
```

This should be controlled by evidence/privacy policy because score disclosure can
leak information about participant updates.

The minimum authoritative receipt can bind a commitment while detailed score
vectors remain protected evidence.

## 18. MultiKrumV1 direction

Do not implement Multi-Krum as:

```text
KrumV1 + num_select
```

without freezing its own profile.

The 2026 robustness analysis treats `m`-MultiKrum as a distinct estimator with a
robustness coefficient depending on `n`, `f`, and `m`.

Therefore reserve:

```rust
pub struct MultiKrumProfileV1 {
    pub max_byzantine: usize,
    pub selected_count: usize,
    pub score_profile: KrumScoreProfileV1,
    pub output_weighting: MultiKrumWeightingV1,
}
```

The implementation must freeze:

```text
m range
score definition
neighbor definition
whether selected vectors are equally or sample-size weighted
finite-arithmetic profile
tie-breaking
```

Do not assume the historical TypeScript or Rust implementation already matches
the desired theorem.

## 19. Theorem evidence vs empirical evidence

Receipts/qualifications should distinguish:

```text
TheoremBound
EmpiricallyTested
Both
```

A mathematical result may prove a robustness coefficient under assumptions that a
production deployment does not satisfy.

An empirical benchmark may exercise realistic attacks without proving a general
guarantee.

Neither evidence class subsumes the other.

## 20. Context-to-qualification matching

A future matcher can conservatively answer:

```rust
match_scope(context, qualification) -> ScopeMatchV1
```

with outcomes such as:

```rust
pub enum ScopeMatchV1 {
    Exact,
    Subsumed,
    OutsideScope { differences: Vec<ScopeDifferenceV1> },
    Indeterminate { missing: Vec<MissingFieldV1> },
}
```

Hard rule:

```text
missing context field
    !=
assumption satisfied
```

Unknown must remain unknown.

## 21. Qualification corpus for MYC-FL-001A3Q

The first robustness-scope qualification should include at least:

1. same algorithm with different `f` gives different profile identity;
2. same vectors under different model subjects cannot share context identity;
3. mixed identity profiles do not silently become Sybil-resistant;
4. unknown identity/Sybil status remains explicit;
5. IID and non-IID profiles are distinct;
6. unknown data regime remains explicit;
7. population and round Byzantine fractions remain distinct;
8. participation scheme changes context identity;
9. numeric profile changes context identity;
10. exact-score tie fixture reproduces deterministic selection;
11. input permutation preserves selection for canonical tie policy;
12. gross outlier fixture;
13. sign-flip fixture;
14. scaling fixture;
15. ALIE-style fixture;
16. min-max/min-sum-style fixture;
17. collusion fixture;
18. Sybil-duplication fixture under a non-Sybil-resistant identity profile;
19. honest non-IID fixture;
20. non-IID plus Byzantine fixture;
21. history-adaptive fixture;
22. selection-aware fixture;
23. a known failing robustness fixture remains recorded as failure rather than hidden;
24. unsupported attack profile cannot yield `QualifiedForProfile`;
25. missing qualification reference cannot yield a qualified disposition;
26. context outside the qualified data regime produces `OutsideQualifiedScope`;
27. context with missing regime data produces `Indeterminate`;
28. receipt binds exact algorithm/numeric/context identities;
29. receipt substitution changes/rejects commitment;
30. exact-head execution and immutable checkout.

These gates establish evidence semantics, not universal robustness.

## 22. Relationship to differential privacy

```text
robust aggregation
    !=
differential privacy
```

DP can add noise that changes aggregation geometry.

Therefore any combined DP + robust-aggregation deployment needs an explicitly
qualified composition profile.

Do not assume a Krum qualification on raw updates transfers automatically to
DP-noised updates.

## 23. Relationship to secure aggregation

```text
secure aggregation
    !=
server can inspect individual updates for Krum
```

Many robust aggregators require pairwise or coordinate-level access to individual
contributions. A confidentiality protocol may intentionally prevent that.

Therefore the architecture must eventually choose among profiles such as:

```text
plaintext-to-qualified-aggregator
confidential compute / MPC capable of robust rule
pre-aggregation client-side validation
secure sum with a compatible robust protocol
```

Do not promise both privacy and arbitrary robust aggregation merely because both
modules exist.

## 24. Relationship to participant trust/reputation

```text
reputation weighted
    !=
Byzantine robust
```

unless a theorem and qualification explicitly bind the reputation process and
adversary model.

Reputation manipulation, cold starts, collusion and Sybil identities can change
the effective threat model.

Trust-weighted aggregation should therefore get a separate algorithm/theorem
profile rather than inheriting Krum evidence.

## 25. Relationship to model promotion

An aggregation receipt is evidence about one aggregation operation.

It does not authorize:

```text
deployment
model promotion
clinical use
financial use
governance action
scientific claim
```

Those are separate authority layers.

## 26. Proposed next implementation sequence

After the existing FL execution lane actually runs:

```text
MYC-FL-001A3a
  canonical AggregationContextV1 + scope types

MYC-FL-001A3b
  AggregationReceiptV1 + context/scope matcher

MYC-FL-001A3Q
  frozen independent robustness fixture corpus

MYC-FL-001A4
  MultiKrumV1 with theorem-bound n/f/m semantics

MYC-FL-001A5
  robust non-IID/adaptive empirical profiles

MYC-FL-SDK
  TS/Python adapters to canonical Rust profiles
```

Names are provisional; theorem boundaries are not.

## 27. Research references to retain in qualification metadata

At minimum record the exact referenced version/identifier when a profile or fixture
uses external literature.

Current relevant references include:

```text
Blanchard et al. (2017)
Machine Learning with Adversaries: Byzantine Tolerant Gradient Descent
Krum / original Byzantine-gradient setting

Bareilles et al. (2026)
arXiv:2602.03899
Byzantine Machine Learning: MultiKrum and an optimal notion of robustness
first formal robustness analysis/bounds for MultiKrum and improved Krum bounds

Subramanian et al. (2026)
arXiv:2608.06637
Bypassing Krum: Selection-Aware Backdoor Attacks in Federated Learning
selection-aware empirical attack surface
```

External citations are evidence/provenance. They do not automatically qualify the
Mycelix implementation.

## Nonclaims

MYC-FL-001A3 does not establish:

- universal Byzantine robustness;
- robustness against an adaptive white-box attacker;
- robustness under arbitrary non-IID data;
- Sybil resistance;
- participant authenticity;
- privacy or confidentiality;
- differential privacy;
- secure aggregation;
- convergence of a training process;
- model accuracy or fairness;
- scientific validity;
- deployment safety;
- governance or economic authority.

Its purpose is narrower and foundational:

```text
make the scope of every future robustness claim explicit,
composable, falsifiable, and impossible to strengthen silently.
```
