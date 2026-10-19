# MYC-FL-000R — Federated Learning Semantic Reconciliation

## Status

Documentation-only reconciliation before any language/runtime migration or API retirement.

This document freezes the differences among current Mycelix federated-learning
implementations so canonicalization does not accidentally preserve historical bugs or
erase intentional semantics.

## Governing theorem

```text
same algorithm name
    !=
same mathematical contract
```

Likewise:

```text
Rust implementation
    !=
canonical merely because it is Rust
```

The canonical implementation must match an explicit frozen mathematical/profile
contract, with independent fixtures, adversarial tests and exact input-domain rules.

## Current canonical candidate

The primary Rust candidate is:

```text
mycelix-workspace/crates/mycelix-fl-core
```

It already contains dedicated modules for aggregation, Byzantine defenses,
compression, adaptive/ensemble defense, pipeline state, Holochain bridging and
related FL infrastructure.

The TypeScript SDK implementation remains useful as historical behavior and a
cross-language differential oracle during migration:

```text
mycelix-workspace/sdk-ts/src/fl/index.ts
```

The long-term target is one authoritative Rust semantic owner plus thin adapters,
not two independently evolving algorithm implementations.

## Current aggregation inventory

At minimum reconcile these public algorithm families:

```text
FedAvg
TrimmedMean
CoordinateMedian
Krum
MultiKrum
GeometricMedian
TrustWeighted
```

Additional adaptive/ensemble defenses require their own follow-on reconciliation;
MYC-FL-000R does not silently declare them equivalent.

## Concrete semantic findings

### Numeric domain differs today

The TypeScript SDK uses `Float64Array`.

The Rust aggregation path currently uses `f32` gradient values.

Therefore byte-for-byte or ultra-tight numerical equivalence is not a valid default
qualification theorem.

A canonical profile must freeze:

```text
numeric width
finite-value requirement
rounding/error tolerance
accumulation semantics
overflow/non-finite disposition
```

If future scientific/financially consequential profiles need `f64`, introduce a
new explicit profile rather than silently widening existing `f32` semantics.

### Krum currently lacks an explicit Byzantine bound

Both current TypeScript `krum()` and Rust `krum()` compute scores using roughly:

```text
nearest_neighbors = n - 2
```

and expose a `numSelect`/`num_select` parameter.

This does not bind an explicit tolerated Byzantine count `f` into the score rule.

A Krum-class qualification must instead name the exact literature/profile theorem,
including the admitted `n`, `f`, neighbor-count and selection constraints.

The initial canonical contract should therefore distinguish:

```text
LegacyKrumNMinus2
```

from a future/theorem-bound profile such as:

```text
KrumV1 { f }
```

Do not simply rename the existing implementation and claim the stronger theorem.

### `krum(num_select > 1)` blurs Krum and Multi-Krum

Current Krum functions can select and average multiple updates.

Canonical semantics should make the distinction explicit:

```text
Krum
    selects one candidate under its exact theorem/profile

MultiKrum
    selects a bounded set under its own theorem/profile
```

A convenience parameter must not silently change the algorithm family while
preserving the same algorithm identifier.

### Multi-Krum differs materially between Rust and TypeScript

The current Rust `multi_krum`:

- accepts explicit Byzantine bound `f`;
- requires `n >= 2*f + 3`;
- scores against `n - f - 2` nearest neighbors;
- restricts `k` relative to `n`/`f`;
- averages selected gradients with equal weight.

The current TypeScript `multiKrum`:

- does not accept explicit `f`;
- uses `n - 2` neighbors;
- defaults selection count to `ceil(n/2)`;
- returns `fedAvg(selectedUpdates)`, which weights by batch size.

Therefore:

```text
Rust MultiKrum
    !=
TypeScript MultiKrum
```

This is a semantic divergence, not a porting discrepancy.

The migration must classify which behavior is mathematically intended and preserve
legacy behavior only under an explicit legacy profile if compatibility requires it.

### Robust selection weighting is not neutral

After robust selection, these alternatives are different theorems:

```text
equal-weight selected gradients
sample-count-weighted selected gradients
trust-weighted selected gradients
```

Do not call all three `MultiKrum` without a profile distinction.

### Geometric median differs in convergence semantics

Current implementations differ in details including:

- numeric precision (`f64` in TypeScript vs `f32` in Rust);
- default tolerance;
- convergence metric (maximum coordinate change vs Euclidean change);
- exact coincident-point threshold behavior.

A frozen profile must bind these choices.

### Finite gradient values are not yet a universal precondition

Current shape validation checks vector dimensions, but robust aggregation paths do
not universally reject `NaN`/`Infinity` in gradient coordinates before sorting,
distance or averaging.

That is dangerous because:

- Rust `partial_cmp` fallbacks can turn non-comparable values into ordering ties;
- JavaScript numeric sort comparators can also behave non-canonically with `NaN`;
- Euclidean distance can become non-finite;
- one malicious coordinate can poison scores and outputs.

Canonical aggregation must reject non-finite authoritative gradient coordinates
before algorithm execution.

### Model-version compatibility is a theorem input

Aggregating updates from different model versions is not ordinary averaging.

The pure aggregator should require an explicit version-compatibility policy rather
than simply inheriting the first update's model version for the result.

The strongest v1 profile should require exact model-version equality unless a
separately qualified transformation/alignment profile exists.

### Participant identity multiplicity matters

One participant appearing multiple times can change:

- weighted averages;
- Byzantine counts;
- robust-neighbor geometry;
- trust-weighted contribution;
- effective quorum/participant count.

Canonical aggregation therefore needs explicit duplicate-participant semantics.

The conservative v1 rule should reject duplicate participant identity inside one
aggregation subject unless an exact profile deliberately models multiple
contributions per participant.

### Batch size is consequential metadata

FedAvg and some legacy robust paths use `batch_size` as weight.

Canonical profiles must establish whether batch size is:

```text
self-declared metadata
validated evidence
capped contribution weight
ignored for this algorithm
```

A Byzantine participant must not gain arbitrary influence merely by declaring an
unbounded batch size if the threat profile assumes adversarial clients.

### Trust weighting is a separate authority/evidence question

A reputation/trust score used as aggregation weight is an input policy, not proof
that an update is correct.

Required distinction:

```text
trust/reputation score
    !=
gradient validity
    !=
scientific truth
    !=
participant authority
```

Trust-weighted aggregation needs an exact score source/profile, finite/range checks,
freshness semantics and provenance if it is ever used on an authority-bearing path.

## Canonical profile types

The first implementation child should introduce or freeze equivalents of:

```rust
AggregationProfileV1 {
    algorithm,
    numeric_profile,
    model_compatibility,
    participant_identity_policy,
    metadata_policy,
    algorithm_parameters,
}
```

### Algorithm-specific profiles

Prefer explicit variants or registered profile identities, conceptually:

```text
FedAvgV1
TrimmedMeanV1 { trim_fraction }
CoordinateMedianV1
KrumV1 { byzantine_bound_f }
MultiKrumV1 { byzantine_bound_f, select_k, selected_weighting }
GeometricMedianV1 { tolerance, max_iterations, convergence_metric }
TrustWeightedV1 { trust_profile, threshold, weight_rule }
```

Do not use one generic bag of optional parameters that permits nonsensical
combinations.

## Input contract

A canonical aggregation subject should bind at least:

```text
round/subject identity
model identity + version
ordered or canonically set-like participant roster
participant identity
one exact gradient/update identity per admitted participant
numeric profile
gradient dimension
finite-value status
batch/contribution metadata profile
algorithm/profile
threat assumptions where relevant
```

Transport order must not change the result for algorithms that are mathematically
set-like. If tie-breaking is necessary, define it deterministically from semantic
identity rather than arrival order.

## Output contract

A canonical aggregation result should bind at least:

```text
aggregation subject
algorithm/profile
input-set commitment
selected/rejected participant identities where algorithm exposes them
output gradient commitment
numeric profile
warnings/diagnostics
qualification reference
```

A result is not by itself a model-promotion decision.

## Receipt / evidence boundary

Introduce a typed `AggregationReceiptV1` rather than relying on an
`aggregationMethod` string and participant count.

Conceptually:

```rust
AggregationReceiptV1 {
    subject_id,
    model_id,
    model_version,
    profile,
    roster_commitment,
    admitted_input_commitment,
    selected_participants,
    rejected_participants,
    output_commitment,
    numeric_profile,
    implementation_qualification_ref,
    terminal_disposition,
}
```

The receipt must not claim:

- training convergence;
- global-model quality;
- fairness;
- Byzantine absence;
- scientific validity;
- privacy;
- secure aggregation;
- authority to promote/deploy the model.

Those are separate theorems.

## Cross-language reconciliation table

For every algorithm/version classify the relationship as one of:

```text
CanonicalEquivalent
EquivalentWithinFrozenTolerance
CorrectedInRust
CorrectedInTypeScript
IntentionalSemanticDifference
LegacyBehavior
LegacyBug
NeedsIndependentValidation
UnsupportedForAuthority
```

Do not use a single `matches` boolean.

## Independent oracle requirement

The canonical theorem should not be:

```text
Rust output == old TypeScript output
```

Instead:

```text
Rust output == frozen mathematical contract/oracle
```

TypeScript remains one independent historical implementation and is useful for
finding accidental divergence, but it cannot define correctness when both
implementations share the same historical misconception.

For simple aggregators, a small independent Python/NumPy or language-neutral
fixture oracle can be sufficient. Krum/Multi-Krum fixtures should additionally
bind the selected theorem/profile explicitly.

## Determinism

For deterministic aggregation profiles, fixed semantic inputs and profile must
produce the same output under the frozen numeric/tolerance contract.

Do not include ambient wall clock, hash-map iteration order or arrival order in
semantic output identity.

Where floating-point reduction order matters, define a canonical accumulation/order
profile or qualify bounded tolerance instead of pretending bitwise identity.

## Adversarial input corpus

MYC-FL-001AQ should include at minimum:

1. empty update set;
2. empty gradient;
3. dimension mismatch;
4. NaN coordinate;
5. positive/negative infinity coordinate;
6. overflow/non-finite distance/result;
7. zero batch size;
8. unreasonably large batch size under bounded-weight profile;
9. non-finite loss/metadata;
10. mixed model versions;
11. duplicate participant identity;
12. same participant with conflicting update identities;
13. reordered participant input;
14. exact tie in robust scores;
15. deterministic tie-break verification;
16. trim fraction boundaries;
17. Byzantine count feasibility boundaries;
18. Krum `n/f` invalid combinations;
19. Multi-Krum `n/f/k` invalid combinations;
20. equal-weight vs sample-weight fixture showing intentional difference;
21. one extreme Byzantine outlier;
22. coordinated Byzantine cluster;
23. honest non-IID minority cluster;
24. geometric-median coincident point;
25. geometric-median non-convergence/max-iteration termination;
26. trust score NaN/infinity/out-of-range;
27. missing/stale trust evidence where required;
28. model-version/profile substitution;
29. numeric-profile substitution;
30. exact-head immutable qualification.

## Fairness / non-IID caution

Robust aggregation can reject honest but distributionally unusual clients.

Therefore:

```text
robust-to-modeled-Byzantine-attacks
    !=
fair to non-IID participants
```

Qualification should include heterogeneous honest-client fixtures and report
selection/exclusion behavior rather than only attack success/failure.

Do not turn a robust-aggregation score into a moral/trust label for a participant.

## Migration sequence

### MYC-FL-000R

This semantic reconciliation.

### MYC-FL-001A

Freeze the canonical input/output/profile/receipt types and harden the pure Rust
aggregation functions without changing unrelated FL orchestration.

Likely first changes:

```text
finite gradient validation
exact model-version validation
duplicate participant validation
explicit Krum/Multi-Krum profiles
explicit selected-weighting rule
deterministic tie-breaking
receipt types
golden vectors
```

### MYC-FL-001AQ

Independent oracle + adversarial corpus + exact-head qualification.

### MYC-FL-SDK

Replace algorithmic TypeScript implementations with compatibility bindings/adapters
after semantic equivalence decisions are frozen.

### MYC-FL-WASM

Expose canonical Rust algorithms to first-party Leptos/browser consumers through
WASM or the shared browser worker as appropriate.

### MYC-FL-PRIV / MYC-FL-SA

Compose qualified DP and secure-aggregation receipts. Do not merge these theorems
into FL aggregation itself.

## TypeScript retirement policy

Do not delete the TypeScript implementation immediately.

Use it first for:

- regression fixtures;
- compatibility behavior inventory;
- differential tests;
- identifying semantic drift;
- migration documentation.

After canonical Rust qualification, TypeScript should become an SDK facade/binding
for authoritative algorithms rather than an independent implementation.

## Holochain boundary

The pure aggregation theorem remains independent of Holochain.

Holochain may own distributed round evidence, roster/participation records and
coordination, but:

```text
valid DHT record
    !=
mathematically qualified aggregate
```

and:

```text
qualified aggregate
    !=
current authority to publish/promote a model
```

## Privacy / secure-aggregation boundary

Differential privacy and secure aggregation are independent dimensions:

```text
FL aggregation
    != DP
    != secure aggregation
```

A later federated round receipt may compose all three, but each retains separate
profile identity and qualification lineage.

## Scientific/evidence boundary

Large participant count or robust aggregation does not establish representativeness,
independence, causal validity or external validity.

A Symthaea/Mycelix scientific bridge must preserve:

```text
federated aggregation success
    != representative sample
    != independent observations
    != causal evidence
```

## Nonclaims

MYC-FL-000R does not establish any current FL implementation as Byzantine-resilient,
secure, private, fair, convergent, scientifically valid or production qualified.

It establishes the reconciliation contract required before one implementation can
be promoted as canonical.