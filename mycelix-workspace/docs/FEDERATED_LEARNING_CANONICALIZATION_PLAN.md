# MYC-FL-001A — Canonical Federated Aggregation Implementation Plan

## Status

Planning child of MYC-FL-000R / draft PR #1796.

This document freezes the smallest Rust implementation tranche that can begin
turning `mycelix-fl-core` into the canonical aggregation owner without silently
changing the meaning of historical algorithms.

## Candidate owner

Canonical candidate:

```text
mycelix-workspace/crates/mycelix-fl-core
```

The first code tranche should harden the pure aggregation boundary only. Do not
combine coordinator state, Holochain, DP, secure aggregation, model promotion or
browser integration into this PR.

## Governing theorem

```text
validated input set
+ exact aggregation profile
+ deterministic/frozen numeric semantics
    -> bounded aggregation result + receipt
```

not:

```text
function named "Krum" returned a vector
    -> Byzantine theorem established
```

## First implementation scope

MYC-FL-001A should add the minimum common validation/profile layer needed by the
existing pure aggregation functions.

Recommended files/modules, adapting to the existing crate layout rather than
forcing names blindly:

```text
aggregation.rs            existing algorithms
aggregation_profile.rs    exact algorithm/numeric/input policy
aggregation_receipt.rs    typed evidence/result identity
validation.rs             finite/model/participant/input checks
fixtures/ or tests/       frozen mathematical vectors
```

If fewer files preserve clarity, prefer fewer files.

## Input hardening — first priority

### Finite gradients

Every admitted gradient coordinate must be finite before sorting, distance,
weighting or accumulation.

Reject:

```text
NaN
+Infinity
-Infinity
```

Do not use `partial_cmp(...).unwrap_or(Ordering::Equal)` as a semantic fallback for
non-comparable values. Non-finite coordinates fail admission instead.

### Exact dimensions

Retain non-empty and equal-dimension checks and include dimension in the subject
profile/receipt.

### Exact model identity/version

Strongest v1 rule:

```text
all admitted updates must name the same exact model identity/version
```

Do not infer the aggregate model version from `updates[0]` when other updates can
name a different model generation.

A later alignment/migration profile can explicitly permit transformed versions.

### Participant uniqueness

Strongest v1 rule:

```text
one participant identity -> at most one admitted update per aggregation subject
```

Reject duplicate participant IDs even if payload bytes happen to match. A future
supersession/multi-contribution profile may relax this explicitly.

### Batch-size policy

For algorithms using sample-count weighting, require non-zero batch size and bind
whether it is:

```text
TrustedMetadata
BoundedSelfDeclared { max }
EvidenceBacked(profile)
```

The first v1 may preserve current behavior while marking batch-size provenance as
an explicit assumption/nonclaim. Do not imply Byzantine robustness if arbitrary
client-declared batch sizes can amplify influence.

### Metadata finiteness

Retain finite loss checks and extend to any metadata used by an algorithm.

Unused metadata should not become semantic identity unless the profile says so.

## Numeric profile

The current Rust implementation uses `f32`; TypeScript historical code uses
`Float64Array`.

Do not widen Rust silently in MYC-FL-001A.

Freeze an initial profile equivalent to:

```text
Float32AggregationV1
```

including:

- finite inputs only;
- deterministic input ordering/tie-breaking where required;
- checked/non-finite output rejection;
- exact tolerance rules for independent fixtures;
- no claim of bitwise portability across arbitrary hardware/compiler changes
  unless separately demonstrated.

A later `Float64AggregationV1` can be introduced independently if scientific use
needs it.

## Algorithm profiles

### FedAvgV1

Bind:

```text
weight = admitted batch_size / total admitted batch_size
```

Require:

- exact model version;
- unique participant;
- finite gradients;
- positive admitted batch sizes;
- finite output.

Do not claim Byzantine robustness.

### TrimmedMeanV1

Bind:

- coordinate-wise operation;
- `trim_fraction` domain;
- exact `floor(n * trim_fraction)` rule;
- finite coordinate admission;
- deterministic sort/tie semantics;
- behavior for too-small cohorts after trimming.

Do not silently return zero for a logically impossible empty trimmed set if the
profile should reject the configuration. Decide and freeze one behavior.

### CoordinateMedianV1

Bind exact even-cohort median rule:

```text
(values[mid - 1] + values[mid]) / 2
```

with finite coordinates and deterministic ordering.

### LegacyKrumNMinus2

Preserve the current historical algorithm only under an explicitly legacy profile
if compatibility fixtures need it:

```text
neighbors = n - 2
num_select may exceed 1
selected outputs sample-count weighted
```

Do not label this profile as the stronger canonical Byzantine theorem.

### KrumV1

The canonical Krum profile should bind explicit Byzantine bound `f` and exact
feasibility/neighbor-count constraints from the selected theorem/reference.

Recommended API direction:

```rust
krum_v1(updates, f) -> Result<SelectedUpdate, AggregationError>
```

Selection of one update stays distinct from Multi-Krum averaging.

If the historical `krum(updates, num_select)` API remains for compatibility,
mark/document it as legacy and prevent new first-party authority-bearing callers
from treating it as KrumV1.

### MultiKrumV1

Bind at least:

```text
f
k
feasibility relation
neighbor_count
selected_weighting
```

The current Rust behavior is a candidate profile:

```text
n >= 2f + 3
neighbor_count = n - f - 2
k <= n - f
equal-weight selected gradients
```

Do not call this canonical until independent reference fixtures validate the exact
selected theorem/constraints.

The TypeScript historical version is a different legacy profile because it has no
explicit `f`, uses `n - 2` and sample-count weighting.

### GeometricMedianV1

Freeze:

```text
numeric profile
tolerance
max_iterations
initial estimate
coincident-point threshold
convergence metric
non-convergence disposition
```

Prefer returning diagnostics such as iterations/converged rather than erasing
whether max-iteration termination occurred.

### TrustWeightedV1

Do not include this in the strongest first qualification unless trust-source
semantics are ready.

When implemented, bind:

```text
trust score profile
source/currentness evidence
allowed score range
missing-score policy
threshold
weight formula
batch-size interaction
```

A score is an input policy, not proof of gradient correctness.

## Deterministic tie-breaking

Robust score ties must not depend on hash-map order or transport arrival.

Tie-break with a semantic identity committed by the aggregation subject, for
example a canonical participant/update ID ordering.

Required invariant:

```text
same semantic input set + same profile
    -> same selected identities
```

under the frozen numeric/tolerance model.

## Aggregation subject

Introduce an exact subject/profile object or equivalent constructor input:

```rust
AggregationSubjectV1 {
    subject_id,
    model_id,
    model_version,
    roster_or_input_set_commitment,
    dimension,
    numeric_profile,
    aggregation_profile,
}
```

Avoid making local vector position or transport order semantic identity.

## Aggregation receipt

Add a typed result/evidence envelope, conceptually:

```rust
AggregationReceiptV1 {
    subject_id,
    profile,
    numeric_profile,
    admitted_input_set_commitment,
    selected_participants,
    rejected_participants,
    output_commitment,
    diagnostics,
    implementation_qualification_ref,
    terminal_disposition,
}
```

First code tranche may use a simple deterministic commitment utility already
canonical in Mycelix; if no such stable cross-domain commitment primitive is ready,
keep commitment plumbing abstract/profile-bound instead of inventing ad hoc JSON
hashing.

## Error vocabulary

Extend the pure error model with typed distinctions such as:

```text
NonFiniteGradient { participant, index }
MixedModelVersion
DuplicateParticipant
InvalidByzantineBound
InfeasibleKrumProfile
NonFiniteOutput
InvalidTrustScore
NonConverged { iterations }
```

Avoid one generic `InvalidArgument` where the qualification corpus needs exact
failure provenance.

## Independent fixture format

Use machine-readable fixtures independent of Rust serde implementation details.

A fixture should bind:

```text
fixture/profile revision
algorithm profile
model/version
participant IDs
batch sizes
input vectors
expected selected IDs where relevant
expected vector or tolerance
expected disposition
```

For deterministic selection algorithms, selected identities are often more useful
than only comparing the final vector.

## Reference/oracle strategy

### Simple aggregators

FedAvg/median/trimmed-mean can use a small independent Python implementation or
explicit rational/decimal fixture derivation.

### Krum/Multi-Krum

Use an implementation written from the frozen theorem rather than translating the
Rust source line-for-line.

The oracle must accept explicit `f` for KrumV1/MultiKrumV1 and reproduce the exact
profile constraints.

### Historical TypeScript

Use TS as a differential compatibility oracle for legacy behavior only.

Record divergences as expected profile differences; do not force canonical Rust to
match a historical mistake.

## Property tests

Add bounded property tests for at least:

```text
input permutation invariance where the profile is set-like
output dimension preservation
FedAvg identical-input idempotence
median within coordinate min/max for finite inputs
trimmed mean within retained coordinate range
selected participant belongs to admitted roster
Multi-Krum selected count == k on successful profile
finite input + admitted bounded profile -> finite output or typed failure
```

Property tests supplement rather than replace fixed independent vectors.

## Non-IID / fairness diagnostics

Add honest heterogeneous fixtures where one honest participant is far from the
majority distribution.

Record whether Krum/Multi-Krum excludes that participant.

This is diagnostic evidence for the explicit nonclaim:

```text
Byzantine robustness != fairness to non-IID clients
```

Do not turn exclusion into a participant-reputation penalty automatically.

## Migration API policy

Do not delete the historical exported functions in the first code PR if they have
unknown consumers.

Recommended sequence:

1. introduce hardened/profiled canonical APIs;
2. run consumer census;
3. add deprecation annotations/docs to legacy APIs;
4. migrate first-party Rust/Holochain callers;
5. bind WASM/TS SDK to canonical Rust;
6. retire duplicate TS algorithm implementations only after differential fixtures
   are preserved.

## MYC-FL-001AQ qualification corpus

Require at least:

1. empty update set rejected;
2. empty gradient rejected;
3. dimension mismatch rejected;
4. NaN gradient rejected with exact coordinate reason;
5. positive/negative infinity rejected;
6. non-finite distance/output fails closed;
7. mixed model versions rejected;
8. duplicate participant rejected;
9. zero batch size rejected for weighted profiles;
10. bounded batch policy enforced where selected;
11. transport-order permutation invariance;
12. deterministic tie fixture;
13. trim fraction boundary fixtures;
14. even/odd coordinate median fixtures;
15. FedAvg independent vector;
16. legacy Krum behavior explicitly distinct from KrumV1;
17. KrumV1 invalid `n/f` combinations rejected;
18. MultiKrumV1 invalid `n/f/k` combinations rejected;
19. independent Krum/Multi-Krum selected-ID vectors;
20. equal-weight vs batch-weight fixture proves profile difference;
21. extreme single Byzantine fixture;
22. coordinated Byzantine-cluster fixture;
23. honest non-IID minority diagnostic fixture;
24. geometric-median coincident-point fixture;
25. geometric-median convergence fixture;
26. geometric-median max-iteration/non-convergence disposition;
27. profile substitution rejected/changes receipt identity;
28. model substitution rejected/changes subject identity;
29. same frozen fixture deterministic across repeated runs under selected profile;
30. exact-head locked qualification and checkout immutability.

## Proposed code stacking

Keep executable work split into small children if needed:

```text
MYC-FL-001A1  finite/model/participant admission hardening
MYC-FL-001A2  aggregation profile types + legacy/canonical split
MYC-FL-001A3  KrumV1/MultiKrumV1 theorem-bound APIs
MYC-FL-001A4  receipts + frozen independent fixtures
MYC-FL-001AQ  dedicated exact-head qualification
```

Names are provisional. Prefer reviewable theorem boundaries over one large PR.

## Deferred integration

Do not include in MYC-FL-001A:

- differential privacy migration;
- secure aggregation;
- Holochain round orchestration;
- model promotion/deployment authority;
- Web Worker/Leptos integration;
- GPU acceleration;
- adaptive defense ensembles;
- reputation updates caused by robust-selection outcomes.

## Nonclaims

A passing canonical aggregation profile does not establish model convergence,
privacy, confidentiality, fairness, representative sampling, independence,
causality, scientific validity, participant honesty or authority to deploy the
resulting model.