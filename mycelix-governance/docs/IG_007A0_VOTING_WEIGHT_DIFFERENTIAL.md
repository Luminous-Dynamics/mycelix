# IG-007A0 — Voting-weight authority differential

Status: **MeasurementOnly / source-bound differential census**

Tracks: #851

This tranche measures a current governance-authority split before any production voting behavior is changed.

## Frozen source subject

Git subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Observed production source blobs:

- voting coordinator: `969b845e6186cbcad507c742a718060844f82eb2`
- voting integrity: `658562c8dfaf6a2f1b97a7bfd5cf0fc8a5ab6e66`

The independent oracle reimplements the two declared formulas. It does not import production Rust. The qualification workflow separately verifies that the frozen source paths still have the exact observed Git blob identities.

## Formula A — multiplicative bounded weight

Observed in the voting coordinator's `compute_vote_weight` path:

```text
with Phi:
R² × (0.7 + 0.3Φ) × (1 + 0.1P) × (1 + 0.05S) × (1 + 0.1D)

without Phi:
R² × 1.0 × (1 + 0.1P) × (1 + 0.05S) × (1 + 0.1D)
```

Inputs are clamped to `[0,1]`; output is clamped to `[0.1,1.5]`.

`cast_vote` reaches this formula through `calculate_vote_weight`.

## Formula B — additive composite weight

Observed in `PhiWeight::composite_weight` in voting integrity:

```text
0.30Φ + 0.25K + 0.20S + 0.15P + 0.10D
```

The explicit Phi-weighted vote path and delegation resolution currently consume this composite-weight method.

The two rules are therefore treated as distinct observed mechanism semantics. This document does not decide which rule should survive.

## Preregistered finite census

The oracle evaluates the Cartesian grid:

```text
{0.0, 0.25, 0.5, 0.75, 1.0}^5
```

for:

```text
(Phi, K/reputation, stake, participation, domain reputation)
```

Total profiles: `3125`.

For Formula A the primary comparison uses `PhiProvenance::Attested`. A second census evaluates `Unavailable`, where Phi must be numerically irrelevant.

## Frozen result commitment

Independent report SHA-256:

`db6d526d5ef6eb206c4a2f88e9032c91441d96d91e7a92079ea9c0c60b68fc73`

## Key finite-grid observations

These are simulation/census observations over the declared finite grid, not universal mechanism theorems.

- Mean signed difference `(multiplicative - additive)`: approximately `-0.1117994921875`.
- Mean absolute difference: approximately `0.25596855574218746`.
- Maximum absolute difference: `0.7125`.
- Multiplicative floor `0.1` is active on `1250 / 3125` attested-grid profiles.
- Multiplicative cap `1.5` is active on `0 / 3125` attested-grid profiles.
- Among `4,018,036` profile pairs that both formulas rank strictly, `992,545` are strict rank inversions.
- An additional `845,518` pairs are tied by one formula but not the other.
- Both formulas are non-decreasing on every one-dimensional grid step in the checked domain; this does **not** make their rankings equivalent.
- Under Formula A with `PhiProvenance::Unavailable`, changing only Phi is inert on the checked grid, as intended by that formula.

The ranking result is particularly important: this is not merely a rescaling. The formulas can prefer different participants.

## Representative divergence

Largest additive-over-multiplicative difference on the grid:

```text
Phi=1.0
K=0.25
Stake=1.0
Participation=1.0
Domain=1.0

additive       = 0.8125
multiplicative = 0.1
```

Largest multiplicative-over-additive difference on the grid:

```text
Phi=1.0
K=1.0
Stake=0.0
Participation=0.0
Domain=1.0

additive       = 0.65
multiplicative = 1.1
```

These fixtures demonstrate the different role assigned to reputation and the multiplicative floor.

## Sensitivity fixtures

With the other four inputs fixed at `0.5`:

- changing stake `0 → 1` changes additive weight by `0.20`;
- the same change changes multiplicative weight by about `0.0117140625`;
- changing reputation `0.25 → 0.5` changes additive weight by `0.0625`;
- the same change changes multiplicative weight by about `0.14013828125`;
- at zero reputation and the other inputs at `0.5`, additive weight is `0.375` while multiplicative weight is floored at `0.1`.

Again, these are finite fixtures, not normative judgments.

## Required decision before convergence

The next policy tranche must explicitly determine whether:

1. these are intentionally different mechanism profiles for different vote paths; or
2. one path is historical drift and should migrate to a versioned canonical successor.

Do not silently replace one formula with the other.

If migration occurs, already-recorded vote weights remain historical facts under their original policy profile. They must not be retroactively recomputed without an explicit versioned migration theorem.

## Relationship to IG-007

Only after the split is classified should a pure HDK-free governance-policy authority be introduced.

That future policy layer should expose content-bound versioned profiles consumable by both Mycelix production zomes and the Symthaea institutional lab.

## Non-claims

This census does not establish:

- which formula is fairer;
- which formula is more meritocratic;
- Sybil resistance;
- capture resistance;
- human legitimacy;
- governance safety;
- authority to change production policy.
