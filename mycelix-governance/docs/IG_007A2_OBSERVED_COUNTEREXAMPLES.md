# IG-007A2 — Observed voting counterexample corpus

Status: **MeasurementOnly / counterexample-first**

Tracks: #873  
Requires the exact IG-007A1 observed profile from #869 / draft #871.

This tranche turns the current source-bound voting semantics into five small deterministic fixtures. The goal is not to label Mycelix unsafe; it is to make outcome-sensitive mechanism differences independently reproducible before any policy migration.

## Bound profile

`mycelix-voting-observed-fca2c107-v1`

Content SHA-256:

`cbbbb3553ce465be989b5f096364ea97ccc9b1c5d6aae67d80177df8d8109763`

## Frozen corpus commitment

`bb1cdcfe2205bcf6e6d718b536cbb73ba29a21c9da7dfa664798d02f9f866d90`

## CE-01 — path-dependent voting weight

Input:

`Phi=1, K=.25, Stake=1, Participation=1, Domain=1, Attested`

Observed:

- legacy/direct multiplicative = `0.1`;
- explicit Phi/additive = `0.8125`.

This is an observed inconsistency linked to #851, not a fairness verdict.

## CE-02 — unavailable-Phi semantics

Input:

`Phi materialized=0, K=1, Stake=0, Participation=0, Domain=1, provenance=Unavailable`

Observed:

- multiplicative unavailable = `1.1`;
- additive materialized-zero = `0.35`.

This isolates the difference between neutral handling of missing Phi and a provenance-insensitive zero Phi contribution.

## CE-03 — caller-selected tier changes the outcome

Same tally state:

- eligible voters = 20;
- voter count = 3;
- weighted approval rate = 55%.

Observed policy evaluation:

- caller Basic → quorum reached, approved;
- caller Constitutional → quorum not reached, rejected.

This proves the caller-supplied tier surface from #855 is outcome-pivotal. It does not assert which tier a real proposal should receive because the authoritative ProposalType→ProposalTier mapping is not yet frozen.

## CE-04 — mixed ethics escalation

Input:

- original tier = Basic;
- ethics Blocked;
- eligible voters = 12;
- voter count = 3;
- approval rate = 70%.

Observed mixed semantics:

- effective threshold tier = Major;
- required count is still calculated from original Basic tier;
- required voters = 3;
- approved = true.

Coherent effective-Major counterfactual:

- required voters = 5;
- approved = false.

This isolates #856 without granting authority to apply the counterfactual in production.

## CE-05 — abstention-assisted participation quorum

Major tier fixture:

- eligible voters = 20;
- 1 For, 0 Against, 4 Abstain;
- voter count = 5;
- positive For weight, zero Against weight.

Observed:

- participation = 25%;
- required voters = 5;
- quorum reached;
- approval denominator excludes abstentions;
- approval rate = 100%;
- approved = true.

This is labeled `PolicyConsequence`, not a bug. A future institutional experiment can test whether the rule has desirable or undesirable properties under specified assumptions.

## Symthaea use

These five receipts are intended as first golden IG-008 cross-implementation fixtures. A successor Mycelix policy should create a new profile and a new expected corpus rather than rewriting this historical observation.

## Non-claims

No live exploit is claimed. No aggregate severity/risk score is emitted. The corpus does not establish fairness, legitimacy, Sybil resistance, or governance safety.