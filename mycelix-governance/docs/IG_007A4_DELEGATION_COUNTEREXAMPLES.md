# IG-007A4 — Delegation integrity/admission counterexamples

Status: **MeasurementOnly / source-bound counterexample extension**

Tracks: #895. Parent: #893 / draft #894.

## Bound observation

This tranche consumes only:

`mycelix-voting-observed-fca2c107-v2`

Profile commitment:

`680af4668889c299b7e0d74531f44894a64a384be71778bca54d3f21ca80ac01`

Production subject and bound blobs remain unchanged from IG-007A3.

The predecessor counterexample corpus remains immutable:

`bb1cdcfe2205bcf6e6d718b536cbb73ba29a21c9da7dfa664798d02f9f866d90`

A4 is an additive extension, not a rewrite.

## New corpus commitment

Schema:

`mycelix-observed-voting-counterexamples-v2`

Corpus SHA-256:

`ee5e7649a773f564b443320689f465080d4641a0f4f09f13c0c49a7087d6dc10`

## CE-06 — repeated delegated records alter tally mass

Issue: #876

Fixture:

```text
Basic tier
eligible voters = 10
same voter identity
per-record For weight = .5
```

One linked record produces:

```text
voter_count = 1
phi_votes_for = .5
required voter count = 3
approved = false
```

Three same-voter records produce under the observed tally semantics:

```text
voter_count = 3
phi_votes_for = 1.5
required voter count = 3
approval rate = 100%
approved = true
```

The classification is `AuthorityGapCounterfactual`.

It models the consequence of combining the observed delegated duplicate-binding gap with the observed per-record tally logic. It does not claim that a live exploit was executed.

## CE-07 — overlapping delegation allocations duplicate represented source mass

Issue: #877

Fixture:

```text
Alice source weight = .5
Alice -> Bob   100%
Alice -> Carol 100%
same applicable scope
```

Each delegate resolution is modeled from the observed per-resolution traversal semantics:

```text
Bob receives Alice contribution   = .5
Carol receives Alice contribution = .5
```

Therefore:

```text
source Alice mass       = .5
represented Alice mass  = 1.0
representation multiple = 2.0
```

This isolates the missing cross-resolution conservation rule. It does **not** decide whether the successor policy should be exclusive delegation or conserved fractional multi-delegation.

## CE-08 — closed-window admission differential

Issue: #892

For a proposal known to be outside its voting window:

```text
direct Phi:
  RejectClosedWindowByDeclaredSourcePolicy

delegated Phi:
  NoObservedWindowRejectionGate
```

The delegated result deliberately does **not** say `Accepted`. Other runtime failures are outside this source-observation fixture.

## CE-09 — below-threshold admission differential

Issue: #892

Fixture:

```text
Major tier
attested Phi = .2
required Phi = .4
```

Observed policy differential:

```text
direct Phi:
  RejectBelowTierPhiThresholdByDeclaredSourcePolicy

delegated Phi:
  NoObservedPhiThresholdRejectionGate
```

Again, absence of an observed rejection gate is not an executed acceptance claim.

## Why this corpus is useful

The future corrected policy gets exact negative regressions:

- CE-06 should stop changing outcome through repeated same-voter delegated records;
- CE-07 should satisfy an explicitly declared conservation model;
- CE-08 should align direct/delegated voting-window admission;
- CE-09 should align direct/delegated eligibility admission.

That allows before/after institutional qualification without rewriting historical evidence.

## Non-claims

This tranche does not establish a live exploit, deployment impact, fairness failure, governance illegitimacy, or authority to migrate production policy.
