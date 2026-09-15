# IG-007A3 — Observed voting profile revision 2

Status: **ObservedSourceBound / same-source observation refinement**

Tracks: #893. Parent: #873 / draft #874.

## Purpose

IG-007A1 froze the first machine-readable observation of the current Mycelix voting subsystem. Subsequent delegation audit found three additional source-visible gaps without changing the frozen production subject.

Revision 2 therefore means:

```text
same production subject + same source blobs + more complete observation
```

It does **not** mean a new production policy or deployment.

## Source identity

Production subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Bound blobs:

- voting coordinator: `969b845e6186cbcad507c742a718060844f82eb2`
- voting integrity: `658562c8dfaf6a2f1b97a7bfd5cf0fc8a5ab6e66`

Profile:

`mycelix-governance/profiles/observed/voting-fca2c107-v2.json`

Canonical payload SHA-256:

`680af4668889c299b7e0d74531f44894a64a384be71778bca54d3f21ca80ac01`

SHA-256 is used only as deterministic profile identity, not as a governance signature.

## Preserved v1 observations

Revision 2 preserves the earlier observations:

- #851 — split voting-weight authority;
- #855 — caller-supplied Phi vote/tally tier;
- #856 — partial ethics escalation;
- direct vs Phi/delegated missing-Phi semantics;
- current tier constants, quorum/approval behavior and circuit-breaker policy.

The v1 profile and validator remain unchanged and independently valid.

## New delegation observations

### #876 — delegated duplicate-vote binding

Direct Phi voting visibly uses agent-level and voter-level duplicate guards. The delegated Phi entrypoint is not observed in the corresponding `enforce_agent_vote_limit` / `record_agent_vote` call set, and the Phi tally has no observed defensive per-voter deduplication set.

Revision 2 records:

```text
delegated duplicate_voter_guard = NoCoordinatorGuardObserved
phi tally defensive_voter_deduplication_observed = false
```

This is an observation of the frozen source, not a live exploit claim.

### #877 — delegation power conservation

Each delegation fraction is individually bounded, but no observed cross-record invariant establishes exclusive delegation or conserved fractional multi-delegation across overlapping active allocations.

Revision 2 records:

```text
outgoing_allocation_conservation =
  NotEstablishedAcrossApplicableActiveDelegations
```

The resolver's visited set is explicitly scoped as:

```text
VisitedSetPerResolutionTraversal
```

which is cycle control inside one traversal, not a theorem that one delegator's power is conserved across separate delegate votes.

### #892 — delegated admission asymmetry

Direct Phi voting visibly checks:

- fail-closed proposal voting window;
- tier Phi threshold when Phi provenance is available.

The delegated Phi entrypoint has no corresponding observed call to either `verify_voting_period` or `meets_threshold`.

Revision 2 therefore records:

```text
proposal_window       = NoVerifyVotingPeriodCallObserved
phi_threshold_admission = NoMeetsThresholdCallObserved
```

Again, this is not upgraded to `ExecutableQualified` until a separate executable cross-check exists.

## Why revision instead of rewriting v1

Scientific lineage must distinguish:

```text
source changed
```

from:

```text
our observation of unchanged source became more complete
```

Rewriting v1 would erase that distinction. Revision 2 therefore has a new profile id, revision and content commitment while retaining the exact same source subject/blobs.

## Validator

`validate_ig007a3_observed_profile_v2.py` fails closed if the profile silently:

- adds a delegated proposal-window gate;
- adds a delegated Phi-threshold gate;
- adds duplicate-vote binding;
- claims delegation conservation;
- changes resolver visited scope;
- removes any observed gap;
- changes source bindings;
- adds generic safety/fairness/meritocracy/Sybil verdicts.

Those may be desirable future production changes, but they must create a **successor production profile**, not mutate the historical observation.

## Next child

IG-007A4 should add golden counterexamples for:

- repeated delegated records (#876);
- overlapping delegation allocations (#877);
- closed-window direct/delegated admission differential (#892);
- below-threshold direct/delegated admission differential (#892).

A corrected future Mycelix profile should intentionally stop reproducing those old semantics where appropriate.

## Non-claims

This revision does not establish deployment currentness, exploitation, normative unfairness, Sybil resistance, governance safety, or authority to change production policy.
