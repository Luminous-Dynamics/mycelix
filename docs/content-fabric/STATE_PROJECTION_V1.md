# Content Fabric State Projection v1

Status: CF-05A draft contract.

## Boundary

CF-05 stores append-only Holochain claims/evidence. CF-05A converts a supplied, validated evidence snapshot into a deterministic **snapshot-state projection** suitable for later policy filtering and planning.

CF-05A has no Holochain, Iroh, filesystem, system-clock, marketplace, or Symthaea dependency.

## Explicit inputs

Projection requires:

1. provider advertisements with action references and action timestamps;
2. availability claims;
3. withdrawals;
4. observations;
5. a caller-supplied snapshot coverage label;
6. a caller-supplied evaluation timestamp.

The function never asks "what time is it?" itself. Identical evidence plus identical evaluation time produces identical output.

## Opaque source references

`ActionRefV1` and `AgentRefV1` are 39-byte opaque references. A Holochain adapter copies `ActionHash::get_raw_39()` / `AgentPubKey::get_raw_39()` into these values. The projection crate therefore preserves source identity without linking HDK/HDI into the planner-side type graph.

## Snapshot coverage

`Partial` means relevant evidence may be missing.

`QueriedIndexesComplete` means only that the caller asserts it fetched the relevant indexes for the local DHT view used to construct the snapshot. It does not mean global Holochain finality.

Consequently the projection uses `NoWithdrawalObserved`, never `NotWithdrawn`.

## Temporal semantics

All validity windows are half-open:

`authored_at <= evaluated_at < expiry`

Advertisement expiry is:

`advertisement.authored_at + advertisement.ttl`

Availability claim expiry is:

`claim.authored_at + claim.ttl`

A snapshot service candidate's effective upper bound is:

`min(advertisement_expiry, claim_expiry, first_observed_withdrawal_time)`

A withdrawal observed at or before the evaluation time suppresses every claim under the advertisement, including claims authored after that withdrawal. Future withdrawals do not affect historical replay before their authored timestamp.

## Candidate language

`SnapshotServiceCandidateV1` means only:

> In the supplied evidence snapshot, at the supplied evaluation timestamp, this provider had a non-expired advertisement and availability claim and no withdrawal had been observed at or before that time.

It does **not** establish:

- current network reachability;
- read authorization;
- cryptographic integrity of a future transfer;
- provider honesty;
- failure-domain independence;
- lease/payment status;
- suitability under hard placement policy.

Those remain separate gates.

## Observations

Observations are projected separately with deterministic age. They never mutate provider claims or silently promote `VerifiedComplete` into global truth.

The later evidence-policy/durability layer decides how much weight independent observations deserve.

## Partial and malformed snapshots

The projection is robust to incomplete views:

- missing parents exclude the dependent record and produce a diagnostic;
- provider mismatches are excluded;
- unsupported algorithms and impossible sizes are excluded;
- zero/overflowing TTLs are excluded;
- evidence that predates its parent advertisement is excluded;
- future evidence is simply not visible in historical replay.

Identical duplicate actions are deduplicated. Conflicting values with the same action reference are all excluded; arrival order never chooses a winner. Cross-record-type action collisions are treated the same way.

## Determinism

Output arrays are canonically sorted by stable source references/time. Projection is invariant to input ordering. Property tests exercise permutations of advertisements, claims, withdrawals, and observations.

## Planner handoff

CF-06 may consume snapshot service candidates, but MUST still perform hard policy filtering before optimization. Failure-domain values remain self-claims until independent evidence establishes them. Symthaea remains proposal-only authority.
