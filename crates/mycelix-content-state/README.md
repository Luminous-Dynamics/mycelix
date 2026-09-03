# mycelix-content-state

Pure deterministic projection from Content Fabric evidence snapshots to **snapshot service candidates**.

This crate exists between CF-05 Holochain coordination and the later Symthaea planner. It deliberately has no Holochain, Iroh, Tokio, filesystem, payment, or planner dependency.

## Why a separate projection layer?

CF-05 stores append-only claims and observations. Holochain link order is not authoritative recency, and absence of a withdrawal in one local DHT view is not global proof that no withdrawal exists.

`mycelix-content-state` therefore requires:

- an explicit evidence snapshot;
- an explicit evaluation timestamp;
- an explicit snapshot-coverage label.

It never reads the wall clock or network itself.

## Time semantics

All lifetimes are half-open:

`authored_at <= now < expiry`

For an availability claim, `effective_until` is the earliest of:

1. the claim's own expiry;
2. the parent advertisement's expiry;
3. the first valid withdrawal observed at or before the evaluation time.

A withdrawal suppresses all claims under that advertisement, including claims authored after the withdrawal.

## Evidence language

The output says **service candidate in this snapshot**, never "provider is definitely available".

When no withdrawal is present it says `NoWithdrawalObserved`, never "not withdrawn". `SnapshotCoverageV1::QueriedIndexesComplete` means only that the caller asserts it queried the relevant indexes for this local DHT view; it is not global finality.

Observations remain a separate evidence stream. `VerifiedComplete` does not silently promote a provider claim into truth.

## Determinism

Projection is independent of input ordering. Conflicting records with the same action reference are excluded instead of choosing whichever one arrived first. Output collections use canonical sorting.

## Holochain adapter boundary

`ActionRefV1` and `AgentRefV1` are opaque 39-byte source references so a conductor/host adapter can map Holochain `ActionHash::get_raw_39()` and `AgentPubKey::get_raw_39()` without making this crate depend on HDK/HDI.
