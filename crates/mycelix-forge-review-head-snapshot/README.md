# Mycelix Forge review-head snapshots

This crate defines provider-neutral review-head snapshot claims without pretending that an observed set is globally complete.

## Exact separation

```text
review-head snapshot claim
        !=
provider observation
        !=
provider-verified completeness
        !=
globally current review state
```

`ReviewHeadSnapshotClaimV1` binds one exact proposal, its authority/project/repository policy commitments, one opaque provider checkpoint, and a canonical set of reviewer-local head claims.

`ReviewHeadCompletenessObservationV1` binds opaque provider evidence to that snapshot and records whether the provider claims only an observed set or proposal-wide completeness.

`EvidenceBoundReviewHeadSnapshotV1` proves only that those opaque evidence commitments name the exact snapshot. It deliberately does not prove their truth.

## Why this exists

A caller can always omit a later review revision. Likewise, an eventually consistent DHT observation can return a useful set of records without proving that no unseen valid record exists elsewhere.

Forge therefore needs a separate concrete-provider theorem before any snapshot may be called complete or current.

## Canonicalization

- heads are sorted by reviewer;
- duplicate reviewers are rejected rather than silently deduplicated;
- serialized heads must already be in strict canonical order;
- proposal, policy context, provider checkpoint, reviewer, exact revision id, and revision sequence are all part of snapshot identity;
- an empty head set is representable because a proposal may legitimately have no reviews, but emptiness is not evidence of completeness.

## Holochain direction

For Holochain 0.7, deterministic bounded source-chain retrieval can help prove lineage segments relative to a specified chain top. Proposal-wide completeness still needs an explicit coverage mechanism, because DHT/link observations are mutable/eventually consistent rather than global-consensus snapshots.

A later adapter should therefore bind a concrete checkpoint plus independently verifiable coverage evidence and must return an indeterminate/fail-closed result when the required coverage cannot be proven.

## Qualification

The dedicated workflow pins Rust 1.96 and runs rustfmt, warnings-denied Clippy, and all-feature tests.