# Mycelix Forge Review State

This crate defines lineage-aware review revisions for collaboration state that may change after an initial review.

A stateless signed review can prove:

```text
reviewer made statement X
```

but it cannot prove:

```text
statement X is still the reviewer's current state
```

because a caller may omit a later conflicting statement.

## Review revisions

`ReviewRevisionV1` binds:

- exact `ChangeProposalId`;
- exact reviewer `PrincipalId`;
- reviewer-local monotonic sequence;
- exact predecessor revision id for non-genesis revisions;
- `Approve`, `RequestChanges`, or `Withdraw`;
- immutable review-context commitment.

Public successor construction inherits proposal and reviewer identity from the predecessor, so a caller cannot switch either while retaining lineage continuity.

## Supplied-chain validation

`validate_supplied_review_revision_chain_v1(...)` verifies:

- sequence zero starts with no predecessor;
- every later sequence increments by exactly one;
- proposal and reviewer remain unchanged;
- every predecessor id equals the exact prior supplied revision;
- the supplied set stays within a defensive v1 size bound.

It returns `ValidatedSuppliedReviewRevisionChainV1`, which commits the complete ordered supplied list.

## Critical non-claim

A validated supplied chain is **not** proof that its final revision is globally current.

```text
valid supplied chain 0 → 1 → 2
    != no revision 3 exists elsewhere
```

Current-head proof requires an append-only collaboration-state provider, snapshot/witness mechanism, or another completeness theorem that can establish no later revision has been omitted.

## Authentication boundary

This crate also does not authenticate review revisions. A later tranche must bind `ReviewRevisionId` itself into the exact principal-authentication action subject.

That ordering matters: authenticating only the underlying decision and then attaching sequence/predecessor metadata afterward would leave the lineage metadata caller-controlled.

## Proposal mutation

Because the exact `ChangeProposalId` is part of every revision, proposal-significant mutation starts a new review lineage. Reviews cannot migrate across proposal identity changes.
