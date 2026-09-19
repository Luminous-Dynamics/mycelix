# FORGE-007 — Authenticated review attestations

`mycelix-forge-review` makes code review a portable evidence object rather than forge-server state.

The protocol deliberately separates three questions:

```text
What did the reviewer decide?
        ↓
ReviewStatement

Did this exact principal authenticate that exact statement?
        ↓
AuthenticatedReviewAttestation

Was that principal structurally eligible for ReviewSource
in the exact authority epoch at the supplied observation time?
        ↓
StructurallyAuthorizedReview
```

None of those steps counts quorum or grants merge authority.

## Review statement

A `ReviewStatement` binds:

- exact `ChangeProposalId`;
- reviewer `PrincipalId`;
- exact authority epoch;
- exact repository-policy state;
- security-significant decision (`Approve` or `RequestChanges`);
- immutable review-context/checklist/findings commitment.

Ordinary discussion/comments remain outside the security-significant statement.

## Cycle-free authentication

The statement is hashed first. FORGE-005A then authenticates that statement digest as the authentication request's `action_subject` with capability `ReviewSource`.

Only afterward does `bind_authenticated_review(...)` combine the statement with the authentication evidence commitment. Authentication evidence is therefore never part of the subject that had to be authenticated, avoiding a self-hash cycle.

## Exact proposal invalidation

The binder re-derives `ChangeProposalId` from the supplied proposal. A one-byte source/commit/tree/policy/intent mutation changes the proposal id, so an old review cannot attach to the changed proposal.

## Authority remains separate

`qualify_review_authority(...)` checks `AuthorityEpoch::is_principal_eligible(..., ReviewSource, observed_at_unix_ms)`.

The observation time is caller-supplied. The resulting type is deliberately named `StructurallyAuthorizedReview`: it does not claim a trusted clock or timestamp authority.

Review quorum is a later theorem and must count distinct eligible principals without double-counting.

## Conservative v1 policy transitions

Review epoch and repository-policy state must equal the proposal's exact bound context. A policy/authority transition invalidates the old review path and requires a reissued proposal/review. Future protocol versions may add explicit lineage-aware migration, but v1 never carries approvals across those boundaries implicitly.
