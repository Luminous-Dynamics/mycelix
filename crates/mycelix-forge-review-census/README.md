# FORGE-007C — Complete review census and conflict-free approval state

`mycelix-forge-review-census` closes a gap deliberately left open by FORGE-007B.

FORGE-007B can prove that a supplied set contains enough distinct authorized `Approve` reviews. It cannot prove that the caller did not omit a security-significant `RequestChanges` review.

FORGE-007C separates that problem into an explicit completeness theorem.

```text
StructurallyAuthorizedReview values
        ↓
canonical ReviewCensus
        ↓
source-state + adapter completeness observation
        ↓
adapter verifier
        ↓
QualifiedCompleteReviewCensus
        ↓
no RequestChanges
+ exact 007B quorum reviews are present
        ↓
ConflictFreeApprovalState
```

## No implicit latest-wins

Protocol v1 permits at most one security-significant review per principal in one census. If the source contains both an `Approve` and a `RequestChanges` from the same principal, construction fails as ambiguous instead of selecting one by timestamp.

A future protocol may define an explicit signed supersession lineage. v1 does not infer one.

## Completeness is adapter-qualified

A caller cannot obtain `QualifiedCompleteReviewCensus` merely by serializing a subset and calling it complete. Qualification requires a `ReviewCompletenessVerifier` whose identity matches the observation and which validates the census against one exact collaboration-state root.

The concrete adapter may later be Radicle, Mycelix/Holochain, a Git collaborative-object log, or another source. Its evidence is bound into the positive result.

## Conflict-free approval

`ConflictFreeApprovalState` requires:

- the review census was positively qualified complete;
- it names the same project, proposal, authority epoch and repository-policy state as the approval quorum;
- no census entry is `RequestChanges`;
- every reviewer/evidence commitment counted by the approval quorum appears exactly in the complete census as `Approve`.

It still does **not** grant merge authority. Merge policy must combine this state with repository correctness, protected-history, build/CI qualification, and any other project policy requirements.

## Claim boundaries

FORGE-007C does not establish trusted time, adapter correctness by itself, repository correctness, or merge authorization. A positive completeness result means the named verifier accepted the exact census/source-state/evidence tuple; consumers still decide which adapter/verifier identities are acceptable policy inputs.
